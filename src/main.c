/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/kernel.h>
#include <zephyr/console/console.h>
#include <zephyr/sys/printk.h>
#include <string.h>
#include <stdlib.h>
#include <zephyr/types.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/crypto.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/uuid.h>

#include <bluetooth/services/throughput.h>
#include <bluetooth/scan.h>
#include <bluetooth/conn.h>
#include <bluetooth/gatt_dm.h>
#include <zephyr/bluetooth/l2cap.h>
#include <zephyr/shell/shell_uart.h>
#include <dk_buttons_and_leds.h>

#define ANDROID_MIN_CONN_INTERVAL 0x0006
#define ANDROID_MAX_CONN_INTERVAL 0x0006

#define IOS_MIN_CONN_INTERVAL 0x000c
#define IOS_MAX_CONN_INTERVAL 0x000c

#define SUPERVISION_TIMEOUT 100
#define CONN_LATENCY 0


#define CONN_STACK_SIZE 1024
K_THREAD_STACK_DEFINE(adv_stack_area, CONN_STACK_SIZE);
struct k_thread adv_thread_data;

/******************************************
 * BLE Configurations
 ******************************************/
typedef struct test_params_s {
  struct bt_le_conn_param *conn_param;
	struct bt_conn_le_phy_param *phy;
	struct bt_conn_le_data_len_param *data_len;
} test_params_t ;

test_params_t ios_optimal = {
  .conn_param = 
    BT_LE_CONN_PARAM(
      IOS_MIN_CONN_INTERVAL, 
      IOS_MAX_CONN_INTERVAL, 
      CONN_LATENCY,
			SUPERVISION_TIMEOUT),
	.phy = BT_CONN_LE_PHY_PARAM_2M,
	.data_len = BT_LE_DATA_LEN_PARAM_MAX
};

test_params_t ios_normal = {
  .conn_param = 
    BT_LE_CONN_PARAM(
      IOS_MIN_CONN_INTERVAL, 
      IOS_MAX_CONN_INTERVAL, 
      CONN_LATENCY,
			SUPERVISION_TIMEOUT),
	.phy = BT_CONN_LE_PHY_PARAM_2M,
	.data_len = BT_LE_DATA_LEN_PARAM_MAX
};

test_params_t android_optimal = {
  .conn_param = 
    BT_LE_CONN_PARAM(
      IOS_MIN_CONN_INTERVAL, 
      IOS_MAX_CONN_INTERVAL, 
      CONN_LATENCY,
			SUPERVISION_TIMEOUT),
	.phy = BT_CONN_LE_PHY_PARAM_2M,
	.data_len = BT_LE_DATA_LEN_PARAM_MAX
};

/******************************************

 ******************************************/
#define DEVICE_NAME	CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)
#define INTERVAL_MIN	0x032	/* 320 units, 400 ms */
#define INTERVAL_MAX	0x032	/* 320 units, 400 ms */
#define THROUGHPUT_CONFIG_TIMEOUT K_SECONDS(20)

static K_SEM_DEFINE(throughput_sem, 0, 1);

static throughput_mode_t throughput_mode = NOTIFICATION;

static volatile bool data_length_req;
static volatile bool test_ready;
static struct bt_conn *default_conn;
static struct bt_throughput throughput;
static struct bt_uuid *uuid128 = BT_UUID_THROUGHPUT;
static struct bt_gatt_exchange_params exchange_params;
static struct bt_le_conn_param *conn_param =
	BT_LE_CONN_PARAM(INTERVAL_MIN, INTERVAL_MAX, 0, 400);

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(BT_DATA_UUID128_ALL,
		0xBB, 0x4A, 0xFF, 0x4F, 0xAD, 0x03, 0x41, 0x5D,
		0xA9, 0x6C, 0x9D, 0x6C, 0xDD, 0xDA, 0x83, 0x04),
};

static const struct bt_data sd[] = {
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

static const char img[][81] = {
#include "img.file"
};

static void button_handler_cb(uint32_t button_state, uint32_t has_changed);

static const char *phy2str(uint8_t phy)
{
	switch (phy) {
	case 0: return "No packets";
	case BT_GAP_LE_PHY_1M: return "LE 1M";
	case BT_GAP_LE_PHY_2M: return "LE 2M";
	case BT_GAP_LE_PHY_CODED: return "LE Coded";
	default: return "Unknown";
	}
}

int conn_configuration_thread();

static void instruction_print(void)
{
	printk("\nType 'config' to change the configuration parameters.\n");
	printk("You can use the Tab key to autocomplete your input.\n");
	printk("Type 'run' when you are ready to run the test.\n");
}

void scan_connecting_error(struct bt_scan_device_info *device_info)
{
	printk("Connecting failed\n");
}

static void exchange_func(
  struct bt_conn *conn, 
  uint8_t att_err,
	struct bt_gatt_exchange_params *params
){
	struct bt_conn_info info = {0};
	int err;

	printk("MTU exchange %s\n", att_err == 0 ? "successful" : "failed");

	err = bt_conn_get_info(conn, &info);
	if (err) {
		printk("Failed to get connection info %d\n", err);
		return;
	}

	if (info.role == BT_CONN_ROLE_CENTRAL) {
		instruction_print();
		test_ready = true;
	}
}

static void discovery_complete(
  struct bt_gatt_dm *dm,
	void *context
){
	int err;
	struct bt_throughput *throughput = context;

	printk("Service discovery completed\n");

	bt_gatt_dm_data_print(dm);
	bt_throughput_handles_assign(dm, throughput);
	bt_gatt_dm_data_release(dm);

	exchange_params.func = exchange_func;

	err = bt_gatt_exchange_mtu(default_conn, &exchange_params);
	if (err) {
		printk("MTU exchange failed (err %d)\n", err);
	} else {
		printk("MTU exchange pending\n");
	}
}

static void discovery_service_not_found(
  struct bt_conn *conn,
	void *context
){
	printk("Service not found\n");
}

static void discovery_error(
  struct bt_conn *conn,
  int err,
  void *context
  )
{
	printk("Error while discovering GATT database: (%d)\n", err);
}

struct bt_gatt_dm_cb discovery_cb = {
	.completed         = discovery_complete,
	.service_not_found = discovery_service_not_found,
	.error_found       = discovery_error,
};

static void connected(struct bt_conn *conn, uint8_t hci_err)
{
	struct bt_conn_info info = {0};
	int err;

	if (hci_err) {
		if (hci_err == BT_HCI_ERR_UNKNOWN_CONN_ID) {
			/* Canceled creating connection */
			return;
		}

		printk("Connection failed (err 0x%02x)\n", hci_err);
		return;
	}

  //conn_configuration_set(ios_optimal.conn_param, ios_optimal.phy, ios_optimal.data_len);

	if (default_conn) {
		printk("Connection exists, disconnect second connection\n");
		bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
		return;
	}

	default_conn = bt_conn_ref(conn);

	err = bt_conn_get_info(default_conn, &info);

	if (err) {
		printk("Failed to get connection info %d\n", err);
		return;
	}

	printk("Connected as %s\n",
	       info.role == BT_CONN_ROLE_CENTRAL ? "central" : "peripheral");

	printk("Conn. interval is %u units\n", info.le.interval);

    // Create the thread
  k_thread_create(
      &adv_thread_data, adv_stack_area,
      K_THREAD_STACK_SIZEOF(adv_stack_area),
      conn_configuration_thread,
      NULL, NULL, NULL,
      5, 0, K_NO_WAIT);
}

static void adv_start(void)
{
	struct bt_le_adv_param *adv_param =
		BT_LE_ADV_PARAM(BT_LE_ADV_OPT_CONNECTABLE |
				BT_LE_ADV_OPT_ONE_TIME,
				BT_GAP_ADV_FAST_INT_MIN_2,
				BT_GAP_ADV_FAST_INT_MAX_2,
				NULL);
	int err;

	err = 
    bt_le_adv_start(
      adv_param, ad, 
      ARRAY_SIZE(ad), 
      sd,
			ARRAY_SIZE(sd));

	if (err) {
		printk("Failed to start advertiser (%d)\n", err);
		return;
	}
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	struct bt_conn_info info = {0};
	int err;

	printk("Disconnected (reason 0x%02x)\n", reason);

	test_ready = false;
	if (default_conn) {
		bt_conn_unref(default_conn);
		default_conn = NULL;
	}

	err = bt_conn_get_info(conn, &info);
	if (err) {
		printk("Failed to get connection info (%d)\n", err);
		return;
	}

	adv_start();
}

static bool le_param_req(struct bt_conn *conn, struct bt_le_conn_param *param)
{
	printk("Connection parameters update request received.\n");

	printk("Minimum interval: %d, Maximum interval: %d\n",
    param->interval_min, 
    param->interval_max);

	printk("Latency: %d, Timeout: %d\n", 
    param->latency, 
    param->timeout);

	return true;
}

static void le_param_updated(
  struct bt_conn *conn, 
  uint16_t interval,
	uint16_t latency, 
  uint16_t timeout
){
	printk("Connection parameters updated.\n"
    " interval: %d, latency: %d, timeout: %d\n",
    interval, 
    latency, 
    timeout);

	k_sem_give(&throughput_sem);
}

static void le_phy_updated(
  struct bt_conn *conn,
	struct bt_conn_le_phy_info *param
){
  int err;
	printk("LE PHY updated: TX PHY %s, RX PHY %s\n",
	  phy2str(param->tx_phy), 
    phy2str(param->rx_phy));

	k_sem_give(&throughput_sem);
}

static void le_data_length_updated(
  struct bt_conn *conn,
	struct bt_conn_le_data_len_info *info
){
	if (!data_length_req) {
		return;
	}

	printk("LE data len updated: TX (len: %d time: %d)"
    " RX (len: %d time: %d)\n", 
    info->tx_max_len,
    info->tx_max_time, 
    info->rx_max_len, 
    info->rx_max_time);

	data_length_req = false;
	k_sem_give(&throughput_sem);
}

static uint8_t throughput_read(const struct bt_throughput_metrics *met)
{
	printk("[peer] received %u bytes (%u KB)"
    " in %u GATT writes at %u bps\n",
    met->write_len, 
    met->write_len / 1024, 
    met->write_count,
    met->write_rate);

	k_sem_give(&throughput_sem);

	return BT_GATT_ITER_STOP;
}

static void throughput_received(const struct bt_throughput_metrics *met)
{
	static uint32_t kb;

	if (met->write_len == 0) {
		kb = 0;
		printk("\n");
		return;
	}

	if ((met->write_len / 1024) != kb) {
		kb = (met->write_len / 1024);
		printk("=");
	}

  printk("\n[local] received %u bytes (%u KB)"
		" in %u GATT writes at %u bps\n",
		met->write_len, 
    met->write_len / 1024,
		met->write_count, 
    met->write_rate);
}

static void throughput_send(const struct bt_throughput_metrics *met)
{
	printk("\n[local] received %u bytes (%u KB)"
		" in %u GATT writes at %u bps\n",
		met->write_len, 
    met->write_len / 1024,
		met->write_count, 
    met->write_rate);
}

static void throughput_mode_change(throughput_mode_t mode)
{
  LOG_INF("Throughput Mode Updated\n")
  throughput_mode = mode;
}

static const struct bt_throughput_cb throughput_cb = {
	.data_read = throughput_read,
	.data_received = throughput_received,
	.data_send = throughput_send,
  .mode_change = throughput_mode_change
};

static struct button_handler button = {
	.cb = button_handler_cb,
};

static void button_handler_cb(uint32_t button_state, uint32_t has_changed)
{
	int err;
	uint32_t buttons = button_state & has_changed;

  printk("\nPeripheral. Starting advertising\n");
  adv_start();

	/* The role has been selected, button are not needed any more. */
	err = dk_button_handler_remove(&button);
	if (err) {
		printk("Button disable error: %d\n", err);
	}
}

static void buttons_init(void)
{
	int err;

	err = dk_buttons_init(NULL);
	if (err) {
		printk("Buttons initialization failed.\n");
		return;
	}

	/* Add dynamic buttons handler. Buttons should be activated only when
	 * during the board role choosing.
	 */
	dk_button_handler_add(&button);
}

static int connection_configuration_set(
  const struct shell *shell,
  const struct bt_le_conn_param *conn_param,
  const struct bt_conn_le_phy_param *phy,
  const struct bt_conn_le_data_len_param *data_len
){
	int err;
	struct bt_conn_info info = {0};

	err = bt_conn_get_info(default_conn, &info);
	if (err) {
		shell_error(shell, "Failed to get connection info %d", err);
		return err;
	}

	if (info.role != BT_CONN_ROLE_CENTRAL) {
		shell_error(shell,
		"'run' command shall be executed only on the central board");
	}

	err = bt_conn_le_phy_update(default_conn, phy);
	if (err) {
		shell_error(shell, "PHY update failed: %d\n", err);
		return err;
	}

	shell_print(shell, "PHY update pending");
	err = k_sem_take(&throughput_sem, THROUGHPUT_CONFIG_TIMEOUT);
	if (err) {
		shell_error(shell, "PHY update timeout");
		return err;
	}

	if (info.le.data_len->tx_max_len != data_len->tx_max_len) {
		data_length_req = true;

		err = bt_conn_le_data_len_update(default_conn, data_len);
		if (err) {
			shell_error(shell, "LE data length update failed: %d",
				    err);
			return err;
		}

		shell_print(shell, "LE Data length update pending");
		err = k_sem_take(&throughput_sem, THROUGHPUT_CONFIG_TIMEOUT);
		if (err) {
			shell_error(shell, "LE Data Length update timeout");
			return err;
		}
	}

	if (info.le.interval != conn_param->interval_max) {
		err = bt_conn_le_param_update(default_conn, conn_param);
		if (err) {
			shell_error(shell,
				    "Connection parameters update failed: %d",
				    err);
			return err;
		}

		shell_print(shell, "Connection parameters update pending");
		err = k_sem_take(&throughput_sem, THROUGHPUT_CONFIG_TIMEOUT);
		if (err) {
			shell_error(shell,
				    "Connection parameters update timeout");
			return err;
		}
	}

	return 0;
}

int conn_configuration_thread()
{
	int err;
	struct bt_conn_info info = {0};

  const struct bt_le_conn_param *conn_param = ios_optimal.conn_param;
  const struct bt_conn_le_phy_param *phy = ios_optimal.phy;
  const struct bt_conn_le_data_len_param *data_len = ios_optimal.data_len;

	err = bt_conn_get_info(default_conn, &info);
	if (err) {
		printk("Failed to get connection info %d", err);
		return err;
	}
	err = bt_conn_le_phy_update(default_conn, phy);
	if (err) {
		printk("PHY update failed: %d\n", err);
		return err;
	}

	printk("PHY update pending\n");
	err = k_sem_take(&throughput_sem, THROUGHPUT_CONFIG_TIMEOUT);
	if (err) {
		printk("PHY update timeout");
		return err;
	}

	if (info.le.data_len->tx_max_len != data_len->tx_max_len) {
		data_length_req = true;

		err = bt_conn_le_data_len_update(default_conn, data_len);
		if (err) {
			printk("LE data length update failed: %d",
				    err);
			return err;
		}

		printk("LE Data length update pending\n");
		err = k_sem_take(&throughput_sem, THROUGHPUT_CONFIG_TIMEOUT);
		if (err) {
			printk("LE Data Length update timeout");
			return err;
		}
	}

	if (info.le.interval != conn_param->interval_max) {
		err = bt_conn_le_param_update(default_conn, conn_param);
		if (err) {
			printk(
				    "Connection parameters update failed: %d",
				    err);
			return err;
		}

		printk("Connection parameters update pending\n");
		err = k_sem_take(&throughput_sem, THROUGHPUT_CONFIG_TIMEOUT);
		if (err) {
			printk(
				    "Connection parameters update timeout");
			return err;
		}
	}
	return 0;
}

int test_run(
  const struct shell *shell,
  const struct bt_le_conn_param *conn_param,
  const struct bt_conn_le_phy_param *phy,
  const struct bt_conn_le_data_len_param *data_len
){
	int err;
	uint64_t stamp;
	int64_t delta;
	uint32_t data = 0;
	uint32_t prog = 0;

	/* a dummy data buffer */
	static char dummy[509];

	if (!default_conn) {
		shell_error(shell, "Device is disconnected %s",
			    "Connect to the peer device before running test");
		return -EFAULT;
	}

	shell_print(shell, "\n==== Starting throughput test ====");

	/* Make sure that all BLE procedures are finished. */
	k_sleep(K_MSEC(500));

	/* get cycle stamp */
	stamp = k_uptime_get_32();

	while (prog < IMG_SIZE) {

    switch (throughput_mode)
    {
      case NOTIFICATION:
        send_notification(dummy, 495);//bt_throughput_write(&throughput, dummy, 495);
        break;

      case INDICATION:
        err = k_sem_take(send_indication(dummy, 495), THROUGHPUT_CONFIG_TIMEOUT);
        if (err) {
          printk("k_sem_take failed (err %d)", err);
          break;
        }
        break;
    }
		/* print graphics */
		printk("%c", img[prog / IMG_X][prog % IMG_X]);
		data += 495;
		prog++;
	}

	delta = k_uptime_delta(&stamp);

	printk("\nDone\n");
	printk("[local] sent %u bytes (%u KB) in %lld ms at %llu kbps\n",
    data, 
    data / 1024, 
    delta, 
    ((uint64_t)data * 8 / delta));

	k_sem_take(&throughput_sem, THROUGHPUT_CONFIG_TIMEOUT);

	instruction_print();

	return 0;
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
	.le_param_req = le_param_req,
	.le_param_updated = le_param_updated,
	.le_phy_updated = le_phy_updated,
	.le_data_len_updated = le_data_length_updated
};

void main(void)
{
	int err;

	printk("Starting Bluetooth Mobile Throughput example\n");

	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return;
	}

	printk("Bluetooth initialized\n");

	err = bt_throughput_init(&throughput, &throughput_cb);
	if (err) {
		printk("Throughput service initialization failed.\n");
		return;
	}

	buttons_init();
}
