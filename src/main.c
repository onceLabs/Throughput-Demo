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

#include <zephyr/sys/byteorder.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/crypto.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/hci_vs.h>

#include "throughput_demo.h"
#include <bluetooth/scan.h>
#include <zephyr/bluetooth/conn.h>
#include <bluetooth/gatt_dm.h>
#include <zephyr/bluetooth/l2cap.h>
#include <zephyr/shell/shell_uart.h>
#include <dk_buttons_and_leds.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(main, 3);

static const unsigned char joe_ascii[] =                                                                                            
"                                        .~?YPGGGPPY?~:..                                            \n"
"                                     :7YPGBBGGGGGBB###BBBGY!:.                                      \n"
"                                   .?5PPPP5PPPGBGBBGGB##&&&##GY~:                                   \n"
"                                 ~JY5PPGPPBGGB##&&########&&&&##BGY:                                \n"
"                               .7Y5PBBGPPPPPGGBGGBB#&&&#&&&######BBBP!                              \n"
"                             ^5GBBGP5Y?77!!7777777?JY5PGB##BB##&##BBBG7                             \n"
"                          :!Y#&#B5?7!!~~~~~~~~~~!!!777??JY55PGB#&&#BBBG^                            \n"
"                         ^###&#P?!~~~~~~~~~~~~~~~~!!!7777??JYY55GB#BBBB5                            \n"
"                        .G#B#BY7~~~~~~~~~~~~~~~~~~~!!!777777???JYY5BB#B?                            \n"
"                        ?#BGGY7~~~~~~~~~^^^^^~~~~~~~!!7777777!!!77?YG&#5.                           \n"
"                        !BBPY7!~~~~~^^^^^::^^^^^^~~~!!!777777!!!!7??JB##!                           \n"
"                        ^GG5?!~~~~~^^^^::::::^^^^~~~!!!77777??777????5B#J                           \n"
"                        7GPJ7!!!~^^^^^::::::^^^~~~~!!!777?????????J??JG#P                           \n"
"                        JGPY?77~^^^^^^::^^^^^^~~~~!!!!!!!7777777?JJJJYP#G                           \n"
"                       .5G55Y7!~^^^^^^^^:::^^^~~!!!!!~^~~!!777777JJJJYG&B                           \n"
"                       .GG55?~^^^^^^^^^^:::^^^~~!!!!!~^~~!77777777?JJJP&#.                          \n"
"                       !GGP57~~~~^^~~!!7!7?Y5YYJJ?7!~!!7JY5PPPP55YYJJJY#&7                          \n"
"                    .^75GGGP?~~~~^~!77?JY5PPGP5YJ?!~!J5PBBBBBBBBBBGPYJY##57.                        \n"
"                   :Y77~~!JP?~~~~~~!7JY5PG##P55J7~^^!5BBBGPB&#BB##BG5YYBGPG?                        \n"
"                   .77~~^^~Y7^~~~~~!7??77J55JJJ7^^^~~JGBG5J5PP55PPPP5YYG5GBJ                        \n"
"                   .7!~^^~!J7^^~~~~~!~~~~~!7??7~^~~~~7JPP5YYYY55YYJJYYYPPPBJ                        \n"
"                    ^7~^^!~7J~~~~~~~~~^~~~~~~^^^^^^^^!7JYYYJ??J?????JY5GG5P!                        \n"
"                     :!~^~^~Y?!~~~~~~^^^^^~~^^^^^^^^^~!7?JYJ????777?JY5G55?                         \n"
"                      .~~^^^?Y7!!~~~~^^^^^^^~~^^^^^^^~!7??J5YJ????JJY5PGJY:                         \n"
"                        :~~^7Y?!!!!~~~~~~~~!!!~~~!!!!7?YP5YGB5YYY55PP5BP5!                          \n"
"                         .~?Y5Y77!!7!!!77!!77~~7J555Y5GB##BBBGP55PGGP5BB7                           \n"
"                           :555JJJ??????7?JYYYY5PPGPPG55GBBBGGBBGGGBGGB?                            \n"
"                            .5PYYJJYYYYYYP555YJJYY55PPPGBBBBBBB##BBBBBB:                            \n"
"                             ~P5YJJY555YPGGGGPY?7!!!7??Y5GB&&#BB#BB#BBY                             \n"
"                              YPYYYYY5P55P5?~!77!~!!777?Y555PPPPBB###B:                             \n"
"                              7J5555P55PPGPJ7~~~~~!?JJJYY5Y555PGB###&?                              \n"
"                             ::~7Y5PPGPGB##GYJ7777?JY5YYYYY5PGB#&&&#Y                               \n"
"                           :~..:~~7YPGGGBBBBGPPYYJJY5P5P55PBB##&&&B~                                \n"
"                         .!?....:^~!7YPGBBGB##BGP5YYY555PGB#&#&##P.                                 \n"
"                        ^!7!..   .~~~!7JPGB#BB#BBPGPGGGGGBB#&##PY?                                  \n"
"                      .~!!77:  .  .^!!!!7?Y5PGB##BBGBBB#B#&#BPY5G?^                                 \n"
"                     ^!!!!!77.  .   :!7!!!!77?JY5PPGGBBBBGPYYJYB#57~.                               \n"
"                  .^!!!!!!7777. .     :!?7!!!!!!!7?JY55P5YJJJJP#BY?!Y7:.                            \n"
"              .:^!!7!!!!!!!77?7.        :!?7!!!!!!!7?JYYYJJJJ5B&G?!~~Y55J!^:.                       \n"
"          .:~!77777?777777!777?!.        .:!??77!!77?JYJJJJJYG##Y~^^:~YY5P55YJ7!^..                 \n"
"     .:^~!77777777????777777777?7.          .~???77?JJJJJ??J5B#G~^::::!YY5PP555555YJ?!^:.           \n"
"::^~!7777777777777777???7777777?J7:           .^!?JJJJJJ???YGBB7::.:77!JYY5GPP5555555555YJJ?7~^.    \n"
"!!777777777777777???7?????7777?JJ?7:           .:^~?JYJ?7?YGBB5:..^?J??555GGGGP5555YYYYYYYYYYYYYJ7^.\n"
"7777777777?77?7?77???JJJJ???????777!.       .^~!!^::^!???YPBB5^:~?YYJY5Y5P55PPPP5555YYYYYYYYJJJJYY55\n"
"7777?7???????????????Y5YYYJ?J?777??7!.    .~?JJ7~^:^:::7J5GG?^~JYPY5GGPY5PP555Y555555YYYYYYYYJJJJJY5\n";

#define ANDROID_MIN_CONN_INTERVAL 0x0006
#define ANDROID_MAX_CONN_INTERVAL 0x0006

#define IOS_MIN_CONN_INTERVAL 0x000c
#define IOS_MAX_CONN_INTERVAL 0x000c

#define IOS_MIN_CONN_INTERVAL_HID 0x0009
#define IOS_MAX_CONN_INTERVAL_HID 0x0009

#define IOS_MIN_CONN_INTERVAL_L 0x0018
#define IOS_MAX_CONN_INTERVAL_L 0x0024

#define SUPERVISION_TIMEOUT 200
#define SUPERVISION_TIMEOUT_ANDROID 100
#define CONN_LATENCY 0

#define CONN_STACK_SIZE 1024
#define TP_THREAD_SIZE 1024

#define DEVICE_NAME	CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)
#define INTERVAL_MIN	0x032	/* 320 units, 400 ms */
#define INTERVAL_MAX	0x032	/* 320 units, 400 ms */
#define THROUGHPUT_CONFIG_TIMEOUT K_SECONDS(20)

K_THREAD_STACK_DEFINE(adv_stack_area, CONN_STACK_SIZE);
K_THREAD_STACK_DEFINE(tp_stack_area, TP_THREAD_SIZE);
struct k_thread adv_thread_data;
struct k_thread tp_thread_data;

static K_SEM_DEFINE(test_run_sem, 0, 1);
static K_SEM_DEFINE(throughput_sem, 0, 1);
static K_SEM_DEFINE(conn_negotiation, 0, 1);
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
      IOS_MIN_CONN_INTERVAL_L, 
      IOS_MAX_CONN_INTERVAL_L, 
      CONN_LATENCY,
			SUPERVISION_TIMEOUT),
	.phy = BT_CONN_LE_PHY_PARAM_2M,
	.data_len = BT_LE_DATA_LEN_PARAM_MAX
};

test_params_t ios_hid = {
  .conn_param = 
    BT_LE_CONN_PARAM(
      IOS_MIN_CONN_INTERVAL_HID, 
      IOS_MAX_CONN_INTERVAL_HID, 
      CONN_LATENCY,
			SUPERVISION_TIMEOUT),
	.phy = BT_CONN_LE_PHY_PARAM_2M,
	.data_len = BT_LE_DATA_LEN_PARAM_MAX
};

test_params_t android_optimal = {
  .conn_param = 
    BT_LE_CONN_PARAM(
      ANDROID_MIN_CONN_INTERVAL, 
      ANDROID_MAX_CONN_INTERVAL, 
      CONN_LATENCY,
			SUPERVISION_TIMEOUT_ANDROID),
	.phy = BT_CONN_LE_PHY_PARAM_2M,
	.data_len = BT_LE_DATA_LEN_PARAM_MAX
};
/******************************************

 ******************************************/
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

static int conn_configuration_thread();
static int start_test();
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

static void set_tx_power(uint8_t handle_type, uint16_t handle, int8_t tx_pwr_lvl)
{
	struct bt_hci_cp_vs_write_tx_power_level *cp;
	struct bt_hci_rp_vs_write_tx_power_level *rp;
	struct net_buf *buf, *rsp = NULL;
	int err;

	buf = bt_hci_cmd_create(BT_HCI_OP_VS_WRITE_TX_POWER_LEVEL,
				sizeof(*cp));
	if (!buf) {
		printk("Unable to allocate command buffer\n");
		return;
	}

	cp = net_buf_add(buf, sizeof(*cp));
	cp->handle = sys_cpu_to_le16(handle);
	cp->handle_type = handle_type;
	cp->tx_power_level = tx_pwr_lvl;

	err = bt_hci_cmd_send_sync(BT_HCI_OP_VS_WRITE_TX_POWER_LEVEL,
				   buf, &rsp);
	if (err) {
		uint8_t reason = rsp ?
			((struct bt_hci_rp_vs_write_tx_power_level *)
			  rsp->data)->status : 0;
		printk("Set Tx power err: %d reason 0x%02x\n", err, reason);
		return;
	}

	rp = (void *)rsp->data;
	printk("Actual Tx Power: %d\n", rp->selected_tx_power);

	net_buf_unref(rsp);
}

static void exchange_func(
  struct bt_conn *conn, 
  uint8_t att_err,
	struct bt_gatt_exchange_params *params
){
	struct bt_conn_info info = {0};
	int err;

	LOG_INF("MTU exchange %s\n", att_err == 0 ? "successful" : "failed");

	err = bt_conn_get_info(conn, &info);
	if (err) {
    LOG_ERR("bt_conn_get_info failed (err %d)\n", err);
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
){
	printk("Error while discovering GATT database: (%d)\n", err);
}

struct bt_gatt_dm_cb discovery_cb = {
	.completed         = discovery_complete,
	.service_not_found = discovery_service_not_found,
	.error_found       = discovery_error,
};

static void connected(
  struct bt_conn *conn, 
  uint8_t hci_err
){
	struct bt_conn_info info = {0};
	int err;

	if (hci_err) {
		if (hci_err == BT_HCI_ERR_UNKNOWN_CONN_ID) {
			/* Canceled creating connection */
			return;
		}

		LOG_ERR("Connection failed (err 0x%02x)\n", hci_err);
		return;
	}

	if (default_conn) {
		LOG_INF("Connection exists, disconnect second connection\n");
		bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
		return;
	}

	default_conn = bt_conn_ref(conn);
  

	err = bt_conn_get_info(default_conn, &info);
	if (err) {
    LOG_ERR("bt_conn_get_info failed (err 0x%02x)\n", hci_err);
		return;
	}

  set_tx_power(BT_HCI_VS_LL_HANDLE_TYPE_CONN, 1, 8);

	LOG_INF("Connected as %s\n", info.role == BT_CONN_ROLE_CENTRAL ? "central" : "peripheral");
	LOG_INF("Conn. interval is %u units\n", info.le.interval);

  // Create the thread
  k_thread_create(
      &adv_thread_data, adv_stack_area,
      K_THREAD_STACK_SIZEOF(adv_stack_area),
      conn_configuration_thread,
      NULL, NULL, NULL,
      5, 0, K_NO_WAIT);

  // Create the thread
  k_thread_create(
      &tp_thread_data, tp_stack_area,
      K_THREAD_STACK_SIZEOF(tp_stack_area),
      start_test,
      NULL, NULL, NULL,
      0, 0, K_NO_WAIT);
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
    LOG_ERR("bt_le_adv_start failed (err %d)\n", err);
		return;
	}
}

static void disconnected(
  struct bt_conn *conn, 
  uint8_t reason
){
	struct bt_conn_info info = {0};
	int err;

	LOG_ERR("Disconnected (reason 0x%02x)\n", reason);

	test_ready = false;
	if (default_conn) {
		bt_conn_unref(default_conn);
		default_conn = NULL;
	}

	err = bt_conn_get_info(conn, &info);
	if (err) {
    LOG_ERR("bt_conn_get_info failed (err %d)\n", err);
		return;
	}

	adv_start();
}

static bool le_param_req(
  struct bt_conn *conn, 
  struct bt_le_conn_param *param
){
	LOG_INF("Connection parameters update request received.\n");

	LOG_INF("Minimum interval: %d, Maximum interval: %d\n",
    param->interval_min, 
    param->interval_max);

	LOG_INF("Latency: %d, Timeout: %d\n", 
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
	LOG_INF("Connection parameters updated.\n"
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
	LOG_INF("LE PHY updated: TX PHY %s, RX PHY %s\n",
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

	LOG_INF("LE data len updated: TX (len: %d time: %d)"
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
	LOG_INF("[peer] received %u bytes (%u KB)"
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
		LOG_INF("\n");
		return;
	}

	if ((met->write_len / 1024) != kb) {
		kb = (met->write_len / 1024);
		LOG_INF("=");
	}

  LOG_INF("\n[local] received %u bytes (%u KB)"
		" in %u GATT writes at %u bps\n",
		met->write_len, 
    met->write_len / 1024,
		met->write_count, 
    met->write_rate);
}

static void throughput_send(const struct bt_throughput_metrics *met)
{
	LOG_INF("\n[local] received %u bytes (%u KB)"
		" in %u GATT writes at %u bps\n",
		met->write_len, 
    met->write_len / 1024,
		met->write_count, 
    met->write_rate);
}

static void throughput_mode_change(throughput_mode_t mode)
{
  LOG_INF("Throughput Mode Updated: %d\n", mode);
  throughput_mode = mode;
}

static void throughput_command_received(throughput_command_t command)
{
  switch (command)
  {
    case START:
      k_sem_give(&test_run_sem);
      break;
    case STOP:
      break;
    case RESET:
      break;
  };
}

static const struct bt_throughput_cb throughput_cb = {
	.data_read = throughput_read,
	.data_received = throughput_received,
	.data_send = throughput_send,
  .mode_change = throughput_mode_change,
  .command_received = throughput_command_received
};

static struct button_handler button = {
	.cb = button_handler_cb,
};

static void button_handler_cb(uint32_t button_state, uint32_t has_changed)
{
	int err;
	uint32_t buttons = button_state & has_changed;

  LOG_INF("\nPeripheral. Starting advertising\n");
  adv_start();

	/* The role has been selected, button are not needed any more. */
	err = dk_button_handler_remove(&button);
	if (err) {
		LOG_ERR("dk_button_handler_remove error: %d\n", err);
	}
}

static void buttons_init(void)
{
	int err;

	err = dk_buttons_init(NULL);
	if (err) {
    LOG_ERR("dk_buttons_init failed (err %d)\n", err);
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

static int conn_configuration_thread()
{
	int err;
	struct bt_conn_info info = {0};

  // const struct bt_le_conn_param *conn_param = ios_hid.conn_param;
  // const struct bt_conn_le_phy_param *phy = ios_hid.phy;
  // const struct bt_conn_le_data_len_param *data_len = ios_hid.data_len;

  // const struct bt_le_conn_param *conn_param = ios_normal.conn_param;
  // const struct bt_conn_le_phy_param *phy = ios_normal.phy;
  // const struct bt_conn_le_data_len_param *data_len = ios_normal.data_len;

  // const struct bt_le_conn_param *conn_param = ios_optimal.conn_param;
  // const struct bt_conn_le_phy_param *phy = ios_optimal.phy;
  // const struct bt_conn_le_data_len_param *data_len = ios_optimal.data_len;

  const struct bt_le_conn_param *conn_param = android_optimal.conn_param;
  const struct bt_conn_le_phy_param *phy = android_optimal.phy;
  const struct bt_conn_le_data_len_param *data_len = android_optimal.data_len;

    /* Make sure that all BLE procedures are finished. */
    k_sleep(K_MSEC(5000));

	err = bt_conn_get_info(default_conn, &info);
	if (err) {    
    LOG_ERR("bt_conn_get_info failed (err %d)\n", err);
		return err;
	}

  /* PHY Update */
	err = bt_conn_le_phy_update(default_conn, phy);
	if (err) {
    LOG_ERR("bt_conn_le_phy_update failed (err %d)\n", err);
		return err;
	}; printk("PHY update pending\n");

	err = k_sem_take(&throughput_sem, THROUGHPUT_CONFIG_TIMEOUT);
	if (err) {
    LOG_ERR("k_sem_take failed (err %d)\n", err);
		return err;
	}

  /* Data length update */
	if (info.le.data_len->tx_max_len != data_len->tx_max_len) {
		data_length_req = true;

		err = bt_conn_le_data_len_update(default_conn, data_len);
		if (err) {
      LOG_ERR("bt_conn_le_data_len_update failed (err %d)\n", err);
			return err;
		}

		printk("LE Data length update pending\n");
		err = k_sem_take(&throughput_sem, THROUGHPUT_CONFIG_TIMEOUT);
		if (err) {
      LOG_ERR("k_sem_take failed (err %d)\n", err);
			return err;
		}
	}

  /* Connection parameter update */
	if (info.le.interval != conn_param->interval_max) {
		err = bt_conn_le_param_update(default_conn, conn_param);
		if (err) {
      LOG_ERR("bt_conn_le_param_update failed (err %d)\n", err);
			return err;
		}

		printk("Connection parameters update pending\n");
		err = k_sem_take(&throughput_sem, THROUGHPUT_CONFIG_TIMEOUT);
		if (err) {
      LOG_ERR("k_sem_take failed (err %d)\n", err);
			return err;
		}
	}

  /* MTU Update */
  exchange_params.func = exchange_func;
	err = bt_gatt_exchange_mtu(default_conn, &exchange_params);
	if (err) {
    LOG_ERR("bt_gatt_exchange_mtu failed (err %d)\n", err);
	} else {
		LOG_INF("MTU exchange pending\n");
	}
	return 0;
}

static int start_test()
{
	int err;
	uint64_t stamp;
	int64_t delta;
	uint32_t data = 0;
	uint32_t prog = 0;

  const struct bt_le_conn_param *conn_param = ios_hid.conn_param;
  // const struct bt_conn_le_phy_param *phy = ios_hid.phy;
  // const struct bt_conn_le_data_len_param *data_len = ios_hid.data_len;

	/* a dummy data buffer */
	static char dummy[509];
  for (;;)
  {
    prog = 0;
    data = 0;
    err = k_sem_take(&test_run_sem, K_FOREVER);

    // err = bt_conn_le_param_update(default_conn, conn_param);
		// if (err) {
    //   LOG_ERR("bt_conn_le_param_update failed (err %d)\n", err);
		// 	return err;
		// }

		// printk("Connection parameters update pending\n");
		// err = k_sem_take(&throughput_sem, THROUGHPUT_CONFIG_TIMEOUT);
		// if (err) {
    //   LOG_ERR("k_sem_take failed (err %d)\n", err);
		// 	return err;
		// }

    if (err) {
      LOG_ERR("k_sem_take failed (err %d)\n", err);
      break;
    }

    if (!default_conn) {
      LOG_INF("Device is disconnected %s", "Connect to the peer device before running test");
      return -EFAULT;
    }
    printk("\n============= Starting throughput test =================\n");
    /* Make sure that all BLE procedures are finished. */
    k_sleep(K_MSEC(500));

    /* get cycle stamp */
    stamp = k_uptime_get_32();
    for (int i = 0; i < sizeof(joe_ascii); i++){
    //while (prog < IMG_SIZE) {

      switch (throughput_mode)
      {
        case NOTIFICATION:
          send_notification(dummy, 495);//bt_throughput_write(&throughput, dummy, 495);
          break;

        case INDICATION:
          err = k_sem_take(send_indication(dummy, 495), THROUGHPUT_CONFIG_TIMEOUT);
          if (err) {
            LOG_ERR("k_sem_take failed (err %d)\n", err);
            break;
          }
          break;
      }
      /* print graphics */
      printk("%c", joe_ascii[i]);//[prog / IMG_X][prog % IMG_X]);
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
  printk("Throughput Mode %d", throughput_mode);
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
