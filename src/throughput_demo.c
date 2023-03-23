/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <string.h>
#include <zephyr/types.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/uuid.h>
#include "throughput_demo.h"

#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(bt_throughput, CONFIG_BT_THROUGHPUT_LOG_LEVEL);

static K_SEM_DEFINE(indicate_sem, 0, 1);

static bool notifications_enabled = false;
static bool indications_enabled = false;
static struct bt_throughput_metrics met;
static const struct bt_throughput_cb *callbacks;

static uint8_t read_fn(
  struct bt_conn *conn, 
  uint8_t err,
	struct bt_gatt_read_params *params, 
  const void *data,
	uint16_t len
){
	struct bt_throughput_metrics metrics;

	memset(&metrics, 0, sizeof(struct bt_throughput_metrics));

	if (data) {
		len = MIN(len, sizeof(struct bt_throughput_metrics));
		memcpy(&metrics, data, len);

		if (callbacks && callbacks->data_read) {
			return callbacks->data_read(&metrics);
		}
	}

	return BT_GATT_ITER_STOP;
}

static ssize_t config_write_callback(
  struct bt_conn *conn,
	const struct bt_gatt_attr *attr, 
  const void *buf,
	uint16_t len, 
  uint16_t offset, 
  uint8_t flags
){
  const uint8_t *write_data = buf;

  switch (write_data[0]){
    case 0: 
      LOG_INF("Throughtput mode change %d", write_data[1]);
      switch (write_data[1])
      {
      case NOTIFICATION:
        if (callbacks && callbacks->mode_change) {
          callbacks->mode_change(NOTIFICATION);
        }
        break;
      case INDICATION:
        if (callbacks && callbacks->mode_change) {
          callbacks->mode_change(INDICATION);
        }
        break;
      case WRITE:
        if (callbacks && callbacks->mode_change) {
          callbacks->mode_change(WRITE);
        }
        break;
      case WRITE_WITHOUT_RESPONSE:
        if (callbacks && callbacks->mode_change) {
          callbacks->mode_change(WRITE_WITHOUT_RESPONSE);
        }
        break;
      default:
        break;
      }
      break;
    case 1:
      LOG_INF("Undefined");
      break;
  }
}

static ssize_t control_write_callback(
  struct bt_conn *conn,
	const struct bt_gatt_attr *attr, 
  const void *buf,
	uint16_t len, 
  uint16_t offset, 
  uint8_t flags
){

  const uint8_t *write_data = buf;
  switch (write_data[0]){
    case 0: 
      LOG_INF("Throughtput mode change %d", write_data[1]);
      switch (write_data[1])
      {
      case START:
        if (callbacks && callbacks->command_received) {
          callbacks->command_received(START);
        }
        break;
      case STOP:
        if (callbacks && callbacks->command_received) {
          callbacks->command_received(STOP);
        }
        break;
      case RESET:
        if (callbacks && callbacks->command_received) {
          callbacks->command_received(RESET);
        }
        break;
      default:
        break;
      }
      break;
    case 1:
      LOG_INF("Undefined");
      break;
  }
  
}

static ssize_t write_callback(
  struct bt_conn *conn,
	const struct bt_gatt_attr *attr, 
  const void *buf,
	uint16_t len, 
  uint16_t offset, 
  uint8_t flags
){
	static uint32_t clock_cycles;
	static uint32_t kb;

	uint64_t delta;

	struct bt_throughput_metrics *met_data = attr->user_data;

	delta = k_cycle_get_32() - clock_cycles;
	delta = k_cyc_to_ns_floor64(delta);

	if (len == 1) {
		/* reset metrics */
		kb = 0;
		met_data->write_count = 0;
		met_data->write_len = 0;
		met_data->write_rate = 0;
		clock_cycles = k_cycle_get_32();
	} else {
		met_data->write_count++;
		met_data->write_len += len;
		met_data->write_rate =
		    ((uint64_t)met_data->write_len << 3) * 1000000000 / delta;
	}

	LOG_INF("Received data. %u", (delta));

	if (callbacks && callbacks->data_received) {
		callbacks->data_received(met_data);
	}

	return len;
}

static ssize_t read_callback(
  struct bt_conn *conn,
	const struct bt_gatt_attr *attr, 
  void *buf,
	uint16_t len, 
  uint16_t offset
){
	const struct bt_throughput_metrics *metrics = attr->user_data;

	len = MIN(sizeof(struct bt_throughput_metrics), len);

	if (callbacks && callbacks->data_send) {
		callbacks->data_send(metrics);
	}

	LOG_DBG("Data send.");

	return bt_gatt_attr_read(
		conn, attr, buf, len, offset, attr->user_data, len);
}

static void _throughput_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
  /* a dummy data buffer */
	static char dummy[495];
	ARG_UNUSED(attr);
  notifications_enabled = (value == BT_GATT_CCC_NOTIFY);
	LOG_INF("Notification %s", notifications_enabled ? "enabled" : "disabled");
}

static void _throughput_cccd_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
  /* a dummy data buffer */
	static char dummy[495];
	ARG_UNUSED(attr);
  indications_enabled = (value == BT_GATT_CCC_INDICATE);
	LOG_INF("Indications %s", indications_enabled ? "enabled" : "disabled");
}

BT_GATT_SERVICE_DEFINE(throughput_svc,
BT_GATT_PRIMARY_SERVICE(BT_UUID_THROUGHPUT),
	BT_GATT_CHARACTERISTIC(BT_UUID_THROUGHPUT_CHAR_N,
		BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP | BT_GATT_CHRC_NOTIFY,
		BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
		read_callback, 
    write_callback, 
    &met),
  BT_GATT_CCC(_throughput_ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
  BT_GATT_CHARACTERISTIC(BT_UUID_THROUGHPUT_CHAR_I,
		BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE | BT_GATT_CHRC_WRITE_WITHOUT_RESP | BT_GATT_CHRC_INDICATE,
		BT_GATT_PERM_READ | BT_GATT_PERM_WRITE,
		read_callback, 
    write_callback, 
    &met),
  BT_GATT_CCC(_throughput_cccd_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
  BT_GATT_CHARACTERISTIC(BT_UUID_THROUGHPUT_CHAR_CONFIG,
		BT_GATT_CHRC_WRITE ,
		BT_GATT_PERM_WRITE,
		NULL, 
    config_write_callback, 
    &met),
  BT_GATT_CHARACTERISTIC(BT_UUID_THROUGHPUT_CHAR_CONTROL,
		BT_GATT_CHRC_WRITE ,
		BT_GATT_PERM_WRITE,
		NULL, 
    control_write_callback, 
    &met),
);
enum {
    HIDS_REMOTE_WAKE = BIT(0),
    HIDS_NORMALLY_CONNECTABLE = BIT(1),
};

struct hids_info {
    uint16_t version; /* version number of base USB HID Specification */
    uint8_t code;     /* country HID Device hardware is localized for. */
    uint8_t flags;
} __packed;

struct hids_report {
    uint8_t id;   /* report id */
    uint8_t type; /* report type */
} __packed;

static struct hids_info info = {
    .version = 0x0000,
    .code = 0x00,
    .flags = HIDS_NORMALLY_CONNECTABLE,
};

enum {
    HIDS_INPUT = 0x01,
    HIDS_OUTPUT = 0x02,
    HIDS_FEATURE = 0x03,
};

static struct hids_report input = {
    .id = 0x01,
    .type = HIDS_INPUT,
};
static uint8_t simulate_input;
static uint8_t ctrl_point;
static uint8_t report_map[] = {
    0x05, 0x01, /* Usage Page (Generic Desktop Ctrls) */
    0x09, 0x02, /* Usage (Mouse) */
    0xA1, 0x01, /* Collection (Application) */
    0x09, 0x01, /*   Usage (Pointer) */
    0xA1, 0x00, /*   Collection (Physical) */
    0x05, 0x09, /*     Usage Page (Button) */
    0x19, 0x01, /*     Usage Minimum (0x01) */
    0x29, 0x03, /*     Usage Maximum (0x03) */
    0x15, 0x00, /*     Logical Minimum (0) */
    0x25, 0x01, /*     Logical Maximum (1) */
    0x95, 0x03, /*     Report Count (3) */
    0x75, 0x01, /*     Report Size (1) */
    0x81, 0x02, /*     Input (Data,Var,Abs,No Wrap,Linear,...) */
    0x95, 0x01, /*     Report Count (1) */
    0x75, 0x05, /*     Report Size (5) */
    0x81, 0x03, /*     Input (Const,Var,Abs,No Wrap,Linear,...) */
    0x05, 0x01, /*     Usage Page (Generic Desktop Ctrls) */
    0x09, 0x30, /*     Usage (X) */
    0x09, 0x31, /*     Usage (Y) */
    0x15, 0x81, /*     Logical Minimum (129) */
    0x25, 0x7F, /*     Logical Maximum (127) */
    0x75, 0x08, /*     Report Size (8) */
    0x95, 0x02, /*     Report Count (2) */
    0x81, 0x06, /*     Input (Data,Var,Rel,No Wrap,Linear,...) */
    0xC0,       /*   End Collection */
    0xC0,       /* End Collection */
};


static ssize_t read_info(struct bt_conn *conn,
                         const struct bt_gatt_attr *attr, void *buf,
                         uint16_t len, uint16_t offset)
{
    return 0;
}

static ssize_t read_report_map(struct bt_conn *conn,
                               const struct bt_gatt_attr *attr, void *buf,
                               uint16_t len, uint16_t offset)
{
    return 0;
}

static ssize_t read_report(struct bt_conn *conn,
                           const struct bt_gatt_attr *attr, void *buf,
                           uint16_t len, uint16_t offset)
{
    return 0;
}

static void input_ccc_changed(const struct bt_gatt_attr *attr, uint16_t value)
{

}

static ssize_t read_input_report(struct bt_conn *conn,
                                 const struct bt_gatt_attr *attr, void *buf,
                                 uint16_t len, uint16_t offset)
{
    return 0;
}

static ssize_t write_ctrl_point(struct bt_conn *conn,
                                const struct bt_gatt_attr *attr,
                                const void *buf, uint16_t len, uint16_t offset,
                                uint8_t flags)
{


    return 0;
}

BT_GATT_SERVICE_DEFINE(hid_svc,
    BT_GATT_PRIMARY_SERVICE(BT_UUID_HIDS),
    BT_GATT_CHARACTERISTIC(BT_UUID_HIDS_INFO, BT_GATT_CHRC_READ,
                           BT_GATT_PERM_READ, read_info, NULL, &info),
    BT_GATT_CHARACTERISTIC(BT_UUID_HIDS_REPORT_MAP, BT_GATT_CHRC_READ,
                           BT_GATT_PERM_READ, read_report_map, NULL, NULL),
    BT_GATT_CHARACTERISTIC(BT_UUID_HIDS_REPORT,
                           BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
                           BT_GATT_PERM_READ_AUTHEN,
                           read_input_report, NULL, NULL),
    // BT_GATT_CCC(input_ccc_changed,
    //             BT_GATT_PERM_READ_AUTHEN | BT_GATT_PERM_WRITE_AUTHEN),
    // BT_GATT_DESCRIPTOR(BT_UUID_HIDS_REPORT_REF, BT_GATT_PERM_READ,
    //                    read_report, NULL, &input),
    BT_GATT_CHARACTERISTIC(BT_UUID_HIDS_CTRL_POINT,
                           BT_GATT_CHRC_WRITE_WITHOUT_RESP,
                           BT_GATT_PERM_WRITE,
                           NULL, write_ctrl_point, &ctrl_point),
);

int bt_throughput_init(
  struct bt_throughput *throughput,
	const struct bt_throughput_cb *cb
){
	if (!throughput || !cb) {
		return -EINVAL;
	}

	callbacks = cb;

	return 0;
}

int bt_throughput_handles_assign(
  struct bt_gatt_dm *dm,
  struct bt_throughput *throughput
){
	const struct bt_gatt_dm_attr *gatt_service_attr =
			bt_gatt_dm_service_get(dm);
	const struct bt_gatt_service_val *gatt_service =
			bt_gatt_dm_attr_service_val(gatt_service_attr);
	const struct bt_gatt_dm_attr *gatt_chrc;
	const struct bt_gatt_dm_attr *gatt_desc;

	if (bt_uuid_cmp(gatt_service->uuid, BT_UUID_THROUGHPUT)) {
		return -ENOTSUP;
	}

	LOG_DBG("Getting handles from Throughput service.");

	/* Throughput Characteristic. */
	gatt_chrc = bt_gatt_dm_char_by_uuid(dm, BT_UUID_THROUGHPUT_CHAR_N);
	if (!gatt_chrc) {
		LOG_ERR("Missing Throughput characteristic.");
	}

	gatt_desc = bt_gatt_dm_desc_by_uuid(dm, gatt_chrc,
					    BT_UUID_THROUGHPUT_CHAR_N);
	if (!gatt_desc) {
		LOG_ERR("Missing Throughput characteristic value descriptor");
		return -EINVAL;
	}

	LOG_DBG("Found handle for Throughput characteristic.");
	throughput->char_handle = gatt_desc->handle;

	/* Assign connection object. */
	throughput->conn = bt_gatt_dm_conn_get(dm);
	return 0;
}

int bt_throughput_read(struct bt_throughput *throughput)
{
	int err;

	throughput->read_params.single.handle = throughput->char_handle;
	throughput->read_params.single.offset = 0;
	throughput->read_params.handle_count = 1;
	throughput->read_params.func = read_fn;

	err = bt_gatt_read(throughput->conn, &throughput->read_params);

	if (err) {
		LOG_ERR("Characteristic read failed.");
	}

	return err;
}

void send_notification(
  const uint8_t *data, 
  uint16_t len
){
  int result;

  /* Only send the response if notifications have been enbabled */
  if (notifications_enabled){
    result = 
      bt_gatt_notify(
        NULL, 
        &throughput_svc.attrs[2],
        data,
        len);

    if (0 != result){
      LOG_ERR("bt_gatt_notify failed (Error: %d)", result);
      return -1;
    }
  }
}

static void on_indication_complete(struct bt_gatt_indicate_params *params)
{
	k_sem_give(&indicate_sem);
}


struct bt_gatt_indicate_params ind_params;
struct k_sem *send_indication(
  const uint8_t *data, 
  uint16_t len
){
  int result;
  
  ind_params.attr = &throughput_svc.attrs[5];
  ind_params.data = data;
  ind_params.len = len;
  ind_params.func = on_indication_complete;
  
  /* Only send the response if notifications have been enbabled */
  if (notifications_enabled){
    result =
      bt_gatt_indicate(
        NULL,
        &ind_params
      );

    if (0 != result){
      LOG_ERR("bt_gatt_indicate failed (Error: %d)", result);
      return -1;
    }
  }

  return &indicate_sem;
}

int bt_throughput_write(
  struct bt_throughput *throughput,
	const uint8_t *data, 
  uint16_t len
){
	return bt_gatt_write_without_response(
    throughput->conn,
    throughput->char_handle,
    data, len, false);
}
