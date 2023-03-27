/*
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/controller.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/sys/util.h>

#define TIMEOUT_SYNC_CREATE K_SECONDS(10)
#define NAME_LEN	    30

static bool per_adv_found;
static bt_addr_le_t per_addr;
static uint8_t per_sid;

static K_SEM_DEFINE(sem_per_adv, 0, 1);
static K_SEM_DEFINE(sem_per_sync, 0, 1);
static K_SEM_DEFINE(sem_per_sync_lost, 0, 1);

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)

#if DT_NODE_HAS_STATUS(LED0_NODE, okay)
#define HAS_LED 1
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
#define BLINK_ONOFF K_MSEC(500)

static struct k_work_delayable blink_work;
static bool led_is_on;

static void blink_timeout(struct k_work *work)
{
	led_is_on = !led_is_on;
	gpio_pin_set(led.port, led.pin, (int)led_is_on);

	k_work_schedule(&blink_work, BLINK_ONOFF);
}
#endif

static bool data_cb(struct bt_data *data, void *user_data)
{
	char *name = user_data;
	uint8_t len;

	switch (data->type) {
	case BT_DATA_NAME_SHORTENED:
	case BT_DATA_NAME_COMPLETE:
		len = MIN(data->data_len, NAME_LEN - 1);
		memcpy(name, data->data, len);
		name[len] = '\0';
		return false;
	default:
		return true;
	}
}

static void scan_recv(const struct bt_le_scan_recv_info *info, struct net_buf_simple *buf)
{
	char name[NAME_LEN];

	(void)memset(name, 0, sizeof(name));

	bt_data_parse(buf, data_cb, name);

	if (!per_adv_found && info->interval && !strcmp(name, "Zephyr PAwR")) {
		per_adv_found = true;

		per_sid = info->sid;
		bt_addr_le_copy(&per_addr, info->addr);

		k_sem_give(&sem_per_adv);
	}
}

static struct bt_le_scan_cb scan_callbacks = {
	.recv = scan_recv,
};

static void sync_cb(struct bt_le_per_adv_sync *sync, struct bt_le_per_adv_sync_synced_info *info)
{
	struct bt_le_per_adv_sync_subevent_params params;
	uint8_t subevents[5];
	char le_addr[BT_ADDR_LE_STR_LEN];
	int err;

	bt_addr_le_to_str(info->addr, le_addr, sizeof(le_addr));
	printk("Synced to %s with %d subevents\n", le_addr, info->num_subevents);

	params.properties = 0;
	params.num_subevents = MIN(ARRAY_SIZE(subevents), info->num_subevents);
	// params.num_subevents = 1;
	params.subevents = subevents;
	// subevents[0] = 2;

	/* Listen to all subevents */
	for (size_t i = 0; i < params.num_subevents; i++) {
		params.subevents[i] = i;
	}

	err = bt_le_per_adv_sync_subevent(sync, &params);
	if (err) {
		printk("Failed to set subevents to sync to (err %d)\n", err);
	}

	k_sem_give(&sem_per_sync);
}

static void term_cb(struct bt_le_per_adv_sync *sync,
		    const struct bt_le_per_adv_sync_term_info *info)
{
	char le_addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(info->addr, le_addr, sizeof(le_addr));

	printk("Sync terminated (reason %d)\n", info->reason);

	k_sem_give(&sem_per_sync_lost);
}

static bool print_ad_field(struct bt_data *data, void *user_data)
{
	ARG_UNUSED(user_data);

	printk("    0x%02X: ", data->type);
	for (size_t i = 0; i < data->data_len; i++) {
		printk("%02X", data->data[i]);
	}

	printk("\n");

	return true;
}

int bt_le_per_adv_set_response_data(struct bt_le_per_adv_sync *per_adv_sync,
				    const struct bt_le_per_adv_response_params *params,
				    const struct net_buf_simple *data);

static struct bt_le_per_adv_response_params rsp_params;

NET_BUF_SIMPLE_DEFINE_STATIC(rsp_buf, 247);

static void recv_cb(struct bt_le_per_adv_sync *sync,
		    const struct bt_le_per_adv_sync_recv_info *info, struct net_buf_simple *buf)
{
	int err;

	/* Echo the data back to the advertiser */
	net_buf_simple_reset(&rsp_buf);
	net_buf_simple_add_mem(&rsp_buf, buf->data, buf->len);

	rsp_params.request_event = info->periodic_event_counter;
	rsp_params.request_subevent = info->subevent;
	// /* Respond in latest subevent possible. */
	// rsp_params.response_subevent = (info->subevent + 4) % 5;
	/* Respond in current subevent. */
	rsp_params.response_subevent = info->subevent;
	rsp_params.response_slot = 0;

	printk("Indication: subevent %d\n", info->subevent);
	bt_data_parse(buf, print_ad_field, NULL);

	err = bt_le_per_adv_set_response_data(sync, &rsp_params, &rsp_buf);
	if (err) {
		printk("Failed to send response (err %d)\n", err);
	}

	printk("Response sent\n");
}

static struct bt_le_per_adv_sync_cb sync_callbacks = {
	.synced = sync_cb,
	.term = term_cb,
	.recv = recv_cb,
};

static struct bt_conn *conn;

void connected(struct bt_conn *c, uint8_t err)
{
	printk("Connected (err 0x%02X)\n", err);

	if (err) {
		conn = NULL;
	} else {
		conn = c;
	}
}

void disconnected(struct bt_conn *conn, uint8_t reason)
{
	printk("Disconnected (reason 0x%02X)\n", reason);
}

BT_CONN_CB_DEFINE(conn_cb) = {
	.connected = connected,
	.disconnected = disconnected,
};

void main(void)
{
	struct bt_le_per_adv_sync_param sync_create_param;
	struct bt_le_per_adv_sync *sync;
	int err;

	printk("Starting Periodic Advertising Synchronization Demo\n");

#if defined(HAS_LED)
	printk("Checking LED device...");
	if (!device_is_ready(led.port)) {
		printk("failed.\n");
		return;
	}
	printk("done.\n");

	printk("Configuring GPIO pin...");
	err = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
	if (err) {
		printk("failed.\n");
		return;
	}
	printk("done.\n");

	k_work_init_delayable(&blink_work, blink_timeout);
#endif /* HAS_LED */

	static const uint8_t addr[6] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66};
	bt_ctlr_set_public_addr(addr);

	/* Initialize the Bluetooth Subsystem */
	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return;
	}

	printk("Scan callbacks register...");
	bt_le_scan_cb_register(&scan_callbacks);
	printk("success.\n");

	printk("Periodic Advertising callbacks register...");
	bt_le_per_adv_sync_cb_register(&sync_callbacks);
	printk("Success.\n");

	printk("Start scanning...");
	err = bt_le_scan_start(BT_LE_SCAN_ACTIVE, NULL);
	if (err) {
		printk("failed (err %d)\n", err);
		return;
	}
	printk("success.\n");

	do {
#if defined(HAS_LED)
		struct k_work_sync work_sync;

		printk("Start blinking LED...\n");
		led_is_on = false;
		gpio_pin_set(led.port, led.pin, (int)led_is_on);
		k_work_schedule(&blink_work, BLINK_ONOFF);
#endif /* HAS_LED */

		printk("Waiting for periodic advertising...\n");
		per_adv_found = false;
		err = k_sem_take(&sem_per_adv, K_FOREVER);
		if (err) {
			printk("failed (err %d)\n", err);
			return;
		}
		printk("Found periodic advertising.\n");

		printk("Creating Periodic Advertising Sync...");
		bt_addr_le_copy(&sync_create_param.addr, &per_addr);
		sync_create_param.options = 0;
		sync_create_param.sid = per_sid;
		sync_create_param.skip = 0;
		sync_create_param.timeout = 0xaa;
		err = bt_le_per_adv_sync_create(&sync_create_param, &sync);
		if (err) {
			printk("failed (err %d)\n", err);
			return;
		}
		printk("success.\n");

		printk("Waiting for periodic sync...\n");
		err = k_sem_take(&sem_per_sync, TIMEOUT_SYNC_CREATE);
		if (err) {
			printk("failed (err %d)\n", err);

			printk("Deleting Periodic Advertising Sync...");
			err = bt_le_per_adv_sync_delete(sync);
			if (err) {
				printk("failed (err %d)\n", err);
				return;
			}
			continue;
		}
		printk("Periodic sync established.\n");

#if defined(HAS_LED)
		printk("Stop blinking LED.\n");
		k_work_cancel_delayable_sync(&blink_work, &work_sync);

		/* Keep LED on */
		led_is_on = true;
		gpio_pin_set(led.port, led.pin, (int)led_is_on);
#endif /* HAS_LED */

		printk("Waiting for periodic sync lost...\n");
		err = k_sem_take(&sem_per_sync_lost, K_FOREVER);
		if (err) {
			printk("failed (err %d)\n", err);
			return;
		}
		printk("Periodic sync lost.\n");
	} while (true);
}
