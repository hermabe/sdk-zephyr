/*
 * Copyright (c) 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>

static uint8_t mfg_data[5][5];

#define NUM_RSP_SLOTS 5

static const struct bt_le_per_adv_param per_adv_params = {
	.interval_min = 0xFF,
	.interval_max = 0xFF,
	.options = 0,
	.num_subevents = 5,
	.subevent_interval = 0x30,
	.response_slot_delay = 0x5,
	.response_slot_spacing = 0x50,
	.num_response_slots = NUM_RSP_SLOTS,
};

static struct bt_le_per_adv_subevent_data_params subevent_data_params[5];

// BUILD_ASSERT(ARRAY_SIZE(ad) == ARRAY_SIZE(subevent_data_params));
BUILD_ASSERT(ARRAY_SIZE(mfg_data) == ARRAY_SIZE(subevent_data_params));

static uint8_t counter;

static void request(struct bt_le_ext_adv *adv, const struct bt_le_per_adv_data_request *request)
{
	int err;
	uint8_t to_send;

	printk("Data request: start %d, count %d\n", request->start, request->count);

	to_send = MIN(request->count, ARRAY_SIZE(subevent_data_params));

	// printk("Setting data for subevents: [");

	for (size_t i = 0; i < to_send; i++) {
		mfg_data[i][ARRAY_SIZE(mfg_data[i]) - 1] = counter++;

		subevent_data_params[i].subevent =
			(request->start + i) % per_adv_params.num_subevents;
		subevent_data_params[i].response_slot_start = 0;
		subevent_data_params[i].response_slot_count = 5;
		subevent_data_params[i].subevent_data_length = ARRAY_SIZE(mfg_data[0]);
		subevent_data_params[i].data = mfg_data[i];

		// printk("%d, ", subevent_data_params[i].subevent);
	}

	// printk("]\n");

	err = bt_le_per_adv_set_subevent_data(adv, to_send, subevent_data_params);
	if (err) {
		printk("Failed to set subevent data (err %d)\n", err);
	}
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

static size_t responses_received;
/* TODO: Get the address from somewhere */
static const bt_addr_le_t peer_addr = {
	.type = BT_ADDR_LE_PUBLIC,
	.a = {{0x11, 0x22, 0x33, 0x44, 0x55, 0x66}},
};

struct bt_conn *conn;

static void response(struct bt_le_ext_adv *adv, struct bt_le_per_adv_response_info *info,
		     struct net_buf_simple *buf)
{
	printk("Response: subevent %d, response slot %d\n", info->subevent, info->response_slot);
	// bt_data_parse(buf, print_ad_field, NULL);

	responses_received++;

	/* Delay the connection attempt a bit */
	if (!conn && ((responses_received % 10) == 0)) {
		struct bt_conn_le_create_synced_param params;
		int err;

		params.subevent = 2;
		params.peer = &peer_addr;

		err = bt_conn_le_create_synced(adv, &params, BT_LE_CONN_PARAM_DEFAULT, &conn);
		if (err) {
			printk("Failed to initiate connection (err %d)\n", err);
		} else {
			printk("Connection initiated at subevent %d\n", params.subevent);
		}
	}
}

static struct bt_le_ext_adv_cb adv_cb = {
	.request = request,
	.response = response,
};

void connected(struct bt_conn *c, uint8_t err)
{
	printk("Connected (err 0x%02X)\n", err);

	if (err) {
		conn = NULL;
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
	struct bt_le_ext_adv *adv;
	int err;

	for (size_t i = 0; i < ARRAY_SIZE(mfg_data); i++) {
		mfg_data[i][0] = ARRAY_SIZE(mfg_data[0]) - 1;
		mfg_data[i][1] = BT_DATA_MANUFACTURER_DATA;
		mfg_data[i][2] = 0x59; /* Nordic */
		mfg_data[i][3] = 0x00;
		mfg_data[i][4] = 0x00;
	}

	printk("Starting Periodic Advertising Demo\n");

	/* Initialize the Bluetooth Subsystem */
	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return;
	}

	/* Create a non-connectable non-scannable advertising set */
	err = bt_le_ext_adv_create(BT_LE_EXT_ADV_NCONN_NAME, &adv_cb, &adv);
	if (err) {
		printk("Failed to create advertising set (err %d)\n", err);
		return;
	}

	/* Set periodic advertising parameters */
	err = bt_le_per_adv_set_param(adv, &per_adv_params);
	if (err) {
		printk("Failed to set periodic advertising parameters"
		       " (err %d)\n",
		       err);
		return;
	}

	/* Enable Periodic Advertising */
	err = bt_le_per_adv_start(adv);
	if (err) {
		printk("Failed to enable periodic advertising (err %d)\n", err);
		return;
	}

	printk("Start Extended Advertising...\n");
	err = bt_le_ext_adv_start(adv, BT_LE_EXT_ADV_START_DEFAULT);
	if (err) {
		printk("Failed to start extended advertising "
		       "(err %d)\n",
		       err);
		return;
	}

	while (true) {
		k_sleep(K_SECONDS(1));
	}
}
