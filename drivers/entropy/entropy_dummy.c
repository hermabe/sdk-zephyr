/*
 * Copyright (c) 2022, Commonwealth Scientific and Industrial Research
 * Organisation (CSIRO) ABN 41 687 119 230.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT zephyr_dummy_entropy

#include <zephyr/drivers/entropy.h>
#include <string.h>

static int entropy_init(const struct device *dev)
{
	/* Nothing to do */
	return 0;
}

static int dummy_entropy_get_entropy(const struct device *dev,
				  uint8_t *buffer, uint16_t length)
{
	/* https://xkcd.com/221/ */
	memset(buffer, 4, length);

	return 0;
}

static int dummy_entropy_get_entropy_isr(const struct device *dev,
				  uint8_t *buffer, uint16_t length, uint32_t flags)
{
	/* https://xkcd.com/221/ */
	memset(buffer, 4, length);

	return 0;
}

/* HCI commands cannot be run from an interrupt context */
static const struct entropy_driver_api entropy_api = {
	.get_entropy = dummy_entropy_get_entropy,
	.get_entropy_isr = dummy_entropy_get_entropy_isr
};

#define ENTROPY_DUMMY_INIT(inst)				  \
	DEVICE_DT_INST_DEFINE(inst, entropy_init,		  \
			      NULL, NULL, NULL,			  \
			      PRE_KERNEL_1,			  \
			      CONFIG_KERNEL_INIT_PRIORITY_DEVICE, \
			      &entropy_api);

DT_INST_FOREACH_STATUS_OKAY(ENTROPY_DUMMY_INIT)
