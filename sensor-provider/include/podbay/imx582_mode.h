/* SPDX-License-Identifier: MIT */
#ifndef PODBAY_IMX582_MODE_H
#define PODBAY_IMX582_MODE_H

#include <stddef.h>
#include <stdint.h>

#define PODBAY_IMX582_ARRAY_WIDTH 8000u
#define PODBAY_IMX582_ARRAY_HEIGHT 6000u
#define PODBAY_IMX582_MAX_MODE_WRITES 40u

struct podbay_imx582_roi {
	uint16_t x;
	uint16_t y;
	uint16_t width;
	uint16_t height;
	uint8_t binning;
};

struct podbay_sensor_write {
	uint16_t address;
	uint8_t value;
};

struct podbay_imx582_mode {
	struct podbay_sensor_write writes[PODBAY_IMX582_MAX_MODE_WRITES];
	size_t write_count;
	uint16_t output_width;
	uint16_t output_height;
};

enum podbay_imx582_result {
	PODBAY_IMX582_OK = 0,
	PODBAY_IMX582_INVALID_ARGUMENT = -1,
	PODBAY_IMX582_INVALID_BINNING = -2,
	PODBAY_IMX582_INVALID_ROI = -3,
	PODBAY_IMX582_PLAN_OVERFLOW = -4,
};

/*
 * Build a deterministic, side-effect-free register plan. The caller owns I2C,
 * delay, retry, and rollback policy. In particular, this function never starts
 * streaming; a kernel adapter must do that only after receiver configuration.
 */
int podbay_imx582_plan_mode(const struct podbay_imx582_roi *roi,
			   struct podbay_imx582_mode *mode);

#endif
