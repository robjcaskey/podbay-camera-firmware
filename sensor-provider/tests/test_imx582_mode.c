/* SPDX-License-Identifier: MIT */
#include "podbay/imx582_mode.h"

#include <assert.h>
#include <stddef.h>

static int find_value(const struct podbay_imx582_mode *mode, uint16_t address)
{
	size_t index;
	for (index = 0; index < mode->write_count; index++) {
		if (mode->writes[index].address == address)
			return mode->writes[index].value;
	}
	return -1;
}

static void test_full_raw_roi(void)
{
	const struct podbay_imx582_roi roi = {2720u, 1392u, 1024u, 384u, 1u};
	struct podbay_imx582_mode mode;

	assert(podbay_imx582_plan_mode(&roi, &mode) == PODBAY_IMX582_OK);
	assert(mode.output_width == 1024u);
	assert(mode.output_height == 384u);
	assert(mode.write_count == 36u);
	assert(mode.writes[0].address == 0x0100u);
	assert(mode.writes[0].value == 0x00u);
	assert(find_value(&mode, 0x0344u) == 0x0au);
	assert(find_value(&mode, 0x0345u) == 0xa0u);
	assert(find_value(&mode, 0x0348u) == 0x0eu);
	assert(find_value(&mode, 0x0349u) == 0x9fu);
	assert(find_value(&mode, 0x0900u) == 0x00u);
	assert(find_value(&mode, 0x0901u) == 0x11u);
	assert(find_value(&mode, 0x0902u) == 0x0au);
	assert(find_value(&mode, 0x034cu) == 0x04u);
	assert(find_value(&mode, 0x034du) == 0x00u);
	assert(mode.writes[mode.write_count - 1u].address == 0x0104u);
	assert(mode.writes[mode.write_count - 1u].value == 0x00u);
}

static void test_binned_roi(void)
{
	const struct podbay_imx582_roi roi = {0u, 0u, 8000u, 6000u, 4u};
	struct podbay_imx582_mode mode;

	assert(podbay_imx582_plan_mode(&roi, &mode) == PODBAY_IMX582_OK);
	assert(mode.output_width == 2000u);
	assert(mode.output_height == 1500u);
	assert(mode.write_count == 26u);
	assert(find_value(&mode, 0x0900u) == 0x01u);
	assert(find_value(&mode, 0x0901u) == 0x44u);
	assert(find_value(&mode, 0x0902u) == 0x08u);
	assert(find_value(&mode, 0x3246u) == -1);
}

static void test_rejections(void)
{
	struct podbay_imx582_mode mode;
	struct podbay_imx582_roi roi = {0u, 0u, 100u, 100u, 3u};

	assert(podbay_imx582_plan_mode(NULL, &mode) == PODBAY_IMX582_INVALID_ARGUMENT);
	assert(podbay_imx582_plan_mode(&roi, &mode) == PODBAY_IMX582_INVALID_BINNING);
	roi.binning = 4u;
	roi.width = 0u;
	assert(podbay_imx582_plan_mode(&roi, &mode) == PODBAY_IMX582_INVALID_ROI);
	roi.width = 100u;
	roi.x = 7996u;
	assert(podbay_imx582_plan_mode(&roi, &mode) == PODBAY_IMX582_INVALID_ROI);
	roi.x = 2u;
	assert(podbay_imx582_plan_mode(&roi, &mode) == PODBAY_IMX582_INVALID_ROI);
}

int main(void)
{
	test_full_raw_roi();
	test_binned_roi();
	test_rejections();
	return 0;
}
