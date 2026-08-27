/* SPDX-License-Identifier: MIT */
#include "podbay/imx582_mode.h"

#include <stdbool.h>
#include <string.h>

static int append(struct podbay_imx582_mode *mode, uint16_t address, uint8_t value)
{
	if (mode->write_count == PODBAY_IMX582_MAX_MODE_WRITES)
		return PODBAY_IMX582_PLAN_OVERFLOW;
	mode->writes[mode->write_count].address = address;
	mode->writes[mode->write_count].value = value;
	mode->write_count++;
	return PODBAY_IMX582_OK;
}

static int append_u16(struct podbay_imx582_mode *mode, uint16_t address,
		      uint16_t value)
{
	int result = append(mode, address, (uint8_t)(value >> 8));
	if (result != PODBAY_IMX582_OK)
		return result;
	return append(mode, (uint16_t)(address + 1u), (uint8_t)value);
}

static bool valid_binning(uint8_t binning)
{
	return binning == 1u || binning == 2u || binning == 4u;
}

#define APPEND(address, value) do { \
	result = append(mode, (address), (value)); \
	if (result != PODBAY_IMX582_OK) \
		return result; \
} while (0)

#define APPEND_U16(address, value) do { \
	result = append_u16(mode, (address), (value)); \
	if (result != PODBAY_IMX582_OK) \
		return result; \
} while (0)

int podbay_imx582_plan_mode(const struct podbay_imx582_roi *roi,
			   struct podbay_imx582_mode *mode)
{
	uint16_t x_end;
	uint16_t y_end;
	int result;

	if (roi == NULL || mode == NULL)
		return PODBAY_IMX582_INVALID_ARGUMENT;
	memset(mode, 0, sizeof(*mode));
	if (!valid_binning(roi->binning))
		return PODBAY_IMX582_INVALID_BINNING;
	if (roi->width == 0u || roi->height == 0u ||
	    roi->x % roi->binning != 0u || roi->y % roi->binning != 0u ||
	    roi->width % roi->binning != 0u ||
	    roi->height % roi->binning != 0u ||
	    (uint32_t)roi->x + roi->width > PODBAY_IMX582_ARRAY_WIDTH ||
	    (uint32_t)roi->y + roi->height > PODBAY_IMX582_ARRAY_HEIGHT)
		return PODBAY_IMX582_INVALID_ROI;

	x_end = (uint16_t)(roi->x + roi->width - 1u);
	y_end = (uint16_t)(roi->y + roi->height - 1u);
	mode->output_width = (uint16_t)(roi->width / roi->binning);
	mode->output_height = (uint16_t)(roi->height / roi->binning);

	/* Keep the sensor stopped while the future kernel adapter configures VIF. */
	APPEND(0x0100u, 0x00u);
	APPEND(0x0104u, 0x01u);
	APPEND_U16(0x0344u, roi->x);
	APPEND_U16(0x0346u, roi->y);
	APPEND_U16(0x0348u, x_end);
	APPEND_U16(0x034au, y_end);

	APPEND(0x0900u, roi->binning == 1u ? 0x00u : 0x01u);
	APPEND(0x0901u, (uint8_t)((roi->binning << 4) | roi->binning));
	APPEND(0x0902u, roi->binning == 1u ? 0x0au : 0x08u);

	APPEND_U16(0x0408u, 0u);
	APPEND_U16(0x040au, 0u);
	APPEND_U16(0x040cu, mode->output_width);
	APPEND_U16(0x040eu, mode->output_height);
	APPEND_U16(0x034cu, mode->output_width);
	APPEND_U16(0x034eu, mode->output_height);

	if (roi->binning == 1u) {
		APPEND(0x3246u, 0x01u);
		APPEND(0x3247u, 0x01u);
		APPEND(0x3620u, 0x00u);
		APPEND(0x3c13u, 0x2au);
		APPEND(0x3f0cu, 0x00u);
		APPEND(0x3f14u, 0x01u);
		APPEND(0x3f80u, 0x02u);
		APPEND(0x3f81u, 0x00u);
		APPEND(0x3f8cu, 0x01u);
		APPEND(0x3f8du, 0x00u);
	}

	APPEND(0x0104u, 0x00u);
	return PODBAY_IMX582_OK;
}
