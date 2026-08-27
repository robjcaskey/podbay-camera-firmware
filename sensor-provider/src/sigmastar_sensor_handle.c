/* SPDX-License-Identifier: MIT */
#include "podbay/sigmastar_sensor_handle.h"

#include <stdbool.h>
#include <string.h>

static const uint16_t callback_offsets[PODBAY_SNR_CALLBACK_COUNT] = {
	[PODBAY_SNR_POWER_ON] = 0x09c4u,
	[PODBAY_SNR_POWER_OFF] = 0x09c8u,
	[PODBAY_SNR_SENSOR_INIT] = 0x09ccu,
	[PODBAY_SNR_RELEASE] = 0x09d4u,
	[PODBAY_SNR_SET_PATTERN] = 0x09d8u,
	[PODBAY_SNR_GET_SENSOR_ID] = 0x09dcu,
	[PODBAY_SNR_GET_RESOLUTION] = 0x09e0u,
	[PODBAY_SNR_GET_CURRENT_RESOLUTION] = 0x09e4u,
	[PODBAY_SNR_SET_RESOLUTION] = 0x09e8u,
	[PODBAY_SNR_GET_ORIENTATION] = 0x09ecu,
	[PODBAY_SNR_SET_ORIENTATION] = 0x09f0u,
	[PODBAY_SNR_AE_STATUS] = 0x09f8u,
	[PODBAY_SNR_GET_EXPOSURE] = 0x09fcu,
	[PODBAY_SNR_SET_EXPOSURE] = 0x0a00u,
	[PODBAY_SNR_GET_GAIN] = 0x0a04u,
	[PODBAY_SNR_SET_GAIN] = 0x0a08u,
	[PODBAY_SNR_GET_EXPOSURE_RANGE] = 0x0a0cu,
	[PODBAY_SNR_GET_GAIN_RANGE] = 0x0a10u,
	[PODBAY_SNR_GET_FPS] = 0x0a14u,
	[PODBAY_SNR_SET_FPS] = 0x0a18u,
	[PODBAY_SNR_GET_SHUTTER_INFO] = 0x0a24u,
	[PODBAY_SNR_GET_RESOLUTION_COUNT] = 0x0a28u,
	[PODBAY_SNR_CUSTOM_FUNCTION] = 0x0a2cu,
};

static void store_u16(uint8_t *handle, size_t offset, uint16_t value)
{
	handle[offset] = (uint8_t)value;
	handle[offset + 1u] = (uint8_t)(value >> 8);
}

static void store_u32(uint8_t *handle, size_t offset, uint32_t value)
{
	handle[offset] = (uint8_t)value;
	handle[offset + 1u] = (uint8_t)(value >> 8);
	handle[offset + 2u] = (uint8_t)(value >> 16);
	handle[offset + 3u] = (uint8_t)(value >> 24);
}

static bool bounded_string(const char *text, size_t size, size_t *length)
{
	size_t index;

	if (text == NULL)
		return false;
	for (index = 0; index < size; index++) {
		if (text[index] == '\0') {
			*length = index;
			return true;
		}
	}
	return false;
}

size_t podbay_snr_callback_offset(enum podbay_snr_callback callback)
{
	if (callback < 0 || callback >= PODBAY_SNR_CALLBACK_COUNT)
		return SIZE_MAX;
	return callback_offsets[callback];
}

int podbay_snr_initialize_handle(void *opaque, size_t handle_size,
				 const struct podbay_snr_handle_config *config)
{
	uint8_t *handle = opaque;
	size_t model_length;
	size_t index;

	if (handle == NULL || config == NULL || config->resolutions == NULL)
		return PODBAY_SNR_HANDLE_INVALID_ARGUMENT;
	if (handle_size != PODBAY_SNR_HANDLE_SIZE)
		return PODBAY_SNR_HANDLE_WRONG_SIZE;
	if (!bounded_string(config->model, PODBAY_SNR_MODEL_SIZE, &model_length))
		return PODBAY_SNR_HANDLE_STRING_TOO_LONG;
	if (config->resolution_count == 0u ||
	    config->resolution_count > PODBAY_SNR_MAX_RESOLUTIONS)
		return PODBAY_SNR_HANDLE_INVALID_RESOLUTIONS;

	for (index = 0; index < config->resolution_count; index++) {
		size_t ignored;
		const struct podbay_snr_resolution *resolution =
			&config->resolutions[index];

		if (resolution->capture_width == 0u ||
		    resolution->capture_height == 0u ||
		    resolution->output_width == 0u ||
		    resolution->output_height == 0u ||
		    !bounded_string(resolution->name,
				    PODBAY_SNR_RESOLUTION_NAME_SIZE, &ignored))
			return PODBAY_SNR_HANDLE_INVALID_RESOLUTIONS;
	}
	for (index = 0; index < PODBAY_SNR_CALLBACK_COUNT; index++) {
		if (config->callbacks[index] == 0u)
			return PODBAY_SNR_HANDLE_MISSING_CALLBACK;
	}

	memset(handle + PODBAY_SNR_MODEL_OFFSET, 0, PODBAY_SNR_MODEL_SIZE);
	memcpy(handle + PODBAY_SNR_MODEL_OFFSET, config->model, model_length);

	/* Observed scalar compatibility profile for the reviewed PW203 ABI. */
	store_u32(handle, 0x002cu, 1u);
	store_u32(handle, 0x0030u, 1u);
	store_u32(handle, 0x0034u, 300000u);
	store_u16(handle, 0x0038u, 32u);
	store_u32(handle, 0x0044u, 2u);
	store_u32(handle, 0x0048u, 2u);
	store_u32(handle, 0x004cu, 1u);
	store_u32(handle, 0x0050u, 1u);
	store_u32(handle, 0x0054u, 1u);
	store_u32(handle, 0x0058u, 1u);
	store_u32(handle, 0x0068u, 1u);
	store_u32(handle, 0x006cu, 0u);
	store_u32(handle, 0x0070u, 0u);
	store_u32(handle, 0x0074u, 1u);
	store_u32(handle, 0x0078u, 1u);
	store_u32(handle, 0x007cu, 4u);
	store_u32(handle, 0x0080u, 0u);
	store_u32(handle, PODBAY_SNR_RESOLUTION_COUNT_OFFSET,
		  (uint32_t)config->resolution_count);
	store_u32(handle, PODBAY_SNR_CURRENT_RESOLUTION_OFFSET, 0u);

	for (index = 0; index < config->resolution_count; index++) {
		const struct podbay_snr_resolution *resolution =
			&config->resolutions[index];
		size_t base = PODBAY_SNR_RESOLUTION_OFFSET +
			index * PODBAY_SNR_RESOLUTION_SIZE;
		size_t name_length = strlen(resolution->name);

		memset(handle + base, 0, PODBAY_SNR_RESOLUTION_SIZE);
		store_u32(handle, base + 0x00u, resolution->capture_width);
		store_u32(handle, base + 0x04u, resolution->capture_height);
		store_u32(handle, base + 0x08u, resolution->maximum_fps);
		store_u32(handle, base + 0x0cu, resolution->pixel_mode);
		store_u32(handle, base + 0x10u, resolution->capture_x);
		store_u32(handle, base + 0x14u, resolution->capture_y);
		store_u32(handle, base + 0x18u, resolution->output_width);
		store_u32(handle, base + 0x1cu, resolution->output_height);
		memcpy(handle + base + PODBAY_SNR_RESOLUTION_NAME_OFFSET,
		       resolution->name, name_length);
	}

	store_u32(handle, 0x0948u, 4u);
	store_u32(handle, 0x094cu, 1u);
	store_u32(handle, 0x0950u, 0u);
	store_u32(handle, 0x0954u, 0u);
	store_u32(handle, 0x095cu, 0u);
	store_u32(handle, 0x0960u, 0u);
	store_u32(handle, 0x09b8u, 10u);
	store_u32(handle, 0x09bcu, 1024u);

	for (index = 0; index < PODBAY_SNR_CALLBACK_COUNT; index++)
		store_u32(handle, callback_offsets[index], config->callbacks[index]);

	return PODBAY_SNR_HANDLE_OK;
}
