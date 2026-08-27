/* SPDX-License-Identifier: MIT */
#include "podbay/sigmastar_sensor_handle.h"

#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static uint32_t load_u32(const uint8_t *bytes, size_t offset)
{
	return (uint32_t)bytes[offset] |
	       (uint32_t)bytes[offset + 1u] << 8 |
	       (uint32_t)bytes[offset + 2u] << 16 |
	       (uint32_t)bytes[offset + 3u] << 24;
}

static struct podbay_snr_handle_config valid_config(void)
{
	static const struct podbay_snr_resolution resolutions[] = {
		{8000, 384, 10, 2, 0, 0, 8000, 384, "8000x384_RAW10_FINE"},
		{1920, 1080, 60, 2, 0, 0, 1920, 1080, "1920x1080_RAW10_PREVIEW"},
		{2000, 1500, 60, 2, 0, 0, 2000, 1500, "2000x1500_RAW10_COARSE"},
	};
	struct podbay_snr_handle_config config = {
		.model = "IMX582_MIPI",
		.resolutions = resolutions,
		.resolution_count = sizeof(resolutions) / sizeof(resolutions[0]),
	};
	size_t index;

	for (index = 0; index < PODBAY_SNR_CALLBACK_COUNT; index++)
		config.callbacks[index] = 0x10000001u + (uint32_t)index * 4u;
	return config;
}

static void test_layout(void)
{
	uint8_t guarded[PODBAY_SNR_HANDLE_SIZE + 2u];
	struct podbay_snr_handle_config config = valid_config();
	uint8_t *handle = &guarded[1];
	size_t index;

	memset(guarded, 0xa5, sizeof(guarded));
	assert(podbay_snr_initialize_handle(handle, PODBAY_SNR_HANDLE_SIZE,
					    &config) == PODBAY_SNR_HANDLE_OK);
	assert(guarded[0] == 0xa5 && guarded[sizeof(guarded) - 1u] == 0xa5);
	assert(memcmp(handle + PODBAY_SNR_MODEL_OFFSET, "IMX582_MIPI", 12u) == 0);
	assert(load_u32(handle, PODBAY_SNR_RESOLUTION_COUNT_OFFSET) == 3u);
	assert(load_u32(handle, PODBAY_SNR_CURRENT_RESOLUTION_OFFSET) == 0u);
	assert(load_u32(handle, PODBAY_SNR_RESOLUTION_OFFSET) == 8000u);
	assert(load_u32(handle, PODBAY_SNR_RESOLUTION_OFFSET + 0x04u) == 384u);
	assert(load_u32(handle, PODBAY_SNR_RESOLUTION_OFFSET +
			PODBAY_SNR_RESOLUTION_SIZE + 0x18u) == 1920u);
	assert(load_u32(handle, PODBAY_SNR_RESOLUTION_OFFSET +
			2u * PODBAY_SNR_RESOLUTION_SIZE + 0x1cu) == 1500u);
	assert(strcmp((const char *)(handle + PODBAY_SNR_RESOLUTION_OFFSET +
			PODBAY_SNR_RESOLUTION_NAME_OFFSET),
		      "8000x384_RAW10_FINE") == 0);

	/* Framework-owned values survive provider initialization. */
	assert(load_u32(handle, PODBAY_SNR_PRIVATE_POINTER_OFFSET) == 0xa5a5a5a5u);
	assert(load_u32(handle, PODBAY_SNR_FRAMEWORK_API_POINTER_OFFSET) ==
	       0xa5a5a5a5u);
	for (index = 0; index < PODBAY_SNR_CALLBACK_COUNT; index++) {
		size_t offset = podbay_snr_callback_offset(
			(enum podbay_snr_callback)index);
		assert(offset < PODBAY_SNR_HANDLE_SIZE - 3u);
		assert(load_u32(handle, offset) == config.callbacks[index]);
	}
	assert(podbay_snr_callback_offset(PODBAY_SNR_CALLBACK_COUNT) == SIZE_MAX);
}

static void test_refusals(void)
{
	uint8_t handle[PODBAY_SNR_HANDLE_SIZE];
	struct podbay_snr_handle_config config = valid_config();
	char long_model[PODBAY_SNR_MODEL_SIZE + 1u];

	memset(handle, 0, sizeof(handle));
	assert(podbay_snr_initialize_handle(handle, sizeof(handle) - 1u, &config) ==
	       PODBAY_SNR_HANDLE_WRONG_SIZE);
	config.callbacks[PODBAY_SNR_SET_FPS] = 0u;
	assert(podbay_snr_initialize_handle(handle, sizeof(handle), &config) ==
	       PODBAY_SNR_HANDLE_MISSING_CALLBACK);
	config = valid_config();
	config.resolution_count = PODBAY_SNR_MAX_RESOLUTIONS + 1u;
	assert(podbay_snr_initialize_handle(handle, sizeof(handle), &config) ==
	       PODBAY_SNR_HANDLE_INVALID_RESOLUTIONS);
	config = valid_config();
	memset(long_model, 'x', sizeof(long_model));
	long_model[sizeof(long_model) - 1u] = '\0';
	config.model = long_model;
	assert(podbay_snr_initialize_handle(handle, sizeof(handle), &config) ==
	       PODBAY_SNR_HANDLE_STRING_TOO_LONG);
}

int main(void)
{
	test_layout();
	test_refusals();
	puts("SigmaStar handle tests passed");
	return 0;
}
