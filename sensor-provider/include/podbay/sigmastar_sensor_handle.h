/* SPDX-License-Identifier: MIT */
#ifndef PODBAY_SIGMASTAR_SENSOR_HANDLE_H
#define PODBAY_SIGMASTAR_SENSOR_HANDLE_H

#include <stddef.h>
#include <stdint.h>

/* Independently measured PW203 firmware 4.4.2.2 ABI facts. */
#define PODBAY_SNR_HANDLE_SIZE 0x0a44u
#define PODBAY_SNR_MODEL_OFFSET 0x0004u
#define PODBAY_SNR_MODEL_SIZE 36u
#define PODBAY_SNR_PRIVATE_POINTER_OFFSET 0x0028u
#define PODBAY_SNR_FRAMEWORK_API_POINTER_OFFSET 0x003cu
#define PODBAY_SNR_RESOLUTION_COUNT_OFFSET 0x0088u
#define PODBAY_SNR_CURRENT_RESOLUTION_OFFSET 0x008cu
#define PODBAY_SNR_RESOLUTION_OFFSET 0x0090u
#define PODBAY_SNR_RESOLUTION_SIZE 0x0048u
#define PODBAY_SNR_RESOLUTION_NAME_OFFSET 0x0020u
#define PODBAY_SNR_RESOLUTION_NAME_SIZE 40u
#define PODBAY_SNR_MAX_RESOLUTIONS 3u

enum podbay_snr_callback {
	PODBAY_SNR_POWER_ON,
	PODBAY_SNR_POWER_OFF,
	PODBAY_SNR_SENSOR_INIT,
	PODBAY_SNR_RELEASE,
	PODBAY_SNR_SET_PATTERN,
	PODBAY_SNR_GET_SENSOR_ID,
	PODBAY_SNR_GET_RESOLUTION,
	PODBAY_SNR_GET_CURRENT_RESOLUTION,
	PODBAY_SNR_SET_RESOLUTION,
	PODBAY_SNR_GET_ORIENTATION,
	PODBAY_SNR_SET_ORIENTATION,
	PODBAY_SNR_AE_STATUS,
	PODBAY_SNR_GET_EXPOSURE,
	PODBAY_SNR_SET_EXPOSURE,
	PODBAY_SNR_GET_GAIN,
	PODBAY_SNR_SET_GAIN,
	PODBAY_SNR_GET_EXPOSURE_RANGE,
	PODBAY_SNR_GET_GAIN_RANGE,
	PODBAY_SNR_GET_FPS,
	PODBAY_SNR_SET_FPS,
	PODBAY_SNR_GET_SHUTTER_INFO,
	PODBAY_SNR_GET_RESOLUTION_COUNT,
	PODBAY_SNR_CUSTOM_FUNCTION,
	PODBAY_SNR_CALLBACK_COUNT,
};

struct podbay_snr_resolution {
	uint32_t capture_width;
	uint32_t capture_height;
	uint32_t maximum_fps;
	uint32_t pixel_mode;
	uint32_t capture_x;
	uint32_t capture_y;
	uint32_t output_width;
	uint32_t output_height;
	const char *name;
};

struct podbay_snr_handle_config {
	const char *model;
	const struct podbay_snr_resolution *resolutions;
	size_t resolution_count;
	uint32_t callbacks[PODBAY_SNR_CALLBACK_COUNT];
};

enum podbay_snr_handle_result {
	PODBAY_SNR_HANDLE_OK = 0,
	PODBAY_SNR_HANDLE_INVALID_ARGUMENT = -1,
	PODBAY_SNR_HANDLE_WRONG_SIZE = -2,
	PODBAY_SNR_HANDLE_STRING_TOO_LONG = -3,
	PODBAY_SNR_HANDLE_INVALID_RESOLUTIONS = -4,
	PODBAY_SNR_HANDLE_MISSING_CALLBACK = -5,
};

/* Return the measured byte offset of a callback slot, or SIZE_MAX. */
size_t podbay_snr_callback_offset(enum podbay_snr_callback callback);

/*
 * Populate only provider-owned fields of an already cleared framework handle.
 * In particular, this never clears or writes offsets 0x28 or 0x3c.
 */
int podbay_snr_initialize_handle(void *handle, size_t handle_size,
				 const struct podbay_snr_handle_config *config);

#endif
