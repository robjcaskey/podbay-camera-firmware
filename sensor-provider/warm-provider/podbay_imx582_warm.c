// SPDX-License-Identifier: GPL-2.0-only
/*
 * Source-authored SigmaStar/IMX582 warm-handoff provider.
 *
 * This provider supplies the reviewed PW203 handle metadata and callback
 * lifecycle, but deliberately performs no I2C, GPIO, reset, MCLK, stream, or
 * persistent operation. It is therefore only an ABI/handoff stage: a caller
 * must preserve an already initialized sensor until cold-start sequencing is
 * independently implemented.
 */
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/string.h>
#include <linux/types.h>

#define PODBAY_HANDLE_SIZE 0x0a44u
#define PODBAY_RESOLUTION_COUNT 3u
#define PODBAY_RESOLUTION_BASE 0x0090u
#define PODBAY_RESOLUTION_SIZE 0x0048u

typedef int (*podbay_sensor_init_handle)(void *handle);

extern int DrvRegisterSensorDriverEx(u32 pad,
				     podbay_sensor_init_handle initialize,
				     void *private_data);
extern int DrvRegisterSensorI2CSlaveID(u32 pad, u32 slave_id);
extern int DrvSensorHandleVer(u32 major, u32 minor);
extern int DrvSensorIFVer(u32 major, u32 minor);
extern int DrvSensorI2CVer(u32 major, u32 minor);
extern int DrvSensorRelease(u32 pad);

struct podbay_resolution {
	u32 capture_width;
	u32 capture_height;
	u32 maximum_fps;
	u32 pixel_mode;
	u32 capture_x;
	u32 capture_y;
	u32 output_width;
	u32 output_height;
	const char *name;
};

struct podbay_state {
	u32 exposure_us;
	u32 gain;
	u32 fps;
};

static struct podbay_state state = {
	.exposure_us = 10000u,
	.gain = 1024u,
	.fps = 10u,
};
static bool registered;

static const struct podbay_resolution resolutions[PODBAY_RESOLUTION_COUNT] = {
	{ 8000u, 384u, 10u, 2u, 0u, 0u, 8000u, 384u,
	  "8000x384_RAW10_FINE" },
	{ 1920u, 1080u, 60u, 2u, 0u, 0u, 1920u, 1080u,
	  "1920x1080_RAW10_PREVIEW" },
	{ 2000u, 1500u, 60u, 2u, 0u, 0u, 2000u, 1500u,
	  "2000x1500_RAW10_COARSE" },
};

static void put_u16(void *opaque, u32 offset, u16 value)
{
	u8 *handle = opaque;

	handle[offset] = (u8)value;
	handle[offset + 1u] = (u8)(value >> 8);
}

static void put_u32(void *opaque, u32 offset, u32 value)
{
	u8 *handle = opaque;

	handle[offset] = (u8)value;
	handle[offset + 1u] = (u8)(value >> 8);
	handle[offset + 2u] = (u8)(value >> 16);
	handle[offset + 3u] = (u8)(value >> 24);
}

static u32 get_u32(const void *opaque, u32 offset)
{
	const u8 *handle = opaque;

	return (u32)handle[offset] |
	       (u32)handle[offset + 1u] << 8 |
	       (u32)handle[offset + 2u] << 16 |
	       (u32)handle[offset + 3u] << 24;
}

static u32 callback_address(const void *callback)
{
	return (u32)(unsigned long)callback;
}

static int podbay_noop(void *handle)
{
	(void)handle;
	return 0;
}

static int podbay_noop_value(void *handle, u32 value)
{
	(void)handle;
	(void)value;
	return 0;
}

static int podbay_get_sensor_id(void *handle, u32 *sensor_id)
{
	(void)handle;
	if (!sensor_id)
		return -EINVAL;
	*sensor_id = 0x0582u;
	return 0;
}

static int podbay_get_resolution_count(void *handle, u32 *count)
{
	(void)handle;
	if (!count)
		return -EINVAL;
	*count = PODBAY_RESOLUTION_COUNT;
	return 0;
}

static int podbay_get_resolution(void *handle, u32 index, void **resolution)
{
	if (!handle || !resolution || index >= PODBAY_RESOLUTION_COUNT)
		return -EINVAL;
	*resolution = (u8 *)handle + PODBAY_RESOLUTION_BASE +
		index * PODBAY_RESOLUTION_SIZE;
	return 0;
}

static int podbay_get_current_resolution(void *handle, u32 *index,
					 void **resolution)
{
	u32 resolution_index;

	if (!handle || !index || !resolution)
		return -EINVAL;
	resolution_index = get_u32(handle, 0x008cu);
	if (resolution_index >= PODBAY_RESOLUTION_COUNT)
		return -ERANGE;
	*index = resolution_index;
	*resolution = (u8 *)handle + PODBAY_RESOLUTION_BASE +
		resolution_index * PODBAY_RESOLUTION_SIZE;
	return 0;
}

static int podbay_set_resolution(void *handle, u32 index)
{
	if (!handle || index >= PODBAY_RESOLUTION_COUNT)
		return -EINVAL;
	put_u32(handle, 0x008cu, index);
	state.fps = index == 0u ? 10u : 60u;
	return 0;
}

static int podbay_get_orientation(void *handle, u32 *orientation)
{
	if (!handle || !orientation)
		return -EINVAL;
	*orientation = get_u32(handle, 0x0080u);
	return 0;
}

static int podbay_set_orientation(void *handle, u32 orientation)
{
	if (!handle || orientation > 3u)
		return -EINVAL;
	put_u32(handle, 0x0080u, orientation);
	return 0;
}

static int podbay_get_exposure(void *handle, u32 *exposure_us)
{
	(void)handle;
	if (!exposure_us)
		return -EINVAL;
	*exposure_us = state.exposure_us;
	return 0;
}

static int podbay_set_exposure(void *handle, u32 exposure_us)
{
	(void)handle;
	state.exposure_us = exposure_us;
	return 0;
}

static int podbay_get_gain(void *handle, u32 *gain)
{
	(void)handle;
	if (!gain)
		return -EINVAL;
	*gain = state.gain;
	return 0;
}

static int podbay_set_gain(void *handle, u32 gain)
{
	(void)handle;
	if (gain < 1024u || gain > 1048576u)
		return -ERANGE;
	state.gain = gain;
	return 0;
}

static int podbay_get_exposure_range(void *handle, u32 *minimum,
				     u32 *maximum)
{
	(void)handle;
	if (!minimum || !maximum)
		return -EINVAL;
	*minimum = 1u;
	*maximum = 1000000u;
	return 0;
}

static int podbay_get_gain_range(void *handle, u32 *minimum, u32 *maximum)
{
	(void)handle;
	if (!minimum || !maximum)
		return -EINVAL;
	*minimum = 1024u;
	*maximum = 1048576u;
	return 0;
}

static int podbay_get_fps(void *handle)
{
	(void)handle;
	return (int)state.fps;
}

static int podbay_set_fps(void *handle, u32 fps)
{
	u32 resolution_index;

	if (!handle || fps == 0u)
		return -EINVAL;
	resolution_index = get_u32(handle, 0x008cu);
	if (resolution_index >= PODBAY_RESOLUTION_COUNT ||
	    fps > resolutions[resolution_index].maximum_fps)
		return -ERANGE;
	state.fps = fps;
	return 0;
}

static int podbay_get_shutter_info(void *handle, u32 *info)
{
	(void)handle;
	if (!info)
		return -EINVAL;
	info[0] = 0u;
	info[1] = 1u;
	info[2] = 1u;
	info[3] = 1u;
	return 0;
}

static int podbay_custom_function(void *handle, u32 command, void *argument)
{
	(void)handle;
	(void)command;
	(void)argument;
	return -EOPNOTSUPP;
}

static void put_callback(void *handle, u32 offset, const void *callback)
{
	put_u32(handle, offset, callback_address(callback));
}

static void put_resolution(void *handle, u32 index,
			   const struct podbay_resolution *resolution)
{
	u32 base = PODBAY_RESOLUTION_BASE + index * PODBAY_RESOLUTION_SIZE;

	memset((u8 *)handle + base, 0, PODBAY_RESOLUTION_SIZE);
	put_u32(handle, base + 0x00u, resolution->capture_width);
	put_u32(handle, base + 0x04u, resolution->capture_height);
	put_u32(handle, base + 0x08u, resolution->maximum_fps);
	put_u32(handle, base + 0x0cu, resolution->pixel_mode);
	put_u32(handle, base + 0x10u, resolution->capture_x);
	put_u32(handle, base + 0x14u, resolution->capture_y);
	put_u32(handle, base + 0x18u, resolution->output_width);
	put_u32(handle, base + 0x1cu, resolution->output_height);
	strscpy((u8 *)handle + base + 0x20u, resolution->name, 40u);
}

static int podbay_initialize_handle(void *handle)
{
	u32 index;

	if (!handle)
		return -EINVAL;

	/* Do not clear the framework's private pointer at 0x28 or API at 0x3c. */
	memset((u8 *)handle + 0x0004u, 0, 36u);
	strscpy((u8 *)handle + 0x0004u, "IMX582_MIPI", 36u);
	put_u32(handle, 0x002cu, 1u);
	put_u32(handle, 0x0030u, 1u);
	put_u32(handle, 0x0034u, 300000u);
	put_u16(handle, 0x0038u, 32u);
	put_u32(handle, 0x0044u, 2u);
	put_u32(handle, 0x0048u, 2u);
	put_u32(handle, 0x004cu, 1u);
	put_u32(handle, 0x0050u, 1u);
	put_u32(handle, 0x0054u, 1u);
	put_u32(handle, 0x0058u, 1u);
	put_u32(handle, 0x0068u, 1u);
	put_u32(handle, 0x006cu, 0u);
	put_u32(handle, 0x0070u, 0u);
	put_u32(handle, 0x0074u, 1u);
	put_u32(handle, 0x0078u, 1u);
	put_u32(handle, 0x007cu, 4u);
	put_u32(handle, 0x0080u, 0u);
	put_u32(handle, 0x0088u, PODBAY_RESOLUTION_COUNT);
	put_u32(handle, 0x008cu, 0u);
	for (index = 0; index < PODBAY_RESOLUTION_COUNT; index++)
		put_resolution(handle, index, &resolutions[index]);

	put_u32(handle, 0x0948u, 4u);
	put_u32(handle, 0x094cu, 1u);
	put_u32(handle, 0x09b8u, 10u);
	put_u32(handle, 0x09bcu, 1024u);

	put_callback(handle, 0x09c4u, podbay_noop);
	put_callback(handle, 0x09c8u, podbay_noop);
	put_callback(handle, 0x09ccu, podbay_noop);
	put_callback(handle, 0x09d4u, podbay_noop);
	put_callback(handle, 0x09d8u, podbay_noop_value);
	put_callback(handle, 0x09dcu, podbay_get_sensor_id);
	put_callback(handle, 0x09e0u, podbay_get_resolution);
	put_callback(handle, 0x09e4u, podbay_get_current_resolution);
	put_callback(handle, 0x09e8u, podbay_set_resolution);
	put_callback(handle, 0x09ecu, podbay_get_orientation);
	put_callback(handle, 0x09f0u, podbay_set_orientation);
	put_callback(handle, 0x09f8u, podbay_noop_value);
	put_callback(handle, 0x09fcu, podbay_get_exposure);
	put_callback(handle, 0x0a00u, podbay_set_exposure);
	put_callback(handle, 0x0a04u, podbay_get_gain);
	put_callback(handle, 0x0a08u, podbay_set_gain);
	put_callback(handle, 0x0a0cu, podbay_get_exposure_range);
	put_callback(handle, 0x0a10u, podbay_get_gain_range);
	put_callback(handle, 0x0a14u, podbay_get_fps);
	put_callback(handle, 0x0a18u, podbay_set_fps);
	put_callback(handle, 0x0a24u, podbay_get_shutter_info);
	put_callback(handle, 0x0a28u, podbay_get_resolution_count);
	put_callback(handle, 0x0a2cu, podbay_custom_function);

	pr_info("podbay IMX582 warm provider initialized 0x%x-byte ABI handle\n",
		PODBAY_HANDLE_SIZE);
	return 0;
}

static int __init podbay_warm_init(void)
{
	int result;

	if (DrvSensorHandleVer(2, 2) < 0 ||
	    DrvSensorIFVer(2, 1) < 0 ||
	    DrvSensorI2CVer(1, 1) < 0)
		return -EPROTO;
	result = DrvRegisterSensorDriverEx(0, podbay_initialize_handle, &state);
	if (result < 0)
		return result;
	registered = true;
	result = DrvRegisterSensorI2CSlaveID(0, 0u);
	if (result < 0) {
		DrvSensorRelease(0);
		registered = false;
		return result;
	}
	pr_info("podbay IMX582 warm provider registered without hardware access\n");
	return 0;
}

static void __exit podbay_warm_exit(void)
{
	if (registered) {
		DrvSensorRelease(0);
		registered = false;
	}
	pr_info("podbay IMX582 warm provider released\n");
}

module_init(podbay_warm_init);
module_exit(podbay_warm_exit);

MODULE_DESCRIPTION("Podbay source-authored IMX582 warm-handoff provider");
MODULE_AUTHOR("Podbay clean-room project");
MODULE_LICENSE("GPL");
