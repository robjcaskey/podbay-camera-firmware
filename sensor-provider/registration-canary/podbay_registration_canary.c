// SPDX-License-Identifier: GPL-2.0-only
/*
 * SigmaStar registration-lifecycle canary.
 *
 * This module registers a deliberately unavailable sensor provider and then
 * releases it on unload. Its callback never dereferences the vendor handle and
 * it performs no I2C, GPIO, clock, reset, stream, or persistent operation.
 */
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/types.h>

typedef int (*podbay_sensor_init_handle)(void *handle);

extern int DrvRegisterSensorDriverEx(u32 pad,
				     podbay_sensor_init_handle initialize,
				     void *private_data);
extern int DrvRegisterSensorI2CSlaveID(u32 pad, u32 slave_id);
extern int DrvSensorHandleVer(u32 major, u32 minor);
extern int DrvSensorIFVer(u32 major, u32 minor);
extern int DrvSensorI2CVer(u32 major, u32 minor);
extern int DrvSensorRelease(u32 pad);

static u8 private_data[132] __aligned(4);
static bool registered;

static int podbay_unavailable_sensor(void *handle)
{
	/* Do not inspect or modify the framework-owned opaque handle. */
	(void)handle;
	pr_info("podbay registration canary callback refused activation\n");
	return -ENODEV;
}

static int __init podbay_registration_canary_init(void)
{
	int result;

	if (DrvSensorHandleVer(2, 2) < 0 ||
	    DrvSensorIFVer(2, 1) < 0 ||
	    DrvSensorI2CVer(1, 1) < 0)
		return -EPROTO;

	result = DrvRegisterSensorDriverEx(0, podbay_unavailable_sensor,
					   private_data);
	if (result < 0)
		return result;
	registered = true;

	result = DrvRegisterSensorI2CSlaveID(0, 0);
	if (result < 0) {
		DrvSensorRelease(0);
		registered = false;
		return result;
	}

	pr_info("podbay registration canary registered without sensor access\n");
	return 0;
}

static void __exit podbay_registration_canary_exit(void)
{
	if (registered) {
		DrvSensorRelease(0);
		registered = false;
	}
	pr_info("podbay registration canary released\n");
}

module_init(podbay_registration_canary_init);
module_exit(podbay_registration_canary_exit);

MODULE_DESCRIPTION("Podbay SigmaStar registration-lifecycle canary");
MODULE_AUTHOR("Podbay clean-room project");
MODULE_LICENSE("GPL");
