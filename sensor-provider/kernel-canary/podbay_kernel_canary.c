// SPDX-License-Identifier: GPL-2.0-only
/*
 * Registration-free kernel/toolchain compatibility canary.
 *
 * This module intentionally has no parameters and touches no device, bus,
 * clock, GPIO, filesystem, or vendor interface.
 */
#include <linux/init.h>
#include <linux/module.h>

static int __init podbay_kernel_canary_init(void)
{
	pr_info("podbay kernel canary loaded\n");
	return 0;
}

static void __exit podbay_kernel_canary_exit(void)
{
	pr_info("podbay kernel canary unloaded\n");
}

module_init(podbay_kernel_canary_init);
module_exit(podbay_kernel_canary_exit);

MODULE_DESCRIPTION("Podbay registration-free PW203 kernel compatibility canary");
MODULE_AUTHOR("Podbay clean-room project");
MODULE_LICENSE("GPL");
