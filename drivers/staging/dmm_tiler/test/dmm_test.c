#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h> /* platform_device() */
#include <linux/errno.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/interrupt.h>
#include <linux/dma-mapping.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/mm.h>
#include <linux/delay.h>
#include <linux/time.h>
#include <linux/list.h>

#include <mach/dmm.h>
static tiler_handle_t handle[5];

static struct platform_driver dmm_test_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "dmm_test",
	},
};

void do_test(void)
{
	handle[0] = tiler_reserve_2d(TILFMT_8BPP, 1920, 1080, 128);
	handle[1] = tiler_reserve_2d(TILFMT_16BPP, 1920/2, 1080/2, 128);
	handle[2] = tiler_reserve_2d(TILFMT_32BPP, 1920/2, 1080/2, 128);

	tiler_print_allocations();
	if (handle[0])
		tiler_release(handle[0]);
	if (handle[1])
		tiler_release(handle[1]);
	if (handle[2])
		tiler_release(handle[2]);

	handle[0] = tiler_reserve_2d(TILFMT_8BPP, 1920, 1080, 4096);
	handle[1] = tiler_reserve_2d(TILFMT_16BPP, 1920/2, 1080/2, 4096);
	tiler_print_allocations();
	if (handle[0])
		tiler_release(handle[0]);
	if (handle[1])
		tiler_release(handle[1]);
	handle[0] = tiler_reserve_1d(32768);
}

void undo_test(void)
{
}

static int __init dmm_test_init(void)
{
	int ret;

	ret = platform_driver_register(&dmm_test_driver);
	do_test();

	return ret;
}

static void __exit dmm_test_exit(void)
{
	undo_test();
	platform_driver_unregister(&dmm_test_driver);
}

module_init(dmm_test_init);
module_exit(dmm_test_exit);

MODULE_LICENSE("GPL v2");
