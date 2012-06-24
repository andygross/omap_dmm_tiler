#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h> /* platform_device() */
#include <linux/errno.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/bitmap.h>
#include <linux/slab.h>

#define SITA_NAME "sita"

#define LUT_SIZE_BITS (256*128)

static unsigned long *lut_bitmap;


static int sita_probe(struct platform_device *dev)
{
	printk(KERN_ERR "sita probe\n");
	return 0;
}

static int sita_remove(struct platform_device *dev)
{
	printk(KERN_ERR "sita remove\n");
	return 0;
}


struct platform_driver sita_driver = {
	.probe = sita_probe,
	.remove = sita_remove,
	.driver = {
		.owner = THIS_MODULE,
		.name = SITA_NAME,
	},
};

/*
   w = width in slots
   h = height in slots
   a = align in slots
   offset = offset in bytes from 4KiB
   pos = position in bitmap for buffer
   page_len = bits per 4KiB
*/
   
int l2r_t2b(uint16_t w, uint16_t h, uint16_t a, int16_t offset,
		unsigned long *pos, unsigned long slots_per_band,
		unsigned long *map, size_t num_bits, size_t stride)
{
	int i;
	unsigned long index;
	unsigned long mask[8];
	bool area_free;
	unsigned long bytes_per_slot = PAGE_SIZE / slots_per_band;
	unsigned long bit_offset = (offset > 0) ? offset / bytes_per_slot : 0;
	unsigned long curr_bit = bit_offset;

	while (curr_bit < num_bits) {
		printk(KERN_ERR "scanning: curr=%ld, offs=%ld\n", curr_bit,
			bit_offset);
		if (bit_offset > 0 )
			*pos = bitmap_find_next_zero_area(map, num_bits,
					curr_bit, w, 1);
		else
			*pos = bitmap_find_next_zero_area(map, num_bits,
					curr_bit, w, a);


		printk(KERN_ERR "found_zero: %ld (%ld,%ld)\n", *pos, (*pos%stride), (*pos / stride));
		if (bit_offset > 0) {
			printk(KERN_ERR "-> %ld == %ld\n", (*pos % slots_per_band), bit_offset);
			if ((*pos % slots_per_band) != bit_offset){
			curr_bit = ALIGN(*pos, slots_per_band) + bit_offset;
			continue;
			}
		}

		/* skip forward to next row if we overlap end of row */
		if ((*pos % stride) + w >= stride) {
			curr_bit = ALIGN(curr_bit, stride) + bit_offset;
			continue;
		}

		bitmap_clear(mask, 0, stride);
		bitmap_set(mask, (*pos % BITS_PER_LONG), w);

		/* assume its true */
		area_free = true;

		/* check subsequent rows to see if complete area is free */
		for (i = 1; i < h; i++) {
			index = *pos / BITS_PER_LONG + i * 8;
			if (bitmap_intersects(&map[index], mask,
				(*pos % BITS_PER_LONG) + w)) {
				area_free = false;
				break;
			}
		}

		if (area_free)
			break;

		/* go forward past this match */
		if (bit_offset > 0)
			curr_bit = ALIGN(*pos, slots_per_band) + bit_offset;
		else
			curr_bit = *pos + 1;
	}

	if (area_free) {
		/* set area as in-use */
		for (i = 0; i < h; i++) {
			index = *pos + i*stride;
			printk(KERN_ERR "setting: %ld\n", index);
			bitmap_set(map, index, w);
		}

		return 0;
	}
	else
		return -ENOMEM;
}

static void dump_lut(void)
{
	int row = 0;
	int col;
	unsigned char *ptr = (unsigned char *)lut_bitmap;
	char *buffer = (char *)kmalloc(256, GFP_KERNEL);

	while (row < 20) {
		for (col = 0; col < 32; col++)
			sprintf(&buffer[col], "%02x", ptr[col]);
		buffer[33] = '\n';
		printk(KERN_ERR "%03d: %s", row, buffer);
		ptr+=32;
		row++;
	}

	kfree(buffer);
}

static int __init sita_init(void)
{
	int bitmap_size = BITS_TO_LONGS(LUT_SIZE_BITS) * sizeof(long);
	unsigned long pos;

	printk(KERN_ERR "sita_init - %d\n", bitmap_size);

	lut_bitmap = (unsigned long *)kmalloc(bitmap_size, GFP_KERNEL);

	bitmap_clear(lut_bitmap, 0, LUT_SIZE_BITS);

	dump_lut();

	l2r_t2b(16, 3, 63, -1, &pos, 64, lut_bitmap, LUT_SIZE_BITS, 256);
	dump_lut();
	l2r_t2b(16, 5, 63, -1, &pos, 64, lut_bitmap, LUT_SIZE_BITS, 256);
	dump_lut();
	l2r_t2b(32, 3, 63, -1, &pos, 64, lut_bitmap, LUT_SIZE_BITS, 256);
	dump_lut();
	l2r_t2b(32, 3, 63, 2048, &pos, 64, lut_bitmap, LUT_SIZE_BITS, 256);
	dump_lut();
	l2r_t2b(16, 3, 63, 1280, &pos, 64, lut_bitmap, LUT_SIZE_BITS, 256);
	dump_lut();
	l2r_t2b(16, 3, 63, 1024, &pos, 64, lut_bitmap, LUT_SIZE_BITS, 256);
	dump_lut();
	l2r_t2b(16, 8, 1, -1, &pos, 64, lut_bitmap, LUT_SIZE_BITS, 256);
	dump_lut();

	return platform_driver_register(&sita_driver);
}

static void __exit sita_exit(void)
{
	printk(KERN_ERR "sita_exit\n");

	kfree(lut_bitmap);
	platform_driver_unregister(&sita_driver);
}

module_init(sita_init);
module_exit(sita_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Andy Gross <andy.gross@ti.com>");
MODULE_DESCRIPTION("OMAP SITA Driver");
MODULE_ALIAS("platform:" SITA_NAME);
