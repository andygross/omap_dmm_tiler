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
	pos		position in bitmap
	w		width in slots
	h		height in slots
	map		ptr to bitmap
	stride		slots in a row
*/
void free_slots(unsigned long pos, uint16_t w, uint16_t h, unsigned long *map,
		uint16_t stride)
{
	int i;

	for (i = 0; i < h; i++, pos += stride)
		bitmap_clear(map, pos, w);
}

/*
	w		width in slots
	pos		ptr to position
	map		ptr to bitmap
	num_bits	number of bits in bitmap
*/
int r2l_b2t_1d(uint16_t w, unsigned long *pos, unsigned long *map,
		size_t num_bits)
{
	unsigned long search_count = 0;
	unsigned long bit;
	bool area_found = false;

	*pos = num_bits - w;

	while (search_count < num_bits) {
		bit = find_next_bit(map, num_bits, *pos);

		if (bit - *pos >= w){
			/* found a long enough free area */
			bitmap_set(map, *pos, w);
			area_found = true;
			break;
		}

		search_count = num_bits - bit + w;
		*pos = bit - w;
	}

	printk(KERN_ERR "Found: %ld (%ld,%ld)-(%ld,%ld)\n", *pos,
		(*pos % 256), (*pos / 256), ((*pos + w - 1) %256),
		 ((*pos + w - 1) /256));
	return (area_found) ? 0 : -ENOMEM;
}

/*
   input   w = width in slots
   input   h = height in slots
   input   a = align in slots	(mask, 2^n-1, 0 is unaligned)
   input   offset = offset in bytes from 4KiB
   output  pos = position in bitmap for buffer
   input   map = bitmap ptr
   input   num_bits = size of bitmap
   input   stride = bits in one row of container
*/
   
int l2r_t2b(uint16_t w, uint16_t h, uint16_t a, int16_t offset,
		unsigned long *pos, unsigned long slots_per_band,
		unsigned long *map, size_t num_bits, size_t stride)
{
	int i;
	unsigned long index;
	unsigned long mask[8];	/* fits 1 256 slot row */
	bool area_free;
	unsigned long bytes_per_slot = PAGE_SIZE / slots_per_band;
	unsigned long bit_offset = (offset > 0) ? offset / bytes_per_slot : 0;
	unsigned long curr_bit = bit_offset;

	/* reset alignement to 1 if we are matching a specific offset */
	a = (offset > 0) ? 1 : a;

	while (curr_bit < num_bits) {
		*pos = bitmap_find_next_zero_area(map, num_bits, curr_bit, w,
				a);

		/* skip forward if we are not at right offset */
		if (bit_offset > 0 && (*pos % slots_per_band != bit_offset)) {
			curr_bit = ALIGN(*pos, slots_per_band) + bit_offset;
			continue;
		}

		/* skip forward to next row if we overlap end of row */
		if ((*pos % stride) + w >= stride) {
			curr_bit = ALIGN(curr_bit, stride) + bit_offset;
			continue;
		}

		/* generate mask that represents out matching pattern */
		bitmap_clear(mask, 0, stride);
		bitmap_set(mask, (*pos % BITS_PER_LONG), w);

		/* assume the area is free until we find an overlap */
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
			curr_bit = *pos + a;
	}

	if (area_free) {
		printk(KERN_ERR "Found area: (%ld,%ld)-(%ld,%ld)\n",
			(*pos%stride), (*pos/stride), (*pos%stride + w - 1),
			(*pos/stride + h - 1));
		/* set area as in-use. iterate over rows */
		for (i = 0, index = *pos; i < h; i++, index += stride)
			bitmap_set(map, index, w);
	}
	
	return (area_free) ? 0 : -ENOMEM;
}

static void dump_lut(unsigned int start, unsigned int range)
{
	int row = start, i;
	unsigned char *buffer = kmalloc(1024, GFP_KERNEL);
	unsigned char *map = (unsigned char *)lut_bitmap;
	unsigned char val;
	int len;

	while(row < range){
		len = 0;
		for (i=0; i < 32; i++){
			val = *(map + i + row*32);

			/* transpose bits */
			val = ((val & 0x80) >> 7) | ((val & 0x40) >> 5) |
				((val & 0x20) >> 3) | ((val & 0x10) >> 1) |
				((val & 0x1) << 7) | ((val & 0x2) << 5) |
				((val & 0x4) << 3) | ((val & 0x8) << 1);
			len += scnprintf(buffer + len, 1024 - i, "%02x", val);
		}

		printk(KERN_ERR "%03d: %s", row, buffer);
		row++;
	}

	printk(KERN_ERR "\n");
	kfree(buffer);
}

static int __init sita_init(void)
{
	int bitmap_size = BITS_TO_LONGS(LUT_SIZE_BITS) * sizeof(long);
	unsigned long pos[8];

	printk(KERN_ERR "sita_init - %d\n", bitmap_size);

	lut_bitmap = (unsigned long *)kmalloc(bitmap_size, GFP_KERNEL);

	bitmap_clear(lut_bitmap, 0, LUT_SIZE_BITS);

	dump_lut(0,128);

	/* 2D tests */
	l2r_t2b(16, 3, 63, -1, &pos[0], 64, lut_bitmap, LUT_SIZE_BITS, 256);
	dump_lut(0, 10);
	l2r_t2b(16, 5, 63, -1, &pos[1], 64, lut_bitmap, LUT_SIZE_BITS, 256);
	dump_lut(0, 10);
	l2r_t2b(32, 3, 63, -1, &pos[2], 64, lut_bitmap, LUT_SIZE_BITS, 256);
	dump_lut(0, 10);
	l2r_t2b(32, 3, 63, 2048, &pos[3], 64, lut_bitmap, LUT_SIZE_BITS, 256);
	dump_lut(0, 10);
	l2r_t2b(16, 3, 63, 1280, &pos[4], 64, lut_bitmap, LUT_SIZE_BITS, 256);
	dump_lut(0, 10);
	l2r_t2b(16, 3, 63, 1024, &pos[5], 64, lut_bitmap, LUT_SIZE_BITS, 256);
	dump_lut(0, 10);
	l2r_t2b(7, 8, 1, -1, &pos[6], 64, lut_bitmap, LUT_SIZE_BITS, 256);
	dump_lut(0, 10);
	l2r_t2b(7, 8, 0, -1, &pos[7], 64, lut_bitmap, LUT_SIZE_BITS, 256);
	dump_lut(0, 10);
	free_slots(pos[6], 7, 8, lut_bitmap, 256);
	dump_lut(0, 10);
	free_slots(pos[0], 16, 3, lut_bitmap, 256);
	free_slots(pos[1], 16, 5, lut_bitmap, 256);
	free_slots(pos[2], 32, 3, lut_bitmap, 256);
	free_slots(pos[3], 32, 3, lut_bitmap, 256);
	free_slots(pos[4], 16, 3, lut_bitmap, 256);
	free_slots(pos[5], 16, 3, lut_bitmap, 256);
	free_slots(pos[7], 7, 8, lut_bitmap, 256);
	dump_lut(0, 10);

	/* 1D tests */
	r2l_b2t_1d(8, &pos[0], lut_bitmap, LUT_SIZE_BITS);
	dump_lut(120,128);
	r2l_b2t_1d(100, &pos[1], lut_bitmap, LUT_SIZE_BITS);
	dump_lut(120,128);
	r2l_b2t_1d(1, &pos[2], lut_bitmap, LUT_SIZE_BITS);
	dump_lut(120,128);
	free_slots(pos[1], 100, 1,lut_bitmap, 256);
	dump_lut(120,128);
	r2l_b2t_1d(30, &pos[3], lut_bitmap, LUT_SIZE_BITS);
	dump_lut(120,128);
	r2l_b2t_1d(90, &pos[1], lut_bitmap, LUT_SIZE_BITS);
	dump_lut(120,128);


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
