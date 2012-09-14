#!/bin/sh

set -e

touch arch/arm/boot/dts/*omap*
touch arch/arm/boot/dts/*twl*

kmake omap5-evm.dtb

kmake uImage modules

kmake uImage-dtb.omap5-evm

sudo mount -L MMCBOOT mnt && sudo cp arch/arm/boot/uImage-dtb.omap5-evm mnt/uImage && sync && sudo umount mnt

