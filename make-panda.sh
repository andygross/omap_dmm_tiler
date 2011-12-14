#!/bin/sh

set -e

touch arch/arm/boot/dts/*omap*
touch arch/arm/boot/dts/*twl*

kmake omap4-panda.dtb

kmake uImage

kmake uImage-dtb.omap4-panda

