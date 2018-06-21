#!/bin/sh

make CROSS_COMPILE=arm-linux-gnueabi- ARCH=arm uImage -j4
cp arch/arm/boot/uImage ./
 
