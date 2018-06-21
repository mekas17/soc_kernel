ifeq ($(CONFIG_ARCH_LX28XX),y)

zreladdr-y	:= 0x80008000
params_phys-y	:= 0x80000100
initrd_phys-y	:= 0x80800000
endif
