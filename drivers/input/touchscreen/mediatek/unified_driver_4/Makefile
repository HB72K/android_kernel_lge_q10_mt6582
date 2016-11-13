include $(srctree)/drivers/misc/mediatek/Makefile.custom

ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/

obj-y	+=  lgtp_common_driver.o
obj-y	+=  lgtp_model_config_i2c.o
obj-y	+=  lgtp_model_config_misc.o
obj-y	+=  lgtp_platform_api_i2c.o
obj-y	+=  lgtp_platform_api_misc.o
obj-y	+=  lgtp_platform_api_power.o

ifeq ($(CONFIG_TOUCHSCREEN_UNIFIED_DRIVER), y)
obj-y	+= MIT300/
endif