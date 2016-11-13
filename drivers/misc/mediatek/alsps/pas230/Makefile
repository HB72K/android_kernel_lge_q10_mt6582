include $(srctree)/drivers/misc/mediatek/Makefile.custom

obj-y	:=  pas230.o

ifeq ($(CONFIG_MTK_PAS230_ALS_INT),y)
	EXTRA_CFLAGS += -DPAS230_ALS_SENSOR_ENABLE
	EXTRA_CFLAGS += -DPAS230_ALS_SENSOR_INT
endif