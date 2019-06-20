#
# Makefile for sensorHub drivers.
#

obj-$(CONFIG_SHUB_ML630Q790)		+= ml630q790.o shub-batching.o shub-input_mcu.o shub-input_exif.o shub-input_common.o
obj-$(CONFIG_HOSTIF_SPI)			+= shub-spi.o
obj-$(CONFIG_HOSTIF_I2C)			+= shub-i2c.o
obj-$(CONFIG_ACC_SENSOR)			+= shub-input_acc.o
obj-$(CONFIG_GYRO_SENSOR)			+= shub-input_gyro.o
obj-$(CONFIG_MAG_SENSOR)			+= shub-input_mag.o
obj-$(CONFIG_ORI_SENSOR)			+= shub-input_orientation.o
obj-$(CONFIG_LINEACC_SENSOR)		+= shub-input_linear_acc.o
obj-$(CONFIG_BARO_SENSOR)			+= shub-input_baro.o
obj-$(CONFIG_GAMEROT_SENSOR)		+= shub-input_game_rotation_vec.o
obj-$(CONFIG_GRAV_SENSOR)	    	+= shub-input_grav.o
obj-$(CONFIG_GYROUNC_SENSOR)		+= shub-input_gyro_uncal.o
obj-$(CONFIG_MAGROT_SENSOR)	    	+= shub-input_mag_rotation_vec.o
obj-$(CONFIG_MAGUNC_SENSOR)	    	+= shub-input_mag_uncalib.o
obj-$(CONFIG_ROTVEC_SENSOR)	    	+= shub-input_rotation_vec.o
obj-$(CONFIG_PEDO_SENSOR)	    	+= shub-input_pedo.o
obj-$(CONFIG_PEDODEC_SENSOR)		+= shub-input_pedodect.o
obj-$(CONFIG_SIGNIFICANT_MOTION)	+= shub-input_significant.o
obj-y								+= shub-input_diag.o
EXTRA_CFLAGS += -Wall
