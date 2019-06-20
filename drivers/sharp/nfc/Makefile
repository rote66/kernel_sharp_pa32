#
# Makefile for the Toshiba NFC I2C drivers.
#

## Compile only the target include Toshiba NFC configure ##
obj-$(CONFIG_TOSHIBA_NFC) += snfc_en.o
ifeq ($(CONFIG_TOSHIBA_NFC),y)

# GPIO Driver
obj-$(CONFIG_TOSHIBA_NFC_GPIO) += GPIO/

# I2C Driver
obj-$(CONFIG_TOSHIBA_NFC_I2C)  += I2C/

endif
