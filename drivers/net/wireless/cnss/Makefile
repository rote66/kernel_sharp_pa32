# Makefile for CNSS platform driver

# [WLAN][SHARP] 2016.03.24 Add for SH Customize Start
ccflags-y += -DSH_WIFI_CUSTOMIZE
# [WLAN][SHARP] 2016.03.24 Add for SH Customize End

obj-$(CONFIG_CNSS_PCI) += cnss_pci.o
obj-$(CONFIG_CNSS_SDIO)	+= cnss_sdio.o
obj-$(CONFIG_CNSS)	+= cnss_common.o
