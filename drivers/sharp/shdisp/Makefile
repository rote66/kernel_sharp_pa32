ifneq (, $(filter F%, $(SH_BUILD_ID)))
EXTRA_CFLAGS += -DSHDISP_FACTORY_MODE_ENABLE
endif
obj-$(CONFIG_SHDISP) += shdisp_kerl.o
obj-$(CONFIG_SHDISP) += shdisp_system.o
obj-$(CONFIG_SHDISP) += shdisp_bl71y8_main.o
obj-$(CONFIG_SHDISP) += shdisp_bl71y8.o
obj-$(CONFIG_SHDISP) += shdisp_led.o
obj-$(CONFIG_SHDISP) += shdisp_pm.o
obj-$(CONFIG_SHDISP) += shdisp_panel.o
obj-$(CONFIG_SHDISP) += shdisp_io.o
ifeq ($(CONFIG_SHDISP_PANEL_HAYABUSA),y)
obj-$(CONFIG_SHDISP) += shdisp_hayabusa.o
endif
obj-$(CONFIG_SHDISP) += shdisp_dbg.o
#obj-$(CONFIG_SHDISP) += shlcdc_eventlog.o
EXTRA_CFLAGS    += -DSH_BUILD_ID=\"$(SH_BUILD_ID)\"
EXTRA_CFLAGS    += -DUSE_LINUX
