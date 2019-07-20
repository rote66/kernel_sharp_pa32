#Android makefile to build kernel as a part of Android Build
PERL		= perl

RECOVERY_KERNEL_TARGET := $(strip $(INSTALLED_RECOVERY_KERNEL_TARGET))
ifeq ($(RECOVERY_KERNEL_TARGET),)
INSTALLED_RECOVERY_KERNEL_TARGET := $(PRODUCT_OUT)/recovery_kernel
endif

TARGET_KERNEL_ARCH := $(strip $(TARGET_KERNEL_ARCH))
ifeq ($(TARGET_KERNEL_ARCH),)
KERNEL_ARCH := arm
else
KERNEL_ARCH := $(TARGET_KERNEL_ARCH)
endif

TARGET_KERNEL_HEADER_ARCH := $(strip $(TARGET_KERNEL_HEADER_ARCH))
ifeq ($(TARGET_KERNEL_HEADER_ARCH),)
KERNEL_HEADER_ARCH := $(KERNEL_ARCH)
else
$(warning Forcing kernel header generation only for '$(TARGET_KERNEL_HEADER_ARCH)')
KERNEL_HEADER_ARCH := $(TARGET_KERNEL_HEADER_ARCH)
endif

RECOVERY_KERNEL_HEADER_DEFCONFIG := $(strip $(RECOVERY_KERNEL_HEADER_DEFCONFIG))
ifeq ($(RECOVERY_KERNEL_HEADER_DEFCONFIG),)
RECOVERY_KERNEL_HEADER_DEFCONFIG := $(RECOVERY_KERNEL_DEFCONFIG)
endif

# Force 32-bit binder IPC for 64bit kernel with 32bit userspace
ifeq ($(KERNEL_ARCH),arm64)
ifeq ($(TARGET_ARCH),arm)
RECOVERY_KERNEL_CONFIG_OVERRIDE := CONFIG_ANDROID_BINDER_IPC_32BIT=y
endif
endif

TARGET_KERNEL_CROSS_COMPILE_PREFIX := $(strip $(TARGET_KERNEL_CROSS_COMPILE_PREFIX))
ifeq ($(TARGET_KERNEL_CROSS_COMPILE_PREFIX),)
KERNEL_CROSS_COMPILE := arm-eabi-
else
KERNEL_CROSS_COMPILE := $(TARGET_KERNEL_CROSS_COMPILE_PREFIX)
endif

ifeq ($(TARGET_PREBUILT_RECOVERY_KERNEL),)

KERNEL_GCC_NOANDROID_CHK := $(shell (echo "int main() {return 0;}" | $(KERNEL_CROSS_COMPILE)gcc -E -mno-android - > /dev/null 2>&1 ; echo $$?))
ifeq ($(strip $(KERNEL_GCC_NOANDROID_CHK)),0)
RECOVERY_KERNEL_CFLAGS := KCFLAGS=-mno-android
endif

mkfile_path := $(abspath $(lastword $(MAKEFILE_LIST)))
current_dir := $(notdir $(patsubst %/,%,$(dir $(mkfile_path))))
ifeq ($(TARGET_KERNEL_VERSION),)
    TARGET_KERNEL_VERSION := 3.18
endif
TARGET_KERNEL := msm-$(TARGET_KERNEL_VERSION)
ifeq ($(TARGET_KERNEL),$(current_dir))
    # New style, kernel/msm-version
    BUILD_ROOT_LOC := ../../
    TARGET_KERNEL_SOURCE := kernel/$(TARGET_KERNEL)
    RECOVERY_KERNEL_OUT := $(TARGET_OUT_INTERMEDIATES)/recovery_kernel/$(TARGET_KERNEL)
    RECOVERY_KERNEL_SYMLINK := $(TARGET_OUT_INTERMEDIATES)/RECOVERY_KERNEL_OBJ
    KERNEL_USR := $(RECOVERY_KERNEL_SYMLINK)/usr
else
    # Legacy style, kernel source directly under kernel
    KERNEL_LEGACY_DIR := true
    BUILD_ROOT_LOC := ../
    TARGET_KERNEL_SOURCE := kernel
    RECOVERY_KERNEL_OUT := $(TARGET_OUT_INTERMEDIATES)/RECOVERY_KERNEL_OBJ
endif

RECOVERY_KERNEL_CONFIG := $(RECOVERY_KERNEL_OUT)/.config

ifeq ($(RECOVERY_KERNEL_DEFCONFIG)$(wildcard $(RECOVERY_KERNEL_CONFIG)),)
$(error Kernel configuration not defined, cannot build kernel)
else

ifeq ($(TARGET_USES_UNCOMPRESSED_KERNEL),true)
$(info Using uncompressed kernel)
TARGET_PREBUILT_INT_RECOVERY_KERNEL := $(RECOVERY_KERNEL_OUT)/arch/$(KERNEL_ARCH)/boot/Image
else
ifeq ($(KERNEL_ARCH),arm64)
TARGET_PREBUILT_INT_RECOVERY_KERNEL := $(RECOVERY_KERNEL_OUT)/arch/$(KERNEL_ARCH)/boot/Image.gz
else
TARGET_PREBUILT_INT_RECOVERY_KERNEL := $(RECOVERY_KERNEL_OUT)/arch/$(KERNEL_ARCH)/boot/zImage
endif
endif

ifeq ($(TARGET_KERNEL_APPEND_DTB), true)
$(info Using appended DTB)
TARGET_PREBUILT_INT_RECOVERY_KERNEL := $(TARGET_PREBUILT_INT_RECOVERY_KERNEL)-dtb
endif

RECOVERY_KERNEL_HEADERS_INSTALL := $(RECOVERY_KERNEL_OUT)/usr
RECOVERY_KERNEL_MODULES_INSTALL := system
RECOVERY_KERNEL_MODULES_OUT := $(TARGET_OUT)/lib/modules

TARGET_PREBUILT_RECOVERY_KERNEL := $(TARGET_PREBUILT_INT_RECOVERY_KERNEL)

define recovery-mv-modules
mdpath=`find $(RECOVERY_KERNEL_MODULES_OUT) -type f -name modules.dep`;\
if [ "$$mdpath" != "" ];then\
mpath=`dirname $$mdpath`;\
ko=`find $$mpath/kernel -type f -name *.ko`;\
for i in $$ko; do mv $$i $(RECOVERY_KERNEL_MODULES_OUT)/; done;\
fi
endef

define recovery-clean-module-folder
mdpath=`find $(RECOVERY_KERNEL_MODULES_OUT) -type f -name modules.dep`;\
if [ "$$mdpath" != "" ];then\
mpath=`dirname $$mdpath`; rm -rf $$mpath;\
fi
endef

ifneq ($(KERNEL_LEGACY_DIR),true)
$(KERNEL_USR): $(RECOVERY_KERNEL_HEADERS_INSTALL)
	rm -rf $(RECOVERY_KERNEL_SYMLINK)
	ln -s recovery_kernel/$(TARGET_KERNEL) $(RECOVERY_KERNEL_SYMLINK)

$(TARGET_PREBUILT_INT_RECOVERY_KERNEL): $(KERNEL_USR)
endif

$(RECOVERY_KERNEL_OUT):
	mkdir -p $(RECOVERY_KERNEL_OUT)

define RECOVERY_CONFIGY
"CONFIG_ANDROID_RECOVERY_BUILD"
endef
define RECOVERY_CONFIGN
""
endef
define RECOVERY_CONFIGYOP
""
endef
define format_recovery_kernel_config_engineering
	perl -le 'if ($$ENV{'SH_BUILD_DEBUG'} eq "y") {@noappear = split(/ /, ($(RECOVERY_CONFIGY) . " " . $(RECOVERY_CONFIGYOP))); } else {@noappear = split(/ /, $(RECOVERY_CONFIGY));} @config_y = @noappear;\
	@config_n = split(/ /, $(RECOVERY_CONFIGN));\
	while (<>) {chomp($$_); $$line = $$_ ; s/^# // ; s/[ =].+$$// ; if (/^CONFIG/) { $$config = $$_ ; \
	if (grep {$$_ eq $$config} @config_y) { $$line = $$_ . "=y" ; @noappear = grep(!/^$$config$$/, @noappear); } \
	elsif (grep {$$_ eq $$config} @config_n) {$$line = "# " . $$_ . " is not set" ; } } \
	print $$line }\
	foreach (@noappear) { print $$_ . "=y"}' $(1) > $(RECOVERY_KERNEL_OUT)/tmp
	rm $(1)
	cp $(RECOVERY_KERNEL_OUT)/tmp $(1)
	$(MAKE) -C $(TARGET_KERNEL_SOURCE) O=$(BUILD_ROOT_LOC)$(RECOVERY_KERNEL_OUT) ARCH=$(KERNEL_ARCH) CROSS_COMPILE=$(KERNEL_CROSS_COMPILE) oldnoconfig
endef

$(RECOVERY_KERNEL_CONFIG): $(RECOVERY_KERNEL_OUT)
	$(MAKE) -C $(TARGET_KERNEL_SOURCE) O=$(BUILD_ROOT_LOC)$(RECOVERY_KERNEL_OUT) ARCH=$(KERNEL_ARCH) CROSS_COMPILE=$(KERNEL_CROSS_COMPILE) $(RECOVERY_KERNEL_DEFCONFIG)
	$(hide) if [ ! -z "$(RECOVERY_KERNEL_CONFIG_OVERRIDE)" ]; then \
			echo "Overriding recovery kernel config with '$(RECOVERY_KERNEL_CONFIG_OVERRIDE)'"; \
			echo $(RECOVERY_KERNEL_CONFIG_OVERRIDE) >> $(RECOVERY_KERNEL_OUT)/.config; \
			$(MAKE) -C $(TARGET_KERNEL_SOURCE) O=$(BUILD_ROOT_LOC)$(RECOVERY_KERNEL_OUT) ARCH=$(KERNEL_ARCH) CROSS_COMPILE=$(KERNEL_CROSS_COMPILE) oldconfig; fi
	$(call format_recovery_kernel_config_engineering, $(RECOVERY_KERNEL_CONFIG))

$(TARGET_PREBUILT_INT_RECOVERY_KERNEL): $(RECOVERY_KERNEL_OUT) $(RECOVERY_KERNEL_HEADERS_INSTALL)
	$(hide) echo "Building recovery kernel..."
	$(hide) rm -rf $(RECOVERY_KERNEL_OUT)/arch/$(KERNEL_ARCH)/boot/dts
	$(MAKE) -C $(TARGET_KERNEL_SOURCE) O=$(BUILD_ROOT_LOC)$(RECOVERY_KERNEL_OUT) ARCH=$(KERNEL_ARCH) CROSS_COMPILE=$(KERNEL_CROSS_COMPILE) $(RECOVERY_KERNEL_CFLAGS)
#	$(MAKE) -C $(TARGET_KERNEL_SOURCE) O=$(BUILD_ROOT_LOC)$(RECOVERY_KERNEL_OUT) ARCH=$(KERNEL_ARCH) CROSS_COMPILE=$(KERNEL_CROSS_COMPILE) $(RECOVERY_KERNEL_CFLAGS) modules
#	$(MAKE) -C $(TARGET_KERNEL_SOURCE) O=$(BUILD_ROOT_LOC)$(RECOVERY_KERNEL_OUT) INSTALL_MOD_PATH=$(BUILD_ROOT_LOC)../$(RECOVERY_KERNEL_MODULES_INSTALL) INSTALL_MOD_STRIP=1 ARCH=$(KERNEL_ARCH) CROSS_COMPILE=$(KERNEL_CROSS_COMPILE) modules_install
#	$(recovery-mv-modules)
#	$(recovery-clean-module-folder)

$(RECOVERY_KERNEL_HEADERS_INSTALL): $(RECOVERY_KERNEL_OUT)
	$(hide) if [ ! -z "$(RECOVERY_KERNEL_HEADER_DEFCONFIG)" ]; then \
			rm -f $(BUILD_ROOT_LOC)$(RECOVERY_KERNEL_CONFIG); \
			$(MAKE) -C $(TARGET_KERNEL_SOURCE) O=$(BUILD_ROOT_LOC)$(RECOVERY_KERNEL_OUT) ARCH=$(KERNEL_HEADER_ARCH) CROSS_COMPILE=$(KERNEL_CROSS_COMPILE) $(RECOVERY_KERNEL_HEADER_DEFCONFIG); \
			$(MAKE) -C $(TARGET_KERNEL_SOURCE) O=$(BUILD_ROOT_LOC)$(RECOVERY_KERNEL_OUT) ARCH=$(KERNEL_HEADER_ARCH) CROSS_COMPILE=$(KERNEL_CROSS_COMPILE) headers_install; fi
	$(hide) if [ "$(RECOVERY_KERNEL_HEADER_DEFCONFIG)" != "$(RECOVERY_KERNEL_DEFCONFIG)" ]; then \
			echo "Used a different defconfig for header generation"; \
			rm -f $(BUILD_ROOT_LOC)$(RECOVERY_KERNEL_CONFIG); \
			$(MAKE) -C $(TARGET_KERNEL_SOURCE) O=$(BUILD_ROOT_LOC)$(RECOVERY_KERNEL_OUT) ARCH=$(KERNEL_ARCH) CROSS_COMPILE=$(KERNEL_CROSS_COMPILE) $(RECOVERY_KERNEL_DEFCONFIG); fi
	$(hide) if [ ! -z "$(RECOVERY_KERNEL_CONFIG_OVERRIDE)" ]; then \
			echo "Overriding recovery kernel config with '$(RECOVERY_KERNEL_CONFIG_OVERRIDE)'"; \
			echo $(RECOVERY_KERNEL_CONFIG_OVERRIDE) >> $(RECOVERY_KERNEL_OUT)/.config; \
			$(MAKE) -C $(TARGET_KERNEL_SOURCE) O=$(BUILD_ROOT_LOC)$(RECOVERY_KERNEL_OUT) ARCH=$(KERNEL_ARCH) CROSS_COMPILE=$(KERNEL_CROSS_COMPILE) oldconfig; fi
	$(call format_recovery_kernel_config_engineering, $(RECOVERY_KERNEL_CONFIG))

endif
endif
