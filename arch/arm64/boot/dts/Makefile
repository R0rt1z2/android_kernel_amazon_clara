dtb-$(CONFIG_ARCH_VEXPRESS) += $(subst ",,$(CONFIG_MSTAR_CHIP_NAME)_ramdisk).dtb
ifneq ($(TARGET_PRODUCT),)
dtb-$(CONFIG_ARCH_VEXPRESS) += $(subst ",,$(CONFIG_MSTAR_CHIP_NAME)_an_$(TARGET_PRODUCT)).dtb
else
dtb-$(CONFIG_ARCH_VEXPRESS) += $(subst ",,$(CONFIG_MSTAR_CHIP_NAME)_an).dtb
endif
ifeq ($(CONFIG_MSTAR_CHIP_NAME), "m7322")
dtb-$(CONFIG_OF) += $(subst ",,$(CONFIG_MSTAR_CHIP_NAME)_an_avb).dtb
dtb-$(CONFIG_OF) += $(subst ",,$(CONFIG_MSTAR_CHIP_NAME)_an).main_dtb
dtb-$(CONFIG_OF) += $(subst ",,$(CONFIG_MSTAR_CHIP_NAME)_overlay_1).dtbo
dtb-$(CONFIG_OF) += $(subst ",,$(CONFIG_MSTAR_CHIP_NAME)_overlay_avb).dtbo
dtb-$(CONFIG_OF) += $(subst ",,$(CONFIG_MSTAR_CHIP_NAME)_overlay_dto).dtbo
dtb-$(CONFIG_OF) += $(subst ",,$(CONFIG_MSTAR_CHIP_NAME)_overlay_28).dtbo
dtb-$(CONFIG_OF) += $(subst ",,$(CONFIG_MSTAR_CHIP_NAME)_overlay_avb28).dtbo
endif

ifeq ($(CONFIG_MSTAR_CHIP_NAME), "m5621")
dtb-$(CONFIG_OF) += $(subst ",,$(CONFIG_MSTAR_CHIP_NAME)_an_avb).dtb
dtb-$(CONFIG_OF) += $(subst ",,$(CONFIG_MSTAR_CHIP_NAME)_an).main_dtb
dtb-$(CONFIG_OF) += $(subst ",,$(CONFIG_MSTAR_CHIP_NAME)_overlay_1).dtbo
dtb-$(CONFIG_OF) += $(subst ",,$(CONFIG_MSTAR_CHIP_NAME)_overlay_avb).dtbo
dtb-$(CONFIG_OF) += $(subst ",,$(CONFIG_MSTAR_CHIP_NAME)_overlay_dto).dtbo
dtb-$(CONFIG_OF) += $(subst ",,$(CONFIG_MSTAR_CHIP_NAME)_overlay_28).dtbo
dtb-$(CONFIG_OF) += $(subst ",,$(CONFIG_MSTAR_CHIP_NAME)_overlay_avb28).dtbo
endif

# TODO: Add the M7332 DTS build rule
# ifeq ($(CONFIG_MSTAR_CHIP_NAME), "m7332")
# endif

targets += dtbs

DTB_NAMES := $(subst $\",,$(CONFIG_BUILD_ARM64_APPENDED_DTB_IMAGE_NAMES))
ifneq ($(DTB_NAMES),)
DTB_LIST := $(addsuffix .dtb,$(DTB_NAMES))
else
DTB_LIST := $(dtb-y)
endif
targets += $(DTB_LIST)

dtbs: $(addprefix $(obj)/, $(DTB_LIST))

clean-files := dts/*.dtb *.dtb dts/*.dtbo *.dtbo dts/*.main_dtb *.main_dtb
