ccflags-y		+= -Idrivers/mstar2/include
ccflags-y		+= -Idrivers/mstar2/drv/gpio/
ccflags-y		+= -Idrivers/mstar2/hal/$(CONFIG_MSTAR_CHIP_NAME)/gpio/

obj-$(CONFIG_AMAZON_SIGN_OF_LIFE)       += sign_of_life.o
obj-$(CONFIG_AMAZON_SIGN_OF_LIFE_MSTAR) += sign_of_life_mstar.o
obj-$(CONFIG_AMAZON_METRICS_LOG)        += amazon_logger.o
obj-$(CONFIG_AMAZON_METRICS_LOG_TEST)   += amazon_logger_test.o
obj-$(CONFIG_AMAZON_SILENT_OTA_BACKLIGHT) += silent_ota_backlight_mstar.o
