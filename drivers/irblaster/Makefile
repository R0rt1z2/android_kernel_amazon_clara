#
# Makefile for the irblaster detection drivers
#

ccflags-y		+= -Idrivers/mstar2/include
ccflags-y		+= -Idrivers/mstar2/drv/gpio/
ccflags-y		+= -Idrivers/mstar2/hal/$(CONFIG_MSTAR_CHIP_NAME)/gpio/
obj-$(CONFIG_IRBLASTER_DETECTION)		+= irblaster_detection.o
