#
# Makefile for the netmux drivers.
#

MODULE_NAME = netmux

obj-$(CONFIG_NETMUX_DRIVER) += $(MODULE_NAME).o
ifeq ($(CONFIG_MACH_SHOLEST),y)
$(MODULE_NAME)-objs := debug.o utility.o config.o direct.o network.o tty.o interface.o protocol.o mux.o channelconfig.o register.o main_merge.o
else
$(MODULE_NAME)-objs := debug.o utility.o config.o direct.o network.o tty.o interface.o protocol.o mux.o channelconfig.o register.o main.o
endif


#obj-m   := $(MODULE_NAME).o
#$(MODULE_NAME)-objs := debug.o utility.o config.o direct.o network.o tty.o interface.o protocol.o mux.o channelconfig.o register.o main.o
