# Comment/uncomment the following line to disable/enable debugging
DEBUG ?= y
# Add your debugging flag (or not) to CFLAGS
ifeq ($(DEBUG),y)
DEBFLAGS = -O -g -DCONFIG_MORSE_DEBUG -DDEBUG # "-O" is needed to expand inlines
else
DEBFLAGS = -O2
endif

# Set 0 to a version number. This is done to match the Linux expectations
override MORSE_VERSION = "0-rel_1_12_4_2024_Jun_11-6-g63cd0768"

#ccflags-y += $(DEBFLAGS) -Werror
ccflags-y += "-DMORSE_VERSION=$(MORSE_VERSION)"
ccflags-$(CONFIG_MORSE_RC) += "-DCONFIG_MORSE_RC"
ccflags-$(CONFIG_MORSE_RC) += "-I$(src)/../mmrc"

ifneq ($(CONFIG_BACKPORT_VERSION),)
# Convert a version string from "vX.Y.Z-stuff" into an integer for comparison with KERNEL_VERSION
ccflags-y += "-DMAC80211_BACKPORT_VERSION_CODE=$(shell echo "$(CONFIG_BACKPORT_VERSION)" | \
		awk -F[v.-] '// {printf("%u", lshift($$2, 16) + lshift($$3, 8) + $$4)}')"
endif

obj-$(CONFIG_WLAN_VENDOR_MORSE) += dot11ah.o

dot11ah-y = main.o \
	    debug.o \
	    tx_11n_to_s1g.o \
	    rx_s1g_to_11n.o \
	    ie.o \
	    tim.o \
	    reg.o \
	    s1g_ieee80211.o \
	    s1g_channels.o

SRC := $(shell pwd)

all:
	$(MAKE) MORSE_VERSION=$(MORSE_VERSION) -C $(KERNEL_SRC) M=$(SRC)

modules_install:
	$(MAKE) MORSE_VERSION=$(MORSE_VERSION) -C $(KERNEL_SRC) M=$(SRC) modules_install

clean:
	rm -f *.o *~ core .depend .*.cmd *.ko *.mod.c
	rm -f Module.markers Module.symvers modules.order
	rm -rf .tmp_versions Modules.symvers
