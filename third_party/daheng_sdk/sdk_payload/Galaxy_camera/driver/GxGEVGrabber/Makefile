obj-m := GxGEVGrabber.o
GxGEVGrabber-objs := driver.o card_core.o ../common/wrapper.o

ABS_PATH := $(abspath $(dir $(lastword $(MAKEFILE_LIST))))

# 添加额外编译参数
ccflags-y += -Wno-declaration-after-statement -Wno-discarded-qualifiers -Wno-missing-prototypes -Wno-missing-declarations
# ccflags-y += -g

KDIR := /lib/modules/$(shell uname -r)/build
PWD := $(shell pwd)

all: card_core.data
	@cp -f card_core.data card_core.o
	@touch .card_core.o.cmd
	$(MAKE) -C $(KDIR) M=$(PWD) modules

clean:
	@rm -f ../common/*.o ../common/.*.cmd
	$(MAKE) -C $(KDIR) M=$(PWD) clean

rebuild: clean all
