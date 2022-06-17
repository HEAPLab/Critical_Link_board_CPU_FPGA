DRV_NAME		= msgdma

KERNEL_SRC		=/home/matteo/linux-socfpga
ARCH			=arm
CROSS_COMPILE	=/opt/gcc-arm-linux-gnueabihf/bin/arm-none-linux-gnueabihf-

obj-m			+= $(DRV_NAME).o

all:
	make ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE) -C $(KERNEL_SRC) M=$(PWD) modules

install: all
	sudo cp $(DRV_NAME).ko /srv/nfs/de1_rootfs/home/root 

clean:
	make -C $(KERNEL_SRC) M=$(PWD) clean
