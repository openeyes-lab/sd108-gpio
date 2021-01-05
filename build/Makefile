obj-m += sd108.o

all:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules

clean:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean

install:
	sudo cp sd108.ko /lib/modules/$(shell uname -r)/kernel/drivers/video/backlight
	sudo depmod -a

load:
	sudo insmod sd108.ko

reload:
	sudo rmmod sd108.ko
	sudo insmod sd108.ko
