
sd108-gpio-objs := sd108.o

obj-m += sd108-gpio.o

all:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules

clean:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean

install:
	sudo cp sd108-gpio.ko /lib/modules/$(shell uname -r)/kernel/drivers/video/backlight
	sudo depmod -a

load:
	sudo insmod sd108-gpio.ko

reload:
	sudo rmmod sd108-gpio.ko
	sudo insmod sd108-gpio.ko
