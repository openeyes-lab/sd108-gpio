#!/usr/bin/sh

echo "Running script $0 with params $1"

if [ -z "$1" ]; then
	sudo apt update
	sudo apt upgrade -y
	echo "Checking headers installation"
	dpkg-query -s raspberrypi-kernel-headers
	if [ $? -eq 1 ]
	then
		sudo apt install -y raspberrypi-kernel-headers
		echo "After reboot relaunch install"
		sudo reboot
	fi
fi

# Uncomment hdmi_force_hotplug=1
sudo sed -i '/hdmi_force_hotplug=1/s/^#//g' /boot/config.txt

if grep -q hdmi_ignore_edid=0xa5000080 /boot/config.txt ; then
	echo "hdmi present"
else
	echo "" | sudo tee -a /boot/config.txt
	echo "[hdmi]" | sudo tee -a /boot/config.txt
	echo "hdmi_ignore_edid=0xa5000080" | sudo tee -a /boot/config.txt
	echo "hdmi_cvt 800 480 60 1 0 0 0" | sudo tee -a /boot/config.txt
	echo "hdmi_group=2" | sudo tee -a /boot/config.txt
	echo "hdmi_mode=87" | sudo tee -a /boot/config.txt
	echo "hdmi_drive=1" | sudo tee -a /boot/config.txt
	echo "config_hdmi_boost=0" | sudo tee -a /boot/config.txt
	echo "display_hdmi_rotate=2" | sudo tee -a /boot/config.txt
fi

# build overlay dtbo
if dtc -@ -b 0 -I dts -O dtb -o sd108-gpio.dtbo dts/sd108-gpio.dts ; then
	sudo chown root:root sd108-gpio.dtbo
	sudo mv sd108-gpio.dtbo /boot/overlays
else
	echo "fail to compile dts"
	exit
fi

if grep -q "dtoverlay=sd108-gpio" /boot/config.txt ; then
	echo "confi.txt already prepared"
else
	echo "dtoverlay=sd108-gpio" | sudo tee -a /boot/config.txt
fi

sudo cp sd108.rules /etc/udev/rules.d

cd build

make
if [ $? -ne 0 ]; then
    echo "Failed to make"
    echo -1
fi

make install
if [ $? -ne 0 ]; then
    echo "Failed to install"
    echo -1
fi

rm *.o
rm *.mod
rm modules.order
rm Module.symvers
rm *.mod.c
rm *.ko
rm .*.cmd

echo "sd108 correctly installed: reboot to make effective"
echo "remember to use sudo raspi-config to enable i2c interface"
