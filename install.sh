#!/usr/bin/bash

function upgrade_system {
	if [ "$upgrade" = false ]; then
		return
	fi
	sudo apt update
	sudo apt upgrade -y
	upgrade=false
}


if [ "$1" == "noupgrade" ]; then
	echo "Running script $0 without upgrade"
	upgrade=false
else
	echo "Running script $0 with upgrade"
	upgrade=true
fi

echo "Checking headers installation"
dpkg-query -s raspberrypi-kernel-headers
if [ $? -eq 1 ]
then
	upgrade_system
	sudo apt install -y raspberrypi-kernel-headers
	#echo "After reboot relaunch install"
	#sudo reboot
fi

echo "Checking dkms installation"
dpkg-query -s dkms
if [ $? -eq 1 ]
then
	sudo apt install -y dkms
	#echo "After reboot relaunch install"
	#sudo reboot
fi

# Uncomment hdmi_force_hotplug=1
sudo sed -i '/hdmi_force_hotplug=1/s/^#//g' /boot/config.txt

# Uncomment i2c_arm
sudo sed -i '/dtparam=i2c_arm=on/s/^#//g' /boot/config.txt

if grep -q hdmi_ignore_edid=0xa5000080 /boot/config.txt ; then
	echo "hdmi present"
else
	echo "" | sudo tee -a /boot/config.txt
	echo "[hdmi]" | sudo tee -a /boot/config.txt
	echo "hdmi_ignore_edid=0xa5000080" | sudo tee -a /boot/config.txt
	echo "hdmi_cvt 800 480 60 6 0 0 0" | sudo tee -a /boot/config.txt
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

# Install dkms
uname_r=$(uname -r)
src="build"
mod="sd108"
ver="1.0"

if [[ -e /usr/src/$mod-$ver || -e /var/lib/dkms/$mod/$ver ]]; then
  echo "Warning previous install exist...removing!"
  sudo dkms remove --force -m $mod -v $ver --all
  sudo rm -rf /usr/src/$mod-$ver
fi

sudo mkdir -p /usr/src/$mod-$ver
sudo cp -a $src/* /usr/src/$mod-$ver/

sudo dkms add -m $mod -v $ver
sudo dkms build $uname_r -m $mod -v $ver 
sudo dkms install --force $uname_r -m $mod -v $ver

echo "sd108 correctly installed: reboot to make effective"


