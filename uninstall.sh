# Comment hdmi_force_hotplug=1 
sudo sed -e '/hdmi_force_hotplug=1/ s/^#*/#/' -i /boot/config.txt

sudo rm /boot/overlays/sd108-gpio.dtbo

sudo sed -i -e "/sd108/d" /boot/config.txt

#sudo rm /lib/modules/$(uname -r)/kernel/drivers/video/backlight/sd108.ko

sudo rm /etc/udev/rules.d/sd108.rules

mod="sd108"
ver="1.0"

sudo dkms remove --force -m $mod -v $ver --all

sudo rm -rf /usr/src/$mod*
