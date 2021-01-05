# Comment hdmi_force_hotplug=1 
sudo sed -e '/hdmi_force_hotplug=1/ s/^#*/#/' -i /boot/config.txt

exit 0

sudo rm /boot/overlays/sd108.dtbo

sudo sed -i -e "/sd108/d" /boot/config.txt

sudo rm /lib/modules/$(uname -r)/kernel/drivers/video/backlight/sd108.ko

sudo rm /etc/udev/rules.d/sd108.rules
