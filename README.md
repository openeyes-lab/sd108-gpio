# Linux driver sd108-gpio

[![License: GPL v3](https://img.shields.io/badge/License-GPL%20v3-blue.svg)](http://www.gnu.org/licenses/gpl-3.0)
[![License: LGPL v3](https://img.shields.io/badge/License-LGPL%20v3-blue.svg)](http://www.gnu.org/licenses/lgpl-3.0)

This repository contains Linux drivers for external GPIO PWM and Backlight driver
implemented on OPEN-EYES-II devices from OPEN-EYES S.r.l.

This driver is licensed under the Gnu Public License.

This driver is tested under the Linux 5.X kernels.

For more information about OPEN-EYES-II devices visit http://www.open-eyes.it

The sd108-gpio Linux driver dialog with firmware on ATMEL ATTINY817 MCU mounted
on OPEN-EYES-RPI multifunctions access system based on Rasberry compute module CM3

Main function of sd108 driver is to enable access to:
* PWM filesys driver that control two frontal leds.
* Backlight filesys driver that control LCD backlight intensity and enable.
* GPIO driver that control external wired input/output signal.

## Manual installation

### Build instructions

Prepare system for build:
```
sudo apt update
sudo apt upgrade
sudo apt-get install raspberrypi-kernel-headers git
```
Download from git:
```
git clone http://server.local/git/LinuxDriver/sd108_hdmi.git
```
Compile and install
```
make
make install
```

### Implement device Tree overlay

source file : dts/sd108-gpio.dts

compile dts file and copy into /boot/overlays directory
```
dtc -@ -b 0 -I dts -O dtb -o sd108-gpio.dtbo dts/sd108-gpio.dts
```
change compiled file owner and move it into /boot/overlays directory
```
sudo chown root:root sd108-gpio.dtbo
sudo mv sd108-gpio.dtbo /boot/overlays
```
add this line
```
dtoverlay=sd108-gpio
```
into the file /boot/config.txt

reboot

### udev rules

Setup udev rules
```
sudo cp sd108.rules /etc/udev/rules.d
```

### HDMI config
To correctly setup hdmi controller, add these lines to the /boot/config.txt file
```
[hdmi]
hdmi_ignore_edid=0xa5000080
hdmi_cvt 800 480 60 1 0 0 0
hdmi_group=2
hdmi_mode=87
hdmi_drive=1
config_hdmi_boost=0
display_hdmi_rotate=2
```
and uncomment
```
hdmi_force_hotplug=1
```

## Automatic install/uninstall

After cloning the file;
to install driver execute:
```
cd sd108_gpio
bash install.sh
```
to uninstall execute:
```
cd sd108_gpio
bash uninstall.sh
```

# Interface involved

The firmware on the MCU implements a SLAVE I2C interface and answers to the
address 0x36.

# Filesys

## PWM

The two PWM channels that drive the frontal LEDs are registered under fixed
pwm chip n.12.
```
/sys/class/pwm/pwmchip12
```
in order to not change if other pwm are loaded.

## GPIO

TODO

## BACKLIGHT

The backlight LCD controls are accessible from:
```
/sys/class/backlight/sd108  
```
to control LCD luminosity use
```
echo 0-10 > /sys/class/backlight/sd108/brightness
```

# Reference

[GPIO Descriptor Driver Interface](https://www.kernel.org/doc/html/v4.18/driver-api/gpio/driver.html)

[Pulse Width Modulation (PWM) interface](https://www.kernel.org/doc/html/latest/driver-api/pwm.html)

[Backlight support](https://www.kernel.org/doc/html/latest/gpu/backlight.html)
