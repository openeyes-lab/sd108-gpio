#!/bin/bash

set -e

for i in {0..10..1}
do
	echo $i > /sys/class/backlight/sd108/brightness
	sleep 0.2	
done

echo 5 > /sys/class/backlight/sd108/brightness
