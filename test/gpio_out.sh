#!/bin/bash

CHIP=gpiochip208

if [ -z $1 ]; then
	echo "need a param"
	exit
fi

if [ $1 == "0" ]; then
	port=208
else
	port=209
fi

echo $port > /sys/class/gpio/export
sleep 1
echo out > /sys/class/gpio/gpio$port/direction
sleep 1
echo 0 > /sys/class/gpio/gpio$port/value
sleep 4
echo 1 > /sys/class/gpio/gpio$port/value
sleep 4
echo 0 > /sys/class/gpio/gpio$port/value

sleep 5

echo $port > /sys/class/gpio/unexport
