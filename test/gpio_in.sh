#!/bin/bash

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
echo in > /sys/class/gpio/gpio$port/direction
sleep 1
cat /sys/class/gpio/gpio$port/value
sleep 1
cat /sys/class/gpio/gpio$port/value
sleep 1
cat /sys/class/gpio/gpio$port/value

sleep 2

echo $port > /sys/class/gpio/unexport
