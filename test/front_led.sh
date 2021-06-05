#!/bin/bash

set -e

CHIP=pwmchip12

echo 0 > /sys/class/pwm/$CHIP/export
sleep 0.2
echo 1000000 > /sys/class/pwm/$CHIP/pwm0/duty_cycle
echo 1 > /sys/class/pwm/$CHIP/pwm0/enable

for i in {1000000..10000000..1000000}
do
	echo $i > /sys/class/pwm/$CHIP/pwm0/duty_cycle
	sleep 0.1	
done

for i in {10000000..1000000..1000000}
do
        echo $i > /sys/class/pwm/$CHIP/pwm0/duty_cycle
        sleep 0.1       
done


sleep 1
echo 0 > /sys/class/pwm/$CHIP/pwm0/enable
echo 0 > /sys/class/pwm/$CHIP/unexport

echo 1 > /sys/class/pwm/$CHIP/export
sleep 0.2
echo 1000000 > /sys/class/pwm/$CHIP/pwm1/duty_cycle
echo 1 > /sys/class/pwm/$CHIP/pwm1/enable

for i in {1000000..10000000..1000000}
do
        echo $i > /sys/class/pwm/$CHIP/pwm1/duty_cycle
        sleep 0.1       
done

for i in {10000000..1000000..1000000}
do
        echo $i > /sys/class/pwm/$CHIP/pwm1/duty_cycle
        sleep 0.1       
done


sleep 1
echo 0 > /sys/class/pwm/$CHIP/pwm1/enable
echo 1 > /sys/class/pwm/$CHIP/unexport

