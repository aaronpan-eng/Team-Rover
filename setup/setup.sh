#!/bin/sh
cat rover_sensor.rules >> /etc/udev/rules.d/51-local.rules
cat vn_sensor.rules >> /etc/udev/rules.d/50-VN-100.rules
cat usb_latency.rules >> /etc/udev/rules.d/49-USB-LATENCY.rules
