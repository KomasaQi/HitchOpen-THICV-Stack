#!/bin/bash

echo "remap the device serial port(ttyUSBX) to  imu_rear"
echo "imu_rear usb connection as /dev/imu_rear , check it using the command : ls -l /dev|grep ttyUSB"
echo "start copy imu_rear.rules to  /etc/udev/rules.d/"
echo "`rospack find gnss_pkg`/scripts/imu_rear.rules"
sudo cp `rospack find gnss_pkg`/script/imu_rear.rules  /etc/udev/rules.d
echo " "
echo "Restarting udev"
echo ""
sudo service udev reload
sudo service udev restart
sudo udevadm control --reload && sudo udevadm trigger
echo "finish "
