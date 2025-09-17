#!/bin/bash

echo "remap the device serial port(ttyUSBX) to  imu_gnss"
echo "imu_gnss usb connection as /dev/imu_gnss , check it using the command : ls -l /dev|grep ttyUSB"
echo "start copy imu_gnss.rules to  /etc/udev/rules.d/"
echo "`rospack find gnss_pkg`/scripts/imu_gnss.rules"
sudo cp `rospack find gnss_pkg`/script/imu_gnss.rules  /etc/udev/rules.d
echo " "
echo "Restarting udev"
echo ""
sudo service udev reload
sudo service udev restart
sudo udevadm control --reload && sudo udevadm trigger
echo "finish "
