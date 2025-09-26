#!/bin/bash

# echo "remap the device serial port(ttyUSBX) to  imu_gnss"
# echo "imu_gnss usb connection as /dev/imu_gnss , check it using the command : ls -l /dev|grep ttyUSB"
# echo "start copy imu_gnss.rules to  /etc/udev/rules.d/"
# echo "`rospack find gnss_pkg`/script/imu_gnss.rules"
# sudo cp `rospack find gnss_pkg`/script/imu_gnss.rules  /etc/udev/rules.d
# echo " "
# echo "Restarting udev"
# echo ""
# sudo service udev reload
# sudo service udev restart
# sudo udevadm control --reload && sudo udevadm trigger
# echo "finish "



#!/bin/bash
# 显式加载 ROS 环境（根据你的 ROS 版本修改，比如 noetic/melodic）
source /opt/ros/noetic/setup.bash
source /home/komasa/HitchOpen-THICV-Stack/devel/setup.bash
echo "remap the device serial port(ttyUSBX) to  imu_gnss"
echo "imu_gnss usb connection as /dev/imu_gnss , check it using the command : ls -l /dev|grep ttyUSB"
echo "start copy imu_gnss.rules to  /etc/udev/rules.d/"

# 先检查rospack是否可用以及包是否存在
if ! command -v rospack &> /dev/null; then
    echo "错误：未找到rospack命令，请检查ROS环境配置"
    exit 1
fi

PKG_PATH=$(rospack find gnss_pkg)
if [ -z "$PKG_PATH" ]; then
    echo "错误：未找到gnss_pkg包，请确认该包已正确安装"
    exit 1
fi

RULES_FILE="$PKG_PATH/script/imu_gnss.rules"  # 统一使用script（根据实际情况调整）
echo "规则文件路径：$RULES_FILE"

if [ -f "$RULES_FILE" ]; then
    sudo cp "$RULES_FILE" /etc/udev/rules.d/
else
    echo "错误：未找到规则文件 $RULES_FILE"
    exit 1
fi

echo " "
echo "Restarting udev"
echo ""
sudo service udev reload
sudo service udev restart
sudo udevadm control --reload && sudo udevadm trigger
echo "finish "