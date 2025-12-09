#!/bin/bash
# 参考 sh chmod_ttyS.sh
echo "current user permission:"
ls -l /dev/ttyS*
# 授权普通用户访问ttyS设备
echo "please input username:"
read USERNAME
sleep 2
sudo usermod -a -G dialout $USERNAME
echo "finsh!"