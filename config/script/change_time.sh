#!/bin/bash

# Change system clock and hardware clock

# 设置系统时钟
echo "请输入当前日期，形式为 月/日/年，例如12/11/2020"
read DATE
echo "请输入当前时刻，形式为 时：分：秒，例如10:10:59"
read CLOCK

echo "**请输入密码**"
read password
echo "**输入密码为$password**"

echo $password | sudo -S date -s $DATE
echo $password | sudo -S date -s $CLOCK
echo $password | sudo -S hwclock -w

echo "
############## ok! ################
"

echo 'Finish !!!'

echo "123" | sudo reboot