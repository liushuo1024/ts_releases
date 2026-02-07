#!/bin/bash

# CAN接口配置脚本
# 自动检测并使用第一个可用的ttyACM设备

echo "=== CAN接口配置脚本 ==="

# 1. 杀死之前的slcand进程
echo "正在清理旧的slcand进程..."
sudo pkill slcand 2>/dev/null
sudo pkill -f "slcand" 2>/dev/null

# 等待进程完全结束
sleep 1

# 2. 自动检测可用的ttyACM设备
echo "检测可用的CAN设备..."

# 查找所有ttyACM设备
ACM_DEVICES=($(ls /dev/ttyACM* 2>/dev/null))

if [ ${#ACM_DEVICES[@]} -eq 0 ]; then
    echo "错误: 找不到任何ttyACM设备"
    echo "请检查:"
    echo "  1. CANable设备是否插入USB"
    echo "  2. 是否有权限访问设备"
    echo "  3. 运行: lsusb | grep -i can"
    exit 1
fi

# 使用第一个找到的设备
TARGET_DEVICE="${ACM_DEVICES[0]}"
echo "找到设备: $TARGET_DEVICE"
echo "可用设备列表:"
printf "  %s\n" "${ACM_DEVICES[@]}"

# 如果找到多个设备，让用户选择
if [ ${#ACM_DEVICES[@]} -gt 1 ]; then
    echo ""
    echo "找到多个CAN设备:"
    for i in "${!ACM_DEVICES[@]}"; do
        echo "  [$i] ${ACM_DEVICES[$i]}"
    done
    
    read -p "请选择设备编号 (默认 0): " CHOICE
    CHOICE=${CHOICE:-0}
    
    if [[ $CHOICE =~ ^[0-9]+$ ]] && [ $CHOICE -lt ${#ACM_DEVICES[@]} ]; then
        TARGET_DEVICE="${ACM_DEVICES[$CHOICE]}"
        echo "已选择: $TARGET_DEVICE"
    else
        echo "无效选择，使用第一个设备: $TARGET_DEVICE"
    fi
fi

# 3. 检查设备是否可访问
echo "检查设备权限..."
if [ ! -r "$TARGET_DEVICE" ]; then
    echo "警告: 无读取权限，尝试修复..."
    sudo chmod 666 "$TARGET_DEVICE" 2>/dev/null
fi

# 4. 启动slcand
echo "启动slcand服务..."
echo "命令: sudo slcand -o -s5 -t hw -S 3000000 $TARGET_DEVICE slcan0"
sudo slcand -o -s5 -t hw -S 3000000 "$TARGET_DEVICE" slcan0

# 等待接口初始化
echo "等待接口初始化..."
for i in {1..5}; do
    echo -n "."
    sleep 1
done
echo ""

# 5. 检查接口是否创建成功
echo "检查slcan0接口..."
if ! ip link show slcan0 >/dev/null 2>&1; then
    echo "错误: slcan0接口未创建成功"
    echo "可能的原因:"
    echo "  1. 设备固件问题"
    echo "  2. 设备繁忙"
    echo "  3. 缺少slcan内核模块"
    echo ""
    echo "尝试手动检查:"
    echo "  dmesg | tail -20"
    exit 1
fi

# 6. 重命名接口
echo "重命名接口为can0..."
sudo ip link set dev slcan0 name can0 2>/dev/null

# 7. 启动接口
echo "启动can0接口..."
sudo ip link set can0 up type can bitrate 250000 2>/dev/null || sudo ip link set up can0

# 8. 显示配置信息
echo ""
echo "=== CAN接口配置完成 ==="
echo "设备: $TARGET_DEVICE"
echo "接口状态:"
ip -details link show can0 2>/dev/null || echo "  无法显示接口状态"
echo ""


echo ""
echo "=== 开始监听CAN流量 ==="
echo "按 Ctrl+C 停止监听"
echo ""

# 10. 开始监听
candump can0 -x
