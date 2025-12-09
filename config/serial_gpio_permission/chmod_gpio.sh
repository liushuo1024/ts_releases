#!/bin/bash
# 参考 sh chmod_gpio.sh
# 授权普通用户运行的嵌入式节点访问gpio
sudo chown root amr_embedded_node 
sudo chmod u+s amr_embedded_node
echo "finsh!"