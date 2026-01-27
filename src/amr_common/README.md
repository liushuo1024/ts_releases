# 节点名:amr_avoid
- 

## 相关硬件

## 依赖关系
### 节点依赖

### 通信依赖
#### 话题:
- 发布:
- 订阅:
        
## 包结构
```cpp
-----avoid
 |--config
 |--include
 |  |- 
 |  └--node
 |     └--
 |--launch
 |--src
 |--CMakeLists.txt
 └--package.xml
```
### 源码说明

## 参数配置
```cpp

## 环境依赖
### jsoncpp库安装
安装步骤如下：
~~~bash
git clone https://github.com/open-source-parsers/jsoncpp/tree/1.8.0   #克隆1.8.0版本jsoncpp
cd jsoncpp-1.8.0
mkdir -p build/debug
cd build/debug
cmake -DCMAKE_BUILD_TYPE=debug -DBUILD_STATIC_LIBS=ON -DBUILD_SHARED_LIBS=OFF -DARCHIVE_INSTALL_DIR=. -G "Unix Makefiles" ../..
make
sudo make install
sudo ldconfig
~~~

