# cyberdog_ai_sports_demo

### 概述

cyberdog_ai_sports_demo为小米机器人cyberdog开源项目，运动计数识别模块使用demo.

开源地址：https://github.com/MiRoboticsLab

### 设计文档

参考文档：https://miroboticslab.github.io/blogs/#/cn/cyberdog_ai_sports_cn

### 源码下载

将本项目工程下载到cyberdog_ws下

### 功能介绍

#### 运动计数

```
//ros msg、srv接口
#include "protocol/srv/sport_manager.hpp"
#include "protocol/msg/sport_counts_result.hpp"
```

### 在机器狗上编译

```shell
#将本地下载的工程拷贝到机器狗上
scp -r /your_path/cyberdog_ai_sports_demo mi@192.168.55.1:/SDCARD/workspace

#进入mi终端,输入密码
ssh mi@192.168.55.1   

#cd到工作空间下
cd /SDCARD/workspace  

#source环境变量
source /opt/ros2/galactic/setup.bash    

#第一次编译某个功能包需要使用--packages-up-to,编译该功能包及其依赖包

colcon build --merge-install --packages-up-to cyberdog_ai_sports_demo

#后续升级单个功能包使用--packages-select，只编译该功能包

colcon build --merge-install --packages-select cyberdog_ai_sports_demo
```

### 运行
#### 在终端启动功能包

``` 
source /opt/ros2/cyberdog/setup.bash

#运行cyberdog_ai_sports_demo,启动深蹲计数，设置10s内识别5个深蹲
ros2 run cyberdog_ai_sports_demo cyberdog_ai_sports_demo --ros-args -r __ns:=/`ros2 node list | grep "mi_" | head -n 1 | cut -f 2 -d "/"

```