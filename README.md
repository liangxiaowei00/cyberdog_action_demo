# cyberdog_action_demo

### 概述

cyberdog_aciton_demo为小米机器人cyberdog开源项目动态手势识别模块(cyberdog_action)使用demo.

开源地址：https://github.com/MiRoboticsLab

### 设计文档

参考文档：https://miroboticslab.github.io/blogs/#/cn/cyberdog_action_cn

### 源码下载

将本项目工程下载到cyberdog_ws下

### 功能介绍
创建一个ros2 client客户端用于请求cyberdog_action模块提供的service，触发一定时间的手势动作识别功能，创建一个ros2 subscription用于接收cyberdog_action发布的的topic,并对识别到的手势进行判断。

### 在机器狗上编译

```shell
#将本地下载的工程拷贝到机器狗上
git clone https://github.com/liangxiaowei00/cyberdog_action_demo.git
scp -r /your_path/cyberdog_action_demo mi@192.168.55.1:/SDCARD/workspace

#进入mi终端,输入密码
ssh mi@192.168.55.1   

#cd到工作空间下
cd /SDCARD/workspace  

#第一次编译某个功能包以及其相关的依赖包需要使用--packages-up-to,编译该功能包及其依赖包

colcon build --merge-install --packages-up-to cyberdog_action_sample

#后续升级单个功能包使用--packages-select，只编译该功能包

colcon build --merge-install --packages-select cyberdog_action_sample

```
####若编译过程中出现/usr/bin/ld: cannot find -lgalaxy-fds-sdk-cpp导致编译错误的问题，请参照 <https://github.com/MiRoboticsLab/cyberdog_ws/issues/24>
### 运行
#### 在终端启动功能包

``` 
source install/setup.bash

#运行cyberdog_action_demo,请求手势识别60s。
ros2 run cyberdog_action_sample cyberdog_action_sample --ros-args -r __ns:=/`ros2 node list | grep "mi_" | head -n 1 | cut -f 2 -d "/"` 


```