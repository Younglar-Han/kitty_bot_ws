# 树莓派环境快速配置

## 换源及ROS2安装

```shell
wget http://fishros.com/install -O fishros && . fishros
```



## 系统更新

```shell
sudo apt update 
sudo apt upgrade
```



## 雷达安装及启动

### 安装

#### 官方安装

```shell
cd ~

mkdir -p ldlidar_ros2_ws/src

cd ~/ldlidar_ros2_ws/src

git clone https://gitee.com/ldrobotSensorTeam/ldlidar_sl_ros2.git

ls /dev/ttyUSB*
```

若串口已连接则输入

```shell
sudo chmod 777 /dev/ttyUSB*
```

（若串口名**不为** /dev/ttyUSB0 则输入

```shell
vim ~/ldlidar_ros2_ws/src/ldldiar_sl_ros2/launch/ld14p.launch.py
```

修改port_name后面的参数为串口名）

```shell
cd ~/ldlidar_ros2_ws/src/ldlidar_sl_ros2/ldlidar_driver/src/

vim sl_transform.cpp 

```

在   “ if (angle > 360) {
      angle -= 360;
    }
    if (angle < 0) {
      angle += 360;
    }"前面一行加上 

angle += 180;

退出编辑

之后进行编译

```shell
cd ~/ldlidar_ros2_ws

colcon build
```



#### 修改后的安装（推荐使用，必须修改雷达串口）

```shell
cd ~

mkdir -p ldlidar_ros2_ws/src

cd ~/ldlidar_ros2_ws/src

git clone https://github.com/Younglar-Han/ldlidar_sl_ros2.git #修改过雷达朝向配置及串口名称配置的代码

ls /dev/ttyUSB*
```



### 关于雷达串口的修改

由于程序需要用到雷达和与下位机通信的两个USB转串口设备，因此串口号（/dev/ttyUSB*）不一定是固定的，先挂载的设备将使用ttyUSB0，而使用串口要用到串口号，因此需要一种固定串口号的方法。

具体可见

https://blog.csdn.net/weixin_44037313/article/details/127172796?ops_request_misc=%257B%2522request%255Fid%2522%253A%2522169546828616800188598766%2522%252C%2522scm%2522%253A%252220140713.130102334.pc%255Fall.%2522%257D&request_id=169546828616800188598766&biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~all~first_rank_ecpm_v1~rank_v31_ecpm-4-127172796-null-null.142

这里只介绍硬固定方法

首先只插入一个设备，输入

```shell
ls /dev/ttyUSB*
```

若显示

```shell
/dev/ttyUSB0
```

则正常，只有一个设备

之后输入

```shell
udevadm info --attribute-walk --name=/dev/ttyUSB0 |grep KERNELS
```

会显示内核类型（此设备挂载在电脑USB的硬件位置，这个位置对于每个USB槽位都是固定的

```shell
    KERNELS=="ttyUSB0"
    KERNELS=="1-1.2:1.0" #这句最关键
    KERNELS=="1-1.2"
    KERNELS=="1-1"
    KERNELS=="usb1"
    KERNELS=="0000:01:00.0"
    KERNELS=="0000:00:00.0"
    KERNELS=="pci0000:00"
    KERNELS=="fd500000.pcie"
    KERNELS=="scb"
    KERNELS=="platform"	
```

之后输入

```shell
cd /etc/udev/rules.d 
sudo vim myrobot.rules #新建一个USB设备命名规则（若之前没有）
```

在文件中输入下列语句并保存

```shell
KERNEL=="ttyUSB*",KERNELS=="1-1.2:1.0", MODE:="0777", GROUP:="dialout", SYMLINK+="tty_LIDAR0"
```

其中`KERNELS=="1-1.2:1.0"`为刚才的内核名称，`MODE:="0777"`意为赋予最高权限，`SYMLINK+="tty_LIDAR0"`意为将此设备重命名为 `tty_LIDAR0`

对应的，也需要修改下位机串口转USB的名称

按上述操作即可，最终需要重命名为 `tty_SERIAL0`

若使用**官方安装**，则同时需要修改雷达启动文件中的雷达节点使用的串口号

输入

```shell
cd ~/ldlidar_ros2_ws/src/ldlidar_sl_ros2/launch
vim ld14p.launch.py
```

将 `def generate_launch_description():`下的 `{'port_name': '/dev/ttyUSB0'},`修改为 `{'port_name': '/dev/tty_LIDAR0'},`

### 雷达启动测试

新开一个终端

在树莓派：

```shell
export ROS_DOMAIN_ID=1

cd ~/ldlidar_ros2_ws

source install/setup.bash

ros2 launch ldlidar_sl_ros2 ld14p.launch.py
```

在电脑：

电脑记得在viewer_ld14p.launch.py文件中注释掉启动雷达节点，否则会报错（因为没有连接雷达），但不影响使用

```shell
export ROS_DOMAIN_ID=1

cd ~/ldlidar_ros2_ws

source install/setup.bash

ros2 launch ldlidar_sl_ros2 viewer_ld14p.launch.py
```

# PC端环境配置

## Nav2安装

```shell
sudo apt install ros-humble-navigation2 ros-humble-nav2*
```

## SLAM-toolbox安装

```shell
sudo apt install ros-humble-slam-toolbox
```

# 快速启动

## 在树莓派

### 启动雷达

```shell
export ROS_DOMAIN_ID=1

cd ~/ldlidar_ros2_ws

source install/setup.bash

ros2 launch ldlidar_sl_ros2 ld14p.launch.py
```

### 启动串口

```shell
export ROS_DOMAIN_ID=1
cd ~/serial_ws/
source install/setup.bash
ros2 run st_driver ros2_serial
```

## 在电脑

### 建图

#### 启动SLAM

```shell
ros2 launch slam_toolbox online_async_launch.py
```

#### 如果想在建图时导航

```shell
ros2 launch nav2_bringup navigation_launch.py
```

#### 启动Rviz以及URDF

```shell
export ROS_DOMAIN_ID=1

cd ~/kitty_bot_ws/

source install/setup.bash

ros2 launch kitty_bot_description display.launch.py
```

### 导航

#### 启动导航

```shell
ros2 launch nav2_bringup bringup_launch.py use_sim_time:=False autostart:=False map:=$HOME/maps/map.yaml params_file:=$HOME/nav2_params.yaml
```

#### 启动RVIZ2

```shell
ros2 run rviz2 rviz2 -d $(ros2 pkg prefix nav2_bringup)/share/nav2_bringup/rviz/nav2_default_view.rviz
```

## 参考文档

[Nav2 — Nav2 1.0.0 documentation](https://docs.nav2.org/)

[Nav2 — Navigation 2 1.0.0 文档 (fishros.org)](http://fishros.org/doc/nav2/index.html)

### 大致思路

树莓派与PC通过局域网连接，树莓派只负责发送激光雷达与里程计话题，导航与建图任务由上位机完成。
