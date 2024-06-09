[`中文版教程在这`](https://github.com/TianxiaoYe-Shawn/ACADO_NMPC_ROS/blob/main/README.md#%E4%BB%8E%E9%9B%B6%E5%BC%80%E5%A7%8B%E6%90%AD%E5%BB%BA%E5%9F%BA%E4%BA%8Eacado%E7%9A%84%E8%BD%A8%E8%BF%B9%E8%B7%9F%E8%B8%AAros%E5%8A%9F%E8%83%BD%E5%8C%85)

# Build ACADO NMPC ROS from scratch



# 从零开始搭建基于ACADO的轨迹跟踪ROS功能包

## 1.Ubuntu和ROS版本

本ros包构建在Ubuntu20.04和ROS Noetic桌面完全版上。建议按照这个版本来，后续的JACKAL模拟器也是在这个版本上构建的。

## 2.创建ROS工作空间

复制以下代码到终端以创建工作空间：

```
mkdir -p /home/~/NMPC_ACADO_ws/src/
cd /home/~/NMPC_ACADO_ws/src/
catkin_init_workspace
cd ~/NMPC_ACADO_ws
catkin_make
echo "source ~/NMPC_ACADO_ws/devel/setup.bash" >> ~/.bashrc
```

你可以将`NMPC_ACADO_ws`修改成你定义的工作空间名。

## 3.下载ROS功能包

进入你工作空间下的`/src`目录，下载功能包：

```
cd NMPC_ACADO_ws/src
git clone https://github.com/TianxiaoYe-Shawn/ACADO_NMPC_ROS.git
```

将功能包移动到`/src`目录下并删除其他的文件:

```
mv ACADO_NMPC_ROS/acado_mpc ..
rm -r ACADO_NMPC_ROS
```

## 4.安装ACADO

安装依赖：
```
sudo apt-get install gcc g++ cmake git gnuplot doxygen graphviz
```
任意选择一个工作目录（我是在`/home/~/`）下载ACADO源码：
```
cd /home/~/
git clone https://github.com/acado/acado.git -b stable ACADOtoolkit
```
安装：
```
cd ACADOtoolkit
mkdir build
cd build
cmake ..
make
sudo make install
```
配置环境变量：
```
echo "source ~/ACADOtoolkit/build/acado_env.sh" >> ~/.bashrc
```

## 5.编译功能包
ACADO的优点就是能够通过符号化的语言生成高效的c代码。首先在`acado_mpc/acado_export_code`中的`symbolic_mpc.cpp`文件修改你自己的mpc模型（第一次跑建议不要动）。

然后生成c代码包：
```
cd /home/~/NMPC_ACADO_ws/src/acado_mpc/acado_export_code
/acado_mpc/acado_export_code
cmake ..
make
./mpc
```
移动生成的代码并搭建静态库
```
mv symbolic_mpc_export/* ../../acado_mpc_export/
cd ../../acado_mpc_export
make
```
编译整个ros功能包
```
cd ~/NMPC_ACADO_ws
catkin_make
```
如果一切成功，至此你的功能包就已经搭建完毕了。如果你没有任何的测试环境，接下来这个JACKAL模拟器是不错的选择。

## 6.搭建JACKAL模拟器
一键安装JACKAL小车模拟器
```
sudo apt-get install ros-noetic-jackal-simulator ros-noetic-jackal-desktop ros-noetic-jackal-navigation
```
修改模拟器的环境，我们需要一个空旷的场地
```
cd /opt/ros/noetic/share/jackal_gazebo/launch/
sudo vi jackal_world.launch
```
按`i`进入编辑模式
将`<arg name="world_name..."`那句修改成`<arg name="world_name" default="$(find gazebo_ros)/launch/empty_world.launch" />`
然后按`ESC`输入`:wq`保存并退出。
至此JACKAL模拟器搭建完毕。

## 7.开始运行！
（以下每一个`rosrun`和`roslaunch`都需要你重新打开一个新的终端）

首先打开JACKAL模拟器：
```
roslaunch jackal_gazebo jackal_world.launch
```
你应该能看到JACKAL小车在gazebo里停着。

然后打开轨迹跟踪的测试环境（包括发布轨迹，配置rviz）：
```
roslaunch acado_mpc tracking_env.launch
```
你应该能看到rviz中有绿色圆形轨迹（你可以在`trajectory_publisher.cpp`文件中自定义你的轨迹）。

然后打开控制输入监视器：
```
rosrun acado_mpc plot_control_input.py
```
你应该能看到监视器显示着两个控制输入话题上的消息，此时由于没有消息发布所以是静止的。

配置mpc的权重参数：
```
roslaunch acado_mpc set_weight.launch
```
`ctrl+c`取消，此时自定义的权重参数已经传入。

紧接着在这个终端里打开mpc控制节点：
```
rosrun acado_mpc mpc_node
```

至此你应该能看到JACKAL小车开始移动并进行轨迹跟踪。
