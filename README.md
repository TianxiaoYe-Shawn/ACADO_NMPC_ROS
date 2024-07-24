[`中文版教程在这`](https://github.com/TianxiaoYe-Shawn/ACADO_NMPC_ROS/blob/main/README.md#%E4%BB%8E%E9%9B%B6%E5%BC%80%E5%A7%8B%E6%90%AD%E5%BB%BA%E5%9F%BA%E4%BA%8Eacado%E7%9A%84%E8%BD%A8%E8%BF%B9%E8%B7%9F%E8%B8%AAros%E5%8A%9F%E8%83%BD%E5%8C%85)

The following is the effect based on the JACKAL gazebo simulator:

https://github.com/user-attachments/assets/46393558-829d-42ab-9687-2b6e7f1e9f6f

# Build ACADO NMPC ROS package from scratch
## 1. Ubuntu and ROS Versions

This ROS package is built on Ubuntu 20.04 and ROS Noetic full desktop version. It is recommended to follow this version, as the JACKAL simulator is also built on it.

## 2. Creating a ROS Workspace

Copy the following code into the terminal to create a workspace:

```
mkdir -p ~/NMPC_ACADO_ws/src/
cd ~/NMPC_ACADO_ws/src/
catkin_init_workspace
cd ~/NMPC_ACADO_ws
catkin_make
echo "source ~/NMPC_ACADO_ws/devel/setup.bash" >> ~/.bashrc
```

You can rename `NMPC_ACADO_ws` to your preferred workspace name.

## 3. Downloading ROS Packages

Navigate to the `/src` directory in your workspace and download the package:

```
cd ~/NMPC_ACADO_ws/src
git clone https://github.com/TianxiaoYe-Shawn/ACADO_NMPC_ROS.git
```


Move the package to the `/src` directory and remove other files:

```
mv ACADO_NMPC_ROS/acado_mpc .
rm -r ACADO_NMPC_ROS
```

## 4. Installing ACADO

Install dependencies:

```
sudo apt-get install gcc g++ cmake git gnuplot doxygen graphviz
```

Choose any working directory (I use `~/`) to download the ACADO source code:

```
cd
git clone https://github.com/acado/acado.git -b stable ACADOtoolkit
```

Install:

```
cd ACADOtoolkit
mkdir build
cd build
cmake ..
make
sudo make install
```

Configure the environment variable:

```
echo "source ~/ACADOtoolkit/build/acado_env.sh" >> ~/.bashrc
```

## 5. Compiling ROS Packages

ACADO's advantage is that it can generate efficient C code through a symbolic language. First, modify your own MPC model in the file `symbolic_mpc.cpp` inside `acado_mpc/acado_export_code` (it's suggested not to modify it on the first run).

Then generate the C code package:

```
cd ~/NMPC_ACADO_ws/src/acado_mpc/acado_export_code
mkdir build && cd build
cmake ..
make
./mpc
```

Move the generated code and build the static library:

```
mv symbolic_mpc_export/* ../../acado_mpc_export/
cd ../../acado_mpc_export
make
```

add `include` folder in your ROS package:
```
cd ~/NMPC_ACADO_ws/src/acado_mpc/
mkdir include
```

Compile the entire ROS package:

```
cd ~/NMPC_ACADO_ws
catkin_make
```

If everything is successful, your package is now fully set up. If you do not have any testing environment, the JACKAL simulator is a good choice next.

## 6. Setting Up JACKAL Simulator

Install the JACKAL simulator with one command:

```
sudo apt-get install ros-noetic-jackal-simulator ros-noetic-jackal-desktop ros-noetic-jackal-navigation
```

Modify the simulator environment, we need a spacious area:

```
cd /opt/ros/noetic/share/jackal_gazebo/launch/
sudo vi jackal_world.launch
```

Press `i` to enter edit mode.

Change the line `<arg name="world_name..."` to `<arg name="world_name" default="$(find gazebo_ros)/launch/empty_world.launch" />`

Then modify the spawn pose of Jackal to match the trajectory in `jackal_world.launch`:

Change the `x` `y` `z` `yaw` value to `0 0 1.0 0.78`

Then press `ESC`, type `:wq` to save and exit.

The JACKAL simulator is now fully set up.

## 7. Start Running!

(For each of the following `rosrun` and `roslaunch`, you need to open a new terminal)

First, open the JACKAL simulator:

```
roslaunch jackal_gazebo jackal_world.launch
```

You should see the JACKAL vehicle parked in Gazebo.

Then open the tracking test environment (including publishing trajectory, configuring rviz):

```
roslaunch acado_mpc tracking_env.launch
```

You should see a green circular trajectory in rviz (you can customize your trajectory in the `trajectory_publisher.cpp` file).

Then open the control input monitor:

```
rosrun acado_mpc plot_control_input.py
```

You should see the monitor displaying messages on two control input topics, which are static since no messages are published yet.

Configure the MPC weight parameters:

```
roslaunch acado_mpc set_weight.launch
```

Then directly `ctrl+c` to cancel, at this point the custom weight parameters have been passed.

Finally, in this terminal, open the MPC control node:

```
rosrun acado_mpc mpc_node
```

At this point, you should see the JACKAL vehicle start moving and following the trajectory.



# 从零开始搭建基于ACADO的轨迹跟踪ROS功能包

## 1.Ubuntu和ROS版本

本ros功能包构建在Ubuntu20.04和ROS Noetic桌面完全版上。建议按照这个版本来，后续的JACKAL模拟器也是在这个版本上构建的。

## 2.创建ROS工作空间

复制以下代码到终端以创建工作空间：

```
mkdir -p ~/NMPC_ACADO_ws/src/
cd ~/NMPC_ACADO_ws/src/
catkin_init_workspace
cd ~/NMPC_ACADO_ws
catkin_make
echo "source ~/NMPC_ACADO_ws/devel/setup.bash" >> ~/.bashrc
```

你可以将`NMPC_ACADO_ws`修改成你定义的工作空间名。

## 3.下载ROS功能包

进入你工作空间下的`/src`目录，下载功能包：

```
cd ~/NMPC_ACADO_ws/src
git clone https://github.com/TianxiaoYe-Shawn/ACADO_NMPC_ROS.git
```

## 4.安装ACADO

安装依赖：

```
sudo apt-get install gcc g++ cmake git gnuplot doxygen graphviz
```

任意选择一个工作目录（我是在`/home/~/`）下载ACADO源码：

```
cd
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

## 5.编译ROS功能包

ACADO的优点就是能够通过符号化的语言生成高效的c代码。首先在`acado_mpc/acado_export_code`中的`symbolic_mpc.cpp`文件修改你自己的mpc模型（第一次跑建议不要动）。

然后生成c代码包：

```
cd ~/NMPC_ACADO_ws/src/acado_mpc/acado_export_code
mkdir build && cd build
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

添加 `include` 文件夹（存放头文件的，目前是空的，你可以把你未来写的头文件放进去）:
```
cd ~/NMPC_ACADO_ws/src/acado_mpc/
mkdir include
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

将`<arg name="world_name..."`那句修改成 `<arg name="world_name" default="$(find gazebo_ros)/launch/empty_world.launch" />`

我们还需要调整小车初始位姿与轨迹开头重合

将 `x` `y` `z` `yaw` 的值修改成 `0 0 1.0 0.78`

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

然后直接`ctrl+c`取消，此时自定义的权重参数已经传入。

紧接着在这个终端里打开mpc控制节点：

```
rosrun acado_mpc mpc_gt_node
```

至此你应该能看到JACKAL小车开始移动并进行轨迹跟踪。
