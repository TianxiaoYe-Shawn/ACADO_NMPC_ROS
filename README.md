[`中文版教程在这`](https://github.com/TianxiaoYe-Shawn/ACADO_NMPC_ROS/blob/main/README.md#%E4%BB%8E%E9%9B%B6%E5%BC%80%E5%A7%8B%E6%90%AD%E5%BB%BA%E5%9F%BA%E4%BA%8Eacado%E7%9A%84%E8%BD%A8%E8%BF%B9%E8%B7%9F%E8%B8%AAros%E5%8A%9F%E8%83%BD%E5%8C%85)

# Build ACADO NMPC ROS from scratch



# 从零开始搭建基于ACADO的轨迹跟踪ROS功能包

## 1.Ubuntu和ROS版本

本ros包构建在Ubuntu20.04和ROS Noetic版本上。建议按照这个版本来，后续的JACKAL模拟器也是在这个版本上构建的。

## 2.创建ROS工作空间

复制以下代码到终端以创建工作空间：

```
mkdir -p NMPC_ACADO_ws/src/
cd NMPC_ACADO_ws/src/
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
