# PlotJuggler

## 系统要求

- #### ros1-noetic

- #### Qt5

- #### ubuntu20.04

  

## 步骤

### 前置条件

```bash
#更新
sudo apt-get update
#创建 catkin 工作空间
mkdir -p ~/tools_ws/src
cd ~/tools_ws/src
catkin_init_workspace
cd ~/tools_ws
#Qt5及相关库
sudo apt-get install -y \
    qtbase5-dev \
    qtdeclarative5-dev \
    qtmultimedia5-dev \
    libqt5svg5-dev \
    libqt5multimedia5-plugins \
    qttools5-dev \
    qttools5-dev-tools
#ros相关依赖
sudo apt-get install -y \
    ros-noetic-rosbag-storage \
    ros-noetic-roscpp \
    ros-noetic-roscpp-serialization \
    ros-noetic-tf \
    ros-noetic-tf2-ros \
    ros-noetic-diagnostic-msgs \
    ros-noetic-nav-msgs \
    ros-noetic-sensor-msgs \
    ros-noetic-geometry-msgs \
    ros-noetic-tf2-msgs
#其他依赖
sudo apt-get install -y \
    libboost-regex-dev \
    libdw-dev \
    binutils-dev \
    cmake \
    build-essential
```



```bash
#安装plotjuggler_msgs(必需)
cd ~/tools_ws/src
git clone https://github.com/facontidavide/plotjuggler_msgs.git
```



>## 提示
>
>plotjuggler v3.7+版本移除了DataStreamROS插件(可以实时订阅ROS话题)
>
>要使用这个功能,必须使用v3.6之前的版本

### 下载源码

```bash
cd ~/tools_ws/src

# 克隆 PlotJuggler 仓库
git clone https://github.com/facontidavide/PlotJuggler.git

# 切换到包含 DataStreamROS 的版本（移除 ROS 之前的最后一个版本）
cd PlotJuggler
git checkout 0d19c30b^  # 切换到移除 ROS 之前的版本

# 创建分支保存这个版本
git checkout -b v3.6-with-ros-streaming
```



### 编译

#### plotjuggler_msgs

```bash
# 使用 catkin_make_isolated 编译
catkin_make_isolated --install --install-space install -DCMAKE_BUILD_TYPE=Release --pkg plotjuggler_msgs

# Source 安装目录
source install/setup.bash
```

#### plotjuggler

```bash
# 编译 PlotJuggler（包含所有 ROS 插件）
catkin_make_isolated --install --install-space install -DCMAKE_BUILD_TYPE=Release

# 编译过程可能需要 10-30 分钟，取决于你的机器性能
```

#### 验证编译结果

```bash
# 检查 DataStreamROS 插件是否编译成功
ls -lh install/lib/plotjuggler/libDataStreamROS.so

# 检查 DataLoadROS 插件（用于加载 .bag 文件）
ls -lh install/lib/plotjuggler/libDataLoadROS.so

# 检查可执行文件
ls -lh install/bin/plotjuggler
ls -lh install/lib/plotjuggler/PlotJuggler
```

### 配置环境

```bash
# 添加到 ~/.bashrc
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
echo "source ~/tools_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```



## 使用方法

### 实时订阅ROS话题并可视化

```bash
#测试
roscore
cd ~/tools_ws
source /opt/ros/noetic/setup.bash
source install/setup.bash
plotjuggler
#另起终端
rostopic pub /test std_msgs/Float64 "data: 3.14" -r 10
```

#### 操作

- 找到`Streaming` -> `双击start` -> `选中/test` -> `点击OK` 
- 左侧栏找到`TimeSeries List` -> ` 找到/test` - > `展开test,找到data` -> `拖拽data到右侧` -> `看到数据可视化`

### 回放rosbag并可视化

以下暂未实现

### 多变量对比,同时可视化

### 数据缩放/平移/同步

### 导出数据:CSV,PNG,JSON

### 自定义数学表达式

## 参考

https://github.com/facontidavide/PlotJuggler

https://github.com/facontidavide/plotjuggler_msgs

http://wiki.ros.org/noetic

https://www.plotjuggler.io/