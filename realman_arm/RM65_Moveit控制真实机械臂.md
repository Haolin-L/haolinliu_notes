# RM65_Moveit控制真实机械臂

## 启动步骤

```bash
# 终端1: 启动rm_driver（连接机器人）
roslaunch rm_driver rm_65_driver.launch

# 终端2: 启动rm_control（提供轨迹执行Action Server）
roslaunch rm_control rm_65_control.launch

# 终端3: 启动MoveIt（路径规划和执行）
roslaunch rm_65_moveit_config demo_realrobot.launch
```



## 问题概述

- **控制器连接失败**：Moveit无法找到正确的控制器
- **关节状态缺失**:Moveit无法获取当前机器人状态



## 问题1:控制器连接失败

### 报错

```bash
[WARN] Waiting for arm/arm_joint_controller/follow_joint_trajectory to come up...
[ERROR] Action client not connected: arm/arm_joint_controller/follow_joint_trajectory
[INFO] Returned 0 controllers in list
[ERROR] Unable to identify any set of controllers that can actuate the specified joints
```



#### 根本原因:

Moveit配置文件同时启动了真实机械臂和gazebo的控制器

- ###### gazebo控制器:`ros_control`提供

- ###### Action_Server路径:`/arm/arm_joint_controller/follow_joint_trajectory`

- 配置文件:`rm_65_moveit_config/config/controllers_gazebo.yaml`

- ###### 真实机械臂控制器:`rm_control`提供

- ###### Action_Server路径:`/rm_group/follow_joint_trajectory`

- ###### 配置文件:`rm_65_moveit_config/config/controllers.yaml`

#### 问题所在:

- 文件:`rm_65_moveit_config/launch/rm_65_moveit_controller_manager.launch.xml`
- Moveit同时加载了两个配置

```bash
<rosparam file=".../controllers.yaml"/>              <!-- 真实机器人 -->
<rosparam file=".../controllers_gazebo.yaml"/>        <!-- Gazebo -->
```

- 优先启动了gazebo`arm/arm_joint_controller`

- 实际启动的是真实机械臂`rm_group`

  

###  解决方案:

##### 文件1:`rm_moveit_config/rm_65_moveit_config/launch/rm_65_moveit_controller_manager.launch.xml`

###### 原文件:

```bash
<launch>
<arg name="fake_execution_type" default="FollowJointTrajectory" />
<arg name="moveit_controller_manager" 
default="moveit_simple_controller_manager/MoveItSimpleControllerManager" />
<param name="moveit_controller_manager" value="$(arg moveit_controller_manager)"/>
<arg name="use_controller_manager" default="true" />
<param name="use_controller_manager" value="$(arg use_controller_manager)" />

<!-- 同时加载两个配置，导致冲突 -->
<rosparam file="$(find rm_65_moveit_config)/config/controllers.yaml"/>
<rosparam file="$(find rm_65_moveit_config)/config/controllers_gazebo.yaml"/> 
</launch>
```

###### 修改后:

```bash
<launch>
<arg name="fake_execution_type" default="FollowJointTrajectory" />
<arg name="moveit_controller_manager" 
default="moveit_simple_controller_manager/MoveItSimpleControllerManager" />
<param name="moveit_controller_manager" value="$(arg moveit_controller_manager)"/>
<arg name="use_controller_manager" default="true" />
<param name="use_controller_manager" value="$(arg use_controller_manager)" />

<!-- 根据环境选择加载哪个配置文件 -->
<arg name="use_gazebo" default="false" />

<!-- 真实机器人配置 -->
<rosparam unless="$(arg use_gazebo)" 
          file="$(find rm_65_moveit_config)/config/controllers.yaml"/>

<!-- Gazebo配置 -->
<rosparam if="$(arg use_gazebo)" 
          file="$(find rm_65_moveit_config)/config/controllers_gazebo.yaml"/>
</launch>
```



##### 文件2:`rm_moveit_config/rm_65_moveit_config/launch/trajectory_execution.launch.xml`

###### 原文件:

```bash
 <arg name="moveit_controller_manager" default="rm_65" />
  <include file="$(find rm_65_moveit_config)/launch/$(arg moveit_controller_manager)_moveit_controller_manager.launch.xml">
    <arg name="fake_execution_type" value="$(arg fake_execution_type)" />
  </include>
```

###### 修改后:

```bash
  <arg name="moveit_controller_manager" default="rm_65" />
  <arg name="use_gazebo" default="false" />
  <include file="$(find rm_65_moveit_config)/launch/$(arg moveit_controller_manager)_moveit_controller_manager.launch.xml">
    <arg name="fake_execution_type" value="$(arg fake_execution_type)" />
    <arg name="use_gazebo" value="$(arg use_gazebo)" />
  </include>
```



##### 文件3:`rm_moveit_config/rm_65_moveit_config/launch/move_group.launch`

###### 原文件:

```bash
  <include ns="move_group" file="$(find rm_65_moveit_config)/launch/trajectory_execution.launch.xml" if="$(arg allow_trajectory_execution)">
    <arg name="moveit_manage_controllers" value="true" />
    <arg name="moveit_controller_manager" value="rm_65" unless="$(arg fake_execution)"/>
    <arg name="moveit_controller_manager" value="fake" if="$(arg fake_execution)"/>
    <arg name="fake_execution_type" value="$(arg fake_execution_type)" />
  </include>
```

###### 修改后:

```bash
 <arg name="use_gazebo" default="false" />
  <include ns="move_group" file="$(find rm_65_moveit_config)/launch/trajectory_execution.launch.xml" if="$(arg allow_trajectory_execution)">
    <arg name="moveit_manage_controllers" value="true" />
    <arg name="moveit_controller_manager" value="rm_65" unless="$(arg fake_execution)"/>
    <arg name="moveit_controller_manager" value="fake" if="$(arg fake_execution)"/>
    <arg name="fake_execution_type" value="$(arg fake_execution_type)" />
    <arg name="use_gazebo" value="$(arg use_gazebo)" />
  </include>
```



##### 文件4:`rm_moveit_config/rm_65_moveit_config/launch/demo_gazebo.launch`

###### 修改后:

```bash
  <include file="$(find rm_65_moveit_config)/launch/move_group.launch">
    <arg name="allow_trajectory_execution" value="true"/>
    <arg name="fake_execution" value="false"/>
    <arg name="info" value="true"/>
    <arg name="debug" value="$(arg debug)"/>
    <arg name="load_robot_description" value="$(arg load_robot_description)"/>
    <arg name="use_gazebo" value="true"/>  <!-- 添加这一行 -->
  </include>
```



## 问题2:关节状态缺失

### 报错

```bash
[INFO] Didn't receive robot state (joint angles) with recent timestamp within 1 seconds.
[WARN] Failed to fetch current robot state.
[WARN] Failed to validate trajectory: couldn't receive full current joint state within 1s
[INFO] ABORTED: CONTROL_FAILED
```



### 根本原因

`demo_realrobot`没有`joint_state_publisher`,导致`robot_state_publisher`无法获取关节状态,Moveit无法获取当前状态

###### 数据流:

```bash
rm_driver → /joint_states → joint_state_publisher → robot_state_publisher → /tf → MoveIt
```



### 解决方案

文件:`rm_moveit_config/rm_65_moveit_config/launch/demo_realrobot.launch`

###### 原文件:

```bash
  <!-- If needed, broadcast static tf for robot root -->
  <!-- <node name="joint_state_publisher" pkg="joint_state_publisher" type="joint_state_publisher">
    <param name="/use_gui" value="$(arg use_gui)"/>
    <rosparam param="/source_list">[/joint_states]</rosparam>
  </node> -->

  <!-- We do not have a robot connected, so publish fake joint states -->
  <!-- <node name="joint_state_publisher" pkg="joint_state_publisher" type="joint_state_publisher" unless="$(arg use_gui)">
    <rosparam param="source_list">[move_group/fake_controller_joint_states]</rosparam>
  </node>
  <node name="joint_state_publisher" pkg="joint_state_publisher_gui" type="joint_state_publisher_gui" if="$(arg use_gui)">
    <rosparam param="source_list">[move_group/fake_controller_joint_states]</rosparam>
  </node> -->

  <!-- Given the published joint states, publish tf for the robot links -->
  <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher" respawn="true" output="screen" />
```

###### 修改后:

```bash
  <!-- Joint state publisher: subscribes to /joint_states from rm_driver and republishes -->
  <!-- This is needed for robot_state_publisher to generate TF transforms -->
  <node name="joint_state_publisher" pkg="joint_state_publisher" type="joint_state_publisher">
    <param name="/use_gui" value="false"/>
    <rosparam param="/source_list">[/joint_states]</rosparam>
  </node>

  <!-- Given the published joint states, publish tf for the robot links -->
  <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher" respawn="true" output="screen" />
```



## 验证方法

```bash
# 终端1: 启动rm_driver（连接机器人）
roslaunch rm_driver rm_65_driver.launch

# 终端2: 启动rm_control（提供轨迹执行Action Server）
roslaunch rm_control rm_65_control.launch

# 终端3: 启动MoveIt（路径规划和执行）
roslaunch rm_65_moveit_config demo_realrobot.launch
```



- ##### 检查控制器配置

  ```bash
  rosparam get /move_group/controller_list
  ```

  ###### 应该看到:

  ```bash
  - name: rm_group
    action_ns: follow_joint_trajectory
    type: FollowJointTrajectory
    default: true
    joints: [joint1, joint2, joint3, joint4, joint5, joint6]
  ```

  ###### 不应该看到:

  `arm/arm_joint_controller`

- ##### 检查Action Server

  ```bash
  rostopic list | grep rm_group
  ```

  ###### 应该看到:

  ```bash
  /rm_group/follow_joint_trajectory/cancel
  /rm_group/follow_joint_trajectory/feedback
  /rm_group/follow_joint_trajectory/goal
  /rm_group/follow_joint_trajectory/result
  /rm_group/follow_joint_trajectory/status
  ```

- ##### 检查关节状态

  ```bash
  rostopic echo /joint_states
  ```

- ##### 检查TF变换

  ```bash
  rosrun tf tf_echo base_link Link6
  ```

  

  