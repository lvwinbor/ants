<!--
 * @Author: Chenyang Zhang && 1727326672@qq.com
 * @Date: 2024-04-07 09:18:38
 * @LastEditors: Chenyang Zhang && 1727326672@qq.com
 * @LastEditTime: 2024-04-07 13:56:33
 * @FilePath: /bit_zcy_001/src/bag_reacar/vehicle_rea/readme.md
 * @Description: 
 * 
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved. 
-->
# 启动文件说明
* 注意：是各个模块的分别启动

<!-- 第一部分 -->
## 0、仿真环境与实车配置
### a、仿真环境配置
仿真环境目前支持五车，启动`bag_simulators`文件夹下的`vehicle_simulator`包的`launch`文件夹下的`Simulator_car_env_（$n）car.launch`系列启动文件。

启动之后，仿真器输出消息： 
- 激光雷达消息`/ant01/registered_scan`   
<!-- $\quad\quad\quad\quad\quad\quad\quad\quad\quad\quad\quad$  -->
- 定位消息`/ant01/state_estimation`以及`/ant01/pose_estimation`

启动之后，仿真器接受消息： 
- 速度指令消息`/ant01/cmd_vel`

注意，默认排序为X方向一致，Y方向从0开始，每隔一米排布一辆。

### b、实车配置
### ① 定位建图模块配置
使用fast_lio作为定位方法。
### ② 底盘驱动模块配置
- 松灵小车：修改官方ROS驱动，使之兼容上文提到的ROS消息即可。
- ant：

<!-- 第二部分 -->
## 1、编队相关功能
编队功能目前为领航与跟随方法，启动`bag_reacar`文件夹下的`vehicle_rea`包的`launch`文件夹下子文件夹`1_car_formation`的`ant00($n)_formation.launch`系列启动文件。
### 1.1. 注意
目前测试最多五车，且需要启动时指定领航者为ant01，具备队形变化功能，预制三种队形：纵队、斜三角、等边三角。
### 1.2. 关于参数配置：
1. 编队间距：`<arg name="headway" default="3"/>`
2. 编队变化计算基准 `<arg name="formation_length_unit" default="1.5"/>`
3. 避障的参数设置：注意避障距离应该小于跟随间距。
### 1.3. 示例：五车编队启动
1. 启动仿真器：`roslaunch vehicle_simulator Simulator_car_env_5car.launch`，注意，等待仿真器完全启动之后再继续操作；
2. 启动每个车功能节点：`roslaunch vehicle_rea ants_00（1-5）_formation.launch`，注意，没啥注意的，参数合适即可；
3. 使用rviz的插件waypoint1向头车发送目标点即可；
4. 队形变化点击rviz左下方插件按钮即可。

<!-- 第三部分 -->
## 2、单机探索相关功能
单机探索功能目前为点云直接生成探索点方法，启动`bag_reacar`文件夹下的`vehicle_rea`包的`launch`文件夹下子文件夹`2_exploration`的`self_exp_ant00($n).launch`系列启动文件。
### 2.1. 注意
配置第一辆车作为探索者。
### 2.2. 参数配置
1. 注意名称空间`<arg name="namespace" default="ant01"/>`和`<arg name="car_id" value="0"/>`要对应，后者为前者减一。
2. 注意`<arg name="is_coordinate" default="false"/>`要修改为false，关闭协同模式。
3. 关于探索范围设置，在`bag_ants`文件夹下的`ants_explorer_unknown >> frontier_space`包的`launch`文件夹下的`frontier_finder.launch`启动文件。
```
  <!-- 超出此范围不再探索 -->
  <arg name="map_size_x" value="400"/>  
  <arg name="map_size_y" value="400"/>
  <arg name="map_size_z" value="3.0"/>
  <arg name="map_origin_x" default="-200"/>
  <arg name="map_origin_y" default="-200"/>
  <arg name="map_origin_z" default="0.0"/>
```
### 2.3. 示例
1. 启动仿真器：`roslaunch vehicle_simulator Simulator_car_env_1car.launch`，注意，等待仿真器完全启动之后再继续操作；
2. 启动每个车功能节点：`roslaunch vehicle_rea self_exp_ant001.launch`，注意，没啥注意的，参数合适即可；
3. 发布触发信号，格式为`geometry_msgs/PoseStamped`，名称为`/move_base_simple/goal`。
4. 等待探索结束即可。

<!-- 第四部分 -->
## 3、多机协同探索相关功能
多机探索功能目前为单机探索的基础上，加入H网格进行任务分配，启动`bag_reacar`文件夹下的`vehicle_rea`包的`launch`文件夹下子文件夹`2_exploration`的`ant00($n)_exp.launch`系列启动文件。
### 3.1. 注意
性能还待优化。
### 3.2. 参数配置
1. 注意配置H网格生成范围，使之与地图待探索范围相匹配，在`bag_ants`文件夹下的`ants_task_allocation >> task_allocation`包的`launch`文件夹下的`single_util_z.launch`启动文件。
```
    <param name="sdf_map/box_min_x" value="-48" type="double"/>
    <param name="sdf_map/box_min_y" value="-3" type="double"/>
    <param name="sdf_map/box_min_z" value="0" type="double"/> 
    <param name="sdf_map/box_max_x" value="3" type="double"/>
    <param name="sdf_map/box_max_y" value="12" type="double"/>
    <param name="sdf_map/box_max_z" value="1.7" type="double"/>
```
2. 注意`<arg name="is_coordinate" default="false"/>`要修改为true，打开协同模式。
3. 注意修改参数`<arg name="drone_num" value="3" />`和`<arg name="drone_id" value="1" />`前者务必与实际启动的数量相匹配，后者是从1开始的序号。

### 3.3. 示例（三车协同探索为例）
1. 启动仿真器：`roslaunch vehicle_simulator Simulator_car_env_3car.launch`，注意，等待仿真器完全启动之后再继续操作；
2. 启动每个车功能节点：`roslaunch vehicle_rea ants_00(1-3)_exp.launch`，注意，没啥注意的，参数合适即可；
3. 发布触发信号，格式为`geometry_msgs/PoseStamped`，名称为`/move_base_simple/goal`。
4. 等待探索结束即可。

<!-- 第五部分 -->
## 4、目标跟踪
### 启动文件目标跟踪布尔量启动即可
