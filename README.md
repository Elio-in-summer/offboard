OFFBOARD
==================

## Demo
### 位置控制示例程序:悬停
- 源文件:  `posctl_hover.cpp`
- 所用接口: `mavros/setpoint_position/local`
- 效果：垂直起飞后悬停在一定高度一段时间，然后降落
- 用法：
```shell
rosrun offboard posctl_hover
```
### 位置控制示例程序:转圈
- 源文件:  `posctl_circle.cpp`
- 所用接口: `mavros/setpoint_position/local`
- 效果：(默认起飞位置为（0,0），前x左y)
  - `case0`: 悬停
  - `case1`: 垂直起飞后悬停在1m位置一段时间，然后开始画圆，半径1m，圆心为（0,-1）,`yaw`不转动
  - `case2`: 垂直起飞后悬停在1m位置一段时间，在原地转动`yaw`角
  - `case3`: 垂直起飞后悬停在1m位置一段时间，然后开始画圆，半径1m，圆心为（0,-1）,`yaw`==转动==

  注： ==一定要注意起飞点的左侧两米范围内无障碍，不然可能会碰撞==，以上case在`posctl_circle.cpp`自行注释选择
- 用法：
```shell
roslaunch offboard posctl_circle.launch
```
- `launch`可调整的参数 
  - `ctrl_rate`: 控制频率(**尽量不要修改**)
  - `repeat_path`: 是否重复轨迹(**尽量不要修改**)
  - `hover_height`：悬停高度
  - `r_circle`：绕圈半径
  - `t_total`：绕圈总时长

### 姿态控制示例程序:转圈
- 源文件:  `attctl_circle.cpp` `controller_node.cpp`
  - `attctl_circle.cpp`: 规划结点，将规划好的点通过话题`/setpoint_pva`发送给控制节点
  - `controller_node.cpp`: 控制结点，接收话题`/setpoint_pva`传过来的规划点(包含位置、速度、加速度、`yaw`)，利用PID计算姿态与油门，通过话题`mavros/setpoint_raw/attitude`发送给`px4`
  
  **注**： 鉴于实际飞行中，无人机体积可能比较大，一旦失控比较危险，因此本`demo`在`controller_node.cpp`中对控制量做了以下限制：
    
  - `Z`轴位置积分上限: `1.5`
  - Z方向加速度: `(-1.5,1.5)`
  - 机身法线与`Z`轴最大夹角: `15度`
  - 总推力: (在`cfg/uav.yaml`)中自行设定

- 所用接口: `mavros/setpoint_raw/attitude`
- 效果：(默认起飞位置为（0,0），前x左y)
  - `case0`: 悬停
  - `case1`: 垂直起飞后悬停在1m位置一段时间，然后开始画圆，半径1m，圆心为（0,-1）,`yaw`不转动
  - `case2`: 垂直起飞后悬停在1m位置一段时间，在原地转动`yaw`角
  - `case3`: 垂直起飞后悬停在1m位置一段时间，然后开始画圆，半径1m，圆心为（0,-1）,`yaw`==转动==
  - `case4`: 在`case3`的基础上指定转圈时的速度与加速度

  **注**： ==一定要注意起飞点的左侧两米范围内无障碍，不然可能会碰撞==，以上case在`attctl_circle.cpp`自行注释选择
- 用法：
```shell
roslaunch offboard attctl_circle.launch
```
- `launch`可调整的参数 
  - `ctrl_rate`: 控制频率(**尽量不要修改**)
  - `repeat_path`: 是否重复轨迹(**尽量不要修改**)
  - `hover_height`：悬停高度
  - `r_circle`：绕圈半径
  - `t_total`：绕圈总时长

