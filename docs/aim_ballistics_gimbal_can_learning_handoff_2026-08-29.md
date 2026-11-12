# 自瞄计算、弹道、云台与下位机通信学习交接

最后更新：2026-08-29（Asia/Shanghai）

## 0. 新对话如何开始

新对话先完整读取本文件，再按第 10 节的顺序教学。不要重新开展模型训练、
gray140 采集、相机帧率诊断或 mini 主机选型。

建议用户在新对话中直接发送：

```text
请先读取：
/home/jo/my-project/rm_vision/rm_auto_aim/docs/aim_ballistics_gimbal_can_learning_handoff_2026-08-29.md

这是专门学习瞄准计算、弹道补偿、云台约束和下位机通信的对话。
请按文档第 10 节开始，从小白角度结合真实源码教学；先讲解和提问，
不要直接修改代码，也不要转回模型训练或帧率诊断。
```

需要核对当前 YOLO 候选版本时，再读取：

```text
/home/jo/my-project/rm_vision/rm_auto_aim/docs/yolo_model_training_handoff_4_wsl_2026-08-26.md
```

更广泛的 ROS、PnP、TF、Tracker 和 KF/EKF 学习进度记录在：

```text
/home/jo/my-project/rm_vision/rm_auto_aim/docs/auto_aim_learning_conversation_handoff_2026-08-26.md
```

## 1. 本学习对话的目标

本对话只聚焦以下问题：

1. PnP 三维坐标怎样变成云台 yaw/pitch。
2. 相机内参、相机到云台外参和枪管偏差分别解决什么问题。
3. 重力、弹速和目标距离怎样决定弹道 pitch 与飞行时间。
4. 图像、推理、通信、云台和弹丸延迟怎样形成运动目标提前量。
5. 为什么云台需要角度、角速度和角加速度约束。
6. CAN、USB 串口、下位机、云台电机之间是什么关系。
7. 如何把参考项目 `sp_vision_25` 的算法迁移为当前 ROS2 项目的独立节点。
8. 如何从模拟输出逐级验收到无发射云台跟随。

学习结束后，用户应能看懂数据从装甲板坐标到下位机命令的全过程，并能判断：

```text
问题属于检测、PnP、坐标、弹道、预测、轨迹规划、通信，还是电机控制。
```

## 2. 两个项目的角色

当前开发主线：

```text
/home/jo/my-project/rm_vision/rm_auto_aim
```

它目前已经具备：

```text
工业相机
  -> /image_raw + /camera_info
  -> YOLO Pose 四角点
  -> 大小装甲板策略 + PnP
  -> /detector/armors
  -> armor_tracker
  -> /tracker/raw_target
  -> target_watchdog
  -> /tracker/target
```

它目前没有完整实现：

```text
瞄准角解算节点
弹道补偿节点
云台命令消息
真实下位机串口/CAN 节点
动态云台/IMU TF 发布
完整开火门控
```

主要参考项目：

```text
/home/jo/my-project/sp_vision_25
```

该项目是无 ROS 依赖的完整自瞄参考实现，包含相机、检测、PnP、手眼外参、
整车估计、弹道、目标预测、轨迹规划、开火决策和下位机通信。

重要边界：参考它的算法与模块划分，不直接复制另一台机器人的参数、坐标约定和
通信协议。

## 3. 当前已确认的弹道实现

### 3.1 静止目标弹道

源码：

```text
/home/jo/my-project/sp_vision_25/tools/trajectory.hpp
/home/jo/my-project/sp_vision_25/tools/trajectory.cpp
```

输入：

```text
v0：弹丸初速度，m/s
d：目标水平距离，m
h：目标相对高度，m
```

输出：

```text
pitch：需要的发射俯仰角，抬头为正
fly_time：弹丸飞行时间，s
unsolvable：在当前弹速和位置下是否无实数解
```

实现使用重力常数：

```text
g = 9.7833 m/s^2
```

它采用无空气阻力的抛体模型：

```text
h = d * tan(pitch) - g * d^2 / (2 * v0^2 * cos(pitch)^2)
```

令 `u = tan(pitch)` 后可转为一元二次方程。代码求出高、低两条可能弹道，选择
飞行时间更短的低抛弹道。

当前限制：

```text
不考虑空气阻力
不考虑弹丸初速波动
不自动知道枪管真实出口位置
不代表已经完成实弹标定
```

### 3.2 运动目标与时间预测

源码：

```text
/home/jo/my-project/sp_vision_25/tasks/auto_aim/aimer.cpp
```

基本过程：

```text
当前目标状态
  -> 补偿图像处理和系统延迟
  -> 选择可射击装甲板
  -> 计算第一次弹丸飞行时间
  -> 预测目标在子弹到达时的位置
  -> 用新位置重新计算飞行时间
  -> 最多迭代 10 次
  -> 相邻飞行时间差小于 0.001 s 时收敛
  -> 输出 yaw/pitch
```

参考代码在弹速小于 `14 m/s` 时回退到 `23 m/s`。这是原机器人的策略，不能直接
复制到当前项目。当前项目必须把弹速来源、允许范围和失效行为参数化。

### 3.3 目标选择

同一 `aimer.cpp` 会根据 tracker 估计的整车中心、装甲板位置和旋转方向选择瞄准点，
并对两个约 45 度装甲板之间的切换做锁定处理。

这属于后续旋转目标功能。当前项目第一版更适合先使用 `/detector/armors` 中真实可见
的装甲板，完成静止和低速目标几何瞄准；不要一开始就把完整反陀螺决策一起迁移。

## 4. 从 PnP 到云台角度

### 4.1 当前相机光学坐标

```text
x_c：向图像右
y_c：向图像下
z_c：相机前方
```

不考虑安装偏差时，视线角可近似写为：

```text
yaw_camera   = atan2(x_c, z_c)
pitch_camera = atan2(-y_c, sqrt(x_c^2 + z_c^2))
```

这只说明目标相对相机光轴的角度，不等于最终枪口角度。

### 4.2 相机到云台外参

正确过程是先做刚体变换：

```text
p_gimbal = R_camera2gimbal * p_camera + t_camera2gimbal
```

其中：

```text
R_camera2gimbal：相机轴相对云台轴的旋转
t_camera2gimbal：相机光心相对云台参考点的平移
```

参考实现：

```text
/home/jo/my-project/sp_vision_25/tasks/auto_aim/solver.cpp
```

参考配置包含每台机器各自的：

```text
R_camera2gimbal
t_camera2gimbal
yaw_offset
pitch_offset
```

这些数值不能复制到当前机器人。尤其在近距离，只有固定 yaw/pitch 偏置往往不足，
因为相机和枪管存在平移视差。

### 4.3 内参与外参的区别

```text
相机内参 K/D：像素射线和镜头畸变
相机到云台外参 R/t：相机在机器人上的安装关系
枪管偏差：发射轴和云台/相机参考轴的差异
```

用户已经完成相机内参标定，但这不自动等于完成相机到云台的外参标定。

## 5. 几何瞄准、弹道补偿与运动预测

三者解决不同问题：

```text
几何瞄准：目标现在位于哪个方向
弹道补偿：弹丸受重力后应该抬高多少
运动预测：弹丸到达时目标将位于哪里
```

静止目标可以先按以下顺序计算：

```text
装甲板相机坐标
  -> 相机到云台坐标变换
  -> 水平距离和高度
  -> 弹道 pitch 与 fly_time
  -> yaw/pitch 安装偏差
  -> 有效瞄准命令
```

运动目标还要加入总预测时间：

```text
t_predict = 图像年龄
          + 推理/处理延迟
          + 通信延迟
          + 云台响应延迟
          + 弹丸飞行时间
```

最基础的匀速预测为：

```text
p_future = p_now + v * t_predict
```

完整自瞄还要处理旋转装甲板、装甲切换、目标加速度和自身运动。

## 6. 为什么云台需要约束

云台是有质量和惯性的真实机构。电机最大扭矩和云台转动惯量决定最大角加速度：

```text
alpha_max = torque_max / inertia
```

三类约束含义：

```text
角度约束：不能超过机械限位
角速度约束：云台最高转速有限
角加速度约束：转速不能瞬间改变
```

目标装甲板切换时，理论瞄准角可能从一侧突然跳到另一侧。无约束命令会导致电流
饱和、跟随滞后、超调、机械冲击以及开火判断错误。

参考高级实现：

```text
/home/jo/my-project/sp_vision_25/tasks/auto_aim/planner/planner.cpp
/home/jo/my-project/sp_vision_25/tasks/auto_aim/planner/tinympc
```

它使用轨迹规划和 TinyMPC，使计划轨迹满足云台加速度能力，并利用计划轨迹判断
开火时机。

当前项目第一版不需要立刻迁移 TinyMPC。先实现：

```text
角度限位 + 最大角速度限制 + 最大角加速度限制 + 失锁停止
```

## 7. CAN、串口与下位机

### 7.1 CAN 是什么

CAN 是 Controller Area Network，适合车辆和机器人内部通信。常见物理信号线为：

```text
CAN_H
CAN_L
```

Linux SocketCAN 通常把设备呈现为：

```text
can0
```

CAN 使用消息 ID 区分数据。经典 CAN 一帧最多携带 8 字节数据。

### 7.2 电脑与电机的关系

“电机使用 CAN”不代表电脑一定直接连接 CAN。可能存在两种结构：

```text
电脑 --USB串口--> 下位机 --CAN--> 电机
```

或：

```text
电脑 --USB-CAN--> 下位机/电机 CAN 总线
```

必须查看当前机器人下位机固件和接线，不能只根据电机型号推断。

### 7.3 参考项目的 CAN 实现

源码：

```text
/home/jo/my-project/sp_vision_25/io/socketcan.hpp
/home/jo/my-project/sp_vision_25/io/cboard.hpp
/home/jo/my-project/sp_vision_25/io/cboard.cpp
```

`CBoard` 当前完成：

```text
接收 IMU 四元数
按图像时间戳插值姿态
接收实时弹速、模式和射击模式
发送 control、shoot、yaw、pitch
SocketCAN 断线后尝试重新打开
```

`configs/standard4.yaml` 中的示例协议：

```text
quaternion_canid: 0x100
bullet_speed_canid: 0x101
send_canid: 0xff
can_interface: can0
```

这些 ID、字段顺序、缩放系数和字节序属于参考控制板协议，不自动适用于当前机器人。

### 7.4 下位机通信节点的职责

建议把协议与 YOLO 分离。通信节点负责：

```text
接收下位机 IMU/云台状态/弹速
发布带时间戳的状态和动态 TF
接收安全瞄准命令
编码串口或 CAN 数据包
检查消息年龄和 valid 状态
通信断开或目标过期时 fail-closed
```

第一阶段必须保持：

```text
shoot = false
```

## 8. 当前项目建议的后续软件结构

以下是待实现设计，不代表仓库已经存在这些包或消息：

```text
/detector/armors
  -> simple_aim
  -> /aim/command
  -> aim_command_watchdog
  -> /aim/safe_command
  -> lower_controller_bridge
  -> serial / CAN / mock
```

下位机反馈方向：

```text
lower_controller_bridge
  -> /gimbal/state
  -> 动态 TF
  -> /bullet_speed
```

建议消息概念：

```text
AimCommand
  header
  valid
  yaw
  pitch
  optional yaw_rate/pitch_rate
  distance
  shoot（第一阶段恒为 false）

GimbalState
  header
  yaw
  pitch
  optional quaternion
  bullet_speed
  communication_ok
```

第一版 `simple_aim` 使用当前可见装甲板；完整 tracker 可并行运行用于观察，后续再
接入运动目标和反陀螺瞄准。

## 9. 迁移时明确禁止直接复制的内容

不要直接从 `sp_vision_25` 复制：

```text
其他机器人的相机内参
R_camera2gimbal / t_camera2gimbal
yaw_offset / pitch_offset
23 m/s 弹速回退
0x100 / 0x101 / 0xff CAN ID
字段缩放和字节序
延迟参数
云台最大加速度
自动开火阈值
```

也不要直接把参考 `Aimer` 接到当前 `/tracker/target.position` 后控制枪口。两个项目的
消息结构、目标语义和坐标约定不同，必须建立明确的适配层和单元测试。

可以优先复用或移植的部分：

```text
无阻力弹道方程
飞行时间迭代思路
算法不可解处理
协议与传输分层思想
SocketCAN 读写结构
IMU 时间插值思路
轨迹约束的设计思想
```

## 10. 推荐学习顺序与当前停点

新对话从第 1 课开始，不需要重讲 ROS package、YOLO 训练或相机内参基础。

### 第 1 课：完整数据流和职责边界

目标：能区分 detector、PnP、tracker、aimer、planner、controller 和 shooter。

使用源码：

```text
sp_vision_25/readme.md
sp_vision_25/src/standard.cpp
sp_vision_25/tasks/auto_aim/aimer.cpp
sp_vision_25/io/cboard.cpp
```

### 第 2 课：坐标系与几何 yaw/pitch

目标：能手算相机坐标中的 yaw/pitch，并解释为什么必须先应用外参。

### 第 3 课：无空气阻力弹道方程

目标：从水平/竖直运动方程推导 `Trajectory`，理解判别式、低抛解和飞行时间。

### 第 4 课：目标运动与总延迟

目标：画出从曝光到弹丸命中的时间线，理解为什么弹道与目标预测需要迭代。

### 第 5 课：装甲板选择与旋转目标

目标：理解整车中心、多个装甲板、装甲切换、锁定和小陀螺选板。

### 第 6 课：云台可执行性

目标：理解角度、速度、加速度、扭矩、惯量、饱和和轨迹突变。

### 第 7 课：CAN 和串口协议

目标：能读懂 CAN ID、8 字节载荷、缩放、字节序、SocketCAN 和串口分帧。

### 第 8 课：ROS2 迁移设计

目标：设计消息、节点、参数、时间戳、mock 后端、watchdog 和 fail-closed 行为。

### 第 9 课：分级测试

目标：能制定以下验收顺序：

```text
纯公式单元测试
录制数据离线回放
mock 通信
真实通信但不驱动电机
低速无发射云台跟随
静止靶实弹
低速移动靶
旋转目标
```

### 当前停点

用户已经知道：

```text
当前 rm_auto_aim 没有完整弹道补偿
几何瞄准与弹道补偿不同
sp_vision_25 确实存在弹道和 CAN 实现
弹道算法计算“应该瞄哪里”
云台约束保证“真实机构能否做到”
```

新对话应从“第 1 课：完整数据流和职责边界”开始，通过一个静止目标和一个旋转
目标的具体例子建立总图，再进入公式。

## 11. 教学方式要求

- 全程中文，从小白角度解释，但不牺牲技术准确性。
- 每次只讲一个核心问题，避免一次堆入全部公式。
- 所有抽象概念尽量配一个带单位的数字例子。
- 先让用户解释物理含义，再检查公式和代码。
- 区分“源码确实实现”“可以离线验证”和“已经在当前机器人实机验收”。
- 不因参考项目已有完整算法就宣称当前项目可以直接发射。
- 学习对话默认不修改代码；用户明确要求实施后再切换到开发任务。
- 不重复模型训练、gray140 姿态采集、帧率诊断或 mini 主机选型。
- 不要求每个文件都做哈希，不进行与学习无关的过度校验。

每节结尾给出：

```text
本节核心结论
一个检查理解的小问题
下一节内容
```

## 12. 实现前仍需取得的真实信息

这些信息目前未知，学习时不能靠猜测填入：

```text
当前机器人下位机控制板型号
电脑到下位机使用 USB 串口还是 USB-CAN
下位机固件仓库或协议文档
CAN ID 或串口帧格式
角度单位、正方向和绝对/相对命令语义
下位机是否上传 IMU、云台角和实时弹速
相机相对云台和枪管的真实安装关系
实际弹丸速度范围
云台角度、角速度、角加速度限制
```

在这些信息尚未取得时，可以学习并实现算法核心、ROS 消息、mock 通信和单元测试，
但不能把真实下位机协议或实机参数标记为完成。
