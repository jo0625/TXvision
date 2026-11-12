# RM 自瞄学习对话迁移说明

最后更新：2026-08-26（Asia/Shanghai）

## 0. 新对话先做什么

新对话开始后，请先完整阅读本文件，然后从“第 7 节：当前停点”继续教学。

不要重新从 ROS package、YOLO 或相机标定开始，也不要因为日期晚于原计划就跳过
KF/EKF、预测、弹道或上机配置。

建议用户在新对话中直接发送：

```text
请先读取：
/home/jo/my-project/rm_vision/rm_auto_aim/docs/auto_aim_learning_conversation_handoff_2026-08-26.md

按文件中的当前停点继续教学，不要重新开始，也不要按日期跳过未学内容。
```

模型训练、候选冻结和 mini 主机迁移是另一条工作线。涉及这些状态时，还必须读取：

```text
/home/jo/my-project/rm_vision/rm_auto_aim/docs/yolo_model_training_handoff_4_wsl_2026-08-26.md
```

本文件只记录学习对话，不替代模型冻结、Git 或部署交接文件。

## 1. 总目标

从宏观到微观理解：

```text
/home/jo/my-project/rm_vision/rm_auto_aim
```

最终能够：

1. 独立解释整条 RoboMaster 自瞄数据链。
2. 看懂关键 ROS、PnP、TF、Tracker、KF/EKF、预测和上机代码。
3. 完成真实相机、动态 TF、跟踪、预测、电控和安全链路验收。
4. 学习完成后生成一套适合培训新生的资料。

主要参考项目：

```text
主线：/home/jo/my-project/rm_vision/rm_auto_aim
后续对照：/home/jo/my-project/sp_vision_25
训练工程：/home/jo/my-project/my_first_autoaim_yolo
```

当前不应同时泛读多个项目。先吃透 `rm_auto_aim` 的现有主链，到手眼外参、时间同步、
预测、弹道和电控阶段，再有针对性地读取 `sp_vision_25`。

## 2. 教学要求与用户偏好

- 全程使用中文。
- 使用项目真实代码，不只讲通用理论。
- 从宏观数据流逐步下钻到公式和代码行。
- 每次控制知识量，使用具体数字例子和短测验。
- 用户通常使用编号给出简短答案。
- 区分概念错误、漏答、符号错误和单纯算术笔误。
- 不要因为日期推进而把没学完的内容标记为完成。
- 用户明确要求“不急着赶进度，不删减内容”。
- 原定 8 月 31 日不再作为强制结课日；如果未学完，只做阶段总结并继续顺延。
- 新生培训资料必须在实际学习完成后生成，不能提前假装结课。
- 用户容易混淆坐标轴符号、`ya/za`、消息嵌套和相近变量，需要加入物理含义复核。
- 用户对具体数值和代码对应关系掌握较好，不应只让其死记结论。

每个小节结束都必须给出：

```text
本节是否完成
今天应学到哪里
建议学习时长
当天剩余内容
下一步
```

## 3. 当前真实进度

### 已完成

```text
ROS package、executable、node、topic 的关系
remap、parameter、QoS、callback、timer、采集线程、watchdog
YOLO conf、NMS、max_det、颜色协议、输出协议和冻结工件
相机 K/D、投影、X/Z、像素射线、CameraInfo
棋盘格标定、分辨率/裁剪对内参的影响、内外参区别
像素/相机光学/装甲/odom 等坐标系
PnP 点序、tvec/rvec、Rodrigues、四元数、IPPE 平面多解
重投影误差、正深度、朝向、时序筛选
大小装甲模型错误对距离尺度的影响
Armor/Armors/Pose/PoseStamped/Header 的消息嵌套
相机坐标到 odom 的 TF 公式、轴方向和手算
tf2_ros::MessageFilter 的等待、放行和丢弃逻辑
Tracker 输入异常过滤、初始化、匹配和状态机
机器人旋转中心、target.position 和初始半径 r 的基础语义
```

### 尚未完成

```text
普通 KF 的完整预测和更新
P、Q、R、K 的完整含义和数值练习
多维状态、矩阵 F/H 和协方差传播
为什么从 KF 进入 EKF
当前 9 维 EKF 的 f/h/J_f/J_h/Q/R
r 的可观测性、参数症状和完整代码行为
装甲跳变、双半径、dz、yaw 连续化
数字 ID 与 Tracker 协议冻结
手眼外参、图像时间戳与 IMU 插值
延迟预测、子弹到达时刻、弹道、选板
电控协议、云台输出、开火门控和 fail-closed
mini 主机部署、生产 launch 和分级实车验收
新生培训资料
```

特别注意：此前日程表中的“8 月 25 日：KF/EKF”只是计划，不是完成记录。
截至本文件生成时，KF 只讲到最基础的预测/测量和一维卡尔曼增益例子，EKF 尚未
正式完成。

## 4. 已确认的关键知识

### 4.1 消息层级

```text
/detector/armors
└── Armors
    ├── Header header
    └── Armor[] armors
        └── Armor.pose (geometry_msgs/Pose)
```

`Pose` 只有位置和四元数，没有时间戳和坐标系。Tracker 将公共 Header 与每块 Pose
临时组成 `PoseStamped`：

```cpp
geometry_msgs::msg::PoseStamped ps;
ps.header = armors_msg->header;
ps.pose = armor.pose;
```

空 `armors=[]` 表示收到了一条有效消息，但该帧没有检测到装甲板。

### 4.2 TF

位置变换：

```text
p_odom = R_odom_camera * p_camera + t_odom_camera
```

中性测试轴映射：

```text
camera x（右） -> odom -y
camera y（下） -> odom -z
camera z（前） -> odom +x
```

所以：

```text
(x_o, y_o, z_o) = (z_c, -x_c, -y_c)
```

测试静态旋转：

```text
Translation: [0, 0, 0]
Quaternion xyzw: [-0.5, 0.5, -0.5, 0.5]
RPY: [-90 deg, 0 deg, -90 deg]

R =
 0  0  1
-1  0  0
 0 -1  0
```

用户已经能正确完成该坐标映射，但曾出现 `camera y` 到 `odom z` 的符号笔误，教学时
仍应要求给出物理解释。

`MessageFilter` 的项目配置：

```text
target frame: odom
queue size: 10
TF buffer timeout: 约 1 s
```

它只判断 TF 是否可用，不亲自做坐标转换。真正转换发生在 `armorsCallback()` 的
`tf2_buffer_->transform(...)`。

### 4.3 TF 当前工程边界

- `rm_auto_aim` 内没有真实机器人动态 TF broadcaster。
- launch 中只有可选的测试静态 `odom -> camera_optical_frame`。
- 实车需要电控/IMU/关节角的 ROS 桥接节点发布动态 TF。
- 单独存在 Odometry 消息不会自动形成 TF 树。
- 当前海康相机在 Bayer 转 BGR 后才调用 `now()`，不是严格硬件曝光时间戳。
- 动态云台必须验收 TF 的存在、方向、平移、动态变化和时间同步。
- 静止敌人、云台转动时，相机坐标会变，但正确转换后的 odom 坐标应基本稳定。

本轮曾临时启动一次测试静态 TF 并通过 `tf2_echo` 验证上述矩阵，随后已停止。生成
本文件时没有遗留该后台进程，也没有运行中的真实相机/Detector/Tracker 链。

### 4.4 Tracker 输入和状态机

TF 转换后先删除：

```text
abs(odom.z) > 1.2
或
sqrt(odom.x^2 + odom.y^2) > 10.0
```

这里的 `odom.z` 是世界高度，不是相机前向距离。

初始化时选择 `distance_to_image_center` 最小的装甲，不是选择三维距离最近或置信度
最高的装甲。

代码实际有五个状态：

```text
LOST
DETECTING
TRACKING
TEMP_LOST
CHANGE_TARGET
```

正常自动主路径：

```text
LOST -> DETECTING -> TRACKING -> TEMP_LOST -> TRACKING/LOST
```

输出合同：

```text
LOST          tracking=false
DETECTING     tracking=false
TRACKING      tracking=true
TEMP_LOST     tracking=true（短时使用 EKF 预测）
CHANGE_TARGET tracking=false
```

一帧匹配成功要求：

```text
同一 tracked_id
三维位置差 < 0.15 m
yaw 差 < 1.0 rad
```

当前默认 YOLO 编号协议曾把所有装甲写成 `"armor"`，因此数字 ID 与 Tracker 的正式
接口仍需以当前部署交接和参数为准重新核查，不能假定已经完成数字分类。

### 4.5 当前已发现但尚未修改的源码风险

`Tracker` 的 `detect_count_` 和 `lost_count_` 没有明确初始化，基础整数可能从未确定值
开始。已向用户解释推荐修复：

```cpp
int detect_count_{0};
int lost_count_{0};
int change_count_{0};
double last_yaw_{0.0};
```

并在 `init()`、`initChange()` 中重置生命周期计数。只添加成员默认值不足以处理
`/tracker/reset` 或重新捕获目标后继承旧计数的问题。

这只是教学中发现并解释的风险，尚未修改源码，也尚未新增测试。

另外：

```cpp
if (detect_count_ > tracking_thres)
```

在 `tracking_thres=5` 时，按计数从零理解需要 6 次成功 update。把 `>` 改成 `>=`
属于跟踪策略变化，不应与未初始化修复混为一次无行为变化的修改。

## 5. 旋转中心、r 和 Target

PnP 测量当前装甲板：

```text
measurement = [xa, ya, za, yaw]
```

Tracker 状态的真实代码顺序：

```text
[xc, vxc, yc, vyc, za, vza, yaw, v_yaw, r]
```

不要照抄旧 README 中不同的状态排列，以代码为准。

当前模型：

```text
xa = xc - r*cos(yaw)
ya = yc - r*sin(yaw)

xc = xa + r*cos(yaw)
yc = ya + r*sin(yaw)
```

`initEKF()` 使用经验初值：

```text
r = 0.2 m
速度全部为 0
```

`r` 是机器人旋转中心到装甲中心的水平距离，不是装甲宽度，也不是单帧 PnP 能直接
测得的量。仓库没有给出精确选择 0.2 m 的标定证据。

状态更新后，代码将当前 `r` 限制为：

```text
0.12 m <= r <= 0.4 m
```

`target.position` 的真实含义：

```text
target.position.x = xc       机器人旋转中心 x
target.position.y = yc       机器人旋转中心 y
target.position.z = za       当前装甲高度
```

它不是当前可见装甲中心，也不是完整意义上的机器人三维质心，不能直接当作枪口瞄准点。

### r 如何变化

确认帧计数本身不会直接修改 `r`。成功匹配后：

```cpp
measurement = [xa, ya, za, measured_yaw];
target_state = ekf.update(measurement);
```

因为观测函数 `h(x)` 含有 `r*cos(yaw)`、`r*sin(yaw)`，测量残差可以通过卡尔曼增益
修正 `target_state(8)`。

预测函数 `f(x)` 不直接改变 `r` 的均值；过程噪声增加其不确定性，真正的连续数值修正
发生在有匹配测量的 EKF update。

静止且只观察同一块装甲时，中心与半径不容易区分；目标旋转、观测方向变化后，`r`
才更容易被观测。四装甲模型发生装甲跳变时，还会交换当前 `r` 和 `another_r`。

## 6. KF 当前只讲到哪里

已经讲过：

```text
状态不是测量
状态可以包含位置和速度
无测量时可用上一帧后验状态、dt 和运动模型暂时预测
```

一维示例：

```text
上一帧位置 3.0 m
速度 1.0 m/s
dt 0.1 s
预测位置 3.1 m
实际测量 3.16 m
```

已经首次引入一维卡尔曼增益：

```text
P = 0.04
R = 0.01
K = P/(P+R) = 0.8

x_update = 3.10 + 0.8*(3.16-3.10) = 3.148 m
```

用户已经正确区分：

```text
3.10 m：模型预测
3.16 m：测量
无测量：依靠上一帧位置、速度、dt 和运动模型预测
```

尚未系统讲解：

```text
先验/后验状态
P 的协方差含义
Q 为什么使预测不确定性增加
R 的来源和标定
K 如何由 P/H/R 计算
更新后 P 为什么减小
二维 [position, velocity] 的 F、H
标准 KF 完整数值循环
```

## 7. 当前停点：新对话从这里继续

用户尚未回答下面四题：

```text
1. 没有新测量时，预测阶段会不会主动改变 r 的数值？
2. yaw=0、中心可信，实际 xa 比预测值更小，r 应倾向增大还是减小？
3. 测量噪声 R 越大，卡尔曼增益 K 通常越大还是越小？
4. K=0.8 表示更相信预测还是更相信测量？
```

预期核心答案：

```text
1. 不会；当前 f(x) 保持 r 均值不变，没有测量就不进行残差更新。
2. 增大。
3. 变小。
4. 更相信测量；本次修正采用 80% 的测量残差。
```

新对话应先批改这四题，然后继续普通 KF，不要直接跳到当前项目 9 维 EKF。

推荐下一教学顺序：

```text
1. 一维位置 KF：先验、残差、K、后验
2. P、Q、R 的物理意义和调参症状
3. 二维状态 [position, velocity] 的 F/H
4. 完成一个从 predict 到 update 的数值例题
5. 再解释线性 KF 为什么不能直接处理 cos(yaw)、sin(yaw)
6. 引入 EKF、f/h 和 Jacobian
7. 对照 tracker_node.cpp 的 9 维代码
```

## 8. 后续完整课程顺序

按掌握度推进，当前估计仍需约 28 至 32 小时：

```text
阶段 A：普通 KF 完整基础与数值例题
阶段 B：当前 9 维 EKF、f/h/J_f/J_h/P/Q/R
阶段 C：yaw 连续化、装甲跳变、双半径、dz、ID 协议
阶段 D：sp_vision_25 的手眼外参和图像/IMU时间同步
阶段 E：延迟预测、子弹飞行时间、弹道、选板
阶段 F：电控协议、云台输出、开火门控和安全
阶段 G：mini 主机部署、生产 launch、性能与故障验收
阶段 H：总考核与新生培训资料
```

如果 8 月 31 日仍未完成，只记录阶段进度，不删减课程，也不把未讲内容标记为完成。

## 9. 当前仓库实现边界

`rm_auto_aim` 已有：

```text
海康相机
YOLO/传统 Detector
PnP
TF Listener/MessageFilter
Tracker/EKF
Target 接口
watchdog
launch/params/诊断工具
```

当前仓库没有完整提供：

```text
真实机器人动态 TF 发布者
曝光时刻与 IMU 精确同步
从机器人中心状态选择未来可击打装甲
完整弹道解算
正式串口/CAN 电控桥
云台控制输出和开火授权
完整生产 launch 与比赛自恢复验收
```

因此 `/tracker/target` 不是整条自瞄链路的终点，更不能直接等同于枪口命令。

## 10. 关键代码路径

```text
消息：
auto_aim_interfaces/msg/Armor.msg
auto_aim_interfaces/msg/Armors.msg
auto_aim_interfaces/msg/Target.msg

YOLO消息发布：
yolo_armor_detector/scripts/yolo_armor_detector_node.py

TF、EKF模型和Target发布：
armor_tracker/src/tracker_node.cpp

Tracker初始化、匹配、状态机、r和装甲跳变：
armor_tracker/src/tracker.cpp

EKF predict/update：
armor_tracker/src/extended_kalman_filter.cpp

Tracker参数：
auto_aim_bringup/params/tracker_params.yaml

YOLO主launch：
auto_aim_bringup/launch/yolo_auto_aim.launch.py

相机时间戳：
hik_camera/src/hik_camera_node.cpp

watchdog：
auto_aim_safety/scripts/target_watchdog_node.py
```

## 11. 本次文件操作

本次只新增了本学习迁移文件，没有修改 Tracker、EKF、参数、模型、测试或部署配置。

