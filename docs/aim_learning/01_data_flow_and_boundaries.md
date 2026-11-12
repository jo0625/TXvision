# 第 1 课：完整数据流和职责边界

## 1. 本课目标

学完本课，应能区分：

```text
Detector、PnP/Solver、Tracker、Aimer、Planner、Shooter、通信桥和 Controller
```

本课只解决一个核心问题：一条自瞄流水线中，每个模块究竟负责什么。

## 2. 真实源码入口

本课主要使用：

- [项目说明](../../../../sp_vision_25/readme.md)
- [基础主循环 standard.cpp](../../../../sp_vision_25/src/standard.cpp)
- [高级规划入口 standard_mpc.cpp](../../../../sp_vision_25/src/standard_mpc.cpp)
- [瞄准器 aimer.cpp](../../../../sp_vision_25/tasks/auto_aim/aimer.cpp)
- [跟踪器 tracker.cpp](../../../../sp_vision_25/tasks/auto_aim/tracker.cpp)
- [位姿解算器 solver.cpp](../../../../sp_vision_25/tasks/auto_aim/solver.cpp)
- [开火判断 shooter.cpp](../../../../sp_vision_25/tasks/auto_aim/shooter.cpp)
- [C 板通信 cboard.cpp](../../../../sp_vision_25/io/cboard.cpp)

## 3. 先建立总图

自瞄不是一个巨大的函数，而是一条数据加工流水线：

```text
现实中的光
  -> 相机图像
  -> Detector：找到装甲板的二维像素点
  -> Solver/PnP：求装甲板三维位置
  -> Tracker：估计目标的连续运动状态
  -> Aimer：计算应该瞄准的未来位置和角度
  -> Planner：把理想角度变成云台做得到的轨迹
  -> Shooter：判断当前是否允许开火
  -> 通信桥：编码并发送命令
  -> 下位机 Controller：闭环驱动电机
  -> 云台实际运动
```

每一级只应使用自己真正需要的信息。如果某一级失败，要能判断问题属于哪一层。

## 4. `standard.cpp` 实际执行了什么

基础入口的核心循环位于 `standard.cpp:57-80`。去掉辅助代码后，执行顺序是：

```cpp
camera.read(img, t);
q = cboard.imu_at(t - 1ms);
solver.set_R_gimbal2world(q);

auto armors = detector.detect(img);
auto targets = tracker.track(armors, t);
auto command = aimer.aim(targets, t, cboard.bullet_speed);

cboard.send(command);
```

这里已经出现一条完整的“观察到发送”链路，但它不是参考项目唯一的运行入口。

## 5. Detector：回答“图像里有什么”

对应调用：

```cpp
auto armors = detector.detect(img);
```

Detector 的输入是图像，输出是装甲板候选，包括装甲板类别、颜色和四个像素点。

它能回答：

```text
图像的哪个位置像装甲板？
它可能是哪一类装甲板？
```

它不能只靠单张二维图像直接回答：

```text
装甲板距离枪口多少米？
目标以多少米每秒移动？
现在应该给云台多少角度？
```

## 6. Solver/PnP：回答“装甲板在三维空间哪里”

`Solver::solve()` 在 `solver.cpp:54` 开始。它使用装甲板的真实尺寸、四个像素点、相机内参和畸变参数调用 `cv::solvePnP()`。

关键结果是：

```cpp
armor.xyz_in_gimbal = R_camera2gimbal_ * xyz_in_camera + t_camera2gimbal_;
armor.xyz_in_world = R_gimbal2world_ * armor.xyz_in_gimbal;
```

所以 Solver/PnP 的职责是：

```text
二维像素点
  -> 相机坐标中的三维位置
  -> 云台坐标中的三维位置
  -> world 坐标中的三维位置
```

一个容易误解的地方是：`standard.cpp` 没有直接写 `solver.solve(armor)`。实际调用藏在 Tracker 内部：

```text
Tracker::set_target()    -> solver_.solve(armor)
Tracker::update_target() -> solver_.solve(armor)
```

因此：

```text
函数调用关系：main -> Tracker -> Solver
算法职责关系：Detector -> Solver/PnP -> Tracker
```

## 7. Tracker：回答“目标正在怎样运动”

单帧 Detector 看到的是一块装甲板。Tracker 处理连续多帧，需要判断：

```text
这一帧和上一帧是否是同一个目标？
目标是否刚出现、正在稳定跟踪，还是暂时丢失？
整车中心在哪里？
目标的平移速度和旋转速度是多少？
没有直接看见的其他装甲板可能在哪里？
```

`tracker.cpp` 中存在以下状态：

```text
lost -> detecting -> tracking -> temp_lost
```

当相邻图像时间间隔超过 `0.1 s` 时，它还会认为相机可能离线并回到 `lost`。这说明 Tracker 不只是“保存上一帧检测框”，它在维护带时间的目标状态。

Tracker 输出的 `Target` 应理解为：

> 根据已有观测，对一辆目标当前运动状态作出的估计。

## 8. Aimer：回答“理论上应该瞄哪里”

基础入口调用：

```cpp
auto command = aimer.aim(targets, t, cboard.bullet_speed);
```

`Aimer::aim()` 从 `aimer.cpp:32` 开始，主要完成：

1. 根据目标状态选择一块装甲板。
2. 把目标从图像时间预测到较新的时刻。
3. 计算初次弹丸飞行时间。
4. 预测弹丸到达时目标的位置。
5. 重新计算飞行时间，最多迭代 10 次。
6. 输出 yaw 和 pitch。

最终角度位于 `aimer.cpp:118-122`：

```cpp
double yaw = std::atan2(final_xyz.y(), final_xyz.x()) + yaw_offset_;
double pitch = -(current_traj.pitch + pitch_offset_);
return {true, false, yaw, pitch};
```

注意返回值中的第二项是 `false`，即这里明确输出 `shoot=false`。

Aimer 负责计算“理论瞄准角”，但它不会自动保证真实云台能瞬间到达这个角度。

## 9. Planner：回答“云台怎样才能做到”

旋转目标切换装甲板时，理论 yaw 可能突然跳变。真实云台有质量、惯量和最大电机扭矩，不可能瞬间改变角度和速度。

Planner 的任务是把理想瞄准轨迹变成带约束的命令：

```text
yaw、yaw_vel、yaw_acc
pitch、pitch_vel、pitch_acc
```

基础 `standard.cpp` 没有调用 Planner。高级入口 `standard_mpc.cpp:70-88` 启动独立规划线程，执行：

```cpp
auto plan = planner.plan(target, gs.bullet_speed);
gimbal.send(
  plan.control, plan.fire,
  plan.yaw, plan.yaw_vel, plan.yaw_acc,
  plan.pitch, plan.pitch_vel, plan.pitch_acc);
```

因此不要把 README 中的完整架构误认为每个可执行程序都同时运行全部模块。

## 10. Shooter：回答“现在能不能开火”

传统 `Shooter::shoot()` 会检查：

```text
视觉控制命令是否有效
是否存在目标
配置是否允许自动开火
命令是否发生突变
云台实际角与期望角是否足够接近
当前瞄准点是否有效
```

一个重要源码事实：`standard.cpp` 虽然构造了 `Shooter shooter`，但主循环没有调用它。`Aimer` 又固定返回 `shoot=false`，所以这条基础入口不会通过 Shooter 打开开火位。

在 `sentry.cpp`、`uav.cpp` 等入口中可以看到：

```cpp
command.shoot = shooter.shoot(...);
```

在高级 Planner 路径中，开火判断又被整合为 `plan.fire`。因此“开火决策”是一种职责，不一定永远对应同一个类。

## 11. 通信桥和 Controller 不是同一个东西

`CBoard::send()` 在 `cboard.cpp:47` 开始。它把 `control`、`shoot`、`yaw`、`pitch` 等字段编码进 8 字节 CAN 帧。

反方向上，`CBoard::callback()` 接收：

```text
IMU 四元数
实时弹速
当前模式
射击模式
```

所以 `CBoard` 是电脑侧的通信桥。它负责传输和协议编码，不负责根据角度误差计算电机电流。

真实闭环通常发生在下位机：

```text
视觉电脑发送目标角度
  -> 下位机读取编码器或 IMU 的实际角度
  -> Controller 计算角度误差
  -> Controller 计算电流或力矩
  -> CAN 电机执行
```

当前机器人的下位机固件和协议尚未取得，因此这部分只能建立职责模型，不能标记为当前机器人已经完成。

## 12. 静止目标例子

假设 Solver 得到目标位置：

```text
x = 4.0 m
y = 0.2 m
z = 0.1 m
```

参考项目 world 坐标下的几何 yaw 约为：

```text
yaw = atan2(0.2, 4.0)
    = 0.050 rad
    = 2.86°
```

这时各模块的职责是：

```text
Detector：稳定找到同一块装甲板
Solver：给出三维位置
Tracker：估计目标速度接近 0
Aimer：计算 yaw，并补偿弹丸重力下坠
Planner：让云台平滑到达目标角度
Shooter：确认误差、安全状态和开火条件
Controller：驱动真实电机跟随
```

静止目标不需要明显的运动提前量，但仍需要弹道补偿。

## 13. 旋转目标例子

假设目标整车原地以 `6 rad/s` 旋转。当前看到正面装甲板，几十毫秒后侧面装甲板转入可射击范围。

```text
Detector：报告每帧真正看到的装甲板
Tracker：估计整车中心、旋转方向和角速度
Aimer：预测弹丸到达时各装甲板的位置并选板
Planner：在装甲板切换前减速，避免角度命令突变
Shooter：只在计划轨迹和射击轨迹足够接近时开火
Controller：执行规划后的角度、速度和加速度命令
```

如果命令突然跳变而云台来不及跟上，Detector 即使完全正确，也不能单独解决这个问题。

## 14. 本课核心结论

```text
Detector 负责看见
Solver/PnP 负责三维定位
Tracker 负责估计运动
Aimer 负责计算应该瞄哪里
Planner 负责让命令可执行
Shooter 负责决定能否开火
通信桥负责传输
下位机 Controller 负责真正驱动电机
```

## 15. 检查题

请在聊天框回答“第1题”：

> 目标正在旋转。Detector 每帧识别都正确，PnP 距离也正确，但装甲板切换时角度命令突然跳变，真实云台来不及转过去。这个问题首先属于 Detector、Tracker、Planner、CBoard 中哪个模块的职责？为什么？

## 16. 下一课

第 2 课将解释同一个目标为什么能同时拥有“相机坐标”“云台坐标”和“world 坐标”，以及 yaw/pitch 的正负号为什么不能靠猜。
