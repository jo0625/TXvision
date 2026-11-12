# 第 2 课：坐标系与几何 yaw/pitch

## 1. 本课目标

学完本课，应能：

1. 解释一个三维坐标为什么必须同时写明所属坐标系。
2. 区分相机内参与相机到云台外参。
3. 手算相机光学坐标中的几何 yaw/pitch。
4. 解释为什么必须先做外参变换，再生成最终云台命令。

本课暂时不推导重力弹道，只讨论“目标现在位于哪个方向”。

## 2. 真实源码入口

本课主要使用：

- [位姿解算器 solver.cpp](../../../../sp_vision_25/tasks/auto_aim/solver.cpp)
- [位姿解算器声明 solver.hpp](../../../../sp_vision_25/tasks/auto_aim/solver.hpp)
- [坐标与角度工具 math_tools.cpp](../../../../sp_vision_25/tools/math_tools.cpp)
- [瞄准器 aimer.cpp](../../../../sp_vision_25/tasks/auto_aim/aimer.cpp)
- [参考机器配置 standard4.yaml](../../../../sp_vision_25/configs/standard4.yaml)

`standard4.yaml` 只用于阅读源码和理解参数结构，其中数值属于另一台机器人，不能复制到当前机器人。

## 3. 坐标必须带“尺子和方向”

只说：

```text
目标坐标是 (0.2, 0.1, 4.0)
```

信息并不完整。必须继续问：

```text
原点在哪里？
x、y、z 分别指向哪里？
单位是米还是毫米？
这个坐标对应哪个时间？
```

同一个物理目标在不同坐标系中会有不同的数字，但指向的是同一个位置。

## 4. 相机光学坐标

OpenCV PnP 常用的相机光学坐标约定是：

```text
x_c：图像右方
y_c：图像下方
z_c：相机前方
```

这是相机的光学约定，不是通用机器人本体坐标约定。

## 5. 相机坐标中的几何 yaw/pitch

若目标相机坐标为：

```text
p_c = (x_c, y_c, z_c)
```

定义“向图像右为正 yaw、向图像上为正 pitch”，则视线角可以写成：

```text
yaw_camera   = atan2(x_c, z_c)
pitch_camera = atan2(-y_c, sqrt(x_c^2 + z_c^2))
```

为什么使用 `atan2` 而不是普通 `atan(x/z)`：

```text
atan2 能区分象限
atan2 在分母接近 0 时更安全
atan2 明确表达“竖直分量”和“水平分量”
```

### 数字例子

假设：

```text
x_c = 0.20 m   目标在图像右边
y_c = 0.10 m   目标在图像下边
z_c = 4.00 m   目标在相机前方 4 m
```

则：

```text
yaw_camera = atan2(0.20, 4.00)
           ≈ +0.0500 rad
           ≈ +2.86°

pitch_camera = atan2(-0.10, sqrt(0.20^2 + 4.00^2))
             ≈ -0.0250 rad
             ≈ -1.43°
```

物理解释：目标在相机右下方，因此按本段定义，yaw 为正、pitch 为负。

## 6. 为什么这还不是最终云台角

相机可能：

```text
装在云台旋转轴旁边
相对云台有轻微歪斜
比枪管高几厘米
比枪管向左或向右几厘米
```

所以“目标相对相机光轴的方向”不等于“枪口或云台应该指向的方向”。

正确刚体变换是：

```text
p_gimbal = R_camera2gimbal * p_camera + t_camera2gimbal
```

其中：

| 量 | 含义 | 单位 |
|---|---|---|
| `p_camera` | 目标在相机坐标中的位置 | m |
| `R_camera2gimbal` | 相机坐标轴怎样旋转到云台坐标轴 | 无单位 |
| `t_camera2gimbal` | 相机原点在云台坐标中的位置 | m |
| `p_gimbal` | 目标在云台坐标中的位置 | m |

执行顺序是先旋转坐标表达，再加原点平移。不能把 `R` 和 `t` 的顺序随意交换。

## 7. 源码中的完整坐标链

`Solver` 构造时从 YAML 读取：

```text
camera_matrix
distort_coeffs
R_camera2gimbal
t_camera2gimbal
R_gimbal2imubody
```

主循环根据与图像时间对齐的 IMU 四元数调用：

```cpp
solver.set_R_gimbal2world(q);
```

随后 `Solver::solve()` 执行：

```cpp
cv::solvePnP(..., rvec, tvec, ...);

cv::cv2eigen(tvec, xyz_in_camera);
armor.xyz_in_gimbal = R_camera2gimbal_ * xyz_in_camera + t_camera2gimbal_;
armor.xyz_in_world = R_gimbal2world_ * armor.xyz_in_gimbal;
```

完整链路是：

```text
装甲板四个像素点
  -> solvePnP
xyz_in_camera
  -> 相机到云台外参 R/t
xyz_in_gimbal
  -> 图像时刻的云台/IMU 姿态
xyz_in_world
```

这里的 `world` 是参考项目内部用于目标估计的稳定坐标表达，不应未经确认就等同于当前 ROS2 系统中的 `map`、`odom` 或其他 TF frame。

## 8. 从真实配置看轴方向转换

参考机器 `standard4.yaml` 中的 `R_camera2gimbal` 大致接近：

```text
[ 0,  0,  1]
[-1,  0,  0]
[ 0, -1,  0]
```

忽略标定产生的小角度误差时，它表达的近似关系是：

```text
相机 +z 前方 -> 云台 +x 前方
相机 +x 右方 -> 云台 -y 方向
相机 +y 下方 -> 云台 -z 方向
```

将前面的例子作理想化转换：

```text
p_camera = (0.20, 0.10, 4.00) m
p_gimbal ≈ (4.00, -0.20, -0.10) m
```

参考项目的 `xyz2ypd()` 使用：

```cpp
yaw = atan2(y, x);
pitch = atan2(z, sqrt(x*x + y*y));
```

于是同一个“相机右下方目标”在这种云台轴定义中得到：

```text
yaw_gimbal   ≈ atan2(-0.20, 4.00) = -2.86°
pitch_gimbal ≈ atan2(-0.10, 4.005) = -1.43°
```

这不是前面算错了，而是正方向定义发生了变化：

```text
相机公式：向右定义为正 yaw
参考云台坐标：+y 近似指向左，因此向右对应负 yaw
```

结论是：角度正负号不能靠经验猜测，必须由坐标轴和下位机协议共同定义。

## 9. 内参和外参解决不同问题

| 参数 | 解决的问题 | 典型内容 |
|---|---|---|
| 相机内参 `K` | 像素与成像射线的关系 | `fx, fy, cx, cy` |
| 畸变参数 `D` | 镜头造成的弯曲 | 径向、切向畸变 |
| 相机到云台外参 `R/t` | 相机在机器人上的安装关系 | 旋转和平移 |
| 枪管偏差 | 发射轴与参考轴不完全重合 | 枪口位置、固定角偏差 |

完成相机内参标定，只能说明“相机如何成像”已经得到描述；它不会自动告诉程序相机相对云台和枪管装在哪里。

## 10. 为什么固定 yaw/pitch 偏置不能完全代替平移外参

假设相机光心与枪管轴横向相差 `0.10 m`。

目标距离 `1 m` 时，这个平移产生的视差约为：

```text
atan2(0.10, 1.00) ≈ 5.71°
```

目标距离 `5 m` 时：

```text
atan2(0.10, 5.00) ≈ 1.15°
```

同一个物理平移在不同距离产生不同角误差，而固定角度偏置不会随距离变化。因此近距离只调 `yaw_offset` 或 `pitch_offset`，可能在某个距离准确、换个距离又偏离。

参考配置中的 `t_camera2gimbal` 也不是零，进一步说明参考实现确实考虑了平移外参。但那些数值属于参考机器，不能复制。

## 11. 几何 pitch 与弹道 pitch 不同

几何 pitch 只表示目标视线方向：

```text
目标比枪口高多少？
```

弹道 pitch 还要回答：

```text
为了抵消飞行期间的重力下坠，枪口应额外抬高多少？
```

`Trajectory::pitch` 的注释规定“抬头为正”，但 `Aimer` 最终又执行：

```cpp
double pitch = -(current_traj.pitch + pitch_offset_);
```

源码注释说明参考项目发送给下位机的 world pitch 采用“向上为负”的命令约定。当前机器人必须与自己的下位机协议重新确认，不能照搬这个负号。

## 12. 常见错误

```text
只写 (x,y,z)，不写 frame_id 和单位
把相机内参当成相机到云台外参
直接用相机 yaw/pitch 控制云台
忽略图像时间戳，使用另一时刻的 IMU 姿态
把参考项目的 world 当成当前 ROS2 的 map
从电机表现反猜正负号，不查协议和坐标定义
用固定角偏置掩盖明显的平移视差
```

## 13. 本课核心结论

```text
像素点先通过 PnP 变成相机三维坐标
相机三维坐标必须通过 R/t 变成云台坐标
需要稳定目标估计时，再结合图像时刻姿态变到 world
yaw/pitch 的正负号由坐标轴和协议决定
内参、外参和枪管偏差是三个不同问题
```

## 14. 检查题

请在聊天框回答“第2题”：

> 目标相机坐标为 `(x_c=+0.20 m, y_c=+0.10 m, z_c=+4.00 m)`。按“相机 x 向右、y 向下、z 向前”的定义，它位于相机画面的哪个方向？相机几何 yaw 和 pitch 分别应为正还是负？若理想外参把它变成云台坐标 `(4.00, -0.20, -0.10) m`，云台坐标中的 `y` 和 `z` 为什么变成负数？

## 15. 下一课

第 3 课将从水平和竖直运动方程推导弹道补偿，解释源码中的判别式、两条抛物线和飞行时间。
