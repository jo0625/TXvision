# RM Vision 弹道补偿接入说明

## 当前代码

弹道补偿已接入 `rm_aimer` 的目标回调。核心模块是
`scripts/ballistic_solver.py`，采用与 SP Vision `tools::Trajectory`
相同的无空气阻力、恒定重力低抛模型，返回抬头为正的俯仰角（rad）与飞行时间（s）。
默认重力为 9.81；若要对照提取的 SP Vision 原版，可设为 9.7833。
上游 MIT 版权说明保留在 `LICENSE.sp_vision`。

本次检查时，工程已经包含弹道求解、坐标转换、飞行时间迭代和发射点 TF 支持。
本次将求解器整理为独立模块，增加专用启动入口、测试和本说明。
原 `aimer_geometry.solve_ballistic_pitch` 导入路径仍可使用。

```text
/tracker/target
  → 重建装甲板位置、选择目标
  → 转到重力坐标系，减去发射点位置
  → 求低抛俯仰角和飞行时间
  → 按固定提前量 + 飞行时间预测目标，再次解算
  → 把发射方向转回 aim_frame
  → /aimer/aim_command
```

最多迭代 10 次，飞行时间变化小于 1 ms 时结束。无解、TF 缺失或不收敛时输出
`valid=false`。原几何模式继续通过 `basic_aimer.launch.py` 使用。

## 启用前需要填写的参数

| 参数 | 含义 |
| --- | --- |
| `projectile_speed` | 配置弹速，m/s，必须大于零；当前 14.7 为用户采纳的落点初步估计 |
| `muzzle_frame` | 真实发射出口的 TF 名称；节点不会凭空创建此坐标系 |
| `gravity_frame` | 默认 `odom`，z 轴必须与重力反方向对齐 |
| `aim_frame` | 专用启动默认 `pitch_joint`，与现有控制器预览接口对应 |
| `prediction_time` | 额外固定提前量，s；默认 0，不自动从消息时间戳推算延迟 |

配置 `rm_serial_driver/config/gimbal_tf.yaml` 中的 `muzzle_frame`
以及 `pitch_to_muzzle_x/y/z`，填写实测的发射出口相对 `pitch_joint`
原点的位移（m，x 向前、y 向左、z 向上）。原有 `gimbal_tf_broadcaster.py`
会在启用这个 frame 后发布对应静态变换。若使用其他 TF 配置文件，应修改实际加载的那份。

`pitch_joint` 随云台转动，它的 z 轴不能直接当作世界竖直方向。节点在
`gravity_frame` 中求解，再把方向向量转回 `aim_frame`；这里不照搬 SP Vision
下位机接口中的 pitch 负号。现有控制器预览仍负责控制器角度约定的转换。

## 构建与启动

在项目目录执行：

```bash
cd /home/jo/Documents/Codex/rm_vision/rm_auto_aim
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select rm_aimer
source install/local_setup.bash
ros2 launch rm_aimer ballistic_aimer.launch.py --show-args
```

确认跟踪器和 TF 已运行。当前采用用户于 2026-11-11 确认的等效弹速 14.7 m/s；
两组落点及角度依据见[本次估计记录](impact_speed_estimate_2026-11-11.md)：

```bash
ros2 launch rm_aimer ballistic_aimer.launch.py \
  projectile_speed:=14.7 muzzle_frame:=muzzle
```

14.7 m/s 对应本次采样的工作条件，具体摩擦轮档位没有记录。历史档位表中
5980 rpm 约 19 m/s、6580 rpm 约 23 m/s 均为估算；换档后应使用对应条件的数据。
视觉端目前不会读取摩擦轮转速或自动跟随换档。
专用启动入口仍要求显式传入弹速，启动参数优先于 YAML 中的值。

这个启动文件启动一个 `aimer` 节点；不要与原几何 aimer 同时向同一话题发布。
它只发布瞄准结果。现有串口转发和控制器预览需要按项目原有方式单独接入。
需要对照时，可以指定 `output_topic:=/aimer/ballistic_preview`，避免两种结果混在一起。

查看结果：

```bash
ros2 topic echo /aimer/aim_command
```

默认配置已填写 `muzzle_frame: muzzle`、`projectile_speed: 14.7` 和
`projectile_speed_source: impact_estimate`，当前 YAML 的 `ballistics_enabled` 为 true。
整机启动入口的默认弹速同步为 14.7；专用 `ballistic_aimer.launch.py` 仍需显式传入弹速
或标定文件。节点启动时读取配置，更改后需重启对应节点。

## 验证与模型范围

单元测试用物理运动方程回代验证水平、侧向和竖直落点，覆盖高低目标、最大射程
和无效参数。现有 ROS 节点测试覆盖几何输出、移动目标预测、不可达目标，以及
带发射点偏移和俯仰旋转的坐标变换。

```bash
colcon test --packages-select rm_aimer --event-handlers console_direct+
colcon test-result --verbose
```

模型没有空气阻力、风或弹速动态反馈。使用配置的固定弹速和匀速目标预测；
固定提前量不会自动补偿处理耗时。TF 取目标时间戳处的姿态，模型没有预测后续发射点运动，
也没有联立求解云台转动引起的出口位置变化。这些限制需要在实际标定和后续扩展时考虑。

## 2026-10-23 用户提供的参数

- 发射机构水平时，出口离地 38 cm，pitch 轴心离地约 18 cm。
- 出口相对 pitch 轴心：向前 2.5 cm、左右居中、向上约 20 cm。
- 已同时更新 `gimbal_tf.yaml` 和整机脚本实际加载的 `gimbal_tf_provisional_handeye.yaml`，两份均发布 `pitch_joint -> muzzle`。
- 弹速来源为用户提供的摩擦轮档位表图片，单位为电机输出轴 rpm。默认档 5980 rpm 对应注释估算约 19 m/s；档位 1 的 6580 rpm 对应注释估算约 23 m/s。不能把 rpm 直接填入 `projectile_speed`，也没有建立实测转速—弹速模型。
- 位置参数是用户近似测量值；俯仰零点、相机外参和原 `yaw_to_pitch_z: 0.178` 未调整。后者仍需核对是轴间距离还是当初记录的离地高度。
- 本次仅更新磁盘配置，没有重启运行节点。节点在启动时读取参数，运行中的配置不会自动刷新。
