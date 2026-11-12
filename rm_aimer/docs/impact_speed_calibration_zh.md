# 实测落点反推等效弹速

保持摩擦轮档位不变，用多发真实落点拟合重力模型的等效弹速，再用该速度重新计算云台补偿。
模型不会控制摩擦轮，也不会采集或触发发射。反推结果不是测速仪读数，不能区分阻力、角度零点和测量偏差。

先固定相机和发射机构，复核外参与物理发射角参考。采集期间保持同一档位和安装状态。
旧手眼图像、装甲板目标位置及画面中的预测击打点均不是实际弹丸落点，不能作为本工具的测速样本。
2026-11-11 已根据用户提供的两组落点采用初步等效弹速 14.7 m/s，记录见
[本次估计依据](impact_speed_estimate_2026-11-11.md)。配置来源为 `impact_estimate`；
当前未生成本工具的标定 JSON，`speed_calibration_file` 保持为空。

## 每发记录

| CSV 字段 | 含义 |
| --- | --- |
| shot_id | 唯一发次编号 |
| gear | 摩擦轮档位标识；同一标定文件只能包含一种档位 |
| horizontal_distance_m | 发射出口到实际落点的水平距离，m；不能用斜距替代 |
| launch_pitch_deg | 实际初始发射方向相对水平面的仰角，度；抬头为正，不是俯仰电机原始编码角或期望角 |
| impact_height_m | 实际落点相对发射出口的高度，m；落点低于出口时为负 |

实际落点所在平面若不垂直于发射方向，应记录对应落点的真实水平距离。
发射角应独立测量，不能使用由假定弹速算出的补偿角代替实际角度。
至少需要 3 发独立记录，建议覆盖不同距离，用以检查同一个等效速度是否适用。
短距离的重力下坠较小，角度或高度的测量误差会显著影响反推速度；输出小残差不等于绝对精度高。

## 创建模板与拟合

从 `rm_auto_aim` 目录运行（脚本本身不需要 ROS）：

```bash
python3 rm_aimer/scripts/impact_speed_calibration.py \
  --template ../outputs/impact_speed_calibration/measurements.csv
```

填写真实记录后：

```bash
python3 rm_aimer/scripts/impact_speed_calibration.py \
  --input ../outputs/impact_speed_calibration/measurements.csv \
  --output ../outputs/impact_speed_calibration/calibration.json
```

输入单位是米和度。重力默认 9.81，可用 `--gravity` 指定，但必须和后续 aimer 一致。
工具保留全部样本，报告每发反推速度、相对初始直线的下坠量、整体高度残差和速度离散度。
同档位各发速度极差超过拟合速度 10% 时给出排查提示，这个阈值不是精度合格标准。
结果包含原始测量、CSV SHA-256 和拟合时间，保留数据来源。已有输出文件不会被覆盖。

模型为 z = r*tan(theta) - g*r^2/(2*v^2*cos(theta)^2)。
在多发数据上最小化高度残差，拟合 1/v^2；不包含空气阻力或外参零点修正。

## 用于弹道解算

在原有参数之后添加标定路径，例如：

```bash
ros2 run rm_aimer basic_aimer_node.py --ros-args \
  --params-file rm_aimer/config/basic_aimer.yaml \
  --params-file rm_aimer/config/controller_preview.yaml \
  -p speed_calibration_file:=/绝对路径/calibration.json
```

或使用专用启动入口：

```bash
ros2 launch rm_aimer ballistic_aimer.launch.py muzzle_frame:=muzzle \
  speed_calibration_file:=/绝对路径/calibration.json
```

实时画面预览：

```bash
bash tools/run_ballistic_preview.sh --speed-calibration /绝对路径/calibration.json
```

`speed_calibration_file` 非空时优先使用文件反推的速度。启动时重新校验记录并计算拟合值；
缺文件、格式错误、记录不足或重力不一致会使节点报错，不会静默回退到估算速度。
标定文件为空时继续使用配置的固定弹速。参数和标定文件在节点启动时读取，更改后需重启。
`/aimer/solution` 提供速度来源 `impact_fit`、档位与测量文件摘要，预览同步显示弹速来源。
换档、安装变动或实际速度特性变化后，应使用对应的新测量结果；当前串口没有档位或测速反馈，不能自动判断这些变化。
