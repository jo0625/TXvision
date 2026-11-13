# EKF R 可执行测试方案

工作分支：`codex/test-handeye-20261110`。本次希望减小运动跟随滞后。
当前候选只由静止数据支持，尚未验证动态效果。

## 1. 当前值与对照值

修改本分支 `auto_aim_bringup/params/tracker_params.yaml` 的 `ekf` 三项，其他参数保持一致：

| 组别 | r_xyz_factor | r_xyz_min（m²） | r_yaw（rad²） |
|---|---:|---:|---:|
| A：之前运行的 R | 0.05 | 0.0001 | 0.02 |
| B：当前已写入候选 | 0.0 | 0.000001 | 0.00005 |

B 的每轴位置标准差为 1 mm，装甲板朝向 yaw 标准差约 0.405°。
`r_yaw` 是测得的装甲板朝向噪声，并非云台 yaw 编码器噪声。
三段静止数据中最大位置方差约 8.54e-7 m²，最大 yaw 方差约 4.87e-5 rad²，因此取上述候选。
测距起点继续为枪口，枪口到装甲板约 2.1 m；PnP 相机距离约 1.93 m，不能直接混用。
原静止统计使用主工作区 TF，本实验用候选手眼外参，先复核当前外参下的静止表现。
原始依据：`/home/jo/Documents/Codex/rm_vision/outputs/ekf_r_20261112/comparison_20261112_142019.md`。

每次改 R 后完整停止并重新启动实验入口。节点启动时缓存 R；`ros2 param set` 不能代替重启。
沿用本实验分支外参、弹速 14.7 m/s、Q、相机曝光与检测配置进行全部 A/B 测试。
`prediction_time=0` 表示额外预测时间为零；瞄准器仍会做弹道飞行时间预测。

## 2. 配置检查（不启动硬件）

在一个终端设置路径：

```bash
R_TEST_DIR=/home/jo/Documents/Codex/rm_vision/outputs/handeye_branch_test_20261111/rm_auto_aim/tools/full_aim_handeye_test
R_DEPENDENCY_DIR=/home/jo/Documents/Codex/2026-10-15/pid-sdk-pnp-11-usb-sdk
bash "$R_TEST_DIR/start_full_aim.sh" --check
```

检查输出的 Tracker 参数文件位于实验分支，弹速为 14.7，三项 R 与本轮组别一致。
此检查只确认配置和现有 TF 库一致性，不代表已通过硬件测试。

## 3. 先做一次 30 秒只接收静止复核

先在旧链路的启动终端按 Ctrl+C，等待清理完成；采样时保留的独立串口节点也应结束，避免两个进程同时占用串口。
保持云台和同一块装甲板静止，枪口距离约 2.1 m，板朝向、照明和曝光保持一致。
以下命令会启动串口只接收、相机、检测、TF 和滤波观察，不启动跟随转发：

```bash
R_STATIC_RUN="/home/jo/Documents/Codex/rm_vision/outputs/handeye_branch_test_20261111/runs/r_static_B_$(date +%Y%m%d_%H%M%S)"
bash "$R_DEPENDENCY_DIR/outputs/tf_deadlock_fix/with_fix.sh" \
  /usr/bin/env PYTHONDONTWRITEBYTECODE=1 ROS_LOG_DIR="$R_STATIC_RUN/ros_logs" \
  /usr/bin/python3 -B "$R_TEST_DIR/capture.py" \
  --output "$R_STATIC_RUN" \
  --expected-tf2 "$R_DEPENDENCY_DIR/work/tf_deadlock/install/tf2/lib/libtf2.so" \
  --warmup 7 --duration 30 --receive-only --prediction-time 0.0 --boundaries
```

目标：复核有效检测持续、没有目标身份切换，并检查静止瞄准指令的波动。
需要统计 R 时，使用原始检测通过图像时间戳对应 TF 转到 odom 后的方差，不能拿滤波后输出的方差当 R。
这一轮能记录新外参下的数据；脚本本身不自动生成噪声或延迟分析报告。

## 4. 同一动作做 A → B → A 对照

先做上面的 B 静止复核，再按表把 R 改为 A 并检查；每轮记录组别与终端打印的输出目录。
后续依次改 B、改 A，均重新启动。三轮结束后把配置恢复 B 或根据结果选定的值。

```bash
bash "$R_TEST_DIR/start_full_aim.sh" --check
bash "$R_TEST_DIR/start_full_aim.sh"
```

第二条会启动真实视觉跟随：每阶段 7 秒预热，15 秒只接收预检，通过后 60 秒跟随。
预检期间保持目标静止，进入跟随阶段后再按同一节奏做下面的动作。
入口会检查已有链路进程及视觉模式反馈；若报错，按日志解决原因为准。

以每轮进入 60 秒跟随窗口为起点，三组使用相同的移动路线：

1. 0–10 秒：目标静止，观察指令与云台抖动。
2. 10–30 秒：保持板朝向，沿横向两个标记之间匀速往返，速度和距离每轮一致。
3. 30–45 秒：用同样路线做启停、换向，观察起步拖尾和换向过冲。
4. 45–60 秒：停在起始标记，观察是否继续摆动及重新稳定的过程。

沿用现有跟随范围；测试目标放在三组都能持续识别的位置。
Ctrl+C 停止，等待脚本清理退出后再启动下一组；无需重编译运行程序。
不要边跑边修改 YAML，也不要用主工作区旧外参入口作为 A 组。

在另一个已 source ROS 与主工作区 install/setup.bash 的终端，可以在节点运行期间核实：
```bash
ROS_DOMAIN_ID=198 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_fastrtps_cpp ros2 param get /armor_tracker ekf.r_xyz_factor
ROS_DOMAIN_ID=198 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_fastrtps_cpp ros2 param get /armor_tracker ekf.r_xyz_min
ROS_DOMAIN_ID=198 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_fastrtps_cpp ros2 param get /armor_tracker ekf.r_yaw
```

## 5. 记录与判断

每轮目录在 `/home/jo/Documents/Codex/rm_vision/outputs/handeye_branch_test_20261111/runs/`，终端会打印具体路径：

- `motion_01/capture.json`、`launch_commands.json` 和 `configuration/`：核实实际参数路径、值、覆盖项与完成状态。
- `motion_01/pipeline.csv`：原始检测、TF、滤波目标、瞄准指令、转发指令和云台反馈。
- `motion_01/tracker.csv`、`aimer.csv`、`serial.csv`：定位链路各阶段时间；不把这些文件的存在等同于已完成延迟分析。
- `preflight_01/`：本轮相同配置下的静止预检记录。

三组至少比较：静止指令角度标准差、运动时检测到滤波的相位滞后、指令到反馈的滞后/误差、丢失与重捕获次数。
跨话题用图像时间戳关联，时间延迟使用统一的主机时间基准；不要用消息到达先后直接推断滤波相位。
滤波目标 `position` 是目标中心，检测 `position` 是装甲板位置，且原始检测在相机系：
比较前必须先做同时间戳 TF 转换，并用滤波中心、半径和朝向还原同一块板的位置。单板时对应
`x_a = x_c - r*cos(yaw)`、`y_a = y_c - r*sin(yaw)`，z 使用状态中的装甲板高度，不能直接相减两个 position。

- B 相比前后两次 A 都减小拖尾，且静止波动与换向摆动可接受：保留 B，再到实际使用距离重复静止和运动测试。
- 检测本身在拖尾：检查曝光、帧时间戳和处理延迟；继续减 R 不能补回已经丢掉的时间。
- 滤波已跟上，但指令或云台仍滞后：分别查看瞄准预测、串口发送和下位机响应，不能把这部分都归因于 R。
- B 抖动明显：先用该距离、板朝向和实际动态条件重新估计测量噪声，判断是位置还是朝向噪声；一次只调整相关项，再重复同动作。
- B 仍有滤波拖尾：根据动态残差判断运动模型/Q，后续单独测试；此轮不同时修改 Q 和 R。

目前没有动态实测支持某个毫秒阈值或通用最佳 R，因此按三轮实测对照判断，不指定凭空的合格门限。
