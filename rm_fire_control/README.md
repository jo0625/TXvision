# 上位机开火许可与串口发送控制

本包提供两种独立入口：原有预览入口只输出判断；新的控制入口把许可接入原生 Linux 串口发送。
电控协议保持原来的 11 字节角度包：按用户说明，收包触发自动发射，停包后电控超时退出自动发射。
本次不修改电控或 IMU 实现。

## 控制链路

`/controller_command_preview/command` + `/serial_bridge/feedback`
→ `fire_permission_control` → `/fire_control/decision` → 受控 `serial_bridge` → 电控。

许可消息携带被审核的角度、坐标帧和源时间戳，串口只发送同一组角度。
受控桥不订阅 `/serial_bridge/aim_command`，普通角度发布不能绕过许可。
新入口强制 require_fire_permission=true，仅支持原生 serial 和原来的 11 字节协议。
Windows 双工及延迟诊断追踪不支持此入口，会明确拒绝启动。
原有 serial_bridge.launch.py 为兼容旧功能保留原行为，不能把它当作受控发射入口。

## 启动

项目根目录构建：

```bash
source /opt/ros/humble/setup.bash
source install/local_setup.bash
colcon build --symlink-install --packages-select rm_serial_driver rm_fire_control
source install/local_setup.bash
ros2 launch rm_fire_control fire_control.launch.py port:=/dev/ttyACM0
```

默认 enabled=false、calibration_verified=true，启动后接收反馈但不发送角度。
新 launch 包含串口桥与许可节点；相机、跟踪、弹道解算、控制角转换需在上游运行。
一个串口只能由一个桥占用，使用新入口前应先结束旧串口桥，避免重复实例。

- 日常开关通过启动参数 `enabled:=true` / `enabled:=false` 选择，不需改代码。
- `calibration_verified:=true` 表示角度坐标约定及标定已确认，是单独的准入条件。
  2026-10-23 按用户确认沿用此前标定结果，已同步为 true；本轮没有重新实测方向和比例。
- 参数启动时加载；许可节点不支持运行时改参数，需重启并传入新值。
- 终止许可节点后，桥在许可过期后停止发送。终止整个 launch 会关闭串口。

只预览可用 `ros2 launch rm_fire_control fire_permission_preview.launch.py`。
预览消息 preview_only=true，受控桥拒绝使用它发射；原有正在运行的预览节点不会变成控制节点。

观察控制判断：

```bash
ros2 topic echo /fire_control/decision
ros2 topic echo /serial_bridge/command_active
```

## 开火条件与停止行为

enabled 和 calibration_verified 均为 true，目标角有效、数据新鲜、电控 mode=6、
帧一致且误差在阈值内，保持 0.15 s 并有至少 3 个不同时间戳的目标后，输出 allowed=true。
默认 yaw/pitch 容差各 1°，是初始设置，不是实测命中精度保证。
只使用上游有效目标和解算结论，不额外读取弹量、热量或摩擦轮就绪状态。

禁止、异常格式、乱序或预览消息会清除待发缓存；新许可必须对应更新的目标。
默认许可寿命 0.1 s、源目标最长 0.2 s，每次发送前同时核对源时钟与单调时钟。
桥还独立核对反馈新鲜度和模式，许可节点退出、仿真时钟暂停、反馈丢失或模式切换均会停包。
失效时清除本地重复发送缓存，并尝试清空原生串口输出缓冲。
已经送出的字节无法撤回；停止发包也不等于电控立刻停火。
实际停止延迟包含上位机检测/调度时间、在途数据和电控的停包超时，需实测确认。

该协议把自动瞄准和发射绑定：禁止发射时也不再更新云台目标。
只有已经满足对准条件时才发送，模块不会靠持续发送角度把偏离的云台拉到目标位置。
若需要自动转向且不发射，必须由电控提供独立的瞄准/开火控制协议。

## 测试

新增测试使用合成数据和 PTY 虚拟串口，验证准入、撤销、失联、错误模式、重放、缓存清理和旁路防护。
不打开 /dev/ttyACM0，不验证实机发射或电控停火超时。

## 完整链路一条命令

先用 Ctrl+C 结束之前的 fire_control.launch.py 或其他占用相机/串口的自瞄启动终端。
完整入口自带全部节点，不要和单独的串口桥同时运行。

```bash
source /opt/ros/humble/setup.bash
source /home/jo/Documents/Codex/rm_vision/rm_auto_aim/install/local_setup.bash
ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  ros2 launch rm_fire_control full_auto_aim.launch.py \
  port:=/dev/ttyACM0 enabled:=true projectile_speed:=14.7
```

该命令开启相机、YOLO、跟踪、目标超时检查、云台/枪口 TF、弹道补偿、控制角转换、
许可判断和唯一的受控串口桥。弹道补偿明确为 true，坐标使用 pitch_joint / odom / muzzle。
沿用用户确认的标定与测试外参。14.7 m/s 是用户采纳的落点初步等效速度，
依据见[测量记录](../rm_aimer/docs/impact_speed_estimate_2026-11-11.md)；
两组数据均来自 2 m 距离，尚未完成不同距离的补偿验证。
满足许可条件时会发送真实角度包，并按电控约定触发自动发射。
Ctrl+C 停止整条链路；任何节点退出也会触发整条链路退出。
想只看完整链路数据，把 enabled:=true 改为 enabled:=false。
参数 start_camera/start_detector 可用于接入回放数据，正常实机使用保持默认 true。

## 每次运行的诊断记录

完整入口默认 `record_run:=true`，每次启动自动创建独立目录：
`<project_root>/test_results/aim_runs/<北京时间>_<唯一标识>/`。
终端显示 `Run log directory:`，目录中保存：

- `events.jsonl`：原始装甲板、原始/受保护目标的 tracking、位置、速度和半径；
  弹道有效/无效状态和原因、选中装甲板、枪口到预测目标的距离、飞行时间、
  重力坐标系中的直瞄俯仰/弹道俯仰及两者差值；
  电控角度转换结果、精确匹配的反馈时间戳及当时的零位/比例；
  电控反馈、许可判断及拒绝原因、串口连接/反馈存活/发送状态；
  相机内参、动态/静态 TF 和参数变化。
- `summary.json`：各话题记录数、有效/无效解算数量、许可原因计数、
  诊断序号缺口、日志容量/错误及通过 ROS 参数服务读取的实际运行参数。

`/serial_bridge/tx_event` 的 `host_write_returned` 包含实际 11 字节包中的
角度、原始十六进制、源指令/许可时间戳，只在原生串口完整写入返回后发布。
这是上位机写入证据，不能代表 MCU 已执行或已经停火。
`command_active` 是状态；发送次数以逐次 `tx_event` 为依据。

记录器是独立订阅节点，每秒刷新文件；Ctrl+C 后保存最终汇总。
使用 best-effort 接收高频数据，诊断事件序号可发现启动漏收或中途缺口，
记录数不应当作系统绝对无丢帧的证明。非有限数保留为字符串 nan/inf/-inf。
每次 JSONL 默认上限 512 MiB；达到上限或写盘失败会明确报错并停止记录，
不会更改发送门控。正常退出时 `closed=true`；强制终止可能留下最后一次刷新后的
尾部缺失，查看 `log_error`、`dropped_counts` 和 `diagnostic_sequence_gaps` 判断完整性。

可用 `run_log_root:=/绝对路径`、`run_log_max_mb:=1024` 调整目录和容量，
或 `record_run:=false` 关闭。本入口不录制图像，日志目录应定期清理或归档。
单独启动许可入口不会自动记录完整链路。

排查弹着点固定偏移时，按源时间戳关联解算、转换、许可和发送事件，再与后续反馈比较。
直瞄角与弹道角的差在同一个重力坐标系计算，不能直接拿电控原始 pitch
减去相机视线 pitch 当作补偿量。反馈追上指令也不能证明枪口光轴和相机外参正确。
2026-10-25 用户确认靶距约 2 m，弹着点重复偏左下、距装甲板中心约 15–20 cm。
按目标平面近似垂直视线估算，合成方向误差约 4.3–5.7°；水平/竖直分量尚未分别测量，
不能给两个轴各加这个角度。该观察未被用于修改零位、外参、弹速或发射阈值。
