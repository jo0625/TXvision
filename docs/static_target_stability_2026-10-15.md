# 静止装甲板稳定性：测量路径与复现

本次仅加入默认关闭的日志及采集/离线分析工具。没有修改检测、角点提取、PnP 求解及选择、类型锁定、跟踪滤波、控制频率或 PID。运行时关闭调试图像显示，并调低既有诊断打印频率，这两个变化只影响日志/显示负载。结果属于该记录条件，不声称与打开 RViz 的性能等同。

## 真实路径及单位

1. `hik_camera/src/hik_camera_node.cpp`：`MV_CC_GetImageBuffer` 得到 SDK 的 `nFrameNum`；原图像 `header.stamp` 是主机 ROS 时钟，不是相机曝光时间。新增 `stability_trace_path` 在内存中保存 SDK 帧号、主机单调 ns、原 stamp、图像大小、像素格式和释放状态，结束后写 CSV；通过相同原 stamp 关联后续帧，不改消息协议。SDK 帧号按 uint32 回绕处理。
2. `yolo_armor_detector_node.py:image_callback`：YOLO 产生框、置信度与类别。当前 frozen candidate 的 `normalized_dual_pnp_latch` 在检测框 ROI 中，用现有灰度阈值 140 提取灯条；实际 PnP 使用这些灯条端点，不是直接把 YOLO 框或网络关键点用于 PnP。网络语义/灯条角点顺序 LT、LB、RB、RT；输入 `cv2.solvePnP(..., SOLVEPNP_IPPE)` 的实际顺序是 **LB、LT、RT、RB**。坐标为原始 1440×1080 图像像素；未用显示缩放后的点。
3. `raw_track_latch.py:solve_light_pnp/process_frame`：首次识别尺寸时，原程序实际评估大小两种 PnP，再锁定类型，后续使用锁定的尺寸。日志同时保留原程序已经计算的候选假设；不会为日志额外求解 PnP。small 为 0.132×0.057 m，large 为 0.223×0.057 m。物点在 x=0 的 yz 平面，PnP 的 rvec 为物体到相机的 Rodrigues 旋转向量（rad），**不是欧拉 yaw/pitch/roll**；tvec 为 camera_optical_frame 的 x右、y下、z前，单位 m。重投影误差是四点欧氏距离均值，单位 px。
4. `armor_from_raw_result`：原 PnP 位姿变成 `/detector/armors`，姿态为四元数，位置仍为光学坐标。`armor.number` 当前为配置字符串 `armor`，不是网络识别的数字编号。日志的 `detector_track_id` 是框 IoU 关联 ID，不是永久物理目标 ID；硬件身份不可用，遮挡后重建或同处换板无法仅靠这个 ID 证明同一物理目标。类别以模型整数类别记录，置信度为 YOLO 框置信度。
5. `armor_tracker/src/tracker_node.cpp:armorsCallback`：在图像 stamp 对应的 TF 上把位姿转到 odom，再运行原有跟踪器/EKF；`/tracker/raw_target` 的位置是机器人模型中心和半径状态，**不等于相机原始装甲板位置**。Target.yaw 是模型姿态角（rad），不是瞄准方向 yaw。消息没有检测框置信度或检测 IoU track ID；多有效候选时，离线分析不冒充已经知道选中了哪块检测装甲板。
6. `rm_aimer/scripts/basic_aimer_node.py:target_callback`：从跟踪状态重建各装甲板位置，按既有 TF 转入 aim_frame，选择最近有效前方点，计算 `atan2(y,x)` 与 `atan2(z,hypot(x,y))`。基础配置为 gimbal_link；既有 controller_preview.yaml 覆盖为 **pitch_joint**。新增日志记录实际选中的重建索引、所有转换点、距离、实际 yaw/pitch（rad）与源 stamp。
7. `controller_command_preview.py`：匹配现有反馈缓存中的实际反馈样本，将 pitch_joint 的几何视线映射为 `controller_joint_targets_preview`。使用原有 pitch offset 0.319451511 rad、scale 1.023925824751 及 yaw sign；日志保留真正被选中的反馈序号、ROS stamp、收到反馈的单调时间、yaw/pitch、转换前后角度和失败原因。不用观察者收到的“最新反馈”冒充实际匹配样本。这套转换仍是既有 provisional 标定，不能当作下位机固件行为已经核实。
8. 正常跟随时，`tools/dynamic_follow_test.py` 还会做既有有效性检查、可选角度门限和死区保持，并发布 `/serial_bridge/aim_command`。死区可能保留旧角度但使用新输入帧的 header，因而 header 只说明本次处理的输入帧，不能证明持有角度最初来自该帧。本次**不运行该运动转发程序**。
9. `serial_bridge_node.py:command_callback/write_command`：打包为原有 11 字节 `A5 + float32 yaw + float32 pitch + 5A5B`，小端，rad。既有可选 latency CSV 记录源帧、源角度、发送调用时间、重复发送/覆盖/过期/异常。本次增加源坐标系和角度、打包后原始 wire bytes；分析实际发送时解码 float32，不把源 float64 直接冒充线上字节。发送接口返回只代表主机侧提交，不证明下位机接收或采用。

## 测量边界

所有内部时间使用 Linux CLOCK_MONOTONIC / Python monotonic_ns，单位 ns；Python 报告的名义分辨率 1 ns 不代表测量准确度 1 ns。不同组件只有在 Linux boot ID 和 time namespace 一致时才关联单调时间窗口。相机 ROS header 只作帧身份；不与单调时间或 MCU 时间相减。此测试不是 USB 延迟测试。

真实相机配置从 SDK 在起止各读取一次：曝光/增益及其自动模式、ROI Width/Height/OffsetX/OffsetY、PixelFormat、设定/实际帧率及触发模式。CSV 保留 getter 返回码。起止相同只证明两次快照一致，不证明两次之间绝无外部修改；测试期间不运行其他相机配置工具。逐帧保留实际尺寸/像素格式和 SDK 帧号。

每个 Python 组件使用有容量上限的内存日志，记录丢弃数；相机最多记录 20,000 帧，默认采集 15+3×30 秒在 100 FPS 下约 10,500 帧。低开销不等于零扰动；内存日志及少量图像引用会增加内存占用。图像示例保留的是已经拥有独立生命周期的解码数组，最多为三类变化各两组前后帧，测试结束才编码/写图；不会保存已释放 SDK 指针，也不会在计时循环写图。示例仅按最大相邻变化排名选取，不把排名当成异常门限。

未检测帧、无效候选、PnP 失败、身份集合变化、失效瞄准、无法关联、帧缺口与日志丢弃都保留。统计按试次、IoU ID、类别、尺寸、坐标系及重建装甲板索引分开；`statistics_by_identity.csv` 给同身份每试次分布，`statistics.csv` 另分连续段。相邻变化只使用连续 SDK 帧/检测帧，角度按 ±π 最短角差展开。Rodrigues 向量分量不作欧拉角回绕；离线另提供相机坐标下装甲板法线的方向角，和瞄准射线角严格区分。

本项目现有距离/姿态合法性门限、0.6 rad 历史运动保护门限及跟随死区不能替代瞄准稳定性验收要求。目前没有找到经确认的静止像素/角度波动验收阈值，因此只报告实测幅度，**是否满足控制要求尚缺验收阈值**。

## 安全采集与复现

用户已确认相机与点亮装甲板固定、机械跟随安全暂停。工具只使用现有 `receive_only:=true`，不创建运动转发器、不新增运动命令。独立 ROS domain 198 避免把测量目标送入原有控制域。只停止本工具自己启动的进程。工具退出后所有子进程退出，不自动恢复机械跟随。

```bash
cd /home/jo/Documents/Codex/rm_vision/rm_auto_aim
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --symlink-install --packages-select hik_camera yolo_armor_detector rm_aimer rm_serial_driver
/home/jo/Documents/Codex/rm_vision/.venv/bin/python tools/capture_static_target.py \
  --output /home/jo/Documents/Codex/rm_vision/rm_auto_aim/test_results/static_target_NEW \
  --stationary-confirmed --duration 30 --repeats 3 --warmup 15
/home/jo/Documents/Codex/rm_vision/.venv/bin/python tools/analyze_static_target.py \
  test_results/static_target_NEW
```

先检查硬件当前没有其他进程占用。串口 `/dev/ttyACM0` 需要当前用户具备现有访问权限；没有权限或反馈时，只能测相机/角点/PnP，不能补造 odom、pitch_joint 或控制器目标。不要为补齐曲线填入伪造的零姿态 TF。

实际发送角度与发送间隔如果没有任何事件，会生成带表头的 send_events.csv 和注明未测的曲线面板；不能把零条事件解释为“发送角度稳定”。如以后在已获授权且机械能保持静止的真实原有发送流程中启用 bridge latency_trace_path，可离线解码其 wire bytes；仍需核实死区持有时的角度原始来源。不得为了本次记录开启新运动发送。

原始 CSV 和配置快照由采集保存，离线分析写新目录；已有不完整采集和分析也保留。原始 detector/pipeline/camera/aimer/preview/serial CSV 不由分析器改写。统计是本机本场景的观测，不是模型精确率/召回率评估。
