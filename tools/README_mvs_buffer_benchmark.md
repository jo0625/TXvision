# MVS SDK 缓冲节点基准

该工具直接复用 `hik_camera` 节点使用的 MVS C API，对 MV-CS016-10UC 的 SDK
图像缓存节点数进行可重复测试。它不会修改现有 ROS 节点。

## 正式运行

关闭 MVS 客户端和所有占用相机的节点，在能够访问 USB 相机的宿主终端执行：

```bash
cd /home/jo/Documents/Codex/rm_vision/rm_auto_aim
tools/run_mvs_buffer_benchmark.sh
```

默认测试 2、4、8、16、32、64 节点。每组包含 2 秒预热、60 秒连续取帧、
60 秒模拟负载，以及 20/50/100/250 ms 各 3 次暂停和恢复，整轮约 14 分钟。
转换负载和暂停基线分别另有 2 秒预热，不计入正式阶段。
模拟负载对真实图像同步执行现有相机节点的 BayerRG8 到 BGR8 转换和一次消息大小
的缓冲拷贝，不包含 ROS/DDS 发布和 YOLO 推理。

固定配置为：

- 1440×1080，OffsetX/OffsetY 均为 0
- BayerRG8
- 100 FPS
- 曝光 3000 µs，增益 12 dB，关闭自动曝光和自动增益
- OneByOne

相机原设置在打开后保存，正常结束、异常或 SIGINT/SIGTERM 退出时均尝试恢复。

可指定输出目录：

```bash
tools/run_mvs_buffer_benchmark.sh /home/jo/camera_buffer_results
```

`--allow-short` 只用于开发验证；分析报告会把不足 60 秒或不足 3 次暂停的结果标为
不符合正式协议。

## 输出

- `per_frame.csv`：逐帧数据和校准后的相机时间戳到应用取帧延迟。
- `configuration_comparison.csv`：节点配置主对比表。
- `stage_summary.csv`：连续、负载和每次暂停恢复阶段的统计。
- `pause_repetitions.csv`：每次暂停的首帧年龄、排空时间、正常延迟恢复时间和丢帧。
- `configuration.csv`：原始设置、固定后实际读回设置和测试参数。
- `clock_calibration.csv`：每组前后的相机时钟校准原始数据。
- `runtime_memory_usb.csv`：RSS 和 USB 接收/错误计数。
- `summary.json`：完整机器可读结果。
- `report.md`：配置、结果和 A/B/C 结论。
- `restoration_status.txt`：退出时恢复相机设置并读回核对的结果。

采集循环不显示图像、不打印、不写磁盘。成功取得的每一帧在同步处理完成后调用
`MV_CC_FreeImageBuffer`；暂停期间不调用 `MV_CC_ClearImageBuffer`。逐帧记录先保存在
内存中，停止该节点配置后才写盘。

## 指标边界

SDK 队列深度来自每次 `MV_CC_GetImageBuffer` 后立即调用的
`MV_CC_GetValidImageNum`，表示队列中剩余有效图像数，不包含当前借出的帧。

相机帧时间戳通过每组前后各 20 次 `DeviceTimestampLatch` 映射到
`CLOCK_MONOTONIC`，使用前后锁存拟合时间比例和偏移，不假定固件时间戳的单位。
误差估计包含锁存残差和半往返时间，不覆盖未观测的非线性漂移。
延迟包含时间戳在相机内的位置、USB、SDK 缓冲和主机调度，
不是纯 USB 时间。MVS 的主机时间戳原值也会保存，但只有毫秒级精度，分析器以
1 ms 作为跨配置收益判定的最低分辨下限，同时检查各正常阶段按 10 秒分箱的延迟中位数变化。
边界锁存误差不能约束整段运行中的非线性时钟漂移；若阶段内变化超过下限，不能仅凭跨配置
延迟差超过 1 ms 宣称减少节点有效。逐帧 CSV 另存 SDK 主机标记至应用返回的辅助指标
`sdk_host_to_app_ms`，该指标也不代表完整相机延迟或纯队列等待。

暂停恢复使用暂停前最近 100 个队列为空帧的局部 p95 基线。
`first_normal_latency_ms` 为首次低于基线加测量下限的时间，
`normal_latency_recovery_ms` 为连续 5 帧满足条件的确认时间，不能把约 40 ms 的五帧观察时间
误称为排空积压需要 40 ms；排空时间另见 `backlog_drain_ms`。

`runtime_memory_usb.csv` 的 before 计数在 StartGrabbing 前读取，可能属于上一取流会话。
不要直接用 after − before。分析器核对 after 是否等于全部原始记录（含预热）加帧号缺口，
一致时报告本次会话计数，否则仅保留原值并标记计数范围未验证。

已完成的正式运行：`test_results/mvs_buffer_nodes_20261015_122011/report.md`。
本轮使用模拟转换/拷贝负载，不能替代完整装甲板识别程序的最坏负载验证。

现有测试程序 `/home/jo/Documents/Codex/2026-10-13/ban/outputs/camera_usb_sdk_bench.py`
使用 `--nodes`，默认值为 16，并显式调用 `MV_CC_SetImageNodeNum`。ROS 相机节点没有
显式设置该值；二者不能混为一谈。本工具对每组值都显式设置，不依赖 SDK 默认值。
