# yolo模型训练承接版_3_wsl

更新时间：2026-08-26（Asia/Shanghai）

用途：将当前监督对话迁移到新 Codex 对话。新对话应先完整读取本文，再决定是否读取更早的历史文档或操作端会话。本文记录的是当前有效状态，不是新的实验授权。

## 1. 新对话入口

工作区：

```text
/home/jo/my-project/rm_vision/rm_auto_aim
```

完整历史交接：

```text
/home/jo/my-project/rm_vision/rm_auto_aim/docs/handoff.md
```

操作端会话 ID：

```text
019fae48-36da-7c53-b009-da3431743c8e
```

操作端会话文件：

```text
/home/jo/.codex/sessions/2026/07/29/rollout-2026-07-29T22-29-56-019fae48-36da-7c53-b009-da3431743c8e.jsonl
```

ROS 环境：

```bash
cd /home/jo/my-project/rm_vision/rm_auto_aim
source install/setup.bash
```

当前对话承担监督端职责：读取操作端最新进展，判断建议是否合理，再生成下一步提示词。除非用户明确要求，不应代替操作端重复进行实机实验。

## 2. 当前一句话结论

冻结候选模型配合 `normalized_dual_pnp_latch` 和实例级 `gray_threshold=140`，已经通过真正左 45 度离线回放以及正面、左 45 度、右 45 度、返回正面的单次连续实机验收。当前识别阶段已通过，下一步是修正一处报告模板残留错误，并利用已有 5176 帧建立输出延迟基线；不需要重新拍摄装甲板。

## 3. 当前冻结候选配置

候选参数文件：

```text
/home/jo/my-project/rm_vision/rm_auto_aim/auto_aim_bringup/params/yolo_detector_params_v8_frozen_candidate.yaml
```

候选模型：

```text
/home/jo/my-project/my_first_autoaim_yolo/models/frozen/armor_pose_real_blank_v1_candidate_20260812/model.pt
```

关键参数：

```text
model: 冻结候选 model.pt
imgsz: 640
conf: 0.40
nms_iou: 0.70
type_strategy: normalized_dual_pnp_latch
gray_threshold: 140
D type boundary: 0.0326
ROI expansion: 10%
MIN_FILL_RATIO: strictly > 0.80
pair light-length ratio: strictly > 0.50
image_qos_depth: 1
```

相机冻结条件：

```text
Hikrobot MV-CS016-10UC
1440x1080 bgr8
30 FPS
3000 us exposure
12 dB gain
```

候选策略会在新轨迹初始化时比较小、大装甲板两套归一化 PnP，通过固定边界 `D < 0.0326 -> small`，否则为 `large`，随后在该轨迹内锁存类型。

## 4. 通用默认配置仍未晋级

通用参数文件：

```text
/home/jo/my-project/rm_vision/rm_auto_aim/auto_aim_bringup/params/yolo_detector_params.yaml
```

它目前仍是：

```text
model: armor_pose_v2_with04/weights/best.pt
conf: 0.55
type_strategy: fixed
armor_type: small
gray_threshold: 160
```

launch 默认仍指向这份通用配置。若启动时省略 `yolo_params`，所有固定分支结果都会按小装甲板尺寸进行 PnP，不会自动区分大小装甲板。

`fixed + armor_type: small` 下，即使 YOLO 能检测到大装甲板，程序仍会把它当成宽约 132 mm 的小装甲板计算，导致类型、距离和姿态不可靠。该分支直接使用 YOLO 四角点，不会使用 gray140 灯条提取，因此只把通用 YAML 的 `gray_threshold` 从 160 改成 140 没有意义。

后续实机测试必须明确加载完整候选 YAML：

```bash
ros2 launch auto_aim_bringup yolo_auto_aim.launch.py \
  yolo_params:=/home/jo/my-project/rm_vision/rm_auto_aim/auto_aim_bringup/params/yolo_detector_params_v8_frozen_candidate.yaml
```

将来若正式部署，需要晋级整份候选配置，而不是只晋级灰度阈值一个数字。

## 5. gray140 参数化实现

已完成：

- `RawTrackLatchProcessor` 增加实例级 `gray_threshold`，兼容默认值为 160。
- `find_lights`、`extract_light_pair` 和处理器调用链显式传递阈值。
- 阈值只接受 `[0, 255]` 内整数，拒绝布尔、浮点和越界值。
- 160 与 140 实例可在同一进程运行，不修改模块全局量，不发生参数串扰。
- ROS 节点声明 `gray_threshold=160`，候选 YAML 单独传入 140。
- 通用 YAML 保持 160，launch 默认未改变。

主要源码：

```text
yolo_armor_detector/scripts/raw_track_latch.py
yolo_armor_detector/scripts/yolo_armor_detector_node.py
yolo_armor_detector/test/test_raw_track_latch.py
yolo_armor_detector/test/test_candidate_config_contract.py
```

本包最新独立测试实际为 48 项通过。`colcon test-result` 曾显示 76 tests、8 skipped，这是工作区测试结果的汇总口径，不表示本包有 76 个新测试。

参数化与安装态回放报告：

```text
real_armor_diagnostics/gray140_candidate_config_promotion_v1/
```

裁决：

```text
GRAY140_CANDIDATE_CONFIGURATION_IMPLEMENTED_OFFLINE_VERIFIED
```

## 6. gray160 到 gray140 的证据链

### 6.1 原始大装甲板 yaw 失败

目录：

```text
real_armor_diagnostics/large_standard_pose_yaw_minimal_live_v1/
```

结果：

- 正面正式段：72/72 green 且 PnP 有效。
- 左 45 度正式段：121/121 有 YOLO 候选，但 0 green。
- 失败发生在 YOLO 之后、PnP 之前，主要是 `insufficient_lights` 和 `light_length_ratio`。
- 右 45 度短时预检同样为 30/30 YOLO、0 green。
- ROI 裁剪不是主因。

### 6.2 fill ratio 0.70 被否决

目录：

```text
real_armor_diagnostics/large_left45_fill_ratio_diagnostic_live_v1/
```

115 张真正左 45 度无损原图上，将 `MIN_FILL_RATIO` 从 0.80 降到 0.70 只恢复了短碎片，没有恢复有效灯条配对。该变量已否决，不应重测。

### 6.3 gray140 真正左 45 度离线回放

目录：

```text
real_armor_diagnostics/large_left45_gray140_offline_shadow_v1/
```

同一批 115 张真正左 45 度无损图和固定 YOLO 框：

| 阈值 | Green | 失败 |
|---|---:|---|
| 160 | 0/115 | 79 `insufficient_lights`，36 `light_length_ratio` |
| 140 | 115/115 | 无 |

gray140 的 115 帧全部为跨图像侧有效配对、selected PnP 有效、类型 `large`。同侧配对、短碎片、多配对歧义和错误类型均为 0。PnP 平均重投影误差中位/P95/最大约为 1.816/2.036/2.145 px。

### 6.4 旧实时左 45 度补测姿态无效

目录：

```text
real_armor_diagnostics/large_gray140_left45_minimal_live_v2/
```

该段 112/112 识别、配对、large、PnP 和同帧重放一致，但正式开始前姿态从约 50 度估算 yaw 漂移到约 27 度，因此不得描述成有效左 45 度证据。它只能作为实时实现与同帧离线重放一致的桥接证据。

## 7. 最新连续 yaw 实机验收

有效目录：

```text
real_armor_diagnostics/large_gray140_candidate_yaw_sweep_live_v2/
```

先前的 `v1` 只因相机尚未连接到 WSL 而得到 0 帧，正式记录从未开始；不要把 `v1` 当算法失败。

`v2` 使用实际生产检测节点、完整候选 YAML 和真实工业相机，只执行了一次连续正式会话。用户依次确认正面、左 45 度、右 45 度和返回正面。每个端点只统计确认后第 1 秒至第 4 秒的 3 秒稳定窗口。

裁决：

```text
GRAY140_CANDIDATE_LIVE_YAW_SWEEP_PASS
```

| 端点 | 稳定帧 | YOLO | Green | Large | PnP 有效 |
|---|---:|---:|---:|---:|---:|
| 初始正面 | 60 | 100% | 100% | 100% | 100% |
| 用户确认左 45 度 | 57 | 100% | 100% | 100% | 100% |
| 用户确认右 45 度 | 60 | 100% | 100% | 100% | 100% |
| 返回正面 | 50 | 100% | 100% | 100% | 100% |

四个端点均无错误类型、同侧配对、短碎片配对、越界点或错误多配对。四张中间代表图已由监督端复核，装甲板和两根灯条均完整入镜。

端点框宽中位数：

```text
初始正面: 622.892 px
左端点:   406.044 px
右端点:   413.330 px
返回正面: 598.822 px
```

左右端点相对初始正面的框宽比例约为 0.652 和 0.664，只用于支持连续 yaw 动作的一致性，不作为物理 yaw 真值。

三个 `transition` 窗口包含操作端提示、用户动作和等待回复的全部时间，不是精确的纯运动窗口。转回正面的交互窗口中出现少量 `insufficient_lights`、`normalized_center_distance` 和 1 次 `fill_ratio`，但这些失败没有延续到稳定端点，不影响通过裁决。

## 8. 最新报告存在的待勘误问题

以下结构化文件可信：

```text
real_armor_diagnostics/large_gray140_candidate_yaw_sweep_live_v2/summary.json
real_armor_diagnostics/large_gray140_candidate_yaw_sweep_live_v2/endpoint_metrics.json
real_armor_diagnostics/large_gray140_candidate_yaw_sweep_live_v2/stage_events.csv
real_armor_diagnostics/large_gray140_candidate_yaw_sweep_live_v2/track_diagnostics.jsonl
```

但 `v2/report.md` 和生成它的 `analyze_yaw_sweep.py` 模板残留两句来自失败 `v1` 的错误文字：

1. 错误声称相机报告 `no Hikrobot USB3 Vision camera found`。
2. 错误声称正式交互从未开始、没有左右确认原文。

这两句必须纯离线修正，然后重新生成 `v2/report.md`。不得改变 `summary.json`、端点指标、原始 JSONL 或 PASS 裁决，也不得为此重新采集。

正确事实是：

- 相机持续发布 1440x1080、约 30 FPS。
- 正式会话已开始并完成，共记录 5176 条唯一诊断。
- 左右端点的用户原文均为“已到位”，其方向来自当时操作提示上下文。
- 返回正面原文为“已返回正面”。

## 9. 延迟与帧率当前状态

WSL 高帧率传输诊断已经结束，不应重新打开：

- 相机可稳定发布约 28-30 FPS。
- Python 检测回调/诊断通常约 15-20 FPS。
- callback 内部中位约 22 ms，主要阶段是模型推理。
- 帧损失主要发生在相机发布后、Python callback 入口前。
- 强制 Fast DDS SHM 没有带来稳定的实质吞吐提升。
- 高速传输优化已明确留到未来原生 Linux mini 主机。
- FPS 不再作为 gray140 识别效果的否决门槛。

最新连续 yaw 的 5176 条诊断可直接建立延迟基线，无需再开相机。监督端预计算：

```text
camera header stamp -> diagnostics recorder receive
count: 5176
min: 约 24.18 ms
median: 约 42.53 ms
P95: 约 73.91 ms
max: 约 504.32 ms
negative ages: 0
```

记录器收到 3994 张 debug 图，对 5176 条诊断的覆盖率约 77.2%。22 张保存 PNG 的文件完成时间减原始 header stamp 为约 150-263 ms，中位约 189 ms，但这个值包含 debug 传输、PNG 编码和写盘，只能作为粗略上界，不能称为 RViz 渲染延迟。

当前没有记录 RViz 真正显示一帧的时刻，因此尚未精确测得 RViz 视觉延迟。

## 10. 下一步唯一任务

不启动相机、不要求用户摆装甲板，执行一次纯离线任务：

1. 修正 `v2/analyze_yaw_sweep.py` 报告模板中的两句错误。
2. 重新生成正确的 `v2/report.md`，保持 PASS 裁决。
3. 新建：

```text
real_armor_diagnostics/gray140_candidate_latency_offline_baseline_v1/
```

4. 使用已有 5176 条 JSONL 计算延迟分布、各阶段延迟和超过 100/200/500 ms 的尖峰。
5. 区分相机发布 FPS、诊断接收 FPS、debug 覆盖率和 PNG 完成时间上界。
6. 明确当前数据不能直接给出 RViz 渲染延迟。
7. 在 `docs/handoff.md` 追加连续 yaw 通过和报告勘误记录。

建议裁决名称：

```text
DETECTOR_OUTPUT_LATENCY_BASELINE_ESTABLISHED
```

完成后再决定是否需要一次无目标、短时、只测 `/detector/result_img` 接收年龄的实时测试。不得直接开始新的装甲板采集。

## 11. 模型训练历史边界

旧 v8 固定数据边界、Val05/Dev06 指标、视频 11、视频 12 和 v9-v9f 训练历史完整记录在 `docs/handoff.md`。

关键结论：

- v8 固定 Val05：Precision 0.9492、Recall 0.9032、F1 0.9256。
- v8 固定 Dev06：Precision 0.9717、Recall 0.9196、F1 0.9450。
- 视频 11 最终审核集：Precision 0.9455、Recall 0.6753、F1 0.7879；它不是严格独立盲测。
- v9-v9f 的“直接加入视频 12 并排列组合”路线已由用户明确终止。
- 不得重新启动 v9-v9f、重复阈值扫描或重新读取 Dev06 来选择模型。
- 当前任务不是继续训练新模型，而是完成已通过候选配置的报告、延迟和后续完整链路验收。

## 12. 用户偏好与监督约束

- 避免重复姿态、重复采集、重复预检和没有新信息的复测。
- 正式记录开始前必须明确通知用户，并等待指定回复。
- 一次只改变一个有意义的变量。
- 不要逐文件哈希，不要做过度防御式校验。
- 不清理暂存、不处理无关 Git 改动、不覆盖旧诊断目录。
- 用户确认的物理左右方向必须原文保存，不从图像侧别推断近侧或远侧。
- PnP 估算角度不是物理真值。
- 不修改模型、曝光、D、PnP、tracker、fill ratio 或配对门槛，除非已有直接证据并得到用户授权。
- WSL 帧率/DDS/USB 吞吐诊断已结束；原生 Linux mini 主机到位后再处理高速传输。
- 相机未连接时应直接说明，不能静默，也不能把硬件缺失写成算法失败。

## 13. 工作区注意事项

工作区存在大量已修改和未跟踪文件，包括 `yolo_armor_detector/`、两份 YAML、`docs/handoff.md` 和全部诊断目录。这些均视为用户或前序操作端工作。

禁止：

- `git reset --hard`
- `git checkout --` 回退现有文件
- 清理未跟踪目录
- 覆盖旧实验结果
- 为了“整洁”处理无关改动

修改相关文件时必须先读取现有内容并在其基础上工作。

## 14. 新对话建议首条指令

```text
请先读取：
/home/jo/my-project/rm_vision/rm_auto_aim/docs/yolo_model_training_handoff_3_wsl_2026-08-26.md

你是监督端。操作端会话ID为：
019fae48-36da-7c53-b009-da3431743c8e

先核对最新操作端消息和交接文件，不启动相机、不重新采集。当前只需要监督
v2报告模板勘误和已有5176帧的纯离线延迟基线任务。保持gray140连续yaw的
PASS裁决，不重新训练模型，不恢复WSL帧率诊断，不清理工作区。
```

## 15. 2026-08-26 冻结任务完成更新

本节覆盖第 10 节“下一步唯一任务”和第 13 节旧工作区状态：

- v2 报告模板中的“相机未找到”和“正式交互未开始”已修正并重新生成报告。
- `summary.json`、`endpoint_metrics.json` 和 `GRAY140_CANDIDATE_LIVE_YAW_SWEEP_PASS` 未改变。
- 已从现有 5176 条记录建立 `DETECTOR_OUTPUT_LATENCY_BASELINE_ESTABLISHED`，中位 42.52 ms、P95 73.97 ms、最大 504.32 ms，负延迟为 0。
- 53 帧超过 100 ms、5 帧超过 200 ms、1 帧超过 500 ms；debug 覆盖率为 3994/5176。
- 当前数据没有 RViz 展示时间戳，仍不能测量 RViz 渲染延迟。
- 候选模型路径已改为 `${YOLO_ARMOR_MODEL}`，完整冻结定义位于 `deployment/yolo_v8_gray140_candidate_20260826/`。
- 模型权重保留在 Git 外，部署清单只记录一个模型 SHA-256 和完整配置身份。
- 工作区源码、配置、工具和文档已经分组提交；大型诊断数据保留本机并由 Git 忽略。

当前下一阶段是确定 mini 主机架构后进行原生 Linux 部署，不再重复 gray140 姿态采集、WSL 帧率诊断或 v9-v9f 训练。
