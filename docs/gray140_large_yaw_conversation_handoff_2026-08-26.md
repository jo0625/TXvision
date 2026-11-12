# Gray140 大装甲板 Yaw 工作对话迁移交接

更新时间：2026-08-26（Asia/Shanghai）

> 本文是本轮长对话的工程交接摘要，不是逐字聊天记录。新对话应先读取本文，
> 再按本文列出的原始 `summary.json`、`report.md` 和 CSV/JSONL 核对细节。

## 1. 新对话应先知道的结论

当前有效的最终实机裁决是：

`GRAY140_CANDIDATE_LIVE_YAW_SWEEP_PASS`

冻结候选 YAML 中的 `gray_threshold=140` 已由实际生产 ROS 检测节点加载，并在一次
连续的大装甲板正面 -> 左约45度 -> 右约45度 -> 返回正面扫动中通过四个稳定端点。
四个端点的 YOLO 候选、`output_valid`、`large` 类型和 selected PnP 有效率均为
100%。

这仍是**候选配置验收**，不是将140部署为通用生产默认值：

- 通用 YAML 仍为 `gray_threshold: 160`。
- 节点默认值仍为160。
- `RawTrackLatchProcessor`兼容默认值仍为160。
- 仅冻结候选 YAML 为140。
- launch 默认仍指向通用 YAML。
- 当前没有相机、检测器、记录器或 RViz 进程残留。

未经用户新的明确指令，不要再次采集、重跑115帧、修改通用默认值、训练 YOLO，
也不要继续 WSL/DDS/FPS 诊断。

## 2. 工作区与固定配置

工作区：

```text
/home/jo/my-project/rm_vision/rm_auto_aim
```

冻结模型：

```text
/home/jo/my-project/my_first_autoaim_yolo/models/frozen/armor_pose_real_blank_v1_candidate_20260812/model.pt
```

关键固定参数：

```text
conf=0.40
type_strategy=normalized_dual_pnp_latch
normalized dual-PnP D boundary=0.0326
ROI expansion=10%
MIN_FILL_RATIO > 0.80
light_length_ratio > 0.50
camera=1440x1080, target 30 FPS, exposure 3000 us, gain 12 dB
default Fast DDS
```

参数文件：

```text
通用配置（160）:
auto_aim_bringup/params/yolo_detector_params.yaml

冻结候选配置（140）:
auto_aim_bringup/params/yolo_detector_params_v8_frozen_candidate.yaml
```

主要实现：

```text
yolo_armor_detector/scripts/raw_track_latch.py
yolo_armor_detector/scripts/yolo_armor_detector_node.py
```

## 3. 当前代码状态

已完成实例级灰度阈值参数化，裁决：

`GRAY140_CANDIDATE_CONFIGURATION_IMPLEMENTED_OFFLINE_VERIFIED`

实现边界：

- `RawTrackLatchProcessor`具有实例级 `gray_threshold`，默认160。
- 阈值只接受 `[0,255]` 内的整数；布尔、非整数和越界值会被拒绝。
- `find_lights`和`extract_light_pair`使用处理器实例传入的阈值。
- 不修改模块全局变量；160和140实例可以在同一进程独立运行而不串扰。
- ROS节点声明 `gray_threshold`，默认160，显式传给处理器并打印生效值。
- 通用 YAML=160，仅候选 YAML=140，launch 默认配置未改变。

验证结果：

```text
colcon build --packages-select yolo_armor_detector --symlink-install: PASS
colcon test --packages-select yolo_armor_detector: PASS
colcon test-result --verbose: 76 tests, 0 errors, 0 failures, 8 skipped
```

配置与回放证据：

```text
real_armor_diagnostics/gray140_candidate_config_promotion_v1/
```

重点文件：`report.md`、`config_contract.json`、`offline_replay_summary.json`、
`test_results.txt`、`evidence_adjudication.md`。

## 4. Gray140 证据链

### 4.1 Gray160 的真实高 yaw 失败

现存并有效的 gray160 高 yaw 诊断：

```text
real_armor_diagnostics/large_standard_pose_yaw_minimal_live_v1/
```

结果：

| 姿态 | 证据类型 | 帧数 | YOLO候选 | Green | 主要失败 |
|---|---|---:|---:|---:|---|
| 0度 | 正式 | 72 | 72/72 | 72/72 | 无 |
| 左约45度 | 正式 | 121 | 121/121 | 0/121 | 82 insufficient_lights；39 light_length_ratio |
| 右约45度 | 短预检 | 30 | 30/30 | 0/30 | 29 insufficient_lights；1 light_length_ratio |

右侧只有短预检证据，不能与两个正式段比较长期比例。三段均为100% YOLO候选，
因此该批高 yaw 失败发生在 YOLO 之后、selected PnP 之前，不支持补训 YOLO，
也不能把未进入 PnP 的候选说成 PnP 失败。

### 4.2 Fill ratio 候选被淘汰

完整逐轮廓记录：

```text
real_armor_diagnostics/large_left45_fill_ratio_diagnostic_live_v1/
```

在115张精确配对的无损 PNG 上，正式 `MIN_FILL_RATIO>0.80` 时为0 green；
将唯一变量降到0.70只放行了约5.9至10.0 px的短碎片，没有恢复有效配对。

裁决：`FILL_RATIO_NOT_DOMINANT`

解释边界：该裁决只淘汰 `MIN_FILL_RATIO=0.70` 这一候选，不能扩写成
“fill ratio 与现象完全无关”。更早4张稀疏代表图曾提示 fill ratio，但样本不足，
最终应以115张完整同帧结果为准。

### 4.3 Gray140 的115帧离线单变量结果

目录：

```text
real_armor_diagnostics/large_left45_gray140_offline_shadow_v1/
```

同一批115张无损 PNG、固定同一批 YOLO 框，只改变灰度阈值：

| 结果 | Gray160 | Gray140 |
|---|---:|---:|
| Green | 0/115 | 115/115 |
| 有效跨图像左右侧配对 | 0/115 | 115/115 |
| selected PnP有效 | 0/115 | 115/115 |
| large输出 | 0/115 | 115/115 |

Gray160失败分布精确复现为：

```text
unable_to_pair:insufficient_lights = 79
unable_to_pair:light_length_ratio = 36
```

Gray140恢复的是完整长灯条，不是短碎片。两侧 selected 灯条长度约98至100 px和
82至83 px，长度比约0.820至0.847。无同侧 selected pair、短碎片 selected pair、
多配对歧义或错误类型。

PnP mean重投影误差 median/P95/max：

```text
1.816 / 2.036 / 2.145 px
```

离线裁决：`GRAY_140_OFFLINE_CANDIDATE_SUPPORTED`

### 4.4 继承的0度实时控制

目录：

```text
real_armor_diagnostics/large_gray140_minimal_live_validation_v1/
```

该轮外部跨话题记录器只精确匹配20/29帧，因此整轮裁决为
`INSUFFICIENT_VALID_DATA`，并且没有开始左45度段。但保留下来的20个精确0度样本
全部为 green、有效跨侧配对、selected PnP有效、large，且140同帧回放一致率100%。

它是有限但固定继承的0度控制证据，不能改写成满足原先30帧门槛的完整验收。

### 4.5 记录链路与WSL节拍结论

source-colocated记录方案把当前处理帧图像、header和诊断作为一个不可拆分对象入队，
解决了外部订阅 `/image_raw` 后按时间戳匹配造成的9条缺图：

```text
real_armor_diagnostics/gray140_source_capture_pipeline_dryrun_v1/
裁决: CAPTURE_EXACTNESS_PASS_PERFORMANCE_LOW
离线: 115/115 PNG与诊断精确对应
实时: 79/79精确对应，覆盖率100%，队列丢弃和写入错误均为0
处理率: 15.797 FPS
```

回调计时：

```text
real_armor_diagnostics/gray140_detector_callback_timing_v1/
裁决: ROS_DELIVERY_OR_EXECUTOR_CADENCE_BOUND
callback total median/P95: 22.117/36.938 ms
predict median: 18.870 ms
callback入口间隔median: 64.025 ms
相机原生发布率median: 27.8 FPS
```

Fast DDS强制SHM：

```text
real_armor_diagnostics/gray140_fastrtps_shm_transport_v1/
裁决: FASTRTPS_SHM_NO_MATERIAL_GAIN
8秒完整callback: 128（15.981 FPS）
SHM通信成功但未达到144帧要求
```

监督决定是停止WSL吞吐/DDS诊断，接受当前约15至20 FPS节拍完成识别验证；
高帧率传输优化留到原生Linux小主机。FPS仅报告，不用于否决gray140识别效果。

## 5. 不能作为左45度真值的112帧

目录：

```text
real_armor_diagnostics/large_gray140_left45_minimal_live_v2/
```

这112帧虽然全部 green、large、有效配对、PnP有效，并且实时与同帧回放112/112一致，
但正式记录时姿态已从接受预览发生明显变化：

```text
接受预览框宽: 322.706 px
正式开始前框宽: 460.466 px
正式段框宽median: 469.093 px
```

监督裁决明确规定：它**不是左45度实机证据**。只允许作为
“gray140实时处理与同帧离线重放一致”的桥接证据。不得依据PnP估算yaw约26.6度
把它重新解释成物理姿态真值。

## 6. 最终连续 Yaw 实机验收

有效目录：

```text
real_armor_diagnostics/large_gray140_candidate_yaw_sweep_live_v2/
```

前一目录必须保留但不计分：

```text
real_armor_diagnostics/large_gray140_candidate_yaw_sweep_live_v1/
```

V1因相机节点报 `no Hikrobot USB3 Vision camera found` 而停止，正式记录0帧，
物理交互未开始。V2恢复时先通过最短连通门，且相机只启动一次并持续运行。

### 6.1 V2实际运行参数

```text
gray_threshold=140
type_strategy=normalized_dual_pnp_latch
model=/home/jo/my-project/my_first_autoaim_yolo/models/frozen/armor_pose_real_blank_v1_candidate_20260812/model.pt
conf=0.40
publish_track_diagnostics=true
publish_debug=true
image_qos_depth=1
```

V2使用实际生产检测节点从候选 YAML 加载140，没有模块全局覆盖。相机在运行时为
1440x1080，30 FPS配置，3000 us，12 dB。`/image_raw`只有检测器一个订阅者；
RViz只显示 `/detector/result_img`。

正式记录在用户精确回复 `开始连续测试` 前保持0帧。物理方向由提示上下文和用户原文
确认，不根据图像左右侧或PnP角度推断：

```text
初始: 准备好
开始: 开始连续测试
左端点: 已到位（处于左转提示上下文）
右端点: 已到位（处于右转提示上下文）
返回: 已返回正面
```

### 6.2 稳定端点结果

各端点排除了确认后的首1秒，只统计随后至4秒的稳定窗口：

| 端点 | 稳定帧 | YOLO候选 | Green/output_valid | Large | selected PnP有效 |
|---|---:|---:|---:|---:|---:|
| 初始正面 | 60 | 100% | 100% | 100% | 100% |
| 左约45度 | 57 | 100% | 100% | 100% | 100% |
| 右约45度 | 60 | 100% | 100% | 100% | 100% |
| 返回正面 | 50 | 100% | 100% | 100% | 100% |

四个端点共同满足：

- 目标框和 selected 灯条点100%在图像边界内。
- 错误类型0。
- 同图像侧 selected pair 0。
- 短碎片 selected pair 0。
- 点集异常/越界0。
- 多green候选帧0。
- 代表图确认large装甲板及两根灯条完整入镜。

端点PnP mean重投影误差median：

```text
初始正面: 4.285 px
左约45度: 2.030 px
右约45度: 2.448 px
返回正面: 3.472 px
```

YOLO框宽median：

```text
初始正面: 622.892 px
左约45度: 406.044 px，正面比值0.652
右约45度: 413.330 px，正面比值0.664
返回正面: 598.822 px
```

压缩比支持扫动一致性，但不是精密物理yaw真值。

### 6.3 过渡段现象

```text
transition_to_left:
  821帧，candidate 100%，green 100%，无失败

transition_to_right:
  728帧，candidate 100%，green 100%
  另有22个 light_ownership_conflict 橙色候选

transition_to_return:
  1194帧，candidate 98.74%，green 95.56%
  insufficient_lights=22
  normalized_center_distance=19
  fill_ratio=1
```

过渡窗口包含提示、用户动作及回复等待，不是纯移动时长。运动期间的附加橙框不否决
四个稳定端点。

完整正式记录共5176条唯一stamp诊断，重复stamp、解析错误和代表图写入错误均为0。
相机发布FPS median为30.0；诊断接收约19.402 FPS。FPS只报告。

最终裁决：

`GRAY140_CANDIDATE_LIVE_YAW_SWEEP_PASS`

权威文件：

```text
real_armor_diagnostics/large_gray140_candidate_yaw_sweep_live_v2/report.md
real_armor_diagnostics/large_gray140_candidate_yaw_sweep_live_v2/summary.json
real_armor_diagnostics/large_gray140_candidate_yaw_sweep_live_v2/endpoint_metrics.json
real_armor_diagnostics/large_gray140_candidate_yaw_sweep_live_v2/stage_events.csv
real_armor_diagnostics/large_gray140_candidate_yaw_sweep_live_v2/track_diagnostics.jsonl
```

## 7. 其他已完成分支

### 7.1 Small装甲板图像边界初始化门

目录：

```text
real_armor_diagnostics/normalized_dual_pnp_init_image_bounds_small_standard_pose_full_chain_live_v1/
```

用户在开始前明确确认small装甲板约135 mm且约15度姿态固定。正式边界事件中，track 18
在frame 4047因一点恰为 `y=1080` 被门控，没有类型锁存、输出或正式dual-PnP；
frame 4048四点首次全部合法，调用一次dual-PnP，`D=0.006882<0.0326`，初始化为
small且 `initialization_frame=4048`。错误large初始化和输出均为0。

裁决：`IMAGE_BOUNDS_INIT_GATE_SMALL_STANDARD_POSE_LIVE_PASS`

### 7.2 旧large记录的yaw复用分析

目录：

```text
real_armor_diagnostics/large_existing_capture_yaw_offline_reanalysis_v1/
```

基数精确复现：2754候选、1555 green、1199 orange。仅265/1199 orange可可靠关联
PnP估算yaw，934/1199（77.90%）为unknown；可靠数据没有证明大yaw覆盖。

监督更正：`[-50,-35)`只是旧记录主PnP估算竖直姿态簇，没有人工姿态真值，不能称为
已确认的官方15度姿态。“大yaw覆盖缺失”必须表述为“现有证据无法证明大yaw覆盖”。

总裁决保持：`EXISTING_EVIDENCE_INSUFFICIENT_FOR_YAW_TRAINING_DECISION`

## 8. 必须遵守的解释边界

- PnP估算yaw/pitch不是人工物理测量真值。
- 图像左/右侧不能擅自解释成物理近侧/远侧。
- 橙框表示YOLO已有候选但后级失败；没有人工逐帧目标存在真值时，不能据此计算
  “YOLO完全无框”的真实漏检率。
- gray160高yaw记录中YOLO候选率100%，不支持因该现象训练YOLO。
- `large_gray140_left45_minimal_live_v2`的112帧不是左45度证据。
- `large_gray140_candidate_yaw_sweep_live_v1`正式记录为0帧；有效实机裁决只来自V2。
- 更早曾有被监督否决并由用户删除的错误yaw实验，不得恢复或引用已删除结果。
- 现存 `large_standard_pose_yaw_minimal_live_v1` 是后来重新建立的gray160高yaw诊断，
  不要与已删除的旧同名错误实验混淆。
- 140通过候选验收不等于已授权修改通用生产默认值。
- 不要自动补拍、扫描灰度阈值、改fill ratio/配对门、改PnP/D/tracker或继续吞吐诊断。

## 9. 当前停止状态与合理下一步

最后一次V2连续扫动结束后，相机、生产检测节点、记录器和RViz均已停止，相关残留进程
数量为0。实验当时Windows和WSL均识别到Hikrobot相机；这只是当时的连通证据，
新会话不能假定当前USB仍保持Attached。

当前工程结论是：`gray_threshold=140`已具备冻结候选配置资格，并完成实际候选ROS节点
连续yaw验收。下一步若涉及把140提升为通用生产默认值、部署到原生Linux小主机，或做
其他装甲板/距离/光照验收，必须由用户另行明确授权并重新预注册边界；不要从本文自动
启动任何实时节点或采集。

## 10. 给新对话的建议开场

可在新对话中直接发送：

```text
请先读取：
/home/jo/my-project/rm_vision/rm_auto_aim/docs/gray140_large_yaw_conversation_handoff_2026-08-26.md

以其中“最终连续Yaw实机验收”和“必须遵守的解释边界”为当前事实边界。
不要自动启动相机、复测、训练或修改通用默认配置。先根据我接下来的新任务继续。
```
