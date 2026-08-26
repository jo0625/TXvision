# Auto-Aim / YOLO 交接说明

更新时间：2026-08-20（Asia/Shanghai）

## 1. 交接结论

2026-08-20运行时更新：`yolo_armor_detector`的可选`raw_track_latch`分支已依据
`live_small_gray_threshold_190_vs_160_v1`单变量实验，将传统灯条灰度阈值从190改为160。
该分支的ROI、轮廓筛选、配对、3.2类型边界、PnP、轨迹锁存和同帧灯条所有权规则均未改变；
`type_strategy`默认仍为`fixed`，独立C++传统检测节点的参数未修改。冻结6168候选和本轮25张
live PNG的正式源码/安装产物验收结果见
`real_armor_diagnostics/gray_threshold_160_runtime_acceptance_v1/`。

当前阶段不是“继续修改视频 11”，而是“冻结 v8 与视频 11，先冻结视频 12 的训练数据，再做 v9 单变量实验”。

- v8 模型、推理参数和固定数据边界已冻结。
- 视频 11 已完成 51 帧全量审核和最终重算；它现在只能作为历史诊断数据，不能再参与 v9 的模型、阈值、分辨率或后处理选择。
- 视频 12 的候选目录目前有 147 张图、147 个标签文件、14 个空标签、195 个目标；最后缺失的 `001110` 标签已经存在。
- 视频 12 已依据用户对整批 147 帧的人工审核确认建立正式审核清单，并已构建新的 v9 单变量数据集快照；详见第 9 节。第 9 节所述归档轮次未启动训练；更早的 v9-v9d 实验结果见第 10 节。
- v9e 只改变初始化 checkpoint 并完成一次训练；固定 Val05 Stage 1 未通过，未进行 Dev06 模型推理或指标评估。完整性预检曾读取 Dev06 文件计算哈希，协议勘误见第 11 节。
- ROS 默认配置仍加载 v2；v8 仅有独立候选参数文件，没有部署为默认模型。

若本文与 2026-07-27 的旧文档冲突，以本文引用的当前 JSON、清单和哈希为准。旧视频 11 文档中的 70 GT、F1=0.8160 等是中间状态，不是最终结果。

## 2. 已验证事实

### 2.1 v8 固定实验边界

```text
Train: 01, 02, 04, 07, 08, 09, 10, train_hard_negative_s1, train_hard_negative_s2
Val:   05
Dev:   06
Legacy holdout: 03
```

约束解释：

- 03、06、11 不得进入训练集。
- 05 用于训练期验证和 checkpoint 选择。
- 06 只作为固定 Dev benchmark / 晋级检查使用，不做面向 06 的反复自由调参。
- 11 已成为历史诊断数据，不得再作为选择输入。
- 下一次冻结后的盲测必须使用全新视频 13。

固定数据集：

```text
/home/jo/my-project/my_first_autoaim_yolo/datasets/armor_pose_v8_fixed_boundary
```

配置：

```text
/home/jo/my-project/my_first_autoaim_yolo/config/armor_pose_v8_fixed_boundary.yaml
```

统计：

| Split | 图片 | 空样本 | 目标 |
|---|---:|---:|---:|
| Train | 601 | 203 | 690 |
| Val05 | 84 | 0 | 124 |
| Dev06 | 121 | 32 | 112 |

关键哈希：

| 工件 | SHA-256 |
|---|---|
| `datasets/armor_pose_v8_fixed_boundary/manifest.csv` | `274d2ab86288e766c7d6567e486846589770b51a534943ad9b7f31dd172a16f4` |
| `datasets/armor_pose_v8_fixed_boundary/boundary.json` | `b11fc742ac086829c926a82fddf7889a2ca0e0f46346666697922c77c88e1e11` |
| `config/armor_pose_v8_fixed_boundary.yaml` | `0558c79ef0571e0ebb49bb50d0bd1c29928320db9339f38d47d80698914bdbf9` |

### 2.2 v8 冻结模型与参数

模型：

```text
/home/jo/my-project/my_first_autoaim_yolo/models/frozen/armor_pose_v8_candidate_20260727/model.pt
SHA-256: 3954a6b25b9be3fb150ff21a0f23abc2767ff9db3d9322c5568464282b59631f
```

冻结清单：

```text
/home/jo/my-project/my_first_autoaim_yolo/models/frozen/armor_pose_v8_candidate_20260727/freeze_manifest.json
SHA-256: ca8991ff41208bcaf280952b0afea11bf626772bc44c11f267d7bce034d2afd3
```

训练基线：

```text
architecture/init: yolov8n-pose.pt
imgsz: 640
batch: 4
epochs: 150
patience: 30
seed: 0
deterministic: true
frozen checkpoint: epoch 132
```

冻结推理与后处理参数：

```text
imgsz=640
conf=0.40
nms_iou=0.70
max_det=300
agnostic_nms=false
min_keypoint_conf=0.15
min_quad_area=25
min_light_length=4
max_light_length_ratio=2.5
detect_color=-1
```

v8 的 05/06 参考结果：

| Split | TP | FP | FN | Precision | Recall | F1 | 背景干净率 |
|---|---:|---:|---:|---:|---:|---:|---:|
| Val05 | 112 | 6 | 12 | 0.9492 | 0.9032 | 0.9256 | 1.0000 |
| Dev06 | 103 | 3 | 9 | 0.9717 | 0.9196 | 0.9450 | 0.96875 |

现有选择规则是：最大化 `min(Val05 F1, Dev06 F1)`；平手时依次比较 Dev 背景干净率、平均 F1、兼容性/延迟。v8 的 `min_f1` 基线为 `0.9256198347107438`。

当前文件没有另行记录一个明确的 v9 绝对晋级阈值。若“晋级”要求必须超过 v8，而不只是按上述规则在候选中排序，必须在训练/查看 v9 结果前写明 `>`、`>=` 以及是否约束单侧 Recall、FP 和背景干净率，不能事后补规则。

### 2.3 视频 11 最终状态

视频 11 的最终标签审核已经完成并冻结：

```text
总帧数: 51
INCLUDE: 50
SKIP: 1，仅 armor_test_11_000090.jpg
有效 GT: 77
已编辑并逐帧确认的工作标签: 9 帧
未解决问题: 0
```

9 个已确认编辑帧：

```text
000060, 000075, 000105, 000150, 000255,
000315, 000420, 000600, 000615
```

最终评估是使用原始一次性 v8 推理保存下来的冻结预测，对冻结后的工作标签重新计算；没有重跑模型。

| 指标 | 数值 |
|---|---:|
| 评估帧 | 50 |
| GT / 预测 | 77 / 55 |
| TP / FP / FN | 52 / 3 / 25 |
| Precision | 0.9454545455 |
| Recall | 0.6753246753 |
| F1 | 0.7878787879 |
| Frame exact accuracy | 0.46 |
| Background clean rate | 0.7142857143 |
| Mean IoU | 0.8310897431 |

严格性限制：视频 11 标签最初有模型辅助，并且在原始 v8 结果可见后继续完成了全量审核。因此这组数值应称为“最终审核集结果”，不是严格独立的盲测泛化估计。

关键路径与哈希：

| 工件 | 路径 | SHA-256 |
|---|---|---|
| 图像清单 | `data/final_blind_test_11/images/armor_test_11` | `9f2e4fd28ab19204f2b30965bb630fb7aa0864e371df8662cb2a5964e38a3da8` |
| 冻结工作标签清单 | `data/final_blind_test_11/full_audit_working_v2/labels/armor_test_11` | `899b2997bcf549ad2f838e04a8eda753eced43693e40ca4ed06ca3a55a51739f` |
| 冻结预测 | `data/final_blind_test_11/reports/v8_final_blind_predictions.json` | `d316f137a2363245ca9c739e6831d2837144677a45e2426f93dc63f361c7412f` |
| 最终 summary | `data/final_blind_test_11/reports/v8_final_blind_full_audit_summary.json` | `234289ff616100fed701b74f4c03adfd136fe25779cdb5db58bfdcd073451db9` |
| 最终逐帧 CSV | `data/final_blind_test_11/reports/v8_final_blind_full_audit.csv` | `44b2b5a8506db800dffe2416a0f57df6afdefdfe642c9febd91c264476ea9209` |
| 重算脚本 | `scripts/recompute_final_blind_11_full_audit.py` | `fd539bf68a71101bd2de7b58afdca564921770b64082c55ce2b6e8dfbca5ca72` |

以上相对路径均以 `/home/jo/my-project/my_first_autoaim_yolo` 为根目录。

### 2.4 视频 11 的 25 个 FN 只读分类

以下是用户提供的视觉现象分类，不是排除规则，也不是已验证根因：

```text
VA 11: 000060, 000075, 000240, 000255, 000270, 000300,
       000375, 000615, 000720, 000735, 000750
VN  2: 000195, 000480
XO  4: 000540, 000555, 000570, 000585
XB  8: 000105, 000420, 000450, 000510, 000525, 000600, 000630, 000645
XF  0
U   0
```

含义：`VA` 疑似大角度/侧面，`VN` 外观和角度较普通，`XO` 疑似遮挡，`XF` 疑似靠近/超出边缘，`XB` 疑似模糊、过曝或欠曝，`U` 无法确定。类别允许多选；当前列表是用户第一次归类的记录，不应自动解释为互斥类别。

### 2.5 视频 12 当前文件状态

原始视频：

```text
/home/jo/my-project/my_first_autoaim_yolo/data/raw_videos_v9/armor_train_12a_angle.mp4
SHA-256: 2fd4ae4abb858f7c56f1a97e824ab4e2ed749366a84ea31bfc1dc1bdbcec88e3
```

已读取元数据：`1280x720`、30 FPS、2191 帧、73.03 秒、约 14.9 MB。

候选目录：

```text
/home/jo/my-project/my_first_autoaim_yolo/data/candidates_v9/armor_train_12a_angle
```

抽帧过程记录为每 15 帧取 1 帧，得到帧号 `000000` 至 `002190` 的 147 张图片。该过程目前没有独立的正式抽帧 manifest，接手时应补入 v9 数据冻结清单。

当前只读统计：

```text
图片: 147
标签文件: 147
空标签: 14
非空标签: 133
目标总数: 195
格式错误: 0
缺失标签: 0
```

`armor_train_12a_angle_001110.txt` 已存在，包含 1 个目标；它是先前唯一缺少审核结果的帧。

候选数据清单哈希使用与视频 11 相同的算法：按文件名排序，依次写入 `filename + NUL + file bytes + NUL` 后计算 SHA-256。

| 工件 | SHA-256 |
|---|---|
| 147 张 JPG 的命名文件清单 | `2a588531b3c488959deef1d8a98c2055c9d656095e3c37e25b0397d54dc8c047` |
| 147 个 TXT 的命名文件清单 | `b77bea082d7a58089dacbde37583089cbd85dfe17438c59b3bd9c2b826bb9f82` |
| `armor_train_12a_angle_prelabel_report.csv` | `91d98dc3d1017a0ee153c138438e454974ced7c17cc5c03887566d22f0740614` |

预标注历史：冻结 v8 以 `conf=0.15`、`min_keypoint_conf=0.15` 生成候选；最初 124 帧有标签、165 个候选目标，23 帧无标签，另有 2 个候选被过滤。预标注只是人工审核起点，不能视为真值来源。

2026-07-30 更新：正式的 147 行审核 manifest 已在独立版本目录冻结。所有帧统一记录为 `USER_BATCH_CONFIRMED`，依据是用户对整批 147 帧的人工审核声明；未记录的逐帧 `n/s` 按键历史统一写为 `NOT_RECORDED`，没有按帧虚构。

### 2.6 ROS 当前状态

默认参数文件：

```text
/home/jo/my-project/rm_vision/rm_auto_aim/auto_aim_bringup/params/yolo_detector_params.yaml
```

它当前仍指向：

```text
/home/jo/my-project/my_first_autoaim_yolo/runs/pose/armor_pose_v2_with04/weights/best.pt
conf=0.55
```

v8 候选参数文件独立存在：

```text
/home/jo/my-project/rm_vision/rm_auto_aim/auto_aim_bringup/params/yolo_detector_params_v8_frozen_candidate.yaml
```

因此不得描述为“v8 已部署”，也不得在 v9 冻结并完成独立验收前切换默认模型。

## 3. 用户观察

- 视频 12 同时包含大角度、纯正面目标和无装甲板画面，不应把整段视频统一称为“大角度数据”。
- 用户声明已经逐张人工审核：正确预标注直接按 `n`；无装甲板帧以及补标/修改后的帧按 `s`、`n` 保存。
- 用户希望 YOLO 倾向高 Recall，并考虑后续测试更低 `conf`、保留多候选、真实运动模糊/失焦/噪声/过曝/暗光数据、按视频切分、分难度统计 Recall 和连续漏检长度。

这些观察可以指导后续实验设计，但不能覆盖冻结边界，也不能自动替代逐帧审核 manifest。

## 4. AI 推测与尚未证实事项

- 视频 12 的大角度样本可能改善类似 `VA` 的漏检，但尚未训练对照模型，不能声称已确认会改善。
- 视频 11 中角度、遮挡、模糊、多目标等只与漏检同时出现，尚未通过控制实验确认是根因。
- 不应把随机高斯模糊等同于真实相机运动模糊；是否有效必须使用与真实成像接近的增强做独立对照。
- “传统视觉在 YOLO 前”目前没有接近 100% 候选召回率的实测证据，不应改主流程。只有先在独立 Dev 数据证明传统候选器在各成像子集均有近乎无漏检的召回率，才值得单独实验。
- 降低 `conf`、修改 NMS、增大 `imgsz`、换模型尺寸或改后处理都可能改变 Precision/Recall，但不能与“新增视频 12 数据”同时改，否则无法判断收益来自哪里。
- v9-v9d 在旧域 Val05 上的 Recall/F1 下降可能与新增数据的分布变化或从通用预训练权重重新初始化导致的遗忘有关，但二者都只是 AI 假设，尚未通过单变量实验确认为根因。

已有反证边界：此前在 05/06 上把 `conf` 从 0.40 降至 0.20 没有改善 Dev06 Recall，反而降低 Precision；`imgsz=960` 未优于 640；`max_det=300` 说明系统并非只保留最高分单框；视频 11 的 raw prediction 与通过几何过滤后的数量均为 55，因此 25 个 FN 不能归因于当前几何过滤器。

## 5. 实际文件修改

历史上已经发生且应保留的修改：

- 视频 11 只在隔离工作副本中编辑了 9 帧标签；原活动标签快照没有被覆盖。
- 视频 11 最终报告只根据冻结预测与冻结工作标签重算；v8 模型没有重跑。
- 视频 12 已生成 147 张候选图、预标注报告和 147 个当前标签文件；原始视频未修改。

本次交接只新增：

```text
/home/jo/my-project/rm_vision/rm_auto_aim/docs/handoff.md
```

本次没有修改标签、模型、参数、预测、历史报告或 ROS 默认配置。

## 6. 禁止事项

1. 不得再修改视频 11 的标签、角点、GT、INCLUDE/SKIP、预测、模型、参数或正式报告。
2. 不得把视频 11 的 25 个 FN 用于选择 v9 模型、阈值、分辨率、NMS、增强或后处理。
3. 不得追溯应用评估后才提出的“5 px / 灯条长度 10%”规则，也不得因 `XO/XB/XF` 分类删除 GT 或整帧改为 SKIP。
4. 不得把 AI 判断或 `.txt` 文件存在自动写成 `USER_CONFIRMED`；审核状态必须保留证据来源。
5. 不得覆盖原始标签、快照、冻结模型或历史报告；v9 必须构建新的独立数据集和输出目录。
6. v9 训练不得包含 03、06、11。06 只用于固定 Dev 评估，不得为了 06 的结果反复改实验。
7. v9 第一个对照实验只允许增加审核后的训练视频 12；模型结构、初始化、`imgsz`、seed、epoch、patience、增强、阈值和后处理均保持 v8 设置。
8. 在视频 12 审核 manifest 和标签哈希冻结前，不得训练 v9、扫描阈值或重算任何“正式”指标。
9. 在 v9 的 05/06 晋级规则预先写清并满足、模型冻结前，不得切换 ROS 默认模型。
10. v9 冻结后，只能用全新视频 13 做下一次盲测；视频 12 是 Train，不是 Test。

## 7. 下一步

> 2026-07-30 状态：第 7.1 至 7.3 项已由第 9 节所列新版本完成；v9-v9e 历史与结果见第 10、11 节。v9e 未通过 Val05 Stage 1，未进行 Dev06 模型评估，未晋级或部署；预检读取 Dev06 文件的协议勘误见第 11 节。

接手后的第一个动作不是训练，而是为视频 12 建立并冻结审核 manifest。

### 7.1 先只读复核

复核以下基线；任何数量或哈希不一致都应先停止并调查：

```text
images=147
labels=147
empty=14
non_empty=133
objects=195
images_manifest_sha256=2a588531b3c488959deef1d8a98c2055c9d656095e3c37e25b0397d54dc8c047
labels_manifest_sha256=b77bea082d7a58089dacbde37583089cbd85dfe17438c59b3bd9c2b826bb9f82
```

### 7.2 建立视频 12 审核清单

在独立版本目录创建 147 行 manifest，至少包含：

```text
frame_id,image_name,label_name,decision,object_count,
image_sha256,label_sha256,review_evidence,source_video,source_frame
```

`decision` 只能是 `ACCEPT / EMPTY / SKIP`。当前文件分布预期对应 133 个非空标签和 14 个空标签，但不要只根据文件存在推断人工确认；应引用用户的整批人工审核声明，并对任何无法映射的帧请求确认。SKIP 帧在构建数据集时必须同时排除图片与标签。

冻结清单必须写入：原视频路径与哈希、分辨率/帧率/总帧数、每 15 帧抽取、预标注模型与参数、147 张图和标签的命名清单哈希、`ACCEPT/EMPTY/SKIP` 数量以及 manifest 自身 SHA-256。

### 7.3 构建 v9 单变量数据集

新建数据集快照，不覆盖 v8 数据集：

```text
Train = v8 固定 Train + 视频 12 中审核通过的 ACCEPT/EMPTY
Val   = 05，不变
Dev   = 06，不变
Exclude = 03, 11，以及视频 12 的全部 SKIP
```

先做重复/近重复检查，确保视频 12 不与 05、06、03、11 发生相邻帧或内容泄漏。输出来源清单、split 统计和 SHA-256。

### 7.4 训练、晋级与下一次盲测

1. 训练前写明 v9 绝对晋级条件；若没有新门槛，至少原样采用现有 `min(Val05 F1, Dev06 F1)` 排序规则并明确与 v8 基线的比较方式。
2. 第一个 v9 训练只改变训练数据，其他变量完全复用 v8。
3. 使用 05 选择 checkpoint；冻结候选后在 06 做固定晋级评估，不查看视频 11 来选任何参数。
4. 如要实验低 `conf`、模糊增强、Top-K 或传统视觉前置，每项单独做预注册对照，只使用允许的开发数据，并单独记录变量和结果。
5. 满足预先写定的 05/06 条件后，复制模型与配置到新的冻结目录，记录所有工件 SHA-256。
6. 保持 ROS 默认 v2，直到用户明确授权部署。
7. 最后只在全新视频 13 上执行一次冻结后的盲测。

## 8. 关键参考文件

```text
/home/jo/my-project/rm_vision/rm_auto_aim/docs/autoaim_v8_handoff_summary_2026-07-27.md
/home/jo/my-project/rm_vision/rm_auto_aim/docs/autoaim_v8_fixed_boundary_freeze_2026-07-27.md
/home/jo/my-project/rm_vision/rm_auto_aim/docs/data_model_version_and_test_protocol_summary_2026-07-27.md
/home/jo/my-project/my_first_autoaim_yolo/data/final_blind_test_11/version_trace/label_audit_status.json
/home/jo/my-project/my_first_autoaim_yolo/data/final_blind_test_11/full_audit_working_v2/WORKING_COPY_MANIFEST.json
/home/jo/my-project/my_first_autoaim_yolo/data/final_blind_test_11/reports/v8_final_blind_full_audit_summary.json
/home/jo/my-project/my_first_autoaim_yolo/models/frozen/armor_pose_v8_candidate_20260727/freeze_manifest.json
/home/jo/my-project/my_first_autoaim_yolo/datasets/armor_pose_v8_fixed_boundary/boundary.json
```

交接时应优先读 JSON/CSV 与实际文件哈希，再读叙述性 Markdown；不得用旧文档中的中间数字覆盖当前冻结事实。

## 9. 2026-07-30 视频 12 归档与 v9 数据快照

用户确认：视频 12 的 147 张图片均已逐张人工审核。正确预标注直接按 `n`；无装甲板帧以及补标/修改后的帧按 `s`、`n` 保存。逐帧按键历史未单独记录，因此 manifest 不推断具体帧使用过哪个按键。

新审核版本：

```text
/home/jo/my-project/my_first_autoaim_yolo/data/review_manifests_v9/armor_train_12a_angle_20260730_user_batch_confirmed_v1
```

审核统计：`ACCEPT=133`、`EMPTY=14`、`SKIP=0`，共 147 行、195 个目标。所有 `review_evidence` 均为 `USER_BATCH_CONFIRMED`，所有 `review_action_history` 均为 `NOT_RECORDED`。

新数据集与配置：

```text
/home/jo/my-project/my_first_autoaim_yolo/datasets/armor_pose_v9_train12_single_variable_20260730_v1
/home/jo/my-project/my_first_autoaim_yolo/config/armor_pose_v9_train12_single_variable_20260730_v1.yaml
```

数据边界：Train 为 v8 固定 Train 加视频 12 的全部 `ACCEPT/EMPTY`；Val05 与 Dev06 逐字节保持不变；03、11、13 和所有 `SKIP` 不进入训练。本版本没有 `SKIP`。

| 工件 | SHA-256 |
|---|---|
| 审核 `audit_manifest.csv` | `f945a7f08abf2d168bc0373dafe39ec7dc050d307d108bced38ed3dc99a037ce` |
| 审核 `freeze_manifest.json` | `16a19c6f3d8bcd80d1328866cdd2b1c2f7ed271464f8849a05415a1d80cc9e4e` |
| `leakage_report.json` | `8354533994afdddc4f5b43a9d683eb2cb7196432a157177843559500f09ab9d0` |
| 数据集 `manifest.csv` | `8f67c5a1fd80124aa0986043fd623941a7342c824c918adfb8aacbbf994908b0` |
| 数据集 `boundary.json` | `c8234cfe95a02476bc6fba1c6b05e46c4b4661393e637445b60c7b3b644cb692` |
| 数据集 `source_stats.csv` | `3f1d5351f76f17f3fbb9f89231eca4b217d4b2c862c096f6bbc5dd14d25b8b31` |
| 数据集配置 | `03e8b5285dbfefff87979a16c24081df70626abe081f3d1490ad3e2d7c06f436` |

数据集统计：Train `748/217/885`，Val05 `84/0/124`，Dev06 `121/32/112`，依次为图片/空样本/目标数。

泄漏检查使用完全 SHA-256 和 16x16 dHash（Hamming 距离不大于 4 视为近重复）扫描视频 12 与 03/05/06/11：完全重复 0，近重复 0，通过。

本轮没有启动 v9 训练、阈值扫描、模型晋级评估、模型冻结或 ROS 部署；ROS 默认仍为 v2、`conf=0.55`。

## 10. 2026-07-29 至 2026-07-30 v9-v9d 历史实验

以下实验使用早于第 9 节 `20260730_v1` 快照的原始 v9 冻结数据。旧数据的图片与新快照一致，但标签目录有 110 个文件字节不同，因此两者不得互换作为字节一致对照。

| 实验 | 完成 epoch | 固定 Val05 F1 | 固定 Dev06 F1 | 结论 |
|---|---:|---:|---:|---|
| v9 | 74 | `0.8771929824561403` | `0.9107981220657276` | `NOT_PROMOTED` |
| v9b | 91 | `0.8820960698689957` | 未授权读取 | `REJECTED_AT_VAL05_STAGE1` |
| v9c | 沿用 v9b 第 91 epoch checkpoint | `0.8841201716738197` | 未授权读取 | `REJECTED_AT_VAL05_STAGE1` |
| v9d | 沿用 v9c checkpoint | `0.904564315352697` | 未授权读取 | `REJECTED_AT_VAL05_STAGE1` |

v9d 只执行了预注册的 Val05 置信度扫描，最佳 `conf=0.25`。上述结果确认这些候选未达到晋级门槛，但不能确认数据分布变化或灾难性遗忘是根因。

后续以 v9 为字节一致对照的实验必须继续使用：

```text
config/armor_pose_v9_train12_single_variable.yaml
dataset manifest: 403148871549ab6688f47e0432256a0b2c2b23a5f51f982c61b2c7d2cd5d5824
dataset boundary: 2c14902d336104059d21e2a564f1b65e73bd3484f5b22d4a10697797c0ff1d91
dataset config: 7b8f43afc80c4a10f328d6f4b89d38534dc4a01ea74b3a27628dea7f4b6b77ab
```

## 11. 2026-07-30 v9e 初始化 checkpoint 单变量实验

v9e 以第 10 节的全量 v9 为对照，唯一实际变量是 `model/init`：从 `yolov8n-pose.pt` 改为冻结 v8 `model.pt`；实验名称与输出目录随之独立。自动参数比较确认实际 `args.yaml` 仅有 `model`、`name`、`save_dir` 三项差异，无额外差异。

训练严格使用第 10 节的原始 v9 数据，不使用第 9 节 `20260730_v1` 快照，也没有修改包括 `001350` 在内的任何标签。训练完成 72 epoch；最佳 fitness 位于第 42 epoch，随后连续 30 epoch 无提升而 early stop。

固定 Val05 Stage 1 结果：

| TP | FP | FN | Precision | Recall | F1 | 结论 |
|---:|---:|---:|---:|---:|---:|---|
| 104 | 2 | 20 | `0.9811320754716981` | `0.8387096774193549` | `0.9043478260869565` | `REJECTED_AT_VAL05_STAGE1` |

Stage 1 要求 `Val05 F1 > 0.9256198347107438`，相等也失败。v9e 未达到门槛，因此 Dev06 未获授权做模型推理或指标评估，也没有生成 Dev06 报告；没有执行阈值扫描、checkpoint 扫描、候选冻结或部署。

协议勘误：训练前的完整性预检逐文件校验了数据 manifest 的全部 953 对图像/标签，其中包含 121 对 Dev06 文件。它没有运行模型或生成 Dev06 指标，但按“不读取 Dev06”的字面要求仍属于预检越界。因此 v9e 不能描述为满足“Dev06 文件从未被读取”；拒绝结论不变，原始冻结工件不覆盖，详见 `protocol_erratum_v1.json`。

关键工件：

| 工件 | SHA-256 |
|---|---|
| `preregistration_v1.json` | `197481c8e26f981daf8025080bcf783aac9f17764f88915a72155342137e262f` |
| `preflight_v1.json` | `92a9f775382a086f2877dd848bba5c04e9b25dd46ddb933ca5b54f06dc4b188b` |
| v9e `best.pt` | `d7eccd12fd85cf49ce8e0f903506cbf26650871021b0a1c014ad0456c7beceb2` |
| v9e `last.pt`（未评估） | `c4736ec58a70e48ff33a4d27c7a625068b3f5d9e71d4f75415ccfd624d1cb724` |
| v9e `results.csv` | `f94c12570342250d4eaed05b051825ab687c17df8d0b40985635c2d47d1a2894` |
| v9e `args.yaml` | `9e8f751447b480571f34e1d9fd99165cdd6ab398cad4d0bc285002ef36582809` |
| `v9e_val05_fixed_v1.json` | `207aa8c9ebd3210e70b05825bf5d048eafcce9029d92537bd2c1dbda1e6723df` |
| `promotion_decision.json` | `6aa1f06bba9d900e6fc98ba90020d2bd7e7dddd7b3ffcc1ca66ff8ebbfa740b2` |
| `protocol_erratum_v1.json` | `ef6fe663fb43183cef1ac3144ac09b91da5f0dcf308de404f47b738ce05f6bc4` |

实验目录：

```text
/home/jo/my-project/my_first_autoaim_yolo/data/experiments_v9/v9e_init_from_v8
/home/jo/my-project/my_first_autoaim_yolo/runs/pose/armor_pose_v9e_train12_init_from_v8
```

v9e 相比 v9 的固定 Val05 F1 有提高，但仍低于 v8 基线；该结果不能把数据分布变化或灾难性遗忘确认为根因。视频 11、视频 13 均未读取或使用，ROS 默认配置未修改。

## 12. 2026-07-30 v9f stride30 单变量实验与路线终止

v9f 补齐旧 v9 实验家族的 2x2 组合：对照 v9e 使用冻结 v8 初始化和原始 v9 全量视频 12 数据，v9f 使用同一冻结 v8 初始化和旧冻结 v9b stride30 数据。相对 v9e 的实际 `args.yaml` 仅有 `data`、`name`、`save_dir` 三项差异，无意外差异。该实验故意使用旧冻结 v9b 数据做字节一致对照，不替代或否定第 9 节的 `20260730_v1` 人工审核快照。

v9f 只训练一次，未 resume 或同名重跑；共完成 49 epoch，最佳 epoch 为 19，随后因 `patience=30` early stop。只使用 `best.pt` 对 Val05 执行了一次固定 Stage 1 评估，没有阈值扫描、checkpoint 扫描或 `last.pt` 评估。

| TP | FP | FN | Precision | Recall | F1 | 结论 |
|---:|---:|---:|---:|---:|---:|---|
| 101 | 1 | 23 | `0.9901960784313726` | `0.8145161290322581` | `0.8938053097345134` | `REJECTED_AT_VAL05_STAGE1` |

Stage 1 要求 `Val05 F1 > 0.9256198347107438`，相等也失败。v9f 未达到门槛，因此 Dev06 未获授权，未打开、哈希、解码、推理或生成报告；预检 trace 中 `images/dev/` 与 `labels/dev/` 命中均为 0。视频 11、视频 13 未读取或使用，模型未冻结或部署，ROS 未读取或修改。

关键工件：

| 工件 | SHA-256 |
|---|---|
| v9b 数据配置 | `4a534ef1b79cc390a4b7baba69665096c19e2697657ed35de877f6848e025d7f` |
| v9b 数据集 manifest | `1058a12d5cdc81698df9a199be26db7f192be0649ff9b61d8e320972147a498c` |
| v9b 数据集 boundary | `4e465c69e3f3d6375106ac5bf19e1dd759aa87052c64a57a6bd258627744011d` |
| `preregistration_v2.json` | `67f095541f17b8824237fe8458ad8889b387bf4c11e200a7b2989f42a166ed16` |
| `preflight_v2.json` | `4fcbf00b6c2484638a7d0ae9c1e1b620af08c060d7eb970d559988de6b0bf3a7` |
| `preflight_v2_open_trace.log` | `9c6378c583714649b6003b5e48b019edef168d5414550276a915e56374ed3bb0` |
| `eval_v9f_fixed_gate.py` | `9a787767a770d835a8f59afcb386b6307ee32908b19ab150b4a173d316cf45a1` |
| v9f `args.yaml` | `c2f5b80f02b9f675f10adf10782d78cdbb5c5ba4e73166745d5586768b9ebcd6` |
| v9f `results.csv` | `d6ad41f9cd0f925f2c81fadbad597fe38cab3dcff7382d2ad90a58087b7fed0d` |
| v9f `best.pt` | `8cf5a95e78081203186897dfc6301387786f9542e24db0ce165eb210a7694d79` |
| v9f `last.pt`（未评估） | `2b2b2dfd334e4b72560417442a4fc560b94d87abcd72016e2f827c6eb8d4d666` |
| `v9f_val05_fixed_v2.json` | `23b6ac43be612d0e5ad45834fe789e8f45ce124ff0b4e47526d786b8c8f5de49` |
| `stage1_decision.json` | `a4851b0468fd1fb568bc212dbc54496ec2407dd3af86bbc53c96037e4944f5b1` |

用户已明确确认终止 v9-v9f“直接加入视频 12 并继续排列组合”的实验路线。不得沿该路线再次训练或评估；任何后续方向必须另行提出并获得用户授权。

## 13. 2026-08-25 gray140 候选配置参数化

已将传统灯条灰度阈值参数化为 `RawTrackLatchProcessor` 实例参数：兼容默认值、ROS 节点默认值和通用 YAML 均保持 `160`，仅冻结候选 YAML 使用 `140`；launch 默认仍指向通用配置。阈值要求为 `[0, 255]` 内整数，160/140 实例可在同一进程独立运行，未修改模型、PnP、D、tracker、fill ratio 或配对门限。

构建通过，测试结果为 76 tests、0 errors、0 failures、8 skipped。使用安装态 runtime 对监督确认的真正左 45° 115 张无损图和固定 YOLO 框回放：160 为 0/115 green（79 `insufficient_lights`、36 `light_length_ratio`），140 为 115/115 green、跨图像侧配对、selected PnP 有效且类型为 large；同侧、短碎片、多配对歧义和错误类型均为 0。112 帧最新实时段只作为 140 运行态与同帧回放 112/112 一致的桥接证据，不作为左 45° 姿态证据。裁决为 `GRAY140_CANDIDATE_CONFIGURATION_IMPLEMENTED_OFFLINE_VERIFIED`，属于候选配置实现，不是正式生产部署。详见 `real_armor_diagnostics/gray140_candidate_config_promotion_v1/`。

## 14. 2026-08-26 连续 yaw 验收与离线延迟基线

gray140 完整候选配置已完成正面、用户确认左 45°、用户确认右 45°和返回正面的单次连续实机验收。四个稳定端点分别记录 60、57、60、50 帧，YOLO 候选、green、large 和 selected PnP valid 均为 100%，裁决为 `GRAY140_CANDIDATE_LIVE_YAW_SWEEP_PASS`。

`large_gray140_candidate_yaw_sweep_live_v2/report.md` 原模板中“相机未找到”和“正式交互未开始”两句已纯离线修正。正确事实是相机持续发布 1440x1080 图像，正式记录完成并取得 5176 条唯一诊断。`summary.json`、`endpoint_metrics.json`、原始 JSONL 和 PASS 裁决未改变。过渡窗口从操作提示开始到用户确认结束，包含提示、物理动作和等待回复，不能解释为纯运动时长。

同一批 5176 条诊断已建立 `DETECTOR_OUTPUT_LATENCY_BASELINE_ESTABLISHED`：

| 指标 | 结果 |
|---|---:|
| camera header 到诊断记录器最小延迟 | 24.18 ms |
| 中位延迟 | 42.52 ms |
| P95 | 73.97 ms |
| 最大延迟 | 504.32 ms |
| 超过 100 ms | 53/5176（1.024%） |
| 超过 200 ms | 5/5176（0.097%） |
| 超过 500 ms | 1/5176（0.019%） |
| 诊断接收 FPS | 19.40 |
| debug 图覆盖率 | 3994/5176（77.16%） |

22 张稀疏保存 PNG 的完成时间相对源时间戳为 150.49-263.07 ms，中位 188.82 ms。该值包含 debug 传输、PNG 编码和写盘，只能作为稀疏上界；没有 RViz 展示时间戳，不能由此声称测得 RViz 渲染延迟。

## 15. 2026-08-26 可迁移候选冻结

冻结目录为 `deployment/yolo_v8_gray140_candidate_20260826/`。名称中的 `yolo_v8` 表示模型路线，`gray140` 表示必须成套使用的候选后处理配置，`candidate` 表示尚未完成 mini 主机整链生产验收，`20260826` 表示冻结批次。

候选 YAML 的模型位置改为 `${YOLO_ARMOR_MODEL}`，只参数化模型文件位置，不改变模型内容或任何算法参数。模型二进制不进入普通 Git；本地冻结目录保留一份 `model.pt`，部署清单记录唯一模型校验值 `9beedce9691e7a602b2912ae617aca991622c1ef3c99107e7a59e0120b9c3dc2`。mini 主机迁移时通过 `YOLO_ARMOR_MODEL` 指向部署模型。

本次冻结不包含 CPU/GPU 推理后端，也不声称已完成 mini 主机完整链路验收。硬件架构确定后再选择 CUDA、CPU、OpenVINO 或其他后端，并依次执行干净检出构建、离线一致性、相机 smoke 和完整链路验收。
