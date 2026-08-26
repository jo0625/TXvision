# RoboMaster 自瞄项目 v8 阶段总结与交接

更新日期：2026-07-27

详细冻结报告：`rm_auto_aim/docs/autoaim_v8_fixed_boundary_freeze_2026-07-27.md`

## 1. 当前结论

v8 最终候选模型已经完成训练、对比、参数冻结和 ROS 接入验证。目前状态为：

```text
FROZEN_PENDING_FINAL_BLIND_TEST_11
```

在全新视频 11 到来之前，不再修改模型权重、输入分辨率、置信度阈值、NMS 或几何后处理参数，也不把 03、06 加入训练。

## 2. 固定实验边界

```text
Train：01、02、04、07、08、09、10、训练硬负样本 s1/s2
Val：05
Dev benchmark：06
Legacy holdout：03
Final blind test：之后提供的全新视频 11
```

执行规则：

- 03 和 06 禁止加入训练集。
- 模型、阈值、分辨率和后处理只能依据 05、06 选择。
- 03 当前有 124 张图片、0 个标签，保留为历史留出集，不参与本轮调参。
- 视频 11 必须是此前未使用的新视频；先人工标注，再使用冻结模型一次性评估，不能根据 11 的结果回头调参。

## 3. 09、10 标签审核结果

构建最终数据集前的统计：

| 来源 | 图片数 | 空样本数 | 目标数 |
|---|---:|---:|---:|
| 09 | 101 | 77 | 45 |
| 10 | 90 | 0 | 163 |

已经把标签框统一为恰好包围四个关键点，并完成以下检查：

- 17 字段格式错误：0
- 重复、交叉或退化关键点：0
- 关键点位于检测框外：0

构建数据集时排除了 3 张经核实重复的纯黑帧：

```text
armor_train_08_000000.jpg
armor_train_08_001050.jpg
armor_train_09_000000.jpg
```

## 4. 冻结数据集

数据集路径：

```text
/home/jo/my-project/my_first_autoaim_yolo/datasets/armor_pose_v8_fixed_boundary
```

配置文件：

```text
/home/jo/my-project/my_first_autoaim_yolo/config/armor_pose_v8_fixed_boundary.yaml
```

| 划分 | 来源 | 图片数 | 空样本数 | 目标数 |
|---|---|---:|---:|---:|
| Train | 01 | 74 | 8 | 129 |
| Train | 02 | 7 | 0 | 13 |
| Train | 04 | 94 | 0 | 187 |
| Train | 07 | 65 | 0 | 101 |
| Train | 08 | 69 | 17 | 52 |
| Train | 09 | 100 | 76 | 45 |
| Train | 10 | 90 | 0 | 163 |
| Train | hard negative s1 | 51 | 51 | 0 |
| Train | hard negative s2 | 51 | 51 | 0 |
| Val | 05 | 84 | 0 | 124 |
| Dev | 06 | 121 | 32 | 112 |

汇总：

```text
Train：601 张，203 张空样本，690 个目标
Val 05：84 张，124 个目标
Dev 06：121 张，32 张空样本，112 个目标
```

边界与泄漏检查：

```text
Train 中来自 03/06 的样本：0
数据集构建后的完全重复图片：0
Train 与 05/06/03 之间 dHash 距离 <= 4：0
```

注意：dHash 是近似重复检查，只能作为启发式证据，不能证明所有图片在语义上都不同。

数据集清单 SHA-256：

```text
274d2ab86288e766c7d6567e486846589770b51a534943ad9b7f31dd172a16f4
```

## 5. 训练与候选模型选择

训练配置：

```text
初始化权重：yolov8n-pose.pt
imgsz：640
batch：4
最大 epochs：150
patience：30
seed：0
deterministic：true
实际最后 epoch：132
Ultralytics fitness 最佳 epoch：102
```

固定条件下的 05/06 对比：

| 模型 | conf | Val 05 F1 | Dev 06 F1 | Dev 背景干净率 |
|---|---:|---:|---:|---:|
| v2 | 0.50 | 0.8634 | 0.8649 | 84.4% |
| v7 | 0.20 | 0.9191 | 0.9231 | 90.6% |
| v8 best，epoch 102 | 0.50 | 0.9283 | 0.8972 | 96.9% |
| **v8 last，epoch 132** | **0.40** | **0.9256** | **0.9450** | **96.9%** |

最终冻结 `v8 last epoch 132`，而不是 Ultralytics 自动保存的 `best.pt`。原因是预先确定的选择目标为：

```text
最大化 min(Val 05 F1, Dev 06 F1)
```

冻结模型在 Dev 06 上的结果：

```text
TP：103
FP：3
FN：9
Precision：0.9717
Recall：0.9196
F1：0.9450
逐帧完全正确率：0.9174
背景干净率：0.96875
```

## 6. 冻结模型与推理参数

只读冻结目录：

```text
/home/jo/my-project/my_first_autoaim_yolo/models/frozen/armor_pose_v8_candidate_20260727
```

模型文件及校验值：

```text
model.pt
SHA-256：3954a6b25b9be3fb150ff21a0f23abc2767ff9db3d9322c5568464282b59631f
```

冻结参数：

```text
imgsz：640
conf：0.40
nms_iou：0.70
max_det：300
agnostic_nms：false
min_keypoint_conf：0.15
min_quad_area：25 px²
min_light_length：4 px
max_light_length_ratio：2.5
detect_color：-1（关闭）
```

在 05/06 上，NMS 0.5 与 0.7，以及若干关键点、几何过滤设置得到相同指标。因此保留上述数值是为了兼容现有 YOLO/ROS 配置，不能声称这些数值被实验验证为更优。

## 7. ROS 状态

候选参数文件：

```text
/home/jo/my-project/rm_vision/rm_auto_aim/auto_aim_bringup/params/yolo_detector_params_v8_frozen_candidate.yaml
```

ROS 检测节点已经显式声明并传入：

```text
nms_iou
max_det
agnostic_nms
```

`yolo_armor_detector` 已通过构建和测试。默认 ROS 参数目前仍指向 v2，在完成视频 11 的盲测前不要切换默认模型。

## 8. 已验证事实与推测

### 已验证事实

- 当前数据集符合固定边界，Train 中没有 03、06。
- 按 05/06 的预定选择规则，v8 last 是本轮最佳候选。
- 冻结权重、推理参数、数据集清单和校验值已经保存。
- ROS 相关修改已经构建并通过测试。

### 尚未验证的推测

- 不能断言 v8 的提升由 09、10 数据直接造成，因为没有进行只移除 09/10 的独立消融实验。
- 不能仅凭 05/06 断言 v8 一定比 v7 有更好的真实场景泛化能力，必须等待视频 11。
- 不能把某个阈值或 NMS 数值称为性能提升的根因，因为部分候选值在 05/06 上结果完全相同。

## 9. 下一步：最终盲测视频 11

用户需要提供全新的：

```text
armor_test_11.mp4
```

收到后执行：

1. 提取视频 11 的图片，保持与 Train、05、06、03 隔离。
2. 先由人工完成正式标签，不先查看冻结模型预测。
3. 验证标签格式和关键点顺序。
4. 使用冻结的 `model.pt`、`imgsz=640`、`conf=0.40` 和既定后处理一次性评估。
5. 保存逐图报告、汇总指标和错误样例，不根据视频 11 重新调参。
