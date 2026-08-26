# RoboMaster 自瞄 v8：视频 11 最终测试记录

更新日期：2026-07-27

## 1. 状态

```text
FINAL_BLIND_TEST_11_COMPLETE_PROVISIONAL_GT_AUDIT_REQUIRED
```

冻结 v8 已按固定参数在视频 11 上执行一次正式推理。未扫描阈值、未改变模型、未重跑其他 v8 参数。

由于本次真值标签采用“v7 自动预标注 + 用户逐帧复审”，并且正式推理后的错误图抽查发现疑似漏标和纳入标准不一致，当前指标应解释为“按现有标签计算的正式结果”，不能描述为严格独立人工盲标结果。

## 2. 输入与冻结校验

原始视频：

```text
/home/jo/my-project/my_first_autoaim_yolo/data/final_blind_test_11/raw/armor_test_11.mp4
SHA-256：2c7f549c894c117bcecd68863bcb74e837bf7381eb8a55657205e72b628c71ed
```

视频参数：

```text
960x720
29.970469 FPS
751 帧
约 25.058 秒
```

固定按每 15 帧抽取一张，共 51 张。

人工复审并机械规范化后的标签：

```text
标签文件：51
空标签：8
目标：71
标签清单 SHA-256：0db0bbf2876efc5a965a8bd088676ce55a7267d4346e1104489531cf461eb1a2
```

冻结模型：

```text
/home/jo/my-project/my_first_autoaim_yolo/models/frozen/armor_pose_v8_candidate_20260727/model.pt
SHA-256：3954a6b25b9be3fb150ff21a0f23abc2767ff9db3d9322c5568464282b59631f
```

## 3. 固定评估参数

```text
imgsz：640
conf：0.40
nms_iou：0.70
max_det：300
agnostic_nms：false
match_iou：0.50
kpt_threshold：0.05
min_keypoint_conf：0.15
min_quad_area：25 px²
min_light_length：4 px
max_light_length_ratio：2.5
detect_color：-1
```

## 4. 按当前标签计算的结果

| 指标 | 结果 |
|---|---:|
| 图片 | 51 |
| 背景帧 | 8 |
| 背景干净帧 | 6 |
| 背景干净率 | 0.7500 |
| GT 目标 | 71 |
| 预测目标 | 55 |
| TP | 51 |
| FP | 4 |
| FN | 20 |
| Precision | 0.9273 |
| Recall | 0.7183 |
| F1 | 0.8095 |
| 逐帧完全正确率 | 0.5686 |
| Mean IoU | 0.8333 |
| Mean keypoint error | 0.00994 |
| Mean inference | 9.13 ms |

相较于 05/06，视频 11 上的主要退化是 Recall，而不是定位精度：匹配成功目标的 Mean IoU 仍为 0.8333，但存在 20 个 FN。

## 5. 错误分布

```text
错误帧：22/51
仅 FN：18 帧
仅 FP：3 帧
同时 FP/FN：1 帧
几何后处理额外过滤：0 个预测
```

20 个 FN 中：

- 15 个来自同帧存在多个装甲的场景；常见表现为识别正面大装甲，但漏掉侧面、小目标或大斜角目标。
- 5 个来自单目标帧，且整帧没有预测：`000270`、`000540`、`000555`、`000570`、`000585`。
- 漏检 GT 的中位框宽约 59.8 px；成功匹配 GT 的中位框宽约 100.4 px。
- 典型困难包括远距离、小装甲、运动模糊、强斜视角以及隔网遮挡。

4 个 FP 的置信度均较高，约为 0.66–0.85。典型错误包括数字板或地面图案相似结构。

## 6. 真值审计问题

正式推理后的错误图抽查发现：

- `armor_test_11_000315.jpg` 的右侧装甲具有数字板和两侧蓝色灯条，疑似是真实装甲但未进入 GT，当前被计为 FP。
- `armor_test_11_000330.jpg` 存在部分可见侧面装甲，是否具备可靠四角需要按统一规则独立判断。
- 一些被计为 FN 的 GT 很小、严重模糊或被网格遮挡；旧标注规则规定“无法可靠判断四角时跳过，并不放入最终测试子集”。视频 11 当前没有执行独立的 curated 子集步骤。

因此不应根据当前 0.8095 F1 直接宣布模型正式通过或失败，也不应据此调整 v8 参数。

## 7. 当前决定

- 不修改冻结模型、阈值或后处理。
- 暂不把 ROS 默认模型从 v2 切换到 v8。
- 保留本次 v8 推理输出，不再次调用模型。
- 若要得到更可靠的诊断指标，应由独立复审者在不看 v8 叠加图的情况下审核全部 51 张原图，并统一执行“可可靠标四角/否则跳过”的规则；之后可直接用已保存预测重新计算，无需再次推理。
- 一旦根据视频 11 的结果修改标签、模型或参数，视频 11 应降级为 Dev；下一次严格最终验收需要全新视频 12。

## 8. 结果文件

```text
/home/jo/my-project/my_first_autoaim_yolo/data/final_blind_test_11/reports/v8_final_blind_summary.json
/home/jo/my-project/my_first_autoaim_yolo/data/final_blind_test_11/reports/v8_final_blind_per_image.csv
/home/jo/my-project/my_first_autoaim_yolo/data/final_blind_test_11/reports/v8_final_blind_predictions.json
/home/jo/my-project/my_first_autoaim_yolo/data/final_blind_test_11/errors/
```

结果文件 SHA-256：

```text
summary：081d24f09409a85f3f10ede9f86e13865ee44ab71c72cc1524d487a362155740
per-image CSV：650a5e38ac09e325be66f1b28ae675f9a2ebe435f74dc4d455ff0e3a952d8fcf
predictions JSON：d316f137a2363245ca9c739e6831d2837144677a45e2426f93dc63f361c7412f
```

## 9. 推理后标签审计修正

用户再次复审 `armor_test_11_000735.jpg`，删除了同一侧面装甲的一个重复标签。修正后：

```text
标签文件：51
空标签：8
目标：70
近重复目标对：0
标签清单 SHA-256：3684cacf5f26b49020fa78377a6a5db0e60a52f78ae1db30a8f8f4438f87bc6f
```

没有重新运行 v8。直接使用首次正式推理保存的预测重新匹配，得到：

| 指标 | 修正前 | 修正后 |
|---|---:|---:|
| GT | 71 | 70 |
| TP | 51 | 51 |
| FP | 4 | 4 |
| FN | 20 | 19 |
| Precision | 0.9273 | 0.9273 |
| Recall | 0.7183 | 0.7286 |
| F1 | 0.8095 | 0.8160 |
| 逐帧完全正确率 | 0.5686 | 0.5686 |

重算记录：

```text
/home/jo/my-project/my_first_autoaim_yolo/data/final_blind_test_11/reports/v8_final_blind_recomputed_after_label_audit.json
```

这次修正属于查看 v8 结果后的标签审计，因此重算结果用于工程诊断，不恢复视频 11 的严格独立盲测属性。
