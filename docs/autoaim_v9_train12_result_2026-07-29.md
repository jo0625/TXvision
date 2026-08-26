# Auto-Aim v9 Train12 单变量实验结果

更新时间：2026-07-29（Asia/Shanghai）

## 结论

v9 Train12 单变量实验已完成，但未通过预注册晋级门槛。保留 v8 冻结候选；不得冻结或部署本次 v9，也不得用本次失败候选执行视频 13 盲测。

预注册门槛：

```text
min(v9 Val05 F1, v9 Dev06 F1) > 0.9256198347107438
```

实测：

```text
min(v9 Val05 F1, v9 Dev06 F1) = 0.8771929824561403
```

因此状态为 `NOT_PROMOTED`。

## 训练

- 实验：`armor_pose_v9_train12_single_variable`
- 唯一变量：在 v8 固定 Train 上增加已冻结审核的视频 12 数据
- 初始化：`yolov8n-pose.pt`
- `imgsz=640`、`batch=4`、`seed=0`、`deterministic=true`
- 计划 150 轮，按 `patience=30` 在完成 74 轮后早停
- Val05 fitness 最佳轮次：44
- `best.pt` SHA-256：`9593022540a88da7c6d77c291f2e2c96fe3d7f58c1d6c477843abe79a03c60be`

## 固定评估

只评估训练期由 Val05 选择的 `best.pt`，且只使用冻结参数 `conf=0.40`；未扫描阈值，未使用视频 11。

| Split | TP | FP | FN | Precision | Recall | F1 | 背景干净率 |
|---|---:|---:|---:|---:|---:|---:|---:|
| Val05 | 100 | 4 | 24 | 0.961538 | 0.806452 | 0.877193 | 1.00000 |
| Dev06 | 97 | 4 | 15 | 0.960396 | 0.866071 | 0.910798 | 0.96875 |

相对 v8：

- `min_f1` 下降 `0.0484268523`
- Val05 Recall 下降 `0.0967741935`
- Dev06 Recall 下降 `0.0535714286`
- Dev06 背景干净率不变

本结果只能证明“按本次固定训练和评估设置，直接加入视频 12 没有晋级”；不能据此声称视频 12 数据本身有害，也不能在缺少独立对照时断言下降根因。

## 工件

机器可读决策：

```text
/home/jo/my-project/my_first_autoaim_yolo/data/review_manifests_v9/armor_train_12a_angle_20260729/v9_train12_promotion_decision.json
SHA-256: 665dfc31ccb4b6036e8a6bd8b4c6d7136dbec2b2b8ff69c9b822bbec45374f26
```

固定评估报告：

```text
/home/jo/my-project/my_first_autoaim_yolo/runs/eval_v9_train12_single_variable/armor_pose_v9_train12_single_variable_conf040_selection.json
SHA-256: b90ab3011cd1c3e63a04008d1ea5a6abc289a0b3c002a6f4e28afcb57c49adf4
```

原始 `handoff.md` 不修改，因为视频 12 冻结清单已记录其 SHA-256。后续接手时应先读本文件和上述决策 JSON，再把原 `handoff.md` 作为冻结边界说明读取。

## 当前边界

- v8 冻结模型保持不变，SHA-256 仍为 `3954a6b25b9be3fb150ff21a0f23abc2767ff9db3d9322c5568464282b59631f`。
- ROS 默认配置仍为 v2、`conf=0.55`。
- 视频 11 仍只能作为历史诊断数据。
- 本次 v9 不得冻结、部署或进入视频 13 盲测。
- 开始下一项实验前必须重新预注册单一变量和晋级规则；不得事后用 Dev06 或视频 11 调参补救本次结果。
