# Auto-Aim v9b Train12 Stride30 结果

更新时间：2026-07-30（Asia/Shanghai）

## 结论

v9b 将视频 12 从每 15 帧取一张降为每 30 帧取一张，其他训练变量不变。实验在 Val05 第一阶段失败，因此没有读取 Dev06，也没有冻结、部署或盲测。

```text
Val05 Stage 1 门槛: F1 > 0.9256198347107438
v9b Val05 F1:     0.8820960698689957
结果:             REJECTED_AT_VAL05_STAGE1
```

固定 `conf=0.40` 指标：TP=101、FP=4、FN=23、Precision=0.9619、Recall=0.8145。

相对完整视频 12 的 v9，F1 从 `0.87719` 小幅提高到 `0.88210`，但距离 v8 仍很远。这说明降低视频 12 的时间重复度只带来轻微改善，不能单独恢复小目标能力。

训练完成 91 轮，最佳 fitness 位于第 61 轮。`best.pt` SHA-256 为 `2d286273624ed28c54562cf3aa80f7b33face90a3227186d3f228b650a40dd38`。

机器可读决策：

```text
/home/jo/my-project/my_first_autoaim_yolo/data/review_manifests_v9/armor_train_12a_angle_stride30_20260729/v9b_promotion_decision.json
```

当前仍保留 v8 冻结候选和 ROS 默认 v2。
