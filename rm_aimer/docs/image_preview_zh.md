# 相机弹道预览

从工作区运行：

```bash
bash rm_auto_aim/tools/run_ballistic_preview.sh
```

打开 http://127.0.0.1:8767 。停止按钮或启动终端的 Ctrl+C 会停止本次启动的节点并释放相机。
已有同名流水线节点时启动器退出，避免重复占用设备。已有流水线可单独运行
`ballistic_image_overlay.py --output <输出目录>`，并确保 aimer 的 `publish_diagnostics` 已启用。

- 青色圆点是按求解方向发射、经过解算飞行时间后的理想落点，包含目标运动预测。
- 紫色十字是发射方向直线在同一飞行时间对应位置的投影；它比落点高出重力下坠量。
- 黄色曲线是同一解算结果对应的理想弹道。它们均经图像采集时刻的 TF 和相机畸变模型投影。

预览使用 `/detector/result_img`，按原相机内参和显示图尺寸比例调整投影。
只叠加与图像时间戳完全匹配、年龄不超过 0.5 秒的有效弹道结果；无目标、TF 缺失、数据超时或标定不匹配时显示原因。
预览页面断流后隐藏旧画面。输出目录保存日志、最近画面和最近一次有效标记画面。

启动器以 `receive_only:=true` 读取姿态，解算输出到 `/aimer/visualization_command`。
当前配置弹速 14.7 m/s 是用户采纳的落点初步估计，依据见
[测量记录](impact_speed_estimate_2026-11-11.md)。标定与发射口位置沿用已有近似测量，
显示结果不代表已验证的多距离命中精度。

已复用的资料：`handeye_calibration_runs/20260917_fixed_board` 的 8 组求解与 2 组留出图像、`test_results/format_effective_target_2026-10-13` 与 `test_results/static_target_20261015_1713` 的装甲板画面。旧手眼结果的留出误差为 112.269 mm / 5.3857°，足以作为预览输入，尚不能建立击打精度结论。账户已加入 dialout 但当前桌面未载入新组时，启动器仅在反馈子进程中使用已有组权限并恢复 ROS 环境。
