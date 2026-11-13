# 新外参整链测试入口

此目录属于 `codex/test-handeye-20261110` 独立工作目录。分支保存候选外参、启动适配和已核对的瞄准代码/配置快照；其余节点复用原工作区的已构建程序。

先做不连接硬件的检查：

```bash
bash /home/jo/Documents/Codex/rm_vision/outputs/handeye_branch_test_20261111/rm_auto_aim/tools/full_aim_handeye_test/start_full_aim.sh --check
```

启动整链测试：

```bash
bash /home/jo/Documents/Codex/rm_vision/outputs/handeye_branch_test_20261111/rm_auto_aim/tools/full_aim_handeye_test/start_full_aim.sh
```

当前脚本流程：每阶段 7 秒预热、15 秒只接收预检，通过原预检后运行 60 秒跟随。在启动终端按一次 Ctrl+C，等待退出清理完成。

TF 从本测试分支的 `rm_serial_driver/config/gimbal_tf.yaml` 加载六个新相机外参。弹速使用用户于 2026-11-11 采纳的落点估计 14.7 m/s，来源为 `impact_estimate`；`speed_calibration_file` 为空，直接使用配置值。估计依据见原工作区 `rm_aimer/docs/impact_speed_estimate_2026-11-11.md`。俯仰零点、比例、枪口位置和预检逻辑保持原值。原脚本实际使用的 yaw ±60°、pitch ±30°范围也保留；旧注释和元数据中的 pitch 20°已在测试副本中纠正为实际的30°。

旧入口的两处指纹差异已逐项核对：瞄准节点新增可选弹速标定加载和诊断输出，配置新增空的标定文件路径和弹速来源字段。这里使用当前源码快照，并保留原有独立 TF 接收线程及单线程瞄准执行器。两份 `reviewed_*_changes.json` 保存了差异。

启动时不再比对 `production_before.json` 中源码和参数文件的指纹；修改这些文件后无需更新该清单。参数一致性检查、TF 修复库检查和整链预检继续执行。每轮输出保存实际配置内容、哈希、路径和节点命令，供回看本轮加载内容。

原工作区继续留在 main，原配置和旧启动模板不修改。记录保存到 `/home/jo/Documents/Codex/rm_vision/outputs/handeye_branch_test_20261111/runs/`。停止测试后，原入口仍对应旧外参；它自身原有的两处指纹不一致不会被此测试副本改写。

本次只验证软件接入和启动前检查，没有运行硬件。这份外参仍是未独立验证的候选值。

## 2026-11-12：EKF R 实验

Tracker 参数由本分支的 `auto_aim_bringup/params/tracker_params.yaml` 加载，启动命令和每轮配置快照使用同一路径。当前候选为 `r_xyz_factor=0.0`、`r_xyz_min=0.000001 m²`、`r_yaw=0.00005 rad²`；Q、外参和 14.7 m/s 弹速维持现值。`--check` 会打印实际参数文件和 R。

运行仍复用主工作区已支持方差下限的 tracker 程序；本分支源码也补齐该下限，以支持将来重编译。R 在节点构造时读取，编辑后需要重启 tracker，不能只用 `ros2 param set`。

具体命令、旧/新 R 对比动作、记录指标和判断方法见 [EKF_R_TEST_PLAN.md](EKF_R_TEST_PLAN.md)。该候选来自当前距离的静止数据，动态跟随与其他距离尚待验证。
