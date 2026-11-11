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

流程沿用原入口：15 秒预热、30 秒只接收预检，通过原预检后运行 60 秒跟随。在启动终端按一次 Ctrl+C，等待退出清理完成。

TF 从本测试分支的 `rm_serial_driver/config/gimbal_tf.yaml` 加载六个新相机外参。俯仰零点、比例、枪口位置、19 m/s 弹速和预检逻辑保持原值。原脚本实际使用的 yaw ±60°、pitch ±30°范围也保留；旧注释和元数据中的 pitch 20°已在测试副本中纠正为实际的30°。

旧入口的两处指纹差异已逐项核对：瞄准节点新增可选弹速标定加载和诊断输出，配置新增空的标定文件路径和弹速来源字段。这里使用当前源码快照，并保留原有独立 TF 接收线程及单线程瞄准执行器。两份 `reviewed_*_changes.json` 保存了差异。

`production_before.json` 对实际加载的候选配置、代码快照和外部配置逐项校验；没有关闭指纹检查。每轮输出还保存实际配置内容、哈希、路径和节点命令。

原工作区继续留在 main，原配置和旧启动模板不修改。记录保存到 `/home/jo/Documents/Codex/rm_vision/outputs/handeye_branch_test_20261111/runs/`。停止测试后，原入口仍对应旧外参；它自身原有的两处指纹不一致不会被此测试副本改写。

本次只验证软件接入和启动前检查，没有运行硬件。这份外参仍是未独立验证的候选值。
