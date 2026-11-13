# 单板 / 整车模型 A/B 测试

分支：`codex/test-single-armor-20261113`，源码基线：主工作区 `ac9dad5`。
原入口实际运行主工作区 tracker，所以这里从该版本继续；2026-11-13 当前手眼测试目录的外参、R、瞄准配置和独立 TF 接收线程一并复制。原工作区和原手眼分支不修改。

单板模式使用六维匀速 Kalman 状态 `[x, vx, y, vy, z, vz]`，直接观测装甲板 XYZ。按相同 ID 和空间距离选板，保留 0.15 m 关联距离、5 帧确认、0.3 s 丢失时间；不使用姿态 yaw 门限、车体中心/半径估计、旋转换板或多板候选选择。Target 的 `armors_num=1`，position/velocity 直接表示单板，yaw/v_yaw/radius_1/radius_2/dz 为零。九维 target_state 仅用于共享输出接口，滤波器和协方差本身均为六维。

平移 Q 使用既有 `sigma2_q_xyz_max=0.1`（原模型角速度为零时的值）。R 保留当前实测候选 `r_xyz_factor=0`、`r_xyz_min=1e-6 m²`；整车模式的 `r_yaw=5e-5 rad²` 保留。外参、14.7 m/s 弹速、prediction_time=0、控制频率和现有预检一致。两份 tracker YAML 只差 `tracker.motion_model`。

## 构建与离线检查

```bash
cd /home/jo/Documents/Codex/rm_vision/outputs/single_armor_test_20261113/rm_auto_aim
source /opt/ros/humble/setup.bash
source /home/jo/Documents/Codex/rm_vision/rm_auto_aim/install/setup.bash
colcon build --symlink-install --packages-select armor_tracker rm_aimer --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select armor_tracker rm_aimer
colcon test-result --verbose
bash tools/single_armor_test/start_full_aim.sh --check single_armor
bash tools/single_armor_test/start_full_aim.sh --check rotating
```

## 对照测试

每轮均为 7 秒预热、15 秒只接收预检、通过后 60 秒跟随；Ctrl+C 停止。准备好目标和设备后分别运行，避免两条链路同时启动：

```bash
bash /home/jo/Documents/Codex/rm_vision/outputs/single_armor_test_20261113/rm_auto_aim/tools/single_armor_test/start_full_aim.sh rotating
bash /home/jo/Documents/Codex/rm_vision/outputs/single_armor_test_20261113/rm_auto_aim/tools/single_armor_test/start_full_aim.sh single_armor
```

不传模型参数时默认 single_armor。建议同一距离、同一块可见装甲板，重复静止、匀速左右移动、反向动作，并交替测试两种模式。单板模型用于当前可见板的平移对照，不具备整车旋转和遮挡换板预测能力；多块同 ID 板仍可能发生空间关联切换。

记录位于 `../runs/`，目录名含模型。`pipeline.csv` 保存装甲数量、位置、速度、半径、tracking 和指令；`tracker.csv` 保存状态和回调时间；`aimer.csv` 保存瞄准解；配置快照和 `tracker_identity_*.json` 记录实际参数与库。确认单板输出始终为一块、零半径。对照顿挫时刻的位置/速度突变、tracking 变化、有效指令间隔和反馈，不能只比较主观感受。仅单板改善说明整车建模/姿态关联路径值得进一步定位，不能据此断言具体根因。

两种模式都使用本分支同一 tracker 二进制；运行时显式加载本分支库并核对已加载路径。相机、检测器、串口及 TF 修复库继续复用现有安装。瞄准验证入口使用本目录源码快照；修改 `rm_aimer/scripts/aimer_geometry.py` 后需同步快照。未自动运行硬件跟随。
