# yolo模型训练承接版_4_wsl

更新时间：2026-08-26（Asia/Shanghai）

## 0. 新对话先执行什么

先读取本文件，再读取：

```text
/home/jo/my-project/rm_vision/rm_auto_aim/docs/yolo_v8_gray140_candidate_freeze_2026-08-26.md
/home/jo/my-project/rm_vision/rm_auto_aim/deployment/yolo_v8_gray140_candidate_20260826/manifest.yaml
```

当前不是继续训练、重复采集或恢复 WSL 帧率诊断。当前阶段是等待 mini 主机
具体架构，然后把已经冻结的候选迁移到原生 Linux 并完成整链验收。

## 1. 角色与工作区

- 当前对话角色：监督端，同时已经直接完成 Git 整理和冻结操作。
- 旧操作端会话 ID：`019fae48-36da-7c53-b009-da3431743c8e`。
- ROS 工作区：`/home/jo/my-project/rm_vision/rm_auto_aim`。
- 外层 `/home/jo/my-project/rm_vision` 不是 Git 仓库；Git 根目录是 `rm_auto_aim`。
- 训练工程：`/home/jo/my-project/my_first_autoaim_yolo`。
- ROS 环境：`source /opt/ros/humble/setup.bash` 后再 `source install/setup.bash`。
- 含 Ultralytics 的现有 venv：`/home/jo/my-project/my_first_autoaim_yolo/.venv`。

## 2. 当前一句话状态

`WSL_VALIDATED_CANDIDATE_FROZEN_FOR_PORTABLE_DEPLOYMENT`

模型、完整候选配置、检测器、tracker 时间保护、目标 watchdog、验证结论和部署
清单已经冻结；mini 主机推理后端、原生 Linux 相机吞吐和控制整链尚未验收。

## 3. Git 状态

```text
branch: main
HEAD: 0938c52 feat(deploy): freeze portable gray140 candidate
tag: yolo-v8-gray140-candidate-20260826
remote status: main 比 origin/main 领先 7 个提交
stash: empty
worktree: clean（创建本交接文件之前）
```

冻结前的重要提交：

```text
0938c52 feat(deploy): freeze portable gray140 candidate
c90849f docs: archive YOLO training and validation handoffs
cb525b3 feat(auto-aim): add YOLO detector and guarded tracking pipeline
c506053 feat(camera): add Hikrobot capture and calibration tools
bd084bb chore: ignore local diagnostic artifacts
```

这些提交和标签尚未推送远程。未经用户明确授权不要自行 push，也不要改写冻结标签。

## 4. 完整候选配置

候选配置文件：

```text
auto_aim_bringup/params/yolo_detector_params_v8_frozen_candidate.yaml
```

关键参数：

```text
model: ${YOLO_ARMOR_MODEL}
model identity: armor_pose_real_blank_v1_candidate_20260812
imgsz: 640
conf: 0.40
type_strategy: normalized_dual_pnp_latch
gray_threshold: 140
D < 0.0326 -> small
D >= 0.0326 -> large
armor_type fallback: small
exposure baseline: 3000 us
gain baseline: 12 dB
camera frame-rate request: 30 FPS
```

模型本地冻结副本：

```text
/home/jo/my-project/rm_vision/rm_auto_aim/deployment/yolo_v8_gray140_candidate_20260826/model.pt
size: 6411992 bytes
SHA-256: 9beedce9691e7a602b2912ae617aca991622c1ef3c99107e7a59e0120b9c3dc2
```

模型二进制由 `.gitignore` 排除，不进入普通 Git。部署清单只对这一份模型记录一次
校验值，不需要对所有文件逐项哈希。

## 5. 通用默认配置仍然不同

通用配置仍是：

```text
model: armor_pose_v2_with04/weights/best.pt
conf: 0.55
type_strategy: fixed
armor_type: small
gray_threshold: 160
```

如果启动时省略 `yolo_params`，launch 会使用通用 `fixed + small`，不会自动区分
大小装甲板。以后晋级必须迁移完整候选 YAML，不能只把通用配置中的 160 改成
140。

## 6. 已接受的验证证据

### 6.1 Gray140 配置实现

裁决：

```text
GRAY140_CANDIDATE_CONFIGURATION_IMPLEMENTED_OFFLINE_VERIFIED
```

- 160 和 140 为实例级参数，不存在模块全局污染。
- 真正左 45° 的 115 帧安装态回放：160 为 0/115 green，140 为 115/115 green。
- 140 下有效配对、selected PnP 和 large 类型均通过，配对风险计数为 0。
- 未修改模型、PnP、D、tracker、fill ratio 或配对几何门限。

### 6.2 连续真实 yaw 验收

裁决：

```text
GRAY140_CANDIDATE_LIVE_YAW_SWEEP_PASS
```

| 稳定端点 | 帧数 | YOLO | Green | Large | PnP |
|---|---:|---:|---:|---:|---:|
| 正面 | 60 | 100% | 100% | 100% | 100% |
| 用户确认左 45° | 57 | 100% | 100% | 100% | 100% |
| 用户确认右 45° | 60 | 100% | 100% | 100% | 100% |
| 返回正面 | 50 | 100% | 100% | 100% | 100% |

左右方向来自当时操作提示上下文，不从图像左右或 PnP 估算反推。PnP 估算姿态
不是物理真值。

原 `v2/report.md` 中“相机未找到”和“正式交互未开始”是模板残留，已经纯离线
修正。`summary.json`、`endpoint_metrics.json`、JSONL 和 PASS 裁决均未改变。

过渡窗口包含提示、物理动作和等待用户确认，不能解释为纯运动时长。

### 6.3 离线延迟基线

裁决：

```text
DETECTOR_OUTPUT_LATENCY_BASELINE_ESTABLISHED
```

定义：相机 header stamp 到 diagnostics recorder 收到消息的时间。

| 指标 | 结果 |
|---|---:|
| 记录数 | 5176 |
| 最小 | 24.18 ms |
| 中位 | 42.52 ms |
| P95 | 73.97 ms |
| 最大 | 504.32 ms |
| 超过 100 ms | 53（1.024%） |
| 超过 200 ms | 5（0.097%） |
| 超过 500 ms | 1（0.019%） |
| 负延迟 | 0 |
| 相机发布 FPS 中位 | 30.0 |
| 诊断接收 FPS | 19.40 |
| debug 覆盖 | 3994/5176（77.16%） |

这个时间包含相机到 detector 的传输、callback 调度、推理、诊断发布和记录器接收，
不是单独模型推理耗时。22 张保存 PNG 的完成年龄为 150.49-263.07 ms，中位
188.82 ms，但还包含 debug 传输、PNG 编码和写盘。没有 RViz 展示时间戳，不能
声称已经测得 RViz 渲染延迟。

## 7. 测试与模型加载状态

- 相关 ROS 包构建通过。
- 最新 ROS 汇总：`78 tests, 0 errors, 0 failures, 8 skipped`。
- 直接 YOLO 测试：`50 passed`。
- 使用 `${YOLO_ARMOR_MODEL}` 的真实 ROS 节点加载 smoke 已通过。
- 节点确认读取部署目录模型，并打印 `conf=0.4`、`gray_threshold=140`。
- 路径 smoke 使用 `warmup=false` 隔离模型位置验证；模型推理效果沿用已经完成的
  连续实机验收，不需要为迁移文档重复采集。

## 8. 当前 WSL 环境基线

```text
architecture: x86_64
Ubuntu: 22.04
ROS: Humble
Python: 3.10.12
PyTorch: 2.12.0+cu130
Ultralytics: 8.4.65
OpenCV: 4.13.0
NumPy: 2.2.6
GPU: RTX 5060 Laptop
NVIDIA driver: 592.01
MVS package: 2025-07-11 amd64
MvCameraControl: 4.8.0.3
```

`venv` 是项目隔离的 Python 环境，不是虚拟机。它保存项目需要的 PyTorch、
Ultralytics 等版本，避免污染系统 Python。系统 Python 当前没有 Ultralytics，启动
YOLO 节点前必须激活对应 venv。

## 9. mini 主机架构与推理后端

`x86_64` 是 CPU 指令集分类，不是 GPU 类型。Intel Core 和 AMD Ryzen 都属于
x86_64；Jetson 通常属于 ARM64/aarch64。它决定 Ubuntu、ROS、MVS SDK 和 Python
wheel 应下载哪种架构版本。

PyTorch GPU 版不能任意组合，必须匹配：

```text
GPU型号 -> 驱动 -> CUDA或ROCm运行库 -> 对应PyTorch构建 -> Python/Ubuntu
```

用户可以自主选择下载，但只能选择官方已经提供并支持的组合。

推荐判断表：

| mini 主机 | 后端方向 | 难度/边界 |
|---|---|---|
| x86_64 + NVIDIA | PyTorch CUDA，必要时 TensorRT | 最接近当前环境，通常最省事 |
| x86_64 + Intel CPU/iGPU | OpenVINO CPU/GPU | 需要 ONNX/OpenVINO 转换与一致性验证 |
| x86_64 + AMD GPU，型号在 ROCm 官方矩阵 | PyTorch ROCm | 需匹配 GPU、ROCm、驱动和 PyTorch |
| x86_64 + AMD GPU，不在 ROCm 官方矩阵 | PyTorch CPU 或其他 CPU runtime | AMD GPU 通常不能直接加速当前 PyTorch 模型 |
| ARM64 Jetson | JetPack + TensorRT | 架构、MVS ARM 包和模型转换工作最多 |

“AMD GPU 不支持 ROCm”不是指所有 AMD GPU，而是指具体型号没有出现在对应
ROCm 版本、Linux 版本和 PyTorch 构建的官方支持矩阵。很多旧卡、部分消费卡和
Ryzen APU 核显可能不在正式支持范围内，必须拿完整型号查矩阵。

OpenVINO 是 Intel 推理运行时，不是某一种 CPU 或 GPU。它主要面向 Intel CPU、
Intel GPU 和 Intel NPU。CPU 模式兼容通常容易但可能速度不足；Intel GPU/NPU 模式
需要对应设备和驱动。AMD GPU 通常不能作为 OpenVINO GPU 设备，即使 AMD CPU
可能仍可尝试 CPU 后端，也需要单独验证性能和官方支持边界。

ONNX 是模型交换格式，不是推理硬件。它用于把 PyTorch 模型交给 OpenVINO、
TensorRT、ONNX Runtime 等后端。

## 10. mini 主机型号确定后的唯一推进顺序

先让用户提供：

```text
mini 主机完整型号
CPU 完整型号
GPU/iGPU/NPU 完整型号
目标 Ubuntu 版本
uname -m 输出（拿到机器后）
```

然后按顺序执行：

1. 根据硬件选择 CUDA、ROCm、OpenVINO、TensorRT 或 CPU 后端。
2. 从干净 checkout 检出 `yolo-v8-gray140-candidate-20260826`。
3. 单独传输 `model.pt`，只核对部署清单中的一个模型校验值。
4. 安装匹配架构的 ROS、MVS SDK 和推理 venv。
5. 设置 `YOLO_ARMOR_MODEL`，构建工作区并运行配置契约测试。
6. 用已有离线素材做结果一致性，不重新训练模型。
7. 连接相机做短时 smoke：设备、分辨率、曝光、增益、时间戳和 topic。
8. 验证 `camera -> detector -> tracker -> safety -> gimbal/control` 完整链路。
9. 再在原生 Linux 上处理吞吐和延迟，不返回 WSL DDS/USB 排查路线。

mini 主机架构不确定不影响当前冻结，但会阻止最终运行环境和整链生产验收。

## 11. 当前启动方式

WSL 当前候选启动前需要：

```bash
cd /home/jo/my-project/rm_vision/rm_auto_aim
source /opt/ros/humble/setup.bash
source install/setup.bash
source /home/jo/my-project/my_first_autoaim_yolo/.venv/bin/activate
export YOLO_ARMOR_MODEL=/home/jo/my-project/rm_vision/rm_auto_aim/deployment/yolo_v8_gray140_candidate_20260826/model.pt
ros2 launch auto_aim_bringup yolo_auto_aim.launch.py \
  yolo_params:=$(ros2 pkg prefix auto_aim_bringup)/share/auto_aim_bringup/params/yolo_detector_params_v8_frozen_candidate.yaml
```

在 mini 主机上只替换 venv 和 `YOLO_ARMOR_MODEL` 的绝对路径，不修改候选 YAML
中的算法参数。

## 12. 数据、Git 与删除边界

本机保留：

```text
frame_rate_diagnostics: 8.8 GB
workflow_dry_runs: 1.5 GB
real_armor_diagnostics: 1.3 GB
deployment model: 6.2 MB
```

前三项主要是诊断、流程演练和真实装甲板验证产物，不是 mini 主机推理依赖。
它们仍可用于复查、回归或后续挑选训练素材，所以目前只是被 Git 忽略，没有删除。

真正训练数据主要位于 `my_first_autoaim_yolo`，也不等同于上述 11.6 GB 诊断目录。
模型推理只需要模型权重、代码、完整 YAML、相机标定和运行环境；重新训练或严格
复现训练时才需要训练图片、标签、数据划分和训练配置。

禁止执行：

- `git reset --hard` 或 `git checkout --` 回退用户工作。
- `git add .` 把大型诊断产物加入 Git。
- 删除上述诊断目录、模型或旧实验记录。
- 未经授权改写冻结标签或推送远程。
- 把通用 YAML 只改成 gray140 后当作完整候选部署。

## 13. 已结束或禁止重复的路线

- 不重新采集 gray140 正面/左右 45°/返回正面。
- 不重新打开 WSL FPS、DDS、USB 或 Fast DDS SHM 诊断。
- 不继续 v9-v9f“直接加入视频 12 并排列组合”的训练路线。
- 不重复读取 Dev06 选择模型，不重新扫描阈值或 checkpoint。
- 不因 RViz 看起来延迟就把 PNG 写盘年龄称为 RViz 延迟。
- 不在缺乏直接证据时修改模型、曝光、D、PnP、tracker、fill ratio 或配对门限。

历史固定指标仍保留：

```text
v8 Val05: Precision 0.9492, Recall 0.9032, F1 0.9256
v8 Dev06: Precision 0.9717, Recall 0.9196, F1 0.9450
视频11最终审核集: Precision 0.9455, Recall 0.6753, F1 0.7879
```

视频 11 不是严格独立盲测；不要混淆这些历史检测指标与当前连续实机端点通过率。

## 14. 用户偏好与监督规则

- 避免重复姿态、重复采集、重复预检和无新信息的复测。
- 正式记录开始前必须明确通知用户，并等待约定回复。
- 一次只改变一个有意义的变量。
- 禁止过度防御，不要对每个文件和每一步重复哈希。
- 可以做必要校验，但要说明校验解决了什么风险。
- 用户确认的物理左右方向必须原文保留，不能从画面左右推断。
- PnP 估算方向不是物理真值。
- 不清理、覆盖或回退无关用户改动。
- 相机或监督通道不可用时要明确报告，不能长期静默。

## 15. 新对话建议首条指令

```text
请先读取：
/home/jo/my-project/rm_vision/rm_auto_aim/docs/yolo_model_training_handoff_4_wsl_2026-08-26.md

你是监督端。当前候选已经在 WSL 冻结，Git 标签为：
yolo-v8-gray140-candidate-20260826

不要重新训练、采集、复测 gray140 姿态，也不要恢复 WSL 帧率诊断。先核对 Git
状态和冻结清单。下一阶段只有在我提供 mini 主机完整 CPU/GPU/架构后，才制定
对应的 CUDA、ROCm、OpenVINO、TensorRT 或 CPU 部署方案；随后按干净检出、
模型传输、离线一致性、相机 smoke 和完整链路验收的顺序执行。
```
