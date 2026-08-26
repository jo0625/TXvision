# RoboMaster 视觉自瞄 V0.1：项目边界、方案说明与下一步

更新时间：2026-07-25

## 1. 结论

当前不建议再新建第三个独立项目。

现有目录实际上已经形成了合理的两层分工：

```text
my_first_autoaim_yolo
  负责数据、标注、训练、评估、模型权重和训练环境

rm_vision/rm_auto_aim
  负责 ROS 运行节点、PnP、跟踪、瞄准、安全保护和整车集成
```

本次新增的运行代码都位于：

```text
/home/jo/my-project/rm_vision/rm_auto_aim
```

但当前运行时仍依赖训练项目中的两项资源：

```text
模型：/home/jo/my-project/my_first_autoaim_yolo/runs/pose/armor_pose_v2_with04/weights/best.pt
环境：/home/jo/my-project/my_first_autoaim_yolo/.venv
```

因此准确说法是：

- ROS 运行代码已经放进 `rm_vision/rm_auto_aim`。
- 数据、权重和 Ultralytics 虚拟环境仍放在 `my_first_autoaim_yolo`。
- 本次没有修改训练数据和模型权重。
- 本次读取了外部交接文档，但没有修改交接项目。

## 2. 本次在 rm_vision 中增加的内容

### 2.1 YOLO ROS 检测包

```text
yolo_armor_detector/
├── CMakeLists.txt
├── package.xml
└── scripts/yolo_armor_detector_node.py
```

节点功能：

```text
/image_raw + /camera_info
  -> sensor_msgs/Image 直接解码
  -> YOLO Pose 四点检测
  -> 四边形面积、灯条长度、关键点置信度检查
  -> 可选颜色检查
  -> 固定尺寸 IPPE PnP
  -> /detector/armors
  -> /detector/result_img
```

图像订阅使用 `Keep Last + depth=1 + Best Effort`，避免推理速度低于相机帧率时积压旧图像。

### 2.2 YOLO 启动文件和参数

```text
auto_aim_bringup/launch/yolo_auto_aim.launch.py
auto_aim_bringup/params/yolo_detector_params.yaml
```

该启动文件只启动 YOLO detector，不启动传统 `armor_detector_node`，避免两个 detector 同时发布：

```text
/detector/armors
```

### 2.3 目标超时保护

```text
auto_aim_safety/
├── CMakeLists.txt
├── package.xml
└── scripts/target_watchdog_node.py
```

当前完整 tracker 的原始输出被重映射为：

```text
/tracker/raw_target
```

watchdog 将新鲜消息转发到：

```text
/tracker/target
```

如果超过 0.5 秒没有收到 tracker 输出，watchdog 会：

```text
发布 tracking=false
请求 /tracker/reset
```

### 2.4 离线测试工具

```text
tools/yolo_offline_test_publisher.py
```

它重复发布指定测试图片和合成 `CameraInfo`，用于验证 ROS、YOLO、PnP 和 tracker 消息链路。

合成内参不能用于实车控制。

## 3. 当前方案为什么看起来和简单自瞄差别很大

YOLO 检测和 PnP 部分与简单自瞄路线一致：

```text
图像 -> 装甲四点 -> PnP -> 装甲三维位置
```

差别来自仓库已有的 `armor_tracker`。

这个 tracker 跟踪的不是单块装甲，而是整台机器人。其内部状态包含：

```text
机器人中心位置
机器人平移速度
装甲高度
机器人 yaw
旋转速度
装甲旋转半径
```

tracker 收到一块装甲后，会假设机器人中心位于装甲后方，并推算其他装甲的位置。

RViz 中：

- 绿色球表示估算的机器人中心。
- 红色板表示 tracker 推算的多块装甲。
- 红色板的数量不等于 YOLO 实际检测数量。
- `MarkerArray` 只是调试可视化，关闭显示不会停止算法。

该完整模型适合后期处理：

- 目标快速旋转。
- 装甲板交替出现。
- 短暂遮挡。
- 运动预测。
- 反陀螺。

但它不是当前 V0.1 静止小装甲瞄准的必要条件。

## 4. 推荐的 V0.1 主链

V0.1 推荐改为单装甲直接瞄准：

```text
真实相机
  -> YOLO Pose 四点检测
  -> 固定小装甲 PnP
  -> 选择离图像中心最近的装甲
  -> 简单低通滤波
  -> 相机坐标转 yaw/pitch
  -> 加相机—枪管偏置
  -> 失锁保护和角度限幅
  -> 云台
  -> 人工授权开火
```

第一阶段不应直接用 `/tracker/target.position` 控制枪口，因为它表示完整 tracker 估算的机器人中心，不一定是当前可见装甲的瞄准位置。

完整 tracker 保留在仓库中，作为 V0.2 或后续移动、旋转目标功能使用。

## 5. 当前推荐的目录结构

不新建第三个项目，而是在现有 `rm_auto_aim` 内继续按 ROS 包隔离：

```text
rm_auto_aim/
├── auto_aim_interfaces/       # ROS 消息接口
├── yolo_armor_detector/       # YOLO 四点检测和 PnP
├── simple_aim/                # 下一步：单装甲滤波和 yaw/pitch
├── auto_aim_safety/           # 超时、失锁和安全输出
├── armor_tracker/             # 完整整车 EKF，V0.2 使用
├── armor_detector/            # 传统 detector，作为备用后端
├── auto_aim_bringup/          # 启动文件和参数
├── tools/                     # 离线测试工具
└── docs/                      # 设计和上机记录
```

其中 `simple_aim/` 尚未实现，是 PnP 实车验证通过后的下一项代码任务。

## 6. 为什么现在不建议新开项目

现在拆成第三个项目会增加以下成本：

- `auto_aim_interfaces` 需要重复安装或跨工作区 source。
- launch、参数和 TF 配置分散。
- 更容易出现当前已经遇到过的消息类型找不到问题。
- detector、瞄准节点和安全节点版本容易不一致。
- 上机时需要维护更多工作区和启动顺序。

当前通过独立 ROS package 已经可以获得足够的模块隔离：

```text
独立编译
独立启动
独立参数
独立测试
可以随时替换 detector 或 tracker
```

只有出现以下情况时，才建议把运行侧单独拆成新的 Git 仓库：

- 需要给另一台车或另一支队伍独立发布。
- 训练项目和机器人运行项目由不同人员长期维护。
- 需要独立版本号、CI、Release 和部署包。
- `rm_auto_aim` 上游需要频繁同步，而本地改动难以维护。

即使以后拆仓库，也应继续保持“训练仓库”和“ROS 运行仓库”两类，而不是为每个节点单独开项目。

## 7. 模型文件后续如何处理

当前参数直接引用训练项目中的 `best.pt`，开发阶段可以继续使用。

上机版本稳定后，建议增加一个独立的运行模型目录，例如：

```text
/home/jo/my-project/rm_vision/runtime_models/armor_pose_v2/best.pt
```

然后将 launch 参数指向这个稳定模型，避免重新训练或清理 `runs/` 后导致实车节点无法启动。

大模型文件不建议直接放入普通 Git 历史。可以采用：

- Git LFS。
- Release 附件。
- 本地部署脚本。
- 明确校验和的外部模型目录。

## 8. 下一步执行顺序

### 8.1 暂时关闭完整 tracker

真实相机 PnP 验证阶段只启动 detector：

```bash
ros2 launch auto_aim_bringup yolo_auto_aim.launch.py \
  start_tracker:=false \
  start_target_watchdog:=false
```

此时使用：

```text
/detector/result_img
/detector/armors
```

暂时不使用：

```text
/tracker/target
/tracker/marker
```

### 8.2 接真实相机并确认话题

```bash
ros2 topic list | grep -E 'image|camera_info'
```

检查图像频率：

```bash
ros2 topic hz <真实图像话题>
```

读取并保存真实内参：

```bash
ros2 topic echo <真实camera_info话题> --once
ros2 topic echo <真实camera_info话题> --once > ~/camera_info_real.yaml
```

必须记录：

```text
width
height
K
D
frame_id
```

### 8.3 使用真实话题启动 detector

示例：

```bash
ros2 launch auto_aim_bringup yolo_auto_aim.launch.py \
  image_raw:=/camera/image_raw \
  camera_info:=/camera/camera_info \
  start_tracker:=false \
  start_target_watchdog:=false
```

实际话题名必须以相机驱动输出为准。

### 8.4 验证 2—3 m PnP

```bash
ros2 topic echo /detector/armors
```

建议记录：

| 测试位置 | 预期结果 |
|---|---|
| 2 m 中央 | `x≈0`、`y≈0`、`z≈2 m` |
| 3 m 中央 | `z≈3 m` |
| 2 m 左侧 | `x<0` |
| 2 m 右侧 | `x>0` |
| 2 m 上方 | `y<0` |
| 2 m 下方 | `y>0` |

只有距离和方向都正确后，才能把 PnP 输出送入云台控制。

### 8.5 实现 simple_aim 节点

下一节点应完成：

```text
订阅 /detector/armors
选择单块目标
计算 yaw/pitch
低通滤波
相机—枪管偏置
角度和角速度限幅
100—200 ms 失锁保护
发布 valid=false 安全状态
```

相机光学坐标下的初始角度公式：

```text
yaw = atan2(x, z)
pitch = -atan2(y, sqrt(x² + z²))
```

实际正负号需要结合云台电机方向验证。

### 8.6 只接云台，不接发射

第一阶段只验证：

- 云台左右方向正确。
- 云台上下方向正确。
- 目标位于中心时输出接近零。
- 装甲丢失后立即停止或进入安全状态。
- 相机—枪管 yaw/pitch 偏置可重复。
- 输出没有明显振荡。

发射机构继续保持断开或人工授权。

## 9. V0.1 验收标准

```text
[ ] 真实 CameraInfo 已保存
[ ] 2 m 和 3 m PnP 距离通过
[ ] 左右、上下坐标方向通过
[ ] result_img 稳定输出
[ ] 单装甲 yaw/pitch 计算正确
[ ] 简单滤波后无明显抖动
[ ] 失锁 100—200 ms 后输出无效
[ ] 云台可以瞄准静止装甲
[ ] 人工开火授权仍然有效
[ ] 完整 tracker 尚未参与首次实车控制
```

## 10. 当前最需要采集的信息

继续开发 `simple_aim` 前，需要获得：

```bash
ros2 topic list | grep -E 'image|camera_info'
```

```bash
ros2 topic echo <真实camera_info话题> --once
```

以及装甲放在已知 2 m 中央位置时：

```bash
ros2 topic echo /detector/armors --once
```

还需要确认云台控制接口：

```text
控制话题或串口协议
消息类型
yaw/pitch 单位
正方向定义
控制模式（角度、角速度或电流）
```

这些信息确认后，再实现 `simple_aim` 和云台接口，避免在相机内参、坐标系和控制方向尚不明确时把错误数值送入实车。
