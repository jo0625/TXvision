# RoboMaster 视觉自瞄 V0.1 当前进度

更新时间：2026-07-25

## 1. 当前目标

V0.1 目标保持为：

```text
单一敌方颜色
单台目标机器人
固定小装甲板
距离约 2—3 m
静止或低速目标
先只控制云台瞄准
人工授权开火
```

当前推荐的首次上机主链：

```text
海康真实相机
  -> YOLO Pose 四点检测
  -> 固定小装甲 PnP
  -> 单装甲选择
  -> 简单低通滤波
  -> yaw/pitch
  -> 云台
  -> 人工授权开火
```

完整四装甲 EKF tracker 暂时保留，但不作为 V0.1 第一次云台控制的输入。

## 2. 项目边界

ROS 运行代码位于：

```text
/home/jo/my-project/rm_vision/rm_auto_aim
```

YOLO 训练项目、模型和虚拟环境位于：

```text
/home/jo/my-project/my_first_autoaim_yolo
```

当前使用模型：

```text
/home/jo/my-project/my_first_autoaim_yolo/runs/pose/armor_pose_v2_with04/weights/best.pt
```

当前不需要再新建第三个项目。训练和 ROS 运行已经自然分开。

## 3. 已完成的 ROS 代码

### 3.1 YOLO ROS detector

新增：

```text
yolo_armor_detector/
```

主要节点：

```text
yolo_armor_detector/scripts/yolo_armor_detector_node.py
```

功能：

- 订阅 `/image_raw` 和 `/camera_info`。
- 不依赖 `cv_bridge`，直接解析 `sensor_msgs/Image`。
- 使用 YOLO Pose 检测装甲四个角点。
- 进行四边形面积、灯条长度和关键点置信度检查。
- 可选传统颜色检查，当前默认关闭：`detect_color=-1`。
- 使用固定小装甲尺寸进行 IPPE PnP。
- 发布 `/detector/armors`。
- 发布 `/detector/result_img`。
- 图像 QoS 使用 Best Effort、Keep Last、depth=1，避免旧帧积压。
- 解码或推理失败时发布空 `Armors`，避免下游继续使用旧目标。

### 3.2 启动文件

新增：

```text
auto_aim_bringup/launch/yolo_auto_aim.launch.py
auto_aim_bringup/params/yolo_detector_params.yaml
```

该 launch 不启动传统 detector，避免两个 detector 同时发布：

```text
/detector/armors
```

### 3.3 target watchdog

新增：

```text
auto_aim_safety/
```

完整 tracker 的原始输出被重映射为：

```text
/tracker/raw_target
```

watchdog 输出：

```text
/tracker/target
```

超过 0.5 秒没有 tracker 输出时：

- 发布 `tracking=false`。
- 请求 `/tracker/reset`。

### 3.4 离线测试工具

新增：

```text
tools/yolo_offline_test_publisher.py
```

它发布指定测试图片和合成 `CameraInfo`，仅用于验证 ROS/PnP 消息链路。

合成内参不能用于实车控制。

## 4. 已完成验证

### 4.1 构建和语法检查

以下包构建成功：

```text
yolo_armor_detector
auto_aim_safety
auto_aim_bringup
```

Python 语法检查通过。

### 4.2 静态图片端到端测试

测试图像：

```text
/home/jo/my-project/my_first_autoaim_yolo/data/final_test_06_frames/armor_test_06/armor_test_06_000000.jpg
```

测试结果：

```text
YOLO 检测到 1 个合法装甲
/detector/armors 正常发布
tracker 进入 tracking=true
停止输入约 0.509 s 后 watchdog 发布 tracking=false
watchdog 成功请求 tracker reset
RViz 可显示 /detector/result_img
RViz 可显示 /tracker/marker
```

### 4.3 对完整 tracker 的结论

RViz 中：

- 绿色球是 tracker 估计的机器人中心。
- 红色板是 tracker 推算的多块装甲。
- 四块红板不代表 YOLO 检测到了四块装甲。
- `MarkerArray` 只是可视化显示。

现有 tracker 跟踪整台机器人，不是只跟踪当前可见装甲。

V0.1 暂时不应直接使用 `/tracker/target.position` 控制枪口，因为该位置表示估算的机器人中心，不一定是当前可见装甲的瞄准位置。

## 5. 真实相机当前状态

相机型号：

```text
Hikrobot MV-CS016-10UC
```

接口：

```text
USB3 Vision
```

USB 标识：

```text
VID:PID = 2bdf:0001
```

Windows usbipd BUSID：

```text
2-3
```

Windows 已执行强制共享：

```text
Shared (forced)
```

相机已成功连接到 WSL，`lsusb` 输出包含：

```text
Bus 002 Device 003: ID 2bdf:0001 Hikrobot MV-CS016-10UC
```

注意：Device 编号可能在重新连接后变化，不能长期假设固定为 `003`。

## 6. MVS SDK 当前状态

已下载并安装：

```text
MVS V5.0.1 Linux x86_64
MVS-5.0.1_x86_64_20260512.deb
```

安装位置：

```text
/opt/MVS
```

SDK 头文件和库已存在：

```text
/opt/MVS/include/MvCameraControl.h
/opt/MVS/lib/64/libMvCameraControl.so.4.8.0.3
```

MVS 客户端启动脚本：

```bash
cd /opt/MVS/bin
./MVS.sh
```

不要直接运行 `/opt/MVS/bin/MVS`，因为包装脚本负责设置其私有 Qt 和动态库路径。

USB udev 规则已建立：

```text
/etc/udev/rules.d/80-drivers-SDK-2bdf.rules
```

当前 USB 设备权限已确认为：

```text
MODE=0666
GROUP=plugdev
```

## 7. MVS 客户端当前状态

MVS 客户端已经能够识别：

```text
MV-CS016-10UC(DA8541046)
```

设备列表显示绿色在线状态。

当前界面状态：

```text
Trigger Mode = Off
图像区域仍为黑色
FPS = N/A
尚未确认连续采集成功
```

连续采集按钮位于相机图像窗口上方工具栏的最左侧，为圆形箭头图标。

当前停点：

```text
点击连续采集
确认 FPS 出现数值
确认实时画面正常显示
```

中文界面尚未切换成功，但不影响相机取图和 SDK 使用。

## 8. USB 转发常用命令

`usbipd` 命令只能在 Windows PowerShell 中执行，不能在 WSL 中执行。

### Windows 管理员 PowerShell

连接到 WSL：

```powershell
usbipd attach --wsl --busid 2-3
```

检查状态：

```powershell
usbipd list
```

从 WSL 断开：

```powershell
usbipd detach --busid 2-3
```

### WSL

检查设备：

```bash
lsusb
```

海康 USB3 Vision 相机不是普通 UVC 相机，通常不会产生：

```text
/dev/video0
```

因此不要使用 `usb_cam` 或 V4L2 作为该相机的正式驱动。

## 9. 下一步执行顺序

### 9.1 完成 MVS 连续取图测试

启动：

```bash
cd /opt/MVS/bin
./MVS.sh
```

在客户端中：

1. 确认设备为绿色在线。
2. 保持 `Trigger Mode = Off`。
3. 点击图像窗口上方最左侧圆形箭头。
4. 确认 `FPS N/A` 变为实际帧率。
5. 确认实时画面出现。

取图成功后记录：

```text
Width
Height
Pixel Format
Resulting Frame Rate
Exposure Time
Gain
```

### 9.2 新增海康 ROS 2 相机包

计划在现有仓库增加：

```text
rm_auto_aim/hik_camera/
```

功能计划：

- 使用 `/opt/MVS/include/MvCameraControl.h`。
- 链接 `/opt/MVS/lib/64/libMvCameraControl.so`。
- 按序列号 `DA8541046` 选择相机，避免误连其他设备。
- 连续取图并转换为 ROS `sensor_msgs/Image`。
- 发布 `/image_raw`。
- 使用 Best Effort、Keep Last、depth=1。
- 使用 `camera_optical_frame`。
- 后续从标定 YAML 发布 `/camera_info`。
- 提供曝光、增益、帧率、像素格式等 ROS 参数。

### 9.3 相机标定

ROS 相机节点正常发布图像后，需要用标定板得到真实：

```text
分辨率
K
D
frame_id
```

保存真实标定文件后才允许进行 PnP 实车距离验证。

### 9.4 真实 PnP 验证

装甲放在已知位置，记录：

| 测试位置 | 预期结果 |
|---|---|
| 2 m 中央 | `x≈0`、`y≈0`、`z≈2 m` |
| 3 m 中央 | `z≈3 m` |
| 2 m 左侧 | `x<0` |
| 2 m 右侧 | `x>0` |
| 2 m 上方 | `y<0` |
| 2 m 下方 | `y>0` |

### 9.5 实现 simple_aim

PnP 验证通过后新增：

```text
rm_auto_aim/simple_aim/
```

计划功能：

```text
订阅 /detector/armors
选择离图像中心最近的单块装甲
计算 yaw/pitch
低通滤波
相机—枪管偏置
角度和角速度限幅
100—200 ms 失锁保护
发布安全瞄准输出
```

初始角度公式：

```text
yaw = atan2(x, z)
pitch = -atan2(y, sqrt(x² + z²))
```

### 9.6 只接云台

第一阶段：

- 不连接发射机构，或保持发射机构禁用。
- 验证 yaw/pitch 正负方向。
- 标定相机光轴与枪管偏置。
- 确认失锁后立即停止跟随。
- 确认输出限幅和机械限位有效。

## 10. 下次继续时的第一件事

从以下动作继续：

```text
在 MVS 中点击连续采集
确认实时画面和实际 FPS
记录相机分辨率、像素格式、帧率、曝光和增益
```

确认厂商客户端取图正常后，下一项代码任务是实现 `hik_camera` ROS 2 package。
