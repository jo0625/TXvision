# YOLO 自瞄 V0.1 运行与上机检查

本链路只启动一个检测后端：`yolo_armor_detector`。不要同时启动传统
`armor_detector_node`，否则两个节点会同时发布 `/detector/armors`。

## 已实现链路

```text
/image_raw + /camera_info
  -> YOLO Pose 四点检测（图像 QoS depth=1）
  -> 几何/颜色检查
  -> 固定尺寸 IPPE PnP
  -> /detector/armors
  -> armor_tracker
  -> /tracker/raw_target
  -> target_watchdog（0.5 s 超时，失效时 tracking=false，并请求 reset）
  -> /tracker/target
```

`/tracker/target` 是供控制端使用的安全输出。控制端仍必须保留人工开火授权，
并再次检查消息时间戳、`tracking`、云台限位和发射机构状态。

## 构建

```bash
source /opt/ros/humble/setup.bash
cd /home/jo/my-project/rm_vision/rm_auto_aim
colcon build --symlink-install --packages-up-to auto_aim_bringup \
  yolo_armor_detector auto_aim_safety
```

## 启动

必须先 source ROS 工作区，再激活含 `ultralytics` 的虚拟环境。虚拟环境需要能
导入 `rclpy` 和 `auto_aim_interfaces`。

```bash
source /opt/ros/humble/setup.bash
source /home/jo/my-project/rm_vision/rm_auto_aim/install/setup.bash
source /home/jo/my-project/my_first_autoaim_yolo/.venv/bin/activate

ros2 launch auto_aim_bringup yolo_auto_aim.launch.py \
  image_raw:=/image_raw \
  camera_info:=/camera_info
```

真实机器人必须提供 `camera_optical_frame -> odom`（或 tracker 参数指定目标系）
的动态/静态 TF。只有台架离线测试时才可临时启用 launch 内的光学坐标转换：

```bash
ros2 launch auto_aim_bringup yolo_auto_aim.launch.py start_static_tf:=true
```

## 上机前检查

1. 确认话题和帧名：

   ```bash
   ros2 topic list | grep -E 'image_raw|camera_info|detector|tracker'
   ros2 topic echo /camera_info --once
   ros2 run tf2_ros tf2_echo odom camera_optical_frame
   ```

2. 保存实际 `CameraInfo` 的分辨率、`K`、`D` 和 `frame_id`。节点收到或发生变化时
   会把这些值打印到日志。
3. 在已知 2—3 m 距离放置静止小装甲：相机光学系中 `z` 应为正且接近实测距离；
   向图像右移动时 `x` 增大，向下移动时 `y` 增大。
4. 只闭环云台，不接发射。标定相机光轴相对枪管的 yaw/pitch 偏置。
5. 停止 tracker 或检测节点，确认 0.5 s 后 `/tracker/target.tracking` 变为 `false`。
6. 检查处理帧率和 `avg_inference_ms`。若 GPU 推理慢于相机帧率，depth=1 会丢弃
   积压旧帧，优先处理最新画面。

## 当前限制

- 模型只按单色、单目标、固定小装甲、约 2—3 m、静止或低速场景验收。
- 默认 `detect_color=-1`；确认真实相机颜色和曝光后再设为红 `0` 或蓝 `1`。
- `armor_number=armor` 是单目标占位 ID，不支持数字分类和多目标身份保持。
- 尚未完成真实相机/rosbag 的距离、坐标方向、帧率和端到端延迟实测。
- 不包含完整弹道补偿，也不会自动授权开火。
