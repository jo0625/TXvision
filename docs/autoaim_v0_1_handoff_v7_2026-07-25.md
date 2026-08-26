# RoboMaster 视觉自瞄 V0.1 交接记录（v7）

更新时间：2026-07-25

## 1. 当前结论

真实海康相机到 ROS 2、YOLO Pose 和 RViz 的二维检测链路已经跑通。

当前模型状态：

```text
ROS 当前仍使用：v2
最新候选模型：v7
v7 候选阈值：conf=0.40
```

v7 在开发基准 `test_06` 上明显优于 v2，但尚未在保留的最终测试集
`test_03` 上完成一次性评估，因此暂时不修改 ROS 默认模型。

## 2. 项目位置

ROS 运行项目：

```text
/home/jo/my-project/rm_vision/rm_auto_aim
```

YOLO 训练、数据、虚拟环境和模型：

```text
/home/jo/my-project/my_first_autoaim_yolo
```

不需要再新建第三个项目。训练和 ROS 运行继续保持分离。

## 3. 相机与 ROS 验证状态

相机：

```text
型号：Hikrobot MV-CS016-10UC
序列号：DA8541046
USB VID:PID：2bdf:0001
Windows usbipd BUSID：2-3
分辨率：1440 x 1080
原始像素格式：BayerRG8
ROS 编码：bgr8
```

已实现 ROS 2 package：

```text
rm_auto_aim/hik_camera
```

主要话题：

```text
/image_raw
```

实测结果：

```text
相机节点日志：约 47—51 FPS
ros2 topic hz /image_raw：约 31.3 FPS
RViz /image_raw：画面正常
```

当前相机参数基线：

```text
frame_rate：60 FPS
exposure_time_us：5000
gain_db：8
Trigger Mode：Off
```

采集较暗场景负样本时临时测试过：

```text
exposure_time_us：12000
gain_db：12
```

上述高曝光参数仅用于数据采集参考。拿到真实装甲后需要重新根据灯条过曝和运动模糊调节。

运行 ROS 相机节点前必须关闭 MVS 客户端，避免相机被独占。

## 4. YOLO ROS 实时验证

检测节点：

```text
yolo_armor_detector/scripts/yolo_armor_detector_node.py
```

输出：

```text
/detector/result_img
/detector/armors
```

真实相机测试结果：

```text
YOLO avg_inference_ms：约 14.6—14.9 ms
/detector/result_img：RViz 正常显示
无装甲场景：detections=0
```

未完成相机标定，因此日志会周期性出现：

```text
Waiting for /camera_info; PnP/Armors output is empty
```

这是正常状态。二维检测正常运行，但 PnP 和三维坐标暂不验收。

完整检测结果发布帧率低于模型纯推理速度，主要开销来自 1440x1080 ROS 图像反序列化、内存复制和结果图发布。该问题不阻塞模型训练，但接云台前需要优化。

## 5. 新增数据工具

### 5.1 ROS 低频图像采样

```text
rm_auto_aim/tools/capture_ros_images.py
```

用途：从 `/image_raw` 低频保存 JPEG，避免录制体积巨大的原始 rosbag。

### 5.2 YOLO Pose 自动预标注

```text
/home/jo/my-project/my_first_autoaim_yolo/scripts/auto_label_pose.py
```

功能：

- 保留已有人工标签。
- 只对未标注图片生成预标签。
- 过滤低关键点置信度和非法四边形。
- 将预测关键点置信度转换为数据集要求的可见性值 `2`。
- 无可靠检测的图片保持未标注，不自动写成错误空样本。

自动预标注只能减少点击量，所有标签仍需人工审核。

## 6. 当前数据划分

v6/v7 使用的数据配置：

```text
/home/jo/my-project/my_first_autoaim_yolo/config/armor_pose_v6.yaml
```

数据集：

```text
/home/jo/my-project/my_first_autoaim_yolo/datasets/armor_pose_v6_positive_hard_negative
```

统计：

```text
Train：413 张
  空样本：129 张
  装甲目标：482 个

Val：146 张
  空样本：62 张
  装甲目标：124 个

Dev test_06：121 张
  空样本：32 张
  装甲目标：112 个
```

检查结果：

```text
缺失标签：0
17 字段格式错误：0
重复关键点：0
```

视频级划分：

```text
Train：01、02、04、07、08 + 独立训练硬负样本
Val：05 + 独立验证硬负样本
Dev benchmark：06
Final test：03
```

同一视频的连续帧不能跨 Train、Val、Test。

## 7. 07 与 08 新视频标签

视频信息：

```text
armor_train_07.mp4：1280x720，30 FPS，36.7 秒
armor_train_08.mp4：1280x720，29.97 FPS，35.2 秒
```

按每 15 帧抽取一张：

```text
07：74 张原始抽帧
08：71 张原始抽帧
```

最终标签：

```text
07：65 张正样本，101 个装甲，9 张跳过
08：52 张正样本，19 张空样本，52 个装甲
```

07 为人工标注。08 先用 v2 自动预标注，再逐帧人工审核。

新视频排除纯黑转场后，与 `test_03`、`test_06` 未发现内容级相似匹配。

## 8. 模型迭代结论

### v2

```text
路径：runs/pose/armor_pose_v2_with04/weights/best.pt
```

原有基线，Recall 较好，但文字、Logo 和部分背景误报较多。

### v4

修复重复关键点，并按完整视频重新划分。总体 F1 略升，但连续片尾文字背景误报恶化，不能替换 v2。

### v5

加入真实相机采集的文字、Logo、二维码、屏幕等硬负样本。背景误报下降，但 Recall 下降。

### v6

在 v5 基础上继续微调，并加入 07、08 正样本。定位和关键点更准，但模型进一步变得保守，Recall 继续下降。

### v7

没有从 v5/v6 继续微调，而是使用完整 v6 数据从官方：

```text
yolov8n-pose.pt
```

重新训练，解决了连续微调造成的偏置和类似灾难性遗忘问题。

模型路径：

```text
/home/jo/my-project/my_first_autoaim_yolo/runs/pose/armor_pose_v7_full_retrain/weights/best.pt
```

## 9. test_06 模型对比

`test_06` 已用于模型和阈值选择，因此它现在是开发基准，不再是最终独立测试集。

| 模型 | conf | TP | FP | FN | Precision | Recall | F1 | 背景干净率 |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| v2 | 0.60 | 96 | 10 | 16 | 0.906 | 0.857 | 0.881 | 90.6% |
| v5 | 0.55 | 93 | 9 | 19 | 0.912 | 0.830 | 0.869 | 93.8% |
| v6 | 0.35 | 90 | 11 | 22 | 0.891 | 0.804 | 0.845 | 93.8% |
| **v7** | **0.40** | **101** | **3** | **11** | **0.971** | **0.902** | **0.935** | **96.9%** |

v7 相对 v2：

```text
TP：+5
FP：-7
FN：-5
F1：+5.45 个百分点
完整帧正确率：90.9%
mean_iou：0.8065
mean_keypoint_error：0.00608
```

v7 的 `conf=0.35` 和 `conf=0.40` 在 test_06 上结果相同，因此候选使用稍保守的：

```text
conf=0.40
```

## 10. 最终测试集 test_03

保留目录：

```text
/home/jo/my-project/my_first_autoaim_yolo/data/benchmark_03_frames/armor_test_03
```

`test_03` 不允许加入训练或验证，也不允许使用 YOLO 自动预标注。

人工标注命令：

```bash
cd /home/jo/my-project/my_first_autoaim_yolo
source .venv/bin/activate

python scripts/label_pose_opencv.py \
  --source data/benchmark_03_frames/armor_test_03 \
  --scale 0.75
```

标注规则：

- 四角清楚的真实装甲：人工标注。
- 无装甲或只有屏幕内容：保存空标签。
- 真实装甲存在但无法可靠判断四角：跳过，之后不放入最终测试子集。
- 不查看任何模型预测，避免最终测试标签受到模型影响。

标注完成后建立 `test_03_curated`，只包含有明确人工结论的图片。

最终一次性评估固定使用：

```text
v2：conf=0.60
v5：conf=0.55
v7：conf=0.40
```

不得根据 `test_03` 结果再次修改阈值。若继续调参，需要另找新的最终测试视频。

## 11. 标签规则

当前 YOLO 类别保持：

```text
0: armor
```

红色和蓝色装甲都标为 `armor`。颜色不是当前 YOLO 类别，后续由传统颜色判断 `detect_color` 处理。

视频颜色、远近、运动、环境仅作为数据属性记录，不作为类别。

统一标注标准：

```text
真实装甲，四角清楚             -> 标注
真实装甲，超出射程但四角清楚   -> 仍然标注
真实装甲，太小或模糊无法标四角 -> 跳过
屏幕中显示的装甲               -> 空标签
没有装甲                       -> 空标签
纯黑、严重模糊、近似重复       -> 可跳过
```

射程限制应由 PnP 距离和控制端安全门限处理，而不是把远距离装甲定义成新的 YOLO 类别。

## 12. ROS 当前模型与后续切换

当前 ROS 参数文件：

```text
/home/jo/my-project/rm_vision/rm_auto_aim/auto_aim_bringup/params/yolo_detector_params.yaml
```

当前仍指向 v2，暂不修改。

只有 v7 通过 `test_03_curated` 最终评估后，才候选修改为：

```text
model: /home/jo/my-project/my_first_autoaim_yolo/runs/pose/armor_pose_v7_full_retrain/weights/best.pt
conf: 0.40
```

切换后仍需使用真实相机检查：

```text
/detector/result_img
/detector/armors
实时处理帧率
屏幕、文字、Logo 和机器人灯光误报
```

## 13. 当前未完成事项

1. 人工标注并整理 `test_03_curated`。
2. 对 v2、v5、v7 做一次性最终评估。
3. 若 v7 通过，更新 ROS 模型路径和 `conf=0.40`。
4. 使用真实相机重新验证二维结果图和误报。
5. 获取真实装甲板后完成相机标定。
6. 发布真实 `/camera_info`。
7. 在 2—3 m 验证 PnP 距离和坐标方向。
8. 实现 `simple_aim`，先只接云台，不连接发射。

## 14. 下次继续时的第一件事

```text
从 test_03 人工标注继续。
不使用自动预标注。
完成后报告标签数量，由 Codex 建立 test_03_curated 并执行最终模型对比。
```
