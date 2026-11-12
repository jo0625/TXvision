# 云台抖动分析界面

保留历史分析和 ROS 实时观察。该工具只订阅消息，不打开串口、不发送目标角，
也不切换云台模式。它不启动或修改现有通信桥接节点。

## 打开界面

在 rm_auto_aim 仓库根目录运行：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$PWD/tools/handeye_calibration/cyclonedds_wsl_local.xml
python3 tools/gimbal_scope/serve_scope.py --port 8765
```

浏览器打开 http://127.0.0.1:8765 。端口已占用时选择其他端口。
终端 Ctrl+C 仅停止这个观察服务，不停止其他 ROS 节点。

2026-09-23 本次已将界面启动为当前 WSL 会话的用户服务
`rm-gimbal-scope.service`，地址为 http://127.0.0.1:8765 。可以用
`systemctl --user status rm-gimbal-scope` 查看，或用
`systemctl --user stop rm-gimbal-scope` 关闭；该临时服务未设置开机自启。

单独生成离线文件：

```bash
python3 tools/gimbal_scope/build_scope.py
```

产物为 `frame_rate_diagnostics/gimbal_scope.html`，直接用浏览器打开即可看历史。
所有曲线库和两份已有记录均嵌入文件，不需要联网；实时入口需要上述本地服务。

## 观察方法

- 默认显示 09-23 异响记录的 13–17 秒，即停止转发后的区间。
- 绿色是实际反馈角，橙色是发往通信节点的目标角；目标角不等于 MCU 内部设定角。
- 下方分别显示模式和上位机写入活动。后者不是电机使能状态或下位机接收确认。
- 切换 Pitch/Yaw、拖动底部范围或滚轮缩放，可以观察变向附近的局部变化。
- 纵轴根据所选区间自动调整。Yaw 会按最短角差连续展开，避免 ±180° 跳变。
- “保存”保留整份历史记录，或实时缓冲内最近 120 秒的数据；暂停时保存冻结的缓冲。
- CSV 导出所选区间，PNG 导出当前曲线。打开记录支持相同 `rows` 格式的 JSON。
- 实时暂停只冻结显示，后台仍接收；继续后返回最新窗口。长时间记录请另用正式记录器。

实时只订阅以下主题，需与现有 ROS 图处于同一个 ROS_DOMAIN_ID：

| 主题 | 用途 |
| --- | --- |
| /serial_bridge/feedback | 主机收到的下位机 yaw、pitch、模式、序号与 MCU 时间戳 |
| /serial_bridge/aim_command | 发给通信节点的 valid=true 目标角 |
| /serial_bridge/command_active | 通信桥报告的发送活动 |

## 数值含义与边界

文件中角度为弧度，界面显示度。横轴使用主机单调时钟的接收时间；
MCU 时间戳另存于反馈行和 CSV，不直接与主机时间相减。

统计区间为 `[开始, 结束)`。峰峰值是最大角减最小角，标准差使用总体标准差。
相邻变化中位数只使用间隔不超过 150 ms 的相邻反馈，避免把断流后的变化当作抖动。
到达频率为 `(点数-1)/(末点时间-首点时间)`，最长间隔包含真实反馈空隙。
反馈和目标角超过 150 ms 的空隙均断线；目标线不向停令之后延伸。
模式依赖反馈，断流处也断线。发送活动是事件状态，仅代表最后收到的活动记录。

这些统计包含主动运动，不能直接叫作“机械抖动强度”。约 20 Hz 反馈不足以判定
高频机械振动频率；目前记录没有电流、MCU 内部设定角和音频，不能据此唯一确定 PID 根因。
09-22 记录是对照材料，不是已验收的无故障基线。

09-23 的七次零值目标仅因已有诊断结论标记为误转发；普通记录中的零角度不自动视为错误。

## 验证

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 -m pytest tools/gimbal_scope/test_scope.py -q
```

测试使用独立 ROS_DOMAIN_ID=195 和随机 HTTP 端口，在隔离图中发布模拟反馈；
不会向现有机器人通信图发布测试目标。覆盖历史统计、导入导出、缩放、窄屏布局、
实时到达、暂停、断流，以及观察节点无控制主题发布。

第三方库：ECharts 5.6.0（Apache-2.0）、Lucide 0.468.0（ISC），本地文件位于 `vendor/`。
