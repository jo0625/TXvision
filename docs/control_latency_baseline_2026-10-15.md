# 双轴视觉控制基础延迟基线：代码已接入，实机采样待完成

本轮在现有主机计时实现和 MCU 接入组件上继续核查与修补，仅改测量、统计及必要记录。
没有调整控制逻辑、控制频率、目标角度、PID、相机参数，
没有连接串口、启动运动测试或刷写固件。默认不启用日志，默认报文仍为原来的 11 字节。

## 真实路径核对

目前的摄像头识别和机械试验是分开的，不能当作已验证贯通的一条链。

1. `ban/outputs/start_armor_position_preview.sh` 启动 `hik_camera` 和 YOLO，明确
   `start_tracker=false`，没有启动串口或控制器。因此该识别测试本身不发送运动命令。
2. 可连接的 ROS 路径为：`hik_camera` → `YoloArmorNode.image_callback` →
   `solve_armor` 或 `raw_track_latch.solve_light_pnp` → `/detector/armors` →
   `ArmorTrackerNode.armorsCallback` → watchdog → `BasicAimerNode.target_callback` →
   `ControllerCommandPreview.command_callback` → 已有的受限跟随转发（例如
   `tools/dynamic_follow_test.py`）→ `/serial_bridge/aim_command` →
   `SerialBridgeNode.command_callback` → `CommandGate.update`（此处实际打包）→
   原有发送定时器 `write_command` → `port.write` 或 `queue_write`。
   不为采样自动接通该路径，不调整原有使能、坐标变换、门限或转发策略。
   若转发器启用了角度死区，它可能沿用旧角度而保留新输入帧的 header；本次关联定义是
   “该输入帧到发送”，不是沿用角度最初产生的时刻。应在条件快照中记录死区，并区分
   未改变目标的心跳命令，不能将它们说成新的机械阶跃。
3. tracker、watchdog 的正常转发、aimer、preview 和现有动态转发保留源图像的
   `header.stamp`，可作为同一主机内部的帧关联键；watchdog 超时会发新的无效消息，
   不将它关联为有效 PnP 目标。ROS 时间戳只作身份键，不用它计算本次主机耗时。
4. 旧机械单次试验 `100fps/outputs/test_angle_20hz.py` 直接 `os.write`，不经过
   ROS/PnP/串口桥。本次没有改动或重新运行其运动序列。

这里的 `ban` 和 `100fps` 均位于 `/home/jo/Documents/Codex/2026-10-13/`。

## 已有计时的起止与重叠

| 数据 | 核实后的起止 | 已覆盖/重叠 | 尚不能说明 |
|---|---|---|---|
| 相机约 9 ms | 用户提供的既往相机至 SDK 估计，本轮未重新采集 | 相机时间戳位置、USB、SDK/调度；不等于纯 USB | 非本次同帧同步测量，不能直接加成总延迟 |
| 8.222 ms | ROS 图像时间戳（SDK 返回之后）到结果观察端回调 | 图像发布、识别及 PnP、结果传输、观察端调度 | 不是单独的 PnP 运算耗时，也不是严格 PnP 返回时刻 |
| yaw 91.9–111.9 ms；pitch 71.9–91.9 ms | 第一个目标阶跃命令 `os.write` 返回后的主机时间，到主机观察到 t90 的反馈区间 | 后续主机/USB/接收解析、等待控制周期采用、控制和机械、采样及反馈投递 | 不是纯机械响应，不含此前 PnP/打包，也不完整包含该次 write 调用 |
| 新增 PnP→提交发送 | 源帧最后一次 `cv2.solvePnP` 返回到发送接口调用前 | 后续几何验证、tracker/aimer/转发、打包、发送定时器等待 | 不是 PnP 计算耗时；多候选按帧关联，不声称对应选中装甲板的唯一对象 ID |
| 新增 MCU 接收→采用 | 收齐包到实际控制路径首次消费新目标 | 包含接收→解析与解析→采用 | 已包含在原机械 t90 区间中，不能再次相加 |
| 新增往返确认 | 主机发送调用前到主机读到完整对应确认 | 下行、MCU 采用、确认排队/上报周期、上行、主机调度 | 不是单程，不能除以二；与 MCU 内部段及原 t90 重叠 |

原比较报告 `100fps/outputs/feedback_20_vs_50hz_comparison.md` 明确：50 Hz 是 **反馈频率**，
命令保持 20 Hz，实测约 **19.953 Hz**；实际控制频率未测。每个频率仅一次试验，
不能从这些区间生成机械响应的稳定 P95 或确认控制频率。
串口桥 YAML 的既有默认发送频率为 50 Hz，与该直接串口试验的 20 Hz 不同。
原条件基线须保留实际启动参数，不能把默认配置误认为旧试验条件，或为了计时把频率改成 50 Hz。
起点在 write 返回后，也意味着不能保证 MCU 处理全过程都晚于计时起点；总延迟合计时
既不能重复添加完整的“等待采用”，也不能假定 write 调用内部完全没有设备侧处理。
旧程序在运行中写 CSV/flush，对调度可能有影响，本次没有把旧数据重新标为低开销测量。

## 主机测量

`latency_trace_path` 非空时启用；`latency_protocol` 默认 `legacy`。

- detector 在两个 PnP 调用路径返回后立即记录单调 ns，并记录源帧身份及帧结果。
  标记位于后续几何检查之前，成功返回不等于最后接受该候选。
- `encode_aim_command` 仅对实际 `struct.pack` 记录开始/完成。旧结构仍为
  `A5 + float32_le(yaw) + float32_le(pitch) + 5A 5B`，11 字节，无 CRC、无序号。
- bridge 每次来源命令、打包结果、每次发送前后、写入字节数/报文、来源重复发送、
  发送前覆盖、无效输入、来源过期、短写、异常、断连均有记录。
- 原有 Windows helper 的编号是 **主机辅助进程请求号**，不是 MCU 命令号。
  日志分别记录排队覆盖/取消和 helper 返回；`queue_write` 返回仅说明主机排队，
  helper 成功也只说明 Windows 写调用完成。Linux `serial.write` 返回只说明主机驱动接受写入，
  不代表 MCU 到达、解析或采用，也没有为测量额外调用 `flush`/`tcdrain`。
- `legacy` 不尝试用遥测角度、遥测序号、模式或相近到达时间猜测命令确认。
  这些命令的采用状态保留为未知，而不是成功或丢失。
- 主机各进程使用 `time.monotonic_ns()`，单位 ns；元数据保存实际时钟分辨率、Linux boot ID 和 time namespace。
  分辨率不等于测量准确度。分析器只在非空且一致的 boot ID、time namespace 和单调时钟定义时关联 PnP 与提交发送；
  双方都缺少身份字段也不视为已验证同一时钟。
- 缓冲上限默认每节点 200000 条；满后丢弃新日志并累计数量，不改动命令处理。
  只在节点正常停止时保存 CSV 和 `.metadata.json`，不在控制/接收回调打印或写盘。
  SIGKILL/掉电会丢失内存日志；采样结束使用 Ctrl+C 并等待节点退出。

`PnP→提交发送` 已包含打包/发送定时器等待，不再把打包耗时加到这段上。
旧包可以被原定时器重复发送，所以同时保留 source_id 与 send_id；统计报告不静默去除重复来源。
打包统计按真实打包事件计数，包括打包后被覆盖而未发送的来源；不会把同一个包的重复发送
复制成多次打包样本。发送频率和逐命令耗时则按每次接口调用记录。

## 关联协议的最小改动

38 字节反馈为 `B5 + <II6fB> + CRC16 + 5B5A`。现有 sequence 与 mcu_timestamp_ms
描述遥测，没有已采用命令的序号和时间字段，也没有确认语义。不能复用成命令 ACK 而改掉其含义。

增加一个独立、显式启用的诊断版本，继续保留原 11 字节命令和 38 字节遥测。
不在老报文尾部追加数据，也不借用 yaw/pitch 浮点位或模式位。无自动探测/切换：
只有两端均接入且固件支持已确认后，才设 `latency_protocol=diagnostic_v1`。

公共头 6 字节：`D3 91 | version=1:u8 | kind:u8 | payload_length:u16_le`；
尾部为头和载荷的 CRC16（初值 FFFF，反射多项式 8408，little-endian）。

| 类型 | 载荷（全为 little-endian） | 总长 |
|---|---|---:|
| kind=1 诊断命令 | session:u64，sequence:u32，原样 11 字节命令 | 31 字节 |
| kind=2 诊断确认 | session:u64，sequence:u32，status:u8，rx_complete_us:u32，parsed_us:u32，adopted_us:u32，control_cycle:u32，control_period_us:u32，control_tick_us:u32 | 45 字节 |

status：1=已采用；2=解析后被覆盖；3=校验/业务解析拒绝；4=明确超时或退出模式而未采用。
损坏的 envelope CRC 下 session/sequence 不可信，只计 invalid，不发送虚假的关联 ACK。
主机超时只标记“采用未知”，不能代替 MCU 确认的未采用。
迟到确认即使先于定期超时检查被处理，也保留一次超时记录；随后明确已采用时，
最终采用状态与超时事件同时保留，两者不是互斥状态。

每次实际提交有独立 wire sequence；同一个来源目标的重复发送也是不同命令。
sequence 回绕时切换 session，迟到 ACK 不能命中新命令；进程重启随机生成新 session。
主机不把遥测 sequence 当作 wire sequence。

已采用 ACK 在控制器使用目标的位置生成并缓冲，由**既有反馈任务**统一发送，
不在 ISR/控制环路内发送。沿用 50 Hz 反馈任务时，确认还可能等待约 0–20 ms，
再加队列/USB BUSY 的等待；不是保证 20 ms 内送达。若仍为 20 Hz 则约 0–50 ms。
旧 38 字节遥测按原周期继续发送；需要 MCU 的既有 TX 仲裁器支持多帧排队，不能抢占正在发送的缓冲。
若现有 USB 通道预算无法容纳诊断 ACK，就记录拥塞/丢弃并延后实测，不能为测量修改反馈或控制频率。

诊断命令由 11 字节变为独立的 31 字节帧，新增确认 45 字节，会增加链路负载。
应分别保存 `legacy` 主机基线与 `diagnostic_v1` 关联基线，后续前后对照保持相同诊断版本、
缓冲容量和反馈调度策略；不能把不同报文长度的差异当成控制算法优化收益。
本次没有启用诊断发送，也未验证该协议在目标 MCU 上的链路占用及测量扰动。

## 下位机接入状态与必须配合的最小修改

当前环境无 `/dev/ttyACM*`、`/dev/ttyUSB*`、`/dev/bus/usb`。
旧审计引用的 `/mnt/e/000workplace-E/rm/rm视觉/串口协议` 也不存在。
可见的 `2026-10-05/wx/.../20.standard_robot_firmware_only` 是另一套遥测/调参副本，
不能确认它是当前 11/38 字节实机固件，因此没有修改该副本，更没有声称已在真实采用点埋点。

提供 `tools/control_latency/mcu_latency_hooks.{c,h}`，已做主机 C 编译及跨语言协议校验，
**尚未集成/交叉编译/刷入目标 MCU**。接入实际固件需要：

1. 提供现有单调硬件计时器，转换为 uint32 µs。应实测其分辨率并记录频率来源，
   目标为 1 MHz/1 µs，不改变已有控制计时器。若只能用 1 ms RTOS tick，须标注 1 ms，
   不能把换算的 µs 当作 µs 精度。`uint32_us` 约 71.6 分钟回绕，内部差用无符号相减；
   单个测量跨度必须小于 2^31 µs（约 35.8 分钟）。分析器检查
   接收→解析 + 解析→采用 = 接收→采用；异常记录保留但不计入完整有效样本。
   重启或更换计时器需结束当前采集，无符号相减不能独自证明未发生重启。
2. 在真正接收完整 **一条命令最后一个字节** 的流组包位置保存 `rx_complete_us`，
   与整包一起送现有解析任务。若 USB 回调只把字节放入队列，不能在稍后的取队列位置
   假装“收齐时间”；若一个 USB 块含多包，分别保存完整包的组包标记。
   回调只能保存时刻/固定记录，不打印、不发 ACK。
3. 保留旧包解析分支。对独立诊断包先用 `latency_decode_command` 校验外壳，
   将得到的 11 字节交**原解析器和原边界/有限值检查**。校验和解析全部结束后
   `latency_parsed(&tag, now_us())`；该时刻不是采用时刻。旧命令不能伪造关联 tag。
4. 将 tag 与目标 yaw/pitch 一起原子地保存/读取，不能分别读取而混配。覆盖尚未使用的
   带 tag 目标时，先 `latency_record(old_tag, LATENCY_SUPERSEDED, ...)`，记录未采用。
   被拒绝或过期/模式退出也分别记录相应状态；不得为了 ACK 强迫控制器采用目标。
5. 在现有控制周期读取新目标且**确实用于该次控制计算**的位置标记 `adopted_us`，
   调用 `latency_record(tag, LATENCY_APPLIED, adopted_us, cycle, period_us, cycle_tick_us)`。
   每个新 tag 只记一次；若只有赋值没有执行控制，不能记 APPLIED。记录该周期计数、
   周期起点及相邻起点的实际间隔，不变更原周期/延时/PID。
6. 既有反馈任务通过 `latency_peek_ack` 取固定缓冲。CDC BUSY 时保留记录并重试，
   接受发送后才 `latency_pop_ack`；TX 缓冲持续有效至传输完成。不要占用旧遥测缓冲。
   64 条环形记录满后丢弃新记录，`latency_dropped_records()` 累计；
   `latency_invalid_commands()` 累计损坏帧。通过现有调试器/诊断导出采集前后值，
   保存为独立 JSON；缺失时统计报告标为未知，不能宣称 MCU 零丢弃。
7. `latency_init` 的临界区回调必须保存/恢复中断屏蔽状态，短且不阻塞。
   接收、控制、反馈任务共享的目标/tag 也要用原有同步机制，库内临界区只保护日志环。

这七处是对另一端的接入要求，不是本次已经完成的固件改动。

## 可执行采集步骤

先在工作区构建更新后的主机包：

```bash
cd /home/jo/Documents/Codex/rm_vision/rm_auto_aim
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --symlink-install --packages-select rm_serial_driver yolo_armor_detector auto_aim_bringup
source install/setup.bash
```

在**原来已经确认的启动命令**上只增加测量参数。detector 的 launch 现在接受：

```text
latency_trace_path:=/home/jo/Documents/Codex/2026-10-15/pid-sdk-pnp-11-usb-sdk/outputs/captures/run01/pnp.csv
```

串口桥原启动参数保留，仅追加：

```text
-p latency_trace_path:=/home/jo/Documents/Codex/2026-10-15/pid-sdk-pnp-11-usb-sdk/outputs/captures/run01/host.csv
-p latency_protocol:=legacy
```

使用 launch 启动串口桥时对应为 `latency_trace_path:=... latency_protocol:=legacy`。
每次使用新路径，禁止覆盖原始记录。既有脚本如果没有透传该 launch 参数，应在原 detector
启动行增加这一项；不要额外启动第二个 detector/串口读取者。
legacy 可先测主机侧，关联 MCU 的样本数预期为 0。

在真实固件接好上节挂点后，**只在两端明确支持时**把这一项换成 `diagnostic_v1`。
维持原来已经运行的命令来源、发送频率、控制频率和反馈周期；不为凑数新建阶跃或运动轨迹。
收集至少 100 条 PnP 可关联且 MCU 确认已采用的命令，同时保留所有失败/未采用/未知状态。
名义 20 Hz 至少需 5 秒，但应延长到 30–60 秒覆盖原有负载，并留约 1 秒确认等待窗口后
再按原有方式停止节点。停止顺序和无效目标处理沿用原流程。

导出 MCU 计数 JSON（示例字段名，数值必须来自实机，不提供虚假零值）：
`{"dropped_before": ..., "dropped_after": ..., "invalid_before": ..., "invalid_after": ...}`。
另保存固件版本、1 µs 计时器来源/实际分辨率、反馈周期、原 PID/限幅及启动参数快照。

离线分析：

```bash
python3 tools/analyze_control_latency.py \
  --host /home/jo/Documents/Codex/2026-10-15/pid-sdk-pnp-11-usb-sdk/outputs/captures/run01/host.csv \
  --pnp /home/jo/Documents/Codex/2026-10-15/pid-sdk-pnp-11-usb-sdk/outputs/captures/run01/pnp.csv \
  --mcu-counters /home/jo/Documents/Codex/2026-10-15/pid-sdk-pnp-11-usb-sdk/outputs/captures/run01/mcu_counters.json \
  --output /home/jo/Documents/Codex/2026-10-15/pid-sdk-pnp-11-usb-sdk/outputs/captures/run01/analysis
```

只测主机可省略 `--mcu-counters`；没有 detector 原始标记也可省略 `--pnp`，该段样本数为 0。
输出 `commands.csv`、`summary.json`、`report.md`，包括每段 n/中位数/P95/最大值、实际提交频率、
反馈到达频率、MCU 时间戳推算的反馈频率、采用时采样的控制周期间隔/频率、所有事件计数。
事件总数优先取缓冲满后仍继续累计的元数据计数，并同时输出实际保存行数。
有日志丢失或非法时间记录时，报告增加质量问题标记；即使达到 100 条，也不隐藏该限制。
P95 使用排序后 (n−1)×0.95 位置的线性插值。各段样本数可以不同：打包包含
随后被覆盖的目标，往返确认包含已采用和明确未采用的确认；未收到确认的命令仍保留。
控制周期计数和周期起点还用于计算两次采用之间所有周期的平均执行频率，内部只相减 MCU 时钟。
Windows 下主机提交频率不是物理串口写入频率；控制频率统计仅覆盖采用时采到的周期，
不能假称完整控制环的最坏周期。100 条要求未满足时报告标为 incomplete_baseline。

## 当前基线结果

| 新增测量 | 有效实机 n | 中位数 | P95 | 最大值 |
|---|---:|---|---|---|
| PnP 返回→提交发送 | 0 | 待实测 | 待实测 | 待实测 |
| 打包 | 0 | 待实测 | 待实测 | 待实测 |
| 发送接口调用 | 0 | 待实测 | 待实测 | 待实测 |
| 往返确认 | 0 | 待实测 | 待实测 | 待实测 |
| MCU 收齐→解析完成 | 0 | 待实测 | 待实测 | 待实测 |
| MCU 解析完成→实际采用 | 0 | 待实测 | 待实测 | 待实测 |
| MCU 收齐→实际采用 | 0 | 待实测 | 待实测 | 待实测 |

本轮实机超时、丢弃、覆盖、未采用数量与实际频率均待实测，不写作 0。
没有生成冒充实测的逐命令记录。旧单次试验的 100 条发送也没有 MCU 确认，不能补足这 100 条。

验证覆盖：Python/C 编解码一致、CRC/分片、32 位回绕、日志有界、超时/迟到确认、
旧 11 字节保持不变，以及真实 bridge 回调通过纯内存端点处理 120 条合成命令并导出关联 CSV。
这些是软件测试，不是相机/机械/MCU 的延迟基线。
本轮 3 个主机包构建通过；计时、协议、真实回调的内存端点、PnP/图像转换及 ROS 节点回归
共 **158 项通过**。其中 4 项 ROS 节点测试最初受沙箱网络限制，使用模拟串口在本地 DDS
可用环境重跑通过；没有打开硬件串口。真实相机→控制器全链路及硬件采用尚未实测。

## 计时公式及本轮核查修补

设主机 H0=PnP 返回、H1/H2=打包开始/完成、H3/H4=调用发送前/返回后、H5=完整确认读入主机；
全部为同一主机单调 ns：PnP→提交=(H3−H0)/1e6 ms，打包=(H2−H1)/1e6 ms，
发送调用=(H4−H3)/1e6 ms，往返确认=(H5−H3)/1e6 ms。
其中 H5 是读取到包含最后一个 ACK 字节的块后立即记录的时间，可能包含串口轮询等待；
不是 USB 硬件到达时刻。Windows helper 读取也包含跨进程投递等待。

下位机 R/P/A=收齐/解析完成/实际采用，使用本设备单调 uint32 µs：
接收→解析=((P−R) mod 2^32)/1000 ms；解析→采用=((A−P) mod 2^32)/1000 ms；
接收→采用=((A−R) mod 2^32)/1000 ms。每段和总跨度须小于半周期，顺序一致才有效。
主机 H 与下位机 R/P/A 从不直接相减。

本轮之前已存在主机挂点、独立诊断协议、MCU 可移植组件和分析器。本轮复用它们，实际修改为：

- 主机晚到确认即使在超时扫描之前处理，也明确记一次超时，并保留确认状态。
- 不再把两个缺失的时钟身份字段视为同一时钟。
- MCU 三个时刻须满足顺序及分段求和关系，异常记录不计入完整有效样本。
- 完整有效样本同时要求打包、发送调用、往返确认及三个 MCU 区间均有效。
- 事件总数使用元数据累计，另列保存行数；日志丢失和异常时间会在基线状态中显式标记。
- 新增相应回归检查，更新采集说明与测量边界。

以上均在日志或离线分析路径；没有更改原控制周期、控制采用条件、运动序列或 PID。
主机时间戳及缓冲追加仍有测量开销；没有实测该开销对实机调度的影响，不声明为零。
