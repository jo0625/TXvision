# Windows 双向桥主机修复（2026-09-23）

## 问题和修复范围

用户要求“告诉我问题是什么再帮我修理”。历史三轮计划180秒的实机双向测试，
实际在40.703、30.933、25.496秒停止反馈，均未通过持续通信验收。
只读基线、正常内存测试都不能代替这些失败试次，也不能排除双向桥的问题。

此次先用故障注入复现了主机实现的两处缺陷，再修改主机桥：

1. Windows helper 在同一个循环中写串口、收反馈和发心跳。一次500 ms慢写会同时
   拖住接收和心跳，测得反馈最大间隔522.889 ms、生成到Windows读取延迟490.630 ms。
2. helper 的单槽等待队列替换旧请求时没有返回终止状态；Python又只在迟到ACK到达后
   清理超时序号。真实helper下500 ms慢写配合40 ms请求期限，恢复后仍残留2个序号；
   合成对端丢弃20个ACK时，空闲0.5秒后残留20个。

上述缺陷已经确定，但尚未证明它们是历史实机永久断流的全部原因。
此前设备I/O异常、重开COM3后仍无反馈的现象仍需物理链路验证。

本次只修改主机桥、内存测试及文档；没有访问电控非开源IMU实现，没有更改IMU/INS
或其数据路径，没有修改固件、烧录、切换控制模式或向真实串口发送目标。

## 实现

- `windows_duplex_helper.cs`：独立写线程；接收和20 ms心跳继续由主循环处理。
  ACK、心跳、反馈使用整帧输出锁，避免并发破坏管道帧。仍只有一个串口写者。
- helper 单槽等待队列被替换的请求返回状态3，表示未执行；执行前再次检查原始期限。
  停止时丢弃等待请求，等待已经开始的写调用结束，不补发旧请求。
- `windows_duplex.py`：将状态3视为请求失败而非连接损坏；超时序号跟踪最多保留
  256条、持续处理管道时最长2秒，迟到旧ACK不能完成新请求。序号耗尽要求重连，禁止复用。
- 真正的设备读/写异常仍终止连接，并注明失败阶段；不将未知结果报成成功，不自动重试写入。
  `Write`返回仅表示主机写操作完成，仍不代表设备已接收或执行。
- `CMakeLists.txt`：纳入独立反馈源的故障恢复回归；旧内存端点补充并发保护。
  新测试端点用每50 ms独立产生的计数记录验证接收，与机器人姿态协议无关。

原始11字节设备命令协议、ROS控制门限、只读路径和IMU相关代码均未修改。
Python与C# helper应一起更新，旧Python客户端不能单独配用新增的状态3。

## 修复前后对照

| 条件/指标 | 修复前 | 修复后 |
| --- | ---: | ---: |
| 150 ms慢写：反馈最大间隔 | 167.370 ms | 65.854 ms |
| 150 ms慢写：主机心跳最大间隔 | 167.372 ms | 35.420 ms |
| 500 ms慢写：反馈最大间隔 | 522.889 ms | 64.114 ms |
| 500 ms慢写：主机心跳最大间隔 | 522.894 ms | 33.441 ms |
| 500 ms慢写：生成到Windows读取最大延迟 | 490.630 ms | 16.432 ms |
| 500 ms慢写＋40 ms期限：结束时残留超时/已发序号 | 2 / 2 | 0 / 0 |
| 永久丢20条ACK：清理结果 | 0.5秒后仍20 / 20，原实现无回收上限 | 2.2秒后0 / 0 |

慢写仍可能超过请求期限，修复不会让超时写变成有效命令。500 ms注入用例中，
正常期限有5次ACK等待超时；40 ms期限有13次ACK等待超时和1次过期拒绝。
期间接收、心跳继续，结束时所有请求记录清空。两轮均无反馈序号跳变，helper正常退出。
超时次数增加不等于接收变差：旧实现心跳也会停，客户端随后因时钟过旧而停止提交；
新实现保持时钟消息，等待中的请求仍按原期限失败或被替换，没有延长期限或追发。

上述数字来自真实Python客户端、真实Windows C# helper与内存端点。
只有“丢20条ACK”一行使用合成管道对端。`COM999`仅用于名称校验，测试入口不构造
真实串口。反馈在Windows时钟下产生；主机间隔使用WSL单调时钟，不跨两种时钟相减。

## 验证状态

修复前新增的三项回归均失败：状态3被视为非法ACK；丢失ACK的序号无法回收；
500 ms慢写期间反馈生成到读取延迟466.057 ms，超过150 ms判据。修复后这三项通过。

额外覆盖：容量淘汰后迟到ACK不误配、非法/未来ACK及当前请求的重复ACK仍拒绝、序号不循环复用、
真实helper为被替换请求返回终止状态、停止时不执行等待请求，以及读错误/写错误/
设备写超时后创建新helper，先接收到反馈再接受新请求。

`colcon build --packages-select rm_serial_driver --symlink-install`通过。
注册后的五组Windows相关测试全部通过：3项基础管道、23项客户端、12项真实Windows
helper/ROS集成、6项独立反馈源故障回归、4项节点选择/调度，共48项，0失败、0跳过。
测试结果在`build/rm_serial_driver/test_results/rm_serial_driver/test_windows_*.xunit.xml`。

修复后正常内存双向负载完整运行600.201秒，实际反馈首尾覆盖600.199秒，
序号从1连续到12445。不是只让进程等待到600秒；记录显示接收持续覆盖整个窗口。

| 正常负载指标 | 修复前 | 修复后 |
| --- | ---: | ---: |
| 实际运行时间，含0.2秒末尾排空 | 600.201 s | 600.201 s |
| 提交/成功确认 | 29268 / 29268 | 29305 / 29305 |
| 周期反馈记录数 | 12451 | 12445 |
| 反馈序号跳变 | 0 | 0 |
| 主机接收块最大间隔 | 78.821 ms | 79.189 ms |
| 主机心跳最大间隔 | 43.645 ms | 44.575 ms |
| Windows生成到读取最大延迟 | 24.875 ms | 25.716 ms |
| 主机写确认最大延迟 | 25.186 ms | 1.927 ms |
| 写超时、拒绝、连接错误 | 均为0 | 均为0 |
| 结束时待发、活动、超时和未清理请求 | 均为0 | 均为0 |
| helper退出码 | 0 | 0 |

正常接收间隔与修复前相近，关键改进发生在慢写和确认异常条件下。
本轮窗口内请求跟踪没有积累；不将内存端点结果描述为真实串口、USB或完整实机验收。

## 数据和复现

- 修复前：`frame_rate_diagnostics/windows_duplex_stability_20260923_*.json`
- 修复后：`frame_rate_diagnostics/windows_duplex_fix_20260923_*.json`
- 修复前报告：[稳定性复核](windows_duplex_stability_review_2026-09-23.md)

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --packages-select rm_serial_driver --symlink-install
colcon test --packages-select rm_serial_driver --return-code-on-test-failure \
  --ctest-args -R test_windows_
/usr/bin/python3 rm_serial_driver/tools/probe_windows_duplex_stability.py \
  --duration 600 --output frame_rate_diagnostics/duplex_after_fix_new.json
/usr/bin/python3 rm_serial_driver/tools/probe_windows_duplex_stability.py \
  --duration 4 --fault slow_write --delay-ms 500 \
  --output frame_rate_diagnostics/duplex_slow_after_fix_new.json
/usr/bin/python3 rm_serial_driver/tools/probe_windows_duplex_ack_loss.py \
  --mode lost --settle-seconds 2.2 \
  --output frame_rate_diagnostics/duplex_ack_after_fix_new.json
```

输出路径必须是新文件，以保留原始记录。

## 实机状态与剩余工作

检查时接收服务仍为`transport=windows, receive_only=true`。此路径不使用此次修改的
双向helper；本轮没有重启服务或启用双向发令。只读服务active仅代表进程运行，
不能单独证明反馈持续或双向桥已经验收。
结束时该服务为active，PID 30055。主机修复已构建，未来启动双向路径会使用新版本。

下一阶段是在既有电机硬件失能条件满足后，对修复版进行真实USB双向通信验证，
确认实际连续接收覆盖完整180秒，记录第一处失败发生在读、写、helper还是设备反馈端。
若仍断流，继续检查非IMU的USB通信与完成状态；不能把离线通过写成实机已修复。
