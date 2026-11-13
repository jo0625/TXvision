# 2026-11-13 软件验证

- `colcon build --symlink-install --packages-select armor_tracker rm_aimer --cmake-args -DBUILD_TESTING=ON`：两个包构建成功。复用主工作区的 auto_aim_interfaces，消息只添加注释，线协议没有改变。
- `colcon test --packages-select armor_tracker rm_aimer`：两个包各 8 个 CTest 项全部通过。
- `colcon test-result --verbose`：`172 tests, 0 errors, 0 failures, 13 skipped`。
- 新增 5 个单板跟踪回归用例：姿态跳变不改变静止位置、三轴速度和短时丢失外推、同 ID 近邻关联及顺序独立、远处同 ID 不触发旋转换板、换目标/重置恢复六维滤波状态。
- ROS 节点测试分别验证 single_armor/rotating 的板数、位置、半径、速度和消息时间戳；单板用交替翻转的姿态输入。
- Python 验证单板零半径几何、预测时间及跨坐标系瞄准指令，原整车与弹道测试通过。
- 两种模式的 `start_full_aim.sh --check` 均通过；两份 tracker YAML 除 motion_model 外一致；实际使用的瞄准几何快照与包源码一致。
- 在原 with_fix.sh 环境中验证动态加载：libarmor_tracker.so 来自本分支 build 目录，libtf2.so 来自原 TF 修复 overlay；验证入口实际导入的单板瞄准几何可用。
- 启动脚本通过 bash 语法检查，Python 启动文件通过 AST 语法检查。

未运行相机、串口或云台实机跟随。软件测试不能证明顿挫已消除；请按 README 使用同一目标和动作交替运行两种模型，并结合 pipeline/tracker/aimer 日志判断。
