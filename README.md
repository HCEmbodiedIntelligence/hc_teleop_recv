# hc_teleop_recv

从 `HC-teleop-robotic` 提取的 ROS 2 Humble 遥操作前端：接收 PICO、处理离合、
映射相对位姿、限制位移及滤波，然后为 `humanoid_motion_server` 发布末端目标。
不依赖原仓库运行环境，不加载 URDF/PyBullet/Pinocchio，不执行机械臂 IK/FK。可选的底盘和夹爪通用输出默认关闭，在管理器网页配置后接入对应驱动。

```mermaid
flowchart LR
  P[PICO UDP / 已有 vrdata] --> R[hc_teleop_recv\n离合、相对映射、滤波]
  R -- PoseStamped / ServoP --> M[humanoid_motion_server\nIK、FK、仲裁、限位]
  M -- 实测 FK PoseStamped --> R
  M -- joint_cmd --> D[humanoid_driver_runtime]
  D -- joint_states --> M
```

## 构建与独立启动

```bash
cd /home/czy/teleop_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select hc_teleop_recv humanoid_manager robot_bringup
source install/setup.bash

ros2 launch hc_teleop_recv hc_teleop_recv.launch.py \
  config_file:=/absolute/path/to/model/hc_teleop.yaml
```

OpenArmX 示例位于模型资源：
`openarmx_driver/deployment/openarmx_v10_bimanual/model/hc_teleop.yaml`。
通用单臂示例是 `config/single_arm.example.yaml`。机器人型号、关节、基座和工具名称均不写在前端代码中。

## 通过 humanoid_manager 切换机器人

在 **robot_model 插件** 的 `manifest.yaml` 中增加：

```yaml
resources:
  # 保留原来的 motion_params/sdk_config/channel_config/tool_config/urdf
  hc_teleop_config: resources/hc_teleop.yaml
```

配置随模型 ZIP 打包、校验和部署，组合清单仍只引用 driver/model ID。
管理器使用与接收器相同的配置校验器，逐通道检查目标是否为 ServoP，
以及 `base_frame`、`tool_frame`、`fk_pose_topic` 是否与 motion 的通道一致。
一个模型只允许选择 `hc_teleop_config` 或旧的 `teleop_config` 之一，防止两个接收器争用输入和目标。

更新 OpenArmX 模型 ZIP（原 driver/composition 已部署时只需更新 model）：

```bash
python3 src/openarmx_driver/tools/create_model_bundle.py /tmp/openarmx-hc-model.zip
ros2 run humanoid_manager humanoid_pluginctl.py validate /tmp/openarmx-hc-model.zip
ros2 run humanoid_manager humanoid_pluginctl.py deploy /tmp/openarmx-hc-model.zip
ros2 run humanoid_manager humanoid_pluginctl.py resolve openarmx_v10_bimanual

ROS_DOMAIN_ID=14 ros2 launch robot_bringup registered_robot.launch.py \
  robot_id:=openarmx_v10_bimanual start_teleop:=true
```

厂商硬件控制器需要按原部署流程启动。`start_driver/start_motion/start_teleop` 分别选择核心进程。
切换机器人时停止旧进程，部署或选择另一 robot_id 后重新启动；不在运动中热切换配置。
使用自定义部署目录时，CLI 在子命令前传 `--root PATH`，launch 传 `plugin_root:=PATH`。

## 输入和操作

- `input.mode: udp`：本包独占 UDP 5005，5006 回复 PICO 自动发现。端口可配置。
  可发布原格式 `/vrdata`，其他录制或显示节点可订阅。
- `input.mode: vrdata`：订阅已有中间件的 `std_msgs/String` JSON；不打开 UDP，也不回发该话题。
  JSON 必须带 `received_stamp_ns`：发送端首次收到输入时的 ROS 时钟纳秒数，
  不能在转发或重放时刷新。UDP 模式发布 `/vrdata` 时自动添加此字段；外部生产者需要同步更新。

每个新输入包最多生成一次手臂目标，目标时间戳保留输入到达时刻，不对旧姿态重复打新时间戳。
UDP 使用 Linux 内核接收时间计算排队年龄；每轮最多处理 64 包，未排空时继续下一轮，
期间不发布中间姿态，仍处理有效按钮边沿和停止检查。排空后只发布最新有效姿态，过期输入丢弃。
  不需要原项目的 IK/控制节点。不要同时运行原项目完整 `run.sh teleop` 和新的运动链。
- PICO v2 保留原二进制格式 `<4sBIdB21f3H6f3H6f>`，四元数 XYZW，SI 单位。
  v1 可以解码和显示，但没有 Grip，不产生 ServoP 目标。
  VR 世界原点可以平移，绝对坐标不使用固定的 ±5 m 门槛；仍拒绝非有限数值。
  机械臂位移限制作用于离合绑定后的相对位移，避免世界原点偏移导致整包按键也被丢弃。
- 每个通道可绑定 left/right/head 位姿及 left/right Grip。OpenArmX 示例用右 Grip 同时控制双臂。
- 默认启动禁用。按右手 A，或调用 `/hc_teleop_recv/set_enabled`（SetBool=true）启用；
  按住 Grip 即可绑定当前手柄姿态和 motion server 的新鲜实测 FK，无需先松开再按下。
- 松开 Grip 停止刷新 ServoP；输入断流、跟踪丢失、FK 超时或输入源切换时停止输出；恢复有效输入和实测 FK 后，握键按住即可自动重新绑定当前姿态。
  ServoP 下游按 motion server 的 lease 到期保持，当前默认 100 ms；停止发布不等于物理急停。
- `/teleop/emergency_stop` 的 true 禁用前端；false 不自动恢复。按 A 或调用服务显式恢复。
- `/hc_teleop_recv/status` 发布状态、原因和收包计数。

VR 状态回传到已接受的 PICO 输入来源 IP、`input.event_port`（默认 UDP 5007）。
首次连接、接收端重启后收到输入、控制状态变化时同步，并以 5 Hz 发送 `teleop_status`
快照；兼容原协议的 `safety_stop` / `safety_resume` 每秒补发一次。
状态由实际启用开关、回位锁定、输入/跟踪/FK 新鲜度和 Grip 状态计算，连接成功不代表
控制已启用。状态补发只通知 VR，不调用使能服务，也不发布 ROS 急停/恢复命令。
不被当前输入会话接受的发送端不能改写状态回传地址。

`teleop_status.payload` 包含 `state`、`reason`、`enabled`、`motion_active`、`input_fresh`
以及每臂 `channels`，同一内容也放在 ROS 状态的 `control_status` 中。
状态包括 `disabled`（请按 A）、`ready`（等待握持键）、`active`（发送目标）、
`motion_active`（回位锁定）、`input_timeout`、`tracking_lost`、`feedback_timeout` 和
`partial`（部分臂不可控）。`active` 表示前端正在发送目标，不保证 IK 可达或机器人已运动。

所有 UDP 通知带接收端实例 `session_id` 和递增 `sequence`，重复发送同一通知时这两项相同。
客户端接入详细状态时应按实例和序号去重/丢弃旧包；`state_sync: true` 表示状态同步，
不应重复弹窗、震动或播放提示音。超过 `valid_for_ms: 1000` 未收到快照应显示“状态未知/连接中断”，
不能继续显示旧的“可控制”。这些客户端显示行为需要 APK 支持；本包不包含 Unity 工程。

按键、摇杆及 Trigger 仍在 `/vrdata` 中供其他上层模块使用。
机械臂执行输出为 Cartesian 目标；可选底盘发布 Twist/TwistStamped，夹爪支持 JointState/Float64 或 GripperCommand Action。关节回零、腰部解算和 Dashboard 不属于本包。不用全零关节值代替实测 FK。

## 状态变化日志

接收节点在默认 INFO 日志中记录以下变化，稳定状态不周期重复打印：

- `input.transport`：位姿端口的 UDP 数据开始/停止到达（`vrdata` 模式则是话题消息）。
  包含发送端、累计包数和断流阈值；发现端口不算位姿输入，非法包也只表示传输到达。
- `input.accepted`：通过协议、来源和时序校验的手柄数据开始/停止到达，包含接收/拒绝计数。
- `controller.<手柄>.pose`：位姿开始变化、静止、跟踪丢失或输入断流。
- `arm.<通道>.fk`：有效实测末端反馈到达/中断；拒绝反馈时附带坐标系、时间戳或姿态错误。
- `arm.<通道>.output`：开始发布 ServoP 或停止及原因；附带位姿手柄、离合手柄、Grip 值、
  阈值和反馈新鲜度，能够区分未握紧、跟踪丢失、FK 超时、暂停等情况。
- `control`、`gripper.<夹爪>`：总使能、回位占用和夹爪控制状态变化。

每项最多每秒一条；短时间内多次切换合并成一条变化路径（最多保留 8 次状态和累计次数），
包括 `stopped -> receiving -> stopped`，不丢掉短暂的启停信息。首次观察记录一次基线状态。
断流沿用配置的输入/FK 超时；位姿累计变化达到 2 mm 或 0.01 rad 才视为运动，连续 0.5 秒
无此变化视为静止，减少跟踪噪声刷屏。这些阈值仅用于日志，不改变控制或安全条件。

例：`arm.right_arm.output: publishing -> ready: Grip released; ... grip=0.000; threshold=0.55`。
真实关节是否运动看驱动节点的 `joint_feedback.<组>` 日志，不能仅凭 `publishing` 判断。
日志写入现有 ROS 节点日志和启动器运行日志；安装更新后，下次重启对应节点生效。

## 坐标语义与数学

一个配置条目对应 motion `channels.yaml` 中的一个 `servo_p` 通道：

```yaml
# motion channels.yaml
channels:
  - name: teleop_arm
    kind: servo_p
    endpoint: /teleop/arm/servo_p
    priority: 50
    group: arm
    base_frame: base_link
    tip_frame: tool0
    fk_pose_topic: /teleop/arm/fk_pose
```

接收器的 `target_pose_topic/fk_pose_topic/base_frame/tool_frame` 必须分别与
`endpoint/fk_pose_topic/base_frame/tip_frame` 一致。输入 FK 和输出目标的 header.frame_id 都是
base_frame；工具语义由通道配置决定。FK 错误坐标、空/冻结/过期时间戳、非法姿态会被拒绝。

`axis_mapping=C` 表示 **VR 跟踪坐标增量到该通道 base_frame 的旋转**，必须正交且 det=+1。
每个通道可配置 `position_axis_signs: [1, 1, 1]`（默认值），按基座 X/Y/Z 轴分别选择 `1` 正向或 `-1` 反向；它只作用于平移，不改变旋转跟随。网页“机械臂遥操作”中可直接选择各轴方向，保存并重启机器人后生效。
按下离合时锁存手柄 `(p_h0,R_h0)` 和实测末端 `(p_e0,R_e0)`：

```text
p_target = p_e0 + position_scale * diag(position_axis_signs) * C * (p_h - p_h0)
R_target = C * (R_h * R_h0.T) * C.T * R_e0
```

算法及径向死区/姿态滤波提取自原 `arm_teleop_math.py`。位移限幅先作用于相对位移，
可选 workspace 在 base_frame 中做 AABB 限幅，随后低通滤波。
实测绑定点在 workspace 外时不输出，避免按 Grip 的首个目标被强行截到边界。

OpenArmX 实机 FK 以 `openarmx_body_link0` 表达，因此对应的 motion ServoP 通道和接收器
也统一使用该 frame。OpenArmX 的三轴映射使用
`C_body=[[0,0,-1],[1,0,0],[0,-1,0]]`，配套 `position_axis_signs=[-1,-1,-1]`。
[Unity 的右/上/前坐标](https://docs.unity3d.com/cn/2022.3/Manual/QuaternionAndEulerRotationsInUnity.html)
到机器人前/左/上的位置换基是
`B=[[0,0,1],[-1,0,0],[0,1,0]]`（det=-1）。配置使用 `C=-B`（det=+1），
使旋转换基 `C*R*C.T` 等于 `B*R*B.T`；平移用三个反向符号还原成 `B`。
因此俯仰抬起朝前、保持扳机朝前时向右外翻，以及左右转向都遵循同一坐标约定。
原矩阵 `[[0,0,-1],[-1,0,0],[0,1,0]]` 配 `[-1,1,1]` 的平移保持不变；
上一版仅修俯仰的 `[[0,0,1],[1,0,0],[0,1,0]]` 会反转前后轴外翻，已由此三轴映射替代。
Grip 首帧仍绑定实测姿态。
这些是机器人配置中的标定值，不更改通用示例或其他机型的默认矩阵。
肩部固定安装变换由机器人模型中的 FK/IK
处理，前端不再把 body-frame 增量旋转到左右肩基。
`left_tool0/right_tool0` 的 0.1801 m TCP 偏移由模型 tools.yaml 和 motion server 处理。
若参考基座会运动，目标按该 base_frame 跟随；更换安装角或选择不同参考基座时更新映射配置。

## 验证

```bash
colcon test --packages-select hc_teleop_recv humanoid_manager robot_bringup
colcon test-result --verbose
```

测试覆盖原 HC 相对姿态公式、双臂不同基座、单臂配置、离合重绑、输入/FK 超时、
网络坏包和序号回绕、部署配置一致性，以及隔离 ROS Domain 中的 UDP → FK → ServoP 接口。

本次还用 OpenArmX 模型启动了实际 `humanoid_motion_server`，以模拟关节反馈完成
FK → 接收器参考绑定 → 双臂 ServoP → IK 关节输出的联调，没有连接硬件。
结果在工作区 `log/hc_teleop_motion_integration.json`，服务端日志在同名 `.log` 文件。
本机默认 Fast DDS 跨进程测试出现 endpoint 可发现但收不到数据的情况；联调使用
仅回环 UDPv4 传输和独立 Domain 224 完成。这是测试环境的通信条件，实机通信和运动尚未验收。

## humanoid_manager 接入与按钮录制

使用 `humanoid_manager` 网页管理器配置本接收端。模型资源 `hc_teleop_config` 可以包含：

```yaml
adapter:
  robot_id: lab_arm
  buttons_topic: /hc_teleop_recv/buttons
```

`robot_id` 可留空；管理器创建副本时会填入新机器人 ID。按钮话题必须与控制输入/输出、FK 和状态话题分离。

- `/hc_teleop_recv/buttons`：`std_msgs/msg/String` JSON，每个通过校验的输入包发布一条；包含 `stamp_ns`、`sequence`、`vr_timestamp`、左右手柄 `inputs` 与 `edges`。
- `edges` 根据连续按住掩码计算，仅发布新的 `pressed`/`released`。初次输入及发送端重连时建立基线，避免把已按住的按钮误作新标记。原始 `pressed_mask`/`released_mask` 同时保留在 `inputs` 中。
- `/hc_teleop_recv/status`：包含接收/拒绝包数、通道状态、配置身份和最新按钮状态。
- `/hc_teleop_recv/get_configuration`：`std_srvs/srv/Trigger`，返回加载时配置及 SHA-256，可核对实际生效的配置；查询不会修改运行参数。

```bash
ros2 service call /hc_teleop_recv/get_configuration std_srvs/srv/Trigger '{}'
```

数据录制、回放、片段无效标记与异常检查都由 `humanoid_manager` 承担。`hc_teleop_recv` 负责输入接收、校验与运动目标转换，无需运行旧的遥操作程序。

## 可选底盘与夹爪

只测试夹爪或底盘时，在网页“机械臂遥操作”取消“启用机械臂遥操作（全部通道）”，
保存后重启机器人，同时保留“同时启动遥操作”。对应配置为 `control.arm_control_enabled: false`。
关闭时保留全部机械臂通道与标定，但不创建机械臂目标发布器，也不绑定 Grip；A 键或接收端
使能服务不会重新开启机械臂输出。夹爪、底盘继续使用各自的启用配置和使能按钮。
该字段默认 `true`，兼容已有配置；需要恢复机械臂时重新勾选、保存并重启。
这个开关仅关闭本接收端的机械臂遥操作输出，厂商电机使能和其他运动控制接口仍由各自服务管理。

在 `humanoid_manager` 网页“机器人配置 → 底盘配置 / 夹爪配置”添加。配置与既有机械臂通道独立，支持 `channels: []` 的纯底盘/夹爪配置；未配置或 enabled=false 时不创建输出发布器。

- 底盘：选择手柄、摇杆轴、方向、死区、按住使能按钮、速度/加速度上限、输出频率和输入超时。接口 Twist 或 TwistStamped，默认 `/cmd_vel`；停止/超时/跟踪丢失/禁用后发送零速度。恢复接口后需要松开再按下使能。
- 夹爪：可添加多个 ID；绑定 trigger/grip 模拟量、使能按钮、开合位置、单位、最大位置速度、努力值；支持 JointState、Float64、`control_msgs/action/GripperCommand`。Action 选项需要系统安装 control_msgs；缺少时显示接口错误，不启动该输出。
- 夹爪在总使能开启、输入跟踪和反馈有效、控制接口就绪时，按住使能按钮即可控制；启用或恢复后无需先松开再按下。松开使能按钮仍停止控制。
- 反馈：JointState 需要正确关节名、新鲜 header 时间；Float64 使用主机收到消息的时间。只有反馈有限、在配置开合范围内且未过期才允许输出，起点来自实测位置。
- 停止夹爪时，普通话题仅在反馈仍新鲜时发送实测保持值，Action 请求取消。控制器取消拒绝会显示错误；具体机械安全行为由实际驱动/控制器定义。
- 页面默认配置没有指向已验证的物理底盘或夹爪。接口尚未连接时显示等待；参数在硬件接入前仍可查看、修改和保存。
- 新接口检查话题冲突；网页“加入录制方案”可把命令/反馈加入机器人 ROS 录制方案。

配置字段定义见 `hc_teleop_recv/peripheral_config.py`，纯数值行为位于 `peripherals.py`，ROS 适配位于 `peripheral_ros.py`。

## 手柄动作与参考重置

配置可包含 `actions`，由网页机器人配置保存。默认不绑定回位/录制手势，也不指定姿态：

```yaml
actions:
  home_pose_id: right_home
  home_gesture_enabled: true
  home_threshold: 0.95
  home_release_threshold: 0.35
  recording_buttons_enabled: true
  mark_gesture_enabled: true
  pause_button: right_secondary
  reset_reference_button: none
  posture_pose_id: right_home
  posture_weight: 0.001
```

`right_home` 必须是在同一机器人配置中保存的初始姿态 ID。X/Y 开始/停止录制；双摇杆按下标记录制；双摇杆向外推到底回位；暂停和重新对齐可选右 B、左 Menu、右 Menu 或不绑定。

新增 `~/reset_reference`（Trigger）只清除相对参考，`~/home`（Trigger）提交回位请求。管理器通过 `~/set_motion_active`（SetBool）完成 ServoP 与 MoveJ 的控制交接。回位完成后自动绑定新 FK，不要求松开再按侧握键。动作执行结果通过 `~/events` 回传头显，并保留在状态的 `last_action`；回位期间 `motion_active=true`。

夹爪配置 `require_enable_button: false` 可让模拟量直接控制夹爪，默认 `true` 保留持续按住使能键的方式。总使能与反馈条件仍适用。

## 低频诊断日志

- 坐标超限错误记录设备（head/left/right）、XYZ（米）、该设备跟踪状态、跟踪标志、序号和 VR 时间戳；UDP 模式同时记录来源地址。
- 拒包、VR 回传失败、恢复使能和停止事件各使用固定的汇总槽：首次事件在下一控制周期输出，此后每类最多每 30 秒输出一次，包含本次汇总数量和最新样本。停止产生事件后，尚未输出的计数仍会在窗口到期时汇总。
- 每类只保存计数、时间和最多 1024 字符的最新样本，不缓存原始数据包或逐帧历史。`last_packet_error` 是最近一次拒包原因，不表示当前仍在持续拒包；应结合收包/拒包计数判断。
- ROS 状态与控制话题的发布频率不受日志限频影响。此改动不放宽输入校验。
