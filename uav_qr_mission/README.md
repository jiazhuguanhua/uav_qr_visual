# uav_qr_mission

## 停点扫描与小车上报（新增）

任务现在为三个注册码分别保存 WGS84 位置。`result.targets[]` 的每项包含 `content`、
`is_drop_target`、`found`、`position_valid`、`position`、采样时间、状态和版本。
`all_localized` 表示三个码都定位成功；原来的 `wgs84_position` 等单目标字段继续对应投放码。
旧 `complete` 仍保留 arena 条件，不用于停点扫描的结束判定。

`SetTask` 新增 `capture_only`：

- `false`：兼容原来的持续识别任务，每个处理帧都可更新注册目标。
- `true`：只接受显式停点采集。飞行期间通过标准话题暂停视觉解码和深度查询，
  接受采集后触发下一张图像；同一采集最多使用一帧。

```bash
rosservice call /uav_qr_mission/set_task \
  "{mission_id: 'survey_01', target_ids: ['QR001','QR002','QR003'], position_target_id: 'QR003', capture_only: true}"
```

`/uav_qr_mission/capture` 使用 `CaptureQr`，是立即返回的控制服务：
`command=START`、任务 ID、唯一本点 capture_id、稳定延时结束后的 `not_before` 和超时秒数。
通过 `result.capture_id/status/done/success/stamp` 获取完成结果。
图像与深度测量时间均必须进入新窗口；收到旧深度时重新触发下一帧。
空检测也是完成了一次采集，不等于找齐目标。定位等待只在有界窗口内进行。

取消使用同一服务的 `command=CANCEL`；取消 ID 会被记住，防止迟到的 START 再次打开窗口。
时间过期的请求拒绝。接收图像与求位置的工作不阻塞控制服务。
`/finish_survey` 按任务 ID 冻结结果，正在采集时拒绝冻结；冻结后不能再启动采集。

`result.header.stamp` 是 2 Hz 发布心跳，与各二维码的测量时间分开。
`geolocation_input_status` 检查当前原始 RTK/IMU 是否就绪；不是保存坐标的年龄。
目标离开视野后保存位置仍有效，第一次有效位置保存后不自动追踪移动目标。

`qr_report_sender.launch` 将新定位结果通过服务送到小车，支持单条、批量、超时重试。
接口、幂等性及 FKIE 接入说明见 [小车服务合同](docs/ugv_display_service.md)。
maxt 的 `qr_stop_survey.launch` 提供九点停顿扫描与最后 5 m 等待的完整 BT。
新接口改变了 ROS MD5，发布/订阅/服务两端须全部使用新构建并重启。

ROS Noetic 任务节点。接收三个目标码和其中一个定位目标，累计找码进度，发布固定的相机到二维码 TF，并独立保存定位目标的场地坐标和 WGS84 经纬高。

| 接口 | 消息类型 | 用途 |
|---|---|---|
| `/uav_qr_mission/set_task` | `uav_qr_mission/SetTask` | 提交本轮任务 ID、三个有效码和一个定位目标 |
| `/uav_odin_qr/detections` | `uav_odin_qr/DetectionArray` | 记录已找到的码，检查同帧重复内容 |
| `/uav_odin_qr/tf_targets` | `uav_odin_qr/QrTransformArray` | 按原图时间戳和中心像素匹配定位目标 |
| `/tf` | `tf2_msgs/TFMessage` | 发布 `ODIN 相机光学系 → uav_qr_report_target` |
| `/uav_qr_mission/result` | `uav_qr_mission/MissionResult` | 保留本轮最新状态，供牛耕程序和晚接入的订阅者读取 |
| `/mavros/global_position/raw/fix` | `sensor_msgs/NavSatFix` | 主天线 ant1 的 WGS84 经纬度、椭球高 |
| `/mavros/imu/data` | `sensor_msgs/Imu` | 机体 FLU 到地理 ENU 的姿态 |
| `/tf_static` | `tf2_msgs/TFMessage` | 配置的机体、相机和两根 RTK 天线安装变换 |

## 启动

把两个包放在 Jetson 的 `~/devices_ws/src`，构建时将任务包加入工作区现有的 catkin 白名单。沿用 ODIN 和 OYO 所在的 ROS Master。

```bash
source /opt/ros/noetic/setup.bash
cd ~/devices_ws
catkin_make -j2 '-DCATKIN_WHITELIST_PACKAGES=uav_oyo;uav_odin_qr;uav_qr_mission'
source devel/setup.bash
roslaunch uav_qr_mission uav_qr_mission.launch result_frame:=arena
```

将 `arena` 换成实际验收坐标系。启动新版 `uav_odin_qr` 时，若图像帧名为空，用其 `camera_frame` 参数填入标定对应的相机光学帧。该光学帧需要在 TF 树中能按测量时间连到场地帧。

另开终端加载工作区，提交示例任务并查看结果。

```bash
source ~/devices_ws/devel/setup.bash
rosservice call /uav_qr_mission/set_task \
  "{mission_id: 'run_01', target_ids: ['QR002', 'QR004', 'QR006'], position_target_id: 'QR004'}"
rostopic echo /uav_qr_mission/result
```

## 上游读取约定

- `found[i]` 对应 `target_ids[i]`，发现后在本轮保持为真。定位码出现两次时，仍可登记已找到，坐标等待唯一匹配。
- `report_tf_valid` 表示有当前观测。使用 `report_tf.header.stamp` 查询 `lookup_transform(场地帧, 'uav_qr_report_target', stamp)`。固定子帧的父帧沿用视觉节点的相机光学帧。父帧到场地的 TF 暂缺时，仍可取得相机相对变换。
- `position_valid` 表示已成功在 `position.header.frame_id` 下确认并保存一次坐标。目标移出视野后，保存的坐标仍属于本轮任务。三个码均找到且位置已保存时，`complete=true`。
- 当前目标失去有效观测时，`report_tf_valid` 变为假。TF 缓存可保留旧记录，上游用 `mission_id` 和有效标志判断能否采用。相同任务 ID 的重复请求保持已有进度，提交新任务 ID 会清空当前任务状态。

## WGS84 定位

仍然只定位 `position_target_id` 指定的二维码。`result` 新增字段：

| 字段 | 含义 |
|---|---|
| `wgs84_position_valid` | 本轮已保存一次有效 WGS84 位置 |
| `wgs84_position.latitude` / `.longitude` | 纬度 / 经度，度 |
| `wgs84_position.altitude` | WGS84 椭球高，米，不是相对地面高度或海拔 |
| `wgs84_position.header.stamp` | 定位使用的二维码深度测量时间 |
| `wgs84_fix_stamp` / `wgs84_attitude_stamp` | 实际选用的主天线定位 / 姿态采样时间 |
| `wgs84_status` | WGS84 定位状态，独立于原有任务 `status` |

`wgs84_position` 使用 `sensor_msgs/NavSatFix`。未定位时有效标志为假，经纬高为 NaN，
`status.status=STATUS_NO_FIX`。成功后该位置保留到新任务，不会因为二维码离开视野而失效；
同任务重复提交不清空位置，新任务清空。重启节点后需要重新下发任务。

WGS84 与 `arena` 分别保存各自首次成功取得的位置，两者可能来自不同测量时刻。
没有 `arena` TF 时，WGS84 仍可有效；原有 `complete` 定义不变，仍要求三个码找到且
`position_valid=true`。需要 WGS84 的消费端应单独检查 `wgs84_position_valid`。

### 安装配置与 TF

默认配置在 `config/geolocation.yaml`，可编辑后重启，也可指定其他文件：

```bash
roslaunch uav_qr_mission uav_qr_mission.launch \
  geolocation_config:=/absolute/path/to/geolocation.yaml
```

现有发布目录的 `qr_ground.launch` 会自动通过任务 launch 加载默认配置。
默认尺寸和命名沿用 `uav_tf_test/config/tf.yaml`，主天线为左侧 `uav_rtk_ant1`：

```text
uav_base_link                         FLU: x前、y左、z上
├── uav_camera                        [-0.105, 0, -0.17] m，朝下
│   └── uav_camera_optical_frame       x右、y下、z前，图像上方朝机头
├── uav_rtk_ant1                       [0, +0.1585, +0.105] m
└── uav_rtk_ant2                       [0, -0.1585, +0.105] m
```

以上安装边由任务节点通过 `/tf_static` 发布；配置会检查重复子帧、环路和断开的树。
不要同时运行发布这些相同子帧的 `uav_tf_test`。如果生产系统已有安装 TF 发布者，
将 `geolocation.publish_static_tf` 设为 `false`，并确保本配置与已有 TF 的外参一致。
计算始终使用本配置中的外参，便于检查实际采用的参数，不会从任意其他 TF 分支推断安装关系。

`wgs84_position.header.frame_id=wgs84` 只是地理坐标标签，不发布名为 `wgs84` 的 TF；
TF 平移要求米制笛卡尔坐标，不能直接填经纬度。

### 转换与时间匹配

1. 相机点转换至机体系，减去 ant1 的机体系安装位置，得到天线到目标的向量。
2. 使用 MAVROS `/mavros/imu/data` 的 FLU→ENU 四元数旋转整个向量，包括安装偏移。
3. 以 ant1 的经纬高建立 ENU 基底，将偏移加到天线的 WGS84 ECEF 坐标，再反算目标经纬高。

本机 MAVROS 的 raw/fix 发布实现已将原始 GPS 海拔转换为椭球高，此处不再重复做大地水准面补偿。
不将 MAVROS 姿态再次按 NED/FRD 转换；不将相机 optical 点再次手动换轴。

以二维码 **深度测量时间** 为基准，选缓存中时间最近的 RTK 和姿态消息。
默认允许 RTK 时间差最多 0.2 秒、姿态最多 0.1 秒，可在配置调整；不插值，
不使用超出容差的“最新数据”代替。比较使用纳秒时间戳，缓存按单调时钟保留 5 秒并有数量上限。
定位或姿态稍晚到达时，最多在该观测接收后 1 秒内重试；超时后等待下一次有效二维码观测。
设备之间的采样时钟必须对齐；时间差容差不等于定位精度保证。

定位拒绝无效 GNSS fix、非法经纬高、无姿态标记、无效四元数和不匹配的相机帧。
`raw/fix` 的 `NavSatStatus` 不能单独区分 RTK fixed / float，因此有效标志表示数据和时间匹配有效，
不保证 RTK 已固定解。输出协方差设为 UNKNOWN：不把天线定位精度冒充包含深度、外参和姿态误差的目标精度。

常见 `wgs84_status`：

| 状态 | 含义 |
|---|---|
| `WAIT_TASK` / `WAIT_OBSERVATION` | 等待任务 / 唯一且深度有效的定位码 |
| `WAIT_FIX` / `WAIT_ATTITUDE` | 尚无有效时间戳的对应输入 |
| `FIX_TIME_MISMATCH` / `ATTITUDE_TIME_MISMATCH` | 最近输入超出测量时间容差 |
| `FIX_INVALID` / `ATTITUDE_INVALID` | 输入数值或有效性标记不符合要求 |
| `CAMERA_FRAME_MISMATCH` | 视觉点的父帧与配置的光学帧不一致 |
| `OBSERVATION_EXPIRED` | 观测已超过等待时间，等下一次观测 |
| `VALID` | 本轮已保存 WGS84 位置 |

### 构建、升级与验证

`MissionResult` 新增字段导致 ROS 消息 MD5 变化。构建后，发布节点和所有订阅该消息的程序
都需要加载同一版本并重启；C++ 消费端也需重新编译。运行中的旧任务节点不会自动热更新。
已有任务进度仅在内存中，重启任务节点会清空，需重新下发任务。

单元测试（在加载此工作区后执行）：

```bash
python3 -m unittest discover -s /path/to/ws/src/uav_qr_mission/test -v
```

ROS 集成测试只允许连接 loopback Master；在两个独立终端加载工作区并设置相同测试网络：

```bash
export ROS_MASTER_URI=http://127.0.0.1:11338 ROS_IP=127.0.0.1
unset ROS_HOSTNAME
# 终端一：
roslaunch -p 11338 uav_qr_mission uav_qr_mission.launch
# 终端二：
python3 /path/to/ws/src/uav_qr_mission/test/ros_contract_wgs84.py -v
```

测试覆盖杆臂和姿态旋转、WGS84 正反算、时间边界、过期/无效输入、延迟数据恢复、同内容歧义、
静态 TF、保存/重置，以及 `arena` 定位和原任务完成逻辑。集成测试使用模拟定位，不代表实机 RTK 精度验收。

### GPS 与电脑时钟偏差（2026-11-16）

`geolocation/gps_clock_bridge: true` 将 GPS epoch 测量时间换算到电脑 ROS 时间后再匹配图像：

```
GPS epoch -> 减去 SYSTEM_TIME(epoch - boot) -> 飞控 boot
          -> 加上 MAVROS TIMESYNC estimated_offset -> 电脑 ROS
```

仅订阅 `/mavlink/from` 的 `SYSTEM_TIME` 与 `/mavros/timesync_status`，不修改 GPS、电脑或飞控时钟，也不修改 MAVROS。GPS 原始 `header.stamp` 仍保存在结果的 `fix_stamp`；IMU 已经由 MAVROS 换算到 ROS 时间，不重复补偿。该配置要求 raw/fix 保留 GPS epoch（当前已修正的 MAVROS 即如此）；若上游已经把 GPS 转成电脑时间，应关闭此开关。

只接受 RTT <= 10 ms 的同步观测，至少三条一致的 SYSTEM_TIME 后才就绪；同步状态超过 3 s、时间锚超过 5 s 或偏移跳变时停止配对并重新收敛。不使用消息接收时刻替代采样时刻。`max_fix_dt_s: 0.2` 约束换算后的采样差，不是 GPS 与电脑的绝对钟差。默认飞控 ID 为 1/1，可用任务节点参数 `clock_system_id`、`clock_component_id` 调整。

时间映射不替代图像输入。`TIMEOUT_WAIT_FRAME` 表示没收到符合窗口要求、时间戳一致的 detections/tf_targets 对；`TIMEOUT_LOCALIZING` 表示已收到图像对但定位未完成。行为树会打印具体采集失败状态，起飞前也会打印 `WAIT_CLOCK_SYNC`、`WAIT_CLOCK_ANCHOR`、`FIX_TIME_MISMATCH` 等等待原因。失败后仍按原策略定点等待，不自动继续投放或改变飞行模式。

只读地面检查（不会发布消息或调用飞控服务）：

```bash
source /home/dlab/qr_visual_release_20261114_Q1tLWg/env.sh
python3 "$QR_RELEASE/ws/src/uav_qr_mission/test/ground_clock_probe.py"
```

修改 Python 和 YAML 后重启二维码任务节点生效；maxt 日志修改已重新编译。不要重复启动 `/uav_qr_mission`：`qr_ground.launch` 已包含该节点，此时巡检入口保持 `start_qr_mission:=false`。
