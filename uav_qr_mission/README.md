# uav_qr_mission

ROS Noetic 任务节点。接收三个目标码和其中一个定位目标，累计找码进度，发布固定的相机到二维码 TF，并记录定位目标在指定坐标系中的点。

| 接口 | 消息类型 | 用途 |
|---|---|---|
| `/uav_qr_mission/set_task` | `uav_qr_mission/SetTask` | 提交本轮任务 ID、三个有效码和一个定位目标 |
| `/uav_odin_qr/detections` | `uav_odin_qr/DetectionArray` | 记录已找到的码，检查同帧重复内容 |
| `/uav_odin_qr/tf_targets` | `uav_odin_qr/QrTransformArray` | 按原图时间戳和中心像素匹配定位目标 |
| `/tf` | `tf2_msgs/TFMessage` | 发布 `ODIN 相机光学系 → uav_qr_report_target` |
| `/uav_qr_mission/result` | `uav_qr_mission/MissionResult` | 保留本轮最新状态，供牛耕程序和晚接入的订阅者读取 |

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
