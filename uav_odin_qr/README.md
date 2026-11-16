# uav_odin_qr

## 停点触发模式

默认仍连续处理图像。新增两个标准消息控制接口，供 `uav_qr_mission` 使用：

- `/uav_odin_qr/continuous`（`std_msgs/Bool`）：false 暂停连续解码/深度查询，清除尚未处理的图像；true 恢复连续模式。
- `/uav_odin_qr/capture_after`（`std_msgs/Time`）：触发下一张采样时间不早于指定时刻的图像，处理一次即等待下次触发；零时间取消。

ODIN 相机仍连续采集；这里从图像流选取稳定延时之后的新帧，不是硬件快门触发。
已经开始处理的旧帧可能完成，但任务节点还会校验时间，不能将它计入新采集。
任务节点是这些控制话题的唯一写入者；不要同时运行多个任务节点控制同一视觉实例。

ROS Noetic 节点。使用 ODIN 驱动的去畸变图解码二维码，输出中心像素，并由 OYO 查询同一图像帧对应的深度和 LiDAR 三维点。输入与查询遵循《OYO · 二维码定位接口交接》。

| 接口 | 类型 | 方向 |
|---|---|---|
| `/odin1/image/undistorted` | `sensor_msgs/Image` | ODIN → 节点 |
| `/uav_oyo/query_pixel` | `uav_oyo/QueryPixel` | 节点 → OYO |
| `/uav_odin_qr/detections` | `uav_odin_qr/DetectionArray` | 节点 → 下游 |
| `/tf` | `tf2_msgs/TFMessage` | 节点 → TF |
| `/uav_odin_qr/tf_targets` | `uav_odin_qr/QrTransformArray` | 节点 → 下游 |

## 构建与启动

包位置为 `~/devices_ws/src/uav_odin_qr`。用 Jetson 的系统 Python 3.8、ROS 的 cv_bridge 和系统 OpenCV、NumPy。依赖 `zxing-cpp==2.2.0` 安装在 `~/devices_ws/.uav_odin_qr_deps`。在 ARM64 上从 PyPI 的源码发行包构建 wheel，再用系统 Python 的 pip 安装到该路径。

```bash
source /opt/ros/noetic/setup.bash
cd ~/devices_ws
catkin_make -j2 --pkg uav_odin_qr '-DCATKIN_WHITELIST_PACKAGES=uav_oyo;uav_odin_qr'
source devel/setup.bash
roslaunch uav_odin_qr uav_odin_qr.launch
```

沿用运行 ODIN 和 OYO 的 ROS Master。启动文件只带起二维码节点。另开一个加载了同一工作区的终端查看数据。

```bash
rostopic echo /uav_odin_qr/detections
```

## 相机系二维码 TF

- 父帧是与 OYO 标定对应的 ODIN 相机光学系。默认使用输入图像的 `header.frame_id`。图像帧名为空或使用其他坐标系时，在启动时传入已确认的光学帧名 `camera_frame:=<实际光学帧名>`。相机系三维点的轴向为 X 右、Y 下、Z 前。若要在 LiDAR 或地图坐标系中查看，该光学帧需要在现有 TF 树中连接到对应父帧。
- 每个有效目标在 `/tf` 中有一个独立子帧 `uav_odin_qr_<会话>_<编号>`，原点为 OYO 返回的相机系 `camera_point`，时间戳为对应深度帧。旋转为单位四元数，子帧轴向与相机对齐。
- `/uav_odin_qr/tf_targets` 把当前帧的二维码内容和中心像素与 TF 子帧配对。同内容的两个码也各有子帧。相邻帧按内容和像素距离关联，匹配有歧义或目标超时后分配新编号。
- 每个处理帧都会发布一条目标列表。深度无效、时间戳无效或父帧不明的目标不发 TF。TF 缓存可能仍能查到旧变换，消费端以新鲜的目标列表判断当前有哪些目标。

## 数据语义

- `header.stamp` 是图像原始采样时间，`header.frame_id` 透传图像帧名。中心是原尺寸去畸变图中四角的算术平均，左上角为像素原点。
- `depth_m` 是相机向前的 Z 深度，单位为米。`lidar_point` 的坐标系和时间戳来自 OYO 的返回值。`sync_dt` 是图像减深度时间，单位为秒。
- `detections: []` 表示该处理帧未解出二维码。已识别但深度不可用时，仍输出内容和中心，`depth_valid=false`，深度、三维坐标和同步差为 `NaN`，`status` 给出原因。
- 新帧尚未同步时按相同图像时间戳短重试。每帧查询总预算默认 0.8 秒。超过预算时不使用旧结果。输入断流时停止产生新结果。

## 验收

`test/ros_contract.py` 在独立测试 Master 上配合模拟 OYO 服务核对像素、时间戳、失败标志、服务超时恢复和同框同内容二维码的独立 TF。实际码布在约 4、8、10 米各采 3 段、每段 30 秒。每段以完整可见码帧为分母，二维正确率和“正确识别且深度有效”比例各至少 90%，连续缺结果不超过 1 秒，错误内容为 0。对褶皱但可见的码帧计入分母。用实际相机及深度流采集三维结果，并抽查中心、同帧 OYO 响应和实测距离误差。
