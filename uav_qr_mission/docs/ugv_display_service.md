# 小车屏幕服务接口

服务名默认 `/ugv/qr_display/update`，类型 `uav_qr_mission/ReportQr`。
已有小车服务需适配此类型，或将发送端 `service` 参数指向实际的同类型服务。
本包不直接操作屏幕，屏幕程序在接收端按下列合同更新自己的 UI。

## 请求与响应

```text
请求 report:
  mission_id: 本轮任务唯一 ID
  targets[]: 一条或多条 QrTargetResult
    content: 二维码原文，例如 QR001
    is_drop_target: 是否投放目标
    found: true
    position_valid: true
    position.header.stamp: 深度测量时刻（ROS secs/nsecs），不是发送时间
    position.header.frame_id: wgs84
    position.latitude / longitude: 度
    position.altitude: WGS84 椭球高，米
    fix_stamp / attitude_stamp: 对应 RTK / 姿态采样时间
    revision: 本轮该码的位置版本，当前首次确认后为 1
响应:
  accepted: true 表示整批数据已被显示后端接收/保存
  status: 诊断文字
```

二维码原文为严格 UTF-8，不用 TF 数字后缀作为目标 ID。
任务始终注册两个普通码和一个投放码，各码分别保存停点采集后取得的首个有效位置。
上报数据只含已定位目标；未定位不会用 (0,0) 冒充有效经纬度。

## 接收端规则

1. 先验证整批字段，再保存，全部成功才返回 `accepted=true`。
2. 用 `(mission_id, content)` 作为行的键，用 `revision` 做版本比较。
   相同版本重试应返回成功，不增加重复行；旧版本不能覆盖新版本。
3. 按 mission_id 分开存储。旧任务的迟到消息不能覆盖当前选定任务的屏幕。
4. UI 更新须经屏幕框架的线程队列执行，不能从 ROS 服务线程直接修改 Qt 控件。
5. 可以先将整批可靠写入屏幕程序的更新队列，再返回成功；不要在真正接收之前确认。

发送端在独立进程内调用服务，超时杀掉该次调用，稍后重试；无线链路或服务卡住不会阻塞飞行线程。
若小车已显示但响应丢失，发送端仍会重试，因此接收端幂等性是接口的一部分。
发送端重启会从任务节点的 latched 完整结果恢复，并重新发送尚无本地确认的记录。
发送端在新位置出现时发送完整有效快照，空闲时默认每 10 秒重发完整快照，
因此小车屏幕进程重启、丢失内存后，即使没有新二维码出现也能恢复已有记录。
周期快照使用原来的测量时间与 revision，不表示重新测量；接收端继续幂等更新。
任务节点自身当前没有磁盘任务恢复，重启后需重新注册任务。

单条首次定位可立即进入下一次发送周期；同周期出现的多条一起发送。默认重试周期 1 秒，
服务调用预算 2 秒，可通过 `qr_report_sender.launch` 调整。独立进程预热和重建另有默认 10 秒上限，
正常情况下在第一次发现目标前已完成预热；进程保持复用，不为每个二维码重复加载 ROS。

## 运行与 FKIE 接入

```bash
source /path/to/qr_ws/devel/setup.bash
roslaunch uav_qr_mission qr_report_sender.launch \
  service:=/ugv/qr_display/update
rostopic echo /qr_report_sender/status
```

小车端须编译同一版本的消息/服务定义；服务含自定义消息，不能仅复制 srv 文本。
将服务加入现有 FKIE 的服务同步配置，核对双向 ROS 地址及 TCPROS 可达性。
无线电和 FKIE 的实际配置不由本包修改；正式接入需在两台机器上验证服务发现、单条、批量、断线重连及重复请求。

上报确认与飞行阶段独立。无线断开时，定位结果继续保留并重试；不会自动宣称已送达。
当前任务完成全网格且投放码有效后即可飞往等待点，不要求小车 ACK 才转场。

可参考 maxt 的 `test/qr_stop_survey_contract.py` 中 `FlightDouble.report`：它模拟逐条/批量接收，
并故意丢失第一次响应，检查重试与按键更新；它不代表真实屏幕或无线链路的验证。
