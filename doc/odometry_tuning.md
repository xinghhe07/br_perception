# 里程计验证与调参记录 (Point-LIO, Mid-360)

> 2026-09-12 · 对应任务书 §十.2「前置动作(必做)」与 §15 技术栈
> 对象: `~/lio_ws/9000_point_lio_ros2_Mid-360` (Point-LIO ROS2 移植版) + Livox Mid-360

## 一、前置动作核查结果 (全部实测)

| 项 | 配置值 | 实测/依据 | 结论 |
|---|---|---|---|
| 点云格式 | `xfer_format=1` (CustomMsg) | `/livox/lidar` = CustomMsg, Point-LIO `lidar_type: 1` ✓ | 正确 (混用会"没点云") |
| 点云频率 | — | **10.0 Hz**, 20064 点/帧 (≈200k pts/s) | 符合 Mid-360 规格 |
| IMU 频率 | `imu_time_inte: 0.005` | **200.1 Hz** | 一致 |
| IMU 单位 | `acc_norm: 1.0` (g) | 静止 \|a\|≈0.97 g, 陀螺 <0.03 rad/s | 正确 (填 9.81 会错) |
| 饱和值 | `satu_acc: 3.0` / `satu_gyro: 35.0` | 陀螺量程 ±2000°/s = 34.9 rad/s | 合理 (激进运动需实车复测) |
| 外参 | `extrinsic_T: [-0.011,-0.02329,0.04412]` | Livox 官方 Mid-360 IMU↔LiDAR 偏移; `extrinsic_est_en=false` | 正确 |
| 时间同步 | `time_lag_imu_to_lidar: 0.0` | 点云与 IMU 同源 (同一台 Mid-360, 共时钟) | 0 正确 |

## 二、实测基线 (静止)

- 静止 **35 s**: 最大偏离 x 24 mm / y 43 mm / z 21 mm; 线性漂移 2–7 mm/min
  (含初始化收敛段, 偏保守)
- 测量方法: 订阅 `/aft_mapped_to_init`, 抛弃前 30 s 收敛段, 统计均值/标准差/峰峰值/线性漂移率

## 三、调参决策: **本平台不做密化**

- **当初调参的那台机器**是 **2 核 VM**（⚠️ 2026-09-25 更正措辞：原文写"本机是 2 核 VM"，
  而现在的算力平台是本机 **GMKtec NucBox K6 / Ryzen 7 7840HS（8C16T）** —— 下面这些结论
  都是**在 2 核 VM 上**得出的）; Point-LIO 单进程已占 ~56% CPU。
- 密化档 (任务书选项 `point_filter_num: 1`、`filter_size_surf/map: 0.3`) 会把 CPU 打满,
  实测全栈同跑时点云已从 10 Hz 掉到 **7.8 Hz** —— 丢帧造成的误差远大于密化收益。
- 结论: 当前 `point_filter_num=3 / filter_size=0.5` 是 2 核环境下的正确工作点。
  换到**本机（GMKtec NucBox K6 / Ryzen 7 7840HS，8C16T）**后可切密化档, 需重新实测对比。

## 四、误差的真正来源与对策

任务书 §十.2 已定性: 场地**自相似/平坦**场景下某些方向弱可观, 属原理性问题, 调参无解。
本方案的对策是**场地先验重定位**, 已在 `src/fusion/coordinate_transformer.cpp` 落地:

- `inject_pose_prior(x, y, yaw)` — BR_RESET 时把启动区/重试区已知位置注入为 odom→world 基准 (§4.1 v1.6)
- `odometry_reliable()` — 退化/振动/超时时置 false 并降权 (×0.5), 输出到 PerceptionFrame (§9.1)
- `relative_pose_delta()` — 退化时改用帧间局部坐标差, 避免世界坐标整体偏移 (§4.1 v1.5)

## 五、待办

- [ ] **实车闭环测试**: 手持雷达走 5–10 m 闭环回原点, 用起终点偏差评估漂移 (比静止测试更有意义)
- [ ] **本机（7840HS，8C16T）**上做密化档 A/B 对比 (同一段闭环轨迹)
- [ ] 时间同步精测: 需要时可跑 LI-Init 标定 `time_lag_imu_to_lidar`
- [ ] 外参实测: 目前用 Livox 官方值, 装车后建议实测复核 (雷达-车体 `base_link←livox_frame` 仍是占位)
