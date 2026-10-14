# 接入灰度摄像头小车

目标接口适合 RT1064 + 灰度摄像头 + 编码器 + 陀螺仪。当前仓库只完成桌面仿真，没有接入厂商驱动，也没有测量 MCU 的耗时。

先明确线的含义：`bg_extract()` 把一个暗色区间中心当作引导线中心。传统白底黑边赛道要先配对左右边界、输出可行走廊中心，再将候选写入 `BGObservation`。否则算法可能把左右边界当成两条路线。搜索和时序接口可以复用，扫描器不能直接复用。

移植时复制 `core/branchguard.c` 与 `core/branchguard.h`，接入数学库。状态应由调用方长期持有，每辆车单独初始化；图像、里程和转角需对应相邻两帧，而不是每次将累计里程传入。

```c
static BGState state;
static BGObservation observation;

void perception_init(void) {
    bg_init(&state);
}

/* Encoder delta is metres, gyro delta is radians, left yaw positive.
   This example uses the synthetic calibration; replace it before driving. */
BGOutput decide(const uint8_t *gray, float ds, float dpsi,
                float dt, int route) {
    bg_extract(gray, BG_WIDTH, &observation);
    return bg_step(&state, &observation, route, ds, dpsi, dt, BG_FULL);
}
```

在静态标定板上先校正 `bg_distance()` 与像素横向投影，再标定轴距、舵机中位、角度与 PWM 的对应关系。用地面点验证投影误差，尤其检查远端扫描行。实车姿态变化明显时，固定投影需要升级。

编码器与陀螺仪的符号要统一：前进 ds 为正、左转 dψ 为正。`dt` 允许 0–0.25 s 之间的正值，其他输入检查见 `bg_step()`。倒车、长时间停顿后的恢复和路线指令切换需要上层自行管理，必要时重新初始化状态。

建议先录制图像和增量运动，在电脑离线回放，观察候选点、选中路径和 `pending` 的频率。能稳定解释每次选择后，再在低速下接入执行器。若 `pending` 连续出现，先排查投影、时间同步和运动误差，直接增加确认时间可能掩盖问题。

输出 `speed` 为速度建议，`steering` 为转向角建议；它们不直接对应电机占空比或舵机 PWM。底层仍需测速闭环、限幅和独立停车逻辑。`stop` 不表示车辆已实际停止。
