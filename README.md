# N-Car 倒立摆小车项目

**平台**: Infineon TC264D (TriCore)  
**开发环境**: AURIX Development Studio  
**编译器**: TASKING C/C++  
**版本**: v2.0  
**更新日期**: 2025-01-09

---

## 📋 目录

- [项目简介](#项目简介)
- [硬件配置](#硬件配置)
- [软件架构](#软件架构)
- [快速开始](#快速开始)
- [IMU姿态解算](#imu姿态解算)
- [参数调整](#参数调整)
- [常见问题](#常见问题)

---

## 项目简介

N-Car 是一个基于 TC264D 微控制器的倒立摆平衡小车项目。采用三环PID控制（速度环→角度环→陀螺仪环），通过 IMU660RB 惯性测量单元实现姿态感知和平衡控制。

### 主要特性

- ✅ **双IMU算法**：一阶互补滤波 + EKF扩展卡尔曼滤波
- ✅ **三环PID控制**：速度环(20ms) → 角度环(5ms) → 陀螺仪环(2ms)
- ✅ **参数可调**：IPS114液晶 + 4按键菜单系统
- ✅ **参数保存**：Flash存储，掉电不丢失
- ✅ **双电机驱动**：动量轮 + 驱动轮

---

## 硬件配置

### 核心硬件
- **主控**: TC264D (TriCore 架构，200MHz)
- **IMU**: IMU660RB (6轴，加速度计+陀螺仪)
- **显示**: IPS114 彩屏 (SPI接口)
- **电机**: 直流减速电机 × 2（动量轮 + 驱动轮）
- **驱动**: TB6612FNG 电机驱动模块
- **编码器**: 正交编码器 × 2

### 引脚分配
详见项目根目录 `推荐IO分配.txt`

---

## 软件架构

### 目录结构

```
N_Car/
├── code/                  # 用户代码
│   ├── imu.c/h           # IMU驱动（双算法支持）
│   ├── pid.c/h           # 三环PID控制
│   ├── motor.c/h         # 电机驱动
│   ├── servo.c/h         # 舵机控制
│   ├── menu.c/h          # 菜单系统
│   ├── param_save.c/h    # 参数保存
│   ├── buzzer.c/h        # 蜂鸣器
│   └── EKF/              # EKF库文件
│       ├── Attitude.c/h          # 姿态解算接口
│       ├── QuaternionEKF.c/h     # 四元数EKF实现
│       ├── kalman_filter.c/h     # 卡尔曼滤波器
│       └── matrix.c/h            # 矩阵运算
├── user/                  # 主程序
│   ├── cpu0_main.c       # CPU0主函数
│   ├── isr.c             # 中断服务程序
│   └── ...
├── libraries/             # 逐飞库
├── Debug/                 # 编译输出
└── CLAUDE.md             # 项目详细文档
```

### 控制架构

```
Timer (1ms) 
    ↓
count % 2 == 0  → imu_update()          (2ms, 陀螺仪环)
    ↓
count % 5 == 0  → angle_control()       (5ms, 角度环)
    ↓
count % 20 == 0 → speed_control()       (20ms, 速度环)
    ↓
motor_control() → PWM输出
```

---

## 快速开始

### 1. 环境准备

1. 安装 **AURIX Development Studio** (v1.9.4+)
2. 导入项目：`File → Import → Existing Projects`
3. 选择 `N_Car` 文件夹

### 2. 编译烧录

```bash
# 方法1: ADS 界面
右键项目 → Build Project

# 方法2: 命令行
cd Debug
make clean
make
```

烧录：连接 MiniWiggler 调试器，点击 `Debug` 按钮

### 3. 首次运行

1. **IMU校准**：
   - 车体水平放置
   - 进入菜单：`Main → IMU → Calibrate`
   - 等待校准完成（约2秒）

2. **机械中值调整**：
   - 观察车体是否垂直
   - 菜单调整：`Main → IMU → Machine Angle`
   - 微调至车体竖直

3. **PID调试**：
   - 从速度环开始：`Main → PID → Speed PID`
   - 逐步调整 Kp、Ki、Kd

---

## IMU姿态解算

### 算法选择

系统支持两种IMU姿态解算算法，通过修改代码切换：

**文件**: `code/imu.c` (约第19行)
```c
// ======================== 算法选择配置 ========================
uint8 imu_algorithm_select = 0; // 0=互补滤波, 1=EKF
// ================================================================
```

修改后需要**重新编译和烧录**！

### 算法对比

| 项目 | 一阶互补滤波 (0) | EKF卡尔曼滤波 (1) |
|-----|-----------------|------------------|
| **计算量** | 低 | 中 |
| **CPU占用** | <1% | ~3% |
| **输出角度** | pitch | roll/pitch/yaw |
| **响应速度** | 快 | 快 |
| **抗干扰性** | 中 | 强 |
| **精度** | 中 | 高 |
| **零偏估计** | 需手动校准 | 自适应估计 |
| **建议场景** | 日常平衡控制（默认） | 高精度姿态需求 |

### 使用建议

- ✅ **新手/调试阶段**：使用互补滤波（默认），简单可靠
- ✅ **比赛/高性能**：使用EKF，精度更高，抗干扰强
- ⚠️ **注意**：EKF需要更多CPU资源，确保系统负载不超标

---

## API 使用说明

### IMU接口

```c
// 初始化
uint8 imu_init(void);  // 返回1=成功, 0=失败

// 数据更新（在2ms中断中调用）
void imu_update(void);

// 获取姿态角（单位：度）
float imu_get_pitch(void);  // 俯仰角（两种算法均有效）
float imu_get_roll(void);   // 横滚角（仅EKF有效）
float imu_get_yaw(void);    // 偏航角（仅EKF有效）

// 陀螺仪校准（车体静止时调用）
void imu_calibrate_gyro(uint16 sample_count);  // 0=默认2000次
```

### 使用示例

```c
void setup(void)
{
    // 初始化IMU
    if (imu_init() == 1) {
        printf("IMU初始化成功\n");
    }
}

void timer_interrupt(void)  // 1ms定时器
{
    static uint16 count = 0;
    count++;
    
    if (count % 2 == 0) {
        imu_update();  // 2ms更新IMU
    }
}

void control_loop(void)
{
    // 获取姿态角用于控制
    float pitch = imu_get_pitch();
    float roll  = imu_get_roll();   // EKF算法有效
    float yaw   = imu_get_yaw();    // EKF算法有效
    
    // PID控制
    // ...
}
```

---

## 参数调整

### 互补滤波参数

**位置**: `code/imu.c`

```c
uint8 gyro_ration = 4;     // 陀螺仪权重（1-10）
uint8 acc_ration = 4;      // 加速度计权重（1-10）
float call_cycle = 0.002f; // 更新周期（秒，固定2ms）
```

**调整建议**：
- `gyro_ration` ↑ → 响应更快，但易受零漂影响
- `acc_ration` ↑ → 抗零漂能力强，但对振动敏感
- 通常保持两者相等，范围 3-6

### EKF参数

**位置**: `code/imu.c` 中的 `imu_init()` 函数

```c
IMU_QuaternionEKF_Init(
    100,        // 四元数过程噪声
    0.00001,    // 陀螺仪零偏过程噪声  
    100000000,  // 加速度计量测噪声
    0.9996,     // 渐消因子（防止发散）
    0.002f,     // 更新周期（秒）⚠️ 必须与实际调用周期一致
    0           // 低通滤波系数（0=不使用）
);
```

**调优指南**：
- 姿态抖动 → 增大量测噪声（如 200000000）
- 响应太慢 → 减小过程噪声（如 50）
- 零偏估计差 → 调整零偏过程噪声（如 0.0001）

⚠️ **重要**：`dt` 参数（第5个）必须设为 `0.002f`（2ms），与 `pid.c` 中 IMU 调用周期一致！

### 机械中值

**位置**: 菜单 `Main → IMU → IMU Params → Machine Angle`

作用：补偿传感器安装角度偏差，使车体竖直时输出为0度。

---

## 常见问题

### Q1: 如何切换IMU算法？

**A**: 修改 `code/imu.c` 约第19行：
```c
uint8 imu_algorithm_select = 0; // 改为1启用EKF
```
修改后需要**重新编译和烧录**。

---

### Q2: 姿态角抖动严重？

**A**:
- **互补滤波**：增大 `acc_ration`（如改为6-8）
- **EKF算法**：增大量测噪声参数
- **硬件**：检查IMU连接，确保安装牢固

---

### Q3: EKF的roll和yaw一直是0？

**A**: 检查算法选择：
```c
printf("当前算法: %d\n", imu_algorithm_select);
// 0=互补滤波（只输出pitch）
// 1=EKF（输出roll/pitch/yaw）
```

---

### Q4: 角度有偏差？

**A**:
1. 静止状态下进行陀螺仪校准（菜单 → Calibrate）
2. 调整机械中值参数（菜单 → Machine Angle）
3. 确保车体水平放置时进行校准

---

### Q5: 修改更新周期？

**A**: 如需修改（不推荐），需同步修改：
1. `pid.c` 中的调用频率：`if (count % N == 0)`
2. `imu.c` 中 `call_cycle` 参数（互补滤波）
3. `imu.c` 中 EKF 的 `dt` 参数

---

### Q6: 编译错误？

**A**: 常见错误：
- **EKF文件找不到**：确保 `code/EKF/` 文件夹完整
- **变量未定义**：检查是否正确包含 `imu.h`
- **链接错误**：右键项目 → Clean Project，然后重新编译

---

## 调试技巧

### 1. 串口输出姿态角

```c
printf("Pitch: %.2f  Roll: %.2f  Yaw: %.2f\n",
    imu_data.pitch, imu_data.roll, imu_data.yaw);
```

### 2. 查看原始数据

```c
printf("ACC: %d,%d,%d  GYRO: %d,%d,%d\n",
    imu_data.acc_x, imu_data.acc_y, imu_data.acc_z,
    imu_data.gyro_x, imu_data.gyro_y, imu_data.gyro_z);
```

### 3. 监控CPU占用

```c
// 在中断中统计执行时间
uint32 start_time = systick_getval();
imu_update();
uint32 end_time = systick_getval();
printf("IMU耗时: %d us\n", (start_time - end_time) / 200);
```

---

## 性能参数

| 项目 | 数值 |
|-----|------|
| **IMU更新频率** | 500Hz (2ms周期) |
| **陀螺仪量程** | ±2000°/s |
| **加速度计量程** | ±8g |
| **角度分辨率** | 0.01° |
| **互补滤波CPU占用** | <1% |
| **EKF CPU占用** | ~3% |

---

## 坐标系说明

```
      Z (yaw)
      ↑
      |
      |
      +-----→ Y (roll)
     /
    /
   ↙
  X (pitch)
```

- **X轴（车体前进方向）**: pitch 俯仰角
- **Y轴（左右方向）**: roll 横滚角  
- **Z轴（竖直向上）**: yaw 偏航角

---

## 开发记录

### v2.0 (2025-01-09)
- ✅ 集成 EKF 扩展卡尔曼滤波算法
- ✅ 支持双算法切换（互补滤波 + EKF）
- ✅ 优化代码结构，添加 Doxygen 注释
- ✅ 修正 EKF 时间参数（1ms → 2ms）
- ✅ 完善文档和使用指南

### v1.0 (2022-09-15)
- ✅ 基础平衡控制实现
- ✅ 三环PID控制
- ✅ 一阶互补滤波
- ✅ 参数菜单系统

---

## 参考资料

- **逐飞TC264开源库**: [https://github.com/SeekFree/TC264_Library](https://github.com/SeekFree/TC264_Library)
- **项目详细文档**: [CLAUDE.md](CLAUDE.md)
- **AURIX官方文档**: Infineon AURIX TC2xx User Manual

---

## 许可协议

本项目基于 **GPL-3.0** 开源协议。

使用逐飞科技 TC264 开源库，遵循其 GPL-3.0 许可。

---

## 联系方式

- **项目**: N-Car 倒立摆小车
- **维护**: N_Car 项目组
- **更新**: 2025-01-09

---

**祝您使用愉快！** 🚗💨
