# Core - 应用核心代码模块

**[根目录](../CLAUDE.md) > Core**

**模块职责**: STM32F1 IMU 项目的核心应用层代码
**状态**: 活跃开发中
**最后更新**: 2025-10-21 18:37:25 CST

---

## 变更记录 (Changelog)

### 2025-10-21 - 模块文档初始化
- 创建 Core 模块详细文档
- IMU 驱动模块化封装完成（v2.0）
- 软 I2C 驱动稳定运行

---

## 模块职责

Core 模块是项目的**应用层核心**，包含：
1. **主程序入口** (`main.c`) - 系统初始化与主循环
2. **IMU 驱动** (`imu.c/h`) - MPU6050 六轴传感器高级 API
3. **软 I2C 驱动** (`soft_i2c.c/h`) - GPIO 位操作模拟 I2C 通信
4. **外设初始化** - UART、TIM、DMA、GPIO 配置（HAL 库封装）

---

## 入口与启动

### 主程序流程 (`main.c`)

```c
int main(void)
{
    /* 1. 系统初始化 */
    HAL_Init();                      // HAL 库初始化
    SystemClock_Config();            // 时钟配置 (72MHz)

    /* 2. 外设初始化 */
    MX_GPIO_Init();                  // GPIO 配置
    MX_DMA_Init();                   // DMA 初始化 (USART3)
    MX_USART1_UART_Init();           // 调试串口 115200
    MX_TIM5_Init();                  // PWM 定时器 (舵机)
    MX_USART2_UART_Init();           // CRSF 接收机 (待配置 420000)
    MX_USART3_UART_Init();           // Jetson 通信 (待配置 921600)

    /* 3. 应用层初始化 */
    if (IMU_Init() != IMU_OK) {
        UartSendText("IMU初始化失败，检查线路和供电！\r\n");
    }

    /* 4. 主循环 */
    while (1) {
        IMU_Data_t imu_data;
        if (IMU_ReadData(&imu_data) == IMU_OK) {
            // 输出 IMU 数据 (10Hz)
            printf("ACC: %.2f %.2f %.2f | GYRO: %.1f %.1f %.1f | TEMP: %.1f°C\r\n",
                   imu_data.accel_x, imu_data.accel_y, imu_data.accel_z,
                   imu_data.gyro_x, imu_data.gyro_y, imu_data.gyro_z,
                   imu_data.temperature);
        }
        HAL_Delay(100);  // 10Hz 采样率
    }
}
```

### 系统时钟配置

```
HSE (8MHz 外部晶振)
    └─> PLL × 9
        └─> SYSCLK = 72MHz
            ├─> AHB = 72MHz
            ├─> APB1 = 36MHz (USART2/3, TIM5)
            │   └─> TIM5 时钟 = 72MHz (APB1 × 2)
            └─> APB2 = 72MHz (USART1, GPIO)
```

---

## 对外接口

### IMU 驱动 API (`imu.h`)

#### 核心函数

| 函数签名 | 功能 | 返回值 |
|---------|------|--------|
| `IMU_Status_t IMU_Init(void)` | 初始化 MPU6050（芯片验证 + 硬件复位 + 参数配置） | `IMU_OK` / `IMU_ERROR` |
| `IMU_Status_t IMU_ReadData(IMU_Data_t *data)` | 读取物理单位数据（g, °/s, °C） | `IMU_OK` / `IMU_ERROR` |
| `IMU_Status_t IMU_ReadRaw(IMU_RawData_t *raw)` | 读取原始 LSB 数据 | `IMU_OK` / `IMU_ERROR` |
| `IMU_Status_t IMU_SetAccelRange(IMU_AccelRange_t range)` | 设置加速度计量程（±2g ~ ±16g） | `IMU_OK` / `IMU_ERROR` |
| `IMU_Status_t IMU_SetGyroRange(IMU_GyroRange_t range)` | 设置陀螺仪量程（±250°/s ~ ±2000°/s） | `IMU_OK` / `IMU_ERROR` |
| `IMU_Status_t IMU_SetDLPF(IMU_DLPF_t dlpf)` | 设置数字低通滤波器（260Hz ~ 5Hz） | `IMU_OK` / `IMU_ERROR` |
| `IMU_Status_t IMU_VerifyChipID(void)` | 验证芯片 ID (WHO_AM_I == 0x68) | `IMU_OK` / `IMU_ERROR` |

#### 数据结构

```c
// 物理单位数据（推荐使用）
typedef struct {
    float accel_x, accel_y, accel_z;  // 加速度 (g)
    float gyro_x, gyro_y, gyro_z;      // 角速度 (°/s)
    float temperature;                 // 温度 (°C)
} IMU_Data_t;

// 原始 LSB 数据
typedef struct {
    int16_t accel_x, accel_y, accel_z;
    int16_t gyro_x, gyro_y, gyro_z;
    int16_t temperature;
} IMU_RawData_t;
```

#### 使用示例

```c
// 初始化
if (IMU_Init() == IMU_OK) {
    printf("IMU 初始化成功\n");
}

// 读取数据
IMU_Data_t data;
if (IMU_ReadData(&data) == IMU_OK) {
    printf("加速度: X=%.2fg, Y=%.2fg, Z=%.2fg\n",
           data.accel_x, data.accel_y, data.accel_z);
    printf("角速度: X=%.1f°/s, Y=%.1f°/s, Z=%.1f°/s\n",
           data.gyro_x, data.gyro_y, data.gyro_z);
    printf("温度: %.1f°C\n", data.temperature);
}

// 修改量程（提高精度）
IMU_SetAccelRange(IMU_ACCEL_RANGE_4G);
IMU_SetGyroRange(IMU_GYRO_RANGE_500DPS);
```

### 软 I2C API (`soft_i2c.h`)

| 函数签名 | 功能 | 返回值 |
|---------|------|--------|
| `void SoftI2C_Init(void)` | 初始化软 I2C（设置 SCL/SDA 为高电平） | 无 |
| `HAL_StatusTypeDef SoftI2C_WriteRegister(uint8_t dev_addr, uint8_t reg_addr, uint8_t value)` | 写单个寄存器 | `HAL_OK` / `HAL_ERROR` |
| `HAL_StatusTypeDef SoftI2C_WriteRegisters(uint8_t dev_addr, uint8_t reg_addr, const uint8_t *data, uint16_t length)` | 写多个寄存器 | `HAL_OK` / `HAL_ERROR` |
| `HAL_StatusTypeDef SoftI2C_ReadRegisters(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t length)` | 读多个寄存器 | `HAL_OK` / `HAL_ERROR` |

#### I2C 通信时序

```
读操作:
START | DEV_ADDR(W) | ACK | REG_ADDR | ACK |
REPEATED_START | DEV_ADDR(R) | ACK | DATA[0] | ACK | ... | DATA[n-1] | NACK | STOP

写操作:
START | DEV_ADDR(W) | ACK | REG_ADDR | ACK | DATA[0] | ACK | ... | DATA[n-1] | ACK | STOP
```

---

## 关键依赖与配置

### 硬件依赖

| 外设 | 引脚 | 配置 | 用途 |
|------|------|------|------|
| **软 I2C SCL** | PA12 | GPIO 开漏输出 + 上拉 + 高速 | MPU6050 时钟线 |
| **软 I2C SDA** | PA11 | GPIO 开漏输出 + 上拉 + 高速 | MPU6050 数据线 |
| **USART1 TX** | PA9 | 复用推挽输出 | 调试串口发送 |
| **USART1 RX** | PA10 | 输入浮空 | 调试串口接收 |
| **TIM5 CH1** | PA0 | TIM5 PWM 输出 | 俯仰舵机 PWM |
| **TIM5 CH2** | PA1 | TIM5 PWM 输出 | 水平舵机 PWM |
| **激光指示** | PA7 | GPIO 推挽输出 | 目标指示灯 |

### 软件依赖

```c
// 必需的 HAL 库头文件
#include "stm32f1xx_hal.h"        // HAL 库核心
#include "stm32f1xx_hal_gpio.h"   // GPIO 操作
#include "stm32f1xx_hal_uart.h"   // UART 通信
#include "stm32f1xx_hal_tim.h"    // 定时器/PWM
#include "stm32f1xx_hal_dma.h"    // DMA 传输

// 应用层头文件
#include "imu.h"                  // IMU 驱动
#include "soft_i2c.h"             // 软 I2C 驱动
#include "imu_port.h"             // 平台移植层
```

### 平台移植层 (`imu_port.h`)

IMU 驱动支持**跨平台移植**，通过 `imu_port.h` 适配不同平台：

| 平台宏定义 | 支持状态 | 备注 |
|-----------|---------|------|
| `IMU_PLATFORM_STM32_HAL` | ✅ 当前平台 | 使用软 I2C |
| `IMU_PLATFORM_STM32_STD` | ✅ 支持 | 标准库版本 |
| `IMU_PLATFORM_ARDUINO` | ✅ 支持 | Arduino Wire 库 |
| `IMU_PLATFORM_ESP32` | ✅ 支持 | ESP-IDF I2C 驱动 |

---

## 数据模型

### MPU6050 寄存器地图（关键部分）

| 寄存器地址 | 名称 | 读/写 | 功能 |
|-----------|------|------|------|
| `0x75` | WHO_AM_I | R | 芯片 ID (固定值 0x68) |
| `0x6B` | PWR_MGMT_1 | R/W | 电源管理（复位、时钟源） |
| `0x6C` | PWR_MGMT_2 | R/W | 传感器使能控制 |
| `0x19` | SMPLRT_DIV | R/W | 采样率分频器 |
| `0x1A` | CONFIG | R/W | 数字低通滤波器配置 |
| `0x1B` | GYRO_CONFIG | R/W | 陀螺仪量程配置 |
| `0x1C` | ACCEL_CONFIG | R/W | 加速度计量程配置 |
| `0x3B-0x48` | 传感器数据 | R | 14 字节连续数据（Accel+Temp+Gyro） |

### 数据转换公式

```c
// 加速度转换 (LSB → g)
accel_g = raw_accel / sensitivity;

// 陀螺仪转换 (LSB → °/s)
gyro_dps = raw_gyro / sensitivity;

// 温度转换 (LSB → °C)
temp_c = (raw_temp / 340.0f) + 36.53f;

// 灵敏度参考表
加速度: ±2g=16384, ±4g=8192, ±8g=4096, ±16g=2048 (LSB/g)
陀螺仪: ±250°/s=131, ±500°/s=65.5, ±1000°/s=32.8, ±2000°/s=16.4 (LSB/(°/s))
```

---

## 测试与质量

### 已通过测试

- [x] **IMU 初始化流程**
  - WHO_AM_I 验证 (0x68)
  - 硬件复位成功
  - 寄存器配置写入成功

- [x] **软 I2C 通信**
  - 单字节写入 (SoftI2C_WriteRegister)
  - 多字节读取 (SoftI2C_ReadRegisters)
  - 连续读取 14 字节数据稳定

- [x] **数据转换精度**
  - 加速度数据范围合理 (~±1g 静止状态)
  - 陀螺仪数据零漂小 (~±5°/s)
  - 温度数据准确 (室温 20-30°C)

- [x] **UART 调试输出**
  - 115200 波特率稳定输出
  - 10Hz 数据流无丢失

### 待测试功能

- [ ] **TIM5 PWM 舵机驱动** (已配置，未编写应用代码)
- [ ] **UART2 CRSF 协议** (硬件已配置，波特率需改为 420000)
- [ ] **UART3 DMA 高速传输** (硬件已配置，波特率需改为 921600)
- [ ] **1kHz IMU 采样** (当前 10Hz)
- [ ] **姿态融合算法**
- [ ] **PID 控制循环**

### 已知问题

1. **USART2/3 波特率配置错误**
   - 当前: 115200 (CubeMX 默认)
   - 需要: UART2=420000, UART3=921600
   - 修复方法: 在 CubeMX 中修改 `IMU.ioc` 文件

2. **IMU 采样率偏低**
   - 当前: 10Hz (主循环 `HAL_Delay(100)`)
   - 目标: 1kHz (需使用定时器中断)

3. **舵机驱动未实现**
   - TIM5 PWM 已配置，但缺少应用层 API

---

## 常见问题 (FAQ)

### Q1: 如何修改 I2C 引脚？

**A1**: 修改 `imu_port.h`:
```c
#define IMU_PORT_SCL_GPIO     GPIOA      // 修改为目标 GPIO 组
#define IMU_PORT_SCL_PIN      GPIO_PIN_12 // 修改为目标引脚
#define IMU_PORT_SDA_GPIO     GPIOA
#define IMU_PORT_SDA_PIN      GPIO_PIN_11
```

### Q2: 如何提高 IMU 采样率？

**A2**: 使用定时器中断替代 `HAL_Delay()`:
```c
// 在 TIM2 中断中采样（配置为 1kHz）
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM2) {
        IMU_ReadData(&g_imuData);  // 全局变量存储
        g_dataReady = 1;           // 设置标志
    }
}
```

### Q3: 软 I2C 速度太慢怎么办？

**A3**: 调整 `soft_i2c.c` 中的延迟函数（需用示波器测量）:
```c
// 减小延迟可提高速度，但需确保不违反 MPU6050 时序要求
static void i2cDelay(void) {
    for (volatile int i = 0; i < 10; i++);  // 调整此值
}
```

### Q4: 如何切换到硬件 I2C？

**A4**: 修改 `imu_port.h` 中的底层接口实现，调用 HAL I2C API:
```c
static inline IMU_Status_t IMU_PORT_I2C_ReadRegs(...) {
    return HAL_I2C_Mem_Read(&hi2c1, dev_addr << 1, reg_addr,
                            I2C_MEMADD_SIZE_8BIT, data, length, 1000);
}
```

---

## 相关文件清单

### 应用代码 (`Core/Src/`)
- `main.c` - 主程序入口与主循环
- `imu.c` - IMU 驱动实现
- `soft_i2c.c` - 软 I2C 驱动实现
- `gpio.c` - GPIO 初始化（CubeMX 生成）
- `usart.c` - UART 初始化（CubeMX 生成）
- `tim.c` - TIM5 初始化（CubeMX 生成）
- `dma.c` - DMA 初始化（CubeMX 生成）
- `stm32f1xx_it.c` - 中断服务函数
- `stm32f1xx_hal_msp.c` - HAL MSP 初始化
- `system_stm32f1xx.c` - 系统初始化
- `syscalls.c` - printf 重定向支持
- `sysmem.c` - 内存管理

### 头文件 (`Core/Inc/`)
- `main.h` - 主程序头文件
- `imu.h` - IMU 驱动 API
- `imu_port.h` - 平台移植层
- `soft_i2c.h` - 软 I2C API
- `6050.h` - 旧版 MPU6050 驱动（已弃用，保留作参考）
- `gpio.h` - GPIO 配置
- `usart.h` - UART 配置
- `tim.h` - TIM 配置
- `dma.h` - DMA 配置
- `stm32f1xx_it.h` - 中断声明
- `stm32f1xx_hal_conf.h` - HAL 库配置

### 旧版文件（已废弃）
- `6050.c/h` - 旧版 MPU6050 驱动（使用标准库，已被 `imu.c/h` 替代）

---

## 下一步改进

1. **实现舵机驱动模块** (`servo.c/h`)
   ```c
   HAL_StatusTypeDef Servo_Init(void);
   HAL_StatusTypeDef Servo_SetAngle(ServoID_t id, float angle);
   float Servo_GetAngle(ServoID_t id);
   ```

2. **优化 IMU 采样**
   - 使用 TIM2 1kHz 中断采样
   - 实现循环缓冲区避免丢失数据

3. **添加 CRSF 协议解析**
   ```c
   HAL_StatusTypeDef CRSF_Init(void);
   HAL_StatusTypeDef CRSF_ProcessFrame(void);
   uint16_t CRSF_GetChannel(uint8_t channel);
   ```

4. **实现 Jetson 通信**
   - DMA 环形缓冲区
   - CRC8 校验
   - 帧解析与封装

5. **代码优化**
   - 减少 `printf` 调用（影响实时性）
   - 使用 DMA 发送替代阻塞发送
   - 添加错误统计与日志

---

**最后更新**: 2025-10-21 18:37:25 CST
**维护状态**: 活跃开发中
