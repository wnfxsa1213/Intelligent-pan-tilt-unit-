# Drivers - STM32 HAL 库与 CMSIS

**[根目录](../CLAUDE.md) > Drivers**

**模块职责**: STM32F1 硬件抽象层 (HAL) 与 CMSIS 核心支持
**来源**: STM32CubeMX 自动生成（基于 STM32Cube FW_F1 V1.8.6）
**状态**: 稳定引用（不修改）

---

## 模块说明

本模块包含 STMicroelectronics 提供的**标准外设库**，为应用层提供硬件抽象。所有驱动代码均由 STM32CubeMX 生成，**不建议手动修改**。

---

## 模块结构

```
Drivers/
├── STM32F1xx_HAL_Driver/        # HAL 硬件抽象层
│   ├── Inc/                     # HAL 头文件
│   │   ├── stm32f1xx_hal.h      # HAL 核心
│   │   ├── stm32f1xx_hal_gpio.h # GPIO 外设
│   │   ├── stm32f1xx_hal_uart.h # UART 外设
│   │   ├── stm32f1xx_hal_tim.h  # 定时器外设
│   │   ├── stm32f1xx_hal_dma.h  # DMA 外设
│   │   └── Legacy/              # 兼容旧版 API
│   ├── Src/                     # HAL 实现源码
│   │   ├── stm32f1xx_hal.c
│   │   ├── stm32f1xx_hal_gpio.c
│   │   ├── stm32f1xx_hal_uart.c
│   │   ├── stm32f1xx_hal_tim.c
│   │   └── ...
│   └── LICENSE.txt              # BSD-3-Clause 许可证
│
└── CMSIS/                        # ARM CMSIS 标准
    ├── Device/ST/STM32F1xx/      # STM32F1 设备支持
    │   ├── Include/
    │   │   ├── stm32f1xx.h       # 设备头文件
    │   │   ├── stm32f103xe.h     # STM32F103xE 系列定义
    │   │   └── system_stm32f1xx.h
    │   └── LICENSE.txt           # Apache-2.0 许可证
    │
    └── Include/                  # CMSIS 核心头文件
        ├── core_cm3.h            # Cortex-M3 核心定义
        ├── cmsis_gcc.h           # GCC 编译器支持
        ├── cmsis_compiler.h      # 编译器抽象
        └── ...
```

---

## 关键功能

### STM32F1xx HAL 库

HAL 库提供**统一的 API** 接口，简化外设操作：

| 外设模块 | 头文件 | 核心功能 |
|---------|--------|---------|
| **GPIO** | `stm32f1xx_hal_gpio.h` | 引脚配置、读写、中断 |
| **UART** | `stm32f1xx_hal_uart.h` | 串口收发、DMA、中断 |
| **TIM** | `stm32f1xx_hal_tim.h` | 定时器、PWM、输入捕获 |
| **DMA** | `stm32f1xx_hal_dma.h` | 直接内存访问 |
| **RCC** | `stm32f1xx_hal_rcc.h` | 时钟配置、复位控制 |
| **CORTEX** | `stm32f1xx_hal_cortex.h` | 中断优先级、SysTick |
| **FLASH** | `stm32f1xx_hal_flash.h` | Flash 编程、擦除 |
| **PWR** | `stm32f1xx_hal_pwr.h` | 电源管理 |
| **EXTI** | `stm32f1xx_hal_exti.h` | 外部中断 |

### CMSIS 核心

CMSIS (Cortex Microcontroller Software Interface Standard) 提供：

| 组件 | 功能 |
|-----|------|
| **核心外设访问** | NVIC、SysTick、SCB 等寄存器定义 |
| **编译器抽象** | 支持 GCC、IAR、Keil 等编译器 |
| **内联函数** | `__disable_irq()`, `__enable_irq()` 等 |
| **数据类型** | `uint8_t`, `int16_t`, `__IO` 等标准类型 |

---

## 使用方式

### HAL 库配置 (`stm32f1xx_hal_conf.h`)

此文件在 `Core/Inc/` 中，定义启用的 HAL 模块：

```c
#define HAL_MODULE_ENABLED
#define HAL_GPIO_MODULE_ENABLED
#define HAL_UART_MODULE_ENABLED
#define HAL_TIM_MODULE_ENABLED
#define HAL_DMA_MODULE_ENABLED
// ... 其他模块
```

### 典型 API 调用流程

```c
// 1. 包含头文件
#include "stm32f1xx_hal.h"

// 2. 定义句柄（由 CubeMX 生成）
UART_HandleTypeDef huart1;

// 3. 初始化外设
HAL_UART_Init(&huart1);

// 4. 调用功能函数
uint8_t data[] = "Hello";
HAL_UART_Transmit(&huart1, data, 5, HAL_MAX_DELAY);

// 5. 检查返回值
HAL_StatusTypeDef status = HAL_UART_Receive(&huart1, buffer, 10, 1000);
if (status != HAL_OK) {
    // 处理错误
}
```

---

## 重要说明

### 不建议手动修改

- **原因**: 下次使用 CubeMX 重新生成代码时会**覆盖**手动修改
- **替代方案**:
  - 在应用层封装 HAL API（如 `Core/Src/` 中的模块）
  - 使用 CubeMX 的 "USER CODE BEGIN/END" 注释区域添加代码

### 许可证信息

| 组件 | 许可证 | 商用限制 |
|-----|--------|---------|
| **HAL 库** | BSD-3-Clause | ✅ 无限制 |
| **CMSIS Device** | Apache-2.0 | ✅ 无限制 |
| **CMSIS Core** | Apache-2.0 | ✅ 无限制 |

### 版本信息

- **HAL 版本**: V1.1.9 (STM32Cube FW_F1 V1.8.6)
- **CMSIS 版本**: V5.6.0
- **工具链兼容**: GCC, Keil MDK-ARM, IAR EWARM

---

## 常见 HAL API 速查

### GPIO 操作

```c
// 读引脚
GPIO_PinState state = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);

// 写引脚
HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);

// 翻转引脚
HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_2);
```

### UART 操作

```c
// 阻塞发送
HAL_UART_Transmit(&huart1, data, length, timeout);

// 阻塞接收
HAL_UART_Receive(&huart1, buffer, length, timeout);

// DMA 发送
HAL_UART_Transmit_DMA(&huart1, data, length);

// DMA 接收
HAL_UART_Receive_DMA(&huart1, buffer, length);
```

### TIM PWM 操作

```c
// 启动 PWM
HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);

// 设置占空比
__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, ccr_value);

// 停止 PWM
HAL_TIM_PWM_Stop(&htim5, TIM_CHANNEL_1);
```

### DMA 操作

```c
// DMA 传输
HAL_DMA_Start(&hdma, src_addr, dst_addr, length);

// 等待传输完成
HAL_DMA_PollForTransfer(&hdma, HAL_DMA_FULL_TRANSFER, timeout);
```

---

## 参考文档

- [STM32F1xx HAL 用户手册 (UM1850)](https://www.st.com/resource/en/user_manual/um1850.pdf)
- [STM32F103 数据手册](https://www.st.com/resource/en/datasheet/stm32f103rc.pdf)
- [STM32F103 参考手册 (RM0008)](https://www.st.com/resource/en/reference_manual/rm0008.pdf)
- [CMSIS 官方文档](https://arm-software.github.io/CMSIS_5/Core/html/index.html)

---

**最后更新**: 2025-10-21 18:37:25 CST
**维护方式**: 仅通过 STM32CubeMX 更新
