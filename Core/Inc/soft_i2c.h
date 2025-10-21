/**
 ******************************************************************************
 * @file    soft_i2c.h
 * @brief   软件模拟I2C通信协议驱动头文件
 * @version 1.0
 * @date    2025
 ******************************************************************************
 * @attention
 *
 * 本驱动使用GPIO位操作(bit-banging)方式模拟I2C总线协议
 *
 * I2C协议基础知识:
 * ===============
 * I2C (Inter-Integrated Circuit) 是一种同步、半双工、多主机串行通信总线
 *
 * 硬件特性:
 * - 仅需两根信号线: SCL(时钟线) 和 SDA(数据线)
 * - 开漏输出 + 上拉电阻配置 (通常4.7kΩ)
 * - 支持多设备共享总线 (通过7位地址区分)
 * - 标准模式: 100kHz, 快速模式: 400kHz, 高速模式: 3.4MHz
 *
 * 信号时序:
 * - 起始条件(START): SCL高电平时，SDA由高变低
 * - 停止条件(STOP):  SCL高电平时，SDA由低变高
 * - 数据传输: SCL低电平时可改变SDA，SCL高电平时读取SDA
 * - 应答位(ACK): 接收方拉低SDA表示确认，保持高电平表示非应答(NACK)
 *
 * 为何使用软件I2C:
 * ===============
 * 优点:
 * 1. 任意GPIO引脚可配置，不受硬件I2C外设引脚限制
 * 2. 便于调试，可通过逻辑分析仪清晰观察每个位的操作
 * 3. 时序完全可控，可适配非标准I2C设备
 * 4. 避免硬件I2C的一些已知Bug (如STM32的I2C busy状态问题)
 *
 * 缺点:
 * 1. 占用CPU时间，无法使用DMA
 * 2. 速度受软件延迟限制，通常低于硬件I2C
 * 3. 无法实现多主机仲裁和时钟拉伸(需额外处理)
 *
 * 硬件连接:
 * =========
 * STM32F1               MPU6050
 * PA11 (SCL) ----+---- SCL
 *                |
 *             [4.7kΩ上拉到VCC]
 *
 * PA12 (SDA) ----+---- SDA
 *                |
 *             [4.7kΩ上拉到VCC]
 *
 * @note 如果使用3.3V系统与5V设备通信，确认设备支持3.3V电平或使用电平转换器
 *
 ******************************************************************************
 */

#ifndef SOFT_I2C_H
#define SOFT_I2C_H

#include "stm32f1xx_hal.h"

/* ==================== 状态码定义 ==================== */

/**
 * @brief I2C操作成功状态码
 * @note  映射到HAL库标准状态，便于与其他HAL函数统一处理
 */
#define SOFT_I2C_OK        HAL_OK

/**
 * @brief I2C操作失败状态码
 * @note  可能原因:
 *        - 从设备无应答(设备地址错误、设备未上电、总线故障)
 *        - 参数错误(空指针、长度为0)
 *        - 总线冲突(多主机环境下)
 */
#define SOFT_I2C_ERROR     HAL_ERROR

/* ==================== 函数声明 ==================== */

/**
 * @brief  初始化软件I2C总线
 * @details
 *         设置SCL和SDA引脚为空闲状态(高电平)
 *         在I2C总线上，空闲状态定义为SCL和SDA均为高电平
 *
 * @retval 无
 *
 * @note   调用前需确保GPIO已通过MX_GPIO_Init()配置为:
 *         - 模式: 开漏输出 (Open-Drain)
 *         - 速度: 高速 (50MHz)
 *         - 上拉: 外部上拉电阻 (内部上拉可选但不推荐)
 * @note   必须在任何I2C操作前调用此函数
 */
void SoftI2C_Init(void);

/**
 * @brief  向I2C设备的单个寄存器写入一个字节
 * @details
 *         这是SoftI2C_WriteRegisters()的便捷封装，用于最常见的单字节写操作
 *
 * @param  dev_addr  I2C设备7位地址 (不包含读写位，例如MPU6050为0x68)
 * @param  reg_addr  目标寄存器地址 (8位)
 * @param  value     要写入的数据 (8位)
 *
 * @retval SOFT_I2C_OK     写入成功，设备发送ACK
 * @retval SOFT_I2C_ERROR  写入失败，可能原因见状态码定义说明
 *
 * @note   I2C传输格式:
 *         START | DEV_ADDR(W) | ACK | REG_ADDR | ACK | VALUE | ACK | STOP
 *
 * @example
 * @code
 *   // 唤醒MPU6050 (向0x6B寄存器写0x00)
 *   SoftI2C_WriteRegister(0x68, 0x6B, 0x00);
 * @endcode
 */
HAL_StatusTypeDef SoftI2C_WriteRegister(uint8_t dev_addr, uint8_t reg_addr, uint8_t value);

/**
 * @brief  向I2C设备的连续寄存器写入多个字节
 * @details
 *         利用I2C自动地址递增特性，向连续寄存器写入数据块
 *         适用于批量配置或写入FIFO等场景
 *
 * @param  dev_addr  I2C设备7位地址
 * @param  reg_addr  起始寄存器地址
 * @param  data      指向待写入数据缓冲区的指针
 * @param  length    要写入的字节数 (1-65535)
 *
 * @retval SOFT_I2C_OK     所有数据写入成功
 * @retval SOFT_I2C_ERROR  写入失败 (参数错误或某个字节未收到ACK)
 *
 * @note   参数验证: data不能为NULL，length不能为0
 * @note   I2C传输格式:
 *         START | DEV_ADDR(W) | ACK | REG_ADDR | ACK | DATA[0] | ACK | ... | DATA[n-1] | ACK | STOP
 * @note   如果某个字节写入失败，会立即发送STOP并返回错误
 *
 * @example
 * @code
 *   uint8_t config[3] = {0x01, 0x02, 0x03};
 *   SoftI2C_WriteRegisters(0x68, 0x1A, config, 3);  // 写入3个连续寄存器
 * @endcode
 */
HAL_StatusTypeDef SoftI2C_WriteRegisters(uint8_t dev_addr, uint8_t reg_addr, const uint8_t *data, uint16_t length);

/**
 * @brief  从I2C设备的连续寄存器读取多个字节
 * @details
 *         执行标准I2C组合读取操作 (写寄存器地址 + 重复起始 + 读数据)
 *         这是MPU6050等传感器读取数据的标准方式
 *
 * @param  dev_addr  I2C设备7位地址
 * @param  reg_addr  起始寄存器地址
 * @param  data      指向接收数据缓冲区的指针
 * @param  length    要读取的字节数 (1-65535)
 *
 * @retval SOFT_I2C_OK     所有数据读取成功
 * @retval SOFT_I2C_ERROR  读取失败 (参数错误或设备无响应)
 *
 * @note   参数验证: data不能为NULL，length不能为0
 * @note   I2C传输格式:
 *         START | DEV_ADDR(W) | ACK | REG_ADDR | ACK |
 *         REPEATED_START | DEV_ADDR(R) | ACK | DATA[0] | ACK | ... | DATA[n-1] | NACK | STOP
 * @note   主机在读取最后一个字节后发送NACK，通知从机停止发送
 *
 * @example
 * @code
 *   uint8_t sensor_data[6];
 *   // 从MPU6050的0x3B寄存器开始读取6字节加速度数据
 *   if (SoftI2C_ReadRegisters(0x68, 0x3B, sensor_data, 6) == SOFT_I2C_OK) {
 *     int16_t ax = (sensor_data[0] << 8) | sensor_data[1];
 *     // ... 处理数据
 *   }
 * @endcode
 */
HAL_StatusTypeDef SoftI2C_ReadRegisters(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t length);

#endif /* SOFT_I2C_H */
