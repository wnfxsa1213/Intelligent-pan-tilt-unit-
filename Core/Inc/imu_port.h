/**
 ******************************************************************************
 * @file    imu_port.h
 * @brief   MPU6050 IMU驱动移植层配置文件
 * @version 2.0
 * @date    2025
 ******************************************************************************
 * @attention
 *
 * 本文件定义了IMU驱动的底层接口,实现平台无关性
 * 移植到新平台时,只需修改本文件即可
 *
 * 支持的平台:
 * - STM32 HAL库
 * - STM32 标准库
 * - Arduino
 * - ESP32
 * - 其他支持I2C的MCU
 *
 ******************************************************************************
 */

#ifndef IMU_PORT_H
#define IMU_PORT_H

#ifdef __cplusplus
extern "C" {
#endif

/* ==================== 平台选择 ==================== */

/**
 * @brief 目标平台选择
 * @note  取消注释对应的平台定义,只能选择一个
 */
#define IMU_PLATFORM_STM32_HAL    1   /* STM32 HAL库平台 */
// #define IMU_PLATFORM_STM32_STD    1   /* STM32 标准库平台 */
// #define IMU_PLATFORM_ARDUINO      1   /* Arduino平台 */
// #define IMU_PLATFORM_ESP32        1   /* ESP32平台 */

/* ==================== 平台相关头文件 ==================== */

#if defined(IMU_PLATFORM_STM32_HAL)
  #include "stm32f1xx_hal.h"
  #include "soft_i2c.h"

  /* HAL库类型定义 */
  typedef HAL_StatusTypeDef IMU_Status_t;
  #define IMU_OK      HAL_OK
  #define IMU_ERROR   HAL_ERROR

#elif defined(IMU_PLATFORM_STM32_STD)
  #include "stm32f10x.h"

  /* 标准库类型定义 */
  typedef enum {
    IMU_OK = 0,
    IMU_ERROR = 1
  } IMU_Status_t;

#elif defined(IMU_PLATFORM_ARDUINO)
  #include <Arduino.h>
  #include <Wire.h>

  /* Arduino类型定义 */
  typedef enum {
    IMU_OK = 0,
    IMU_ERROR = 1
  } IMU_Status_t;

#elif defined(IMU_PLATFORM_ESP32)
  #include <driver/i2c.h>

  /* ESP32类型定义 */
  typedef esp_err_t IMU_Status_t;
  #define IMU_OK      ESP_OK
  #define IMU_ERROR   ESP_FAIL

#else
  #error "Please define a platform: IMU_PLATFORM_STM32_HAL, IMU_PLATFORM_STM32_STD, IMU_PLATFORM_ARDUINO, or IMU_PLATFORM_ESP32"
#endif

#include <stdint.h>

/* ==================== 硬件配置 ==================== */

/**
 * @brief MPU6050 I2C设备地址
 * @note  AD0引脚接地=0x68, 接VCC=0x69
 */
#define IMU_PORT_I2C_ADDRESS    0x68U

/**
 * @brief I2C引脚定义 (仅STM32平台需要)
 */
#if defined(IMU_PLATFORM_STM32_HAL) || defined(IMU_PLATFORM_STM32_STD)
  #define IMU_PORT_SCL_GPIO     GPIOA
  #define IMU_PORT_SCL_PIN      GPIO_PIN_12
  #define IMU_PORT_SDA_GPIO     GPIOA
  #define IMU_PORT_SDA_PIN      GPIO_PIN_11
#endif

/**
 * @brief Arduino引脚定义
 */
#if defined(IMU_PLATFORM_ARDUINO)
  #define IMU_PORT_SDA_PIN      A4    /* Arduino Uno/Nano默认SDA */
  #define IMU_PORT_SCL_PIN      A5    /* Arduino Uno/Nano默认SCL */
#endif

/**
 * @brief ESP32引脚定义
 */
#if defined(IMU_PLATFORM_ESP32)
  #define IMU_PORT_SDA_PIN      21    /* ESP32默认SDA */
  #define IMU_PORT_SCL_PIN      22    /* ESP32默认SCL */
  #define IMU_PORT_I2C_FREQ     100000  /* I2C时钟频率100kHz */
#endif

/* ==================== 底层接口函数 ==================== */

/**
 * @brief  初始化I2C总线
 * @note   在使用IMU前必须先调用此函数
 * @retval IMU_OK: 初始化成功
 */
static inline IMU_Status_t IMU_PORT_I2C_Init(void);

/**
 * @brief  向I2C设备写入单个寄存器
 * @param  dev_addr  设备7位I2C地址
 * @param  reg_addr  寄存器地址
 * @param  data      要写入的数据
 * @retval IMU_OK: 写入成功, IMU_ERROR: 写入失败
 */
static inline IMU_Status_t IMU_PORT_I2C_WriteReg(uint8_t dev_addr, uint8_t reg_addr, uint8_t data);

/**
 * @brief  从I2C设备读取多个寄存器
 * @param  dev_addr  设备7位I2C地址
 * @param  reg_addr  起始寄存器地址
 * @param  data      接收缓冲区指针
 * @param  length    读取字节数
 * @retval IMU_OK: 读取成功, IMU_ERROR: 读取失败
 */
static inline IMU_Status_t IMU_PORT_I2C_ReadRegs(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t length);

/**
 * @brief  毫秒级延迟函数
 * @param  ms  延迟时间(毫秒)
 * @retval 无
 */
static inline void IMU_PORT_Delay_ms(uint32_t ms);

/* ==================== 平台适配实现 ==================== */

#if defined(IMU_PLATFORM_STM32_HAL)

/**
 * @brief STM32 HAL库平台适配实现
 */
static inline IMU_Status_t IMU_PORT_I2C_Init(void)
{
  SoftI2C_Init();
  return IMU_OK;
}

static inline IMU_Status_t IMU_PORT_I2C_WriteReg(uint8_t dev_addr, uint8_t reg_addr, uint8_t data)
{
  return SoftI2C_WriteRegister(dev_addr, reg_addr, data);
}

static inline IMU_Status_t IMU_PORT_I2C_ReadRegs(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t length)
{
  return SoftI2C_ReadRegisters(dev_addr, reg_addr, data, length);
}

static inline void IMU_PORT_Delay_ms(uint32_t ms)
{
  HAL_Delay(ms);
}

#elif defined(IMU_PLATFORM_ARDUINO)

/**
 * @brief Arduino平台适配实现
 */
static inline IMU_Status_t IMU_PORT_I2C_Init(void)
{
  Wire.begin();
  Wire.setClock(100000);  /* 100kHz */
  return IMU_OK;
}

static inline IMU_Status_t IMU_PORT_I2C_WriteReg(uint8_t dev_addr, uint8_t reg_addr, uint8_t data)
{
  Wire.beginTransmission(dev_addr);
  Wire.write(reg_addr);
  Wire.write(data);
  return (Wire.endTransmission() == 0) ? IMU_OK : IMU_ERROR;
}

static inline IMU_Status_t IMU_PORT_I2C_ReadRegs(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t length)
{
  Wire.beginTransmission(dev_addr);
  Wire.write(reg_addr);
  if (Wire.endTransmission(false) != 0) return IMU_ERROR;

  Wire.requestFrom(dev_addr, length);
  for (uint16_t i = 0; i < length; i++) {
    if (Wire.available()) {
      data[i] = Wire.read();
    } else {
      return IMU_ERROR;
    }
  }
  return IMU_OK;
}

static inline void IMU_PORT_Delay_ms(uint32_t ms)
{
  delay(ms);
}

#endif

#ifdef __cplusplus
}
#endif

#endif /* IMU_PORT_H */
