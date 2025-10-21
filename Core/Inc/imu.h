/**
 ******************************************************************************
 * @file    imu.h
 * @brief   MPU6050惯性测量单元(IMU)驱动 - 用户API接口
 * @version 2.0 (模块化封装版本)
 * @date    2025
 ******************************************************************************
 * @attention
 *
 * 本驱动封装了MPU6050六轴传感器的完整功能
 * - 支持多平台移植(STM32/Arduino/ESP32等)
 * - 提供原始数据和物理单位数据
 * - 工业级初始化流程(WHO_AM_I验证+硬件复位)
 * - 完善的错误处理机制
 *
 * 快速开始:
 * @code
 *   // 1. 初始化
 *   if (IMU_Init() == IMU_OK) {
 *     // 2. 读取数据
 *     IMU_Data_t data;
 *     IMU_ReadData(&data);
 *     // 3. 使用数据
 *     printf("Accel: %.2f g\n", data.accel_x);
 *   }
 * @endcode
 *
 ******************************************************************************
 */

#ifndef IMU_H
#define IMU_H

#ifdef __cplusplus
extern "C" {
#endif

#include "imu_port.h"  /* 平台移植层 */

/* ==================== 版本信息 ==================== */

#define IMU_VERSION_MAJOR   2
#define IMU_VERSION_MINOR   0
#define IMU_VERSION_PATCH   0

/* ==================== 数据结构定义 ==================== */

/**
 * @brief IMU原始数据结构体
 * @note  所有数据均为16位有符号整数LSB值,需转换为物理单位
 */
typedef struct
{
  int16_t accel_x;      /**< X轴加速度原始值 (LSB) */
  int16_t accel_y;      /**< Y轴加速度原始值 (LSB) */
  int16_t accel_z;      /**< Z轴加速度原始值 (LSB) */
  int16_t gyro_x;       /**< X轴角速度原始值 (LSB) */
  int16_t gyro_y;       /**< Y轴角速度原始值 (LSB) */
  int16_t gyro_z;       /**< Z轴角速度原始值 (LSB) */
  int16_t temperature;  /**< 温度原始值 (LSB) */
} IMU_RawData_t;

/**
 * @brief IMU物理单位数据结构体
 * @note  已转换为实际物理单位,可直接使用
 */
typedef struct
{
  float accel_x;        /**< X轴加速度 (g, 重力加速度) */
  float accel_y;        /**< Y轴加速度 (g) */
  float accel_z;        /**< Z轴加速度 (g) */
  float gyro_x;         /**< X轴角速度 (°/s) */
  float gyro_y;         /**< Y轴角速度 (°/s) */
  float gyro_z;         /**< Z轴角速度 (°/s) */
  float temperature;    /**< 温度 (°C) */
} IMU_Data_t;

/**
 * @brief 加速度计量程选择
 */
typedef enum
{
  IMU_ACCEL_RANGE_2G  = 0,  /**< ±2g  (灵敏度: 16384 LSB/g) */
  IMU_ACCEL_RANGE_4G  = 1,  /**< ±4g  (灵敏度: 8192 LSB/g) */
  IMU_ACCEL_RANGE_8G  = 2,  /**< ±8g  (灵敏度: 4096 LSB/g) */
  IMU_ACCEL_RANGE_16G = 3   /**< ±16g (灵敏度: 2048 LSB/g) */
} IMU_AccelRange_t;

/**
 * @brief 陀螺仪量程选择
 */
typedef enum
{
  IMU_GYRO_RANGE_250DPS  = 0,  /**< ±250°/s  (灵敏度: 131 LSB/(°/s)) */
  IMU_GYRO_RANGE_500DPS  = 1,  /**< ±500°/s  (灵敏度: 65.5 LSB/(°/s)) */
  IMU_GYRO_RANGE_1000DPS = 2,  /**< ±1000°/s (灵敏度: 32.8 LSB/(°/s)) */
  IMU_GYRO_RANGE_2000DPS = 3   /**< ±2000°/s (灵敏度: 16.4 LSB/(°/s)) */
} IMU_GyroRange_t;

/**
 * @brief 数字低通滤波器配置
 */
typedef enum
{
  IMU_DLPF_260HZ = 0,  /**< 带宽260Hz, 延迟0ms */
  IMU_DLPF_184HZ = 1,  /**< 带宽184Hz, 延迟2.0ms */
  IMU_DLPF_94HZ  = 2,  /**< 带宽94Hz,  延迟3.0ms */
  IMU_DLPF_44HZ  = 3,  /**< 带宽44Hz,  延迟4.9ms */
  IMU_DLPF_21HZ  = 4,  /**< 带宽21Hz,  延迟8.5ms */
  IMU_DLPF_10HZ  = 5,  /**< 带宽10Hz,  延迟13.8ms */
  IMU_DLPF_5HZ   = 6   /**< 带宽5Hz,   延迟19.0ms (默认,噪声最低) */
} IMU_DLPF_t;

/* ==================== 核心API函数 ==================== */

/**
 * @brief  初始化MPU6050传感器
 * @details
 *         执行完整的初始化序列:
 *         1. 验证芯片ID (WHO_AM_I)
 *         2. 硬件复位
 *         3. 配置时钟源(PLL)
 *         4. 设置量程和滤波器
 *
 * @retval IMU_OK      初始化成功
 * @retval IMU_ERROR   初始化失败(I2C通信错误或芯片ID不匹配)
 *
 * @note   必须在读取数据前调用
 * @note   建议在初始化后延迟50ms再读取数据
 *
 * @example
 * @code
 *   if (IMU_Init() == IMU_OK) {
 *     printf("IMU初始化成功\n");
 *   } else {
 *     printf("IMU初始化失败,请检查硬件连接\n");
 *   }
 * @endcode
 */
IMU_Status_t IMU_Init(void);

/**
 * @brief  读取IMU原始数据
 * @param  raw  指向IMU_RawData_t结构体的指针
 *
 * @retval IMU_OK      读取成功
 * @retval IMU_ERROR   读取失败(参数为NULL或I2C通信失败)
 *
 * @note   需要手动转换为物理单位,推荐使用IMU_ReadData()
 *
 * @example
 * @code
 *   IMU_RawData_t raw;
 *   if (IMU_ReadRaw(&raw) == IMU_OK) {
 *     // 手动转换: accel_g = raw.accel_x / 16384.0f
 *   }
 * @endcode
 */
IMU_Status_t IMU_ReadRaw(IMU_RawData_t *raw);

/**
 * @brief  读取IMU物理单位数据 (推荐使用)
 * @param  data  指向IMU_Data_t结构体的指针
 *
 * @retval IMU_OK      读取成功
 * @retval IMU_ERROR   读取失败(参数为NULL或I2C通信失败)
 *
 * @note   自动转换为物理单位,直接可用
 *
 * @example
 * @code
 *   IMU_Data_t data;
 *   if (IMU_ReadData(&data) == IMU_OK) {
 *     printf("加速度: X=%.2fg, Y=%.2fg, Z=%.2fg\n",
 *            data.accel_x, data.accel_y, data.accel_z);
 *     printf("角速度: X=%.1f°/s, Y=%.1f°/s, Z=%.1f°/s\n",
 *            data.gyro_x, data.gyro_y, data.gyro_z);
 *     printf("温度: %.1f°C\n", data.temperature);
 *   }
 * @endcode
 */
IMU_Status_t IMU_ReadData(IMU_Data_t *data);

/* ==================== 配置函数 ==================== */

/**
 * @brief  设置加速度计量程
 * @param  range  量程选择 (IMU_ACCEL_RANGE_2G ~ IMU_ACCEL_RANGE_16G)
 *
 * @retval IMU_OK      设置成功
 * @retval IMU_ERROR   设置失败
 *
 * @note   量程越小,精度越高;量程越大,测量范围越广
 * @note   默认为±2g,适合静态姿态测量
 */
IMU_Status_t IMU_SetAccelRange(IMU_AccelRange_t range);

/**
 * @brief  设置陀螺仪量程
 * @param  range  量程选择 (IMU_GYRO_RANGE_250DPS ~ IMU_GYRO_RANGE_2000DPS)
 *
 * @retval IMU_OK      设置成功
 * @retval IMU_ERROR   设置失败
 *
 * @note   量程越小,精度越高;量程越大,测量范围越广
 * @note   默认为±2000°/s,适合快速旋转场景
 */
IMU_Status_t IMU_SetGyroRange(IMU_GyroRange_t range);

/**
 * @brief  设置数字低通滤波器
 * @param  dlpf  滤波器配置 (IMU_DLPF_260HZ ~ IMU_DLPF_5HZ)
 *
 * @retval IMU_OK      设置成功
 * @retval IMU_ERROR   设置失败
 *
 * @note   带宽越低,噪声越小,但延迟越大
 * @note   默认为5Hz,适合大多数应用
 */
IMU_Status_t IMU_SetDLPF(IMU_DLPF_t dlpf);

/**
 * @brief  设置采样率分频器
 * @param  divider  分频值 (0-255)
 *
 * @retval IMU_OK      设置成功
 * @retval IMU_ERROR   设置失败
 *
 * @note   采样率 = 陀螺仪输出率 / (1 + divider)
 * @note   陀螺仪输出率通常为1kHz (DLPF启用时)
 * @note   例如: divider=9 → 采样率=1000/(1+9)=100Hz
 */
IMU_Status_t IMU_SetSampleRate(uint8_t divider);

/* ==================== 工具函数 ==================== */

/**
 * @brief  验证MPU6050芯片ID
 *
 * @retval IMU_OK      芯片ID正确 (0x68)
 * @retval IMU_ERROR   芯片ID错误或通信失败
 *
 * @note   用于诊断I2C通信问题
 */
IMU_Status_t IMU_VerifyChipID(void);

/**
 * @brief  软件复位MPU6050
 *
 * @retval IMU_OK      复位成功
 * @retval IMU_ERROR   复位失败
 *
 * @note   复位后需要重新初始化
 */
IMU_Status_t IMU_Reset(void);

/**
 * @brief  获取驱动版本号
 *
 * @retval 32位版本号 (格式: 0xMMmmpppp, M=主版本,m=次版本,p=补丁版本)
 *
 * @example
 * @code
 *   uint32_t version = IMU_GetVersion();
 *   printf("IMU驱动版本: %d.%d.%d\n",
 *          (version >> 24) & 0xFF,
 *          (version >> 16) & 0xFF,
 *          version & 0xFFFF);
 * @endcode
 */
uint32_t IMU_GetVersion(void);

/* ==================== 内部寄存器定义 (高级用户) ==================== */

#define MPU6050_REG_SMPLRT_DIV      0x19U
#define MPU6050_REG_CONFIG          0x1AU
#define MPU6050_REG_GYRO_CONFIG     0x1BU
#define MPU6050_REG_ACCEL_CONFIG    0x1CU
#define MPU6050_REG_ACCEL_XOUT_H    0x3BU
#define MPU6050_REG_PWR_MGMT_1      0x6BU
#define MPU6050_REG_PWR_MGMT_2      0x6CU
#define MPU6050_REG_WHO_AM_I        0x75U

#ifdef __cplusplus
}
#endif

#endif /* IMU_H */
