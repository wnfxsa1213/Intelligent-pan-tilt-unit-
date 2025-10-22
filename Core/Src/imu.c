/**
 ******************************************************************************
 * @file    imu.c
 * @brief   MPU6050 IMU驱动实现 (模块化封装版本)
 * @version 2.0
 * @date    2025
 ******************************************************************************
 */

#include "imu.h"

/* ==================== 灵敏度查表 ==================== */

static const float ACCEL_SENSITIVITY_LUT[] = {
  16384.0f, 8192.0f, 4096.0f, 2048.0f
};

static const float GYRO_SENSITIVITY_LUT[] = {
  131.0f, 65.5f, 32.8f, 16.4f
};

/* ==================== 私有变量 ==================== */

static IMU_AccelRange_t current_accel_range = IMU_ACCEL_RANGE_2G;
static IMU_GyroRange_t current_gyro_range = IMU_GYRO_RANGE_2000DPS;
static float            g_accel_sensitivity = 16384.0f;
static float            g_gyro_sensitivity  = 16.4f;

/* ==================== 私有函数声明 ==================== */

static float IMU_ConvertAccel(int16_t raw_value);
static float IMU_ConvertGyro(int16_t raw_value);
static float IMU_ConvertTemperature(int16_t raw_value);
static void  IMU_ClearData(IMU_Data_t *data);

/* ==================== 核心API实现 ==================== */

IMU_Status_t IMU_Init(void)
{
  IMU_Status_t status;
  uint8_t who_am_i = 0;

  /* 初始化I2C总线 */
  status = IMU_PORT_I2C_Init();
  if (status != IMU_OK) {
    return status;//错误
  }

  /* 延迟让MPU6050上电稳定 */
  IMU_PORT_Delay_ms(50);

  /* 步骤0: 验证芯片ID */
  status = IMU_PORT_I2C_ReadRegs(IMU_PORT_I2C_ADDRESS, MPU6050_REG_WHO_AM_I, &who_am_i, 1);
  if (status != IMU_OK || who_am_i != 0x68U) {
    return IMU_ERROR;  /* 芯片ID验证失败 */
  }

  /* 步骤1: 硬件复位 */
  status = IMU_PORT_I2C_WriteReg(IMU_PORT_I2C_ADDRESS, MPU6050_REG_PWR_MGMT_1, 0x80U);
  if (status != IMU_OK) {
    return status;
  }
  IMU_PORT_Delay_ms(100);  /* 等待复位完成 */

  /* 步骤2: 配置时钟源(PLL with X-axis gyroscope) */
  status = IMU_PORT_I2C_WriteReg(IMU_PORT_I2C_ADDRESS, MPU6050_REG_PWR_MGMT_1, 0x01U);
  if (status != IMU_OK) {
    return status;
  }

  /* 步骤3: 启用所有传感器轴 */
  status = IMU_PORT_I2C_WriteReg(IMU_PORT_I2C_ADDRESS, MPU6050_REG_PWR_MGMT_2, 0x00U);
  if (status != IMU_OK) {
    return status;
  }

  /* 步骤4: 配置采样率 (1000Hz / (1+7) = 125Hz) */
  status = IMU_PORT_I2C_WriteReg(IMU_PORT_I2C_ADDRESS, MPU6050_REG_SMPLRT_DIV, 0x07U);
  if (status != IMU_OK) {
    return status;
  }

  /* 步骤5: 配置数字低通滤波器 (5Hz) */
  status = IMU_PORT_I2C_WriteReg(IMU_PORT_I2C_ADDRESS, MPU6050_REG_CONFIG, 0x06U);
  if (status != IMU_OK) {
    return status;
  }

  /* 步骤6: 配置陀螺仪量程 (±2000°/s) */
  status = IMU_SetGyroRange(IMU_GYRO_RANGE_2000DPS);
  if (status != IMU_OK) {
    return status;
  }

  /* 步骤7: 配置加速度计量程 (±2g) */
  status = IMU_SetAccelRange(IMU_ACCEL_RANGE_2G);
  if (status != IMU_OK) {
    return status;
  }

  return IMU_OK;
}

IMU_Status_t IMU_ReadRaw(IMU_RawData_t *raw)
{
  if (raw == NULL) {
    return IMU_ERROR;
  }

  uint8_t buffer[14] = {0};

  /* 从0x3B寄存器开始连续读取14字节 */
  IMU_Status_t status = IMU_PORT_I2C_ReadRegs(IMU_PORT_I2C_ADDRESS, MPU6050_REG_ACCEL_XOUT_H, buffer, 14);
  if (status != IMU_OK) {
    return status;
  }

  /* 组合数据 (大端格式: 高字节在前) */
  raw->accel_x = (int16_t)((buffer[0] << 8) | buffer[1]);
  raw->accel_y = (int16_t)((buffer[2] << 8) | buffer[3]);
  raw->accel_z = (int16_t)((buffer[4] << 8) | buffer[5]);
  raw->temperature = (int16_t)((buffer[6] << 8) | buffer[7]);
  raw->gyro_x = (int16_t)((buffer[8] << 8) | buffer[9]);
  raw->gyro_y = (int16_t)((buffer[10] << 8) | buffer[11]);
  raw->gyro_z = (int16_t)((buffer[12] << 8) | buffer[13]);

  return IMU_OK;
}

IMU_Status_t IMU_ReadData(IMU_Data_t *data)
{
  if (data == NULL) {
    return IMU_ERROR;
  }

  IMU_RawData_t raw;
  IMU_Status_t status = IMU_ReadRaw(&raw);
  if (status != IMU_OK) {
    return status;
  }

  if ((raw.accel_x == -1) && (raw.accel_y == -1) &&
      (raw.accel_z == -1) && (raw.gyro_x == -1) &&
      (raw.gyro_y == -1) && (raw.gyro_z == -1)) {
    IMU_ClearData(data);
    return IMU_ERROR;
  }

  const float temperature = IMU_ConvertTemperature(raw.temperature);
  if ((temperature < -50.0f) || (temperature > 100.0f)) {
    IMU_ClearData(data);
    return IMU_ERROR;
  }

  /* 转换为物理单位 */
  data->accel_x = IMU_ConvertAccel(raw.accel_x);
  data->accel_y = IMU_ConvertAccel(raw.accel_y);
  data->accel_z = IMU_ConvertAccel(raw.accel_z);
  data->gyro_x = IMU_ConvertGyro(raw.gyro_x);
  data->gyro_y = IMU_ConvertGyro(raw.gyro_y);
  data->gyro_z = IMU_ConvertGyro(raw.gyro_z);
  data->temperature = temperature;

  return IMU_OK;
}

/* ==================== 配置函数实现 ==================== */

IMU_Status_t IMU_SetAccelRange(IMU_AccelRange_t range)
{
  uint8_t config = (uint8_t)(range << 3);
  IMU_Status_t status = IMU_PORT_I2C_WriteReg(IMU_PORT_I2C_ADDRESS, MPU6050_REG_ACCEL_CONFIG, config);
  if (status == IMU_OK) {
    current_accel_range = range;
    if ((uint32_t)range < (uint32_t)(sizeof(ACCEL_SENSITIVITY_LUT) / sizeof(ACCEL_SENSITIVITY_LUT[0]))) {
      g_accel_sensitivity = ACCEL_SENSITIVITY_LUT[(uint32_t)range];
    } else {
      g_accel_sensitivity = ACCEL_SENSITIVITY_LUT[IMU_ACCEL_RANGE_2G];
    }
  }
  return status;
}

IMU_Status_t IMU_SetGyroRange(IMU_GyroRange_t range)
{
  uint8_t config = (uint8_t)(range << 3);
  IMU_Status_t status = IMU_PORT_I2C_WriteReg(IMU_PORT_I2C_ADDRESS, MPU6050_REG_GYRO_CONFIG, config);
  if (status == IMU_OK) {
    current_gyro_range = range;
    if ((uint32_t)range < (uint32_t)(sizeof(GYRO_SENSITIVITY_LUT) / sizeof(GYRO_SENSITIVITY_LUT[0]))) {
      g_gyro_sensitivity = GYRO_SENSITIVITY_LUT[(uint32_t)range];
    } else {
      g_gyro_sensitivity = GYRO_SENSITIVITY_LUT[IMU_GYRO_RANGE_2000DPS];
    }
  }
  return status;
}

IMU_Status_t IMU_SetDLPF(IMU_DLPF_t dlpf)
{
  return IMU_PORT_I2C_WriteReg(IMU_PORT_I2C_ADDRESS, MPU6050_REG_CONFIG, (uint8_t)dlpf);
}

IMU_Status_t IMU_SetSampleRate(uint8_t divider)
{
  return IMU_PORT_I2C_WriteReg(IMU_PORT_I2C_ADDRESS, MPU6050_REG_SMPLRT_DIV, divider);
}

/* ==================== 工具函数实现 ==================== */

IMU_Status_t IMU_VerifyChipID(void)
{
  uint8_t who_am_i = 0;
  IMU_Status_t status = IMU_PORT_I2C_ReadRegs(IMU_PORT_I2C_ADDRESS, MPU6050_REG_WHO_AM_I, &who_am_i, 1);
  if (status != IMU_OK || who_am_i != 0x68U) {
    return IMU_ERROR;
  }
  return IMU_OK;
}

IMU_Status_t IMU_Reset(void)
{
  IMU_Status_t status = IMU_PORT_I2C_WriteReg(IMU_PORT_I2C_ADDRESS, MPU6050_REG_PWR_MGMT_1, 0x80U);
  if (status == IMU_OK) {
    IMU_PORT_Delay_ms(100);
  }
  return status;
}

uint32_t IMU_GetVersion(void)
{
  return ((uint32_t)IMU_VERSION_MAJOR << 24) |
         ((uint32_t)IMU_VERSION_MINOR << 16) |
         (uint32_t)IMU_VERSION_PATCH;
}

/* ==================== 私有函数实现 ==================== */

static float IMU_ConvertAccel(int16_t raw_value)
{
  return (float)raw_value / g_accel_sensitivity;
}

static float IMU_ConvertGyro(int16_t raw_value)
{
  return (float)raw_value / g_gyro_sensitivity;
}

static float IMU_ConvertTemperature(int16_t raw_value)
{
  /* 温度转换公式: Temperature(°C) = raw / 340.0 + 36.53 */
  return ((float)raw_value / 340.0f) + 36.53f;
}

static void IMU_ClearData(IMU_Data_t *data)
{
  if (data == NULL) {
    return;
  }

  data->accel_x = 0.0f;
  data->accel_y = 0.0f;
  data->accel_z = 0.0f;
  data->gyro_x  = 0.0f;
  data->gyro_y  = 0.0f;
  data->gyro_z  = 0.0f;
  data->temperature = 0.0f;
}
