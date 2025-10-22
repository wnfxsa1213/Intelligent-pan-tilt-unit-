/**
 ******************************************************************************
 * @file    soft_i2c.c
 * @brief   软件模拟I2C通信协议驱动实现 (优化版本)
 * @version 1.1
 * @date    2025
 ******************************************************************************
 * @attention
 *
 * 本版本修复了原始实现的关键问题:
 * 1. 优化SDA引脚方向切换 - 使用更高效的开漏输出模式控制
 * 2. 增加SCL时钟拉伸超时错误处理
 * 3. 调整时序延迟以提高兼容性
 *
 * I2C时序说明:
 * ============
 * 1. 起始条件 (START): SCL高时SDA下降沿
 * 2. 停止条件 (STOP):  SCL高时SDA上升沿
 * 3. 数据位传输: SCL低时改变SDA,SCL高时采样SDA
 * 4. 应答位 (ACK/NACK): 接收方拉低SDA=ACK,保持高=NACK
 *
 ******************************************************************************
 */

#include "soft_i2c.h"

/* ==================== 硬件引脚配置 ==================== */

#define SOFT_I2C_SCL_PORT GPIOA
#define SOFT_I2C_SCL_PIN  GPIO_PIN_12  /* PA12 = SCL (时钟线) */
#define SOFT_I2C_SDA_PORT GPIOA
#define SOFT_I2C_SDA_PIN  GPIO_PIN_11  /* PA11 = SDA (数据线) */

/* ==================== 私有函数声明 ==================== */

static void SoftI2C_Delay(void);
static HAL_StatusTypeDef SoftI2C_SCL_High(void);
static void SoftI2C_SCL_Low(void);
static void SoftI2C_SDA_High(void);
static void SoftI2C_SDA_Low(void);
static uint8_t SoftI2C_SDA_Read(void);
static void SoftI2C_StartCondition(void);
static void SoftI2C_StopCondition(void);
static HAL_StatusTypeDef SoftI2C_WriteByte(uint8_t data);
static uint8_t SoftI2C_ReadByte(uint8_t ack);

/* ==================== 公共函数实现 ==================== */

/**
 * @brief  初始化软件I2C总线
 * @details
 *         配置GPIO为开漏输出模式,设置总线空闲状态
 *         使用开漏模式可直接通过输出高电平释放总线,无需切换输入输出
 *
 * @retval 无
 */
void SoftI2C_Init(void)
{
  /* 配置SCL为开漏输出 */
  GPIO_InitTypeDef init = {0};
  init.Pin = SOFT_I2C_SCL_PIN;
  init.Mode = GPIO_MODE_OUTPUT_OD;
  init.Pull = GPIO_PULLUP;
  init.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(SOFT_I2C_SCL_PORT, &init);

  /* 配置SDA为开漏输出 */
  init.Pin = SOFT_I2C_SDA_PIN;
  HAL_GPIO_Init(SOFT_I2C_SDA_PORT, &init);

  /* 设置总线空闲状态 */
  SoftI2C_SCL_High();
  SoftI2C_SDA_High();
  HAL_Delay(1U);
}

/**
 * @brief  向I2C设备的单个寄存器写入一个字节
 */
HAL_StatusTypeDef SoftI2C_WriteRegister(uint8_t dev_addr, uint8_t reg_addr, uint8_t value)
{
  return SoftI2C_WriteRegisters(dev_addr, reg_addr, &value, 1U);
}

/**
 * @brief  向I2C设备的连续寄存器写入多个字节
 */
HAL_StatusTypeDef SoftI2C_WriteRegisters(uint8_t dev_addr, uint8_t reg_addr, const uint8_t *data, uint16_t length)
{
  if (data == NULL || length == 0U)
  {
    return HAL_ERROR;
  }

  SoftI2C_StartCondition();

  /* 发送设备地址+写位 */
  if (SoftI2C_WriteByte((uint8_t)(dev_addr << 1U)) != HAL_OK)
  {
    SoftI2C_StopCondition();
    return HAL_ERROR;
  }

  /* 发送寄存器地址 */
  if (SoftI2C_WriteByte(reg_addr) != HAL_OK)
  {
    SoftI2C_StopCondition();
    return HAL_ERROR;
  }

  /* 依次发送所有数据字节 */
  for (uint16_t i = 0U; i < length; ++i)
  {
    if (SoftI2C_WriteByte(data[i]) != HAL_OK)
    {
      SoftI2C_StopCondition();
      return HAL_ERROR;
    }
  }

  SoftI2C_StopCondition();
  return HAL_OK;
}

/**
 * @brief  从I2C设备的连续寄存器读取多个字节
 */
HAL_StatusTypeDef SoftI2C_ReadRegisters(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t length)
{
  if (data == NULL || length == 0U)
  {
    return HAL_ERROR;
  }

  /* 步骤1: 发送起始条件 + 设备地址(写) + 寄存器地址 */
  SoftI2C_StartCondition();
  if (SoftI2C_WriteByte((uint8_t)(dev_addr << 1U)) != HAL_OK)
  {
    SoftI2C_StopCondition();
    return HAL_ERROR;
  }

  if (SoftI2C_WriteByte(reg_addr) != HAL_OK)
  {
    SoftI2C_StopCondition();
    return HAL_ERROR;
  }

  /* 步骤2: 重复起始 + 设备地址(读) */
  SoftI2C_StartCondition();
  if (SoftI2C_WriteByte((uint8_t)((dev_addr << 1U) | 0x01U)) != HAL_OK)
  {
    SoftI2C_StopCondition();
    return HAL_ERROR;
  }

  /* 步骤3: 依次读取所有数据字节 */
  for (uint16_t i = 0U; i < length; ++i)
  {
    const uint8_t ack = (i + 1U) < length ? 1U : 0U;
    data[i] = SoftI2C_ReadByte(ack);
  }

  SoftI2C_StopCondition();
  return HAL_OK;
}

/* ==================== 私有函数实现 - 底层时序控制 ==================== */

/**
 * @brief  I2C时序延迟函数
 * @details
 *         优化延迟循环次数至100,减少阻塞时间
 *         在72MHz系统时钟下约1.5-2μs,对应I2C频率约250-330kHz
 *
 * @retval 无
 */
static void SoftI2C_Delay(void)
{
  for (volatile uint32_t i = 0U; i < 100U; ++i)
  {
    __NOP();
  }
}

/**
 * @brief  设置SCL引脚为高电平并检测时钟拉伸
 * @details
 *         开漏模式下输出高电平即释放引脚
 *         增加时钟拉伸超时检测,防止死锁
 *
 * @retval HAL_OK     SCL成功拉高
 * @retval HAL_ERROR  时钟拉伸超时
 */
static HAL_StatusTypeDef SoftI2C_SCL_High(void)
{
  HAL_GPIO_WritePin(SOFT_I2C_SCL_PORT, SOFT_I2C_SCL_PIN, GPIO_PIN_SET);
  SoftI2C_Delay();

  /* 检测从机时钟拉伸,最多等待1000次循环 */
  uint32_t timeout = 1000U;
  while ((HAL_GPIO_ReadPin(SOFT_I2C_SCL_PORT, SOFT_I2C_SCL_PIN) == GPIO_PIN_RESET) && (timeout-- > 0U))
  {
    __NOP();
  }

  if (timeout == 0U)
  {
    /* 时钟拉伸超时 - 从机可能挂死或总线短路 */
    return HAL_ERROR;
  }

  SoftI2C_Delay();
  return HAL_OK;
}

/**
 * @brief  设置SCL引脚为低电平
 */
static void SoftI2C_SCL_Low(void)
{
  HAL_GPIO_WritePin(SOFT_I2C_SCL_PORT, SOFT_I2C_SCL_PIN, GPIO_PIN_RESET);
  SoftI2C_Delay();
}

/**
 * @brief  设置SDA引脚为高电平 (释放总线)
 * @details
 *         开漏模式下输出高电平即释放SDA,允许从机控制
 *         无需切换为输入模式,避免GPIO重新初始化开销
 */
static void SoftI2C_SDA_High(void)
{
  HAL_GPIO_WritePin(SOFT_I2C_SDA_PORT, SOFT_I2C_SDA_PIN, GPIO_PIN_SET);
  SoftI2C_Delay();
}

/**
 * @brief  设置SDA引脚为低电平
 */
static void SoftI2C_SDA_Low(void)
{
  HAL_GPIO_WritePin(SOFT_I2C_SDA_PORT, SOFT_I2C_SDA_PIN, GPIO_PIN_RESET);
  SoftI2C_Delay();
}

/**
 * @brief  读取SDA引脚当前状态
 * @details
 *         开漏模式下可直接读取引脚电平,无需切换为输入
 *
 * @retval 1  SDA为高电平
 * @retval 0  SDA为低电平
 */
static uint8_t SoftI2C_SDA_Read(void)
{
  return (HAL_GPIO_ReadPin(SOFT_I2C_SDA_PORT, SOFT_I2C_SDA_PIN) == GPIO_PIN_SET) ? 1U : 0U;
}

/* ==================== I2C协议条件生成 ==================== */

/**
 * @brief  生成I2C起始条件 (START Condition)
 * @details
 *         时序: SDA高→SCL高→SDA低→SCL低
 */
static void SoftI2C_StartCondition(void)
{
  SoftI2C_SDA_High();
  SoftI2C_SCL_High();
  SoftI2C_SDA_Low();
  SoftI2C_SCL_Low();
}

/**
 * @brief  生成I2C停止条件 (STOP Condition)
 * @details
 *         时序: SCL低→SDA低→SCL高→SDA高
 */
static void SoftI2C_StopCondition(void)
{
  SoftI2C_SCL_Low();
  SoftI2C_SDA_Low();
  SoftI2C_SCL_High();
  SoftI2C_SDA_High();
}

/* ==================== 字节级读写 ==================== */

/**
 * @brief  通过I2C总线发送一个字节并等待应答
 * @details
 *         发送8位数据(MSB优先),然后读取ACK位
 *         无需切换SDA输入输出,开漏模式可直接读取
 *
 * @param  data  要发送的8位数据
 *
 * @retval HAL_OK     收到从机ACK
 * @retval HAL_ERROR  收到NACK或通信失败
 */
static HAL_StatusTypeDef SoftI2C_WriteByte(uint8_t data)
{
  /* 发送8个数据位 (从最高位开始) */
  for (uint8_t bit = 0U; bit < 8U; ++bit)
  {
    SoftI2C_SCL_Low();

    /* 设置SDA为当前最高位的值 */
    if ((data & 0x80U) != 0U)
    {
      SoftI2C_SDA_High();
    }
    else
    {
      SoftI2C_SDA_Low();
    }

    data <<= 1U;

    /* SCL时钟脉冲 */
    if (SoftI2C_SCL_High() != HAL_OK)
    {
      return HAL_ERROR;  /* 时钟拉伸超时 */
    }
    SoftI2C_SCL_Low();
  }

  /* 读取应答位 (释放SDA让从机控制) */
  SoftI2C_SDA_High();  /* 释放SDA */
  if (SoftI2C_SCL_High() != HAL_OK)
  {
    return HAL_ERROR;
  }

  /* 读取ACK: 低电平=ACK,高电平=NACK */
  const uint8_t ack_bit = SoftI2C_SDA_Read();
  SoftI2C_SCL_Low();

  return (ack_bit == 0U) ? HAL_OK : HAL_ERROR;
}

/**
 * @brief  从I2C总线读取一个字节并发送应答
 * @details
 *         读取8位数据(MSB优先),然后发送ACK/NACK
 *
 * @param  ack  1=发送ACK(继续读取), 0=发送NACK(停止读取)
 *
 * @retval 读取到的8位数据
 */
static uint8_t SoftI2C_ReadByte(uint8_t ack)
{
  uint8_t data = 0U;

  /* 释放SDA,允许从机控制数据线 */
  SoftI2C_SDA_High();

  /* 依次读取8个数据位 (从最高位开始) */
  for (uint8_t bit = 0U; bit < 8U; ++bit)
  {
    data <<= 1U;

    SoftI2C_SCL_Low();
    SoftI2C_SCL_High();

    /* 读取SDA上的位值 */
    if (SoftI2C_SDA_Read() != 0U)
    {
      data |= 0x01U;
    }

    SoftI2C_SCL_Low();
  }

  /* 发送应答位 */
  if (ack != 0U)
  {
    SoftI2C_SDA_Low();   /* ACK */
  }
  else
  {
    SoftI2C_SDA_High();  /* NACK */
  }

  SoftI2C_SCL_High();
  SoftI2C_SCL_Low();
  SoftI2C_SDA_High();  /* 释放SDA */

  return data;
}
