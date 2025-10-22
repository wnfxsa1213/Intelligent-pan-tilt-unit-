/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "servo.h"
#include "crsf.h"
#include "watchdog.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "imu.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
  void     (*task_func)(void);
  uint32_t period_ms;
  uint32_t last_run_tick;
  uint8_t  priority;
  const char *name;
} Task_t;

typedef struct
{
  float    pitch_deg;
  float    yaw_deg;
  uint32_t timestamp_ms;
  bool     pending;
} ServoCommand_t;

static void Task_IMU(void);
static void Task_CRSF(void);
static void Task_Servo(void);
static void Task_Watchdog(void);

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ARRAY_SIZE(array) \
  (sizeof(array) / sizeof((array)[0]) + \
   sizeof(char[1 - 2 * __builtin_types_compatible_p(typeof(array), typeof(&(array)[0]))]) * 0U)
#define CRSF_DEBUG_TEST 1U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static bool g_servo_available      = false;
static bool g_servo_fault_reported = false;
static ServoCommand_t g_servo_command = {0.0f, 0.0f, 0U, false};
static Task_t g_task_table[] = {
  { Task_Watchdog,  10U, 0U, 0U, "Watchdog" },
  { Task_CRSF,      20U, 0U, 1U, "CRSF" },
  { Task_Servo,     20U, 0U, 2U, "Servo" },
  { Task_IMU,      100U, 0U, 3U, "IMU" }
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static float CrsfChannelToAngle(uint16_t value);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
 * @brief  通过UART发送文本字符串
 * @details
 *         封装HAL_UART_Transmit函数，简化串口文本输出
 *         自动计算字符串长度，阻塞方式发送
 *
 * @param  text  指向以NULL结尾的C字符串的指针
 *
 * @retval 无
 *
 * @note   使用UART1 (huart1) 作为输出端口
 * @note   参数验证: 拒绝NULL指针，防止程序崩溃
 * @note   阻塞模式 (HAL_MAX_DELAY): 等待数据完全发送完成
 * @note   发送前需确保UART已通过MX_USART1_UART_Init()初始化
 */
static void UartSendText(const char *text)
{
  /* 防御性编程: 检查空指针 */
  if (text == NULL)
  {
    return;  /* 空指针，直接返回避免错误 */
  }

  /* 发送字符串到UART1 */
  const size_t len = strnlen(text, UINT16_MAX);
  if (len == 0U)
  {
    return;
  }

  HAL_UART_Transmit(&huart1, (uint8_t *)text, (uint16_t)len, 50U);
}

static float CrsfChannelToAngle(uint16_t value)
{
  if (value <= CRSF_CHANNEL_VALUE_MIN)
  {
    return SERVO_MIN_ANGLE_DEG;
  }
  if (value >= CRSF_CHANNEL_VALUE_MAX)
  {
    return SERVO_MAX_ANGLE_DEG;
  }

  const float normalized = (float)(value - CRSF_CHANNEL_VALUE_MIN) /
                           (float)(CRSF_CHANNEL_VALUE_MAX - CRSF_CHANNEL_VALUE_MIN);
  return SERVO_MIN_ANGLE_DEG +
         normalized * (SERVO_MAX_ANGLE_DEG - SERVO_MIN_ANGLE_DEG);
}

static bool Servo_ApplyAngles(float pitch_angle, float yaw_angle)
{
  const ServoStatus_t pitch_status = Servo_SetAngle(SERVO_PITCH, pitch_angle);
  const ServoStatus_t yaw_status   = Servo_SetAngle(SERVO_YAW, yaw_angle);

  if ((pitch_status != SERVO_STATUS_OK) || (yaw_status != SERVO_STATUS_OK))
  {
    if (!g_servo_fault_reported)
    {
      UartSendText("艹，舵机命令写不进去，检查TIM5和供电！\r\n");
      g_servo_fault_reported = true;
    }
    return false;
  }

  return true;
}

static void Task_IMU(void)
{
  const uint32_t task_start = HAL_GetTick();
  IMU_Data_t imu_data;

  if (IMU_ReadData(&imu_data) == IMU_OK)
  {
    char uart_buffer[160] = {0};
    const int length = snprintf(
        uart_buffer,
        sizeof(uart_buffer),
        "ACC[g]: %.2f %.2f %.2f | GYRO[deg/s]: %.1f %.1f %.1f | TEMP[C]: %.1f\r\n",
        imu_data.accel_x, imu_data.accel_y, imu_data.accel_z,
        imu_data.gyro_x,  imu_data.gyro_y,  imu_data.gyro_z,
        imu_data.temperature);

    if ((length > 0) && (length < (int)sizeof(uart_buffer)))
    {
      HAL_UART_Transmit(&huart1, (uint8_t *)uart_buffer, (uint16_t)length, 50U);
    }
  }
  else
  {
    UartSendText("[ERROR] IMU数据读取失败，请检查I2C通信状态\r\n");
  }

  const uint32_t task_elapsed = HAL_GetTick() - task_start;
  if (task_elapsed > 50U)
  {
    UartSendText("[WARN] Task_IMU执行超时，可能存在硬件故障\r\n");
  }
}

static void Task_CRSF(void)
{
  CRSF_Update();

  CRSF_Data_t rc_snapshot;
  if (CRSF_PullLatest(&rc_snapshot))
  {
    if (rc_snapshot.link_active)
    {
      g_servo_command.pitch_deg    = CrsfChannelToAngle(rc_snapshot.channels[0U]);
      g_servo_command.yaw_deg      = CrsfChannelToAngle(rc_snapshot.channels[1U]);
      g_servo_command.timestamp_ms = rc_snapshot.timestamp_ms;
      g_servo_command.pending      = true;
    }
    else
    {
      g_servo_command.pending = false;
    }

#if CRSF_DEBUG_TEST
    static uint32_t last_debug_tick = 0U;
    const uint32_t now_tick = HAL_GetTick();
    if ((now_tick - last_debug_tick) >= 500U)
    {
      char debug_buf[160];
      const int written = snprintf(
          debug_buf,
          sizeof(debug_buf),
          "[CRSF] frame=%lu err=%lu link=%u ch0=%u ch1=%u pending=%u\r\n",
          (unsigned long)rc_snapshot.frame_counter,
          (unsigned long)rc_snapshot.frame_error_counter,
          rc_snapshot.link_active ? 1U : 0U,
          (unsigned int)rc_snapshot.channels[0U],
          (unsigned int)rc_snapshot.channels[1U],
          g_servo_command.pending ? 1U : 0U);
      if (written > 0)
      {
        HAL_UART_Transmit(&huart1, (uint8_t *)debug_buf, (uint16_t)written, 1000U);
      }
      last_debug_tick = now_tick;
    }
#endif
  }
}

static void Task_Servo(void)
{
  if (!g_servo_available)
  {
    g_servo_command.pending = false;
    return;
  }

  if (!g_servo_command.pending)
  {
    return;
  }

  if (!Servo_ApplyAngles(g_servo_command.pitch_deg, g_servo_command.yaw_deg))
  {
    g_servo_available       = false;
    g_servo_command.pending = false;
    return;
  }

  g_servo_command.pending = false;
}

static void Task_Watchdog(void)
{
  Watchdog_Kick();
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_TIM5_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  const HAL_StatusTypeDef servo_status = Servo_Init();
  if (servo_status == HAL_OK)
  {
    g_servo_available = true;
  }
  else
  {
    UartSendText("[WARN] 舵机PWM初始化失败，系统运行于无舵机降级模式，请检查TIM5时钟和供电\r\n");
  }

  CRSF_Init();
  Watchdog_Init();

  if (Watchdog_WasReset())
  {
    UartSendText("[WARN] 检测到看门狗复位事件，请排查主循环阻塞问题\r\n");
  }

  /* ==================== 初始化IMU传感器 (新API) ==================== */

  /* 初始化IMU (内部自动完成I2C初始化、芯片ID验证、硬件复位等) */
  const IMU_Status_t imu_status = IMU_Init();
  if (imu_status != IMU_OK)
  {
    /* 初始化失败 - 可能的原因:
     * 1. I2C连接问题 (SCL/SDA接线错误或虚焊)
     * 2. MPU6050供电不足或未上电
     * 3. I2C地址错误 (检查AD0引脚状态)
     * 4. 芯片ID验证失败 (WHO_AM_I != 0x68)
     */
    UartSendText("[ERROR] IMU初始化失败，系统运行于无IMU降级模式，请检查I2C线路和传感器供电\r\n");
  }
  else
  {
    /* 初始化成功，传感器已准备就绪 */
    UartSendText("IMU初始化成功，数据马上给你怼出来。\r\n");
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    const uint32_t now = HAL_GetTick();
    Task_t *selected_task = NULL;

    for (size_t i = 0U; i < ARRAY_SIZE(g_task_table); i++)
    {
      Task_t *task = &g_task_table[i];

      if (task->period_ms == 0U)
      {
        continue;
      }

      if ((now - task->last_run_tick) >= task->period_ms)
      {
        if ((selected_task == NULL) || (task->priority < selected_task->priority))
        {
          selected_task = task;
        }
      }
    }

    if (selected_task != NULL)
    {
#ifdef DEBUG
      const uint32_t start_tick = HAL_GetTick();
#endif

      selected_task->task_func();
      selected_task->last_run_tick = now;

#ifdef DEBUG
      const uint32_t elapsed = HAL_GetTick() - start_tick;
      const uint32_t warn_threshold = (selected_task->period_ms > 1U)
                                        ? (selected_task->period_ms / 2U)
                                        : 1U;
      if (elapsed > warn_threshold)
      {
        char warn_buf[80];
        const int written = snprintf(
            warn_buf,
            sizeof(warn_buf),
            "[WARN] Task %s overrun: %lu ms\r\n",
            selected_task->name,
            (unsigned long)elapsed);
        if (written > 0)
        {
          UartSendText(warn_buf);
        }
      }
#endif

      continue;
    }

    __WFI();

  /* USER CODE END 3 */
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
