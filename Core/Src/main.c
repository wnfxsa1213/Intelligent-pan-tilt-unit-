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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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
  HAL_UART_Transmit(&huart1, (uint8_t *)text, (uint16_t)strlen(text), HAL_MAX_DELAY);
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

  /* ==================== 初始化IMU传感器 (新API) ==================== */

  /* 初始化IMU (内部自动完成I2C初始化、芯片ID验证、硬件复位等) */
  if (IMU_Init() != IMU_OK)
  {
    /* 初始化失败 - 可能的原因:
     * 1. I2C连接问题 (SCL/SDA接线错误或虚焊)
     * 2. MPU6050供电不足或未上电
     * 3. I2C地址错误 (检查AD0引脚状态)
     * 4. 芯片ID验证失败 (WHO_AM_I != 0x68)
     */
    UartSendText("艹，IMU初始化失败，自己检查下线路和供电！\r\n");
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

    /* ==================== 读取并输出IMU传感器数据 (新API) ==================== */

    IMU_Data_t imu_data;  /* 物理单位数据缓冲区 */

    /* 读取IMU数据 (自动转换为物理单位: g, °/s, °C) */
    if (IMU_ReadData(&imu_data) == IMU_OK)
    {
      char uart_buffer[160];

      /* 格式化输出 - 使用浮点数,更简洁易读 */
      const int length = snprintf(
          uart_buffer,
          sizeof(uart_buffer),
          "ACC[g]: %.2f %.2f %.2f | GYRO[deg/s]: %.1f %.1f %.1f | TEMP[C]: %.1f\r\n",
          imu_data.accel_x, imu_data.accel_y, imu_data.accel_z,
          imu_data.gyro_x, imu_data.gyro_y, imu_data.gyro_z,
          imu_data.temperature);

      if (length > 0)
      {
        HAL_UART_Transmit(&huart1, (uint8_t *)uart_buffer, (uint16_t)length, HAL_MAX_DELAY);
      }
    }
    else
    {
      UartSendText("艹，IMU读数又翻车了，赶紧排查！\r\n");
    }

    HAL_Delay(100U);  /* 10Hz输出频率 */

  /* USER CODE END 3 */
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
