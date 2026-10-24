/* USER CODE BEGIN Header */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "FreeRTOS.h"
#include "cmsis_os2.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "task.h"
#include "a4988.h"
#include "motor.h"
#include "protocol.h"
#include "motor_control.h"
#include "encoder.h"
#include "pid.h"
#include "mecanum.h"
#include "gyro.h"
#include "base_protocol.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>
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

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
DMA_HandleTypeDef hdma_i2c1_rx;
DMA_HandleTypeDef hdma_i2c1_tx;
DMA_HandleTypeDef hdma_i2c2_rx;
DMA_HandleTypeDef hdma_i2c2_tx;

TIM_HandleTypeDef htim1;                          // 编码器 3，PE9 和 PE11
TIM_HandleTypeDef htim2, htim3, htim4;            // 编码器 0 到 2
TIM_HandleTypeDef htim8;                          // 车轮 PWM
TIM_HandleTypeDef htim15;                         // 步进脉冲

UART_HandleTypeDef huart1;  // 手动控制串口
UART_HandleTypeDef huart3;  // 陀螺仪串口
UART_HandleTypeDef huart5;  // 波形串口，PC12
UART_HandleTypeDef huart8;  // ROS 2 主控串口
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_uart8_rx;

// 默认任务
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
// 电机任务
osThreadId_t motorTaskHandle;
const osThreadAttr_t motorTask_attributes = {
  .name = "motorTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
// 命令任务
osThreadId_t commandTaskHandle;
const osThreadAttr_t commandTask_attributes = {
  .name = "commandTask",
  .stack_size = 768 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
// 姿态任务
osThreadId_t imuTaskHandle;
const osThreadAttr_t imuTask_attributes = {
  .name = "imuTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */
// 云台状态
A4988_HandleTypeDef hpan_driver;
A4988_HandleTypeDef htilt_driver;
MT6701 encoder_pan;
MT6701 encoder_tilt;
float zero_pan, zero_tilt;
volatile uint8_t  circle_on;
volatile float    circle_r, circle_spd, circle_ph, circle_cx, circle_cy;

// 底盘控制状态
PID_Controller_t g_pid[4];
float g_base_speed_rpm[4];
PID_Controller_t g_yaw_pid, g_omega_pid;
volatile float g_imu_yaw;
volatile float g_imu_rate_z;
float g_yaw_target, g_yaw_rate_cmd;
uint8_t g_yaw_hold_en, g_omega_en;
volatile uint32_t last_cmd_tick;
volatile uint32_t g_last_imu_update;
static float    prev_imu_yaw;
static uint32_t prev_imu_tick;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_UART5_Init(void);
static void MX_TIM15_Init(void);
static void MX_I2C2_Init(void);
static void MX_UART8_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM8_Init(void);
static void Waveform_SendPidTelemetry(void);
void StartDefaultTask(void *argument);
void StartMotorTask(void *argument);
void StartCommandTask(void *argument);
void StartImuTask(void *argument);

/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// 陀螺仪接收完成回调
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  if (huart->Instance == USART3) {
    Gyro_UART_RxCallback(huart, Size);
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART3) Gyro_HandleUartError(huart);
  if (huart->Instance == UART8) BaseProtocol_HandleUartError(huart);
}
/* USER CODE END 0 */

int main(void)
{
  MPU_Config();
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_TIM15_Init();
  MX_I2C2_Init();
  MX_USART3_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM1_Init();
  MX_TIM8_Init();
  MX_UART5_Init();
  MX_UART8_Init();

  // 初始化云台编码器
  MT6701_Init(&encoder_pan,  &hi2c1, MT6701_I2C_ADDR_PAN);
  MT6701_Init(&encoder_tilt, &hi2c2, MT6701_I2C_ADDR_TILT);

  // 初始化步进驱动
  A4988_PinConfigTypeDef cfg;
  A4988_PIN_CFG(cfg, PAN_STEP_GPIO_Port, PAN_STEP_Pin, PAN_DIR_GPIO_Port, PAN_DIR_Pin,
                PAN_EN_GPIO_Port, PAN_EN_Pin, PAN_MS1_GPIO_Port, PAN_MS1_Pin,
                PAN_MS2_GPIO_Port, PAN_MS2_Pin, PAN_MS3_GPIO_Port, PAN_MS3_Pin,
                PAN_SLEEP_GPIO_Port, PAN_SLEEP_Pin, PAN_RESET_GPIO_Port, PAN_RESET_Pin);
  A4988_Init(&hpan_driver, &cfg);

  A4988_PIN_CFG(cfg, TILT_STEP_GPIO_Port, TILT_STEP_Pin, TILT_DIR_GPIO_Port, TILT_DIR_Pin,
                TILT_EN_GPIO_Port, TILT_EN_Pin, TILT_MS1_GPIO_Port, TILT_MS1_Pin,
                TILT_MS2_GPIO_Port, TILT_MS2_Pin, TILT_MS3_GPIO_Port, TILT_MS3_Pin,
                TILT_SLEEP_GPIO_Port, TILT_SLEEP_Pin, TILT_RESET_GPIO_Port, TILT_RESET_Pin);
  A4988_Init(&htilt_driver, &cfg);

  // 初始化云台轴参数
  Motor_Init(&g_motor[MOTOR_PAN], &hpan_driver,
             PAN_GEAR_RATIO, 0, 0.0f, 0.0f);
  g_motor[MOTOR_PAN].dir_inverted = 1;   // 反转水平轴方向
  Motor_Init(&g_motor[MOTOR_TILT], &htilt_driver,
             TILT_GEAR_RATIO, 1, TILT_ANGLE_MIN, TILT_ANGLE_MAX);
  g_motor[MOTOR_TILT].dir_inverted = 1;  // 反转俯仰轴方向

  g_motor[MOTOR_PAN].current_angle = 0.0f;
  g_motor[MOTOR_PAN].target_angle  = 0.0f;
  g_motor[MOTOR_TILT].current_angle = 0.0f;
  g_motor[MOTOR_TILT].target_angle  = 0.0f;

  // 唤醒步进驱动
  A4988_Wakeup(&hpan_driver);
  A4988_Wakeup(&htilt_driver);

  // 启动串口协议和步进脉冲定时器
  Protocol_Init();
  HAL_TIM_Base_Start_IT(&htim15);

  HAL_UART_Transmit(&huart1, (uint8_t *)"\r\n[BOOT]\r\n", 9, 500);

  // 初始化底盘 PWM 和编码器
  DC_Motor_Init();
  Encoder_Init();

  // 初始化 PID
  for (int i = 0; i < 4; i++)
    PID_Init(&g_pid[i], 0.80f, 2.00f, 0.00f, 100.0f, -100.0f);
  PID_Init(&g_yaw_pid,   YAW_KP, YAW_KI, YAW_KD, YAW_OUT_MAX, YAW_OUT_MIN);
  PID_Init(&g_omega_pid, OMEGA_KP, OMEGA_KI, OMEGA_KD, OMEGA_OUT_MAX, OMEGA_OUT_MIN);

  // 启动陀螺仪接收
  Gyro_Init(&huart3);

  // 启动 ROS 2 串口协议
  if (BaseProtocol_Init(&huart8) != HAL_OK) Error_Handler();

  last_cmd_tick = HAL_GetTick();

  // 启动调度器
  osKernelInitialize();

  defaultTaskHandle  = osThreadNew(StartDefaultTask,  NULL, &defaultTask_attributes);
  motorTaskHandle    = osThreadNew(StartMotorTask,    NULL, &motorTask_attributes);
  commandTaskHandle  = osThreadNew(StartCommandTask,  NULL, &commandTask_attributes);
  imuTaskHandle      = osThreadNew(StartImuTask,      NULL, &imuTask_attributes);

  osKernelStart();
  while (1) {}
}

// 系统时钟
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);
  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 60;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) Error_Handler();
}

// I2C1，水平轴编码器
static void MX_I2C1_Init(void)
{
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x307075B1;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK) Error_Handler();
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK) Error_Handler();
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK) Error_Handler();
}

// I2C2，俯仰轴编码器
static void MX_I2C2_Init(void)
{
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x307075B1;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK) Error_Handler();
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK) Error_Handler();
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK) Error_Handler();
}

// USART1，手动控制
static void MX_USART1_UART_Init(void)
{
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK) Error_Handler();
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK) Error_Handler();
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK) Error_Handler();
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK) Error_Handler();
  CLEAR_BIT(huart1.Instance->CR3, USART_CR3_RXFTCFG | USART_CR3_TXFTCFG);
  { GPIO_InitTypeDef rx_init = {0};
    rx_init.Pin   = GPIO_PIN_10;
    rx_init.Mode  = GPIO_MODE_AF_PP;
    rx_init.Pull  = GPIO_PULLUP;
    rx_init.Speed = GPIO_SPEED_FREQ_LOW;
    rx_init.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &rx_init); }
}

// USART3，陀螺仪
static void MX_USART3_UART_Init(void)
{
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK) Error_Handler();
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK) Error_Handler();
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK) Error_Handler();
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK) Error_Handler();
  CLEAR_BIT(huart3.Instance->CR3, USART_CR3_RXFTCFG | USART_CR3_TXFTCFG);
  { GPIO_InitTypeDef rx_init = {0};
    rx_init.Pin   = GPIO_PIN_9;
    rx_init.Mode  = GPIO_MODE_AF_PP;
    rx_init.Pull  = GPIO_PULLUP;
    rx_init.Speed = GPIO_SPEED_FREQ_LOW;
    rx_init.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOD, &rx_init); }
}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  huart5.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart5.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart5.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_HalfDuplex_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart5, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart5, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

}

// UART8，ROS 2 主控
static void MX_UART8_Init(void)
{
  huart8.Instance = UART8;
  huart8.Init.BaudRate = 115200;
  huart8.Init.WordLength = UART_WORDLENGTH_8B;
  huart8.Init.StopBits = UART_STOPBITS_1;
  huart8.Init.Parity = UART_PARITY_NONE;
  huart8.Init.Mode = UART_MODE_TX_RX;
  huart8.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart8.Init.OverSampling = UART_OVERSAMPLING_16;
  huart8.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart8.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart8.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart8) != HAL_OK) Error_Handler();
  if (HAL_UARTEx_SetTxFifoThreshold(&huart8, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK) Error_Handler();
  if (HAL_UARTEx_SetRxFifoThreshold(&huart8, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK) Error_Handler();
  if (HAL_UARTEx_DisableFifoMode(&huart8) != HAL_OK) Error_Handler();
  CLEAR_BIT(huart8.Instance->CR3, USART_CR3_RXFTCFG | USART_CR3_TXFTCFG);
  { GPIO_InitTypeDef rx_init = {0};
    rx_init.Pin   = GPIO_PIN_0;
    rx_init.Mode  = GPIO_MODE_AF_PP;
    rx_init.Pull  = GPIO_PULLUP;
    rx_init.Speed = GPIO_SPEED_FREQ_LOW;
    rx_init.Alternate = GPIO_AF8_UART8;
    HAL_GPIO_Init(GPIOE, &rx_init); }
}

/* 波形帧包含 10 个通道，发送频率为 10Hz。 */
static void Waveform_SendPidTelemetry(void)
{
  char frame[192];
  float wheel_target[4];
  float wheel_actual[4];
  float yaw_target;
  float yaw_actual;
  uint8_t base_moving = 0U;
  uint32_t primask = __get_PRIMASK();

  __disable_irq();
  for (int i = 0; i < 4; i++) {
    wheel_target[i] = g_pid[i].target;
    wheel_actual[i] = (float)g_motor_encoders[i].speed_rpm;
    if (g_base_speed_rpm[i] != 0.0f) base_moving = 1U;
  }
  yaw_actual = g_imu_yaw;
  yaw_target = g_yaw_hold_en ? g_yaw_target : yaw_actual;
  if (primask == 0U) __enable_irq();

  /* 底盘停止时不发送波形。 */
  if (base_moving == 0U) return;

  int len = snprintf(frame, sizeof(frame),
                     "{B%.1f:%.1f:%.1f:%.1f:%.1f:%.1f:%.1f:%.1f:%.1f:%.1f}$",
                     wheel_target[0], wheel_actual[0],
                     wheel_target[1], wheel_actual[1],
                     wheel_target[2], wheel_actual[2],
                     wheel_target[3], wheel_actual[3],
                     yaw_target, yaw_actual);
  if (len > 0 && len < (int)sizeof(frame)) {
    HAL_UART_Transmit(&huart5, (uint8_t *)frame, (uint16_t)len, 20);
  }
}

// TIM15，步进脉冲定时器
static void MX_TIM15_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  htim15.Instance = TIM15;
  htim15.Init.Prescaler = 47;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 99;
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 0;
  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim15) != HAL_OK) Error_Handler();
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim15, &sClockSourceConfig) != HAL_OK) Error_Handler();
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK) Error_Handler();
}

// TIM1 到 TIM4，车轮编码器
#define MX_TIMx_ENCODER_INIT(tim, inst, period) do { \
  TIM_Encoder_InitTypeDef sCfg = {0}; TIM_MasterConfigTypeDef sMst = {0}; \
  tim.Instance = inst; tim.Init.Prescaler = 0; tim.Init.CounterMode = TIM_COUNTERMODE_UP; \
  tim.Init.Period = period; tim.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1; \
  tim.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE; \
  sCfg.EncoderMode = TIM_ENCODERMODE_TI12; \
  sCfg.IC1Polarity = TIM_ICPOLARITY_RISING; sCfg.IC1Selection = TIM_ICSELECTION_DIRECTTI; \
  sCfg.IC1Prescaler = TIM_ICPSC_DIV1; sCfg.IC1Filter = 0; \
  sCfg.IC2Polarity = TIM_ICPOLARITY_RISING; sCfg.IC2Selection = TIM_ICSELECTION_DIRECTTI; \
  sCfg.IC2Prescaler = TIM_ICPSC_DIV1; sCfg.IC2Filter = 0; \
  HAL_TIM_Encoder_Init(&tim, &sCfg); \
  sMst.MasterOutputTrigger = TIM_TRGO_RESET; sMst.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE; \
  HAL_TIMEx_MasterConfigSynchronization(&tim, &sMst); \
} while(0)

static void MX_TIM2_Init(void) { MX_TIMx_ENCODER_INIT(htim2, TIM2, 4294967295); }
static void MX_TIM3_Init(void) { MX_TIMx_ENCODER_INIT(htim3, TIM3, 65535); }
static void MX_TIM4_Init(void) { MX_TIMx_ENCODER_INIT(htim4, TIM4, 65535); }
static void MX_TIM1_Init(void) { MX_TIMx_ENCODER_INIT(htim1, TIM1, 65535); }

// TIM8，四路车轮 PWM
static void MX_TIM8_Init(void)
{
  TIM_ClockConfigTypeDef sCC = {0}; TIM_MasterConfigTypeDef sMC = {0};
  TIM_OC_InitTypeDef sOC = {0};
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 239;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 99;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK) Error_Handler();
  sCC.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sCC) != HAL_OK) Error_Handler();
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK) Error_Handler();
  sMC.MasterOutputTrigger = TIM_TRGO_RESET; sMC.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMC) != HAL_OK) Error_Handler();
  sOC.OCMode = TIM_OCMODE_PWM1; sOC.Pulse = 0;
  sOC.OCPolarity = TIM_OCPOLARITY_HIGH; sOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sOC, TIM_CHANNEL_1) != HAL_OK) Error_Handler();
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sOC, TIM_CHANNEL_2) != HAL_OK) Error_Handler();
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sOC, TIM_CHANNEL_3) != HAL_OK) Error_Handler();
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sOC, TIM_CHANNEL_4) != HAL_OK) Error_Handler();
  /* TIM8 为高级定时器，需要配置刹车和死区后才能输出。 */
  TIM_BreakDeadTimeConfigTypeDef sBreak = {0};
  sBreak.OffStateRunMode = TIM_OSSR_ENABLE;
  sBreak.OffStateIDLEMode = TIM_OSSI_ENABLE;
  sBreak.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreak.DeadTime = 0;
  sBreak.BreakState = TIM_BREAK_DISABLE;
  sBreak.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreak.BreakFilter = 0;
  sBreak.AutomaticOutput = TIM_AUTOMATICOUTPUT_ENABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreak) != HAL_OK) Error_Handler();
}

// DMA 中断配置
static void MX_DMA_Init(void)
{
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 5, 0); HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 5, 0); HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 5, 0); HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 5, 0); HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 5, 0); HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 5, 0); HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 5, 0); HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0); HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
}

// GPIO 初始化
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  // 设置引脚初始电平
  HAL_GPIO_WritePin(GPIOA, PAN_EN_Pin|PAN_MS1_Pin|PAN_MS2_Pin|PAN_MS3_Pin
                          |PAN_SLEEP_Pin|PAN_RESET_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, PAN_DIR_Pin|PAN_STEP_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOD, TILT_DIR_Pin|TILT_STEP_Pin|TILT_EN_Pin|TILT_MS1_Pin
                          |TILT_MS2_Pin|TILT_MS3_Pin|TILT_SLEEP_Pin|TILT_RESET_Pin, GPIO_PIN_RESET);
  // 车轮方向引脚
  HAL_GPIO_WritePin(GPIOC, M1_IN1_Pin|M1_IN2_Pin|M2_IN1_Pin|M2_IN2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, M3_IN1_Pin|M3_IN2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, M4_IN1_Pin|M4_IN2_Pin, GPIO_PIN_RESET);

  // 水平轴引脚
  GPIO_InitStruct.Pin = PAN_EN_Pin|PAN_MS1_Pin|PAN_MS2_Pin|PAN_MS3_Pin
                          |PAN_SLEEP_Pin|PAN_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = PAN_DIR_Pin|PAN_STEP_Pin;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  // 俯仰轴引脚
  GPIO_InitStruct.Pin = TILT_DIR_Pin|TILT_STEP_Pin|TILT_EN_Pin|TILT_MS1_Pin
                          |TILT_MS2_Pin|TILT_MS3_Pin|TILT_SLEEP_Pin|TILT_RESET_Pin;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
  // 车轮方向引脚
  GPIO_InitStruct.Pin = M1_IN1_Pin|M1_IN2_Pin|M2_IN1_Pin|M2_IN2_Pin;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = M3_IN1_Pin|M3_IN2_Pin;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = M4_IN1_Pin|M4_IN2_Pin;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  HAL_SYSCFG_AnalogSwitchConfig(SYSCFG_SWITCH_PA0, SYSCFG_SWITCH_PA0_CLOSE);

  /* 设置关键引脚电平，复位为高，细分为 16，驱动默认关闭。 */
  HAL_GPIO_WritePin(PAN_RESET_GPIO_Port, PAN_RESET_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(PAN_MS1_GPIO_Port, PAN_MS1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(PAN_MS2_GPIO_Port, PAN_MS2_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(PAN_MS3_GPIO_Port, PAN_MS3_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(TILT_RESET_GPIO_Port, TILT_RESET_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(TILT_MS1_GPIO_Port, TILT_MS1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(TILT_MS2_GPIO_Port, TILT_MS2_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(TILT_MS3_GPIO_Port, TILT_MS3_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(TILT_EN_GPIO_Port, TILT_EN_Pin, GPIO_PIN_SET);
  // 步进引脚使用高速模式
  GPIO_InitStruct.Pin = PAN_STEP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(PAN_STEP_GPIO_Port, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = TILT_STEP_Pin;
  HAL_GPIO_Init(TILT_STEP_GPIO_Port, &GPIO_InitStruct);
  /* 配置 PC6 到 PC9 的 PWM 复用功能，只在这里设置一次。 */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  GPIOC->MODER &= ~((3<<12)|(3<<14)|(3<<16)|(3<<18));
  GPIOC->MODER |=  (2<<12)|(2<<14)|(2<<16)|(2<<18);
  GPIOC->AFR[0] = (GPIOC->AFR[0] & ~((0xFUL<<24)|(0xFUL<<28))) |
                  (3UL<<24)|(3UL<<28);
  GPIOC->AFR[1] = (GPIOC->AFR[1] & ~0xFF) | (3<<0)|(3<<4);
}

// FreeRTOS 任务
// 默认任务，读取云台编码器
void StartDefaultTask(void *argument)
{
  osDelay(200);
  for (;;) {
    osDelay(100);
    if (MT6701_Read(&encoder_pan) == HAL_OK) {
      float raw = MT6701_GetAngle(&encoder_pan);
      float diff = raw - zero_pan;
      if (diff > 180.0f) diff -= 360.0f;
      else if (diff < -180.0f) diff += 360.0f;
      float a = diff / PAN_GEAR_RATIO;
      if (g_motor[MOTOR_PAN].state == MOTOR_IDLE) {
        g_motor[MOTOR_PAN].current_angle = a;
        g_motor[MOTOR_PAN].current_step  = (int32_t)(a * g_motor[MOTOR_PAN].steps_per_degree);
      }
    }
    if (MT6701_Read(&encoder_tilt) == HAL_OK) {
      float raw = MT6701_GetAngle(&encoder_tilt);
      float diff = raw - zero_tilt;
      if (diff > 180.0f) diff -= 360.0f;
      else if (diff < -180.0f) diff += 360.0f;
      float a = diff / TILT_GEAR_RATIO;
      if (g_motor[MOTOR_TILT].state == MOTOR_IDLE) {
        g_motor[MOTOR_TILT].current_angle = a;
        g_motor[MOTOR_TILT].current_step  = (int32_t)(a * g_motor[MOTOR_TILT].steps_per_degree);
      }
    }
  }
}

// 电机任务，更新云台轨迹
void StartMotorTask(void *argument)
{
  Motor_SetTim15Priority();
  for (int i = 0; i < 3; i++) {
    MT6701_Read(&encoder_pan);
    MT6701_Read(&encoder_tilt);
    osDelay(5);
  }
  if (MT6701_OK(&encoder_pan))  zero_pan  = MT6701_GetAngle(&encoder_pan);
  if (MT6701_OK(&encoder_tilt)) zero_tilt = MT6701_GetAngle(&encoder_tilt);

  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xPeriod = pdMS_TO_TICKS(1);
  for (;;) {
    vTaskDelayUntil(&xLastWakeTime, xPeriod);
    if (circle_on) {
      float dt = 1.0f / (float)MOTOR_CTRL_FREQ;
      float omega = circle_spd * 3.14159265f / 180.0f;
      circle_ph += circle_spd * dt;
      if (circle_ph > 360.0f) circle_ph -= 360.0f;
      float rad = circle_ph * 3.14159265f / 180.0f;
      float vp = -circle_r * sinf(rad) * omega;
      float vt =  circle_r * cosf(rad) * omega;
      {
        float tp = Motor_GetAngle(&g_motor[MOTOR_PAN])  + vp * dt;
        float tt = Motor_GetAngle(&g_motor[MOTOR_TILT]) + vt * dt;
        if (tt < TILT_ANGLE_MIN || tt > TILT_ANGLE_MAX) vt = -vt;
      }
      Motor_Spin(&g_motor[MOTOR_PAN],  vp);
      Motor_Spin(&g_motor[MOTOR_TILT], vt);
    }
    Motor_Update(&g_motor[MOTOR_PAN]);
    Motor_Update(&g_motor[MOTOR_TILT]);
  }
}

/* 同时处理手动控制协议和 ROS 2 底盘协议。 */
void StartCommandTask(void *argument)
{
  osDelay(20);
  Protocol_SendWelcome();
  TickType_t xLastMon = xTaskGetTickCount();
  TickType_t xLastBaseReport = xTaskGetTickCount();
  TickType_t xLastWaveform = xTaskGetTickCount();
  for (;;) {
    Protocol_Poll();
    Protocol_CheckTimeout();
    if (Protocol_LineReady()) Protocol_ProcessLine();
    BaseProtocol_Poll();

    TickType_t now = xTaskGetTickCount();
    if (g_monitor && (now - xLastMon) >= pdMS_TO_TICKS(500)) {
      xLastMon = now;
      Protocol_SendStatus();
    }
    if ((now - xLastBaseReport) >= pdMS_TO_TICKS(1000)) {
      xLastBaseReport = now;
      Protocol_SendBaseTelemetry();
    }
    if ((now - xLastWaveform) >= pdMS_TO_TICKS(100)) {
      xLastWaveform = now;
      Waveform_SendPidTelemetry();
    }
    osDelay(1);
  }
}

/* 保存最新姿态数据，供航向控制和遥测使用。 */
void StartImuTask(void *argument)
{
  osDelay(20);
  for (;;) {
    float yaw = 0.0f;
    float rate_z = 0.0f;
    Gyro_Poll();
    uint8_t updates = Gyro_TakeUpdates(&yaw, &rate_z);
    if ((updates & GYRO_UPDATE_YAW) != 0U) g_imu_yaw = yaw;
    if ((updates & GYRO_UPDATE_RATE) != 0U) g_imu_rate_z = rate_z;
    if (updates != 0U) g_last_imu_update = HAL_GetTick();
    osDelay(2);
  }
}

// 中断回调
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM6)
  {
    HAL_IncTick();

    // 每 20 毫秒执行一次底盘控制
    static uint16_t pid_tick = 0;
    pid_tick++;
    if (pid_tick >= 20) {
      pid_tick = 0;
      Encoder_Update_Speed();

      bool imu_timeout = (HAL_GetTick() - g_last_imu_update) > 100;
      if (imu_timeout) { g_omega_en = 0; g_yaw_hold_en = 0; PID_Reset(&g_omega_pid); PID_Reset(&g_yaw_pid); }

      for (int i = 0; i < 4; i++) g_pid[i].target = g_base_speed_rpm[i];

      if (!imu_timeout) {
        if (g_omega_en) {
          float actual_omega = 0.0f; uint32_t now = HAL_GetTick();
          if (prev_imu_tick != 0) {
            float dt = (now - prev_imu_tick) / 1000.0f;
            if (dt > 0.02f && dt < 0.5f) {
              float delta = g_imu_yaw - prev_imu_yaw;
              while (delta > 180.0f) delta -= 360.0f;
              while (delta < -180.0f) delta += 360.0f;
              actual_omega = (delta * 3.14159265358979f / 180.0f) / dt;
            }
          }
          prev_imu_yaw = g_imu_yaw; prev_imu_tick = now;
          if (fabsf(actual_omega) > 50.0f) actual_omega = 0.0f;
          g_omega_pid.target = g_yaw_rate_cmd;
          float omega_out = PID_Compute(&g_omega_pid, actual_omega);
          if (omega_out > OMEGA_OUT_MAX) omega_out = OMEGA_OUT_MAX;
          if (omega_out < OMEGA_OUT_MIN) omega_out = OMEGA_OUT_MIN;
          g_pid[0].target -= omega_out; g_pid[2].target -= omega_out;
          g_pid[1].target += omega_out; g_pid[3].target += omega_out;
        } else if (g_yaw_hold_en) {
          float yaw_error = g_yaw_target - g_imu_yaw;
          while (yaw_error > 180.0f) yaw_error -= 360.0f;
          while (yaw_error < -180.0f) yaw_error += 360.0f;
          /* 航向误差已折算到正负 180 度以内，避免跨越边界时产生接近 360 度的误差。 */
          g_yaw_pid.target = g_yaw_target;
          float yaw_output = PID_Compute(
              &g_yaw_pid, g_yaw_target - yaw_error);
          if (fabsf(yaw_error) > 30.0f) g_yaw_pid.integral = 0.0f;
          if (fabsf(yaw_error) < YAW_DEADBAND_DEG) {
            yaw_output = 0.0f;
            g_yaw_pid.integral = 0.0f;
          } else if (fabsf(yaw_output) < YAW_MIN_CORRECTION_RPM) {
            /* 接近目标时补偿静摩擦，同时避免增益过大引起振荡。 */
            yaw_output = copysignf(YAW_MIN_CORRECTION_RPM, yaw_error);
          }
          if (yaw_output > YAW_OUT_MAX) yaw_output = YAW_OUT_MAX;
          if (yaw_output < YAW_OUT_MIN) yaw_output = YAW_OUT_MIN;
          g_pid[0].target -= yaw_output; g_pid[2].target -= yaw_output;
          g_pid[1].target += yaw_output; g_pid[3].target += yaw_output;
        }
        for (int i = 0; i < 4; i++) {
          if (g_pid[i].target > 300.0f) g_pid[i].target = 300.0f;
          if (g_pid[i].target < -300.0f) g_pid[i].target = -300.0f;
        }
      }
      for (int i = 0; i < 4; i++) {
        float out;
        if (g_base_speed_rpm[i] == 0.0f) {
            out = 0.0f;
            PID_Reset(&g_pid[i]);
        } else {
            // 方向改变时清空积分，提升响应速度
            if ((g_pid[i].target > 0 && g_pid[i].integral < 0) ||
                (g_pid[i].target < 0 && g_pid[i].integral > 0))
                g_pid[i].integral = 0;
            out = PID_Compute(&g_pid[i], g_motor_encoders[i].speed_rpm);
            if (out > 100.0f) out = 100.0f;
            if (out < -100.0f) out = -100.0f;
        }
        DC_Motor_SetSpeed(i, (int16_t)out);
      }
    }
    return;
  }

}

// MPU 配置
void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};
  HAL_MPU_Disable();
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

void Error_Handler(void)
{
  __disable_irq();
  Motor_Stop(&g_motor[MOTOR_PAN]);
  Motor_Stop(&g_motor[MOTOR_TILT]);
  for (int i = 0; i < 4; i++) DC_Motor_SetSpeed(i, 0);
  while (1) {}
}
