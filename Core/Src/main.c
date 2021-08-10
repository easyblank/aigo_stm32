/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "MPU9250.h"
#include "math.h"
#include "stdlib.h"
#include "motor.h"
#include "gps.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
extern uint32_t motor_speed;
extern uint32_t motor_PWM[];
const uint32_t border = 2000;
extern float_t gyroBias[], accelBias[];
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t byte_serial = 0;

uint8_t waypoint[32] = {0};
uint32_t motor_DIFF[4] = {0};

/***********lidar************/

bool scan_start = false;
uint8_t rx3_start[7] = {0};
uint8_t rx3_data[5] = {0};
uint8_t Q = 0;
bool S = false;
uint16_t angle;
uint16_t d;
uint16_t distance[360] = {0};
int16_t avg_DIFF = 0;
uint32_t last = 0;
uint8_t scan_command[2] = {0xA5,0x20};
uint8_t stop_command[2] = {0xA5,0x25};
uint8_t soft_reboot[2] = {0xA5,0x40};
uint8_t scan_express[2] = {0xA5,0x82};
uint8_t scan_force[2] = {0xA5,0x21};
uint8_t device_info[2] = {0xA5,0x50};
uint8_t health_status[2] = {0xA5,0x52};
uint8_t sample_rate[2] = {0xA5,0x59};
uint8_t scan_response[7] = {0xa5, 0x5a, 0x5, 0x0, 0x0, 0x40, 0x81};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Initialize_AIGO(void);
void AIGO_Bluetooth(void);

void AvoidCollision(void);
void ReceiveLidar(void);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
  if(huart->Instance == USART1){
    switch(byte_serial){
    case '1':
      mvForward(motor_speed);
      break;
    case '2':
      mvBackward(motor_speed);
      break;
    case '3':
      mvLeft(motor_speed);
      break;
    case '4':
      mvRight(motor_speed);
      break;
    case '5':
      Stop();
      break;
    }
    HAL_UART_Receive_IT(&huart1, &byte_serial, 1);
  }
  if(huart->Instance == USART2){
    setWaypoint();
    HAL_UART_Receive_DMA(&huart2, waypoint, 32);
  }
  if(huart->Instance == USART3){
    ReceiveLidar();
    HAL_UART_Receive_DMA(&huart3, rx3_data, 10);
  }
}
bool array_element_of_index_equal(uint8_t a[], uint8_t b[], uint8_t size) {
   uint8_t i;
   for(i=0; i<size; i++){
      if( a[i] != b[i] )
         return false;
   }
   return true;
}
int16_t array_avg_compare(uint16_t distance[]){

   uint32_t sum_R = 0;
   uint32_t sum_L = 0;
   uint8_t len_L = 0;
   uint8_t len_R = 0;
   uint16_t avg_R = 0;
   uint16_t avg_L = 0;
   int16_t avg_diff = 0;

   for(int i=0; i<90; i++){
      sum_R += distance[i];
      if(distance[i]!=0){
         len_R++;
      }
   }
   avg_R = sum_R/len_R;

   for(int i=270; i<360; i++){
      sum_L += distance[i];
      if(distance[i]!=0){
         len_L++;
      }
   }
   avg_L = sum_L/len_L;

   avg_diff = avg_R - avg_L;

   return avg_diff;
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM6_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  Initialize_AIGO();
  Startup();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    ReceiveLidar();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    goWaypoint();
  }
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void Initialize_AIGO(){

  //Initialize Motor PWM
  //LF RF : GPIOB
  //LB RB : GPIOD

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, SET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, RESET);

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, SET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, RESET);

  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, SET);
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, RESET);

  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, SET);
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, RESET);


  //Initialize Encoder
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);

  HAL_UART_Receive_DMA(&huart1, &byte_serial, 1);
  HAL_UART_Receive_DMA(&huart2, waypoint, 15);
  HAL_UART_Receive_DMA(&huart3, rx3_data, 10);
  HAL_TIM_Base_Start_IT(&htim6);

  calibrateMPU9250(gyroBias, accelBias);

  while(MPU9250_Init());

  HAL_UART_Transmit(&huart3, scan_command, 2, 100);
}



void AvoidCollision(){

}
void ReceiveLidar(){
  if(scan_start){

     Q = rx3_data[0]>>2;
     S = (rx3_data[0] & 0x01) ? 1 : 0;
     angle = (rx3_data[2]<<7 | rx3_data[1]>>1)/64;
     d = (rx3_data[4]<<8 | rx3_data[3])/4;
     if(d >= 3000){
      distance[angle] = 3000;
     }
     else{
      distance[angle] = d;
     }

     if(S == 1){
      avg_DIFF = array_avg_compare(distance);
      if(avg_DIFF > border || avg_DIFF*(-1) > border){
        if(avg_DIFF>0)
          avg_DIFF = border;
        else
          avg_DIFF = (-1) * border;
      }
      motor_PWM[0] += avg_DIFF;
      motor_PWM[1] -= avg_DIFF;
      motor_PWM[2] += avg_DIFF;
      motor_PWM[3] -= avg_DIFF;
     }
    }
   else{
    if(HAL_UART_Receive(&huart3, rx3_start, 7, 10) == HAL_OK){
     if (array_element_of_index_equal(rx3_start, scan_response, 7)){
      scan_start = true;
     }
    }
   }
}
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

#ifdef  USE_FULL_ASSERT
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
