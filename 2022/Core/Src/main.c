/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "Chassis.hpp"

//#include "drv_bsp.h"
//#include "drv_uart.h"


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

Class_Chassis Chassis;

Class_Sensor Sensor;

uint8_t tx_buffer[9]={0xAB};
uint8_t rx_buffer[10];
uint8_t RxData[16];


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/**
 * @brief HAL��UART����DMA�����ж�
 *
 * @param huart UART���??
 * @param Size ����
 */ 
void Serialplot_Call_Back(uint8_t *Buffer, uint16_t Length)
{
//    if (rx_buffer[0] == '0')
//    {
//        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
//    }
//    else if (rx_buffer[0] == '1')
//    {
//        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
//    }
//    else if (rx_buffer[0] == '2')
//    {
//        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
//    }
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
  MX_DMA_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_1);													//启动时钟
	
//	Uart_Init(&huart2, rx_buffer, 10, Serialplot_Call_Back);

  //底盘初始�???
  Chassis.Init(CHASSIS_MOTOR_PWM_DRIVER_TIM, CHASSIS_MOTOR_CALCULATE_TIM);
  Chassis.Set_Control_Method(Control_Method_OPENLOOP);     //Control_Method_OMEGA   OPENLOOP

  //使能计算时钟
  HAL_TIM_Base_Start_IT(&CHASSIS_MOTOR_CALCULATE_TIM);
  
  //使能遥控器DMA-UART
//  HAL_UART_Receive_DMA(&CHASSIS_DR16_CONTROL_UART, Chassis.DR16.Pack, DR16_PACK_LENGTH);

	HAL_UARTEx_ReceiveToIdle_DMA(&huart3,RxData,16);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  
  SpeedTypeDef v_front=
  {
    0, 0.5, 0,  0,0,0,0
  };
  SpeedTypeDef v_back=
  {
    0, -0.5, 0,  0,0,0,0
  };
  SpeedTypeDef v_right=
  {
    0.5, 0, 0,   0,0,0,0
  };  
  SpeedTypeDef v_left=
  {
    -0.8, 0, 0,   0,0,0,0
  }; 
  SpeedTypeDef v_rotate=
  {
    0, 0, 0.8,    0,0,0,0
  };
  SpeedTypeDef v_crotate=
  {
    0, 0, -1,    0,0,0,0
  };
  SpeedTypeDef v_stop=
  {
    0, 0, 0,    0,0,0,0
  };
  
//  SpeedTypeDef v_front=
//  {
//    0, 0.5, 0
//  };
//  SpeedTypeDef v_back=
//  {
//    0, -0.5, 0
//  };
//  SpeedTypeDef v_left=
//  {
//    -1.0, 0, 0
//  };
// 	
//  SpeedTypeDef v_right=
//  {
//    0.8, 0, 0
//  };   
//  SpeedTypeDef v_rotate=
//  {
//    0, 0, 0.8
//  };
//  
//  SpeedTypeDef v_crotate=
//  {
//    0, 0, -1
//  };
//  SpeedTypeDef v_stop=
//  {
//    0, 0, 0
//  };


//  Chassis.Set_Velocity(v_stop);
//  HAL_Delay(5000);
//  Chassis.Set_Velocity(v_right);
//  HAL_Delay(4000);
//	Chassis.Set_Velocity(v_back);
//  HAL_Delay(4000);
//  Chassis.Set_Velocity(v_stop);
//  HAL_Delay(10000);


//  Chassis.Set_Velocity(v_right);
//  while(1)
//  {
//		 
//		 if(!Sensor.IsOnTrack(3) && Sensor.IsOnTrack(1))
//		 {

//				break;
//		 }
////     Chassis.Motion_CorrectWhenMovingAtX();
//  }
//		Chassis.Set_Velocity(v_stop);
//		HAL_Delay(2000);
//  while(Sensor.IsOnTrack(1) && !Sensor.IsOnTrack(0))
//  {
//		  Chassis.Set_Velocity(v_back);
//			
////    Chassis.Motion_CorrectWhenMovingAtY();
//  }
//  Chassis.Set_Velocity(v_stop);
//	HAL_Delay(10000);


  while (1)
  {

//    Chassis.Set_Velocity(v_stop);
//    HAL_Delay(2000);
//    Chassis.Set_Velocity(v_back);
//    HAL_Delay(6000);
//		Chassis.Set_Velocity(v_stop);
//    HAL_Delay(5000);
//    Chassis.Set_Velocity(v_right);
//    HAL_Delay(2000);
//    Chassis.Set_Velocity(v_rotate);
//    HAL_Delay(2000);
//    Chassis.Set_Velocity(v_crotate);
//    HAL_Delay(2000);
//    Chassis.Set_Velocity(v_left);
//    HAL_Delay(2000);
//    Chassis.Set_Velocity(v_back);
//    HAL_Delay(2000);


//    static uint32_t flag;
//    if (flag == 2500)
//    {
//        flag = 0;
//    }
//    float Omega_Now;
////		float Hall_Encoder_Count;
//    Omega_Now = Chassis.Motor[0].Get_Omega_Now();
////		Hall_Encoder_Count = Chassis.Motor[0].Hall_Encoder_Count;
//    for (uint8_t i = 0; i < 4; i++)
//    {
//        tx_buffer[i + 1] = *((char *)(&Omega_Now) + i);
//    }
//		
//		float led_status;
//    led_status = !HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);
//    for (uint8_t i = 0; i < 4; i++)
//    {
////        tx_buffer[i + 5] = *((char *)(&Hall_Encoder_Count) + i);
//    }
//		
//    flag++;
//    UART_Send_Data(&huart2, tx_buffer, 9);


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef*huart,uint16_t Size)
{
	if(huart==&huart3)//��Ϊ�ص��������������ڹ��ã�����Ҫ���ж����ĸ�����?
	{
		HAL_UARTEx_ReceiveToIdle_DMA(&huart3,RxData,16);
		for(int j=0;j<16;j++)
			Sensor.track[j]=RxData[j];
		//HAL_UART_Receive_DMA(&huart3,RxData,16);//����һ���µ��ж�ʽ����?
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
