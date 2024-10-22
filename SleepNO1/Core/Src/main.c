/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "Shoot.h"
#include "Chassis.hpp"
#include "com_to_raspi.h"
#include "camera_pos_dj.h"
#include "Task.h"

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
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint32_t cnt=0;
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
  MX_TIM14_Init();
  MX_TIM13_Init();
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  MX_TIM11_Init();
  MX_TIM8_Init();
  MX_USART3_UART_Init();
  MX_UART5_Init();
  /* USER CODE BEGIN 2 */
  Task_TIM_Init(); //first!
  SHOOT_Init();
	CAMERA_POS_DJ_Init();
  JiXieBi_Init();
  com_raspi_Init();

//	HAL_UART_Receive_DMA(&huart3,(&huart3)->pRxBuffPtr,7);
//  HAL_UART_Receive_IT(&huart3,(&huart3)->pRxBuffPtr,7);
//  HAL_UARTEx_ReceiveToIdle_DMA(&huart3,(&huart3)->pRxBuffPtr,7);
//  HAL_UARTEx_ReceiveToIdle_IT(&huart3,(&huart3)->pRxBuffPtr,7);
  send_message_to_raspi(0x33);

//---------------------------------底盘调试代码------------------------------------------------------
  HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);	//启动时钟
  //底盘初始
  Chassis.Init(CHASSIS_MOTOR_PWM_DRIVER_TIM, CHASSIS_MOTOR_CALCULATE_TIM);
  Chassis.Set_Control_Method(Control_Method_ANGLE);     //Control_Method_OMEGA   OPENLOOP  ANGLE
  //�?????????????????????:
  //使能计算时钟
  HAL_TIM_Base_Start_IT(&CHASSIS_MOTOR_CALCULATE_TIM);


  // SpeedTypeDef v_front=
  // {
  //   0, 0.4, 0,  0,0,0,0
  // };
  // SpeedTypeDef v_back=
  // {
  //   0, -0.4, 0,  0,0,0,0
  // };
  // SpeedTypeDef v_right=
  // {
  //   0.4, 0, 0,   0,0,0,0
  // };  
  // SpeedTypeDef v_left=
  // {
  //   -0.4, 0, 0,   0,0,0,0
  // }; 
  // SpeedTypeDef v_rotate=
  // {
  //   0, 0, 1.0,    0,0,0,0
  // };
  // SpeedTypeDef v_crotate=
  // {
  //   0, 0, -1.0 ,    0,0,0,0
  // };
  // SpeedTypeDef v_stop=
  // {
  //   0, 0, 0,    0,0,0,0
  // };
  
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  // __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, 4000);
  // HAL_GPIO_WritePin(GPIOE,GPIO_PIN_2,GPIO_PIN_SET);
  // HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3,GPIO_PIN_RESET);

  // __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, 8000);
  // HAL_GPIO_WritePin(GPIOE,GPIO_PIN_4,GPIO_PIN_SET);
  // HAL_GPIO_WritePin(GPIOE,GPIO_PIN_5,GPIO_PIN_RESET);

  // __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_3, 8000);
  // HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7,GPIO_PIN_SET);
  // HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8,GPIO_PIN_RESET);

  // __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_4, 8000);
  // HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,GPIO_PIN_SET);
  // HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_RESET);
  
  while (1)
  {
    Task_Schedule();
    // if(cnt == 5000000)  JiXieBi_JIAQU();
    cnt++;
    
    //if(cnt % 5000000 == 66666){

    //   SHOOT_set_speed(730); //600-750;
     //  SHOOT_START();
    // CAMERA_POS_DJ_ANGLE_SET(0);
    // CAMERA_POS_DJ_ANGLE_ADD(90);
      //SHOOT_START();
    //   JiXieBi_JIAQU();
    //Chassis.Set_add_rad(0,0,100*PI*CHASSIS_ROTATE_RAD_TO_WHEEL_RAD);
    //   // Chassis.Set_add_rad(0,0,CHASSIS_ROTATE_RAD_TO_WHEEL_RAD*PI);//117 10pi ,244 20pi ,184 20pi*4.045 - 10 = 244
    //}
    // else if(cnt == 4000000){
    //   //  Chassis.Set_add_rad(0,0,-0.5*PI*CHASSIS_ROTATE_RAD_TO_WHEEL_RAD);
    // }
    // else if(cnt == 6000000){
    //   //  Chassis.Set_add_rad(0,0,-0.5*PI*CHASSIS_ROTATE_RAD_TO_WHEEL_RAD);
    // }
    // else if(cnt == 8000000){
    //   //  Chassis.Set_add_rad(0,0,-0.5*PI*CHASSIS_ROTATE_RAD_TO_WHEEL_RAD);
    // }
    // else if(cnt == 10000000){
    //   //  Chassis.Set_add_rad(0,0,-0.5*PI*CHASSIS_ROTATE_RAD_TO_WHEEL_RAD);
    // }
    // else if(cnt == 12000000){
    //   //  Chassis.Set_add_rad(0,0,-0.5*PI*CHASSIS_ROTATE_RAD_TO_WHEEL_RAD);
    // }
    // SERVOCMD_MOVE_TIME_WRITE(4,800,1000);
    // HAL_Delay(1000);
    // MOCALUN_start(750);
    // HAL_Delay(3000);
    // MOCALUN_stop();
    // HAL_Delay(2000);
    //---------------------------------底盘调试代码------------------------------------------------------
  // if(cnt == 1)Chassis.Set_add_rad(4*PI,0,0);
  // if(cnt == 10000000) Chassis.Set_add_rad(-4*PI,0,0); 
  // if(cnt == 20000000) Chassis.Set_add_rad(0,4*PI,0); 
  // if(cnt == 30000000) Chassis.Set_add_rad(0,-4*PI,0); 

  // Chassis.Motor[0].Set_Angle_Target(10*PI);
  // Chassis.Motor[1].Set_Angle_Target(10*PI);
  // Chassis.Motor[2].Set_Angle_Target(10*PI);
  // Chassis.Motor[3].Set_Angle_Target(10*PI);

  //  Chassis.Set_Velocity(v_stop);
  //  HAL_Delay(2000); 
  //  Chassis.Set_Velocity(v_back);
  //  HAL_Delay(2000); Chassis.Set_Velocity(v_stop); HAL_Delay(1000); 
	//  Chassis.Set_Velocity(v_front);
  //  HAL_Delay(2000); Chassis.Set_Velocity(v_stop); HAL_Delay(1000);
  //  Chassis.Set_Velocity(v_right);
  //  HAL_Delay(2000); Chassis.Set_Velocity(v_stop); HAL_Delay(1000);
  //  Chassis.Set_Velocity(v_rotate);
  //  HAL_Delay(2000); Chassis.Set_Velocity(v_stop); HAL_Delay(1000);
  //  Chassis.Set_Velocity(v_crotate);
  //  HAL_Delay(2000); Chassis.Set_Velocity(v_stop); HAL_Delay(1000);
  //  Chassis.Set_Velocity(v_left);
  //  HAL_Delay(2000); Chassis.Set_Velocity(v_stop); HAL_Delay(1000);
  //  Chassis.Set_Velocity(v_back);
  //  HAL_Delay(2000); Chassis.Set_Velocity(v_stop); HAL_Delay(1000);

  // __HAL_TIM_SetCompare(&Driver_PWM_TIM, TIM_CHANNEL_2, 16000);
  // HAL_GPIO_WritePin(GPIOE,GPIO_PIN_4,GPIO_PIN_SET);
  // HAL_GPIO_WritePin(GPIOE,GPIO_PIN_5,GPIO_PIN_RESET);

  // HAL_Delay(2000);
  // __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_4, 0);
  // __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_3, 0);
  // __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, 0);
  //__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, 4000);

  // ---------------------------------机械臂调试代�????????????????????????????????????????-----------------------------------------------------
//  SERVOCMD_MOVE_TIME_WRITE(4,500,1000);
//  SERVOCMD_MOVE_TIME_WRITE(3,500,1000);
//  HAL_Delay(2000);
//  SERVOCMD_MOVE_TIME_WRITE(3,800,1000);
//  SERVOCMD_MOVE_TIME_WRITE(4,1000,1000);
//  HAL_Delay(2000);
// ---------------------------------Camera_Pos_DJ-----------------------------------------------------
  //  CAMERA_POS_DJ_ANGLE(0);
	//  HAL_Delay(2000);
  //  CAMERA_POS_DJ_ANGLE(45);
	//  HAL_Delay(2000);
	//  CAMERA_POS_DJ_ANGLE(90);
  //  HAL_Delay(2000);
  //  CAMERA_POS_DJ_ANGLE(135);
	//  HAL_Delay(2000);
	//    CAMERA_POS_DJ_ANGLE(180);
	//  HAL_Delay(2000);
  //  CAMERA_POS_DJ_ANGLE(225);
	//  HAL_Delay(2000);
	//  CAMERA_POS_DJ_ANGLE(270);
  //  HAL_Delay(2000);
  //  CAMERA_POS_DJ_ANGLE(315);
	//  HAL_Delay(2000);
  // ------------------------------------beng-test------------------------------------------------
	 
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
 void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
   if(huart == &RASPI_USINGUART){
     handle_received_data();
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
