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
#include "tim.h"
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
uint32_t count = 0;
uint32_t todo = 0,lef=0;
int32_t stop =1;
int16_t cnt1=0,cnt2=0;
GPIO_PinState status1[10],status2[10];
  char t;

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
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM8_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim8);
  

  // for(lef = 0;lef<10;lef++){
  //   status1[lef]=GPIO_PIN_SET;
  //   status2[lef]=GPIO_PIN_SET;
  // }

  //HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
  //__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_2,790);


  //HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
  //HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);
  __HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_1,1200);
  __HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_2,1200);
  HAL_Delay(3000);
  __HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_1,750);
  __HAL_TIM_SetCompare(&htim3,TIM_CHANNEL_2,750);

  HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_1);
  __HAL_TIM_SET_AUTORELOAD(&htim4,1000); //2ms
  __HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_1,500);
  HAL_GPIO_WritePin(ENA__GPIO_Port,ENA__Pin,GPIO_PIN_SET);
  HAL_GPIO_WritePin(DIR__GPIO_Port,DIR__Pin,GPIO_PIN_SET);//when zhushi it move to mocalun
  HAL_GPIO_WritePin(DT_5V_GPIO_Port,DT_5V_Pin,GPIO_PIN_SET);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		// if(stop){
    //   HAL_GPIO_WritePin(ENA__GPIO_Port,ENA__Pin,GPIO_PIN_RESET);  
    // }
    count++;
		//if(count==100000)HAL_GPIO_WritePin(ENA__GPIO_Port,ENA__Pin,GPIO_PIN_SET);
		//if(count==200000)HAL_GPIO_WritePin(ENA__GPIO_Port,ENA__Pin,GPIO_PIN_SET);
    // if (count % 100000 == 0 && count < 200000000){
    //   __HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_2,999+count/100000);
    // }
    
    
    // if(count==5000000){

    // }
    if(count == 300000){ //1/T平滑
     __HAL_TIM_SET_AUTORELOAD(&htim4,400); //200us
     __HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_1,200); 
    
    }
    if(count == 600000){ //1/T平滑
     __HAL_TIM_SET_AUTORELOAD(&htim4,200); //2100us
     __HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_1,100); 
    //__HAL_TIM_SetCompare(&htim2,TIM_CHANNEL_2,790);
    }
    // if( cnt1>=7 || cnt2>=7 ){
    //   HAL_GPIO_WritePin(ENA__GPIO_Port,ENA__Pin,GPIO_PIN_RESET);  
    // }
    // if(todo){
    //   todo = 0;
    //   if(status1[0]==GPIO_PIN_RESET) cnt1--;
    //   if(status2[0]==GPIO_PIN_RESET) cnt2--;
    //   for(t=0;t<9;t++){
    //     status1[t]=status1[t+1];
    //     status2[t]=status2[t+1];
    //   }
    //   status1[9] = HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_14);
    //   status2[9] = HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_15);
    //   if(status1[9]==GPIO_PIN_RESET) cnt1++;
    //   if(status2[9]==GPIO_PIN_RESET) cnt2++;
  
    // }
		//else if(status1 || status2){
    //  HAL_GPIO_WritePin(ENA__GPIO_Port,ENA__Pin,GPIO_PIN_SET);  
    //}
    if(count == 750000){
      __HAL_TIM_SET_AUTORELOAD(&htim4,150); //100us
      __HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_1,75);   
    }
    if(count == 900000){
      
      __HAL_TIM_SET_AUTORELOAD(&htim4,126); //100us
      __HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_1,63);   
    }
    if(count == 1050000){
      __HAL_TIM_SET_AUTORELOAD(&htim4,110); //100us
      __HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_1,55);   
    }
    if(count == 1200000){
      __HAL_TIM_SET_AUTORELOAD(&htim4,100); //100us
      __HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_1,50);   
    }
    // if(count == 2500000){
    //   __HAL_TIM_SET_AUTORELOAD(&htim4,90); //100us
    //   __HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_1,45);   
    // }
    // if(count == 2750000){
    //   __HAL_TIM_SET_AUTORELOAD(&htim4,84); //100us
    //   __HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_1,42);   
    // }
    // if(count == 3000000){
    //   __HAL_TIM_SET_AUTORELOAD(&htim4,76); //100us
    //   __HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_1,38);   
    // }
    // if(count == 3250000){
    //   __HAL_TIM_SET_AUTORELOAD(&htim4,72); //100us
    //   __HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_1,36);   
    // }
    // if(count == 3500000){
    //   __HAL_TIM_SET_AUTORELOAD(&htim4,66); //100us
    //   __HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_1,33);   
    // }

    // if(stop){
    //   HAL_GPIO_WritePin(ENA__GPIO_Port,ENA__Pin,GPIO_PIN_RESET);
    //   __HAL_TIM_SET_AUTORELOAD(&htim4,1000); //200us
    //   __HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_1,500);
    // }
      
    // if(count == 5000){
    //   lef++;
    //   HAL_GPIO_TogglePin(PUL__GPIO_Port,PUL__Pin);
    //   count = 0;
    // }

    // if(lef==1000000)break;
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
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_PIN){
  switch (GPIO_PIN)
  {
  case GPIO_PIN_14://上面那个光电�? //笔记：启动时会莫名触发中�?
    {
      //stop = 1;
      if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_14)==GPIO_PIN_SET && HAL_GPIO_ReadPin(DIR__GPIO_Port,DIR__Pin)==GPIO_PIN_RESET){ //must
        HAL_GPIO_WritePin(ENA__GPIO_Port,ENA__Pin,GPIO_PIN_RESET);
        HAL_GPIO_WritePin(DIR__GPIO_Port,DIR__Pin,GPIO_PIN_SET);
      }
      break;  
    }
    
  case GPIO_PIN_15:
    {
      //stop = 1;
      if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_15)==GPIO_PIN_SET && HAL_GPIO_ReadPin(DIR__GPIO_Port,DIR__Pin)==GPIO_PIN_SET){
        HAL_GPIO_WritePin(ENA__GPIO_Port,ENA__Pin,GPIO_PIN_RESET);
        HAL_GPIO_WritePin(DIR__GPIO_Port,DIR__Pin,GPIO_PIN_RESET);
      }
      break;
    }
  default:
    break;
  }
	
      
//      //__HAL_TIM_SET_AUTORELOAD(&htim4,1000); //200us
//      //HAL_GPIO_WritePin(ENA__GPIO_Port,ENA__Pin,GPIO_PIN_RESET);
//      //HAL_GPIO_WritePin(ENA__GPIO_Port,ENA__Pin,GPIO_PIN_RESET);
  
}
// void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){

//   if(htim->Instance == htim8.Instance){
//     todo=1;
//   }
 
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
