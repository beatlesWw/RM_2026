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
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdio.h"
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
#define limit(a,max) \
{if(a>max) a=max;		\
if(a<-max) a=-max;}  \
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
struct  keys
{ uint8_t key_check;
	uint8_t key_juade;
	uint8_t key_flag;
};
struct keys key;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
short	Encode1Count = 0;
short	LASTEncode1Count = 0;
float Motor1Speed=0.00;
float Kp=6;
float Ki=0.006;
float Kd=6;
float err=0;
float goal_speed=0.00;
float last_err=0;
float err_difference=0;
float err_sum=0;
float pid_out=0;
float pwm_value=0;



void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{ if(htim->Instance==TIM3)   //10ms定时器
	{
		key.key_check =HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_12 );
		
			switch(key.key_juade)
			{
				case 0:										//检测按键按下
				{
					if(key.key_check==0)
					{key.key_juade =1;}
				}break ;
				
				case 1:										//是否真的按下
				{
					if(key.key_check==0)
					{key.key_juade =2;
					 key.key_flag =1;
					}
					else key.key_juade =0;
				}break ;
				
				case 2:										//按键释放状态
				{ 
					if(key.key_check==1)
					{key.key_juade =0;
					//key[i].key_flag =1;  
					}
				}break ;
			
		  }
	
			if(key.key_flag ==1)
		{ 
			if(goal_speed ==4)
				goal_speed =0;
			goal_speed +=1;
			key.key_flag =0;
		}
		
			//pid控制过程
		err_difference=err-last_err ;
		err=goal_speed-Motor1Speed ;
		err_sum +=err ;
		limit(err_sum,6);
		last_err=err;
		pid_out =Kp*err+Kd*err_difference+Ki*err_sum  ;
		pwm_value +=pid_out ;
		if(pwm_value>=1000) pwm_value =1000;
		__HAL_TIM_SET_COMPARE (&htim1,TIM_CHANNEL_1 ,pwm_value );
	}
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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim1 ,TIM_CHANNEL_1 ); 
  HAL_TIM_PWM_Start (&htim2,TIM_CHANNEL_ALL );
	HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_ALL);
  HAL_TIM_Base_Start_IT (&htim2);
	HAL_TIM_Base_Start_IT (&htim3);	
	char message[] = "";
	__HAL_TIM_SET_COMPARE (&htim1,TIM_CHANNEL_1 ,0 );  
	
			
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
			HAL_GPIO_WritePin (GPIOA ,GPIO_PIN_3,GPIO_PIN_SET );
  HAL_GPIO_WritePin (GPIOA ,GPIO_PIN_4,GPIO_PIN_RESET );

	Encode1Count =(short)__HAL_TIM_GET_COUNTER (&htim2);
//	Encode1Count = TIM2->CNT;

	Motor1Speed =(float)Encode1Count *10/21.3/11/4;// r/s
	__HAL_TIM_SET_COUNTER(&htim2,0);

	sprintf (message,"counter: %f",Motor1Speed );	
	HAL_UART_Transmit(&huart1,(uint8_t*)message,strlen(message),100);
  HAL_Delay (100);
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
