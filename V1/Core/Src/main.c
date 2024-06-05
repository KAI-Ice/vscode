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
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "delay.h"
#include "lcd.h"
#include "gui.h"
#include "test.h"
#include "touch.h"
#include "OLED.h"
#include "car.h"
#include "niming.h"

//#define FLASH_SAVE_ADDR 	0x08007000
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
void test_Init_Start(void);
void test_while(void);
void test_LCD(void);
void test_Touch(void);
void test_OLED(void);
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
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM6_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
    test_Init_Start();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	test_while();
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
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
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
/***********************/
void test_Init_Start(void)
{
	OLED_Init();
	LCD_Init();	   //液晶屏初始化
	pid_Init(&car);
	HAL_TIM_Base_Start_IT(&htim6);  //启动定时器1，并使能中断
//	static FLASH_EraseInitTypeDef EraseInitStruct={
//		.TypeErase=FLASH_TYPEERASE_SECTORS,
//		.PageAddress=FLASH_SAVE_ADDR,
//		.NbSectors=1
//		
//	}
}

void test_while(void)
{
//	  delay_ms(200);
	  test_LCD();
//	  test_Touch();
	  test_OLED();
	  PID(&car);
	Set_Motor(car.A.PWM,car.B.PWM);
	ANO_DT_Send_F2(car.A.rospeed,3.0*100,car.B.rospeed,3.0*100);
}

/********************/
void test_LCD(void)
{
//	static int16_t AA=70;
//	main_test(); 		//测试主界面
	if(car.A.rospeed&(1<<16))	//如果是负数的话
	{
		LCD_ShowNum(5,15,4294967296-car.A.rospeed,5,15);
	}
	else
	{
		LCD_ShowNum(5,15,car.A.rospeed,5,15);
	}
	if(car.B.rospeed&(1<<16))	//如果是负数的话
	{
		LCD_ShowNum(5,30,4294967296-car.B.rospeed,5,15);
	}
	else
	{
		LCD_ShowNum(5,30,car.B.rospeed,5,15);
	}
}

void test_Touch(void)
{
//	static uint8_t A=0,B=0;
	Touch_Test();
//	HAL_UART_Transmit(&huart1, &A, 8,500);
//	HAL_UART_Transmit(&huart1, &B, 8,500);
//	static uint8_t A=0x0A;
//	HAL_UART_Transmit(&huart1,&A,1,500);
//	A++;
//	delay_ms(500);
//	A++;
//	B++;
}

void test_OLED(void)
{
	//显示转速和速度
	OLED_ShowSignedNum(8*0,16*0,car.A.rospeed,3,OLED_8X16);
	OLED_ShowSignedNum(8*0,16*1,car.B.rospeed,3,OLED_8X16);
	OLED_ShowString(8*4,16*0,"r/min",OLED_8X16);
	OLED_ShowString(8*4,16*1,"r/min",OLED_8X16);
	OLED_ShowFloatNum(8*0,16*2,car.A.velocity,2,2,OLED_8X16);
	OLED_ShowFloatNum(8*0,16*3,car.B.velocity,2,2,OLED_8X16);
	OLED_ShowString(8*6,16*2,"m/s",OLED_8X16);
	OLED_ShowString(8*6,16*3,"m/s",OLED_8X16);
	
	//显示PID参数
	OLED_ShowString(8*9,16*0,"P:",OLED_8X16);
	OLED_ShowFloatNum(8*11,16*0,car.A.PID.Kp,1,2,OLED_8X16);
	OLED_ShowString(8*9,16*1,"I:",OLED_8X16);
	OLED_ShowFloatNum(8*11,16*1,car.A.PID.Ki,1,2,OLED_8X16);
	OLED_ShowString(8*9,16*2,"D:",OLED_8X16);
	OLED_ShowFloatNum(8*11,16*2,car.A.PID.Kd,1,2,OLED_8X16);
	OLED_Update();
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
