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
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
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
 
static uint16_t s_adcRes[10][3];
static uint16_t adc_res[3];
void adc_init(void)
{
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)s_adcRes, 30);
}
uint16_t Adc1(void)
{
    uint32_t res = 0;
    for(uint8_t i = 0; i< 10; i++)
    {
        res += s_adcRes[i][0];
    }
    res = res / 10;
//    printf("ADC1: %d \r\n", res);
    return res;
}
uint16_t Adc2(void)
{
    uint32_t res = 0;
    for(uint8_t i = 0; i< 10; i++)
    {
        res += s_adcRes[i][1];
    }
    res = res / 10;
//    printf("ADC2: %d \r\n", res);
    return res;
}
uint16_t Adc3(void)
{
    uint32_t res = 0;
    for(uint8_t i = 0; i< 10; i++)
    {
        res += s_adcRes[i][2];
    }
    res = res / 10;
//    printf("ADC3: %d \r\n", res);
    return res;
}
int16_t COIL_X=0, COIL_Y=0, COIL_Z=0;
#define Max_Cycle Cycle - 10//限制幅度
#define Min_Cycle -(Cycle - 10)

typedef struct//结构体
{
    int targetValue;//设定值
    int Error;//误差
    int prevError;//上次误差
    int Integral;//误差积分
    float Kp;//比例系数
    float Ki;//没用
    float Kd;//微分系数
}PID;

PID xPID,yPID;

void Init_PID_Parameter(void)
{
    xPID.targetValue = 2450; //  
    yPID.targetValue = 2600;
    xPID.Error = 0;
    yPID.Error = 0;
    xPID.prevError=0;
    yPID.prevError=0;

    xPID.Ki = 0.0f;  
    yPID.Ki = 0.0f;

    xPID.Kp = 13.0f;
    yPID.Kp = 13.0f;
    
    xPID.Kd = 33.5f;
    yPID.Kd = 33.5f;
}
int16_t  Calc_PID(PID *pid,uint16_t Pos)//位置PID
{
    int16_t output;
    int16_t Differential;                              //微分变量

    pid->Error = (pid->targetValue - Pos);         //      误差有正负

    pid->Integral += pid->Error;                   //      误差累计

    Differential = pid->Error - pid->prevError;    //      微分部分本次误差减去上次误差

    output = (pid->Kp * pid->Error + pid->Integral * pid->Ki + pid->Kd * Differential);//通过比例 积分 微分参数算出输出控制量 可省略积分部分

    pid->prevError = pid->Error;                   //      本次误差赋值为上次

    if(output>Max_Cycle)                           //      超过上限则输出恒定值
        output = Max_Cycle;

    if(output<Min_Cycle)
        output = Min_Cycle;

    return output;
}
/**
  * @brief  电机PWM幅值
  * @param  moto1：电机1PWM
  * @param  moto2：电机2PWM
  * @retval 无
  */
void set_PWM(int16_t coil1,int16_t coil2)
{	
	if(coil1 > 0)			
	{
		PWM2_Duty(coil1);
		PWM1_Duty(0);
	}
	else 	          
	{
		PWM2_Duty(0);
		PWM1_Duty(-coil1);
	}
	
	if(coil2 > 0)	
	{
		PWM3_Duty(coil2);
		PWM4_Duty(0);
	}
	else 
	{
		PWM3_Duty(0);
		PWM4_Duty(-coil2);
	}
}
void set_LED(uint8_t statue)
{
    if(statue == 0)
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
    }
    if(statue == 1)
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
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
    uint8_t x = 0;
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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_TIM14_Init();
  /* USER CODE BEGIN 2 */

  adc_init();
  Init_PID_Parameter();
  HAL_Delay(10);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    xPID.Kp = yPID.Kp;
    xPID.Kd = yPID.Kd;
    adc_res[0] = Adc1();
    adc_res[1] = Adc2();
    adc_res[2] = Adc3();
//    printf("ADC1: %d \r\n", s_adcRes[0][0]);
//    printf("ADC1: %d \r\n", s_adcRes[0][1]);
//    printf("ADC1: %d \r\n", s_adcRes[0][2]);
    x = !x;
    if(adc_res[2] < 3200)
        set_LED(x);
    else
        set_LED(0);

    HAL_Delay(500);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the peripherals clocks
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_ADC;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_SYSCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */



void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)    //定时中断回调函数
{
	if(htim->Instance == TIM14)  
	{
        //传递ADC值
        //PID计算PWM
        COIL_X = Calc_PID(&xPID,Adc1());//位置PID
        COIL_Y = Calc_PID(&yPID,Adc2());//位置PID
        //没有浮子时关闭线圈
        if(Adc3() > 3200)
        {
            COIL_X = 0;
            COIL_Y = 0;
            PWM2_Duty(0);
            PWM1_Duty(0);
            PWM3_Duty(0);
            PWM4_Duty(0);
        }
    //    printf("PWM: %d , %d  \r\n", COIL_X, COIL_Y);
        //设置线圈PWM
        set_PWM(COIL_X, COIL_Y);		
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
