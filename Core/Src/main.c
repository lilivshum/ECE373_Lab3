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
#include "adc.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "LCD.h"
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

int h = 0; // timer hours
int m = 0; // timer minutes
int s = 15; // timer seconds

// time value for paused time
int ph = 0;
int pm = 0;
int ps = 0;

int paused_buffer = 0;

char string_val [50];

char string_lc [5];

char mode;// t = timer, p = pause, s = set

int buzz = 0; // tells the buzzer to buzz

void make_00(int num, char* str){


}

uint32_t ADC_OBTAIN_VAL(char* str){
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 1000);
	uint32_t val = HAL_ADC_GetValue(&hadc1);
	// uint32_t voltage = ((val)/4096.0)*3.3;
	// int frac = (int)((voltage - (int)voltage)*100);


	sprintf(str,"%4d",(int)val);
	        //HAL_Delay(500);
	HAL_ADC_Stop(&hadc1);
	return val;
}


void inc_time(int* _h, int* _m, int* _s, char* str){
	*(_s) = *(_s)+1;
		if(*_s == 60){
			*_s = 0;
			*(_m) = *(_m)+1;
			if(*_m == 60){
				*_m = 0;
				*(_h)= *(_h)+1;
			}
		}

	int buffer = sprintf(str, "%2d : %2d : %2d", *_h, *_m,*_s);

}

void inc_pause_time(char* str){
		inc_time(&ph, &pm, &ps, str);
}

void dec_timer_time(char* str){
	if(s > 0 || m  > 0|| h > 0){
		s--;
			if(s == -1){
				m--;
				s = 59;
				if(m == -1){
					h--;
					m = 59;
				}
			}
	}
	int buffer = sprintf(str, "%2d : %2d : %2d", h, m, s);
}

void set_time(int _h, int _m, int _s, char* str){
	h = _h;
	m = _m;
	s = _s;
	int buffer = sprintf(str, "%d : %d : %d", h, m, s);
}

void perform_action(char* str){
	switch (mode){
		case 'p':
			inc_pause_time(str);
			paused_buffer++;
//			if(paused_buffer >= 5){
//				// HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
//				buzz = 1;
//			}
			break;
		case 's':
			// set_time(0, 1, 0, str);
			str = "Hello";
			break;
		case 't':
			paused_buffer = 0;
			dec_timer_time(str);
			break;
	}
	if(paused_buffer > 5){
		buzz = 1;
	} else {
		buzz = 0;
	}
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == button_Pin){
		if(mode == 's'){ //logic could be improved here but wtv
			mode = 't';
			// HAL_Delay(5);
		} else if (mode == 't'){
			mode = 's';
			// HAL_Delay(5);
		}

	} else if(GPIO_Pin == inc_Pin){
		if(mode == 's'){

		}


	}
}

void Display(){
	// LCD_Clear();
	LCD_Display_String((uchar)0, (uchar)0, (uchar*)string_val);
	LCD_Display_Char((uchar)mode , (uchar)0, (uchar)1);
	LCD_Display_String((uchar)2, (uchar)1, (uchar*)string_lc);
}

// timer interrupt handler function
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim == &htim2){
		perform_action(string_val);
	}
	if(htim == &htim3){
		uint32_t val = ADC_OBTAIN_VAL(string_lc);
		if(val > 200){
			mode = 'p';
			// HAL_Delay(50);
		} else if(mode != 's') {
			mode = 't';
		}
	}
	if(buzz){
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
	} else {
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
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
	int buffer = sprintf(string_val, "%d : %d : %d", h, m, s);
	mode = 't';

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
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  LCD_init();
  LCD_Clear();
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_Base_Start_IT(&htim3);
  // LCD_Write_Data(64);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  Display();
	  HAL_Delay(300);
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
