
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "adc.h"
#include "can.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "DaqSlave.h"
#include <stdarg.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
uint8_t vetTx[8];
uint16_t leitura_Pot;
uint16_t leitura_Bet;
uint32_t ext1;
uint32_t ext2;
uint32_t ext3;
uint8_t cont = 0;
volatile unsigned long oldtime;
uint16_t tempInt=0;

extern uint16_t leitura_PotInt;
extern uint16_t leitura_BetinaInt;
extern uint32_t Dado_1;
extern uint32_t Dado_2;
extern uint32_t Dado_3;
extern int16_t IRcan0;
extern int16_t IRcan1;
extern int16_t IRcan2;
extern int16_t IRcan3;
extern int16_t IRcan4;
extern int16_t IRcan5;
extern int16_t IRcan6;
extern int16_t IRcan7;
extern int16_t IRcan8;
extern int16_t IRcan9;
extern int16_t IRcan10;
extern int16_t IRcan11;
extern int16_t IRcan12;
extern int16_t IRcan13;
extern int16_t IRcan14;
extern int16_t IRcan15;
extern int16_t IRmedia[16];
extern uint16_t temp;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

void UART_print(char* format, ...){

	char buffer[100];
	uint8_t size = 0;
	buffer[0] = '\0';

	va_list argList;

	va_start(argList, format);
	size = vsprintf(buffer, format, argList);

	HAL_UART_Transmit(&huart1, (uint8_t *)buffer, size, 10);

	va_end(argList);

}
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_ADC2_Init();
  MX_CAN_Init();
  MX_I2C2_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  CAN_ConfigFilter();
  CAN_ConfigFrames();
  CAN_Receive();

  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_Base_Start_IT(&htim3);
  oldtime= HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
  HAL_Delay(5);
 lerEEPROM();
 writeTrimmingValue();
 setConfiguration();
  uint8_t contmlx = 0;
  while (1)
  {

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
	 // HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
	 // HAL_Delay(50);
	//  UART_print("teste");

	  /*TERMOPAR*/
      if(HAL_GetTick() - oldtime> 200)
      {
    	  temp = read_max6675(&hspi1, GPIOA, GPIO_PIN_1);
    	  tempInt = (uint16_t)(temp*100);
    	//  UART_print("temp: %d\t", tempInt);
    	  oldtime = HAL_GetTick();
      }

	  /*POTENCIOMETRO*/

	  HAL_ADC_Start(&hadc1);
	  HAL_ADC_PollForConversion(&hadc1,ADC_CHANNEL_4);
	 // leitura_Bet = HAL_ADC_GetValue(&hadc1);
	  //UART_print("senp: %d\n", leitura_Bet);


	  HAL_ADC_Start(&hadc2);
	  HAL_ADC_PollForConversion(&hadc2,ADC_CHANNEL_9);
	  leitura_Pot = HAL_ADC_GetValue(&hadc2);
	  //UART_print("pot: %d\t", leitura_Pot);
	 // HAL_Delay(200);


	  /*MELEXIS*/
	 	  if(contmlx == 16){
	 		  if (checkConfig())
	 		  {
	 			  lerEEPROM();
	 			  writeTrimmingValue();
	 			  setConfiguration();
	 		  }
	 		  contmlx = 0;
	 	  }
	 	  contmlx++;
	 	//readPTAT();
		  readIR();
	 	 	    	//HAL_Delay(50);
	  /*EXTENSIOMETRIA*/

	  // ext1 = ReadCount_1();
	  // ext2 = ReadCount_2();
	  // ext3 = ReadCount_3();
	  // UART_print("ext1: %d\n", ext3);

	  HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
	  HAL_Delay(50);


	  //transmit_dados();

  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
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
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  leitura_PotInt = Pot_map(leitura_Pot);
  leitura_BetinaInt = leitura_Bet;
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}

void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */

  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  Dado_1 = ext1;
  Dado_2 = ext2;
  Dado_3 = ext3;
  if(cont == 6){
  	IRcan0 = IRmedia[0];			//n botei o for de proposito
  	IRcan1 = IRmedia[1];
  	IRcan2 = IRmedia[2];
  	IRcan3 = IRmedia[3];
  	IRcan4 = IRmedia[4];
  	IRcan5 = IRmedia[5];
  	IRcan6 = IRmedia[6];
  	IRcan7 = IRmedia[7];
  	IRcan8 = IRmedia[8];
  	IRcan9 = IRmedia[9];
  	IRcan10 = IRmedia[10];
  	IRcan11 = IRmedia[11];
  	IRcan12 = IRmedia[12];
  	IRcan13 = IRmedia[13];
  	IRcan14 = IRmedia[14];
  	IRcan15 = IRmedia[15];

  	cont = 0;
  }
  cont++;
  /* USER CODE BEGIN TIM3_IRQn 1 */

  /* USER CODE END TIM3_IRQn 1 */
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
	  UART_print("aaaa");
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
