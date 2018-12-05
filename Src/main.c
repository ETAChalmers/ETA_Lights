
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
  * COPYRIGHT(c) 2018 STMicroelectronics
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
#include "stm32f4xx_hal.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "globals.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
   uint32_t SyncFrame = 0x3FFF8800;     // 1111111111 1111100010 0000000000
   uint32_t SyncFrameMask = 0x20000000; // 1000000000 0000000000 0000000000

  uint32_t ResetFrame = 0x7FFF4;       // 1111111111 11111 0100
  uint32_t ResetFrameMask = 0x40000;   // 1000000000000000000

  uint32_t DataHeaderFrame = 0x7FFF2;     // 111111111111111 0010
  uint32_t DataHeaderFrameMask = 0x40000; // 100000000000000 0000
  uint32_t DataFrameMask = 0b1000000000000;//0x1000;        // 1000000000000
  uint32_t DataFilled    = 0b011111111111;
  uint32_t DataEmpty     = 0b0000000000000;

  const uint8_t lights[360]={
		  0	,
		  0	,
		  0	,
		  0	,
		  0	,
		  8	,
		  8	,
		  16	,
		  16	,
		  24	,
		  32	,
		  40	,
		  48	,
		  56	,
		  64	,
		  72	,
		  88	,
		  96	,
		  104	,
		  120	,
		  136	,
		  144	,
		  160	,
		  176	,
		  192	,
		  208	,
		  224	,
		  240	,
		  256	,
		  280	,
		  296	,
		  312	,
		  336	,
		  352	,
		  376	,
		  392	,
		  416	,
		  440	,
		  464	,
		  480	,
		  504	,
		  528	,
		  552	,
		  576	,
		  600	,
		  624	,
		  648	,
		  680	,
		  704	,
		  728	,
		  752	,
		  776	,
		  808	,
		  832	,
		  856	,
		  888	,
		  912	,
		  936	,
		  968	,
		  992	,
		  1016	,
		  1048	,
		  1072	,
		  1096	,
		  1128	,
		  1152	,
		  1176	,
		  1200	,
		  1232	,
		  1256	,
		  1280	,
		  1304	,
		  1336	,
		  1360	,
		  1384	,
		  1408	,
		  1432	,
		  1456	,
		  1480	,
		  1504	,
		  1528	,
		  1552	,
		  1576	,
		  1600	,
		  1616	,
		  1640	,
		  1664	,
		  1680	,
		  1704	,
		  1720	,
		  1736	,
		  1760	,
		  1776	,
		  1792	,
		  1808	,
		  1832	,
		  1848	,
		  1856	,
		  1872	,
		  1888	,
		  1904	,
		  1912	,
		  1928	,
		  1936	,
		  1952	,
		  1960	,
		  1968	,
		  1984	,
		  1992	,
		  2000	,
		  2008	,
		  2008	,
		  2016	,
		  2024	,
		  2024	,
		  2032	,
		  2032	,
		  2040	,
		  2040	,
		  2040	,
		  2040	,
		  2040	,
		  2040	,
		  2040	,
		  2032	,
		  2032	,
		  2024	,
		  2024	,
		  2016	,
		  2008	,
		  2008	,
		  2000	,
		  1992	,
		  1984	,
		  1968	,
		  1960	,
		  1952	,
		  1936	,
		  1928	,
		  1912	,
		  1904	,
		  1888	,
		  1872	,
		  1856	,
		  1848	,
		  1832	,
		  1808	,
		  1792	,
		  1776	,
		  1760	,
		  1736	,
		  1720	,
		  1704	,
		  1680	,
		  1664	,
		  1640	,
		  1616	,
		  1600	,
		  1576	,
		  1552	,
		  1528	,
		  1504	,
		  1480	,
		  1456	,
		  1432	,
		  1408	,
		  1384	,
		  1360	,
		  1336	,
		  1304	,
		  1280	,
		  1256	,
		  1232	,
		  1200	,
		  1176	,
		  1152	,
		  1128	,
		  1096	,
		  1072	,
		  1048	,
		  1016	,
		  992	,
		  968	,
		  936	,
		  912	,
		  888	,
		  856	,
		  832	,
		  808	,
		  776	,
		  752	,
		  728	,
		  704	,
		  680	,
		  648	,
		  624	,
		  600	,
		  576	,
		  552	,
		  528	,
		  504	,
		  480	,
		  464	,
		  440	,
		  416	,
		  392	,
		  376	,
		  352	,
		  336	,
		  312	,
		  296	,
		  280	,
		  256	,
		  240	,
		  224	,
		  208	,
		  192	,
		  176	,
		  160	,
		  144	,
		  136	,
		  120	,
		  104	,
		  96	,
		  88	,
		  72	,
		  64	,
		  56	,
		  48	,
		  40	,
		  32	,
		  24	,
		  16	,
		  16	,
		  8	,
		  8	,
		  0	,
		  0	,
		  0	,
		  0	,
		  0	,
		  0	,
		  0	,
		  0	,
		  0	,
		  0	,
		  0	,
		  0	,
		  0	,
		  0	,
		  0	,
		  0	,
		  0	,
		  0	,
		  0	,
		  0	,
		  0	,
		  0	,
		  0	,
		  0	,
		  0	,
		  0	,
		  0	,
		  0	,
		  0	,
		  0	,
		  0	,
		  0	,
		  0	,
		  0	,
		  0	,
		  0	,
		  0	,
		  0	,
		  0	,
		  0	,
		  0	,
		  0	,
		  0	,
		  0	,
		  0	,
		  0	,
		  0	,
		  0	,
		  0	,
		  0	,
		  0	,
		  0	,
		  0	,
		  0	,
		  0	,
		  0	,
		  0	,
		  0	,
		  0	,
		  0	,
		  0	,
		  0	,
		  0	,
		  0	,
		  0	,
		  0	,
		  0	,
		  0	,
		  0	,
		  0	,
		  0	,
		  0	,
		  0	,
		  0	,
		  0	,
		  0	,
		  0	,
		  0	,
		  0	,
		  0	,
		  0	,
		  0	,
		  0	,
		  0	,
		  0	,
		  0	,
		  0	,
		  0	,
		  0	,
		  0	,
		  0	,
		  0	,
		  0	,
		  0	,
		  0	,
		  0	,
		  0	,
		  0	,
		  0	,
		  0	,
		  0	,
		  0	,
		  0	,
		  0	,
		  0	,
		  0	,
		  0	,
		  0	,
		  0	,
		  0	,
		  0	,
		  0	,
		  0	,
		  0	,
		  0	,
		  0	,
		  0	,
		  0	,
		  0	,
		  0	,
		  0	,
		  0	,
		  0	,
		  0};



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
extern uint8_t NXT_BIT;

 void Bangbang(uint32_t Mask, uint32_t Data){
 for( uint32_t a = Mask; a > 0; a >>= 1 ){

		  while(NXT_BIT != 0){
		  }
			  if (Data & a)
			  	  NXT_BIT = 1;
			  	  //HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
		  	  else
			  	  NXT_BIT = 2;
			  	  //HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);


		while(NXT_BIT != 0){
		}
			  if ((Data & a)^a)
				  NXT_BIT = 1;
			  	  //HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
			  else
				  NXT_BIT = 2;
			  	  //HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

 	 	}
 }

 void BangReg(uint32_t Mask, uint32_t Data){
  for( uint32_t a = Mask; a > 0; a >>= 1 ){

 		  while(NXT_BIT != 0){
 		  }
 			  if (Data & a)
 			  	  NXT_BIT = 1;
 			  	  //HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
 		  	  else
 			  	  NXT_BIT = 2;
 			  	  //HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  }
}

 void SendColor(uint32_t Red,uint32_t Green,uint32_t Blue){
	 Bangbang(DataFrameMask,Red);
	 Bangbang(DataFrameMask,Green);
	 Bangbang(DataFrameMask,Blue);


 }

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
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  MX_TIM4_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_Base_Start_IT(&htim4);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


  while (1)
  {
	  Bangbang(ResetFrameMask,ResetFrame);
	  HAL_Delay(3);
	  Bangbang(SyncFrameMask,SyncFrame);
	  HAL_Delay(2);

	  for (int k=0; k<360; k++){
		  Bangbang(ResetFrameMask,ResetFrame);
		  HAL_Delay(1);
		  Bangbang(SyncFrameMask,SyncFrame);
		  	  HAL_Delay(1);
		  Bangbang(DataHeaderFrameMask,DataHeaderFrame);
		  //for(int i = 0; i < 91; i++)
		  SendColor(lights[(k)%360],lights[k+120],lights[(k+240)%360]);
		  HAL_Delay(1);
		  Bangbang(DataHeaderFrameMask,DataHeaderFrame);
	  }



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

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

    /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
