
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
  uint32_t DataFrameMask = 0b1000000000000;//0x1000;        // 1000000000000 //the Dataframe size must be +1 the size of the data
  uint32_t DataFilled    = 0b011111111111; //NOTE the first bit MUST be a zero
  uint32_t DataEmpty     = 0b000000000000;
  uint8_t LEDnum = 102;//How many LEDs are in series?
  uint8_t Synctime = 5;// how long is the synctime in (ms)? this is dependent on how many leds you have
  uint8_t Partyspeed = 900; //A added delay (in ms) to the partymode, change this to change how fast it's blinking
  uint8_t volatile Buttons = 0x00;
  const uint16_t colors[24]={
		  0b011111111111,0b000000000000,0b011111111111,0b000000000000,0b011111111111,0b000000000000,0b011111111111,0b000000000000,//R
		  0b000000000000,0b011111111111,0b011111111111,0b000000000000,0b000000000000,0b011111111111,0b001011111111,0b000000000000,//G
		  0b000000000000,0b000000000000,0b000000000000,0b011111111111,0b011111111111,0b011111111111,0b000000000000,0b000000000000};//B
  	  	  //0             1              2             3                4             5				6				7
  	  	  //																								//Number 7 will never show
  const uint16_t partycolors[12]={
		  0b011111111111,0b000000000000,0b000000000000,0b011111111111,  //Note the first column is a copy of the last
		  0b000000000000,0b000000000000,0b011111111111,0b000000000000,
		  0b000000000000,0b011111111111,0b000000000000,0b000000000000
  };

  const uint8_t lights[360]={
		     0, 0, 0, 0, 0, 1, 1, 2, 2, 3, 4, 5, 6, 7, 8, 9, 11, 12, 13, 15, 17, 18, 20, 22, 24, 26, 28, 30, 32, 35, 37, 39,
		    42, 44, 47, 49, 52, 55, 58, 60, 63, 66, 69, 72, 75, 78, 81, 85, 88, 91, 94, 97, 101, 104, 107, 111, 114, 117, 121, 124, 127, 131, 134, 137,
		   141, 144, 147, 150, 154, 157, 160, 163, 167, 170, 173, 176, 179, 182, 185, 188, 191, 194, 197, 200, 202, 205, 208, 210, 213, 215, 217, 220, 222, 224, 226, 229,
		   231, 232, 234, 236, 238, 239, 241, 242, 244, 245, 246, 248, 249, 250, 251, 251, 252, 253, 253, 254, 254, 255, 255, 255, 255, 255, 255, 255, 254, 254, 253, 253,
		   252, 251, 251, 250, 249, 248, 246, 245, 244, 242, 241, 239, 238, 236, 234, 232, 231, 229, 226, 224, 222, 220, 217, 215, 213, 210, 208, 205, 202, 200, 197, 194,
		   191, 188, 185, 182, 179, 176, 173, 170, 167, 163, 160, 157, 154, 150, 147, 144, 141, 137, 134, 131, 127, 124, 121, 117, 114, 111, 107, 104, 101, 97, 94, 91,
		    88, 85, 81, 78, 75, 72, 69, 66, 63, 60, 58, 55, 52, 49, 47, 44, 42, 39, 37, 35, 32, 30, 28, 26, 24, 22, 20, 18, 17, 15, 13, 12,
		    11, 9, 8, 7, 6, 5, 4, 3, 2, 2, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		     0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		     0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		     0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};




/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
extern uint8_t NXT_BIT;

 void Bangbang(uint32_t Mask, uint32_t Data){
	 //This function manchesterencodes a datapacket sent to it and tells a timer function
	 //to send bits
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
	 //same as BangBang exept it does not enocde
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

 void Sync(uint8_t time){//Syncs LEDs
	 Bangbang(ResetFrameMask,ResetFrame);
	 //Sends a resetframe to the LEDs, this must be done after each poweron cycle or just random times
	 HAL_Delay(1);
	 Bangbang(SyncFrameMask,SyncFrame);
	 //Tells the LEDs what position in the order they are, Technically not needed this often
	 HAL_Delay(time); //A delay dependent on the number of LEDs in series, the frame must propagate though the entire chain


 }

 void SendColor(uint32_t Red,uint32_t Green,uint32_t Blue){ //accepts red,green,blue
	 //MUST be preceeded with a "Bangbang(DataHeaderFrameMask,DataHeaderFrame);"
	 //in a series of databytes
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
	  u_int16_t partystage;//Used in partymode
	  //PLEACE NOTE 'Buttons' is erased after each itteration hence
	  switch(HAL_GPIO_ReadPin(D11_GPIO_Port,D11_Pin)){
	  case GPIO_PIN_SET:
		  Buttons =0x00;
		  break;

	  case GPIO_PIN_RESET:
		  Buttons =0x01;
		  break;
	  default:
		  break;
	  }
	  switch(HAL_GPIO_ReadPin(D10_GPIO_Port,D10_Pin)){

	  	  case GPIO_PIN_SET:
	  		Buttons =(Buttons | 0b00000010);
	  		  break;
	  	  default:
	  		  break;
	  	  }
	  switch(HAL_GPIO_ReadPin(D12_GPIO_Port,D12_Pin)){
	  	  	  case GPIO_PIN_SET:
	  	  		Buttons =(Buttons | 0b00000100);
	  	  		  break;
	  	  	  default:
	  	  		  break;
	  	  	  }

	  if(HAL_GPIO_ReadPin(D2_GPIO_Port,D2_Pin)){//D2 is the "send" switch
		  HAL_GPIO_WritePin(LD2_GPIO_Port,LD2_Pin,GPIO_PIN_SET);



		  if(Buttons==0b00000111){//Party mode

		  			  for (int partystage=0; partystage<2; partystage++){
		  			  				  Sync(Synctime);

		  			  				  Bangbang(DataHeaderFrameMask,DataHeaderFrame); //Tells the LEDs that data is comming, also toggles the color of the LEDs
		  			  				  for(int i = 0; i < LEDnum/2; i++){ //One package in series for each LED
		  			  				  SendColor(partycolors[partystage],partycolors[partystage+4],partycolors[partystage+8]);
		  			  				  SendColor(partycolors[partystage+1],partycolors[partystage+5],partycolors[partystage+9]);
		  			  				  }

		  			  				  HAL_Delay(1);
		  			  				  Bangbang(DataHeaderFrameMask,DataHeaderFrame); //applies the color of the LEDs
		  			  				  //no data is sent but that is ok, the LEDs hate you anyway
		  			  				  HAL_Delay(Partyspeed);
		  			  }
		  		partystage= (partystage+1)%3;
		  }else{//Partymode done

//CORE CODE
			  for (int q=0; q<20; q++){//in this application sending 20 identical packages is great,
				  // it makes sure for a steady color with few dead pixels when transmission is done
				  Sync(Synctime);

				  Bangbang(DataHeaderFrameMask,DataHeaderFrame); //Tells the LEDs that data is comming, also toggles the color of the LEDs
				  for(int i = 0; i < LEDnum; i++) //One package in series for each LED
				  SendColor(colors[Buttons],colors[Buttons+8],colors[Buttons+16]); //Color-data

				  HAL_Delay(1);
				  Bangbang(DataHeaderFrameMask,DataHeaderFrame); //applies the color of the LEDs
				  //no data is sent but that is ok
				  HAL_Delay(20);
			  }
		  }
	  }
//Core code done

	  HAL_GPIO_WritePin(LD2_GPIO_Port,LD2_Pin,GPIO_PIN_RESET); //Debug, the onboard LED shows if it is sending


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
