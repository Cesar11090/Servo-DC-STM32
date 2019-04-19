/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2019 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

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
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM1_Init(void);
/* static void INtoA(void); */

/* USER CODE BEGIN PFP */
int oneshot;

//VARIABLES CONVERSION INTEGER TO TEXT/////////////////////////////////////////////////////
int posicion=0;
int lastposicion;
int pulses;
int cumulative_pulses;
int umil=0;
int centena=0;
int decena=0;
int unidad=0;
int variable=0;

//VARIABLES DE POSICION////////////////////////////////////////////////////////////////////
int pwm=0;
int kp=50; //Constante proporcional**
int ki=0; //Constante integral**
int kd=10; //Constante derivativo**
int integral=0;  //Control Integral**
int derivative=0; //Control derivativo**
int last_error=0; // Last error**
int error=0;
int current_posicion=0;
int target_posicion=0;
int state=0;
int x=1;
int counter=0;
int enable=0;

uint8_t text[20];

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

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM4_Init();
  MX_USB_DEVICE_Init();
  MX_TIM1_Init();
  HAL_TIM_Encoder_Start(&htim1,TIM_CHANNEL_ALL);
  HAL_Delay(1000);
  int Count=0;
  int prevCount=0;
  int overFlowCnt=0;
  int encCnt=0;
  int prevencCnt=0;


   GPIOB -> ODR |= GPIO_PIN_6;   //ON PB6
   GPIOB -> ODR |= GPIO_PIN_7;   //ON PB7
   __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
     HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
   __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);
     HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
   GPIOC -> ODR |= GPIO_PIN_13;   //OFF LED

   while(x==1)
 {
	//ENABLE///////////////////////////////////////////////////////////////
   if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14)==0)
             {
       	     enable=1;
             }



    /* USER CODE END 3 */
   while (enable==1)
  {

   //ENCODER///////////////////////////////////////
	  prevencCnt=encCnt;
	  prevCount=Count;
	  Count = TIM1->CNT;

	  if(prevCount>65525 && prevCount<=65535 && Count<10)  //POSITIVE OVER FLOW
		  {
		  overFlowCnt ++;
		  }

	  if(prevCount>=0 && prevCount<15 && Count>65525)     //NEGATIVE OVER FLOW
	  	  {
	  	  overFlowCnt --;
	  	  }

	  encCnt=(65535*overFlowCnt)+Count;  //CALC ENCODER POSICION

	  variable=encCnt-prevencCnt;        //CALC DELTA ENCODER VALUE

 //CUMULATIVE PULSES///////////////////////////////

	  cumulative_pulses=cumulative_pulses-variable;

 //MOTION PWM ///////////////////////////////////////////////////////////////////////////////////////////

	  //Calculate the error****
      error=cumulative_pulses;

      //Calculate Integral******
	  integral=integral+error;

	  //Calculate Derivativo******
	  derivative=error-last_error;

	  //Calculate the control variable****
	  pwm=(kp*error)+(ki*integral)+(kd*derivative);

	  //Limit the control variable******
	  if(pwm>35000)
		  {
			pwm=35000;
		  }
      else if(pwm<-35000)
		  {
			  pwm=-35000;
		  }

      //If the control variable is positive******
      if(pwm>0)
    	  {
    	      __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
    	      GPIOB -> ODR |= GPIO_PIN_7;   //ON PB7
    	      __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, pwm);
    		  GPIOB -> ODR &= ~GPIO_PIN_6; //OFF PB6
    	  }

      //If the control variable is negative******
      else if(pwm<0)
      	  {
    	  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);
    	  GPIOB -> ODR |= GPIO_PIN_6;   //ON PB6
    	  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, -pwm);
    	  GPIOB -> ODR &= ~GPIO_PIN_7; //OFF PB7
      	  }

      //If the control variable is "0"******
      else if(error==0)
                {
              	  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
              	  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);
              	  GPIOB -> ODR |= GPIO_PIN_6;   //ON PB6
              	  GPIOB -> ODR |= GPIO_PIN_7;   //ON PB7

                }

      //Save last error******
      last_error=error;

      //ON POSITION******
      if(error<5 && error>-5)
      {
    	  GPIOC -> ODR &= ~GPIO_PIN_13; //ON LED
      }
      else
      {
    	  GPIOC -> ODR |= GPIO_PIN_13;   //OFF LED
      }

//PULSES//////////////////////////////////////////////////////////////////////////////////////

      if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12)) //READ HIGH STATE PIN
          {
    	  state=0;
          }
      else if(state==0)                        //TRANSITION HIGH TO LOW
      {
    	  state=1;                             //WAIT TO STATE PIN HIGH AGAIN
    	  if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13)==1)
    	      	cumulative_pulses=cumulative_pulses+2;
    	      	else
    	      	cumulative_pulses=cumulative_pulses-2;
      }

//ENABLE//////////////////////////////////////////////////////////////////////////////////////

      if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14)) //READ HIGH STATE PIN
                {
          	    enable=0;
          	  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
          	  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);
          	  GPIOB -> ODR |= GPIO_PIN_6;   //ON PB6
          	  GPIOB -> ODR |= GPIO_PIN_7;   //ON PB7
                }


  }
  /* USER CODE END 3 */

}
}

//INTERRUPT PB12////////////////////////////////////////////////////////////////////////////////////////////////

/*
void EXTI15_10_IRQHandler(void)
{
    if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12))
    {
    	if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13)==0)
    	cumulative_pulses=cumulative_pulses+5;
    	else
    	cumulative_pulses=cumulative_pulses-5;

    }
      HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_12);
 }
*/


//INTEGER TO ASCII//////////////////////////////////////////////////////////////////////////////////////////////
/*
void INtoA(void)
{
//CARACTERES DE VALOR
 umil=cumulative_pulses/1000;
 centena=(cumulative_pulses-umil*1000)/100;
 decena=(cumulative_pulses-umil*1000-centena*100)/10;
 unidad=cumulative_pulses-umil*1000-centena*100-decena*10;
 umil=umil+48;
 centena=centena+48;
 decena=decena+48;
 unidad=unidad+48;
 text[5]=10;
 text[4]=13;
 text[3]=unidad;
 text[2]=decena;
 text[1]=centena;
 text[0]=umil;
}*/

//CLOCK////////////////////////////////////////////////////////////////////////////////////////////////////////////
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
    Error_Handler();
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
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}


//ENCODER/////////////////////////////////////////////////////////////////////////////////////////////////
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 5;
  sConfig.IC2Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 5;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}

//PWM///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 35000;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_SET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PB12 */
    GPIO_InitStruct.Pin = GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /*Configure GPIO pin : PB13 */
    GPIO_InitStruct.Pin = GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB6 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
