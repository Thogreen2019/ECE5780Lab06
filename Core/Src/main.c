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

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  HAL_Init();
  SystemClock_Config();
	
	//Enable RCC
	RCC -> APB1ENR |= RCC_APB1ENR_DACEN;
	RCC -> AHBENR |= RCC_AHBENR_GPIOCEN;
	RCC -> AHBENR |= RCC_AHBENR_GPIOAEN;
	RCC -> APB2ENR |= RCC_APB2ENR_ADC1EN;
 
	/*//6.1
  //Enable LEDs (GPIOC Pins 6-9)
	GPIOC -> MODER |= (1<<18); //Set Green LED to Output
	GPIOC -> OTYPER &= (0<<9); //Set Green LED to no push/pull
	GPIOC -> OSPEEDR &= (0<<18); //Set Green LED to Low Speed
	GPIOC -> PUPDR &= (0<<18); //Set Green LED to no Pull up/down
	
	GPIOC -> MODER |= (1<<12); //Set Red LED to Output
	GPIOC -> OTYPER &= (0<<6); //Set Red LED to no push/pull
	GPIOC -> OSPEEDR &= (0<<12); //Set Red LED to Low Speed
	GPIOC -> PUPDR &= (0<<12); //Set Red LED to no Pull up/down
	
	GPIOC -> MODER |= (1<<14); //Set Blue LED to Output
	GPIOC -> OTYPER &= (0<<7); //Set Blue LED to no push/pull
	GPIOC -> OSPEEDR &= (0<<14); //Set Blue LED to Low Speed
	GPIOC -> PUPDR &= (0<<14); //Set Blue to no Pull up/down
	
	GPIOC -> MODER |= (1<<16); //Set Orange LED to Output
	GPIOC -> OTYPER &= (0<<8); //Set Orange LED to no push/pull
	GPIOC -> OSPEEDR &= (0<<16); //Set Orange LED to Low Speed
	GPIOC -> PUPDR &= (0<<16); //Set Orange LED to no Pull up/down

  //Select PA0 as ADC input
  GPIOA -> MODER &= 0 & (0<<1);

  //Configure ADC to 8-bit resolution, continuous conversion mode, software trigger
  ADC1 -> CFGR1 |= ADC_CFGR1_CONT; // Continuous conversion mode
  ADC1 -> CFGR1 &= ~ADC_CFGR1_RES; // 8-bit resolution

  ADC1 -> CHSELR |= ADC_CHSELR_CHSEL0; //Select PA0 as ADC input channel
 
  ADC1 -> CR |= ADC_CR_ADCAL; //Perform ADC calibration
  while(ADC1->CR & ADC_CR_ADCAL); //Wait for calibration to finish

  ADC1 -> CR |= ADC_CR_ADEN; //Enable ADC
  while(!(ADC1->ISR & ADC_ISR_ADRDY)){} //Wait for ADC to be ready

  ADC1 -> CR |= ADC_CR_ADSTART; // Start ADC conversion

	uint16_t thresholds[] = {500, 1500, 3000, 4000}; //Threshold values for LED activation: values of 20k potentiometer

	while(1){
     uint16_t adc_value = ADC1->DR; //Read ADC data register

     //Adjust LEDs based on ADC value
     for (int i = 0; i < 4; i++) {
       if (adc_value >= thresholds[i]) {
         GPIOC -> ODR = (1 << (6 + i)); // Turn on LED
       } else {
         GPIOC -> ODR = (1 << (22 + i)); // Turn off LED
       }
     }
  }*/

  //6.2
  //Set PA4 to DAC output mode
  GPIOA -> MODER |= (1<<8);
  GPIOA -> MODER |= (1<<9);

  GPIOA -> PUPDR  &= (0<<8);
  GPIOA -> PUPDR &= (0<<9);

	//Enable DAC channel 1
  DAC->CR |= 1;
  DAC->CR |= (1<<3);
  DAC->CR |= (1<<4);
  DAC->CR |= (1<<5);

  // Sine Wave: 8-bit, 32 samples/cycle
  const uint8_t sine_wave[32] = {127,151,175,197,216,232,244,251,254,251,244,
  232,216,197,175,151,127,102,78,56,37,21,9,2,0,2,9,21,37,56,78,102};

  uint8_t index = 0;

  while (1) {
    DAC->DHR8R1 = sine_wave[index]; //Write the next value from the wave-table to DAC channel 1 data register

    index = (index + 1); //Increment index 
    if(index == 32)
    {
      index = 0;
    }

    HAL_Delay(1); //1ms delay
  }
  
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
