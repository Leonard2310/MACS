/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body - PIR + OLED + LED (no servo)
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "gpio.h"
#include "ssd1306.h"
#include "ssd1306_fonts.h"

/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_I2C1_Init();

  /* Inizializza OLED */
  ssd1306_Init();
  ssd1306_Fill(Black);
  ssd1306_SetCursor(10, 10);
  ssd1306_WriteString("Sistema PIR", Font_11x18, White);
  ssd1306_SetCursor(10, 40);
  ssd1306_WriteString("In attesa...", Font_7x10, White);
  ssd1306_UpdateScreen();

  /* Stato iniziale: LED rosso acceso */
  HAL_GPIO_WritePin(GPIOE, Red_LED_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOE, Blue_LED_Pin, GPIO_PIN_RESET);

  while (1)
  {
      /* Leggi stato PIR */
      GPIO_PinState pirState = HAL_GPIO_ReadPin(PIR_Signal_GPIO_Port, PIR_Signal_Pin);

      if (pirState == GPIO_PIN_SET)
      {
          // Movimento rilevato
          HAL_GPIO_WritePin(GPIOE, Blue_LED_Pin, GPIO_PIN_SET);
          HAL_GPIO_WritePin(GPIOE, Red_LED_Pin, GPIO_PIN_RESET);

          ssd1306_Fill(Black);
          ssd1306_SetCursor(10, 10);
          ssd1306_WriteString("MOVIMENTO!", Font_11x18, White);
          ssd1306_SetCursor(10, 40);
          ssd1306_WriteString("Presenza rilevata", Font_7x10, White);
          ssd1306_UpdateScreen();
      }
      else
      {
          // Nessun movimento
          HAL_GPIO_WritePin(GPIOE, Blue_LED_Pin, GPIO_PIN_RESET);
          HAL_GPIO_WritePin(GPIOE, Red_LED_Pin, GPIO_PIN_SET);

          ssd1306_Fill(Black);
          ssd1306_SetCursor(10, 10);
          ssd1306_WriteString("Nessun mov.", Font_11x18, White);
          ssd1306_SetCursor(30, 40);
          ssd1306_WriteString("Area libera", Font_7x10, White);
          ssd1306_UpdateScreen();
      }

      HAL_Delay(200);  // 5 aggiornamenti al secondo
  }
}

/**
  * @brief System Clock Configuration
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    Error_Handler();

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|
                                RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
    Error_Handler();

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    Error_Handler();
}

/**
  * @brief Error Handler
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1) {}
}
