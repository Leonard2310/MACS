/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body - PIR + OLED + LED + Push Button + Servo
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "gpio.h"
#include "ssd1306.h"
#include "ssd1306_fonts.h"
#include <stdio.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim4;
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define GREEN_LED_Pin GPIO_PIN_10
#define GREEN_LED_GPIO_Port GPIOE

#define PIR3_LED_Pin GPIO_PIN_3
#define PIR3_LED_GPIO_Port GPIOB
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile uint8_t secondPIR_triggered = 0;
volatile uint8_t pir4_show_message = 0;  // Flag per PIR4
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define COUNTDOWN_TIME 10

void Servo_SetAngle(uint8_t angle)
{
    uint16_t pulse = 1000 + ((angle * 1000) / 180);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, pulse);
}

void Servo2_SetAngle(uint8_t angle)
{
    uint16_t pulse = 1000 + ((angle * 1000) / 180);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, pulse);
}

// Callback per interrupt EXTI
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    // PIR2 - per il flusso biglietto/cancello
    if (GPIO_Pin == GPIO_PIN_2)
    {
        secondPIR_triggered = 1;
    }

    // PIR3 - COMPLETAMENTE INDIPENDENTE
    // Solo accende LED e avvia timer, nient'altro!
    else if (GPIO_Pin == GPIO_PIN_9)
    {
        HAL_GPIO_WritePin(PIR3_LED_GPIO_Port, PIR3_LED_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(PIR3_LED2_GPIO_Port, PIR3_LED2_Pin, GPIO_PIN_SET);
        __HAL_TIM_SET_COUNTER(&htim4, 0);
        HAL_TIM_Base_Start_IT(&htim4);
    }

    // PIR4 - Nuovo sensore su PB15
    else if (GPIO_Pin == GPIO_PIN_15)
    {
        pir4_show_message = 1;
    }
}

// Callback Timer - spegne LED PIR3 dopo 2 secondi
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM4)
    {
        HAL_GPIO_WritePin(PIR3_LED_GPIO_Port, PIR3_LED_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(PIR3_LED2_GPIO_Port, PIR3_LED2_Pin, GPIO_PIN_RESET);
        HAL_TIM_Base_Stop_IT(&htim4);
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
  uint8_t confirmed = 0;
  char buffer[32];
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
  MX_TIM3_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  ssd1306_Init();
  ssd1306_Fill(Black);
  ssd1306_SetCursor(10, 10);
  ssd1306_WriteString("Sistema PIR", Font_11x18, White);
  ssd1306_SetCursor(10, 40);
  ssd1306_WriteString("In attesa...", Font_7x10, White);
  ssd1306_UpdateScreen();

  HAL_GPIO_WritePin(GPIOE, Blue_LED_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOE, Red_LED_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(PIR3_LED_GPIO_Port, PIR3_LED_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(PIR3_LED2_GPIO_Port, PIR3_LED2_Pin, GPIO_PIN_RESET);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      // === GESTIONE PIR4: Mostra messaggio quando rileva movimento ===
      if (pir4_show_message)
      {
          ssd1306_Fill(Black);
          ssd1306_SetCursor(5, 15);
          ssd1306_WriteString("MOVIMENTO", Font_11x18, White);
          ssd1306_SetCursor(5, 40);
          ssd1306_WriteString("PIR4 RILEVATO", Font_7x10, White);
          ssd1306_UpdateScreen();

          HAL_Delay(3000);  // Mostra per 3 secondi

          pir4_show_message = 0;

          // Ripristina schermata base
          ssd1306_Fill(Black);
          ssd1306_SetCursor(10, 10);
          ssd1306_WriteString("Sistema PIR", Font_11x18, White);
          ssd1306_SetCursor(10, 40);
          ssd1306_WriteString("In attesa...", Font_7x10, White);
          ssd1306_UpdateScreen();
      }

      // === GESTIONE PRIMO PIR ===
      GPIO_PinState pirState = HAL_GPIO_ReadPin(PIR_Signal_GPIO_Port, PIR_Signal_Pin);

      if (pirState == GPIO_PIN_SET)
      {
          HAL_GPIO_WritePin(GPIOE, Red_LED_Pin, GPIO_PIN_SET);
          HAL_GPIO_WritePin(GPIOE, Blue_LED_Pin, GPIO_PIN_RESET);
          HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, GPIO_PIN_RESET);

          ssd1306_Fill(Black);
          ssd1306_SetCursor(10, 10);
          ssd1306_WriteString("MOVIMENTO!", Font_11x18, White);
          ssd1306_SetCursor(10, 35);
          ssd1306_WriteString("Attendere conferma", Font_7x10, White);
          ssd1306_UpdateScreen();

          confirmed = 0;

          for (int t = COUNTDOWN_TIME; t >= 0; t--)
          {
              ssd1306_Fill(Black);
              ssd1306_SetCursor(10, 10);
              ssd1306_WriteString("CONFERMA IN:", Font_7x10, White);

              sprintf(buffer, "%2d sec", t);
              ssd1306_SetCursor(40, 35);
              ssd1306_WriteString(buffer, Font_11x18, White);
              ssd1306_UpdateScreen();

              if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8) == GPIO_PIN_RESET)
              {
                  confirmed = 1;
                  break;
              }

              HAL_Delay(1000);
          }

          ssd1306_Fill(Black);
          ssd1306_SetCursor(10, 20);

          if (confirmed)
          {
              ssd1306_WriteString("BIGLIETTO", Font_11x18, White);
              ssd1306_SetCursor(10, 45);
              ssd1306_WriteString("COMPRATO", Font_11x18, White);
              ssd1306_UpdateScreen();

              HAL_GPIO_WritePin(GPIOE, Red_LED_Pin, GPIO_PIN_SET);
              HAL_GPIO_WritePin(GPIOE, Blue_LED_Pin, GPIO_PIN_RESET);

              __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 1000);
              HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
              HAL_Delay(500);

              Servo_SetAngle(0);
              HAL_Delay(2000);
              Servo_SetAngle(90);
              HAL_Delay(2000);
              Servo_SetAngle(180);
              HAL_Delay(2000);

              Servo_SetAngle(0);
              HAL_Delay(1000);
              HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3);

              ssd1306_Fill(Black);
              ssd1306_SetCursor(10, 10);
              ssd1306_WriteString("Attendere", Font_11x18, White);
              ssd1306_SetCursor(10, 35);
              ssd1306_WriteString("passaggio...", Font_11x18, White);
              ssd1306_UpdateScreen();

              HAL_NVIC_DisableIRQ(EXTI4_IRQn);

              secondPIR_triggered = 0;

              while (!secondPIR_triggered)
              {
                  HAL_Delay(100);
              }

              HAL_GPIO_WritePin(GPIOE, Red_LED_Pin, GPIO_PIN_RESET);
              HAL_GPIO_WritePin(GPIOE, Blue_LED_Pin, GPIO_PIN_RESET);
              HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, GPIO_PIN_SET);

              ssd1306_Fill(Black);
              ssd1306_SetCursor(10, 10);
              ssd1306_WriteString("PASSAGGIO", Font_11x18, White);
              ssd1306_SetCursor(10, 40);
              ssd1306_WriteString("RILEVATO!", Font_11x18, White);
              ssd1306_UpdateScreen();

              HAL_Delay(2000);

              ssd1306_Fill(Black);
              ssd1306_SetCursor(10, 20);
              ssd1306_WriteString("Apertura", Font_11x18, White);
              ssd1306_SetCursor(10, 45);
              ssd1306_WriteString("cancello...", Font_11x18, White);
              ssd1306_UpdateScreen();

              __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 1000);
              HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
              HAL_Delay(500);

              Servo2_SetAngle(0);
              HAL_Delay(2000);
              Servo2_SetAngle(90);
              HAL_Delay(2000);
              Servo2_SetAngle(180);
              HAL_Delay(2000);

              Servo2_SetAngle(0);
              HAL_Delay(1000);
              HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_3);

              ssd1306_Fill(Black);
              ssd1306_SetCursor(10, 20);
              ssd1306_WriteString("Cancello", Font_11x18, White);
              ssd1306_SetCursor(10, 45);
              ssd1306_WriteString("chiuso!", Font_11x18, White);
              ssd1306_UpdateScreen();

              HAL_Delay(2000);

              HAL_NVIC_EnableIRQ(EXTI4_IRQn);
          }
          else
          {
              ssd1306_WriteString("ANNULLAMENTO", Font_11x18, White);
              ssd1306_UpdateScreen();

              HAL_GPIO_WritePin(GPIOE, Red_LED_Pin, GPIO_PIN_RESET);
              HAL_GPIO_WritePin(GPIOE, Blue_LED_Pin, GPIO_PIN_SET);
              HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, GPIO_PIN_RESET);
          }

          HAL_Delay(3000);

          ssd1306_Fill(Black);
          ssd1306_SetCursor(10, 10);
          ssd1306_WriteString("Sistema PIR", Font_11x18, White);
          ssd1306_SetCursor(10, 40);
          ssd1306_WriteString("In attesa...", Font_7x10, White);
          ssd1306_UpdateScreen();

          HAL_GPIO_WritePin(GPIOE, Blue_LED_Pin, GPIO_PIN_SET);
          HAL_GPIO_WritePin(GPIOE, Red_LED_Pin, GPIO_PIN_RESET);
          HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, GPIO_PIN_RESET);
      }

      HAL_Delay(100);
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
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
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
