/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "gpio.h"

/* USER CODE BEGIN PTD */
typedef enum {
    STATE_IDLE,
    STATE_MOTION_DETECTED
} SystemState_t;
/* USER CODE END PTD */

/* USER CODE BEGIN PD */
#define MOTION_TIMEOUT_MS 5000   // Tempo di ritorno in stato IDLE
/* USER CODE END PD */

/* USER CODE BEGIN PV */
volatile uint8_t motionDetected = 0;
volatile uint32_t motionDetectedTime = 0;

static SystemState_t currentState = STATE_IDLE;
static SystemState_t lastState = STATE_IDLE;
/* USER CODE END PV */

/* USER CODE BEGIN PFP */
void Set_Servo_Angle(uint8_t angle);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
/**
 * @brief Imposta l'angolo del servomotore (0–90°)
 * @param angle Angolo desiderato in gradi
 */
void Set_Servo_Angle(uint8_t angle)
{
    if (angle > 90) angle = 90; // sicurezza
    // Mappa 0°→1000 µs, 90°→2000 µs
    uint32_t pulse_length = 1000U + ((uint32_t)angle * 1000U / 90U);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, pulse_length);
}
/* USER CODE END 0 */

/**
  * @brief  Application entry point.
  */
int main(void)
{
  /* Reset periferiche e init HAL */
  HAL_Init();

  /* Configurazione clock di sistema */
  SystemClock_Config();

  /* Inizializzazione periferiche */
  MX_GPIO_Init();
  MX_TIM3_Init();

  /* Avvio PWM su TIM3 CH3 (PB0) */
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);

  /* Stato iniziale: LED blu ON, servo chiuso */
  HAL_GPIO_WritePin(GPIOE, Blue_LED_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOE, Red_LED_Pin, GPIO_PIN_RESET);
  Set_Servo_Angle(0);

  /* Loop principale */
  while (1)
  {
      uint32_t now = HAL_GetTick();

      // --- Gestione stato movimento ---
      if (motionDetected)
      {
          currentState = STATE_MOTION_DETECTED;
          if ((now - motionDetectedTime) > MOTION_TIMEOUT_MS)
          {
              motionDetected = 0;
              currentState = STATE_IDLE;
          }
      }
      else currentState = STATE_IDLE;

      // --- Cambiamento di stato ---
      if (currentState != lastState)
      {
          if (currentState == STATE_MOTION_DETECTED)
          {
              // Movimento → porta aperta
              HAL_GPIO_WritePin(GPIOE, Red_LED_Pin, GPIO_PIN_SET);
              HAL_GPIO_WritePin(GPIOE, Blue_LED_Pin, GPIO_PIN_RESET);
              Set_Servo_Angle(90);
          }
          else
          {
              // Idle → porta chiusa
              HAL_GPIO_WritePin(GPIOE, Red_LED_Pin, GPIO_PIN_RESET);
              HAL_GPIO_WritePin(GPIOE, Blue_LED_Pin, GPIO_PIN_SET);
              Set_Servo_Angle(0);
          }

          lastState = currentState;
      }

      HAL_Delay(10);
  }
}

/* USER CODE BEGIN 4 */
/**
 * @brief Callback dell'interrupt EXTI per PIR
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == PIR_Signal_Pin)
    {
        motionDetected = 1;
        motionDetectedTime = HAL_GetTick();
    }
}
/* USER CODE END 4 */

/**
  * @brief System Clock Configuration
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Oscillatore HSI + PLL */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    Error_Handler();

  /** Clock CPU e bus */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|
                                RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
