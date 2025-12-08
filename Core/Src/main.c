/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body - Museum Access Control System
 * @description    : This project implements an automated ticketing and access
 *                   control system for a museum using PIR motion sensors,
 *                   servo motors for turnstiles/gates, OLED display for user
 *                   feedback, LEDs for status indication, and a push button
 *                   for ticket purchase confirmation.
 *
 * @hardware       : STM32F303VCT6 Discovery Board
 *                   - PIR1 (PB4): Entrance sensor - detects visitors approaching (polled)
 *                   - PIR2 (PB2): Entry gate sensor - confirms visitor entry (EXTI2)
 *                   - PIR3 (PB9): Ambient lighting sensor - independent (EXTI9)
 *                   - PIR4 (PB15): Exit sensor - detects visitors leaving (EXTI15)
 *                   - Servo1 (PB0/TIM3_CH3): Ticket turnstile
 *                   - Servo2 (PB10/TIM2_CH3): Entry gate
 *                   - Servo3 (PB1/TIM1_CH3N): Exit gate
 *                   - SSD1306 OLED Display (I2C1: PB6/PB7)
 *                   - Blue LED (PE8): System idle/ready
 *                   - Red LED (PE9): Transaction in progress
 *                   - Green LED (PE10): Access granted
 *                   - PIR3_LED (PB3), PIR3_LED2 (PB11): Ambient lighting
 *                   - Button (PB8): Ticket purchase confirmation
 *
 * @features       : - Visitor counter with max capacity (4 visitors)
 *                   - Automatic ticket sales blocking when museum is full
 *                   - Smooth servo motor movements for gates
 *                   - Independent ambient lighting system
 *                   - Exit gate controlled entirely by TIM6 interrupt (15ms period)
 *
 * @authors        : Leonardo Catello, Salvatore Maione, Luisa Ciniglio, Roberta Granata
 * @version        : 1.2 - Fixed TIM6 timing for exit gate
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

/* External timer handles - declared in tim.c
 * htim1: PWM timer for Servo3 (exit gate) on PB1, Channel 3N
 * htim2: PWM timer for Servo2 (entry gate) on PB10, Channel 3
 * htim3: PWM timer for Servo1 (ticket turnstile) on PB0, Channel 3
 * htim4: Base timer for PIR3 LED auto-off timeout (2 seconds)
 * htim6: Base timer for Servo3 movement control (15ms period) - NON-BLOCKING */
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim6;

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* LED Pin Definitions
 * Note: GREEN_LED (PE10) is configured in gpio.c but not named in CubeMX */
#define GREEN_LED_Pin GPIO_PIN_10       // Green LED on PE10 - indicates access granted
#define GREEN_LED_GPIO_Port GPIOE

/* PIR3_LED is already defined in main.h, redefined here for local clarity */
#define PIR3_LED_Pin GPIO_PIN_3         // PIR3 ambient lighting LED on PB3
#define PIR3_LED_GPIO_Port GPIOB

/* Museum capacity configuration */
#define MAX_VISITORS 4                  // Maximum number of visitors allowed in museum

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/**
 * @brief Flag set by PIR2 interrupt to indicate visitor has entered through entry gate
 * @note  Volatile because it's modified in ISR and read in main loop
 */
volatile uint8_t secondPIR_triggered = 0;

/**
 * @brief State machine for exit gate operation (controlled by TIM6 interrupt every 15ms)
 */
typedef enum {
    EXIT_GATE_IDLE = 0,
    EXIT_GATE_OPENING,
    EXIT_GATE_OPEN,
    EXIT_GATE_CLOSING
} ExitGateState_t;

volatile ExitGateState_t exit_gate_state = EXIT_GATE_IDLE;
volatile uint16_t exit_gate_timer = 0;      // Counter for 5 second wait
volatile uint8_t exit_gate_angle = 0;       // Current servo angle

/**
 * @brief Current number of visitors inside the museum
 * @note  Incremented when visitor enters (Servo2), decremented when visitor exits (Servo3)
 *        When reaches MAX_VISITORS, ticket sales are blocked
 */
volatile uint8_t visitor_count = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/** @brief Time in seconds for user to confirm ticket purchase */
#define COUNTDOWN_TIME 10

/**
 * @brief  Sets the servo motor to a specific angle
 * @param  htim: Pointer to the timer handle controlling the servo PWM
 * @param  channel: Timer channel (TIM_CHANNEL_3 or TIM_CHANNEL_4)
 * @param  angle: Desired angle in degrees (0-180)
 * @note   Maps angle (0-180°) to pulse width (500-2500µs) for SG90 servo
 *         Standard servo timing: 500µs = 0°, 1500µs = 90°, 2500µs = 180°
 *         Timer configuration (TIM2/TIM3):
 *         - Prescaler = 7 → Timer clock = 8MHz / 8 = 1MHz
 *         - Period = 19999 → PWM period = 20ms (50Hz)
 *         - Compare values 500-2500 directly map to µs
 * @retval None
 */
void Servo_SetAngle(TIM_HandleTypeDef *htim, uint32_t channel, uint8_t angle)
{
    /* Clamp angle to valid range */
    if (angle > 180) angle = 180;

    /* Calculate pulse width: linear mapping from 0-180° to 500-2500µs
     * Formula: pulse = 500 + (angle * 2000 / 180)
     * Using integer arithmetic to avoid floating point operations */
    uint32_t pulse = 500 + (angle * 2000 / 180);

    /* Set the PWM compare value to control servo position */
    __HAL_TIM_SET_COMPARE(htim, channel, pulse);
}

/**
 * @brief  External interrupt callback for GPIO pins
 * @param  GPIO_Pin: The pin that triggered the interrupt
 * @note   Handles interrupts from:
 *         - PIR2 (PB2, GPIO_PIN_2): Entry gate passage detection
 *         - PIR3 (PB9, GPIO_PIN_9): Ambient lighting activation
 *         - PIR4 (PB15, GPIO_PIN_15): Exit gate activation (only if visitors > 0)
 *         Note: PIR1 (PB4) is handled via polling in main loop
 * @retval None
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    /* PIR2 (PB2) - Detects visitor passing through entry gate after ticket purchase */
    if (GPIO_Pin == GPIO_PIN_2)
    {
        secondPIR_triggered = 1;
    }

    /* PIR3 (PB9) - Independent ambient lighting sensor
     * Turns on LEDs immediately and starts timer for auto-off */
    else if (GPIO_Pin == GPIO_PIN_9)
    {
        /* Turn on both ambient lighting LEDs */
        HAL_GPIO_WritePin(PIR3_LED_GPIO_Port, PIR3_LED_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(PIR3_LED2_GPIO_Port, PIR3_LED2_Pin, GPIO_PIN_SET);

        /* Reset and start TIM4 for LED timeout
         * TIM4 config: Prescaler=7999, Period=1999 → 2 second timeout
         * Calculation: 8MHz / 8000 = 1kHz; 2000 ticks / 1kHz = 2s */
        __HAL_TIM_SET_COUNTER(&htim4, 0);
        HAL_TIM_Base_Start_IT(&htim4);
    }

    /* PIR4 (PB15) - Exit sensor - activates exit gate servo ONLY IF VISITORS > 0 */
    else if (GPIO_Pin == GPIO_PIN_15)
    {
        /* Only activate exit if there are visitors inside and gate is idle */
        if (visitor_count > 0 && exit_gate_state == EXIT_GATE_IDLE)
        {
            exit_gate_state = EXIT_GATE_OPENING;
            exit_gate_angle = 0;

            // Start PWM and timer interrupt for servo control
            HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
            __HAL_TIM_SET_COUNTER(&htim6, 0);
            HAL_TIM_Base_Start_IT(&htim6);
        }
    }
}

/**
 * @brief  Timer period elapsed callback
 * @param  htim: Pointer to the timer handle that triggered the callback
 * @note   TIM4: Used to turn off ambient lighting LEDs after timeout (2s)
 *         TIM6: Used to control exit gate servo movement (15ms interrupts)
 *               - Opening: 0° → 180° in steps of 2° (90 steps × 15ms = 1.35s)
 *               - Wait: 333 interrupts × 15ms = 5 seconds
 *               - Closing: 180° → 0° in steps of 2° (90 steps × 15ms = 1.35s)
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    /* TIM4 callback - Turn off ambient lighting LEDs after 2 second timeout */
    if (htim->Instance == TIM4)
    {
        HAL_GPIO_WritePin(PIR3_LED_GPIO_Port, PIR3_LED_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(PIR3_LED2_GPIO_Port, PIR3_LED2_Pin, GPIO_PIN_RESET);
        HAL_TIM_Base_Stop_IT(&htim4);
    }

    /* TIM6 callback - Control exit gate servo movement (called every 15ms)
     * TIM6 Configuration: Prescaler=119, Period=999
     * Timer frequency: 8MHz / 120 = 66.67kHz
     * Interrupt period: 1000 / 66.67kHz = 15ms */
    else if (htim->Instance == TIM6)
    {
        switch (exit_gate_state)
        {
            case EXIT_GATE_IDLE:
                // Nothing to do - waiting for PIR4 trigger
                break;

            case EXIT_GATE_OPENING:
                // Gradually open gate: 0° → 180° in 2° steps
                exit_gate_angle += 2;
                if (exit_gate_angle >= 180)
                {
                    exit_gate_angle = 180;
                    exit_gate_state = EXIT_GATE_OPEN;
                    exit_gate_timer = 0;  // Reset counter for 5 second wait
                }
                Servo_SetAngle(&htim1, TIM_CHANNEL_3, exit_gate_angle);
                break;

            case EXIT_GATE_OPEN:
                // Wait 5 seconds with gate fully open
                exit_gate_timer++;
                if (exit_gate_timer >= 333)  // 333 × 15ms = 4995ms ≈ 5 seconds
                {
                    exit_gate_state = EXIT_GATE_CLOSING;
                }
                break;

            case EXIT_GATE_CLOSING:
                // Gradually close gate: 180° → 0° in 2° steps
                if (exit_gate_angle >= 2)
                {
                    exit_gate_angle -= 2;
                    Servo_SetAngle(&htim1, TIM_CHANNEL_3, exit_gate_angle);
                }
                else
                {
                    // Gate fully closed
                    exit_gate_angle = 0;
                    Servo_SetAngle(&htim1, TIM_CHANNEL_3, 0);
                    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);
                    HAL_TIM_Base_Stop_IT(&htim6);  // Stop timer interrupt

                    // Decrement visitor counter
                    if (visitor_count > 0)
                    {
                        visitor_count--;
                    }

                    exit_gate_state = EXIT_GATE_IDLE;
                }
                break;
        }
    }
}

/**
 * @brief  Updates the display with current visitor count
 * @param  buffer: Character buffer for sprintf formatting
 * @retval None
 */
void Update_Display_Idle(char *buffer)
{
    ssd1306_Fill(Black);
    ssd1306_SetCursor(20, 5);
    ssd1306_WriteString("MUSEUM", Font_11x18, White);

    if (visitor_count >= MAX_VISITORS)
    {
        ssd1306_SetCursor(35, 28);
        ssd1306_WriteString("FULL", Font_11x18, White);
    }
    else
    {
        ssd1306_SetCursor(15, 28);
        ssd1306_WriteString("Welcome!", Font_11x18, White);
    }

    sprintf(buffer, "Visitors: %d/%d", visitor_count, MAX_VISITORS);
    ssd1306_SetCursor(15, 50);
    ssd1306_WriteString(buffer, Font_7x10, White);
    ssd1306_UpdateScreen();
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  uint8_t confirmed = 0;      /* Flag to track if ticket was confirmed */
  char buffer[32];            /* Buffer for sprintf display formatting */
  uint8_t last_visitor_count = 0; /* Track changes in visitor count */
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
  MX_TIM1_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */

  /* ========== OLED Display Initialization ========== */
  ssd1306_Init();
  Update_Display_Idle(buffer);

  /* ========== LED Initial State ========== */
  /* Blue LED ON = System ready and waiting for visitors */
  HAL_GPIO_WritePin(GPIOE, Blue_LED_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOE, Red_LED_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(PIR3_LED_GPIO_Port, PIR3_LED_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(PIR3_LED2_GPIO_Port, PIR3_LED2_Pin, GPIO_PIN_RESET);

  /* ========== Servo Motors Initialization ========== */
  /* Initialize Servo1 (ticket turnstile) to closed position (0°) - TIM3_CH3 */
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  Servo_SetAngle(&htim3, TIM_CHANNEL_3, 0);
  HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3);

  /* Initialize Servo2 (entry gate) to closed position (0°) - TIM2_CH3 */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  Servo_SetAngle(&htim2, TIM_CHANNEL_3, 0);
  HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_3);

  /* Initialize Servo3 (exit gate) to closed position (0°) - TIM1_CH3N */
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
  Servo_SetAngle(&htim1, TIM_CHANNEL_3, 0);
  HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

      /* ================================================================
       * ENTRANCE PIR SENSOR (PIR1 on PB4) - VISITOR DETECTION
       * ================================================================
       * This sensor is polled (not interrupt-driven) because the system
       * needs to be in idle state to start a new ticket transaction.
       *
       * Flow: Detect visitor -> Check capacity -> Show countdown
       *       -> Wait for button press -> If confirmed: open turnstile
       *       -> Wait for PIR2 -> Open entry gate -> Increment counter
       *       -> Return to idle
       * ================================================================ */

      GPIO_PinState pirState = HAL_GPIO_ReadPin(PIR_Signal_GPIO_Port, PIR_Signal_Pin);

      if (pirState == GPIO_PIN_SET)
      {
          /* ---------- Check Museum Capacity ---------- */
          if (visitor_count >= MAX_VISITORS)
          {
              /* Museum is full - display warning message */
              HAL_GPIO_WritePin(GPIOE, Red_LED_Pin, GPIO_PIN_SET);
              HAL_GPIO_WritePin(GPIOE, Blue_LED_Pin, GPIO_PIN_RESET);

              ssd1306_Fill(Black);
              ssd1306_SetCursor(20, 5);
              ssd1306_WriteString("MUSEUM", Font_11x18, White);
              ssd1306_SetCursor(35, 28);
              ssd1306_WriteString("FULL", Font_11x18, White);
              ssd1306_SetCursor(15, 50);
              ssd1306_WriteString("Please wait...", Font_7x10, White);
              ssd1306_UpdateScreen();

              HAL_Delay(3000);

              /* Return to idle with full status */
              HAL_GPIO_WritePin(GPIOE, Red_LED_Pin, GPIO_PIN_RESET);
              HAL_GPIO_WritePin(GPIOE, Blue_LED_Pin, GPIO_PIN_SET);

              Update_Display_Idle(buffer);
          }
          else
          {
              /* ---------- Visitor Detected - Start Ticket Process ---------- */
              /* Red LED ON = Transaction in progress */
              HAL_GPIO_WritePin(GPIOE, Red_LED_Pin, GPIO_PIN_SET);
              HAL_GPIO_WritePin(GPIOE, Blue_LED_Pin, GPIO_PIN_RESET);
              HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, GPIO_PIN_RESET);

              /* Display welcome message */
              ssd1306_Fill(Black);
              ssd1306_SetCursor(15, 5);
              ssd1306_WriteString("Welcome!", Font_11x18, White);
              ssd1306_SetCursor(5, 30);
              ssd1306_WriteString("Press button to", Font_7x10, White);
              ssd1306_SetCursor(5, 45);
              ssd1306_WriteString("buy ticket", Font_7x10, White);
              ssd1306_UpdateScreen();

              confirmed = 0;

              /* ---------- Countdown Timer with Button Check ---------- */
              /* Visitor has COUNTDOWN_TIME seconds to press the button */
              for (int t = COUNTDOWN_TIME; t >= 0; t--)
              {
                  /* Update countdown display */
                  ssd1306_Fill(Black);
                  ssd1306_SetCursor(5, 5);
                  ssd1306_WriteString("Purchase in:", Font_7x10, White);

                  sprintf(buffer, "%2d sec", t);
                  ssd1306_SetCursor(40, 28);
                  ssd1306_WriteString(buffer, Font_11x18, White);

                  sprintf(buffer, "Visitors: %d/%d", visitor_count, MAX_VISITORS);
                  ssd1306_SetCursor(15, 50);
                  ssd1306_WriteString(buffer, Font_7x10, White);
                  ssd1306_UpdateScreen();

                  /* Check if confirmation button (PB8) is pressed (active low with pull-up) */
                  if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8) == GPIO_PIN_RESET)
                  {
                      confirmed = 1;
                      break;
                  }

                  /* Wait 1 second - Exit gate works independently via TIM6 interrupt */
                  HAL_Delay(1000);
              }

              if (confirmed)
              {
                  /* ========== TICKET CONFIRMED - BEGIN ENTRY SEQUENCE ========== */

                  /* Display ticket purchased message */
                  ssd1306_Fill(Black);
                  ssd1306_SetCursor(10, 10);
                  ssd1306_WriteString("Ticket", Font_11x18, White);
                  ssd1306_SetCursor(10, 35);
                  ssd1306_WriteString("Purchased!", Font_11x18, White);
                  ssd1306_UpdateScreen();

                  HAL_Delay(2000);

                  /* ---------- SERVO 1 (TIM3_CH3): Ticket Turnstile ---------- */
                  ssd1306_Fill(Black);
                  ssd1306_SetCursor(20, 20);
                  ssd1306_WriteString("Please", Font_11x18, White);
                  ssd1306_SetCursor(10, 45);
                  ssd1306_WriteString("proceed...", Font_11x18, White);
                  ssd1306_UpdateScreen();

                  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 1000);
                  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);

                  /* Smooth opening motion: 0° -> 180° (step 2°, 15ms delay) */
                  for (int angolo = 0; angolo <= 180; angolo += 2)
                  {
                      Servo_SetAngle(&htim3, TIM_CHANNEL_3, angolo);
                      HAL_Delay(15);
                  }

                  /* Keep turnstile open for 5 seconds */
                  HAL_Delay(5000);

                  /* Smooth closing motion: 180° -> 0° */
                  for (int angolo = 180; angolo >= 0; angolo -= 2)
                  {
                      Servo_SetAngle(&htim3, TIM_CHANNEL_3, angolo);
                      HAL_Delay(15);
                  }

                  HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3);

                  /* ---------- Wait for Visitor to Pass Through (PIR2 on PB2) ---------- */
                  ssd1306_Fill(Black);
                  ssd1306_SetCursor(20, 20);
                  ssd1306_WriteString("Please", Font_11x18, White);
                  ssd1306_SetCursor(25, 45);
                  ssd1306_WriteString("wait...", Font_11x18, White);
                  ssd1306_UpdateScreen();

                  /* Disable EXTI4 (PIR1) to prevent new transactions during wait */
                  HAL_NVIC_DisableIRQ(EXTI4_IRQn);

                  /* Clear flag and wait for PIR2 interrupt */
                  secondPIR_triggered = 0;
                  while (!secondPIR_triggered)
                  {
                      HAL_Delay(100);
                  }

                  /* ---------- Entry Detected - Open Entry Gate ---------- */
                  /* Green LED ON = Access granted */
                  HAL_GPIO_WritePin(GPIOE, Red_LED_Pin, GPIO_PIN_RESET);
                  HAL_GPIO_WritePin(GPIOE, Blue_LED_Pin, GPIO_PIN_RESET);
                  HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, GPIO_PIN_SET);

                  /* Display access granted message */
                  ssd1306_Fill(Black);
                  ssd1306_SetCursor(30, 10);
                  ssd1306_WriteString("Access", Font_11x18, White);
                  ssd1306_SetCursor(20, 35);
                  ssd1306_WriteString("Granted!", Font_11x18, White);
                  ssd1306_UpdateScreen();

                  HAL_Delay(2000);

                  /* Display gate opening message */
                  ssd1306_Fill(Black);
                  ssd1306_SetCursor(25, 20);
                  ssd1306_WriteString("Entry", Font_11x18, White);
                  ssd1306_SetCursor(15, 45);
                  ssd1306_WriteString("opening...", Font_11x18, White);
                  ssd1306_UpdateScreen();

                  /* ---------- SERVO 2 (TIM2_CH3): Entry Gate ---------- */
                  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 1000);
                  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);

                  /* Smooth opening motion: 0° -> 180° */
                  for (int angolo = 0; angolo <= 180; angolo += 2)
                  {
                      Servo_SetAngle(&htim2, TIM_CHANNEL_3, angolo);
                      HAL_Delay(15);
                  }

                  /* Keep entry gate open for 5 seconds */
                  HAL_Delay(5000);

                  /* Smooth closing motion: 180° -> 0° */
                  for (int angolo = 180; angolo >= 0; angolo -= 2)
                  {
                      Servo_SetAngle(&htim2, TIM_CHANNEL_3, angolo);
                      HAL_Delay(15);
                  }

                  HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_3);

                  /* Increment visitor counter AFTER gate closes */
                  visitor_count++;
                  last_visitor_count = visitor_count;

                  /* Display welcome message with updated counter */
                  ssd1306_Fill(Black);
                  ssd1306_SetCursor(25, 10);
                  ssd1306_WriteString("Enjoy", Font_11x18, White);
                  ssd1306_SetCursor(10, 35);
                  ssd1306_WriteString("your visit!", Font_11x18, White);
                  ssd1306_UpdateScreen();

                  HAL_Delay(2000);

                  /* Re-enable EXTI4 interrupt (PIR1) */
                  HAL_NVIC_EnableIRQ(EXTI4_IRQn);
              }
              else
              {
                  /* ========== TIMEOUT - TICKET NOT PURCHASED ========== */
                  ssd1306_Fill(Black);
                  ssd1306_SetCursor(10, 10);
                  ssd1306_WriteString("Purchase", Font_11x18, White);
                  ssd1306_SetCursor(10, 35);
                  ssd1306_WriteString("Cancelled", Font_11x18, White);
                  ssd1306_UpdateScreen();

                  /* Blue LED ON = Back to idle state */
                  HAL_GPIO_WritePin(GPIOE, Red_LED_Pin, GPIO_PIN_RESET);
                  HAL_GPIO_WritePin(GPIOE, Blue_LED_Pin, GPIO_PIN_SET);
                  HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, GPIO_PIN_RESET);
              }

              /* Wait before returning to idle state */
              HAL_Delay(3000);

              /* ---------- Return to Idle State ---------- */
              Update_Display_Idle(buffer);

              /* Reset LEDs to idle state (Blue ON only) */
              HAL_GPIO_WritePin(GPIOE, Blue_LED_Pin, GPIO_PIN_SET);
              HAL_GPIO_WritePin(GPIOE, Red_LED_Pin, GPIO_PIN_RESET);
              HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, GPIO_PIN_RESET);
          }
      }

      /* ================================================================
       * EXIT GATE HANDLER - Fully independent via TIM6 interrupt
       * ================================================================
       * PIR4 triggers exit gate (Servo3) which is controlled entirely
       * by TIM6 interrupt (every 15ms) - no manual handler needed
       * Display update for visitor count happens automatically below
       * ================================================================ */

      /* Update display only when visitor count changes (after exit) */
      if (visitor_count != last_visitor_count && pirState != GPIO_PIN_SET)
      {
          last_visitor_count = visitor_count;
          Update_Display_Idle(buffer);
      }

      /* Main loop delay - prevents excessive CPU usage during polling */
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_TIM1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
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
    /* Stay here indefinitely - system halted due to error */
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
