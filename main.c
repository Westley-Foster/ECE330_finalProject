/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body Final Project SP2025
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "seg7.h"

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
I2C_HandleTypeDef hi2c1;
RTC_HandleTypeDef hrtc;
SPI_HandleTypeDef hspi1;
TIM_HandleTypeDef htim7;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM7_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_RTC_Init(void);

/* USER CODE BEGIN PFP */

/* Calendar / RTC helpers */
void calendar_init_config(void);
static uint16_t Read_ADC_Channel(uint8_t channel);
static void Decode_Time_From_TR(uint32_t tr, uint8_t *ph, uint8_t *pm, uint8_t *ps);
static uint32_t Encode_Time_BCD(uint8_t h, uint8_t m, uint8_t s);
static void Decode_Date_From_DR(uint8_t *pMonth, uint8_t *pDay, uint8_t *pYear);

/* Display helpers */
static void Display_Clock_View(void);
static void Display_Alarm_View(void);
static void Handle_UI(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* Original lab globals (for LEDs, music, etc.) */
char ramp = 0;
char RED_BRT = 0;
char GREEN_BRT = 0;
char BLUE_BRT = 0;
char RED_STEP = 1;
char GREEN_STEP = 2;
char BLUE_STEP = 3;
char DIM_Enable = 0;
char Music_ON = 0;
int  TONE = 0;
int  COUNT = 0;
int  INDEX = 0;
int  Note = 0;
int  Save_Note = 0;
int  Vibrato_Depth = 1;
int  Vibrato_Rate  = 40;
int  Vibrato_Count = 0;
char Animate_On = 0;
char Message_Length = 0;
char *Message_Pointer;
char *Save_Pointer;
int  Delay_msec = 0;
int  Delay_counter = 0;

/* HELLO ECE-330L marquee message (still here if you want to use it later) */
char Message[] =
{
  SPACE,SPACE,SPACE,SPACE,SPACE,SPACE,SPACE,SPACE,SPACE,SPACE,
  CHAR_H,CHAR_E,CHAR_L,CHAR_L,CHAR_O,SPACE,CHAR_E,CHAR_C,CHAR_E,DASH,
  CHAR_3,CHAR_3,CHAR_0,CHAR_L,
  SPACE,SPACE,SPACE,SPACE,SPACE,SPACE,SPACE,SPACE,SPACE,SPACE
};

/* Declare array for Song */
Music Song[100];

/* ---------- UI state machine ---------- */
typedef enum {
  UI_CLOCK_VIEW = 0,   /* show time/date alternating */
  UI_CLOCK_SET  = 1,   /* set clock (time + day) using pots */
  UI_ALARM_VIEW = 2,   /* show alarm/time/date alternating */
  UI_ALARM_SET  = 3    /* set alarm time using pots */
} UIState_t;

volatile UIState_t g_uiState = UI_CLOCK_VIEW;

/* Clock view pages: 0 = time, 1 = date */
uint8_t  g_clockPage        = 0;
uint32_t g_lastPageSwitchMs = 0;

/* State for B1 edge detection */
static uint8_t   g_prevB1     = 0;
static UIState_t g_lastState  = UI_CLOCK_VIEW;

/* Values while in SET modes */
static uint8_t g_setHours   = 0;
static uint8_t g_setMinutes = 0;
static uint8_t g_setDay     = 1;

static uint8_t g_alarmHours   = 0;
static uint8_t g_alarmMinutes = 0;
static uint8_t g_alarmSeconds = 0;

/* Alarm fired latch so it only triggers once per match */
static uint8_t g_alarmFired = 0;

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals (CubeMX-generated) */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_TIM7_Init();
  MX_USART3_UART_Init();
  MX_RTC_Init();

  /* USER CODE BEGIN 2 */

  /********************************************************************
   * Direct hardware setup for lab (GPIO, ADC, TIM7, RTC access)
   ********************************************************************/

  /* Allow access to RTC / backup domain */
  PWR->CR |= (1U << 8);   /* DBP = 1 */

  /* Unlock RTC write protection */
  RTC->WPR = 0xCA;
  RTC->WPR = 0x53;

  /*** Configure GPIOs for 7-seg and LEDs ***/
  GPIOD->MODER = 0x55555555;   // Port D all outputs (segment lines, LEDs, etc.)
  GPIOE->MODER = 0x55555555;   // Port E all outputs (digit enables, etc.)
  GPIOC->MODER = 0x00000000;   // Port C all inputs
  GPIOE->ODR   = 0xFFFF;       // Set all Port E pins high initially

  /* Make PA1, PA2, PA3 analog for the potentiometers (leave PA0/B1 alone) */
  GPIOA->MODER |= (3U << (1 * 2))   // PA1 analog
                | (3U << (2 * 2))   // PA2 analog
                | (3U << (3 * 2));  // PA3 analog

  /*** Configure ADC1 (for PA1, PA2, PA3) ***/
  RCC->APB2ENR |= (1U << 8);   // Enable ADC1 clock
  ADC1->SMPR2  |= 1U;          // 15 clock cycles per sample
  ADC1->CR2    |= 1U;          // Turn on ADC1

  /*** Configure TIM7 (used by audio / PWM logic) ***/
  TIM7->PSC  = 199;   // 50 MHz / 200 = 250 kHz
  TIM7->ARR  = 1;     // 250 kHz / 2 = 125 kHz ISR for PWM counter
  TIM7->DIER |= 1U;   // Enable TIM7 interrupt
  TIM7->CR1  |= 1U;   // Start TIM7

  /* Jeopardy Song (unchanged) */
  Song[0].note = A4;      Song[0].size = quarter; Song[0].tempo = 1400; Song[0].space = 10;  Song[0].end = 0;
  Song[1].note = D5;      Song[1].size = quarter; Song[1].tempo = 1400; Song[1].space = 10;  Song[1].end = 0;
  Song[2].note = A4;      Song[2].size = quarter; Song[2].tempo = 1400; Song[2].space = 10;  Song[2].end = 0;
  Song[3].note = D4;      Song[3].size = quarter; Song[3].tempo = 1400; Song[3].space = 10;  Song[3].end = 0;
  Song[4].note = A4;      Song[4].size = quarter; Song[4].tempo = 1400; Song[4].space = 10;  Song[4].end = 0;
  Song[5].note = D5;      Song[5].size = quarter; Song[5].tempo = 1400; Song[5].space = 10;  Song[5].end = 0;
  Song[6].note = A4;      Song[6].size = quarter; Song[6].tempo = 1400; Song[6].space = 10;  Song[6].end = 0;
  Song[7].note = rest;    Song[7].size = quarter; Song[7].tempo = 1400; Song[7].space = 10;  Song[7].end = 0;
  Song[8].note = A4;      Song[8].size = quarter; Song[8].tempo = 1400; Song[8].space = 10;  Song[8].end = 0;
  Song[9].note = D5;      Song[9].size = quarter; Song[9].tempo = 1400; Song[9].space = 10;  Song[9].end = 0;
  Song[10].note = A4;     Song[10].size = quarter; Song[10].tempo = 1400; Song[10].space = 10; Song[10].end = 0;
  Song[11].note = D5;     Song[11].size = quarter; Song[11].tempo = 1400; Song[11].space = 10; Song[11].end = 0;
  Song[12].note = Fs5_Gb5;Song[12].size = quarter; Song[12].tempo = 1400; Song[12].space = 100;Song[12].end = 0;
  Song[13].note = rest;   Song[13].size = _8th;   Song[13].tempo = 1400; Song[13].space = 10; Song[13].end = 0;
  Song[14].note = E5;     Song[14].size = _8th;   Song[14].tempo = 1400; Song[14].space = 10; Song[14].end = 0;
  Song[15].note = D5;     Song[15].size = _8th;   Song[15].tempo = 1400; Song[15].space = 10; Song[15].end = 0;
  Song[16].note = Cs5_Db5;Song[16].size = _8th;   Song[16].tempo = 1400; Song[16].space = 10; Song[16].end = 0;
  Song[17].note = B4;     Song[17].size = _8th;   Song[17].tempo = 1400; Song[17].space = 10; Song[17].end = 0;
  Song[18].note = As4_Bb4;Song[18].size = _8th;   Song[18].tempo = 1400; Song[18].space = 10; Song[18].end = 0;
  Song[19].note = A4;     Song[19].size = quarter;Song[19].tempo = 1400; Song[19].space = 10; Song[19].end = 0;
  Song[20].note = D5;     Song[20].size = quarter;Song[20].tempo = 1400; Song[20].space = 10; Song[20].end = 0;
  Song[21].note = A4;     Song[21].size = quarter;Song[21].tempo = 1400; Song[21].space = 10; Song[21].end = 0;
  Song[22].note = Fs4_Gb4;Song[22].size = _8th;   Song[22].tempo = 1400; Song[22].space = 10; Song[22].end = 0;
  Song[23].note = G4;     Song[23].size = _8th;   Song[23].tempo = 1400; Song[23].space = 10; Song[23].end = 0;
  Song[24].note = A4;     Song[24].size = quarter;Song[24].tempo = 1400; Song[24].space = 10; Song[24].end = 0;
  Song[25].note = D5;     Song[25].size = quarter;Song[25].tempo = 1400; Song[25].space = 10; Song[25].end = 0;
  Song[26].note = A4;     Song[26].size = quarter;Song[26].tempo = 1400; Song[26].space = 10; Song[26].end = 0;
  Song[27].note = rest;   Song[27].size = quarter;Song[27].tempo = 1400; Song[27].space = 10; Song[27].end = 0;
  Song[28].note = D5;     Song[28].size = quarter;Song[28].tempo = 1400; Song[28].space = 100;Song[28].end = 0;
  Song[29].note = rest;   Song[29].size = _8th;   Song[29].tempo = 1400; Song[29].space = 10; Song[29].end = 0;
  Song[30].note = B4;     Song[30].size = _8th;   Song[30].tempo = 1400; Song[30].space = 10; Song[30].end = 0;
  Song[31].note = A4;     Song[31].size = quarter;Song[31].tempo = 1400; Song[31].space = 100;Song[31].end = 0;
  Song[32].note = G4;     Song[32].size = quarter;Song[32].tempo = 1400; Song[32].space = 100;Song[32].end = 0;
  Song[33].note = Fs4_Gb4;Song[33].size = quarter;Song[33].tempo = 1400; Song[33].space = 100;Song[33].end = 0;
  Song[34].note = E4;     Song[34].size = quarter;Song[34].tempo = 1400; Song[34].space = 100;Song[34].end = 0;
  Song[35].note = D4;     Song[35].size = quarter;Song[35].tempo = 1400; Song[35].space = 100;Song[35].end = 0;
  Song[36].note = rest;   Song[36].size = quarter;Song[36].tempo = 1400; Song[36].space = 10; Song[36].end = 0;
  Song[37].note = C5;     Song[37].size = quarter;Song[37].tempo = 1400; Song[37].space = 10; Song[37].end = 0;
  Song[38].note = F5;     Song[38].size = quarter;Song[38].tempo = 1400; Song[38].space = 10; Song[38].end = 0;
  Song[39].note = C5;     Song[39].size = quarter;Song[39].tempo = 1400; Song[39].space = 10; Song[39].end = 0;
  Song[40].note = F4;     Song[40].size = _8th;   Song[40].tempo = 1400; Song[40].space = 10; Song[40].end = 0;
  Song[41].note = F4;     Song[41].size = _8th;   Song[41].tempo = 1400; Song[41].space = 10; Song[41].end = 0;
  Song[42].note = C5;     Song[42].size = quarter;Song[42].tempo = 1400; Song[42].space = 10; Song[42].end = 0;
  Song[43].note = F5;     Song[43].size = quarter;Song[43].tempo = 1400; Song[43].space = 10; Song[43].end = 0;
  Song[44].note = C5;     Song[44].size = quarter;Song[44].tempo = 1400; Song[44].space = 10; Song[44].end = 0;
  Song[45].note = rest;   Song[45].size = quarter;Song[45].tempo = 1400; Song[45].space = 10; Song[45].end = 0;
  Song[46].note = C5;     Song[46].size = quarter;Song[46].tempo = 1400; Song[46].space = 10; Song[46].end = 0;
  Song[47].note = F5;     Song[47].size = quarter;Song[47].tempo = 1400; Song[47].space = 10; Song[47].end = 0;
  Song[48].note = C5;     Song[48].size = quarter;Song[48].tempo = 1400; Song[48].space = 10; Song[48].end = 0;
  Song[49].note = F5;     Song[49].size = quarter;Song[49].tempo = 1400; Song[49].space = 10; Song[49].end = 0;
  Song[50].note = A5;     Song[50].size = quarter;Song[50].tempo = 1400; Song[50].space = 0;  Song[50].end = 0;
  Song[51].note = A5;     Song[51].size = _8th;   Song[51].tempo = 1400; Song[51].space = 10; Song[51].end = 0;
  Song[52].note = G5;     Song[52].size = _8th;   Song[52].tempo = 1400; Song[52].space = 10; Song[52].end = 0;
  Song[53].note = F5;     Song[53].size = _8th;   Song[53].tempo = 1400; Song[53].space = 10; Song[53].end = 0;
  Song[54].note = E5;     Song[54].size = _8th;   Song[54].tempo = 1400; Song[54].space = 10; Song[54].end = 0;
  Song[55].note = D5;     Song[55].size = _8th;   Song[55].tempo = 1400; Song[55].space = 10; Song[55].end = 0;
  Song[56].note = Cs5_Db5;Song[56].size = _8th;   Song[56].tempo = 1400; Song[56].space = 10; Song[56].end = 0;
  Song[57].note = C5;     Song[57].size = quarter;Song[57].tempo = 1400; Song[57].space = 10; Song[57].end = 0;
  Song[58].note = F5;     Song[58].size = quarter;Song[58].tempo = 1400; Song[58].space = 10; Song[58].end = 0;
  Song[59].note = C5;     Song[59].size = quarter;Song[59].tempo = 1400; Song[59].space = 10; Song[59].end = 0;
  Song[60].note = A4;     Song[60].size = _8th;   Song[60].tempo = 1400; Song[60].space = 10; Song[60].end = 0;
  Song[61].note = As4_Bb4;Song[61].size = _8th;   Song[61].tempo = 1400; Song[61].space = 10; Song[61].end = 0;
  Song[62].note = C5;     Song[62].size = quarter;Song[62].tempo = 1400; Song[62].space = 10; Song[62].end = 0;
  Song[63].note = F5;     Song[63].size = quarter;Song[63].tempo = 1400; Song[63].space = 10; Song[63].end = 0;
  Song[64].note = C5;     Song[64].size = quarter;Song[64].tempo = 1400; Song[64].space = 10; Song[64].end = 0;
  Song[65].note = rest;   Song[65].size = _16th;  Song[65].tempo = 1400; Song[65].space = 10; Song[65].end = 0;
  Song[66].note = C5;     Song[66].size = _16th;  Song[66].tempo = 1400; Song[66].space = 10; Song[66].end = 0;
  Song[67].note = D5;     Song[67].size = _16th;  Song[67].tempo = 1400; Song[67].space = 10; Song[67].end = 0;
  Song[68].note = E5;     Song[68].size = _16th;  Song[68].tempo = 1400; Song[68].space = 10; Song[68].end = 0;
  Song[69].note = F5;     Song[69].size = quarter;Song[69].tempo = 1400; Song[69].space = 100;Song[69].end = 0;
  Song[70].note = rest;   Song[70].size = _8th;   Song[70].tempo = 1400; Song[70].space = 10; Song[70].end = 0;
  Song[71].note = D5;     Song[71].size = _8th;   Song[71].tempo = 1400; Song[71].space = 10; Song[71].end = 0;
  Song[72].note = C5;     Song[72].size = quarter;Song[72].tempo = 1400; Song[72].space = 100;Song[72].end = 0;
  Song[73].note = As4_Bb4;Song[73].size = quarter;Song[73].tempo = 1400; Song[73].space = 100;Song[73].end = 0;
  Song[74].note = A4;     Song[74].size = quarter;Song[74].tempo = 1400; Song[74].space = 100;Song[74].end = 0;
  Song[75].note = rest;   Song[75].size = quarter;Song[75].tempo = 1400; Song[75].space = 100;Song[75].end = 0;
  Song[76].note = G4;     Song[76].size = quarter;Song[76].tempo = 1400; Song[76].space = 100;Song[76].end = 0;
  Song[77].note = rest;   Song[77].size = quarter;Song[77].tempo = 1400; Song[77].space = 100;Song[77].end = 0;
  Song[78].note = F4;     Song[78].size = quarter;Song[78].tempo = 1400; Song[78].space = 100;Song[78].end = 0;

  Song[99].note = rest;   Song[99].size = quarter;Song[99].tempo = 1400; Song[99].space = 10; Song[99].end = 1;

  Save_Note = Song[0].note;  // Needed for vibrato effect
  INDEX     = 0;
  Music_ON  = 0;

  /* Initialize marquee pointers (not strictly used by this UI, but kept) */
  Message_Pointer = &Message[0];
  Save_Pointer    = &Message[0];
  Message_Length  = sizeof(Message)/sizeof(Message[0]);
  Delay_msec      = 200;
  Animate_On      = 0;

  /* Initialize RTC time/date to a known value (24-hour mode) */
  calendar_init_config();

  g_uiState          = UI_CLOCK_VIEW;
  g_lastState        = UI_CLOCK_VIEW;
  g_clockPage        = 0;
  g_lastPageSwitchMs = HAL_GetTick();
  g_alarmFired       = 0;

  /* TEMP TEST: force music at startup to verify audio path.
   * Once you confirm sound works, you can comment these two lines out.
   */
  // INDEX    = 0;
  // Music_ON = 1;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    Handle_UI();
    /* USER CODE END 3 */
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

  /** Configure the main internal regulator output voltage
  */

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState       = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState       = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM       = 8;
  RCC_OscInitStruct.PLL.PLLN       = 336;
  RCC_OscInitStruct.PLL.PLLP       = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ       = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                   |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{
  /* USER CODE BEGIN I2C1_Init 0 */
  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */
  /* USER CODE END I2C1_Init 1 */

  hi2c1.Instance             = I2C1;
  hi2c1.Init.ClockSpeed      = 100000;
  hi2c1.Init.DutyCycle       = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1     = 0;
  hi2c1.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2     = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN I2C1_Init 2 */
  /* USER CODE END I2C1_Init 2 */
}

/**
  * @brief RTC Initialization Function (HAL)
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{
  RTC_TimeTypeDef  sTime  = {0};
  RTC_DateTypeDef  sDate  = {0};
  RTC_AlarmTypeDef sAlarm = {0};

  hrtc.Instance            = RTC;
  hrtc.Init.HourFormat     = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv   = 127;
  hrtc.Init.SynchPrediv    = 255;
  hrtc.Init.OutPut         = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType     = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* Set an initial time/date (HAL-level; will be overridden by calendar_init_config) */
  sTime.Hours          = 0x00;
  sTime.Minutes        = 0x00;
  sTime.Seconds        = 0x00;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }

  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month   = RTC_MONTH_JANUARY;
  sDate.Date    = 0x1;
  sDate.Year    = 0x0;
  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }

  /* Configure Alarm A (will be reconfigured by UI) */
  sAlarm.AlarmTime.Hours          = 0x0;
  sAlarm.AlarmTime.Minutes        = 0x0;
  sAlarm.AlarmTime.Seconds        = 0x0;
  sAlarm.AlarmTime.SubSeconds     = 0x0;
  sAlarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sAlarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
  sAlarm.AlarmMask                = RTC_ALARMMASK_NONE;
  sAlarm.AlarmSubSecondMask       = RTC_ALARMSUBSECONDMASK_ALL;
  sAlarm.AlarmDateWeekDaySel      = RTC_ALARMDATEWEEKDAYSEL_DATE;
  sAlarm.AlarmDateWeekDay         = 0x1;
  sAlarm.Alarm                    = RTC_ALARM_A;
  if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{
  /* USER CODE BEGIN SPI1_Init 0 */
  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */
  /* USER CODE END SPI1_Init 1 */

  hspi1.Instance               = SPI1;
  hspi1.Init.Mode              = SPI_MODE_MASTER;
  hspi1.Init.Direction         = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize          = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity       = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase          = SPI_PHASE_1EDGE;
  hspi1.Init.NSS               = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit          = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode            = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial     = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN SPI1_Init 2 */
  /* USER CODE END SPI1_Init 2 */
}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{
  /* USER CODE BEGIN TIM7_Init 0 */
  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */
  /* USER CODE END TIM7_Init 1 */

  htim7.Instance               = TIM7;
  htim7.Init.Prescaler         = 0;
  htim7.Init.CounterMode       = TIM_COUNTERMODE_UP;
  htim7.Init.Period            = 65535;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN TIM7_Init 2 */
  /* USER CODE END TIM7_Init 2 */
}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{
  /* USER CODE BEGIN USART3_Init 0 */
  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */
  /* USER CODE END USART3_Init 1 */

  huart3.Instance          = USART3;
  huart3.Init.BaudRate     = 115200;
  huart3.Init.WordLength   = UART_WORDLENGTH_8B;
  huart3.Init.StopBits     = UART_STOPBITS_1;
  huart3.Init.Parity       = UART_PARITY_NONE;
  huart3.Init.Mode         = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN USART3_Init 2 */
  /* USER CODE END USART3_Init 2 */
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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : CS_I2C_SPI_Pin */
  GPIO_InitStruct.Pin   = CS_I2C_SPI_Pin;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin   = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PDM_OUT_Pin */
  GPIO_InitStruct.Pin       = PDM_OUT_Pin;
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull      = GPIO_NOPULL;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PDM_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin  = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;      /* simple input so we can poll */
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : I2S3_WS_Pin */
  GPIO_InitStruct.Pin       = I2S3_WS_Pin;
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull      = GPIO_NOPULL;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(I2S3_WS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin  = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CLK_IN_Pin */
  GPIO_InitStruct.Pin       = CLK_IN_Pin;
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull      = GPIO_NOPULL;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(CLK_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin Audio_RST_Pin */
  GPIO_InitStruct.Pin   = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin|Audio_RST_Pin;
  GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull  = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : I2S3_MCK_Pin I2S3_SD_Pin */
  GPIO_InitStruct.Pin       = I2S3_MCK_Pin|I2S3_SD_Pin;
  GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull      = GPIO_NOPULL;
  GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin  = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MEMS_INT2_Pin */
  GPIO_InitStruct.Pin  = MEMS_INT2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MEMS_INT2_GPIO_Port, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */

/* ----------------- RTC / Calendar helpers ----------------- */

/* Initialize time/date in RTC using direct register writes */
void calendar_init_config(void)
{
  /* Enter initialization mode */
  RTC->ISR |= RTC_ISR_INIT;
  while (!(RTC->ISR & RTC_ISR_INITF)) {
    /* wait for INITF */
  }

  /* Set initial time: 17:35:00 (24-hour) */
  RTC->TR =
      (0x1U << 20) |  /* hour tens = 1 -> 10 */
      (0x7U << 16) |  /* hour units = 7 -> 7  => 17h */
      (0x3U << 12) |  /* minute tens = 3 -> 30 */
      (0x5U <<  8) |  /* minute units = 5 -> 5  => 35m */
      (0x0U <<  4) |  /* second tens = 0 */
      (0x0U);         /* second units = 0      => 00s */

  /* Set date: 21 November 2025 (YYMMDD = 25 11 21) */
  RTC->DR =
      (0x2U << 20) |  /* year tens = 2 -> 20 */
      (0x5U << 16) |  /* year units = 5 -> 5  => 25 */
      (0x1U << 12) |  /* month tens = 1 -> 10 */
      (0x1U <<  8) |  /* month units = 1 -> 1  => 11 */
      (0x2U <<  4) |  /* day tens   = 2 -> 20 */
      (0x1U);         /* day units  = 1 -> 1  => 21 */

  /* 24-hour format */
  RTC->CR &= ~RTC_CR_FMT;

  /* Exit init mode */
  RTC->ISR &= ~RTC_ISR_INIT;

  /* Re-sync shadow registers */
  RTC->ISR &= ~RTC_ISR_RSF;
  while (!(RTC->ISR & RTC_ISR_RSF)) {
    /* wait */
  }
}

/* Read ADC1 single conversion on given channel (0..15) */
static uint16_t Read_ADC_Channel(uint8_t channel)
{
  ADC1->SQR3 = (channel & 0x1FU);  /* select channel */
  ADC1->CR2 |= ADC_CR2_SWSTART;    /* start conversion */

  while (!(ADC1->SR & ADC_SR_EOC)) {
    /* wait */
  }
  return (uint16_t)ADC1->DR;
}

/* Decode BCD time from RTC->TR into h/m/s */
static void Decode_Time_From_TR(uint32_t tr, uint8_t *ph, uint8_t *pm, uint8_t *ps)
{
  uint8_t ht = (tr >> 20) & 0x3U;
  uint8_t hu = (tr >> 16) & 0xFU;
  uint8_t mt = (tr >> 12) & 0x7U;
  uint8_t mu = (tr >>  8) & 0xFU;
  uint8_t st = (tr >>  4) & 0x7U;
  uint8_t su = (tr      ) & 0xFU;

  uint8_t h = (uint8_t)(ht * 10U + hu);
  uint8_t m = (uint8_t)(mt * 10U + mu);
  uint8_t s = (uint8_t)(st * 10U + su);

  if (ph) *ph = h;
  if (pm) *pm = m;
  if (ps) *ps = s;
}

/* Encode (h,m,s) into TR-style BCD so Seven_Segment() can show 00HHMMSS */
static uint32_t Encode_Time_BCD(uint8_t h, uint8_t m, uint8_t s)
{
  if (h > 23U) h = 23U;
  if (m > 59U) m = 59U;
  if (s > 59U) s = 59U;

  uint8_t ht = (h / 10U) & 0x3U;
  uint8_t hu = (h % 10U) & 0xFU;
  uint8_t mt = (m / 10U) & 0x7U;
  uint8_t mu = (m % 10U) & 0xFU;
  uint8_t st = (s / 10U) & 0x7U;
  uint8_t su = (s % 10U) & 0xFU;

  uint32_t tr = 0;
  tr |= ((uint32_t)ht << 20);
  tr |= ((uint32_t)hu << 16);
  tr |= ((uint32_t)mt << 12);
  tr |= ((uint32_t)mu <<  8);
  tr |= ((uint32_t)st <<  4);
  tr |= ((uint32_t)su);
  return tr;
}

/* Decode date from RTC->DR into month/day/year (0–99) */
static void Decode_Date_From_DR(uint8_t *pMonth, uint8_t *pDay, uint8_t *pYear)
{
  uint32_t dr = RTC->DR;

  uint8_t yt = (dr >> 20) & 0xFU;
  uint8_t yu = (dr >> 16) & 0xFU;
  uint8_t mt = (dr >> 12) & 0x1U;
  uint8_t mu = (dr >>  8) & 0xFU;
  uint8_t dt = (dr >>  4) & 0x3U;
  uint8_t du = (dr      ) & 0xFU;

  uint8_t year  = (uint8_t)(yt * 10U + yu);
  uint8_t month = (uint8_t)(mt * 10U + mu);
  uint8_t day   = (uint8_t)(dt * 10U + du);

  if (pMonth) *pMonth = month;
  if (pDay)   *pDay   = day;
  if (pYear)  *pYear  = year;
}

/* ----------------- Display helpers ----------------- */

/* TIME / DATE alternating display on main clock screen */
static void Display_Clock_View(void)
{
  uint32_t now = HAL_GetTick();

  /* Toggle between time and date every 2000 ms */
  if ((now - g_lastPageSwitchMs) >= 2000U)
  {
    g_clockPage ^= 1U;
    g_lastPageSwitchMs = now;
  }

  if (g_clockPage == 0U)
  {
    /* TIME page: directly show RTC->TR (00HHMMSS) */
    Seven_Segment(RTC->TR);
  }
  else
  {
    /* DATE page: show MMDDYY encoded into HHMMSS slots */
    uint8_t month, day, year;
    Decode_Date_From_DR(&month, &day, &year);

    uint32_t encoded = Encode_Time_BCD(month, day, year);
    Seven_Segment(encoded);
  }
}

/* Alarm screen: alternate RTC time, date, and alarm time */
static void Display_Alarm_View(void)
{
  static uint8_t  page   = 0;   /* 0 = RTC time, 1 = date, 2 = alarm */
  static uint32_t lastMs = 0;
  uint32_t now = HAL_GetTick();

  if ((now - lastMs) >= 2000U)
  {
    page = (page + 1U) % 3U;
    lastMs = now;
  }

  if (page == 0U)
  {
    /* Live RTC time */
    Seven_Segment(RTC->TR);
  }
  else if (page == 1U)
  {
    /* Date as MMDDYY */
    uint8_t month, day, year;
    Decode_Date_From_DR(&month, &day, &year);
    uint32_t encoded = Encode_Time_BCD(month, day, year);
    Seven_Segment(encoded);
  }
  else
  {
    /* Alarm time from ALRMAR (encoded like TR: HHMMSS) */
    Seven_Segment(RTC->ALRMAR);
  }
}

/* Commit clock settings (g_setHours, g_setMinutes, g_setDay) into RTC */
static void Commit_Clock_Settings(void)
{
  /* Enter init mode */
  RTC->ISR |= RTC_ISR_INIT;
  while (!(RTC->ISR & RTC_ISR_INITF)) {
    /* wait */
  }

  /* Write new time (seconds = 0) */
  uint32_t newTR = Encode_Time_BCD(g_setHours, g_setMinutes, 0U);
  RTC->TR = newTR;

  /* Keep year/month, update day only */
  uint32_t dr = RTC->DR;
  uint8_t yt = (dr >> 20) & 0xFU;
  uint8_t yu = (dr >> 16) & 0xFU;
  uint8_t mt = (dr >> 12) & 0x1U;
  uint8_t mu = (dr >>  8) & 0xFU;

  if (g_setDay < 1U)  g_setDay = 1U;
  if (g_setDay > 31U) g_setDay = 31U;

  uint8_t dt = (g_setDay / 10U) & 0x3U;
  uint8_t du = (g_setDay % 10U) & 0xFU;

  uint32_t newDR = 0;
  newDR |= ((uint32_t)yt << 20);
  newDR |= ((uint32_t)yu << 16);
  newDR |= ((uint32_t)mt << 12);
  newDR |= ((uint32_t)mu <<  8);
  newDR |= ((uint32_t)dt <<  4);
  newDR |= ((uint32_t)du);
  RTC->DR = newDR;

  RTC->CR  &= ~RTC_CR_FMT;   /* 24-h format */
  RTC->ISR &= ~RTC_ISR_INIT; /* Exit init mode */

  /* Re-sync */
  RTC->ISR &= ~RTC_ISR_RSF;
  while (!(RTC->ISR & RTC_ISR_RSF)) {
    /* wait */
  }
}

/* Commit alarm settings (g_alarmHours, g_alarmMinutes, g_alarmSeconds) */
static void Commit_Alarm_Settings(void)
{
  /* Disable Alarm A while updating */
  RTC->CR &= ~RTC_CR_ALRAE;
  while (!(RTC->ISR & RTC_ISR_ALRAWF)) {
    /* wait for alarm write flag */
  }

  /* Encode full HH:MM:SS for the alarm (TR-style BCD) */
  uint32_t encodedTime = Encode_Time_BCD(g_alarmHours, g_alarmMinutes, g_alarmSeconds);

  /*
   * Build ALRMAR:
   *  - low 24 bits: HHMMSS from encodedTime
   *  - bit 31 (MSK4) = 1 to IGNORE DATE
   *  -> alarm will trigger any day when HH:MM:SS match
   */
  uint32_t alrmar = 0;
  alrmar |= (encodedTime & 0x00FFFFFFU);  /* copy time digits */
  alrmar |= RTC_ALRMAR_MSK4;             /* mask date/weekday compare */

  RTC->ALRMAR = alrmar;

  /* Enable Alarm A and its interrupt flag */
  RTC->CR |= RTC_CR_ALRAE | RTC_CR_ALRAIE;

  /* Clear any previous alarm flag */
  RTC->ISR &= ~RTC_ISR_ALRAF;

  /* Reset software latch so next match will fire */
  g_alarmFired = 0;
}

/* ----------------- Main UI handler (B1 + pots) ----------------- */

static void Handle_UI(void)
{
  /* Read blue user button B1 (PA0) */
  uint8_t b1 = HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) ? 1U : 0U;

  /* Rising edge on B1: advance state and commit any pending SET values */
  if (b1 && !g_prevB1)
  {
    /* Commit settings when leaving a SET state */
    if (g_uiState == UI_CLOCK_SET)
    {
      Commit_Clock_Settings();
    }
    else if (g_uiState == UI_ALARM_SET)
    {
      Commit_Alarm_Settings();
    }

    /* Advance state: CLOCK_VIEW -> CLOCK_SET -> ALARM_VIEW -> ALARM_SET -> CLOCK_VIEW ... */
    switch (g_uiState)
    {
      case UI_CLOCK_VIEW: g_uiState = UI_CLOCK_SET;  break;
      case UI_CLOCK_SET:  g_uiState = UI_ALARM_VIEW; break;
      case UI_ALARM_VIEW: g_uiState = UI_ALARM_SET;  break;
      default:            g_uiState = UI_CLOCK_VIEW; break;
    }
    g_lastState = g_uiState;
  }
  g_prevB1 = b1;

  /* Handle current state */
  if (g_uiState == UI_CLOCK_VIEW)
  {
    /* Normal clock: alternate time/date every ~2s */
    Display_Clock_View();
  }
  else if (g_uiState == UI_CLOCK_SET)
  {
    /* CLOCK SET MODE
     *   PA1 (ADC1_IN1) -> Hours  (0–23)
     *   PA2 (ADC1_IN2) -> Minutes(0–59)
     *   PA3 (ADC1_IN3) -> Day    (1–31)
     * Display shows HHMM00 while adjusting.
     */

    uint16_t adcH = Read_ADC_Channel(1U);   /* PA1 */
    uint16_t adcM = Read_ADC_Channel(2U);   /* PA2 */
    uint16_t adcD = Read_ADC_Channel(3U);   /* PA3 */

    g_setHours   = (uint8_t)((adcH * 24U) / 4096U);
    g_setMinutes = (uint8_t)((adcM * 60U) / 4096U);
    g_setDay     = (uint8_t)((adcD * 31U) / 4096U) + 1U;
    if (g_setDay > 31U) g_setDay = 31U;

    uint32_t preview = Encode_Time_BCD(g_setHours, g_setMinutes, 0U);
    Seven_Segment(preview);
  }
  else if (g_uiState == UI_ALARM_VIEW)
  {
    /* Alarm screen: cycles RTC -> date -> alarm time */
    Display_Alarm_View();
  }
  else /* UI_ALARM_SET */
  {
    /* ALARM SET MODE
     *   PA1 (ADC1_IN1) -> Alarm hours   (0–23)
     *   PA2 (ADC1_IN2) -> Alarm minutes (0–59)
     *   PA3 (ADC1_IN3) -> Alarm seconds (0–59)
     */

    uint16_t adcH = Read_ADC_Channel(1U);  /* PA1 */
    uint16_t adcM = Read_ADC_Channel(2U);  /* PA2 */
    uint16_t adcS = Read_ADC_Channel(3U);  /* PA3 */

    g_alarmHours   = (uint8_t)((adcH * 24U) / 4096U);
    g_alarmMinutes = (uint8_t)((adcM * 60U) / 4096U);
    g_alarmSeconds = (uint8_t)((adcS * 60U) / 4096U);

    uint32_t preview = Encode_Time_BCD(g_alarmHours, g_alarmMinutes, g_alarmSeconds);
    Seven_Segment(preview);
  }

  /* Alarm handling (software compare, active in any state):
   * Compare low 24 bits (HHMMSS) of TR and ALRMAR.
   */
  uint32_t nowTR   = RTC->TR    & 0x00FFFFFFU;
  uint32_t alarmTR = RTC->ALRMAR & 0x00FFFFFFU;

  if ((RTC->CR & RTC_CR_ALRAE) && !g_alarmFired && (nowTR == alarmTR))
  {
    g_alarmFired = 1;

    /* Turn on Jeopardy */
    INDEX    = 0;
    Music_ON = 1;

    /* Turn on LD5 as visual indicator */
    HAL_GPIO_WritePin(GPIOD, LD5_Pin, GPIO_PIN_SET);
  }

  /* Re-arm once time moves away from alarm time */
  if (nowTR != alarmTR)
  {
    g_alarmFired = 0;
    /* Optionally turn off LD5 when no longer matching */
    HAL_GPIO_WritePin(GPIOD, LD5_Pin, GPIO_PIN_RESET);
  }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
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
  (void)file;
  (void)line;
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
