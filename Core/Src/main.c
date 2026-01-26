/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// --- NTC parameters ---
#define ADC_MAX_COUNTS   4095.0f     // 12-bit ADC
#define VREF_VOLTS       3.3f        // assuming VDDA ~ 3.3V
#define RFIX_OHMS        10000.0f     // 10k fixed resistor to 3.3V
#define R0_OHMS          10000.0f     // NTC nominal at 25C
#define T0_KELVIN        298.15f      // 25C in Kelvin
#define BETA_K           3950.0f

// --- One-point calibration offsets (milli-°C) ---
// Reference: 37.100 °C
// Measured averages: CH0=36.64 °C, CH1=36.89 °C
#define CAL_OFFS_CH0_mC  (460)   // +0.46 °C
#define CAL_OFFS_CH1_mC  (210)   // +0.21 °C


#define NRF_CE_PORT   GPIOB
#define NRF_CE_PIN    GPIO_PIN_0
#define NRF_CSN_PORT  GPIOB
#define NRF_CSN_PIN   GPIO_PIN_1

static inline void nrf_ce(int on)
{
  HAL_GPIO_WritePin(NRF_CE_PORT, NRF_CE_PIN, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static inline void nrf_csn(int on)
{
  HAL_GPIO_WritePin(NRF_CSN_PORT, NRF_CSN_PIN, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

#define NRF_CMD_R_REGISTER    0x00
#define NRF_CMD_W_REGISTER    0x20
#define NRF_CMD_NOP           0xFF

#define NRF_REG_CONFIG        0x00
#define NRF_REG_RF_CH         0x05
#define NRF_REG_RF_SETUP      0x06
#define NRF_REG_STATUS        0x07


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static int32_t ntc_adc_to_mC(uint16_t adc_counts)
{
  // Convert ADC counts -> voltage at divider midpoint
  float v = ( (float)adc_counts / ADC_MAX_COUNTS ) * VREF_VOLTS;

  // Guard against impossible / saturating values
  if (v <= 0.001f)  return -273150;  // effectively "invalid very cold"
  if (v >= (VREF_VOLTS - 0.001f)) return 999999; // effectively "invalid very hot"

  // Divider: V = Vref * (Rntc / (Rfix + Rntc))
  // => Rntc = Rfix * V / (Vref - V)
  float r_ntc = RFIX_OHMS * v / (VREF_VOLTS - v);

  // Beta formula:
  // 1/T = 1/T0 + (1/B) * ln(R/R0)
  float invT = (1.0f / T0_KELVIN) + (1.0f / BETA_K) * logf(r_ntc / R0_OHMS);
  float tempK = 1.0f / invT;
  float tempC = tempK - 273.15f;

  // milli-Celsius
  int32_t mC = (int32_t)lroundf(tempC * 1000.0f);
  return mC;
}

static int adc_read_one(ADC_HandleTypeDef *hadc, uint32_t channel, uint16_t *out)
{
  ADC_ChannelConfTypeDef sConfig = {0};

  sConfig.Channel = channel;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;

  if (HAL_ADC_ConfigChannel(hadc, &sConfig) != HAL_OK) return -1;
  if (HAL_ADC_Start(hadc) != HAL_OK) return -2;
  if (HAL_ADC_PollForConversion(hadc, 50) != HAL_OK) return -3;

  *out = (uint16_t)HAL_ADC_GetValue(hadc);

  (void)HAL_ADC_Stop(hadc);
  return 0;
}

static int adc_read_two(uint16_t *out0, uint16_t *out1)
{
  int rc;

  rc = adc_read_one(&hadc1, ADC_CHANNEL_0, out0); // PA0
  if (rc) return rc;

  rc = adc_read_one(&hadc1, ADC_CHANNEL_1, out1); // PA1
  if (rc) return rc;

  return 0;
}


static void uart_printf(const char *fmt, ...)
{
  char buf[128];
  va_list ap;
  va_start(ap, fmt);
  int n = vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);

  if (n < 0) return;
  if (n > (int)sizeof(buf)) n = sizeof(buf);

  HAL_UART_Transmit(&huart1, (uint8_t*)buf, (uint16_t)strlen(buf), HAL_MAX_DELAY);
}

static uint8_t nrf_spi_xfer(uint8_t b)
{
  uint8_t rx = 0;
  HAL_SPI_TransmitReceive(&hspi1, &b, &rx, 1, HAL_MAX_DELAY);
  return rx;
}

static uint8_t nrf_read_reg(uint8_t reg)
{
  uint8_t v;
  nrf_csn(0);
  nrf_spi_xfer(NRF_CMD_R_REGISTER | (reg & 0x1F));
  v = nrf_spi_xfer(NRF_CMD_NOP);
  nrf_csn(1);
  return v;
}

static void nrf_write_reg(uint8_t reg, uint8_t v)
{
  nrf_csn(0);
  nrf_spi_xfer(NRF_CMD_W_REGISTER | (reg & 0x1F));
  nrf_spi_xfer(v);
  nrf_csn(1);
}

static void nrf_init_basic(void)
{
  nrf_ce(0);
  nrf_csn(1);
  HAL_Delay(5);

  // Example: set channel
  nrf_write_reg(NRF_REG_RF_CH, 76);

  // RF_SETUP: data rate / power.
  // We'll keep something conservative here first. (Exact bits depend on nRF24L01+)
  // 0x06 is a common "1Mbps, 0dBm" baseline; we'll refine later.
  nrf_write_reg(NRF_REG_RF_SETUP, 0x06);

  // CONFIG: PWR_UP=1 (bit1), PRIM_RX=1 (bit0) => RX mode
  // plus CRC enabled.
  nrf_write_reg(NRF_REG_CONFIG, 0x0F);

  HAL_Delay(2);   // nRF needs ~1.5ms after PWR_UP
  nrf_ce(1);      // start listening (PRX mode)
}


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

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  // ADC calibration (STM32F1)
  (void)HAL_ADCEx_Calibration_Start(&hadc1);

  nrf_ce(0);
  nrf_csn(1);
  HAL_Delay(10);

  uint8_t st = nrf_read_reg(NRF_REG_STATUS);
  uint8_t cfg = nrf_read_reg(NRF_REG_CONFIG);

  uart_printf("NRF STATUS=0x%02X CONFIG=0x%02X\r\n", st, cfg);

  uint8_t ch0 = nrf_read_reg(NRF_REG_RF_CH);
  uart_printf("RF_CH before = %u\r\n", ch0);

  nrf_write_reg(NRF_REG_RF_CH, 76); // channel 76 (2.476 GHz)
  uint8_t ch1 = nrf_read_reg(NRF_REG_RF_CH);
  uart_printf("RF_CH after  = %u\r\n", ch1);


  nrf_init_basic();
  uart_printf("After init: CONFIG=0x%02X RF_CH=%u RF_SETUP=0x%02X\r\n",
              nrf_read_reg(NRF_REG_CONFIG),
              nrf_read_reg(NRF_REG_RF_CH),
              nrf_read_reg(NRF_REG_RF_SETUP));



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	uint16_t a0 = 0, a1 = 0;
	int rc = adc_read_two(&a0, &a1);

	if (rc == 0) {
	  int32_t t0_mC = ntc_adc_to_mC(a0);
	  int32_t t1_mC = ntc_adc_to_mC(a1);

	  // Apply one-point calibration offset (skip if conversion returned "invalid")
	  if (t0_mC > -200000 && t0_mC < 200000) t0_mC += CAL_OFFS_CH0_mC;
	  if (t1_mC > -200000 && t1_mC < 200000) t1_mC += CAL_OFFS_CH1_mC;


	  uart_printf("ADC0=%u ADC1=%u  T0=%ld mC  T1=%ld mC\r\n",
				  (unsigned)a0, (unsigned)a1,
				  (long)t0_mC, (long)t1_mC);
	} else {
	  uart_printf("ADC read error rc=%d\r\n", rc);
	}

	HAL_Delay(1000);

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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);

  /*Configure GPIO pins : PB0 PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
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
