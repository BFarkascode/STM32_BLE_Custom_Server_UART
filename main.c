/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
  ******************************************************************************
  ******************************************************************************
  ******************************************************************************
  *
  * v1 Basic implementation of SSD1315 driver
  *
  * v2 Advanced implementation of SSD1315 driver
  * 	Practical difference between basic and advanced is minimal
  * 	Added BMP280 readout
  * 	Send temperature measurement to screen
  *
  * v3 Added BLE
  * 	Replaced blocking I2C functions and delay to not corrupt BLE
  * 	BLE still refuses to play well with the I2C
  * 	Removed I2C and generated only a simple UART pipe sent to the screen (for now)
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "MyBitmap.h"
#include "SPI.h"
#include "driver_ssd1315_advance.h"
#include "image.h"
#include "stdio.h"
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

IPCC_HandleTypeDef hipcc;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_IPCC_Init(void);
static void MX_RTC_Init(void);
static void MX_RF_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//BMP280 temperature compensation calculation for fixed point results
/*int32_t compensate_temperature(int32_t sensor_adc_readout, uint16_t dig_T1, int16_t dig_T2, int16_t dig_T3) {

	int32_t var1, var2;

	var1 = ((((sensor_adc_readout >> 3) - (dig_T1 << 1)))	* dig_T2) >> 11;
	var2 = (((((sensor_adc_readout >> 4) - dig_T1) * ((sensor_adc_readout >> 4) - dig_T1)) >> 12) * dig_T3) >> 14;

	return ((var1 + var2) * 5 + 128) >> 8;
}

int32_t temperature = 0;

uint8_t scan_reply;																			//used for the scan loop

enum_Yes_No_Selector scanning_bus;
enum_Yes_No_Selector Tx_finished;
enum_Yes_No_Selector Rx_finished;

uint8_t Tx_number_of_bytes;
uint8_t* bytes_to_send_ptr;
uint8_t* bytes_received_ptr;

uint8_t buf[1];

//read BMP280 sensor
const uint8_t T_adc[3] = {0xFA, 0xFB, 0xFC};											//this is where the ADC temp values will go
static uint8_t T_out[3] = {0x0, 0x0, 0x0};

//We rebuild the parameters from the readout
//these values are coming from the T_comp readout, see I2C project. We bypass the readout since they are constant values anyway.
const uint16_t dig_T1 = (0x6b << 8) | 0x36;
const int16_t dig_T2 = (0x65 << 8) | 0x7d;
const int16_t dig_T3 = (0x0 << 8) | 0x8c;*/

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
  /* Config code for STM32_WPAN (HSE Tuning must be done before system clock configuration) */
  MX_APPE_Config();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

/* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* IPCC initialisation */
  MX_IPCC_Init();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
//  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_RTC_Init();
  MX_RF_Init();
  /* USER CODE BEGIN 2 */

  uint8_t res;

//initiate screen
  res = ssd1315_advance_init(SSD1315_INTERFACE_SPI, SSD1315_ADDR_SA0_0);
  if (res != 0)
  {
      return 1;
  }

//this clears the screen
  res = ssd1315_advance_clear();
  if (res != 0)
  {
      ssd1315_interface_debug_print("ssd1315: clear screen failed.\n");
      (void)ssd1315_advance_deinit();

      return 1;
  }

//this draws a string on the screen
  res = ssd1315_advance_string(0, 0, "Hello world", 11, 1, SSD1315_FONT_16);
  if (res != 0)
  {
      ssd1315_interface_debug_print("ssd1315: show string failed.\n");
      (void)ssd1315_advance_deinit();

      return 1;
  }

  HAL_Delay(1000);
//this draws a point
#ifdef draw_point
  res = ssd1315_advance_write_point(38, 38, 1);
  if (res != 0)
  {
      (void)ssd1315_advance_deinit();

      return 1;
  }
#endif

//this draw a rectangle
#ifdef draw_rectangle
  res = ssd1315_advance_rect(0, 0, 31, 31, 1);
  if (res != 0)
  {
      (void)ssd1315_advance_deinit();

      return 1;
  }
#endif

//this fades the screen
#ifdef fade
  res = ssd1315_advance_fade_blinking(SSD1315_FADE_BLINKING_MODE_BLINKING, 0);
  if (res != 0)
  {
      (void)ssd1315_advance_deinit();

      return 1;
  }
#endif

//this scrolls the screen
#ifdef scroll
  res = ssd1315_advance_vertical_left_horizontal_scroll(0, 7, 0, SSD1315_SCROLL_FRAME_2);
  if (res != 0)
  {
      (void)ssd1315_advance_deinit();

      return 1;
  }
#endif

//this publishes an image
  //Note: image loading goes along the 64 pixel side first, then goes to the 128 side
  //bmp image is 128x128 to compensate for some kind of transformation artifact where every 64x direction pixel is counter half as many times as necessary
  	  	  //image matrix is still 8310 bytes, as should be, so probably something goes wrong with the image generation
  res = ssd1315_advance_picture(0, 0, 127, 63, head);
  if (res != 0)
  {
      (void)ssd1315_advance_deinit();

      return 1;
  }

  HAL_Delay(1000);

  res = ssd1315_advance_clear();
/*
   //we define the calibration message matrices
//   uint8_t reset_sensor[2] = {0xE0, 0xB6};												//reset register and value for resetting for BMP280
   uint8_t std_setup[2] = {0xF4, 0x27};													//we define a standard mode with no oversampling

   //BMP280 init
   //BMP280 I2C address is 0x77
 //  HAL_I2C_Master_Transmit(&hi2c1, (0x77 << 1), std_setup, 2,1000);

   TIM2Config();
   I2CConfig(0x20);
   I2C1IRQPriorEnable();

   //we set teh device address to 0x77
   uint8_t device_addr = 0x77;

    //BMP280 init
    I2CTX(device_addr, 2, std_setup);													//standard mode, no oversampling
    Delay_ms(200);*/

  /* USER CODE END 2 */

  /* Init code for STM32_WPAN */
  MX_APPE_Init();

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    MX_APPE_Process();

    /* USER CODE BEGIN 3 */

	  //--------------------//

/*	  //read BMP280 sensor
	  uint8_t T_adc[3] = {0xFA, 0xFB, 0xFC};											//this is where the ADC temp values will go
	  uint8_t T_out[3] = {0x0, 0x0, 0x0};

	  //We rebuild the parameters from the readout
	  //these values are coming from the T_comp readout, see I2C project. We bypass the readout since they are constant values anyway.
	  uint16_t dig_T1 = (0x6b << 8) | 0x36;
	  int16_t dig_T2 = (0x65 << 8) | 0x7d;
	  int16_t dig_T3 = (0x0 << 8) | 0x8c;*/

	  //BMP280 temperature ADC readout
	  //we execute each readout separate to avoid the while loop block without the readout's code
	  //also we store the readout values in a static variable since if the bus is busy, we skip the readout
	  	  //this way we ensure that eventually we will have the right values captured eventually without delays use

    /*            	I2CReadout(0x77, 1, &T_adc[0], &T_out[0]);
                	I2CReadout(0x77, 1, &T_adc[1], &T_out[1]);
                	I2CReadout(0x77, 1, &T_adc[2], &T_out[2]);

                	int32_t adc_T = (T_out[0] << 12) | (T_out[1] << 4) | (T_out[2]>>4);				//we rebuild the 20 bit temperature value

                	int32_t temperature = compensate_temperature(adc_T, dig_T1, dig_T2, dig_T3);*/

 /*               	I2CReadout(0x77, 1, &T_adc[0], &T_out[0]);
                	I2CReadout(0x77, 1, &T_adc[1], &T_out[1]);
                	I2CReadout(0x77, 1, &T_adc[2], &T_out[2]);

                	int32_t adc_T = (T_out[0] << 12) | (T_out[1] << 4) | (T_out[2]>>4);				//we rebuild the 20 bit temperature value

                	int32_t temperature = compensate_temperature(adc_T, dig_T1, dig_T2, dig_T3);

          	  //--------------------//

    			  //publish temperature value to screen

    			  char str[10];

    			  sprintf(str, "%u", temperature);

    			  ssd1315_advance_string(0, 0, str, 10, 1, 0x10);*/

	  //--------------------//
	  //send temperature to BLE


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

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_MEDIUMHIGH);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE
                              |RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 32;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the SYSCLKSource, HCLK, PCLK1 and PCLK2 clocks dividers
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK4|RCC_CLOCKTYPE_HCLK2
                              |RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK2Divider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.AHBCLK4Divider = RCC_SYSCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SMPS|RCC_PERIPHCLK_RFWAKEUP;
  PeriphClkInitStruct.RFWakeUpClockSelection = RCC_RFWKPCLKSOURCE_HSE_DIV1024;
  PeriphClkInitStruct.SmpsClockSelection = RCC_SMPSCLKSOURCE_HSI;
  PeriphClkInitStruct.SmpsDivSelection = RCC_SMPSCLKDIV_RANGE1;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN Smps */

  /* USER CODE END Smps */
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
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x10707DBC;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief IPCC Initialization Function
  * @param None
  * @retval None
  */
static void MX_IPCC_Init(void)
{

  /* USER CODE BEGIN IPCC_Init 0 */

  /* USER CODE END IPCC_Init 0 */

  /* USER CODE BEGIN IPCC_Init 1 */

  /* USER CODE END IPCC_Init 1 */
  hipcc.Instance = IPCC;
  if (HAL_IPCC_Init(&hipcc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IPCC_Init 2 */

  /* USER CODE END IPCC_Init 2 */

}

/**
  * @brief RF Initialization Function
  * @param None
  * @retval None
  */
static void MX_RF_Init(void)
{

  /* USER CODE BEGIN RF_Init 0 */

  /* USER CODE END RF_Init 0 */

  /* USER CODE BEGIN RF_Init 1 */

  /* USER CODE END RF_Init 1 */
  /* USER CODE BEGIN RF_Init 2 */

  /* USER CODE END RF_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = CFG_RTC_ASYNCH_PRESCALER;
  hrtc.Init.SynchPrediv = CFG_RTC_SYNCH_PRESCALER;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable the WakeUp
  */
  if (HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 0, RTC_WAKEUPCLOCK_RTCCLK_DIV16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

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
  hspi1.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOH, GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin : PH0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pins : PC9 PC8 */
  GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

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
