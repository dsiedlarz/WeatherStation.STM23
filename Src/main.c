/**
 ******************************************************************************
 * File Name          : main.c
 * Description        : Main program body
 ******************************************************************************
 * This notice applies to any and all portions of this file
 * that are not between comment pairs USER CODE BEGIN and
 * USER CODE END. Other portions of this file, whether
 * inserted by the user or by software development tools
 * are owned by their respective copyright owners.
 *
 * Copyright (c) 2017 STMicroelectronics International N.V.
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "usb_host.h"

/* USER CODE BEGIN Includes */
#include "stm32412g_discovery_lcd.h"
#include "stm32412g_discovery_sd.h"
#include "humidity.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

I2S_HandleTypeDef hi2s3;

QSPI_HandleTypeDef hqspi;

SD_HandleTypeDef hsd;

UART_HandleTypeDef huart2;

SRAM_HandleTypeDef hsram1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_FSMC_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_I2S3_Init(void);
static void MX_QUADSPI_Init(void);
static void MX_SDIO_SD_Init(void);
static void MX_USART2_UART_Init(void);
void MX_USB_HOST_Process(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

typedef struct {
	float STLM75_temperature;
	float TSL25721_light;
	float HTS221_humidity;
	float HTS221_temperature;
	float LPS331_pressure;
	float LPS331_temperature;
} WeatherMeasurement;

void init_humidity() {
	HUM_TEMP_InitTypeDef* initDef;
	initDef->Power_Mode = HTS221_MODE_ACTIVE;
	initDef->Data_Update_Mode = HTS221_BDU_CONTINUOUS;
	initDef->OutputDataRate = HTS221_ODR_1Hz;
	initDef->Reboot_Mode = HTS221_BOOT_NORMALMODE;
	initDef->Temperature_Resolution = HTS221_T_RES_AVG_32;
	initDef->Humidity_Resolutin = HTS221_H_RES_AVG_32;

	HTS221_Init(initDef);
}

void init_pressure() {
	uint8_t data[1];
	data[0] = 0b11100000;
	HAL_I2C_Mem_Write(&hi2c2, 0xBA, 0x20, I2C_MEMADD_SIZE_8BIT, data, 1, 100);

}

void get_humidity(WeatherMeasurement* weatherMeasurement) {
	float humidity = 0;
	float temperature = 0;

	HTS221_GetHumidity(&humidity);
	weatherMeasurement->HTS221_humidity = humidity;

	HTS221_GetTemperature(&temperature);
	weatherMeasurement->HTS221_temperature = temperature;
}

void read_temperature(WeatherMeasurement* weatherMeasurement) {
	uint8_t buff[2];

	HAL_I2C_Mem_Read(&hi2c2, 0x91, 0x00, I2C_MEMADD_SIZE_8BIT, buff, 2, 100);
	weatherMeasurement->STLM75_temperature = (buff[0] + (0.5 * (buff[1] >> 7)));
}

void read_pressure(WeatherMeasurement* weatherMeasurement) {

	uint8_t data[3];

	float val = 0;

	do {
		HAL_I2C_Mem_Read(&hi2c2, 0xBB, 0x28, I2C_MEMADD_SIZE_8BIT, data, 3,
				100);

		uint32_t press;

		press = (int32_t) (int8_t) data[2] << 16 | (uint16_t) data[1] << 8
				| data[0];

		val = (press / 4096) / 1000;
	} while (val < 100);
	weatherMeasurement->LPS331_pressure = val;

	//temperature
	HAL_I2C_Mem_Read(&hi2c2, 0xBB, 0x2B, I2C_MEMADD_SIZE_8BIT, data, 2, 100);

	float temp;

	temp = ((float) ((uint16_t) data[1] << 8 | data[0]));

	temp = temp / 480;
	temp = temp + 42.5;
	temp = temp / 4.5;

	weatherMeasurement->LPS331_temperature = temp;
}

void read_light(WeatherMeasurement* weatherMeasurement) {
	uint8_t data[26];
	uint8_t data2[20];
	uint16_t c0, c1;
	float lux1, lux2, cpl, value;

	data2[0] = 0;
	data2[1] = 0;

	data[0] = 0x00;
	HAL_I2C_Mem_Write(&hi2c2, 0x72, 0x0F, I2C_MEMADD_SIZE_8BIT, data, 1, 100);
	data[0] = 0xED;
	HAL_I2C_Mem_Write(&hi2c2, 0x72, 0x01, I2C_MEMADD_SIZE_8BIT, data, 1, 100);

	data[0] = 0x03;
	HAL_I2C_Mem_Write(&hi2c2, 0x72, 0x00, I2C_MEMADD_SIZE_8BIT, data, 1, 100);

	HAL_Delay(500);

	HAL_I2C_Mem_Read(&hi2c2, 0x73, 0x00, I2C_MEMADD_SIZE_8BIT, data, 25, 100);

	c0 = (((uint16_t) data[21]) << 8) | (uint16_t) data[20];
	c1 = (((uint16_t) data[23]) << 8) | (uint16_t) data[22];

	int ATIME_ms = 50;
	int AGAINx = 1;
	int GA = 1;
	cpl = (ATIME_ms * AGAINx) / (GA * 53);

	lux1 = ((float) c0 - 2 * c1) * 20;
//		lux2 = ((float) 0.6 * c1 - (float)c0)/cpl;
//lux1 = c0*3;
	lux2 = c1 * 6;

	if (lux1 > lux2) {
		value = lux1;
	} else {
		value = lux2;
	}


	weatherMeasurement->TSL25721_light = value;
}

void HTS221_Get_Temperature(int16_t *value) {
	int16_t T0_out, T1_out, T_out, T0_degC_x8_u16, T1_degC_x8_u16;
	int16_t T0_degC, T1_degC;
	uint8_t buffer[4], tmp;
	uint32_t tmp32;
	uint8_t data[0];
	data[0] = 1;

	/*1. Read from 0x32 & 0x33 registers the value of coefficients T0_degC_x8 and T1_degC_x8*/
	HAL_I2C_Mem_Read(&hi2c2, 0xBF, 0x32, I2C_MEMADD_SIZE_8BIT, buffer, 2, 100);

	/*2. Read from 0x35 register the value of the MSB bits of T1_deg C and T0_deg C */
	HAL_I2C_Mem_Read(&hi2c2, 0xBF, 0x35, I2C_MEMADD_SIZE_8BIT, &tmp, 1, 100);

	/*Calculate the T0_deg C and T1_deg C values*/
	T0_degC_x8_u16 = (((uint16_t) (tmp & 0x03)) << 8) | ((uint16_t) buffer[0]);
	T1_degC_x8_u16 = (((uint16_t) (tmp & 0x0C)) << 6) | ((uint16_t) buffer[1]);
	T0_degC = T0_degC_x8_u16 >> 3;
	T1_degC = T1_degC_x8_u16 >> 3;
	/*3. Read from 0x3C & 0x3D registers the value of T0_OUT*//*4. Read from 0x3E & 0x3F registers the value of T1_OUT*/
	HAL_I2C_Mem_Read(&hi2c2, 0xBF, 0x3C, I2C_MEMADD_SIZE_8BIT, buffer, 4, 100);

	T0_out = (((uint16_t) buffer[1]) << 8) | (uint16_t) buffer[0];
	T1_out = (((uint16_t) buffer[3]) << 8) | (uint16_t) buffer[2];
	/* 5.Read from 0x2A & 0x2B registers the value T_OUT (ADC_OUT).*/
	HAL_I2C_Mem_Read(&hi2c2, 0xBF, 0x2A, I2C_MEMADD_SIZE_8BIT, buffer, 2, 100);
	T_out = (((uint16_t) buffer[1]) << 8) | (uint16_t) buffer[0];
	/* 6. Compute the Temperature value by linear interpolation*/
	tmp32 = ((uint32_t) (T_out - T0_out))
			* ((uint32_t) (T1_degC - T0_degC) * 10);
	*value = tmp32 / (T1_out - T0_out) + T0_degC * 10;

	*value = *value / 10;
}

void HTS221_Get_Humidity(uint16_t* value) {
	int16_t H0_T0_out, H1_T0_out, H_T_out;
	int16_t H0_rh, H1_rh;
	uint8_t buffer[2];
	uint32_t tmp;
	/* 1. Read H0_rH and H1_rH coefficients*/
	HAL_I2C_Mem_Read(&hi2c2, 0xBF, 0x30, I2C_MEMADD_SIZE_8BIT, buffer, 2, 100);

	H0_rh = buffer[0] >> 1;
	H1_rh = buffer[1] >> 1;
	/*2. Read H0_T0_OUT */
	HAL_I2C_Mem_Read(&hi2c2, 0xBF, 0x36, I2C_MEMADD_SIZE_8BIT, buffer, 2, 100);

	H0_T0_out = (((uint16_t) buffer[1]) << 8) | (uint16_t) buffer[0];
	/*3. Read H1_T0_OUT */
	HAL_I2C_Mem_Read(&hi2c2, 0xBF, 0x3A, I2C_MEMADD_SIZE_8BIT, buffer, 2, 100);

	H1_T0_out = (((uint16_t) buffer[1]) << 8) | (uint16_t) buffer[0];
	/*4. Read H_T_OUT */
	HAL_I2C_Mem_Read(&hi2c2, 0xBF, 0x28, I2C_MEMADD_SIZE_8BIT, buffer, 2, 100);

	H_T_out = (((uint16_t) buffer[1]) << 8) | (uint16_t) buffer[0];
	/*5. Compute the RH [%] value by linear interpolation */
	tmp = ((uint32_t) (H_T_out - H0_T0_out))
			* ((uint32_t) (H1_rh - H0_rh) * 10);
	*value = tmp / (H1_T0_out - H0_T0_out) + H0_rh * 10;
	/* Saturation condition*/
	if (*value > 1000)
		*value = 1000;

	*value = *value / 10;
}

static void set_theme(void)
{
  /* Clear the LCD */
  BSP_LCD_Clear(LCD_COLOR_WHITE);

  /* Set LCD Demo description */
  BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
  BSP_LCD_FillRect(0, 0, BSP_LCD_GetXSize(), 80);
  BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
  BSP_LCD_SetBackColor(LCD_COLOR_BLUE);
  BSP_LCD_SetFont(&Font16);
  BSP_LCD_DisplayStringAt(0, 20, (uint8_t *)"Weather station", CENTER_MODE);
  BSP_LCD_SetFont(&Font12);
  BSP_LCD_DisplayStringAt(0, 50, (uint8_t *)"created by Dawid Siedlarz", CENTER_MODE);
  BSP_LCD_DisplayStringAt(0, 64, (uint8_t *)"v1.0", CENTER_MODE);

   /* Set the LCD Text Color */
  BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
  BSP_LCD_DrawRect(10, 90, BSP_LCD_GetXSize() - 20, BSP_LCD_GetYSize()- 101);
  BSP_LCD_DrawRect(11, 91, BSP_LCD_GetXSize() - 22, BSP_LCD_GetYSize()- 102);

  BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
  BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
 }

void print_measurement(WeatherMeasurement* weatherMeasurement) {
	set_theme();

	char msg[255];

	sprintf(msg, "Temperature[c]: %.2f", weatherMeasurement->STLM75_temperature);
	BSP_LCD_DisplayStringAt(0, 100, (uint8_t*) (msg), CENTER_MODE);

	sprintf(msg, "Pressure[hpa]: %.2f", weatherMeasurement->LPS331_pressure);
	BSP_LCD_DisplayStringAt(0, 120, (uint8_t*) (msg), CENTER_MODE);

	sprintf(msg, "Light[lux]: %.2f", weatherMeasurement->TSL25721_light);
	BSP_LCD_DisplayStringAt(0, 140, (uint8_t*) (msg), CENTER_MODE);

	sprintf(msg, "Humidity[perc]: %.2f", weatherMeasurement->HTS221_humidity);
	BSP_LCD_DisplayStringAt(0, 160, (uint8_t*) (msg), CENTER_MODE);
}

/* USER CODE END 0 */

int main(void) {

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
	MX_FSMC_Init();
	MX_I2C1_Init();
	MX_I2C2_Init();
	MX_I2S3_Init();
	MX_QUADSPI_Init();
	MX_SDIO_SD_Init();
	MX_USART2_UART_Init();
	MX_USB_HOST_Init();

	/* USER CODE BEGIN 2 */


	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	/* LCD Initialization */
	BSP_LCD_Init();

	init_humidity();
	init_pressure();

	/* Clear the LCD */
	BSP_LCD_Clear(LCD_COLOR_WHITE);
	WeatherMeasurement weatherMeasurement;

	while (1) {
 	    read_temperature(&weatherMeasurement);
		read_pressure(&weatherMeasurement);
	    get_humidity(&weatherMeasurement);
		read_light(&weatherMeasurement);
		print_measurement(&weatherMeasurement);

		HAL_Delay(500);

		/* USER CODE END WHILE */
//    MX_USB_HOST_Process();
		/* USER CODE BEGIN 3 */

	}
	return 1;
	/* USER CODE END 3 */
}

/** System Clock Configuration
 */
void SystemClock_Config(void) {

	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

	/**Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE()
	;

	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI
			| RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = 16;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 4;
	RCC_OscInitStruct.PLL.PLLN = 72;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 3;
	RCC_OscInitStruct.PLL.PLLR = 2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_I2S_APB1
			| RCC_PERIPHCLK_SDIO | RCC_PERIPHCLK_CLK48;
	PeriphClkInitStruct.PLLI2S.PLLI2SN = 50;
	PeriphClkInitStruct.PLLI2S.PLLI2SM = 4;
	PeriphClkInitStruct.PLLI2S.PLLI2SR = 2;
	PeriphClkInitStruct.PLLI2S.PLLI2SQ = 2;
	PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48CLKSOURCE_PLLQ;
	PeriphClkInitStruct.SdioClockSelection = RCC_SDIOCLKSOURCE_CLK48;
	PeriphClkInitStruct.PLLI2SSelection = RCC_PLLI2SCLKSOURCE_PLLSRC;
	PeriphClkInitStruct.I2sApb1ClockSelection = RCC_I2SAPB1CLKSOURCE_PLLI2S;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_HSI, RCC_MCODIV_1);

	/**Configure the Systick interrupt time
	 */
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

	/**Configure the Systick
	 */
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	/* SysTick_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* I2C1 init function */
static void MX_I2C1_Init(void) {

	hi2c1.Instance = I2C1;
	hi2c1.Init.ClockSpeed = 100000;
	hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

}

/* I2C2 init function */
static void MX_I2C2_Init(void) {

	hi2c2.Instance = I2C2;
	hi2c2.Init.ClockSpeed = 200000;
	hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c2.Init.OwnAddress1 = 0;
	hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c2.Init.OwnAddress2 = 0;
	hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c2) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

}

/* I2S3 init function */
static void MX_I2S3_Init(void) {

	hi2s3.Instance = SPI3;
	hi2s3.Init.Mode = I2S_MODE_MASTER_TX;
	hi2s3.Init.Standard = I2S_STANDARD_PHILIPS;
	hi2s3.Init.DataFormat = I2S_DATAFORMAT_16B;
	hi2s3.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
	hi2s3.Init.AudioFreq = I2S_AUDIOFREQ_32K;
	hi2s3.Init.CPOL = I2S_CPOL_LOW;
	hi2s3.Init.ClockSource = I2S_CLOCK_PLL;
	hi2s3.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_ENABLE;
	if (HAL_I2S_Init(&hi2s3) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

}

/* QUADSPI init function */
static void MX_QUADSPI_Init(void) {

	hqspi.Instance = QUADSPI;
	hqspi.Init.ClockPrescaler = 255;
	hqspi.Init.FifoThreshold = 1;
	hqspi.Init.SampleShifting = QSPI_SAMPLE_SHIFTING_NONE;
	hqspi.Init.FlashSize = 1;
	hqspi.Init.ChipSelectHighTime = QSPI_CS_HIGH_TIME_1_CYCLE;
	hqspi.Init.ClockMode = QSPI_CLOCK_MODE_0;
	hqspi.Init.FlashID = QSPI_FLASH_ID_1;
	hqspi.Init.DualFlash = QSPI_DUALFLASH_DISABLE;
	if (HAL_QSPI_Init(&hqspi) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

}

/* SDIO init function */
static void MX_SDIO_SD_Init(void) {

	hsd.Instance = SDIO;
	hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
	hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
	hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
	hsd.Init.BusWide = SDIO_BUS_WIDE_1B;
	hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
	hsd.Init.ClockDiv = 0;
	if (HAL_SD_Init(&hsd) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	HAL_SD_ConfigWideBusOperation(&hsd, SDIO_BUS_WIDE_4B);

}

/* USART2 init function */
static void MX_USART2_UART_Init(void) {

	huart2.Instance = USART2;
	huart2.Init.BaudRate = 115200;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart2) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

}

/** Configure pins as
 * Analog
 * Input
 * Output
 * EVENT_OUT
 * EXTI
 PE4   ------> S_DATAIN3DFSDM1
 PC2   ------> S_CKOUTDFSDM1
 PB1   ------> S_DATAIN0DFSDM1
 PB11   ------> I2S_CKIN
 PA8   ------> RCC_MCO_1
 */
static void MX_GPIO_Init(void) {

	GPIO_InitTypeDef GPIO_InitStruct;

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOE_CLK_ENABLE()
	;
	__HAL_RCC_GPIOC_CLK_ENABLE()
	;
	__HAL_RCC_GPIOF_CLK_ENABLE()
	;
	__HAL_RCC_GPIOH_CLK_ENABLE()
	;
	__HAL_RCC_GPIOA_CLK_ENABLE()
	;
	__HAL_RCC_GPIOB_CLK_ENABLE()
	;
	__HAL_RCC_GPIOG_CLK_ENABLE()
	;
	__HAL_RCC_GPIOD_CLK_ENABLE()
	;

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOE, LED3_Pin | LED4_Pin | LED1_Pin | LED2_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOF, LCD_BLCTRL_Pin | EXT_RESET_Pin | CTP_RST_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LCD_RESET_GPIO_Port, LCD_RESET_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(USB_OTGFS_PPWR_EN_GPIO_Port, USB_OTGFS_PPWR_EN_Pin,
			GPIO_PIN_SET);

	/*Configure GPIO pins : LED3_Pin LED4_Pin LED1_Pin LED2_Pin */
	GPIO_InitStruct.Pin = LED3_Pin | LED4_Pin | LED1_Pin | LED2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

	/*Configure GPIO pin : DFSDM_DATIN3_Pin */
	GPIO_InitStruct.Pin = DFSDM_DATIN3_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF8_DFSDM1;
	HAL_GPIO_Init(DFSDM_DATIN3_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : LCD_BLCTRL_Pin EXT_RESET_Pin CTP_RST_Pin */
	GPIO_InitStruct.Pin = LCD_BLCTRL_Pin | EXT_RESET_Pin | CTP_RST_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

	/*Configure GPIO pin : DFSDM_CKOUT_Pin */
	GPIO_InitStruct.Pin = DFSDM_CKOUT_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF8_DFSDM1;
	HAL_GPIO_Init(DFSDM_CKOUT_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : JOY_SEL_Pin */
	GPIO_InitStruct.Pin = JOY_SEL_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(JOY_SEL_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : DFSDM_DATIN0_Pin */
	GPIO_InitStruct.Pin = DFSDM_DATIN0_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF8_DFSDM1;
	HAL_GPIO_Init(DFSDM_DATIN0_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : JOY_RIGHT_Pin JOY_LEFT_Pin */
	GPIO_InitStruct.Pin = JOY_RIGHT_Pin | JOY_LEFT_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

	/*Configure GPIO pins : JOY_UP_Pin JOY_DOWN_Pin LCD_TE_Pin USB_OTGFS_OVRCR_Pin */
	GPIO_InitStruct.Pin = JOY_UP_Pin | JOY_DOWN_Pin | LCD_TE_Pin
			| USB_OTGFS_OVRCR_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

	/*Configure GPIO pin : M2_CKIN_Pin */
	GPIO_InitStruct.Pin = M2_CKIN_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
	HAL_GPIO_Init(M2_CKIN_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : LCD_RESET_Pin */
	GPIO_InitStruct.Pin = LCD_RESET_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LCD_RESET_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : CODEC_INT_Pin CTP_INT_Pin */
	GPIO_InitStruct.Pin = CODEC_INT_Pin | CTP_INT_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

	/*Configure GPIO pin : USB_OTGFS_PPWR_EN_Pin */
	GPIO_InitStruct.Pin = USB_OTGFS_PPWR_EN_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(USB_OTGFS_PPWR_EN_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : M2_CKINA8_Pin */
	GPIO_InitStruct.Pin = M2_CKINA8_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF0_MCO;
	HAL_GPIO_Init(M2_CKINA8_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : uSD_DETECT_Pin */
	GPIO_InitStruct.Pin = uSD_DETECT_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(uSD_DETECT_GPIO_Port, &GPIO_InitStruct);

}

/* FSMC initialization function */
static void MX_FSMC_Init(void) {
	FSMC_NORSRAM_TimingTypeDef Timing;

	/** Perform the SRAM1 memory initialization sequence
	 */
	hsram1.Instance = FSMC_NORSRAM_DEVICE;
	hsram1.Extended = FSMC_NORSRAM_EXTENDED_DEVICE;
	/* hsram1.Init */
	hsram1.Init.NSBank = FSMC_NORSRAM_BANK1;
	hsram1.Init.DataAddressMux = FSMC_DATA_ADDRESS_MUX_DISABLE;
	hsram1.Init.MemoryType = FSMC_MEMORY_TYPE_SRAM;
	hsram1.Init.MemoryDataWidth = FSMC_NORSRAM_MEM_BUS_WIDTH_16;
	hsram1.Init.BurstAccessMode = FSMC_BURST_ACCESS_MODE_DISABLE;
	hsram1.Init.WaitSignalPolarity = FSMC_WAIT_SIGNAL_POLARITY_LOW;
	hsram1.Init.WaitSignalActive = FSMC_WAIT_TIMING_BEFORE_WS;
	hsram1.Init.WriteOperation = FSMC_WRITE_OPERATION_ENABLE;
	hsram1.Init.WaitSignal = FSMC_WAIT_SIGNAL_DISABLE;
	hsram1.Init.ExtendedMode = FSMC_EXTENDED_MODE_DISABLE;
	hsram1.Init.AsynchronousWait = FSMC_ASYNCHRONOUS_WAIT_DISABLE;
	hsram1.Init.WriteBurst = FSMC_WRITE_BURST_DISABLE;
	hsram1.Init.ContinuousClock = FSMC_CONTINUOUS_CLOCK_SYNC_ONLY;
	hsram1.Init.WriteFifo = FSMC_WRITE_FIFO_ENABLE;
	hsram1.Init.PageSize = FSMC_PAGE_SIZE_NONE;
	/* Timing */
	Timing.AddressSetupTime = 15;
	Timing.AddressHoldTime = 15;
	Timing.DataSetupTime = 255;
	Timing.BusTurnAroundDuration = 15;
	Timing.CLKDivision = 16;
	Timing.DataLatency = 17;
	Timing.AccessMode = FSMC_ACCESS_MODE_A;
	/* ExtTiming */

	if (HAL_SRAM_Init(&hsram1, &Timing, NULL) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  None
 * @retval None
 */
void _Error_Handler(char * file, int line) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	while (1) {
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT

/**
 * @brief Reports the name of the source file and the source line number
 * where the assert_param error has occurred.
 * @param file: pointer to the source file name
 * @param line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	 ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */

}

#endif

HUM_TEMP_StatusTypeDef HTS221_IO_Init(void) {

	return HUM_TEMP_OK;
}
HUM_TEMP_StatusTypeDef HTS221_IO_Write(uint8_t* pBuffer, uint8_t DeviceAddr,
		uint8_t RegisterAddr, uint16_t NumByteToWrite) {

	HAL_I2C_Mem_Write(&hi2c2, DeviceAddr, RegisterAddr, I2C_MEMADD_SIZE_8BIT,
			pBuffer, NumByteToWrite, 100);
	return HUM_TEMP_OK;
}
HUM_TEMP_StatusTypeDef HTS221_IO_Read(uint8_t* pBuffer, uint8_t DeviceAddr,
		uint8_t RegisterAddr, uint16_t NumByteToRead) {
	HAL_I2C_Mem_Read(&hi2c2, DeviceAddr, RegisterAddr, I2C_MEMADD_SIZE_8BIT,
			pBuffer, NumByteToRead, 100);
	return HUM_TEMP_OK;
}
/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
