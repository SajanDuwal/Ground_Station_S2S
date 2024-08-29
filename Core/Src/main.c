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
#include "app_subghz_phy.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "radio_driver.h"
#include "com_debug.h"
#include "ax25_packet.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define MAIN_CMD_LENGTH			(13)  //  main 13 bytes command received from UART2
#define RX_PAYLOAD_LENGTH 		(105) //  maximum data receiving capacity from radio

#define FREQ_435_MHZ            (435313000) //UP-LINK
#define FREQ_437_MHZ			(437375000)	//DOWN-LINK

#define PA_DUTY_CYCLE           (0x07)
#define HP_MAX                  (0x00)
#define PA_SEL                  (0x01)

#define POWER                   (0x0E)
#define RAMP_TIME               (0x02)

#define GFSK_BR_1200            1200
#define GFSK_FDEV_1200          3000

#define GFSK_BR_4800            4800
#define GFSK_FDEV_4800          12000
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SUBGHZ_HandleTypeDef hsubghz;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
PacketParams_t pkt_params;
ModulationParams_t mod_params;

uint8_t main_cmd_len = MAIN_CMD_LENGTH;
uint8_t main_cmd[MAIN_CMD_LENGTH];

uint8_t temp_tx_buffer[100];
int tx_buffer_len = 0;
uint8_t tx_buffer[100];

uint8_t rx_buffer_len = RX_PAYLOAD_LENGTH;
uint8_t rx_buffer[RX_PAYLOAD_LENGTH];

int TX_FLAG = 0;
int rssi_value = 0;
int checksum_error_count = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
int countsDataBetweenFlags(uint8_t *data, int data_length);
void setPacketParams(uint8_t buffer_length);
void setModulationParams(unsigned long bitRate, unsigned long fDev);
void radioConfig(uint8_t *buffer, uint8_t buffer_len);
void DioIrqHndlr(RadioIrqMasks_t radioIrq);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int countsDataBetweenFlags(uint8_t *data, int data_length) {
	int found_first_7e = 0;
	int start_index = 0, end_index = 0;

	for (int i = 0; i < data_length; i++) {
		if (data[i] == 0x7e) {
			if (!found_first_7e) {
				found_first_7e = 1;
				start_index = i;
			} else {
				end_index = i;
				break;
			}
		}
	}

	if (end_index > start_index) {
		return end_index - start_index + 1;
	} else {
		return -1; // Return -1 if two 0x7E flags are not found
	}
}

void setPacketParams(uint8_t buffer_length) {
	pkt_params.PacketType = PACKET_TYPE_GFSK;
	pkt_params.Params.Gfsk.PayloadLength = buffer_length;
	pkt_params.Params.Gfsk.PreambleLength = 8;
	pkt_params.Params.Gfsk.PreambleMinDetect = RADIO_PREAMBLE_DETECTOR_08_BITS;
	pkt_params.Params.Gfsk.SyncWordLength = 3 << 3;
	pkt_params.Params.Gfsk.AddrComp = RADIO_ADDRESSCOMP_FILT_OFF;
	pkt_params.Params.Gfsk.HeaderType = RADIO_PACKET_FIXED_LENGTH;
	pkt_params.Params.Gfsk.CrcLength = RADIO_CRC_2_BYTES_CCIT;
	pkt_params.Params.Gfsk.DcFree = RADIO_DC_FREEWHITENING;
	SUBGRF_SetPacketParams(&pkt_params);

}
void setModulationParams(unsigned long bitRate, unsigned long fDev) {
	mod_params.PacketType = PACKET_TYPE_GFSK;
	mod_params.Params.Gfsk.Bandwidth = RX_BW_29300;
	mod_params.Params.Gfsk.BitRate = bitRate;
	mod_params.Params.Gfsk.Fdev = fDev;
	mod_params.Params.Gfsk.ModulationShaping = MOD_SHAPING_G_BT_1;
	SUBGRF_SetModulationParams(&mod_params);

}

void radioConfig(uint8_t *buffer, uint8_t buffer_len) {
	SUBGRF_SetBufferBaseAddress(0x00, 0x00);
	SUBGRF_SetPayload(buffer, buffer_len);
	SUBGRF_SetSyncWord(( uint8_t[] ) { 0xC1, 0x94, 0xC1, 0x00, 0x00, 0x00, 0x00,
					0x00 });
	SUBGRF_SetWhiteningSeed(0x01FF);
	SUBGRF_SetPaConfig(PA_DUTY_CYCLE, HP_MAX, PA_SEL, 0x01);
	SUBGRF_SetTxParams(RFO_LP, POWER, RAMP_TIME);

	SUBGRF_Init(DioIrqHndlr);
	SUBGRF_SetDioIrqParams(
			IRQ_TX_DONE | IRQ_PREAMBLE_DETECTED | IRQ_RX_DONE
					| IRQ_RX_TX_TIMEOUT | IRQ_SYNCWORD_VALID,
			IRQ_TX_DONE | IRQ_PREAMBLE_DETECTED | IRQ_RX_DONE
					| IRQ_RX_TX_TIMEOUT | IRQ_SYNCWORD_VALID, IRQ_RADIO_NONE,
			IRQ_RADIO_NONE);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart == &huart2) {

		uint8_t header = 0x00;
		if (main_cmd[0] == header || main_cmd[0] != 0x53) {

			for (int loop1 = 0; loop1 < sizeof(main_cmd); loop1++) {
				main_cmd[loop1] = main_cmd[loop1 + 1];
			}
		}

		myDebug("\n-->Main command Received: 0x%x\r\n", main_cmd);

		if (sizeof(main_cmd) == main_cmd_len) {
//			myDebug("-->Command ACK: 0x%x\r\n", main_cmd);
			for (int i = 0; i < main_cmd_len; i++) {
				myDebug("%02x ", main_cmd[i]);
			}
			myDebug("\r\n");
			TX_FLAG = 1;
		} else {
			myDebug("-->Command Not ACK: 0x%x\r\n", main_cmd);
			memset(main_cmd, '\0', main_cmd_len);
			TX_FLAG = 0;
		}
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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_SubGHz_Phy_Init();
  /* USER CODE BEGIN 2 */

	HAL_TIM_Base_Start(&htim2);

	myDebug("########## Slippers2Sat Ground Station: BEGIN ##########\r\n");
	myDebug("########## COMMUNICATION PARAMETERS ##########\r\n");
	myDebug("Modulation: GFSK PACKET\r\n");
	myDebug("FREQUENCY MODES: DOWNLINK FREQ: %luHz, UPLINK FREQ: %lu Hz\r\n",
	FREQ_437_MHZ, FREQ_435_MHZ);
	myDebug("STM32 BSP_SubGHz-WL Radio: Low Power\n");
	myDebug(
			"POWER CONFIG:::: \n"
					"\t PA_DUTY_CYCLE: %x, HP_MAX: %x, PA_SEL: %x, POWER TX: %u dBm\r\n",
			PA_DUTY_CYCLE, HP_MAX, PA_SEL, POWER);

	myDebug(
			"\n########## Operation Starts, Perform any one operation: ##########\r\n");
	myDebug("1. Transmit Command of 13 bytes\r\n");
	myDebug("2. Wait to receive beacon from Satellite\r\n");

	setPacketParams(rx_buffer_len);
	setModulationParams(GFSK_BR_4800, GFSK_FDEV_4800);
	radioConfig(rx_buffer, rx_buffer_len);

	myDebug("\n########## RX Configuration: ##########\n");

	myDebug("FREQUENCY MODS: Downlink FREQ: %lu Hz\r\n", FREQ_437_MHZ);
	myDebug("Bit Rate: 	%d\n\r", mod_params.Params.Gfsk.BitRate);
	myDebug("Frequency Deviation: 	%d\n\r", mod_params.Params.Gfsk.Fdev);
	myDebug("RECEVING BANDWIDTH: 	%d\n\r", mod_params.Params.Gfsk.Bandwidth);
	myDebug("Packet Type 			%d\n\r", pkt_params.PacketType);
	myDebug("PayloadLength 			%d\n\r", pkt_params.Params.Gfsk.PayloadLength);
	myDebug("PreambleLength 		%d\n\r", pkt_params.Params.Gfsk.PreambleLength);
	myDebug("PreambleMinDetect		%d\n\r",
			pkt_params.Params.Gfsk.PreambleMinDetect);
	myDebug("HeaderType 			%d\n\r", pkt_params.Params.Gfsk.HeaderType);
	myDebug("______________*******************______________\r\n");

	SUBGRF_SetRfFrequency(FREQ_437_MHZ);
	SUBGRF_SetSwitch(RFO_LP, RFSWITCH_RX);
	SUBGRF_SetRxBoosted(0xFFFFFF);

	HAL_UART_Receive_DMA(&huart2, main_cmd, main_cmd_len);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
    /* USER CODE END WHILE */
    MX_SubGHz_Phy_Process();

    /* USER CODE BEGIN 3 */

		delay_us(500000);

		if (TX_FLAG) {

			getAX25Packet(main_cmd, main_cmd_len);

			tx_buffer_len = countsDataBetweenFlags(temp_tx_buffer,
					sizeof(temp_tx_buffer));

//			myDebug("AX.25 complete GS packet ready to TX: 0x%x\r\n", temp_tx_buffer);
			for (int j = 0; j < tx_buffer_len; j++) {
				tx_buffer[j] = temp_tx_buffer[j];
//				myDebug("%02x ", tx_buffer[j]);
			}
//			myDebug("\r\n");

//			myDebug("size of tx_buffer = %d\r\n", tx_buffer_len);

			memset(main_cmd, '\0', main_cmd_len);
			memset(temp_tx_buffer, '\0', sizeof(temp_tx_buffer));

			setPacketParams(tx_buffer_len);
			setModulationParams(GFSK_BR_1200, GFSK_FDEV_1200);
			radioConfig(tx_buffer, tx_buffer_len);

			myDebug("########## TX Configuration: ##########\n");

			myDebug("FREQUENCY MODS: UPLINK FREQ: %lu Hz\r\n", FREQ_435_MHZ);
			myDebug("Bit Rate: 	%d\n\r", mod_params.Params.Gfsk.BitRate);
			myDebug("Frequency Deviation: 	%d\n\r",
					mod_params.Params.Gfsk.Fdev);
			myDebug("RECEVING BANDWIDTH: 	%d\n\r",
					mod_params.Params.Gfsk.Bandwidth);
			myDebug("Packet Type 			%d\n\r", pkt_params.PacketType);
			myDebug("PayloadLength 			%d\n\r",
					pkt_params.Params.Gfsk.PayloadLength);
			myDebug("PreambleLength 		%d\n\r",
					pkt_params.Params.Gfsk.PreambleLength);
			myDebug("PreambleMinDetect		%d\n\r",
					pkt_params.Params.Gfsk.PreambleMinDetect);
			myDebug("HeaderType 			%d\n\r", pkt_params.Params.Gfsk.HeaderType);
			myDebug("______________*******************______________\r\n");

			SUBGRF_SetRfFrequency(FREQ_435_MHZ);
			SUBGRF_SetSwitch(RFO_LP, RFSWITCH_TX);
			SUBGRF_SendPayload(tx_buffer, tx_buffer_len, 0);
		}

		HAL_UART_Receive_DMA(&huart2, main_cmd, main_cmd_len);

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS_PWR;
  RCC_OscInitStruct.HSEDiv = RCC_HSE_DIV1;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
  RCC_OscInitStruct.PLL.PLLN = 6;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the SYSCLKSource, HCLK, PCLK1 and PCLK2 clocks dividers
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK3|RCC_CLOCKTYPE_HCLK
                              |RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1
                              |RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK3Divider = RCC_SYSCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SUBGHZ Initialization Function
  * @param None
  * @retval None
  */
void MX_SUBGHZ_Init(void)
{

  /* USER CODE BEGIN SUBGHZ_Init 0 */

  /* USER CODE END SUBGHZ_Init 0 */

  /* USER CODE BEGIN SUBGHZ_Init 1 */

  /* USER CODE END SUBGHZ_Init 1 */
  hsubghz.Init.BaudratePrescaler = SUBGHZSPI_BAUDRATEPRESCALER_8;
  if (HAL_SUBGHZ_Init(&hsubghz) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SUBGHZ_Init 2 */

  /* USER CODE END SUBGHZ_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 48-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000001-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED1_Pin|LED2_Pin|LED3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, FE_CTRL3_Pin|FE_CTRL2_Pin|FE_CTRL1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED1_Pin LED2_Pin LED3_Pin */
  GPIO_InitStruct.Pin = LED1_Pin|LED2_Pin|LED3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : FE_CTRL3_Pin FE_CTRL2_Pin FE_CTRL1_Pin */
  GPIO_InitStruct.Pin = FE_CTRL3_Pin|FE_CTRL2_Pin|FE_CTRL1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : B1_Pin B2_Pin */
  GPIO_InitStruct.Pin = B1_Pin|B2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : B3_Pin */
  GPIO_InitStruct.Pin = B3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(B3_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void DioIrqHndlr(RadioIrqMasks_t radioIrq) {
	if (radioIrq == IRQ_TX_DONE) {
		TX_FLAG = 0;
		myDebug("\nGS Command Transmitted Successful, Length: %d\r\n",
				tx_buffer_len);
		for (int i = 0; i < tx_buffer_len; i++) {
			myDebug("%02x ", tx_buffer[i]);
		}

		myDebug("\r\n");
		memset(tx_buffer, '\0', sizeof(tx_buffer));

		setPacketParams(rx_buffer_len);
		setModulationParams(GFSK_BR_4800, GFSK_FDEV_4800);
		radioConfig(rx_buffer, rx_buffer_len);

		myDebug("\n########## RX Configuration: ##########\n");

		myDebug("FREQUENCY MODS: Downlink FREQ: %lu Hz\r\n", FREQ_437_MHZ);
		myDebug("Bit Rate: 	%d\n\r", mod_params.Params.Gfsk.BitRate);
		myDebug("Frequency Deviation: 	%d\n\r", mod_params.Params.Gfsk.Fdev);
		myDebug("RECEVING BANDWIDTH: 	%d\n\r",
				mod_params.Params.Gfsk.Bandwidth);
		myDebug("Packet Type 			%d\n\r", pkt_params.PacketType);
		myDebug("PayloadLength 			%d\n\r",
				pkt_params.Params.Gfsk.PayloadLength);
		myDebug("PreambleLength 		%d\n\r",
				pkt_params.Params.Gfsk.PreambleLength);
		myDebug("PreambleMinDetect		%d\n\r",
				pkt_params.Params.Gfsk.PreambleMinDetect);
		myDebug("HeaderType 			%d\n\r", pkt_params.Params.Gfsk.HeaderType);
		myDebug("______________*******************______________\r\n");

		SUBGRF_SetRfFrequency(FREQ_437_MHZ);
		SUBGRF_SetSwitch(RFO_LP, RFSWITCH_RX);
		SUBGRF_SetRxBoosted(0xFFFFFF);

		HAL_UART_Receive_DMA(&huart2, main_cmd, main_cmd_len);

	}

	if (radioIrq == IRQ_RX_DONE) {
		TX_FLAG = 0;
		SUBGRF_GetPayload(rx_buffer, &rx_buffer_len, RX_PAYLOAD_LENGTH);
		rssi_value = SUBGRF_GetRssiInst();

		uint8_t temp_rx_buffer_len = 0;
		temp_rx_buffer_len = countsDataBetweenFlags(rx_buffer, rx_buffer_len);

		if (temp_rx_buffer_len != -1) {

//			myDebug("\nSatellite Data Received, Length: %d and RSSI: %d dBm\r\n", temp_rx_buffer_len, rssi_value);

			uint8_t temp_check_buff[temp_rx_buffer_len];
			for (int i = 0; i < temp_rx_buffer_len; i++) {
				temp_check_buff[i] = rx_buffer[i];
//				myDebug("%02x ", temp_check_buff[i]);
			}
//			myDebug("\r\n");

//			memset(rx_buffer, '\0', sizeof(rx_buffer_len));

			uint8_t crc_buff_len = temp_rx_buffer_len - 4;
			uint8_t crc_buff[crc_buff_len];

//			myDebug("\nSatellite Data Testing: 0x%x\r\n");

			int j = 0;
			for (int i = 1; i <= crc_buff_len; i++) {
				crc_buff[j] = temp_check_buff[i];
//				myDebug("%02x ", crc_buff[j]);
				j++;
			}
//			myDebug("\r\n");

			uint16_t crc = 0;
			crc = calculateCRC_CCITT_AX25(crc_buff, crc_buff_len);

			uint8_t calc_crc[2];
			calc_crc[0] = (crc >> 8) & 0xFF;
			calc_crc[1] = crc & 0xFF;

			uint8_t msb_crc = temp_check_buff[temp_rx_buffer_len - 3];
			uint8_t lsb_crc = temp_check_buff[temp_rx_buffer_len - 2];

			if (calc_crc[0] == msb_crc && calc_crc[1] == lsb_crc) {
//				myDebug("Satellite Data checksum correct: 0x%x\r\n");

				uint8_t gs_cmd_buff[150];
				int gs_cmd_len = bit_destuffing(crc_buff, gs_cmd_buff,
						crc_buff_len);
				gs_cmd_len--;

				myDebug(
						"\nSatellite Real Data, Length: %d bytes  and RSSI: %d dBm\r\n",
						gs_cmd_len, rssi_value);

				uint8_t main_gs_cmd[gs_cmd_len];
				for (int i = 0; i < gs_cmd_len; i++) {
					main_gs_cmd[i] = gs_cmd_buff[i];
					myDebug("%02x ", main_gs_cmd[i]);
				}
				myDebug("\r\n");
				myDebug("__________\r\n");
				memset(main_gs_cmd, '\0', gs_cmd_len);
			} else {
				checksum_error_count++;
				myDebug(
						"Satellite Data checksum error: 0x%x and no of packets: %d\r\n",
						checksum_error_count);
				for (int i = 0; i < sizeof(temp_check_buff); i++) {
					myDebug("%02x ", temp_check_buff[i]);
				}
				myDebug("\r\n");
				myDebug("__________\r\n");
				memset(rx_buffer, '\0', sizeof(rx_buffer_len));
				memset(crc_buff, '\0', sizeof(crc_buff));
			}
		} else {
			myDebug("Satellite Data not completely received, Length: %d \r\n",
					sizeof(rx_buffer));
			for (int i = 0; i < sizeof(rx_buffer); i++) {
				myDebug("%02x ", rx_buffer[i]);
			}
			myDebug("\r\n");
			myDebug("__________\r\n");
			memset(rx_buffer, '\0', sizeof(rx_buffer_len));
		}

		SUBGRF_SetRfFrequency(FREQ_437_MHZ);
		SUBGRF_SetSwitch(RFO_LP, RFSWITCH_RX);
		SUBGRF_SetRxBoosted(0xFFFFFF);

		HAL_UART_Receive_DMA(&huart2, main_cmd, main_cmd_len);
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
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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
