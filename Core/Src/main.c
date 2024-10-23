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
#include "dma.h"
#include "app_subghz_phy.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "radio_driver.h"
#include "com_debug.h"
#include "ax25_packet.h"
#include "digipeater_generator.h"
#include "error_handler.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

#define GS_CMD_LENGTH			(20) // source call sign 7, 13 bytes command

#define MAIN_CMD_LENGTH			(13)  //  main 13 bytes command
#define RX_PAYLOAD_LENGTH 		(105) //  maximum data receiving capacity from radio

#define MAIN_CMD_LENGTH_TEMP	(38)  //  main 38 bytes command ASCII

#define CMD_LENGTH_TEMP			(52)  //  main 39 bytes command received from UART2

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

/* USER CODE BEGIN PV */
PacketParams_t pkt_params;
ModulationParams_t mod_params;

uint8_t gs_cmd_len = GS_CMD_LENGTH;
uint8_t gs_cmd[GS_CMD_LENGTH];

uint8_t cmd_temp_len = CMD_LENGTH_TEMP;
uint8_t cmd_temp[CMD_LENGTH_TEMP];

uint8_t main_cmd_temp_len = MAIN_CMD_LENGTH_TEMP;
uint8_t main_cmd_temp[MAIN_CMD_LENGTH_TEMP];

uint8_t main_cmd_len = MAIN_CMD_LENGTH;
uint8_t main_cmd[MAIN_CMD_LENGTH];

uint8_t temp_tx_buffer[100];
int tx_buffer_len = 0;
uint8_t tx_buffer[100];

uint8_t temp_tx_dp_buffer[150];

uint8_t rx_buffer_len = RX_PAYLOAD_LENGTH;
uint8_t rx_buffer[RX_PAYLOAD_LENGTH];

int TX_FLAG = 0;
int rssi_value = 0;
int checksum_error_count = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void setPacketParams(uint8_t buffer_length);
void setModulationParams(unsigned long bitRate, unsigned long fDev);
void radioConfig(uint8_t *buffer, uint8_t buffer_len);
void DioIrqHndlr(RadioIrqMasks_t radioIrq);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart == &huart2) {

		uint8_t newLine = 0x0a;

		uint8_t header = 0x00;

		if (cmd_temp[0] == header || cmd_temp[0] == newLine) {

			for (int loop1 = 0; loop1 < sizeof(cmd_temp); loop1++) {
				cmd_temp[loop1] = cmd_temp[loop1 + 1];
			}
		}

		if (cmd_temp[0] == 0x4d && cmd_temp[1] == 0x4f &&     	//MO
				cmd_temp[2] == 0x44 && cmd_temp[3] == 0x45 &&  	// DE
				cmd_temp[4] == 0x54 && cmd_temp[5] == 0x45 &&  	 //TE
				cmd_temp[6] == 0x4c && cmd_temp[7] == 0x45) {			//LE

			TX_FLAG = 0;

			setPacketParams(rx_buffer_len);
			setModulationParams(GFSK_BR_4800, GFSK_FDEV_4800);
			radioConfig(rx_buffer, rx_buffer_len);

			myDebug("\n########## RX Configuration: ##########\n");

			myDebug("FREQUENCY MODS: Downlink FREQ: %lu Hz\r\n", FREQ_437_MHZ);
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

			SUBGRF_SetRfFrequency(FREQ_437_MHZ);
			SUBGRF_SetSwitch(RFO_LP, RFSWITCH_RX);
			SUBGRF_SetRxBoosted(0xFFFFFF);

			HAL_UART_Receive_DMA(&huart2, cmd_temp, cmd_temp_len);

		} else if (cmd_temp[0] == 0x4d && cmd_temp[1] == 0x4f &&     //MO
				cmd_temp[2] == 0x44 && cmd_temp[3] == 0x45 &&  	// DE
				cmd_temp[4] == 0x44 && cmd_temp[5] == 0x49 &&   //DI
				cmd_temp[6] == 0x47 && cmd_temp[7] == 0x49) {		//GI

			TX_FLAG = 0;

			setPacketParams(rx_buffer_len);
			setModulationParams(GFSK_BR_1200, GFSK_FDEV_1200);
			radioConfig(rx_buffer, rx_buffer_len);

			myDebug("\n########## RX Configuration: ##########\n");

			myDebug("FREQUENCY MODS: Downlink FREQ: %lu Hz\r\n", FREQ_437_MHZ);
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

			SUBGRF_SetRfFrequency(FREQ_437_MHZ);
			SUBGRF_SetSwitch(RFO_LP, RFSWITCH_RX);
			SUBGRF_SetRxBoosted(0xFFFFFF);

			HAL_UART_Receive_DMA(&huart2, cmd_temp, cmd_temp_len);

		} else if (cmd_temp[0] == 0x53 && cmd_temp[1] == 0x32
				&& cmd_temp[2] == 0x53) {								 //S2S

			TX_FLAG = 0;

			// Section 1: dp_head (first 3 bytes)
			unsigned char dp_head[4];
			strncpy((char*) dp_head, (char*) cmd_temp, 3);
			dp_head[3] = '\0'; // Null-terminate the string

			// Section 2: destination_callsign (from byte 4 to the byte before '>' or 0x3e)
			unsigned char *dest_end = (unsigned char*) strchr(
					(char*) cmd_temp + 3, 0x3e); // Find the '>' (0x3e)
			int dest_size = dest_end - (cmd_temp + 3); // Get the size excluding '>'
			// Ensure the destination_callsign is exactly 7 bytes
			unsigned char destination_callsign[8]; // Array size of 8 to include 7 bytes plus null terminator
			// Copy the actual destination_callsign data
			strncpy((char*) destination_callsign, (char*) cmd_temp + 3,
					dest_size);
			// Shift each element of destination_callsign by 1 bit to the left
			for (int i = 0; i < dest_size; i++) {
				destination_callsign[i] <<= 1;  // Perform left shift by 1 bit
			}
			// If dest_size is less than 7, pad with 0x40 until it reaches 6 bytes, then set the last byte to 0xe0
			if (dest_size < 7) {
				for (int i = dest_size; i < 7; i++) {
					destination_callsign[i] = 0x40;  // Padding with 0x40
				}
				destination_callsign[6] = 0xE0;  // Set the last byte to 0xe0
			}
			destination_callsign[7] = '\0'; // Null-terminate (not strictly necessary for binary data, but good practice)

			// Section 3: source_callsign (from '>' to the byte before ',' or 0x2C)
			unsigned char *src_start = dest_end + 1; // Start after '>'
			unsigned char *src_end = (unsigned char*) strchr((char*) src_start,
					0x2C); // Find the ',' (0x2c)
			int src_size = src_end - src_start; // Get the size
			// Ensure the source_callsign is exactly 7 bytes
			unsigned char source_callsign[8]; // Array size of 8 to include 7 bytes plus null terminator
			// Copy the actual source_callsign data
			strncpy((char*) source_callsign, (const char*) src_start, src_size);
			// Shift each element of source_callsign by 1 bit to the left
			for (int i = 0; i < src_size; i++) {
				source_callsign[i] <<= 1;  // Perform left shift by 1 bit
			}
			// If src_size is less than 7, pad with 0x40 until it reaches 6 bytes, then set the last byte to 0xe0
			if (src_size < 7) {
				for (int i = src_size; i < 7; i++) {
					source_callsign[i] = 0x40;  // Padding with 0x40
				}
				source_callsign[6] = 0xE0;  // Set the last byte to 0xe0
			}
			source_callsign[7] = '\0'; // Null-terminate (optional for binary data)

			// Section 4: packet_path (from ',' to the byte before ':' or 0x3A)
			unsigned char *path_start = src_end + 1; // Start after ','
			unsigned char *path_end = (unsigned char*) strchr(
					(char*) path_start, 0x3A); // Find the ':' (0x3a)
			int path_size = path_end - path_start; // Get the size
			unsigned char packet_path[path_size + 1];
			strncpy((char*) packet_path, (const char*) path_start, path_size);
			packet_path[path_size] = '\0'; // Null-terminate

			// Section 5: message_field (from ':' to before '20 37 65')
			unsigned char *msg_start = path_end + 1; // Start after ':'
			unsigned char *msg_end = (unsigned char*) strstr((char*) msg_start,
					"\x20\x37\x65"); // Find '20 37 65'
			int msg_size = msg_end - msg_start; // Get the size before '20 37 65'
			unsigned char message_field[msg_size + 1];
			strncpy((char*) message_field, (const char*) msg_start, msg_size);
			message_field[msg_size] = '\0'; // Null-terminate

			// Combine all sections into one array
			int total_size = 3 + 7 + 7 + 7 + msg_size; // Calculate total size
			uint8_t final_packet[total_size + 1]; // Add 1 for null termination (if needed for debug)
			int offset = 0;

			// Copy Section 1: dp_head
			memcpy(final_packet + offset, dp_head, 3);
			offset += 3;

			// Copy Section 2: destination_callsign
			memcpy(final_packet + offset, destination_callsign, 7);
			offset += 7;

			// Copy Section 3: source_callsign
			memcpy(final_packet + offset, source_callsign, 7);
			offset += 7;

			// Copy Section 4: packet_path
			memcpy(final_packet + offset, packet_path, 7);
			offset += 7;

			// Copy Section 5: message_field
			memcpy(final_packet + offset, message_field, msg_size);
			offset += msg_size;

			final_packet[total_size] = '\0'; // Null-terminate for safety (optional for binary data)

			// Print the entire final packet in hex format
			myDebug("Digipeater Packet receive from DP-Station:\n");
			for (int i = 0; i < total_size; i++) {
				myDebug("%02x ", final_packet[i]);
			}
			myDebug("\n");

			getDigipeaterPacket(final_packet, msg_size);

			tx_buffer_len = countsDataBetweenFlags(temp_tx_dp_buffer,
					sizeof(temp_tx_dp_buffer));

			memset(tx_buffer, '\0', tx_buffer_len);

//			myDebug("Digipeater Packet complete, ready to TX: 0x%x\r\n",
//					temp_tx_dp_buffer);
			for (int j = 0; j < tx_buffer_len; j++) {
				tx_buffer[j] = temp_tx_dp_buffer[j];
//				myDebug("%02x ", tx_buffer[j]);
			}
//			myDebug("\r\n");
//			myDebug("size of tx_buffer = %d\r\n", tx_buffer_len);

			memset(cmd_temp, '\0', sizeof(cmd_temp));
			memset(temp_tx_dp_buffer, '\0', sizeof(temp_tx_dp_buffer));

			delay_us(500000);

			setPacketParams(tx_buffer_len);
			setModulationParams(GFSK_BR_1200,
			GFSK_FDEV_1200);
			radioConfig(tx_buffer, tx_buffer_len);

			myDebug("\n########## Digipeater TX Configuration: ##########\n");

			myDebug("FREQUENCY MODS: DOWNLINK FREQ: %lu Hz\r\n",
			FREQ_437_MHZ);
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
			myDebug("__________*******************__________\r\n");

			SUBGRF_SetRfFrequency(FREQ_437_MHZ);
			SUBGRF_SetSwitch(RFO_LP, RFSWITCH_TX);
			SUBGRF_SendPayload(tx_buffer, tx_buffer_len, 0);

		} else {

			uint8_t source_call_sign_temp[13];
			int a = 0;
			for (int loop2 = 39; loop2 < 52; loop2++) {
				source_call_sign_temp[a] = cmd_temp[loop2];
//				myDebug("%02x ", source_call_sign_temp[a]);
				a++;
			}
//			myDebug("\r\n");

			int source_call_sign_len_temp = countsDataBeforeFirstSpace(
					source_call_sign_temp, sizeof(source_call_sign_temp));

			int source_call_sign_len = source_call_sign_len_temp;

//			myDebug("length of call sign %d\n", source_call_sign_len_temp);

			for (int j = 0; j < source_call_sign_len_temp; j++) {
				// Check if the current element is 0x2D
				if (source_call_sign_temp[j] == 0x2D) {
					--source_call_sign_len;
					continue; // Skip this element
				}
			}

			uint8_t source_call_sign[source_call_sign_len];

			int l = 0; // Index for source_call_sign array

			for (int j = 0; j < source_call_sign_len_temp; j++) {
				// Check if the current element is 0x2D
				if (source_call_sign_temp[j] == 0x2D) {
					continue; // Skip this element
				}

				// Copy the next valid element to source_call_sign[k]
				source_call_sign[l] = source_call_sign_temp[j] << 1;

//				myDebug("%02x ", source_call_sign[l]);

				l++;
			}
//			myDebug("\n");

			for (int loop2 = 0; loop2 < sizeof(main_cmd_temp); loop2++) {
				main_cmd_temp[loop2] = cmd_temp[loop2];
//				myDebug("%02x ", main_cmd_temp[loop2]);
			}

			int input_length = sizeof(main_cmd_temp) / sizeof(main_cmd_temp[0]);

			// Temporary array to store non-space characters
			uint8_t temp_chars[input_length];
			int temp_count = 0;

			// Filter out space characters (ASCII 0x20)
			for (int i = 0; i < input_length; i++) {
				if (main_cmd_temp[i] != 0x20) {
					temp_chars[temp_count++] = main_cmd_temp[i];
				}
			}

			// Calculate the number of bytes
			int byte_count = temp_count / 2;

			// Output byte array
			uint8_t byte_array[byte_count];

			// Convert pairs of characters to bytes
			for (int i = 0; i < byte_count; i++) {
				uint8_t high_nibble = acciiToHex(temp_chars[2 * i]);
				uint8_t low_nibble = acciiToHex(temp_chars[2 * i + 1]);

				byte_array[i] = (high_nibble << 4) | low_nibble;
			}

			myDebug("\n-->Main command Received: 0x%x\r\n", main_cmd);

			if (sizeof(byte_array) == main_cmd_len) {
				//			myDebug("-->Command ACK: 0x%x\r\n", main_cmd);
				for (int i = 0; i < main_cmd_len; i++) {
					main_cmd[i] = byte_array[i];
//					myDebug("%02x ", main_cmd[i]);
				}
//				myDebug("\r\n");

				int k = 0;
				for (int i = 0; i < main_cmd_len; i++) {
					gs_cmd[k] = main_cmd[i];
					myDebug("%02x ", gs_cmd[k]);
					k++;
				}
				myDebug("\n");

				for (int i = 0; i < source_call_sign_len; i++) {
					gs_cmd[k] = source_call_sign[i];
					myDebug("%02x ", gs_cmd[k]);
					k++;
				}
				myDebug("\n");

				// Check if k is less than 20, and fill remaining bytes
				if (k < 20) {
					// Fill the remaining bytes with 0x40
					for (int i = k; i < 19; i++) {
						gs_cmd[i] = 0x40;
					}

					// Set the last byte to 0x61
					gs_cmd[19] = 0x61;
				}

//				for (int i = 0; i < gs_cmd_len; i++) {
//					myDebug("%02x ", gs_cmd[i]);
//				}
//				myDebug("\r\n");

				TX_FLAG = 1;

			} else {
				myDebug("-->Command Not ACK: 0x%x\r\n", main_cmd);

				for (int i = 0; i < main_cmd_len; i++) {
					myDebug("%02x ", main_cmd[i]);
				}
				myDebug("\r\n");

				memset(main_cmd, '\0', main_cmd_len);
				TX_FLAG = 0;
			}
		}

	}
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
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
	MX_USART1_UART_Init();
	/* USER CODE BEGIN 2 */

	HAL_TIM_Base_Start(&htim2);

	SUBGRF_Init(DioIrqHndlr);
	SUBGRF_SetDioIrqParams(
			IRQ_TX_DONE | IRQ_PREAMBLE_DETECTED | IRQ_RX_DONE
					| IRQ_RX_TX_TIMEOUT | IRQ_SYNCWORD_VALID,
			IRQ_TX_DONE | IRQ_PREAMBLE_DETECTED | IRQ_RX_DONE
					| IRQ_RX_TX_TIMEOUT | IRQ_SYNCWORD_VALID, IRQ_RADIO_NONE,
			IRQ_RADIO_NONE);

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

	HAL_UART_Receive_DMA(&huart2, cmd_temp, cmd_temp_len);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */
		MX_SubGHz_Phy_Process();

		/* USER CODE BEGIN 3 */

		delay_us(500000);

		if (TX_FLAG) {

			getAX25Packet(gs_cmd, gs_cmd_len);

			tx_buffer_len = countsDataFromLastFlag(temp_tx_buffer,
					sizeof(temp_tx_buffer));

//			myDebug("AX.25 complete GS packet ready to TX: 0x%x\r\n", temp_tx_buffer);
			for (int j = 0; j < tx_buffer_len; j++) {
				tx_buffer[j] = temp_tx_buffer[j];
//				myDebug("%02x ", tx_buffer[j]);
			}
//			myDebug("\r\n");

//			myDebug("size of tx_buffer = %d\r\n", tx_buffer_len);

			memset(main_cmd, '\0', main_cmd_len);
			memset(gs_cmd, '\0', gs_cmd_len);
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

		HAL_UART_Receive_DMA(&huart2, cmd_temp, cmd_temp_len);

	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

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
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Configure the SYSCLKSource, HCLK, PCLK1 and PCLK2 clocks dividers
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK3 | RCC_CLOCKTYPE_HCLK
			| RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.AHBCLK3Divider = RCC_SYSCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
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

		HAL_UART_Receive_DMA(&huart2, cmd_temp, cmd_temp_len);

	}

	if (radioIrq == IRQ_RX_DONE) {
		TX_FLAG = 0;
		memset(rx_buffer, '\0', sizeof(rx_buffer));
		SUBGRF_GetPayload(rx_buffer, &rx_buffer_len, RX_PAYLOAD_LENGTH);
		rssi_value = SUBGRF_GetRssiInst();

		uint8_t temp_rx_buffer_len = 0;
		temp_rx_buffer_len = countsDataFromLastFlag(rx_buffer, rx_buffer_len);

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
//				gs_cmd_len--;

				myDebug(
						"\nSatellite Real Data, Length: %d bytes  and RSSI: %d dBm\r\n",
						gs_cmd_len, rssi_value);

				uint8_t main_gs_cmd[gs_cmd_len];
				for (int i = 0; i < gs_cmd_len; i++) {
					main_gs_cmd[i] = gs_cmd_buff[i];
					myDebug("%02x ", main_gs_cmd[i]);
				}

				HAL_UART_Transmit(&huart2, main_gs_cmd, sizeof(main_gs_cmd),
						2000);

				myDebug("\r\n");
				myDebug("__________\r\n");
				memset(main_gs_cmd, '\0', gs_cmd_len);

			} else {
				checksum_error_count++;
				myDebug(
						"Satellite Data checksum error and no of error packets: %d\r\n",
						checksum_error_count);
				for (int i = 0; i < sizeof(temp_check_buff); i++) {
					myDebug("%02x ", temp_check_buff[i]);
				}

				HAL_UART_Transmit(&huart2, temp_check_buff,
						sizeof(temp_check_buff), 2000);

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

			HAL_UART_Transmit(&huart2, rx_buffer, sizeof(rx_buffer), 2000);

			myDebug("\r\n");
			myDebug("__________\r\n");
			memset(rx_buffer, '\0', sizeof(rx_buffer_len));
		}

		SUBGRF_SetRfFrequency(FREQ_437_MHZ);
		SUBGRF_SetSwitch(RFO_LP, RFSWITCH_RX);
		SUBGRF_SetRxBoosted(0xFFFFFF);

		HAL_UART_Receive_DMA(&huart2, cmd_temp, cmd_temp_len);
	}
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
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
