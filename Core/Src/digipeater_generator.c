/*
 * digipeater_generator.c
 *
 *  Created on: Oct 13, 2024
 *      Author: sajanduwal
 */


#include "digipeater_generator.h"

#define INFO__DP_LENGTH		(100)

extern uint8_t temp_tx_dp_buffer[150];

uint8_t dp_buffer[150] = { 0 };
uint8_t dp_buff_stuffed[150] = { 0 };

uint8_t info_dp_packet[INFO__DP_LENGTH];

void getHDLCDigipeaterPacket(uint8_t *data, uint8_t data_len) {

//	myDebug("After Bit stuffing: \n");
//	myDebug("length of stuffed_packet: %d\r\n", data_len);
//	for (int j = 0; j < data_len; j++) {
//		myDebug("%02x ", data[j]);
//	}
//	myDebug("\r\n");

	uint16_t crc = 0;
	crc = calculateCRC_CCITT_AX25(data, data_len);

	temp_tx_dp_buffer[0] = 0x7e;

	int i = 1;
	for (int k = 0; k < data_len; k++) {
		temp_tx_dp_buffer[i] = data[k];
		i++;
	}

	// Store CRC result in the packet array
	temp_tx_dp_buffer[i] = (crc >> 8) & 0xFF; // Most significant byte
	i++;
	temp_tx_dp_buffer[i] = crc & 0xFF;        // Least significant byte
	i++;

	temp_tx_dp_buffer[i] = 0x7e;

//	myDebug("Digipeater complete packet: 0x%x\r\n", temp_tx_dp_buffer);
//	for (int j = 0; j <= i; j++) {
//		myDebug("%02x ", temp_tx_dp_buffer[j]);
//	}
//	myDebug("\r\n");
//	myDebug("size of tx_dp_buffer = %d\r\n", i + 1);

	//testing

//	uint8_t tem[150];
//	int destuffed_size = bit_destuffing(dp_buff_stuffed, tem, data_len);
//	myDebug("De-stuffing: \n");
//	myDebug("length of de-stuffed_packet: %d\r\n", destuffed_size - 1);
//	for (int j = 0; j < destuffed_size - 1; j++) {
//		myDebug("%02x ", tem[j]);
//	}
//	myDebug("\r\n");

	//..

	memset(dp_buffer, '\0', sizeof(dp_buffer));
	memset(dp_buff_stuffed, '\0', sizeof(dp_buff_stuffed));
}

void getDigipeaterPacket(uint8_t *OBC_digipeater_datad_Field, uint8_t size_of_msg_field) {

	//destination field
	dp_buffer[0] = OBC_digipeater_datad_Field[3]; //	ASCII-> , HEX->0x20, 1 bit shifted->0b01000000, Shifted HEX->0x40
	dp_buffer[1] = OBC_digipeater_datad_Field[4]; //	ASCII-> , HEX->0x20, 1 bit shifted->0b01000000, Shifted HEX->0x40
	dp_buffer[2] = OBC_digipeater_datad_Field[5]; //	ASCII-> , HEX->0x20, 1 bit shifted->0b01000000, Shifted HEX->0x40
	dp_buffer[3] = OBC_digipeater_datad_Field[6]; //	ASCII-> , HEX->0x20, 1 bit shifted->0b01000000, Shifted HEX->0x40
	dp_buffer[4] = OBC_digipeater_datad_Field[7]; //	ASCII-> , HEX->0x20, 1 bit shifted->0b01000000, Shifted HEX->0x40
	dp_buffer[5] = OBC_digipeater_datad_Field[8]; //	ASCII-> , HEX->0x20, 1 bit shifted->0b01000000, Shifted HEX->0x40
	dp_buffer[6] = OBC_digipeater_datad_Field[9]; //	0b111SSID0, SSID->0000 and LSB is set to 0; if more addresses follow, HEX->0xE0

	//source field   9N2SI -0
	dp_buffer[7] = OBC_digipeater_datad_Field[10]; //	ASCII->9, HEX->0x39, 1 bit shifted->0b01110010, Shifted HEX->0x72
	dp_buffer[8] = OBC_digipeater_datad_Field[11]; //	ASCII->N, HEX->0x4E, 1 bit shifted->0b10011100, Shifted HEX->0x9C
	dp_buffer[9] = OBC_digipeater_datad_Field[12]; //	ASCII->2, HEX->0x32, 1 bit shifted->0b01100100, Shifted HEX->0x64
	dp_buffer[10] = OBC_digipeater_datad_Field[13]; //	ASCII->S, HEX->0x53, 1 bit shifted->0b10100110, Shifted HEX->0xA6
	dp_buffer[11] = OBC_digipeater_datad_Field[14]; //	ASCII->I, HEX->0x49, 1 bit shifted->0b10010010, Shifted HEX->0x92
	dp_buffer[12] = OBC_digipeater_datad_Field[15]; //	ASCII-> , HEX->0x20, 1 bit shifted->0b01000000, Shifted HEX->0x40
	dp_buffer[13] = OBC_digipeater_datad_Field[16]; //	0b111SSID0, SSID->0000 and LSB is set to 0; if more addresses follow, HEX->0xE0

	// dp initiator call sign
	dp_buffer[14] = OBC_digipeater_datad_Field[17]; //
	dp_buffer[15] = OBC_digipeater_datad_Field[18]; //
	dp_buffer[16] = OBC_digipeater_datad_Field[19]; //
	dp_buffer[17] = OBC_digipeater_datad_Field[20]; //
	dp_buffer[18] = OBC_digipeater_datad_Field[21]; //
	dp_buffer[19] = OBC_digipeater_datad_Field[22]; //
	dp_buffer[20] = OBC_digipeater_datad_Field[23]; //

	//control bit
	dp_buffer[21] = 0x03; //  Unnumbered Information Frame, AX.25 is always 0b00000011 i.e 0x03 in HEX

	//protocol identifier
	dp_buffer[22] = 0xF0; //	No Layer-3 Implemented so, 0b11110000 i.e 0xF0 in HEX

	// S2S Digipeater Identity
	dp_buffer[23] = OBC_digipeater_datad_Field[0]; //	ASCII-> S
	dp_buffer[24] = OBC_digipeater_datad_Field[1]; //	ASCII-> 2
	dp_buffer[25] = OBC_digipeater_datad_Field[2]; //	ASCII-> S
	dp_buffer[26] = 0x44; //	ASCII-> D
	dp_buffer[27] = 0x50; //	ASCII-> P
	dp_buffer[28] = 0x41; //	ASCII-> A
	dp_buffer[29] = 0x50; //	ASCII-> P
	dp_buffer[30] = 0x4E; //	ASCII-> N

	int i = 31;
	int j = 24;
	for (int m = 0; m < size_of_msg_field; m++) {
		dp_buffer[i] = OBC_digipeater_datad_Field[j];
		i++;
		j++;
	}

//	myDebug("Before Bit stuffing:");
//	myDebug("\nReal Data, Length: %d bytes", i);
//	myDebug("\r\n");
//	for (int j = 0; j < i; j++) {
//		myDebug("%02x ", dp_buffer[j]);
//	}
//	myDebug("\r\n");

	// Bit Stuffing
	int stuffed_size = bit_stuffing(dp_buffer, dp_buff_stuffed, i);

	getHDLCDigipeaterPacket(dp_buff_stuffed, stuffed_size);
}
