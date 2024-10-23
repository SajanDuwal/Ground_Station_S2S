/*
 * ax25_packet.c
 *
 *  Created on: Aug 29, 2024
 *      Author: sajanduwal
 */



#include "ax25_packet.h"

extern uint8_t temp_tx_buffer[100];

uint8_t ax_buffer[100] = { 0 };
uint8_t ax_buff_stuffed[100] = { 0 };

void getHDLCPacket(uint8_t *data, uint8_t data_len) {

//	myDebug("After Bit stuffing: \n");
//	for (int j = 0; j < data_len; j++) {
//		myDebug("%02x ", data[j]);
//	}
//	myDebug("\r\n");
//	myDebug("length of stuffed_packet: %d\r\n", data_len);

	uint16_t crc = 0;
	crc = calculateCRC_CCITT_AX25(data, data_len);

	temp_tx_buffer[0] = 0x7e;

	int i = 1;
	for (int k = 0; k < data_len; k++) {
		temp_tx_buffer[i] = data[k];
		i++;
	}

	// Store CRC result in the packet array (from packet[1] to end of for loop)
	temp_tx_buffer[i] = (crc >> 8) & 0xFF; // Most significant byte
	i++;
	temp_tx_buffer[i] = crc & 0xFF;        // Least significant byte
	i++;

	temp_tx_buffer[i] = 0x7e;

//	myDebug("AX.25 complete packet: 0x%x\r\n", temp_tx_buffer);
//	for (int j = 0; j <= i; j++) {
//		myDebug("%02x ", temp_tx_buffer[j]);
//	}
//	myDebug("\r\n");
//	myDebug("size of tx_buffer = %d\r\n", i + 1);

	memset(ax_buffer, '\0', sizeof(ax_buffer));
	memset(ax_buff_stuffed, '\0', sizeof(ax_buff_stuffed));

}

void getAX25Packet(uint8_t *infoField, uint8_t size) {

	//destination field   9N2SI -0
	ax_buffer[0] = 0x72; //	ASCII->9, HEX->0x39, 1 bit shifted->0b01110010, Shifted HEX->0x72
	ax_buffer[1] = 0x9C; //	ASCII->N, HEX->0x4E, 1 bit shifted->0b10011100, Shifted HEX->0x9C
	ax_buffer[2] = 0x64; //	ASCII->2, HEX->0x32, 1 bit shifted->0b01100100, Shifted HEX->0x64
	ax_buffer[3] = 0xA6; //	ASCII->S, HEX->0x53, 1 bit shifted->0b10100110, Shifted HEX->0xA6
	ax_buffer[4] = 0x92; //	ASCII->I, HEX->0x49, 1 bit shifted->0b10010010, Shifted HEX->0x92
	ax_buffer[5] = 0x40; //	ASCII-> , HEX->0x20, 1 bit shifted->0b01000000, Shifted HEX->0x40
	ax_buffer[6] = 0xE0; //	0b111SSID0, SSID->0000 and LSB is set to 0; if more addresses follow, HEX->0xE0

	//source field   9N2SI -0
	ax_buffer[7] = infoField[13]; //	ASCII->9, HEX->0x39, 1 bit shifted->0b01110010, Shifted HEX->0x72
	ax_buffer[8] = infoField[14]; //	ASCII->N, HEX->0x4E, 1 bit shifted->0b10011100, Shifted HEX->0x9C
	ax_buffer[9] = infoField[15]; //	ASCII->2, HEX->0x32, 1 bit shifted->0b01100100, Shifted HEX->0x64
	ax_buffer[10] = infoField[16]; //	ASCII->S, HEX->0x53, 1 bit shifted->0b10100110, Shifted HEX->0xA6
	ax_buffer[11] = infoField[17]; //	ASCII->I, HEX->0x49, 1 bit shifted->0b10010010, Shifted HEX->0x92
	ax_buffer[12] = infoField[18]; //	ASCII-> , HEX->0x20, 1 bit shifted->0b01000000, Shifted HEX->0x40
	ax_buffer[13] = infoField[19]; //	0b011SSID1, SSID->0000 and LSB is set to 1 if this is the last address, HEX->0x61

	//control bit
	ax_buffer[14] = 0x03; //  Unnumbered Information Frame, AX.25 is always 0b00000011 i.e 0x03 in HEX

	//protocol identifier
	ax_buffer[15] = 0xF0; //	No Layer-3 Implemented so, 0b11110000 i.e 0xF0 in HEX

	int i = 16;

	for (int k = 0; k < size-7; k++) {
		ax_buffer[i] = infoField[k];
		i++;
	}

//	myDebug("Before Bit stuffing:");
//	myDebug("\nReal Command, Length: %d \r\n", i);
//	for (int j = 0; j < i; j++) {
//		myDebug("%02x ", ax_buffer[j]);
//	}
//	myDebug("\r\n");

	// Bit Stuffing
	int stuffed_size = bit_stuffing(ax_buffer, ax_buff_stuffed, i);

//	uint8_t tem[100];

//	int destuffed_size = bit_destuffing(ax_buff_stuffed, tem, stuffed_size);
//	myDebug("De-stuffing: \n");
//	for (int j = 0; j < destuffed_size-1; j++) {
//		myDebug("%02x ", tem[j]);
//	}
//	myDebug("\r\n");
//	myDebug("length of de-stuffed_packet: %d\r\n", destuffed_size);

	getHDLCPacket(ax_buff_stuffed, stuffed_size);
}
