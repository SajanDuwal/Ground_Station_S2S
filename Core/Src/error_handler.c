/*
 * error_handler.c
 *
 *  Created on: Aug 29, 2024
 *      Author: sajanduwal
 */

#include "error_handler.h"

// Function to calculate CRC-CCITT for AX.25 frames
uint16_t calculateCRC_CCITT_AX25(const uint8_t *data, size_t length) {
	uint16_t crc = 0xFFFF; // Initialize CRC register with 0xFFFF
	uint16_t CRC_POLY = 0x1021; // CRC polynomial for CCITT (0x1021)

	// Iterate through each byte of the input data
	for (size_t i = 0; i < length; i++) {
		crc ^= ((uint16_t) data[i] << 8); // XOR CRC with next byte of input data

		// Iterate through each bit of the current byte
		for (int j = 0; j < 8; j++) {
			if (crc & 0x8000) { // If MSB of CRC is 1
				crc = (crc << 1) ^ CRC_POLY; // Left shift CRC and XOR with polynomial
			} else {
				crc <<= 1; // Left shift CRC
			}
		}
	}

	return crc; // Return calculated CRC
}

uint16_t calc_CRC(const uint8_t *data, size_t length) {

	uint16_t crcReg = 0xFFFF;	// Initialize the CRC register with 0xFFFF
	uint16_t calc = 0x8408;		// Polynomial for CRC-16
	uint16_t w;
	int i, j;
	uint8_t calc_data[length];  // in 16 bytes, 14 are data bytes

	// Copy data into calc_data
	for (i = 0; i < length; i++) {
		calc_data[i] = data[i];
		// Iterate over each byte of data
		for (j = 0; j < 8; j++) {
			w = (crcReg ^ calc_data[i]) & 0x0001; // XOR the LSB of crcReg with the LSB of calc_data
			crcReg = crcReg >> 1;			// Right-shift the crcReg by 1 bit
			if (w == 1) {
				crcReg = crcReg ^ calc;	// If w is 1, XOR the crcReg with the polynomial
			}
			calc_data[i] = calc_data[i] >> 1;// Right-shift the data byte by 1 bit
		}
	}
	crcReg = crcReg ^ 0xFFFF;						// Final XOR with 0xFFFF
	return crcReg;
}

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


int countsDataFromLastFlag(uint8_t *data, int data_length) {
	int end_index = -1;

	// Traverse the array from the end to find the last occurrence of 0x7e
	for (int i = data_length - 1; i >= 0; i--) {
		if (data[i] == 0x7e) {
			end_index = i;
			break;
		}
	}

	// If the last 0x7e is found, return the count from that index to the beginning
	if (end_index != -1) {
		return end_index + 1;
	} else {
		return -1; // Return -1 if no 0x7E flag is found
	}
}


int countsDataBeforeFirstSpace(uint8_t *data, int data_length) {
    int start_index = -1;

    // Traverse the array from the beginning to find the first occurrence of 0x20
    for (int i = 0; i < data_length; i++) {
        if (data[i] == 0x20) {
            start_index = i;
            break;
        }
    }

    // If the first 0x20 is found, return the count from that index to the end
    if (start_index != -1) {
        return start_index;
    } else {
        return -1; // Return -1 if no 0x20 flag is found
    }
}


int bit_stuffing(uint8_t *data, uint8_t *output_data, int length) {
	int out_index = 0;
	int bit_count = 0; // Count of consecutive 1 bits
	uint8_t current_byte = 0;
	int bit_pos = 7;
	int stuffed_size = 0; // Track size of output data after bit stuffing
	int bits_stuffed = 0; // Number of bits stuffed since last size increase

	//myDebug("Error handler: before Bit stuffing \n");

	for (int i = 0; i < length; i++) {
		for (int bit = 7; bit >= 0; bit--) {
			int bit_val = (data[i] >> bit) & 1;

//			myDebug("%d ", bit_val);

			current_byte |= (bit_val << bit_pos);
			bit_pos--;

			if (bit_val) {
				bit_count++;
				if (bit_count == 5) {
					// Insert a 0 bit after five consecutive 1s
					if (bit_pos < 0) {
						output_data[out_index++] = current_byte;
						stuffed_size++;
						current_byte = 0;
						bit_pos = 7;
					}
					current_byte &= ~(1 << bit_pos);
					bit_pos--;
					bits_stuffed++;
					bit_count = 0;
				}
			} else {
				bit_count = 0;
			}

			if (bit_pos < 0) {
				output_data[out_index++] = current_byte;
				stuffed_size++;
				current_byte = 0;
				bit_pos = 7;
			}

			// Check if we've stuffed enough bits to increase size
			if (bits_stuffed >= 9 && bit_pos >= 0) {
				stuffed_size++;
				bits_stuffed = 0; // Reset bits_stuffed after increasing size
			}
		}
	}
	//myDebug("\n");

	if (bit_pos < 7) {
		output_data[out_index++] = current_byte;
		stuffed_size++;
	}

	//myDebug("Error handler: After bit stuffing \n");

//	for (int i = 0; i < out_index; i++) {
//		for (int bit = 7; bit >= 0; bit--) {
//			int bit_val = (output_data[i] >> bit) & 1;
//
//			myDebug("%d ", bit_val);
//		}
//	}
//	myDebug("\n");

	for (int i = 0; i < length; i++) {
		output_data[i] = data[i];
	}

	out_index = length;

	return out_index; // Return the size of output data after bit stuffing
}

int bit_destuffing(uint8_t *data, uint8_t *output_data, int length) {
	int out_index = 0;
	int bit_count = 0;
	uint8_t current_byte = 0;
	int bit_pos = 7;

	for (int i = 0; i < length; i++) {
		for (int bit = 7; bit >= 0; bit--) {
			int bit_val = (data[i] >> bit) & 1;

			if (bit_val) {
				bit_count++;
				current_byte |= (bit_val << bit_pos);
				bit_pos--;
			} else {
				if (bit_count == 5) {
					// Skip this bit as it is a stuffed bit
					bit_count = 0;
					continue;
				} else {
					bit_count = 0;
					current_byte |= (bit_val << bit_pos);
					bit_pos--;
				}
			}

			if (bit_pos < 0) {
				output_data[out_index++] = current_byte;
				current_byte = 0;
				bit_pos = 7;
			}
		}
	}

	// Ensure the last byte is written if it's partially filled
	if (bit_pos < 7) {
		output_data[out_index++] = current_byte;
	}

//	myDebug("Error handler: After bit de-stuffing \n");
//
//	for (int i = 0; i < out_index; i++) {
//		for (int bit = 7; bit >= 0; bit--) {
//			int bit_val = (output_data[i] >> bit) & 1;
//
//			myDebug("%d ", bit_val);
//		}
//	}
//	myDebug("\n");

	for (int i = 0; i < length; i++) {
		output_data[i] = data[i];
	}

	out_index = length;

	return out_index;
}

uint8_t acciiToHex(uint8_t ascii) {
	if (ascii >= '0' && ascii <= '9') {
		return ascii - '0';
	} else if (ascii >= 'A' && ascii <= 'F') {
		return ascii - 'A' + 10;
	} else if (ascii >= 'a' && ascii <= 'f') {
		return ascii - 'a' + 10;
	} else {
		// Invalid character for hex conversion
		return 0xFF;
	}
}







