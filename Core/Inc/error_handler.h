/*
 * error_handler.h
 *
 *  Created on: Aug 29, 2024
 *      Author: sajanduwal
 */

#ifndef INC_ERROR_HANDLER_H_
#define INC_ERROR_HANDLER_H_

#include "main.h"
#include "com_debug.h"

uint16_t calculateCRC_CCITT_AX25(const uint8_t *data, size_t length);

uint16_t calc_CRC(const uint8_t *data, size_t length);

int countsDataBetweenFlags(uint8_t *data, int data_length);

int countsDataFromLastFlag(uint8_t *data, int data_length);

int countsDataBeforeFirstSpace(uint8_t *data, int data_length);

int bit_stuffing(uint8_t *data, uint8_t *output_data, int length);

int bit_destuffing(uint8_t *data, uint8_t *output_data, int length);

uint8_t acciiToHex(uint8_t ascii);

#endif /* INC_ERROR_HANDLER_H_ */
