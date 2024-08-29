/*
 * com_debug.h
 *
 *  Created on: Aug 29, 2024
 *      Author: sajanduwal
 */

#ifndef INC_COM_DEBUG_H_
#define INC_COM_DEBUG_H_

#include "main.h"
#include "stdarg.h"
#include "stdio.h"
#include "string.h"

extern UART_HandleTypeDef huart2;

extern TIM_HandleTypeDef htim2;

void myDebug(const char *fmt, ...);

int bufferSize(char *buffer);

void delay_us(uint32_t us);


#endif /* INC_COM_DEBUG_H_ */
