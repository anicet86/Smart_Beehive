/*
 * uart2Tx_degug.h
 *
 *  Created on: Jan 6, 2026
 *      Author: kemazhu
 */

#ifndef UART2TX_DEBUG_H_
#define UART2TX_DEBUG_H_
#include "uart2Tx_debug.h"


void uart2_init(void);
int _write(int fd, char* ptr, int len);

#endif /* UART2TX_DEBUG_H_ */
