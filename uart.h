/*
 * uart.h
 *
 *  Created on: 17.5.2013
 *      Author: Vincek
 */

#ifndef UART_H_
#define UART_H_

void UARTSend(const unsigned char *pucBuffer, unsigned long ulCount);
void UART0IntHandler(void);

#endif /* UART_H_ */
