/*
 * usart.h
 *
 *  Created on: May 30, 2014
 *      Author: lim
 */
#ifndef USART_H_
#define USART_H_

#define BAUDRATE 57600
#define BAUDDIVIDER (F_CPU/(16*BAUDRATE)-1)

#define BAUDRATE_115200 115200
#define BAUDDIVIDER_115200 (F_CPU/(16*BAUDRATE_115200)-1) //7.68 for 16MHz
#define BAUDRATE_57600 57600
#define BAUDDIVIDER_57600 (F_CPU/(16*BAUDRATE_57600)-1) //16.36
#define BAUDRATE_38400 38400
#define BAUDDIVIDER_38400 (F_CPU/(16*BAUDRATE_38400)-1) //25.04
#define BAUDRATE_31250 31250 //MIDI
#define BAUDDIVIDER_31250 (F_CPU/(16*BAUDRATE_31250)-1) //31
#define BAUDRATE_19200 19200
#define BAUDDIVIDER_19200 (F_CPU/(16*BAUDRATE_19200)-1) //51.08
#define BAUDRATE_9600 9600
#define BAUDDIVIDER_9600 (F_CPU/(16*BAUDRATE_9600)-1) //103.16

#define USART_TX_BUF 32
#define USART_RX_BUF 32

void usart_init(void);
void usart_putchr(char ch);
char usart_chrready(void);
char usart_getchr(void);
void usart_printstr(char *string);
char usart_chartohex(char c);
char usart_hextochar(char c);
void usart_printhex(char c);

#endif /* USART_H_ */
