/* ports.h
 *
 *  Created on: Sep 26, 2014
 *      Author: lim
 */

#ifndef PORTS_H_
#define PORTS_H_

#define PORT_A0 PORTA,DDRA,PINA,PA0
#define PORT_A1 PORTA,DDRA,PINA,PA1
#define PORT_A2 PORTA,DDRA,PINA,PA2
#define PORT_A3 PORTA,DDRA,PINA,PA3
#define PORT_A4 PORTA,DDRA,PINA,PA4
#define PORT_A5 PORTA,DDRA,PINA,PA5
#define PORT_A6 PORTA,DDRA,PINA,PA6
#define PORT_A7 PORTA,DDRA,PINA,PA7

#define PORT_B0 PORTB,DDRB,PINB,PB0
#define PORT_B1 PORTB,DDRB,PINB,PB1
#define PORT_B2 PORTB,DDRB,PINB,PB2
#define PORT_B3 PORTB,DDRB,PINB,PB3
#define PORT_B4 PORTB,DDRB,PINB,PB4
#define PORT_B5 PORTB,DDRB,PINB,PB5
#define PORT_B6 PORTB,DDRB,PINB,PB6
#define PORT_B7 PORTB,DDRB,PINB,PB7

#define PORT_C0 PORTC,DDRC,PINC,PC0
#define PORT_C1 PORTC,DDRC,PINC,PC1
#define PORT_C2 PORTC,DDRC,PINC,PC2
#define PORT_C3 PORTC,DDRC,PINC,PC3
#define PORT_C4 PORTC,DDRC,PINC,PC4
#define PORT_C5 PORTC,DDRC,PINC,PC5
#define PORT_C6 PORTC,DDRC,PINC,PC6
#define PORT_C7 PORTC,DDRC,PINC,PC7

#define PORT_D0 PORTD,DDRD,PIND,PD0
#define PORT_D1 PORTD,DDRD,PIND,PD1
#define PORT_D2 PORTD,DDRD,PIND,PD2
#define PORT_D3 PORTD,DDRD,PIND,PD3
#define PORT_D4 PORTD,DDRD,PIND,PD4
#define PORT_D5 PORTD,DDRD,PIND,PD5
#define PORT_D6 PORTD,DDRD,PIND,PD6
#define PORT_D7 PORTD,DDRD,PIND,PD7

#define PORT_E0 PORTE,DDRE,PINE,PE0
#define PORT_E1 PORTE,DDRE,PINE,PE1
#define PORT_E2 PORTE,DDRE,PINE,PE2
#define PORT_E3 PORTE,DDRE,PINE,PE3
#define PORT_E4 PORTE,DDRE,PINE,PE4
#define PORT_E5 PORTE,DDRE,PINE,PE5
#define PORT_E6 PORTE,DDRE,PINE,PE6
#define PORT_E7 PORTE,DDRE,PINE,PE7

#define PORT_F0 PORTF,DDRF,PINF,PF0
#define PORT_F1 PORTF,DDRF,PINF,PF1
#define PORT_F2 PORTF,DDRF,PINF,PF2
#define PORT_F3 PORTF,DDRF,PINF,PF3
#define PORT_F4 PORTF,DDRF,PINF,PF4
#define PORT_F5 PORTF,DDRF,PINF,PF5
#define PORT_F6 PORTF,DDRF,PINF,PF6
#define PORT_F7 PORTF,DDRF,PINF,PF7

#if defined(__AVR_ATtiny13__)
#define PORTA_D8 PORT_B0 //MOSI/AIN0/OC0A/PCINT0
#define PORTA_D9 PORT_B1 //MISO/AIN1/OC0B/INT0/PCINT1
#define PORTA_D10 PORT_B2 //SCK/ADC1/T0/PCINT2
#define PORTA_D11 PORT_B3 //PCINT3/CLKI/ADC3
#define PORTA_D12 PORT_B4 //PCINT4/ADC2
#define PORTA_D13 PORT_B5 //PCINT5/RESET/ADC0/dW

#elif defined(__AVR_ATmega32U4__)
#define PORTA_D0 PORT_D2
#define PORTA_D1 PORT_D3
#define PORTA_D2 PORT_D1
#define PORTA_D3 PORT_D0
#define PORTA_D4 PORT_D4
#define PORTA_D5 PORT_C6
#define PORTA_D6 PORT_D7
#define PORTA_D7 PORT_E6
#define PORTA_D8 PORT_B4
#define PORTA_D9 PORT_B5
#define PORTA_D10 PORT_B6
#define PORTA_D11 PORT_B7
#define PORTA_D12 PORT_D6
#define PORTA_D13 PORT_C7
#define PORTA_D14 PORT_B3 //MISO
#define PORTA_D15 PORT_B1 //SCK
#define PORTA_D16 PORT_B2 //MOSI
#define PORTA_A0 PORT_F7
#define PORTA_A1 PORT_F6
#define PORTA_A2 PORT_F5
#define PORTA_A3 PORT_F4
#define PORTA_A4 PORT_F1
#define PORTA_A5 PORT_F0
#define TXLED PORT_D5
#define RXLED PORT_B0
#elif defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168P__) || defined(__AVR_ATmega168__)
#define PORTA_D0 PORT_D0
#define PORTA_D1 PORT_D1
#define PORTA_D2 PORT_D2
#define PORTA_D3 PORT_D3
#define PORTA_D4 PORT_D4
#define PORTA_D5 PORT_D5
#define PORTA_D6 PORT_D6
#define PORTA_D7 PORT_D7
#define PORTA_D8 PORT_B0
#define PORTA_D9 PORT_B1
#define PORTA_D10 PORT_B2
#define PORTA_D11 PORT_B3 //MOSI
#define PORTA_D12 PORT_B4 //MISO
#define PORTA_D13 PORT_B5 //SCK
#define PORTA_A0 PORT_C0
#define PORTA_A1 PORT_C1
#define PORTA_A2 PORT_C2
#define PORTA_A3 PORT_C3
#define PORTA_A4 PORT_C4 //SDA
#define PORTA_A5 PORT_C5 //SCL
#define TXLED PORT_B5
#define RXLED PORT_B5
#elif defined(__AVR_ATmega16__) || defined(__AVR_ATmega32__)
#define PORTA_D0 PORT_B0 //RXD
#define PORTA_D1 PORT_B1 //TXD
#define PORTA_D2 PORT_B2
#define PORTA_D3 PORT_B3
#define PORTA_D4 PORT_B4
#define PORTA_D5 PORT_B5 //MOSI
#define PORTA_D6 PORT_B6 //MISO
#define PORTA_D7 PORT_B7 //SCK
#define PORTA_D8 PORT_D0
#define PORTA_D9 PORT_D1
#define PORTA_D10 PORT_D2
#define PORTA_D11 PORT_D3
#define PORTA_D12 PORT_D4
#define PORTA_D13 PORT_D5
#define PORTA_D14 PORT_D6
#define PORTA_D15 PORT_D7
#define PORTA_D16 PORT_C0
#define PORTA_D17 PORT_C1
#define PORTA_D18 PORT_C2
#define PORTA_D19 PORT_C3
#define PORTA_D20 PORT_C4
#define PORTA_D21 PORT_C5
#define PORTA_D22 PORT_C6
#define PORTA_D23 PORT_C7
#define PORTA_A0 PORT_A0
#define PORTA_A1 PORT_A1
#define PORTA_A2 PORT_A2
#define PORTA_A3 PORT_A3
#define PORTA_A4 PORT_A4
#define PORTA_A5 PORT_A5
#define PORTA_A6 PORT_A6
#define PORTA_A7 PORT_A7
#define TXLED PORT_B7
#define RXLED PORT_B7
#endif

#define SETPM(port, ddr, pin, portnum) port = (port | (1<<portnum))
#define CLRPM(port, ddr, pin, portnum) port = (port & (~(1<<portnum)))
#define INVPM(port, ddr, pin, portnum) port = (port ^ (1<<portnum))
#define SETDM(port, ddr, pin, portnum) ddr = (ddr | (1<<portnum))
#define CLRDM(port, ddr, pin, portnum) ddr = (ddr & (~(1<<portnum)))
#define PINVM(port, ddr, pin, portnum) ((pin & (1<<portnum)) != 0)
#define PORTM(port, ddr, pin, portnum) port
#define DDRM(port, ddr, pin, portnum) ddr
#define PINM(port, ddr, pin, portnum) pin
#define PORTNM(port, ddr, pin, portnum) portnum

#define SETP(arg) SETPM(arg)
#define CLRP(arg) CLRPM(arg)
#define INVP(arg) INVPM(arg)
#define SETD(arg) SETDM(arg)
#define CLRD(arg) CLRDM(arg)
#define PINV(arg) PINVM(arg)
#define PORT(arg) PORTM(arg)
#define DDR(arg) DDRM(arg)
#define PIN(arg) PINM(arg)
#define PORTN(arg) PORTNM(arg)

#endif /* PORTS_H_ */
