#include <avr/io.h>
#include <avr/interrupt.h>
#include "ports.h"
#include "usart.h"
#include "macro.h"

#define SSR0 PORTA_D2
#define SSR1 PORTA_D3
#define SSR2 PORTA_D4
#define SSR3 PORTA_D5
#define SSR4 PORTA_D6
#define SSR5 PORTA_D7
#define SSR6 PORTA_D8
#define SSR7 PORTA_D9
#define SSR8 PORTA_A0
#define SSR9 PORTA_D12
#define SSR10 PORTA_D11
#define SSR11 PORTA_D10
#define SSRT PORTA_D13 //PORT_B5

#define MEASURES 32

#define measure_pinm(__port, __ddr, __pin, __nn, __portt, __ddrt, __pint, __nnt) \
		uint8_t tmp = 0;\
		asm volatile (\
				"	cli"							"\n\t"\
				"	cbi %[portt],%[nnt]"			"\n\t" /*PORTT=0*/\
				"	cbi %[port],%[nn]"				"\n\t" /*PORT=0*/\
				"	sbi %[ddr],%[nn]"				"\n\t" /*DDR=1*/\
				"	cbi %[ddr],%[nn]"				"\n\t" /*DDR=0*/\
				"	cbi %[port],%[nn]"				"\n\t" /*PORT=0*/\
				"	sbi %[portt],%[nnt]"			"\n\t" /*PORTT=1*/\
				"	rjmp mloop11%="					"\n\t"\
				"mloop10%=:"						"\n\t"\
				"	subi %[t],0xFF"					"\n\t" /*t+=1*/\
				"mloop11%=:"						"\n\t"\
				"	sbis %[pin],%[nn]"				"\n\t" /*PIN*/\
				"	rjmp mloop10%="					"\n\t"\
				"	sbi %[portt],%[nnt]"			"\n\t" /*PORTT=1*/\
				"	sbi %[port],%[nn]"				"\n\t" /*PORT=1*/\
				"	sbi %[ddr],%[nn]"				"\n\t" /*DDR=1*/\
				"	cbi %[ddr],%[nn]"				"\n\t" /*DDR=0*/\
				"	cbi %[port],%[nn]"				"\n\t" /*PORT=0*/\
				"	cbi %[portt],%[nnt]"			"\n\t" /*PORTT=0*/\
				"	rjmp mloop21%="					"\n\t"\
				"mloop20%=:"						"\n\t"\
				"	subi %[t],0xFF"					"\n\t" /*t+=1*/\
				"mloop21%=:"						"\n\t"\
				"	sbic %[pin],%[nn]"				"\n\t" /*PIN*/\
				"	rjmp mloop20%="					"\n\t"\
				"	sei"							"\n\t"\
				:\
				[t] "+d" (tmp)\
				:\
				[port] "I" (_SFR_IO_ADDR(__port)),\
				[ddr] "I" (_SFR_IO_ADDR(__ddr)),\
				[pin] "I" (_SFR_IO_ADDR(__pin)),\
				[nn] "I" (__nn),\
				[portt] "I" (_SFR_IO_ADDR(__portt)),\
				[nnt] "I" (__nnt)\
		);

#define measure_pin(port, portt) measure_pinm(port, portt)

uint16_t timer1_lastTime;

/*
void timer1_init(void) {
#if defined (__AVR_ATmega32U4__)
	TCCR1A = (0<<COM1A1)|(0<<COM1A0)|(0<<COM1B1)|(0<<COM1B0)|(0<<COM1C1)|(0<<COM1C0)|(0<<WGM11)|(0<<WGM10);
	TCCR1B = (0<<ICNC1)|(0<<ICES1)|(0<<WGM13)|(0<<WGM12)|(0<<CS12)|(0<<CS11)|(1<<CS10); //Normal, clk/1
	TCCR1C = (0<<FOC1A)|(0<<FOC1B)|(0<<FOC1C);
	TIMSK1 = (0<<ICIE1)|(0<<OCIE1C)|(0<<OCIE1B)|(0<<OCIE1A)|(0<<TOIE1);
#elif defined (__AVR_ATmega328P__)
	TCCR1A = (0<<COM1A1)|(0<<COM1A0)|(0<<COM1B1)|(0<<COM1B0)|(0<<WGM11)|(0<<WGM10);
	TCCR1B = (0<<ICNC1)|(0<<ICES1)|(0<<WGM13)|(0<<WGM12)|(0<<CS12)|(0<<CS11)|(1<<CS10); //Normal, clk/1
	TIMSK0 = (0<<ICIE1)|(0<<OCIE1B)|(0<<OCIE1A)|(0<<TOIE1);
#else
	TCCR1A = (0<<COM1A1)|(0<<COM1A0)|(0<<COM1B1)|(0<<COM1B0)|(0<<WGM11)|(0<<WGM10);
	TCCR1B = (0<<ICNC1)|(0<<ICES1)|(0<<WGM13)|(0<<WGM12)|(0<<CS12)|(0<<CS11)|(1<<CS10); //Normal, clk/1
	TIMSK = (0<<TICIE1)|(0<<OCIE1B)|(0<<OCIE1A)|(0<<TOIE1);
#endif
}

void resetTimer1(void) {
	TCNT1 = 0;
}

uint16_t timer1_measureTime(void) {
	uint16_t time = TCNT1;
	uint16_t result = time - timer1_lastTime;
	timer1_lastTime = time;
	return result;
}
*/

void init(void) {

	PORTD = 0;
	DDRD  = 0;
	PORTB = 0;
	DDRB =  0;
	PORTC = 0;
	DDRC =  0;

	SETD(SSRT);
	CLRP(SSRT);

#ifdef __AVR_ATmega32U4__
	USBCON = (0<<USBE)|(1<<FRZCLK)|(0<<OTGPADE)|(0<<VBUSTE);
#endif

	usart_init();
//	timer1_init();
	sei();
}

//uint16_t measure_for_pin(volatile uint8_t *pin, uint8_t pinn)
//{
//	uint16_t result = 0;
//	uint8_t pinmask = 1<<pinn;
//	cli();
//	timer1_measureTime();
//	for (uint8_t i = 0; i < MEASURES; i++) {
//		CLRP(SSRT);
//		while (((*pin) & pinmask) != 0) {}
//		*(pin+1) |= pinmask; //DDR = 1
//		*(pin+2) &= ~pinmask; //PORT = 0
//		*(pin+1) &= ~pinmask; //DDR = 0
//		SETP(SSRT);
//		while (!(((*pin) & pinmask) != 0)) {}
//		*(pin+1) |= pinmask; //DDR = 1
//		*(pin+2) |= pinmask; //PORT = 1
//		*(pin+1) &= ~pinmask; //DDR = 0
//		*(pin+2) &= ~pinmask; //PORT = 0
//	}
//	result = timer1_measureTime();
//	sei();
//	return result;
//}

uint16_t measure_for_ssr0()
{
	uint16_t result = 0;
	for (uint8_t i = 0; i < MEASURES; i++) {
		measure_pin(SSR0, SSRT);
		result += tmp;
	}
	return result;
}

uint16_t measure_for_ssr1()
{
	uint16_t result = 0;
	for (uint8_t i = 0; i < MEASURES; i++) {
		measure_pin(SSR1, SSRT);
		result += tmp;
	}
	return result;
}

uint16_t measure_for_ssr2()
{
	uint16_t result = 0;
	for (uint8_t i = 0; i < MEASURES; i++) {
		measure_pin(SSR2, SSRT);
		result += tmp;
	}
	return result;
}

uint16_t measure_for_ssr3()
{
	uint16_t result = 0;
	for (uint8_t i = 0; i < MEASURES; i++) {
		measure_pin(SSR3, SSRT);
		result += tmp;
	}
	return result;
}


uint16_t measure_for_ssr4()
{
	uint16_t result = 0;
	for (uint8_t i = 0; i < MEASURES; i++) {
		measure_pin(SSR4, SSRT);
		result += tmp;
	}
	return result;
}

uint16_t measure_for_ssr5()
{
	uint16_t result = 0;
	for (uint8_t i = 0; i < MEASURES; i++) {
		measure_pin(SSR5, SSRT);
		result += tmp;
	}
	return result;
}

uint16_t measure_for_ssr6()
{
	uint16_t result = 0;
	for (uint8_t i = 0; i < MEASURES; i++) {
		measure_pin(SSR6, SSRT);
		result += tmp;
	}
	return result;
}

uint16_t measure_for_ssr7()
{
	uint16_t result = 0;
	for (uint8_t i = 0; i < MEASURES; i++) {
		measure_pin(SSR7, SSRT);
		result += tmp;
	}
	return result;
}

uint16_t measure_for_ssr8()
{
	uint16_t result = 0;
	for (uint8_t i = 0; i < MEASURES; i++) {
		measure_pin(SSR8, SSRT);
		result += tmp;
	}
	return result;
}

uint16_t measure_for_ssr9()
{
	uint16_t result = 0;
	for (uint8_t i = 0; i < MEASURES; i++) {
		measure_pin(SSR9, SSRT);
		result += tmp;
	}
	return result;
}

uint16_t measure_for_ssr10()
{
	uint16_t result = 0;
	for (uint8_t i = 0; i < MEASURES; i++) {
		measure_pin(SSR10, SSRT);
		result += tmp;
	}
	return result;
}

uint16_t measure_for_ssr11()
{
	uint16_t result = 0;
	for (uint8_t i = 0; i < MEASURES; i++) {
		measure_pin(SSR11, SSRT);
		result += tmp;
	}
	return result;
}

void print_version()
{
	usart_printstr("V1.2\n\r");
}

int main(void)
{
	init();

	usart_printstr("\n\rCapacity sensor\n\r");
	print_version();

	uint16_t tarr[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
	uint16_t minarr[12] = {UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX, UINT16_MAX};
	uint16_t maxarr[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
	uint8_t cm = 0;
	uint8_t m = 0;
	uint8_t cal_min_cycles = 255;
	uint8_t cal_max_cycles = 255;

	while (1) {
		if (usart_chrready()) {
			char ch = usart_getchr();
			switch (ch) {
				case 'h':
					{
						usart_printstr(	"\n\r"
										"h - help\n\r"
										"m - measure once\n\r"
										"c - start continuous measure\n\r"
										"s - stop continuous measure\n\r"
										"r - reset calibration\n\r"
										"n - calibrate min\n\r"
										"x - calibrate max\n\r"
										"R - reset\n\r"
										"b - binary num mode\n\r"
										"t - hex text num mode (default)\n\r"
										"w - raw data mode\n\r"
										"l - normalized data mode (default)\n\r"
										"v - version\n\r");
						break;
					}
				case 'v':
					{
						print_version();
						break;
					}
				case 'm':
					{
						m = 1;
						break;
					}
				case 'c':
					{
						cm = 1;
						break;
					}
				case 's':
					{
						cm = 0;
						break;
					}
				case 'r':
					{
						for (uint8_t i=0; i<12; i++) {
							minarr[i] = UINT16_MAX;
							maxarr[i] = 0;
						}
						break;
					}
				case 'n':
					{
						cal_min_cycles = 255;
						break;
					}
				case 'R':
					{
						void *bl = (void *)0x3c00; //for ATMega328 with bootloader
						goto *bl;
						break;
					}
				case 'x':
					{
						cal_max_cycles = 255;
						break;
					}
				default:
					break;
			}
		}

		tarr[0] = measure_for_ssr0();
		tarr[1] = measure_for_ssr1();
		tarr[2] = measure_for_ssr2();
		tarr[3] = measure_for_ssr3();
		tarr[4] = measure_for_ssr4();
		tarr[5] = measure_for_ssr5();
		tarr[6] = measure_for_ssr6();
		tarr[7] = measure_for_ssr7();
		tarr[8] = measure_for_ssr8();
		tarr[9] = measure_for_ssr9();
		tarr[10] = measure_for_ssr10();
		tarr[11] = measure_for_ssr11();

		if (cal_min_cycles) {
			for (uint8_t ti = 0; ti < 12; ti++) {
				minarr[ti] = MIN(minarr[ti], tarr[ti]);
			}
			cal_min_cycles--;
		}

		if (cal_max_cycles) {
			for (uint8_t ti = 0; ti < 12; ti++) {
				maxarr[ti] = MAX(maxarr[ti], tarr[ti]);
			}
			cal_max_cycles--;
		}

		if (m || cm) {
			for (uint8_t ti = 0; ti < 12; ti++) {
				uint16_t tt = tarr[ti];

				if (minarr[ti]<UINT16_MAX) {
					if ((maxarr[ti]>0) && (minarr[ti]<maxarr[ti])) {
						uint16_t delta = maxarr[ti] - minarr[ti];
						uint16_t td = (tarr[ti]>minarr[ti])?(tarr[ti]-minarr[ti]):0;
						uint32_t tmp = ((int32_t)td*255)/delta;
						tt = (tmp<255)?(uint16_t)tmp:255;
					} else {
						tt = (tarr[ti]>minarr[ti])?(tarr[ti]-minarr[ti]):0;
					}
				}

				usart_printhex(HI(tt));
				usart_printhex(LO(tt));
				usart_putchr(' ');
			}
			usart_printstr("\n\r");
			m = 0;
		}
	}

	return 0;
}

#ifdef __AVR_ATmega32U4__
ISR(USB_COM_vect) {
}

ISR(USB_GEN_vect) {
}
#endif
