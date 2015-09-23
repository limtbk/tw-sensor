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
#define SSR8 PORTA_D10
#define SSR9 PORTA_D11
#define SSR10 PORTA_D12
#define SSR11 PORTA_A0
#define SSRT PORTA_D13 //PORT_B5

uint16_t timer1_lastTime;

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
	timer1_init();
	sei();
}

uint16_t measure_for_pin(volatile uint8_t *pin, uint8_t pinn)
{
	uint16_t result = 0;
//	uint8_t tmp = 0;
	uint8_t pinmask = 1<<pinn;
	cli();
	timer1_measureTime();
	for (uint8_t i = 0; i < 32; i++) {
//		tmp = 0;
		CLRP(SSRT);
		while (((*pin) & pinmask) != 0) {
//			tmp++;
		}
		*(pin+1) |= pinmask; //DDR = 1
		*(pin+2) &= ~pinmask; //PORT = 0
		*(pin+1) &= ~pinmask; //DDR = 0
		SETP(SSRT);
		while (!(((*pin) & pinmask) != 0)) {
//			tmp++;
		}
		*(pin+1) |= pinmask; //DDR = 1
		*(pin+2) |= pinmask; //PORT = 1
		*(pin+1) &= ~pinmask; //DDR = 0
		*(pin+2) &= ~pinmask; //PORT = 0
//		result += tmp;
	}
	result = timer1_measureTime();
	sei();
	return result;
}

uint16_t measure_for_pin2()
{
	uint16_t result = 0;
	for (uint8_t i = 0; i < 1; i++) {
		uint8_t tmp = 0;
		cli();
		asm volatile (
//				"	cli"							"\n\t"
				"	subi %[t],0xFF"					"\n\t"
				"	cbi 0x05,5"						"\n\t" //PORTB5=0
				"	rjmp mloop11"					"\n\t"
				"mloop10:"							"\n\t"
				"	subi %[t],0xFF"					"\n\t"
				"mloop11:"							"\n\t"
				"	sbic 0x09,2"					"\n\t" //PIND2
				"	rjmp mloop10"					"\n\t"
//				"	sei"							"\n\t"
				:
				[t] "+d" (tmp)
				:
		);

		asm volatile (
//				"	cli"							"\n\t"
				"	sbi 0x05,5"						"\n\t" //PORTB5=1
				"	rjmp mloop21"					"\n\t"
				"mloop20:"							"\n\t"
				"	subi %[t],0xFF"					"\n\t"
				"mloop21:"							"\n\t"
				"	sbis 0x09,2"					"\n\t" //PIND2
				"	rjmp mloop20"					"\n\t"
//				"	sei"							"\n\t"
				:
				[t] "+d" (tmp)
				:
		);
		sei();
		result += tmp;
	}

	return result;
}

int main(void)
{
	init();

	usart_printstr("\n\rCapacity sensor\n\r");

	uint16_t tarr[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

	while (1) {

		tarr[0] = measure_for_pin2();
//		tarr[0] = measure_for_pin(&PIN(SSR0), PORTN(SSR0));
		tarr[1] = measure_for_pin(&PIN(SSR1), PORTN(SSR1));
		tarr[2] = measure_for_pin(&PIN(SSR2), PORTN(SSR2));
		tarr[3] = measure_for_pin(&PIN(SSR3), PORTN(SSR3));
		tarr[4] = measure_for_pin(&PIN(SSR4), PORTN(SSR4));
		tarr[5] = measure_for_pin(&PIN(SSR5), PORTN(SSR5));
		tarr[6] = measure_for_pin(&PIN(SSR6), PORTN(SSR6));
		tarr[7] = measure_for_pin(&PIN(SSR7), PORTN(SSR7));
		tarr[8] = measure_for_pin(&PIN(SSR8), PORTN(SSR8));
		tarr[9] = measure_for_pin(&PIN(SSR9), PORTN(SSR9));
		tarr[10] = measure_for_pin(&PIN(SSR10), PORTN(SSR10));
		tarr[11] = measure_for_pin(&PIN(SSR11), PORTN(SSR11));

		for (uint8_t ti = 0; ti < 12; ti++) {
			usart_printhex(HI(tarr[ti]));
			usart_printhex(LO(tarr[ti]));
			usart_putchr(' ');
		}
		usart_printstr("\n\r");
	}

	return 0;
}

#ifdef __AVR_ATmega32U4__
ISR(USB_COM_vect) {
}

ISR(USB_GEN_vect) {
}
#endif
