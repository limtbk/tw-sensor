/*
 * macro.h
 *
 *  Created on: May 30, 2014
 *      Author: lim
 */

#ifndef MACRO_H_
#define MACRO_H_

#define HI(x) ((x)>>8)
#define LO(x) ((x) & 0xFF)
#define bv(bit) (1<<(bit))
#define cbi(reg,bit) reg &= ~(bv(bit))
#define sbi(reg,bit) reg |= (bv(bit))
#define ibi(reg,bit) reg ^= (bv(bit))
#define outb(addr, data) addr = (data)
#define inb(addr) (addr)
#define bit(var,bit) ((((var)&(1<<(bit)))!=0)?1:0)

#define MIN(a,b) (((a)<(b))?(a):(b))
#define MAX(a,b) (((a)>(b))?(a):(b))

#endif /* MACRO_H_ */
