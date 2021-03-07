/*
 * UART.h
 *
 * Created: 23.11.2015 21:52:16
 *  Author: Pavel
 */ 


#ifndef UART_H_
#define UART_H_

#include "../settings.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdlib.h>

#define UART_SLOTS 3
#define BAUND_DIVIDER (F_CPU/(16*UART_BAUND_RATE)-1)//Вычисляется значение для регистров UBRRL и UBRRH

#define HI(x) ((x)>>8)//Макрос получает старший байт значения
#define LO(x) ((x)& 0xFF)//Макрос получает младший байт значения

class UART{
	private:
		void (*uartslots[UART_SLOTS])(uint8_t);//uart slot
	public:
		void init();
		void clear(void);
		void add(void (*_f)(uint8_t));
		void add(uint8_t _slot, void (*_f)(uint8_t));
		void write(uint8_t data);
		void write(int data);
		void write(char* data, unsigned int size);	
		void write(char* data);
		void receive(uint8_t data);
};

#endif /* UART_H_ */

extern UART serial;