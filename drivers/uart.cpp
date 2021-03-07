/*
 * UART.cpp
 *
 * Created: 23.11.2015 21:54:18
 *  Author: Pavel
 */ 
#include "uart.h"


void UART::init(){
	UCSR0A = 0;//Регист UCSRA информирует о состоянии UART. Обнуляем его.

	//В данные регистры записывается скорость, т.к значение скорости занимает 2 байта,
	//то значение записывается в виде старшего и младшего байта по отдельности
	UBRR0L = LO(BAUND_DIVIDER);
	UBRR0H = HI(BAUND_DIVIDER);

	UCSR0B |= 1<<RXEN0|1<<TXEN0|1 << RXCIE0;
	
    //Set frame format = 8-N-1
    UCSR0C|=1<<USBS0|1<<UCSZ01|1<<UCSZ00;
}

void UART::clear(void){//clear slots
	uint8_t slots = UART_SLOTS;
	for (uint8_t _i = 0; _i < slots; _i++){
		uartslots[_i] = 0;
	}
}

void UART::add(void (*_f)(uint8_t)){
	uint8_t slots = UART_SLOTS;
	for (uint8_t _i = 0; _i < slots; _i++){
		uartslots[_i] = _f;
	}
}

void UART::add(uint8_t _slot, void (*_f)(uint8_t)){
	if (_slot < UART_SLOTS){
		uartslots[_slot]=_f;//put function into slot
	}
}

void UART::receive(uint8_t data){
	for (uint8_t buttonslot = 0; buttonslot < UART_SLOTS; buttonslot++)
	{
		if (uartslots[buttonslot])
		{
			(*uartslots[buttonslot])(data);
		}
	}
}

void UART::write(uint8_t data){
	while(!(UCSR0A & (1<<UDRE0)));
	UDR0=data;
}

void UART::write(int data){
	char buff[8];
	itoa(data,buff,10);
	write(buff);
}

void UART::write(char* data, unsigned int size){
	for(unsigned int i=0;i<size; i++)
	{
		while(!(UCSR0A&(1<<UDRE0))){};
		UDR0 = (*data);
		data++;
	}
}

void UART::write(char* data){
	while ((*data)!='\0'){
		while(!(UCSR0A&(1<<UDRE0))){};
		UDR0 = (*data);
		data++;
	}
}


//Прерывание для входящих данных по uart
ISR(USART_RX_vect){
	//!!!Не реализованно!!!
	serial.receive(UDR0);
}

