/*
 * settings.h
 *
 * Created: 10.02.2016 13:34:53
 *  Author: Pavel
 */ 


#ifndef SETTINGS_H_
#define SETTINGS_H_

#define F_CPU 16000000UL

//=============================НАСТРОЙКИ===========================================

#define PPM 0
//#define SBUS 1

#define VOLTAGE_WARNING_TIME 5000 //Время предупреждения о низком заряде аккумулятора
#define VOLTAGE_PER_CELL_WARNING 3.5 //Предупреждать при разряде банки до 3.5 вольт
#define VOLTAGES_MEASURE_COUNT 10 //Количество замеров напряжения для усреднения
#define RESISTOR_0 56000.0 //Номинал резистора 1 для делителя напряжения
#define RESISTOR_1 22000.0 //Номинал резистора 2 для делителя напряжения
#define AVCC 4.57 //Опорное напряжение

#define MAVLINK_TIMEOUT 1500 //Тайм-аут для сохранения последней позиции
#define LOCK_SCREEN_TIMEOUT 3000 //Продолжительность нажатия удержания кнопки для активации модуля

#define IDLE_LED_PIN PINB5 //Пин индикатора ожидания
#define BUTTON_PIN PIND4 //Пин кнопки
#define MOSFET_PIN PIND2 //Пин MOSFET
#define BUZZER_PIN PIND3 //Пин бузера
#define INPUT_VOLTAGE_PIN 3//Аналоговый пин на котором мерим напряжение через делитель напряжения

#define EEPROM_LAT_ADDRESS 0 //Адрес хранения LAT
#define EEPROM_LON_ADDRESS 4 //Адрес хранения LON
#define EEPROM_HDG_ADDRESS 8 //Адрес хранения направления

//=============================НАСТРОЙКИ===========================================

#ifdef PPM 
	#define UART_BAUND_RATE 57600L //Скорость UART
#endif

#ifdef SBUS
	#define UART_BAUND_RATE 9600L //Скорость UART
#endif

#endif /* SETTINGS_H_ */