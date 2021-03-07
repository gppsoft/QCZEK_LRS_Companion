/*
 * settings.h
 *
 * Created: 10.02.2016 13:34:53
 *  Author: Pavel
 */ 


#ifndef SETTINGS_H_
#define SETTINGS_H_

#define F_CPU 16000000UL

//=============================���������===========================================

#define PPM 0
//#define SBUS 1

#define VOLTAGE_WARNING_TIME 5000 //����� �������������� � ������ ������ ������������
#define VOLTAGE_PER_CELL_WARNING 3.5 //������������� ��� ������� ����� �� 3.5 �����
#define VOLTAGES_MEASURE_COUNT 10 //���������� ������� ���������� ��� ����������
#define RESISTOR_0 56000.0 //������� ��������� 1 ��� �������� ����������
#define RESISTOR_1 22000.0 //������� ��������� 2 ��� �������� ����������
#define AVCC 4.57 //������� ����������

#define MAVLINK_TIMEOUT 1500 //����-��� ��� ���������� ��������� �������
#define LOCK_SCREEN_TIMEOUT 3000 //����������������� ������� ��������� ������ ��� ��������� ������

#define IDLE_LED_PIN PINB5 //��� ���������� ��������
#define BUTTON_PIN PIND4 //��� ������
#define MOSFET_PIN PIND2 //��� MOSFET
#define BUZZER_PIN PIND3 //��� ������
#define INPUT_VOLTAGE_PIN 3//���������� ��� �� ������� ����� ���������� ����� �������� ����������

#define EEPROM_LAT_ADDRESS 0 //����� �������� LAT
#define EEPROM_LON_ADDRESS 4 //����� �������� LON
#define EEPROM_HDG_ADDRESS 8 //����� �������� �����������

//=============================���������===========================================

#ifdef PPM 
	#define UART_BAUND_RATE 57600L //�������� UART
#endif

#ifdef SBUS
	#define UART_BAUND_RATE 9600L //�������� UART
#endif

#endif /* SETTINGS_H_ */