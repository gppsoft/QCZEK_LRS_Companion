/*
* QCZEK_LCD.cpp
*
* Created: 12.01.2020 18:16:58
* Author : Pavel
*/

#include "settings.h"

#include <avr/io.h>
#include <stdlib.h>
#include <stdio.h>
#include <util/delay.h>

#include "drivers/u8glib/u8g.h"
#include "common/mavlink.h"
#include "atmegaPins.h"
#include "drivers/uart.h"
#include "bitmaps.h"
#include "utils.h"
#include "drivers/eeprom.h"
#include "time.h"
#include "rus6x12.h"

#define DEBUG false

//=============DRIVERS==========================
u8g_t display;
UART serial;
EEPROM eeprom;
Time time;
//=============DRIVERS==========================

//=============MAVLINK==========================
mavlink_message_t msg;
mavlink_status_t status;

float latitude, longitude;
uint16_t hdg;
int32_t alt;
uint8_t speed;
uint8_t satellites_visible, fix_type;
uint8_t rssi;

float roll;
float pitch;

float current;
float voltage;

enum MAVLINK_STATUS{
	WAITING, IN_AIR
};

MAVLINK_STATUS mavlinkStatus = WAITING;

//=============MAVLINK==========================

char stringBuffer[32];
char floatBuffer[16];

bool unblock = false;

volatile unsigned long safety_button_pressed_time =0;
volatile unsigned long last_mavlink_time = 0;
volatile unsigned long last_voltage_warning_time = 0;

void mavlink_receive(uint8_t c){

	if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
		
		last_mavlink_time = time.millis();
		mavlinkStatus = IN_AIR;
		
		switch(msg.msgid) {
			case MAVLINK_MSG_ID_HEARTBEAT:
			break;
			case MAVLINK_MSG_ID_HOME_POSITION:
			break;
			case MAVLINK_MSG_ID_RADIO_STATUS:
			break;
			case MAVLINK_MSG_ID_COMMAND_INT:
			break;
			case MAVLINK_MSG_ID_VFR_HUD:
			break;
			case MAVLINK_MSG_ID_ATTITUDE:
			mavlink_attitude_t attitude;
			mavlink_msg_attitude_decode(&msg, &attitude);
			roll = attitude.roll;
			pitch = attitude.pitch;
			break;
			case MAVLINK_MSG_ID_SYS_STATUS: {
				mavlink_sys_status_t packet;
				mavlink_msg_sys_status_decode(&msg, &packet);
				
				voltage = packet.voltage_battery / 1000.0;
				
				if(packet.current_battery > 0){
					current = packet.current_battery / 100.0;
				}
				
				break;
			}
			case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
			mavlink_global_position_int_t packet;
			mavlink_msg_global_position_int_decode(&msg, &packet);
			
			if(packet.hdg == 65535) packet.hdg = 0;
			if(packet.alt == 65535) packet.alt = 0;
			
			hdg = packet.hdg/100;
			alt = packet.relative_alt/1000;
			
			break;
			case MAVLINK_MSG_ID_RC_CHANNELS_RAW: {
				mavlink_rc_channels_raw_t packet;
				mavlink_msg_rc_channels_raw_decode(&msg, &packet);
				rssi = packet.rssi;
				break;
			}
			case MAVLINK_MSG_ID_GPS_RAW_INT: {
				mavlink_gps_raw_int_t packet;
				mavlink_msg_gps_raw_int_decode(&msg, &packet);
				
				if(packet.vel == 65535) packet.vel = 0;
				
				latitude = (float)packet.lat / 10000000L;
				longitude = (float)packet.lon / 10000000L;
				
				fix_type = packet.fix_type;
				satellites_visible = packet.satellites_visible;
				speed = (uint8_t) (packet.vel / 100 * 3.6);
				
				break;
			}
			
		}
	}
}

void init_ADC()
{
	ADMUX = (1<<REFS0);
	ADCSRA = (1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);
}


void init_hardware(){

	init_ADC();
	
	setPinMode(DDRD,MOSFET_PIN,OUTPUT);
	setPinMode(DDRD,BUZZER_PIN,OUTPUT);
	
	setPinMode(DDRB,IDLE_LED_PIN,OUTPUT);
	digitalWrite(PORTB,IDLE_LED_PIN,HIGH);
	
	setPinMode(DDRD,BUTTON_PIN,INPUT);
	
	time.init();

	serial.init();
	serial.add(0,mavlink_receive);
	
	u8g_InitI2C(&display, &u8g_dev_ssd1306_128x64_i2c, U8G_I2C_OPT_FAST);
	
	sei();//Разрешаем прерывания
}

void delay(int ms){
	while (0 < ms){
		_delay_ms(1);
		--ms;
	}
}

void beep(uint16_t millis=1000){
	digitalWrite(PORTD,BUZZER_PIN,HIGH);
	delay(millis);
	digitalWrite(PORTD,BUZZER_PIN,LOW);
}

void startRadiation(){
	unblock=true;
	if(!DEBUG) beep();
	digitalWrite(PORTB,IDLE_LED_PIN,LOW);
	digitalWrite(PORTD,MOSFET_PIN,HIGH);
}

uint16_t analogRead(int pin){
	ADMUX = pin|(ADMUX & 0xF8); //Настраиваем пин
	ADCSRA |= 1<<ADSC;//Запуск преобразования
	while(ADCSRA & (1<<ADSC));//Ждем, пока АЦП закончит преобразование (ADSC = 0)
	return ADC;//Возвращаем результат
}

float getInputVoltage()
{
	if(DEBUG) return 16.8;
	
	float value=0.0;
	float result= 0.0;

	for(int i=0;i<VOLTAGES_MEASURE_COUNT;i++){
		value = (analogRead(INPUT_VOLTAGE_PIN) * AVCC) / 1024.0;
		result+= value / (RESISTOR_1/(RESISTOR_0+RESISTOR_1));
	}

	return result / VOLTAGES_MEASURE_COUNT;
}

uint8_t getCellCount(){
	return (getInputVoltage()+0.5) / VOLTAGE_PER_CELL_WARNING;
}

void saveLastPosition(){
	if(latitude>0 && longitude>0){
		eeprom.save((uint8_t*)&latitude,sizeof(latitude),EEPROM_LAT_ADDRESS);
		eeprom.save((uint8_t*)&longitude,sizeof(longitude),EEPROM_LON_ADDRESS);
		eeprom.save((uint8_t*)&hdg,sizeof(hdg),EEPROM_HDG_ADDRESS);
	}
}

void loadLastPosition(){
	latitude = eeprom.readFloat(EEPROM_LAT_ADDRESS);
	longitude = eeprom.readFloat(EEPROM_LON_ADDRESS);
	hdg = eeprom.readInt16(EEPROM_HDG_ADDRESS);
}

void showLockScreen(){
	
	
	u8g_FirstPage(&display);

	do
	{
		u8g_SetFont(&display, rus6x12);
		u8g_DrawStrP(&display,0,12,U8G_PSTR("!!!ПРОВЕРЬ АНТЕННУ!!!"));
		u8g_DrawStrP(&display,0,28,U8G_PSTR("Нажмите и удерживайте"));
		u8g_DrawStrP(&display,0,36,U8G_PSTR("кнопку 3 секунды для"));
		u8g_DrawStrP(&display,0,44,U8G_PSTR("запуска передатчика."));
		
		dtostrf(getInputVoltage(),3,1,floatBuffer);
		sprintf_P(stringBuffer, PSTR("Питание:%sV"), floatBuffer);
		u8g_DrawStr(&display,0,60,stringBuffer);
	
	} while (u8g_NextPage(&display));
	
	if(digitalRead(PIND,BUTTON_PIN))
	{
		safety_button_pressed_time = time.millis();
		
		while(time.millis()-safety_button_pressed_time < LOCK_SCREEN_TIMEOUT && digitalRead(PIND,BUTTON_PIN));
		
		if(digitalRead(PIND,BUTTON_PIN)){
			startRadiation();
		}
	}
	else
	{
		safety_button_pressed_time = 0;
	}
}


void showSplashScreen(){
	
	#ifdef SBUS
		serial.write("QCZEK Companion 1.0(SBUS)\n");
	#endif
	
	#ifdef PPM
		serial.write("QCZEK Companion 1.0(PPM)\n");
	#endif

	u8g_FirstPage(&display);

	do
	{
		u8g_SetFont(&display, u8g_font_6x12);
		
		#ifdef PPM
			u8g_DrawStrP(&display,34,8,U8G_PSTR("433 MHz (PPM)"));
		#endif
		
		#ifdef SBUS
			u8g_DrawStrP(&display,34,8,U8G_PSTR("433 MHz (SBUS)"));
		#endif
		
		u8g_SetFont(&display, u8g_font_10x20);
		u8g_DrawStrP(&display,72,32,U8G_PSTR("QCZEK"));
		u8g_SetFont(&display, u8g_font_6x12);
		u8g_DrawStrP(&display,72,44,U8G_PSTR("COMPANION"));
		u8g_SetFont(&display, u8g_font_04b_24);
		u8g_DrawStrP(&display,70,64,U8G_PSTR("by sakhpilots.ru"));
		u8g_DrawXBMP(&display,16,16,48,48,with_plane);
	} while ( u8g_NextPage(&display) );
	
	delay(2000);
}

void showLastSavedPosScreen(){

	loadLastPosition();
	
	u8g_SetFont(&display, rus6x12);
	
	if(mavlinkStatus==WAITING){
		u8g_DrawStrP(&display,16,10,U8G_PSTR("ОЖИДАНИЕ MAVLINK"));
	}
	
	u8g_DrawStrP(&display,4,24,U8G_PSTR("Последние координаты"));

	dtostrf(latitude,10,7,floatBuffer);
	sprintf_P(stringBuffer, PSTR("LAT:%s"), floatBuffer);
	u8g_DrawStr(&display,4,40,stringBuffer);

	
	dtostrf(longitude,10,7,floatBuffer);
	sprintf_P(stringBuffer, PSTR("LON:%s"), floatBuffer);
	u8g_DrawStr(&display,4,50,stringBuffer);
	
	sprintf_P(stringBuffer, PSTR("HDG:%u"), hdg);
	u8g_DrawStr(&display,4,60,stringBuffer);
	u8g_DrawFrame(&display,0,28,127,36);

}

void showMavlinkDataScreen(){
	
	u8g_SetFont(&display, u8g_font_6x12);
	
	dtostrf(latitude,10,7,floatBuffer);
	sprintf_P(stringBuffer, PSTR("LAT:%s"), floatBuffer);
	u8g_DrawStr(&display,0,8,stringBuffer);
	
	dtostrf(longitude,10,7,floatBuffer);
	sprintf_P(stringBuffer, PSTR("LON:%s"), floatBuffer);
	u8g_DrawStr(&display,0,16,stringBuffer);
	
	if(fix_type==2){
		sprintf_P(stringBuffer, PSTR("SAT:%u(2D),HDG:%u"), satellites_visible,hdg);
		}else if(fix_type>=3){
		sprintf_P(stringBuffer, PSTR("SAT:%u(3D),HDG:%u"), satellites_visible,hdg);
		}else{
		sprintf_P(stringBuffer, PSTR("SAT:%u(No),HDG:%u"), satellites_visible,hdg);
	}
	
	u8g_DrawStr(&display,0,24,stringBuffer);
	
	sprintf_P(stringBuffer, PSTR("ALT:%um"),alt);
	u8g_DrawStr(&display,0,32,stringBuffer);

	sprintf_P(stringBuffer, PSTR("SPEED:%ukm/h"),speed);
	u8g_DrawStr(&display,0,40,stringBuffer);
	
	sprintf_P(stringBuffer, PSTR("RSSI:%u"), map(rssi,0,255,0,100));
	u8g_DrawStr(&display,0,48,stringBuffer);
	
	dtostrf(voltage,4,2,floatBuffer);
	sprintf_P(stringBuffer, PSTR("VOLTAGE:%sV"), floatBuffer);
	u8g_DrawStr(&display,0,56,stringBuffer);
	
	dtostrf(current,current>=10?5:4,2,floatBuffer);
	sprintf_P(stringBuffer, PSTR("CURRENT:%sA"), floatBuffer);
	u8g_DrawStr(&display,0,64,stringBuffer);
}

void showInputVoltageScreen(){
	
	
	float inputVoltage = getInputVoltage();
	
	u8g_FirstPage(&display);
	
	do
	{
		u8g_SetFont(&display, rus6x12);
		u8g_DrawStrP(&display,34,14,U8G_PSTR("НАПРЯЖЕНИЕ"));
	
		dtostrf(inputVoltage,3,1,floatBuffer);
		sprintf_P(stringBuffer, PSTR("%sV"), floatBuffer);
		u8g_SetFont(&display, u8g_font_10x20);
		u8g_DrawStr(&display,38,34,stringBuffer);
		
		sprintf_P(stringBuffer, PSTR("(%uS)"), getCellCount());
	    u8g_DrawStr(&display,42,56,stringBuffer);
		
	} while (u8g_NextPage(&display));
	
	_delay_ms(1500);
}

void checkVoltagePerCell(){
	if((getInputVoltage()/getCellCount())<VOLTAGE_PER_CELL_WARNING)
	{	
		if(time.millis()-last_voltage_warning_time>VOLTAGE_WARNING_TIME)
		{
			last_voltage_warning_time = time.millis();
			if(!DEBUG) beep(150);
			showInputVoltageScreen();
		}
	}
}

void checkMavlink(){
	if(time.millis() - last_mavlink_time > MAVLINK_TIMEOUT && mavlinkStatus==IN_AIR)
	{
		mavlinkStatus = WAITING;
		saveLastPosition();
	}
}

void checkButtonPressed(){
	if(digitalRead(PIND,BUTTON_PIN))
	{
		safety_button_pressed_time = time.millis();
		while(time.millis()-safety_button_pressed_time < 200 && digitalRead(PIND,BUTTON_PIN));
		if(!DEBUG) beep(150);
		showInputVoltageScreen();
	}
	else
	{
		safety_button_pressed_time = 0;
	}
}


int main(void)
{
	init_hardware();
	
	if(digitalRead(PIND,BUTTON_PIN)){
		startRadiation();
	}else{
		showSplashScreen();
	}
	

	while (true)
	{
		checkVoltagePerCell();
		
		if(unblock)
		{
		
			checkMavlink();
			checkButtonPressed();
			
			u8g_FirstPage(&display);
			
			do
			{
				if(mavlinkStatus == WAITING)
				{
					showLastSavedPosScreen();
				}
				else
				{
					showMavlinkDataScreen();
				}
				
				
			} while (u8g_NextPage(&display));
			
		}
		else
		{
			showLockScreen();
		}
		
	}
}












