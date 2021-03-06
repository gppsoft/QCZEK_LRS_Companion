/*
 * bitmaps.h
 *
 * Created: 26.10.2019 21:05:32
 *  Author: Pavel
 */ 

#include <avr/pgmspace.h>

#ifndef BITMAPS_H_
#define BITMAPS_H_


const unsigned char with_plane[] PROGMEM = {
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0E, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x1F, 0x00, 0x00, 0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x00, 0x00,
	0x80, 0x7F, 0x00, 0x00, 0x00, 0x00, 0x80, 0xFF, 0x00, 0x00, 0x00, 0x00,
	0x00, 0xFF, 0x01, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x03, 0x00, 0x00, 0x00,
	0x00, 0xFE, 0x0F, 0x00, 0x00, 0x00, 0x00, 0xFC, 0x1F, 0x00, 0x00, 0x00,
	0x00, 0xF8, 0x3F, 0x00, 0x0F, 0x00, 0x00, 0xF8, 0x7F, 0x80, 0x0F, 0x00,
	0x00, 0xF0, 0xFF, 0xE0, 0x0F, 0x00, 0x00, 0xE0, 0xFF, 0xF3, 0x0F, 0x00,
	0x00, 0xE0, 0xFF, 0xFF, 0x07, 0x00, 0x00, 0xC0, 0xFF, 0xFF, 0x03, 0x00,
	0x00, 0x80, 0xFF, 0xFF, 0x03, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0x01, 0x00,
	0x00, 0x00, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0xFE, 0xFF, 0x00, 0x00,
	0x00, 0x00, 0xFC, 0xFF, 0x01, 0x00, 0x00, 0x00, 0xFC, 0xFF, 0x01, 0x00,
	0x00, 0x00, 0xF8, 0xFF, 0x03, 0x00, 0x00, 0x00, 0xF8, 0xFF, 0x07, 0x00,
	0x00, 0x00, 0xFC, 0xFF, 0x0F, 0x00, 0x00, 0x00, 0xFE, 0xFF, 0x1F, 0x00,
	0x00, 0x00, 0xFF, 0xFF, 0x3F, 0x00, 0x00, 0x00, 0x3F, 0xFF, 0x3F, 0x00,
	0x00, 0x80, 0x1F, 0xFC, 0x7F, 0x00, 0x38, 0xC0, 0x0F, 0xF8, 0xFF, 0x00,
	0xF8, 0xE0, 0x03, 0xE0, 0xFF, 0x01, 0xFC, 0xE1, 0x01, 0xC0, 0xFF, 0x03,
	0xF8, 0xF3, 0x00, 0x80, 0xFF, 0x07, 0xF0, 0x3F, 0x00, 0x00, 0xFE, 0x0F,
	0xE0, 0x1F, 0x00, 0x00, 0xFC, 0x1F, 0xE0, 0x1F, 0x00, 0x00, 0xF0, 0x1F,
	0xC0, 0x3F, 0x00, 0x00, 0xE0, 0x1F, 0x80, 0x7F, 0x00, 0x00, 0xC0, 0x0F,
	0x00, 0xFF, 0x00, 0x00, 0x00, 0x07, 0x00, 0xFE, 0x00, 0x00, 0x00, 0x00,
	0x00, 0xFC, 0x01, 0x00, 0x00, 0x00, 0x00, 0xF0, 0x01, 0x00, 0x00, 0x00,
	0x00, 0xE0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	
};

const unsigned char remote_control [] PROGMEM = {
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF0, 0xFC, 0xFC, 0xF4, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04,
	0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, 0xF0, 0xF0, 0x30,
	0x30, 0xF0, 0xF0, 0xF0, 0xF0, 0xE0, 0xFF, 0xFF, 0xFF, 0xFF, 0xE0, 0xF0, 0xF0, 0xF0, 0xF0, 0x30,
	0x30, 0xF0, 0xF0, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00,
	0x00, 0xC0, 0xF0, 0xF9, 0xFE, 0xF8, 0xFC, 0xEC, 0xCC, 0xEC, 0xFC, 0xFC, 0xEF, 0xCF, 0xEF, 0xFC,
	0xFC, 0xFB, 0xF7, 0xEF, 0xDF, 0xBF, 0xFF, 0xFF, 0xFF, 0xFF, 0xBF, 0xDF, 0xEF, 0xF7, 0xFB, 0xFC,
	0xFE, 0xEF, 0xCF, 0xEF, 0xFC, 0xFC, 0xEC, 0xCC, 0xEC, 0xFC, 0xF8, 0xFE, 0xF1, 0xE0, 0xC0, 0x00,
	0x00, 0xFF, 0xFF, 0xFF, 0x83, 0x39, 0x44, 0x82, 0x82, 0x82, 0x92, 0x92, 0x92, 0x92, 0x44, 0x39,
	0x83, 0xFF, 0xFF, 0xF7, 0xC7, 0xFF, 0x7F, 0xF9, 0xF9, 0x7F, 0xFF, 0xC7, 0xF7, 0xFF, 0xFF, 0x03,
	0x79, 0x84, 0x92, 0x92, 0x92, 0x92, 0x82, 0x82, 0x84, 0x79, 0x01, 0xFF, 0xFF, 0xFF, 0xFF, 0x00,
	0x00, 0x7F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE, 0xFE, 0xFE, 0xFE, 0xFE, 0xFE, 0xFE, 0xFE, 0xFE, 0xFF,
	0x1F, 0x9F, 0x9F, 0x9F, 0x9F, 0x1F, 0x18, 0x99, 0x99, 0x18, 0x1F, 0x9F, 0x9F, 0x9F, 0x9F, 0x1F,
	0xFE, 0xFE, 0xFE, 0xFE, 0xFE, 0xFE, 0xFE, 0x7E, 0xFE, 0xFE, 0xFF, 0xFF, 0xFF, 0xFF, 0x7F, 0x00,
	0x00, 0x00, 0x00, 0x03, 0x07, 0x0F, 0x0F, 0x3E, 0x3C, 0x3F, 0x3D, 0x3C, 0x3F, 0x3F, 0x3F, 0x3F,
	0x30, 0x32, 0x32, 0x30, 0x32, 0x32, 0x32, 0x30, 0x32, 0x30, 0x32, 0x32, 0x32, 0x30, 0x32, 0x30,
	0x3F, 0x3F, 0x3F, 0x3E, 0x3C, 0x3D, 0x3F, 0x3C, 0x3E, 0x0F, 0x0F, 0x07, 0x03, 0x00, 0x00, 0x00
};

const unsigned char plane [] PROGMEM = {
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x80,
	0xC0, 0xE0, 0xF0, 0xF8, 0xFC, 0xFE, 0xFF, 0xFF, 0xFE, 0x7C, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x7C, 0xF8, 0xF8, 0xF8, 0xF0,
	0xF0, 0xE0, 0xC0, 0x80, 0x80, 0x80, 0xC0, 0xC0, 0xE0, 0xF0, 0xF8, 0xFC, 0xFE, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0x7F, 0x3F, 0x0F, 0x07, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x03, 0xC7, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x3F, 0x1F,
	0x07, 0x03, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, 0xE0, 0xF0, 0xF8, 0xFC, 0xFE, 0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x7F, 0x3F, 0x3F, 0x1F, 0x1F, 0x3F, 0x7F, 0xFF, 0xFE, 0xF8, 0xF0,
	0xE0, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x00, 0x00,
	0xC0, 0xE0, 0xF0, 0xF8, 0xFC, 0xFE, 0xFF, 0xFF, 0xFF, 0xFF, 0x7F, 0x3F, 0x1F, 0x1F, 0x0F, 0x07,
	0x07, 0x03, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x03,
	0x07, 0x0F, 0x0F, 0x1E, 0xF8, 0xF0, 0xF0, 0xF0, 0xF8, 0xFC, 0xFE, 0xFF, 0x7F, 0x1F, 0x0F, 0x06,
	0x00, 0x01, 0x03, 0x03, 0x07, 0x03, 0x03, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20,
	0x78, 0xFC, 0xFE, 0x7F, 0x3F, 0x1F, 0x1F, 0x0F, 0x07, 0x03, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00
};


#endif /* BITMAPS_H_ */