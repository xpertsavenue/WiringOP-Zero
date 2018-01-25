
/*
 * hx711.h:
 * 24-Bit Analog-to-Digital Converter (ADC) for weight measure
 * Adaptation of https://github.com/pommoisel/HX711
 *
 * Copyright (c) 2013 Gordon Henderson.
 ***********************************************************************
 * This file is part of wiringPi:
 *	https://projects.drogon.net/raspberry-pi/wiringpi/
 *
 *    wiringPi is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU Lesser General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    wiringPi is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU Lesser General Public License for more details.
 *
 *    You should have received a copy of the GNU Lesser General Public License
 *    along with wiringPi.  If not, see <http://www.gnu.org/licenses/>.
 ***********************************************************************
 */

#ifndef _HX711_H_
#define _HX711_H_

#include <inttypes.h>

#ifdef __cplusplus
extern "C" {
#endif

extern uint8_t hx711Setup(uint8_t clockPin, uint8_t dataPin, uint8_t skipSetup, uint8_t instId);
extern uint8_t hx711IsReady(uint8_t instId);
extern void hx711SetGain(uint8_t gain, uint8_t instId);
extern int hx711Read(uint8_t instId);

extern int hx711ReadAverage(uint8_t times, uint8_t instId);
extern int hx711GetRawValue(uint8_t times, uint8_t instId);
extern float hx711GetUnits(uint8_t times, uint8_t instId);
extern void hx711Tare(uint8_t times, uint8_t instId);
extern void hx711SetScale(float scale, uint8_t instId);
extern void hx711SetOffset(int offset, uint8_t instId);
extern void hx711PowerDown(uint8_t instId);
extern void hx711PowerUp(uint8_t instId);
extern int hx711GetOffset(uint8_t instId);
extern float hx711GetScale(uint8_t instId);

#ifdef __cplusplus
}
#endif

#endif //_HX711_H_
