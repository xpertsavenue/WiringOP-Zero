
/*
 * OneWire.h:
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

#ifndef _ONEWIRE_H_
#define _ONEWIRE_H_

#include <inttypes.h>

#define ONEWIRE_CMD_CONVERTTEMP    0x44
#define ONEWIRE_CMD_RSCRATCHPAD    0xbe
#define ONEWIRE_CMD_WSCRATCHPAD    0x4e
#define ONEWIRE_CMD_CPYSCRATCHPAD  0x48
#define ONEWIRE_CMD_RECEEPROM      0xb8
#define ONEWIRE_CMD_RPWRSUPPLY     0xb4
#define ONEWIRE_CMD_SEARCHROM      0xf0
#define ONEWIRE_CMD_READROM        0x33
#define ONEWIRE_CMD_MATCHROM       0x55
#define ONEWIRE_CMD_SKIPROM        0xcc
#define ONEWIRE_CMD_ALARMSEARCH    0xec

#ifdef __cplusplus
extern "C"
{
#endif
    extern uint64_t OneWireSearchNextAddress(uint8_t instId, uint64_t lastAddress, int *lastDiscrepancy, uint8_t* is_error);
    extern int OneWireReset(uint8_t instId);
    extern int OneWireCrcCheck(unsigned long long int data8x8bit, uint8_t len);
    extern uint8_t OneWireCrc8(unsigned char addr[], unsigned int len);
    extern uint8_t OneWireOneWireInit(uint8_t dataPin, uint8_t skipSetup, uint8_t instId);
    extern void OneWireWriteBit(uint8_t instId, uint8_t bit);
    extern void OneWireWriteByte(uint8_t instId, uint8_t byte);
    extern int OneWireSetDevice(uint8_t instId, unsigned long long int rom);
    extern uint8_t OneWireSearchRom(uint8_t instId);
    extern int OneWireSkipRom(uint8_t instId);
    extern uint8_t OneWireReadByte(uint8_t instId);
    extern uint8_t OneWireReadBit(uint8_t instId);
    extern unsigned long long int OneWireReadRoom(uint8_t instId);
    extern unsigned long long int OneWireGetRomsCnt();
    extern unsigned long long int OneWireGetRom(int idx);
#ifdef __cplusplus
}
#endif

#endif //_ONEWIRE_H_
