#include <stdio.h>
#include <inttypes.h>
#include <wiringPi.h>

#include "hx711.h"

//channel A, gain 128
#define GAIN_128 128

//channel A, gain 64
#define GAIN_64 64

//channel B, gain 32
#define GAIN_32 32

uint8_t sGainBits[256];
float sScale[256];
int sOffset[256];

uint8_t sClockPin[256];
uint8_t sDataPin[256];

uint8_t hx711Setup(uint8_t clockPin, uint8_t dataPin, uint8_t skipSetup, uint8_t instId)
{
	sGainBits[instId] = 1;
	sScale[instId] = 1.0f;
	sOffset[instId] = 0;
	sClockPin[instId] = clockPin;
	sDataPin[instId] = dataPin;
	if ((!skipSetup) && wiringPiSetup() == -1)
	{
		printf("initialization failed");
		return 255;
	}
	pinMode(sClockPin[instId], OUTPUT);
	pinMode(sDataPin[instId], INPUT);
	return 0;
}

uint8_t hx711IsReady(uint8_t instId)
{
	return digitalRead(sDataPin[instId]) == LOW;
}

void hx711SetGain(uint8_t gain, uint8_t instId)
{
	switch (gain)
	{
	case GAIN_128:
		sGainBits[instId] = 1;
		break;
	case GAIN_64:
		sGainBits[instId] = 3;
		break;
	case GAIN_32:
		sGainBits[instId] = 2;
		break;
	default:
		//invalid gain, ignore
		break;
	}

	digitalWrite(sClockPin[instId], LOW);
	hx711Read(instId);
}

int hx711Read(uint8_t instId)
{
	// wait for the chip to become ready
	while (!hx711IsReady(instId))
		;

	int data = 0;
	// pulse the clock pin 24 times to read the data
	for (uint8_t i = 24; i--;)
	{
		digitalWrite(sClockPin[instId], HIGH);

		digitalRead(sDataPin[instId]);
		data |= (digitalRead(sDataPin[instId]) << i);

		digitalWrite(sClockPin[instId], LOW);
	}

	// set the channel and the gain factor for the next reading using the clock pin
	for (int i = 0; i < sGainBits[instId]; i++)
	{
		digitalWrite(sClockPin[instId], HIGH);
		digitalWrite(sClockPin[instId], LOW);
	}

	if (data & 0x800000)
	{
		data |= (long)~0xffffff;
	}

	return data;
}

int hx711ReadAverage(uint8_t times, uint8_t instId)
{
	int64_t sum = 0;
	for (uint8_t i = 0; i < times; i++)
	{
		sum += hx711Read(instId);
	}
	return sum / times;
}

int hx711GetRawValue(uint8_t times, uint8_t instId)
{
	return hx711ReadAverage(times, instId) - sOffset[instId];
}

float hx711GetUnits(uint8_t times, uint8_t instId)
{
	return hx711GetRawValue(times, instId) / sScale[instId];
}

void hx711Tare(uint8_t times, uint8_t instId)
{
	uint64_t sum = hx711ReadAverage(times, instId);
	hx711SetOffset(sum, instId);
}

void hx711SetScale(float scale, uint8_t instId)
{
	sScale[instId] = scale;
}

void hx711SetOffset(int offset, uint8_t instId)
{
	sOffset[instId] = offset;
}

void hx711PowerDown(uint8_t instId)
{
	digitalWrite(sClockPin[instId], LOW);
	digitalWrite(sClockPin[instId], HIGH);
}

void hx711PowerUp(uint8_t instId)
{
	digitalWrite(sClockPin[instId], LOW);
}

int hx711GetOffset(uint8_t instId)
{
	return sOffset[instId];
}

float hx711GetScale(uint8_t instId)
{
	return sScale[instId];
}
