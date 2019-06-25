// codebase from https://micro-pi.ru/%D0%BF%D0%BE%D0%B4%D0%BA%D0%BB%D1%8E%D1%87%D0%B5%D0%BD%D0%B8%D0%B5-OneWire-%D0%BA-orange-pi-bpi-rpi/

#include <stdio.h>
#include <inttypes.h>
#include <wiringPi.h>

#include "onewire.h"
#define MAX_SLAVES_CNT 1000

static uint8_t sDataPin[256];
static uint64_t sRoms[MAX_SLAVES_CNT];
static uint64_t sRomsCnt = 0;


uint8_t OneWireOneWireInit(uint8_t dataPin, uint8_t skipSetup, uint8_t instId)
{
	if ((!skipSetup) && wiringPiSetup() == -1)
	{
		printf("initialization failed");
		return 255;
	}
	sDataPin[instId] = dataPin;
	pinMode(dataPin, INPUT);

	return 0;
}

/*
 * reset
 */
int OneWireReset(uint8_t instId)
{
	int response;
	uint8_t pin = sDataPin[instId];
	pinMode(pin, OUTPUT);
	digitalWrite(pin, LOW);
	delayMicroseconds(480);

	// Когда ONE WIRE устройство обнаруживает положительный перепад, он ждет от 15us до 60us
	pinMode(pin, INPUT);
	delayMicroseconds(60);

	// и затем передает импульс присутствия, перемещая шину в логический «0» на длительность от 60us до 240us.
	response = digitalRead(pin);
	delayMicroseconds(410);

	// если 0, значит есть ответ от датчика, если 1 - нет
	return response;
}

/*
 * send 1 bit
 */
void OneWireWriteBit(uint8_t instId, uint8_t bit)
{
	uint8_t pin = sDataPin[instId];
	if (bit & 1)
	{
		// логический «0» на 10us
		pinMode(pin, OUTPUT);
		digitalWrite(pin, LOW);
		delayMicroseconds(10);
		pinMode(pin, INPUT);
		delayMicroseconds(55);
	}
	else
	{
		// логический «0» на 65us
		pinMode(pin, OUTPUT);
		digitalWrite(pin, LOW);
		delayMicroseconds(65);
		pinMode(pin, INPUT);
		delayMicroseconds(5);
	}
}

/*
 * send 1 byte
 */
void OneWireWriteByte(uint8_t instId, uint8_t byte)
{
	uint8_t i = 8;
	while (i--)
	{
		OneWireWriteBit(instId, byte & 1);
		byte >>= 1;
	}
}

/*
 * get 1 bit
 */
uint8_t OneWireReadByte(uint8_t instId)
{
	uint8_t i = 8, byte = 0;
	while (i--)
	{
		byte >>= 1;
		byte |= (OneWireReadBit(instId) << 7);
	}
	return byte;
}

/*
 * get 1 byte
 */
uint8_t OneWireReadBit(uint8_t instId)
{
	uint8_t pin = sDataPin[instId];
	uint8_t bit = 0;
	// logic '0' for 3us
	pinMode(pin, OUTPUT);
	digitalWrite(pin, LOW);
	delayMicroseconds(3);

	// make free line and wait 10us
	pinMode(pin, INPUT);
	delayMicroseconds(10);

	// read value
	bit = digitalRead(pin);

	// wait 45us and return value
	delayMicroseconds(45);
	return bit;
}

/*
 * read ROM of slave device (code 64 bits)
 */
uint64_t OneWireReadRoom(uint8_t instId)
{
	uint64_t oneWireDevice;
	if (OneWireReset(instId) == 0)
	{
		OneWireWriteByte(instId, ONEWIRE_CMD_READROM);
		//  code of famaly
		oneWireDevice = OneWireReadByte(instId);
		// serial number
		oneWireDevice |= (uint16_t)OneWireReadByte(instId) << 8 | (uint32_t)OneWireReadByte(instId) << 16 | (uint32_t)OneWireReadByte(instId) << 24 | (uint64_t)OneWireReadByte(instId) << 32 | (uint64_t)OneWireReadByte(instId) << 40 | (uint64_t)OneWireReadByte(instId) << 48;
		// CRC
		oneWireDevice |= (uint64_t)OneWireReadByte(instId) << 56;
	}
	else
	{
		return 1;
	}
	return oneWireDevice;
}

/*
 * Command for matching ROM, allows to acces to selected device
 */
int OneWireSetDevice(uint8_t instId, uint64_t rom)
{
	uint8_t i = 64;
	int res_stat = 0;
	res_stat = OneWireReset(instId);

	if(res_stat != 0)
	{
		return 255;
	}

	OneWireWriteByte(instId, ONEWIRE_CMD_MATCHROM);
	while (i--)
	{
		OneWireWriteBit(instId, rom & 1);
		rom >>= 1;
	}

	return 0;
}

/*
 * check CRC, returns '0', if no errors
 * and not '0', if errors exists
 */
int OneWireCrcCheck(uint64_t data8x8bit, uint8_t len)
{
	uint8_t dat, crc = 0, fb, stByte = 0;
	do
	{
		dat = (uint8_t)(data8x8bit >> (stByte * 8));
		// bits counter in byte
		for (int i = 0; i < 8; i++)
		{
			fb = crc ^ dat;
			fb &= 1;
			crc >>= 1;
			dat >>= 1;
			if (fb == 1)
			{
				crc ^= 0x8c; // polynom
			}
		}
		stByte++;
	} while (stByte < len); // bytes counter in array
	return crc;
}

uint8_t OneWireCrc8(unsigned char addr[], unsigned int len)
{
	uint8_t crc = 0;
	while (len--)
	{
		uint8_t inbyte = *addr++;
		for (uint8_t i = 8; i; i--)
		{
			uint8_t mix = (crc ^ inbyte) & 0x01;
			crc >>= 1;
			if (mix)
			{
				crc ^= 0x8c;
			}
			inbyte >>= 1;
		}
	}
	return crc;
}

/*
 * search devices
 */
uint8_t OneWireSearchRom(uint8_t instId)
{
	sRomsCnt = MAX_SLAVES_CNT;
	uint64_t lastAddress = 0;
	int lastDiscrepancy = 0;
	int err = 0;
	int i = 0;
	do
	{
		do
		{

			uint8_t is_err = 0;
			lastAddress = OneWireSearchNextAddress(instId, lastAddress, &lastDiscrepancy, &is_err);
			if(is_err == 0)
			{
				int crc = OneWireCrcCheck(lastAddress, 8);
				if (crc == 0)
				{
					sRoms[i++] = lastAddress;
					err = 0;
				}
				else
				{
					err++;
				}
			}
			else
			{
				err++;
				if (err > 3)
				{
					return 255;
				}
			}

		} while (err != 0);
	} while (lastDiscrepancy != 0 && i < sRomsCnt);
	sRomsCnt = i;

	return 0;
}

unsigned long long int OneWireGetRomsCnt()
{
	return sRomsCnt;
}

unsigned long long int OneWireGetRom(int idx)
{
	return sRoms[idx];
}

/*
 * search next connected device
 */
uint64_t OneWireSearchNextAddress(uint8_t instId, uint64_t lastAddress, int *lastDiscrepancy, uint8_t* is_error)
{
	*is_error = 0;
	uint64_t newAddress = 0;
	int searchDirection = 0;
	int idBitNumber = 1;
	int lastZero = 0;
	int reset_stat = 0;
	reset_stat = OneWireReset(instId);
	if(reset_stat != 0)
	{
		*is_error = 1;
		return 0;
	}
	OneWireWriteByte(instId, ONEWIRE_CMD_SEARCHROM);

	while (idBitNumber < 65)
	{
		int idBit = OneWireReadBit(instId);
		int cmpIdBit = OneWireReadBit(instId);

		// id_bit = cmp_id_bit = 1
		if (idBit == 1 && cmpIdBit == 1)
		{
			printf("error: id_bit = cmp_id_bit = 1\n");
			*is_error = 0;
		}
		else if (idBit == 0 && cmpIdBit == 0)
		{
			// id_bit = cmp_id_bit = 0
			if (idBitNumber == *lastDiscrepancy)
			{
				searchDirection = 1;
			}
			else if (idBitNumber > *lastDiscrepancy)
			{
				searchDirection = 0;
			}
			else
			{
				if ((uint8_t)(lastAddress >> (idBitNumber - 1)) & 1)
				{
					searchDirection = 1;
				}
				else
				{
					searchDirection = 0;
				}
			}
			if (searchDirection == 0)
			{
				lastZero = idBitNumber;
			}
		}
		else
		{
			// id_bit != cmp_id_bit
			searchDirection = idBit;
		}
		newAddress |= ((uint64_t)searchDirection) << (idBitNumber - 1);
		OneWireWriteBit(instId, searchDirection);
		idBitNumber++;
	}
	*lastDiscrepancy = lastZero;
	return newAddress;
}

/*
 * skip ROM
 */
int OneWireSkipRom(uint8_t instId)
{
	OneWireReset(instId);
	OneWireWriteByte(instId, ONEWIRE_CMD_SKIPROM);
}