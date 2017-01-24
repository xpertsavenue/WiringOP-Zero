/************************************************************
  Copyright (C), 2014, lemaker.org.
  FileName:      lnIOBoard.c
  Author:         peter
  Version :      1.0
  Date:          2014.10.10 
  Description: be used to test LN IOBoard
  Function List: 
  History:
      <author>    <time>     <version >   <desc>
          peter     2014.10.10	  1.0		  create
  
  Compile: gcc lnIOBoard.c -o lnIOBoard -lwiringPiDev -lwiringPi
***********************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <wiringPi.h>
#include <lcd.h>
#include <wiringSerial.h>

#define FAIL -1

#define I2C_BASE 100
#define PCF8591_CHANNEL 4
#define LED_MAX_NUM 8

void uSage(void)
{
	printf("Usage: lnIOBoard [i2c | lcd | led | spi | uart]\n");
}

void at45BufWrite(unsigned char val)
{
	int ret;
	unsigned char wBuf[5] = {0x84, 0xff, 0x00, 0x02,0xff}; //write data to buffer 1

	wBuf[4] = val;
	
	printf("SPI: write data: 0x%02x\n", wBuf[4]);
	
	ret = wiringPiSPIDataRW(0, wBuf, sizeof(wBuf));
	if(ret < 0)
	{
		printf("Write data to the AT45DB041D failed!\n");
		return;
	}
}

unsigned char at45BufRead(void)
{
	int ret;
  unsigned char rBuf[6] = {0xD4, 0xff, 0x00, 0x02, 0xff, 0xff}; //read data from buffer 1

	ret = wiringPiSPIDataRW(0, rBuf, sizeof(rBuf));
	if(ret < 0)
	{
		printf("Read data from the AT45DB041D failed!\n");
		return;
	}
	return rBuf[5];
}

int uartTxtoRxTest(int txfd, int rxfd, unsigned char tCh)
{
	int flg;
	
	serialPutchar(txfd, tCh);
	delay(100);  //delay 100ms
	if(tCh == serialGetchar(rxfd))
	{
		flg  =  1;
	}
	else
	{
		flg  = 0;
	}	

	return flg;
}
/*
*	Function: ioBoardI2cTest
* Description: be used to test I2C interface on the board
*/
void ioBoardI2cTest(int addr, int outv, int chmax)
{
	int val;
	int ch;
	
	pcf8591Setup(I2C_BASE, addr); //address >>1
	
	analogWrite(I2C_BASE, outv); //set the analog output(3.3v*128/256)
	printf("I2C: please measure the output voltage!and Expectation value: 1.65v\n");

	for(ch=0; ch<chmax; ch++)
	{
		val = analogRead(I2C_BASE + ch);
		printf("I2C: the channel %d input voltage: %d and expectation value range from 0 to 256\n", ch, val);
	}
}

/*
*	Function: ioBoardLcdTest
* Description: be used to test Lcd interface on the board
*/
void ioBoardLcdTest(int bits, int rows, int cols)
{
	int lcdHandle;
	lcdHandle = lcdInit (rows, cols, 8, 21, 23, 0,1,2,3,4,5,6,7) ;
	if(lcdHandle < 0)
	{
		printf("Lcd init failed!\n");
		return;
	}

	printf("LCD: the LCD will display character, please view!\n");
	
	lcdPosition (lcdHandle, 0, 0) ; lcdPuts (lcdHandle, "Lcd Test OK") ;
	lcdPosition (lcdHandle, 0, 1) ; lcdPuts (lcdHandle, "www.lemaker.org") ;
}

/*
*	Function: ioBoardLedTest
* Description: be used to test led on the board
*/
void ioBoardLedTest(int cycles, int dly)
{
	int pinLed;
	int cnt = 0;
	
	for(pinLed=0; pinLed<LED_MAX_NUM; pinLed++)
	{
		pinMode(pinLed, OUTPUT);
	}

	pinLed = 0;

	printf("LED: the leds will blink, please view!\n");
	
	while(cnt < cycles) //blink three times
	{
		digitalWrite(pinLed, HIGH);
		delay(dly);
		digitalWrite(pinLed, LOW);
		delay(dly);
		pinLed ++;
		if(pinLed > LED_MAX_NUM-1)
		{
			pinLed = 0;
			cnt++;
		}
	}
}


/*
*	Function: ioBoardSpiTest
* Description: be used to test SPI interface on the board
*
*   host             slave
*   MISO  < - >   MISO
*   MOSI  < - >   MOSI
*   CLK    < - >   CLK
*   CE0/1 < - >   CS
*/
void ioBoardSpiTest()
{
	int fd;
	unsigned char wData = 0x34; //write value
	unsigned char retv;
	
	system("gpio load spi");  //load spi driver
	
	fd = wiringPiSPISetup(0, 5000000); //channel:0  5M
	if(fd < 0)
	{
		printf("Open the SPI device failed!\n");
		return;
	}
   at45BufWrite(wData);
	delay(100); //delay 100ms
	retv = at45BufRead();
	
	printf("SPI: read  data: 0x%02x\n", retv);

	if(wData == retv)
	{
		printf("SPI: the spi interface is OK!\n");
	}
	else
	{
		printf("SPI: the spi interface is not OK! Please check out!\n");
	}
	close(fd);
}

/*
*	Function: ioBoardUartTest
* Description: be used to test UART3 and UART7 interface on the board
*
*   H2               H3
*   TX3  < - >   RX7
*   RX3  < - >   TX7
*/
int ioBoardUartTest()
{
	int uart3fd;
	int uart7fd;
	int uart3flg;
	int uart7flg;
	char tCh = 'x';
	char gCh;
	
	uart3fd = serialOpen ("/dev/ttyS2", 115200);
	if(uart3fd < 0)
	{
    	fprintf (stderr, "Unable to open UART3 device: %s\n", strerror (errno)) ;
    	return FAIL ;		
	}
	
	uart7fd =  serialOpen ("/dev/ttyS3", 115200);
	if(uart7fd < 0)
	{
    	fprintf (stderr, "Unable to open UART7 device: %s\n", strerror (errno)) ;
    	return FAIL ;		
	}

	if((1== uartTxtoRxTest(uart3fd, uart7fd, 't')) && (1== uartTxtoRxTest(uart7fd, uart3fd, 'r')))
	{
		printf("UART:the uart3 and uart7 interface is OK!\n");
	}
	else 
	{
		printf("UART:the uart3 and uart7 interface is not OK! Please check out!\n");
	}
	serialClose(uart3fd);
	serialClose(uart7fd);

	return 0;
}
void ioBoardTest(void)
{
	printf("------------------LN IOBoard self-check start-------------------\n");
	ioBoardLcdTest(8, 2, 16);
	ioBoardLedTest(3, 500); 
	ioBoardI2cTest(0x48, 0x80, PCF8591_CHANNEL);
	ioBoardSpiTest();
	ioBoardUartTest();
	printf("------------------LN IOBoard self-check over-------------------\n");
}

int main(int argc, char **argv)
{
	wiringPiSetup();
	
	if(argc != 2)
	{
		ioBoardTest();
		return 0;
	}

	if (0 == strcasecmp(argv[1], "i2c"))
	{
		ioBoardI2cTest(0x48, 0x80, PCF8591_CHANNEL);
	}
	else if (0 == strcasecmp(argv[1], "lcd"))
	{
		ioBoardLcdTest(8, 2, 16);
	}
	else if (0 == strcasecmp(argv[1], "led"))
	{
		ioBoardLedTest(3, 500); 
	}
	else if (0 == strcasecmp(argv[1], "spi"))
	{
		ioBoardSpiTest();
	}
	else if(0 == strcasecmp(argv[1], "uart"))
	{
		ioBoardUartTest();
	}
	else
	{
		uSage();
		return FAIL;
	}
	return 0;
}
