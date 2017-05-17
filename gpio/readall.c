/*
 * readall.c:
 *	The readall functions - getting a bit big, so split them out.
 *	Copyright (c) 2012-2013 Gordon Henderson
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


#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <ctype.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>

#include <wiringPi.h>

extern int wpMode ;

#ifndef TRUE
#  define       TRUE    (1==1)
#  define       FALSE   (1==2)
#endif

/*
 * doReadallExternal:
 *	A relatively crude way to read the pins on an external device.
 *	We don't know the input/output mode of pins, but we can tell
 *	if it's an analog pin or a digital one...
 *********************************************************************************
 */

static void doReadallExternal (void)
{
  int pin ;

  printf ("+------+---------+--------+\n") ;
  printf ("|  Pin | Digital | Analog |\n") ;
  printf ("+------+---------+--------+\n") ;

  for (pin = wiringPiNodes->pinBase ; pin <= wiringPiNodes->pinMax ; ++pin)
    printf ("| %4d |  %4d   |  %4d  |\n", pin, digitalRead (pin), analogRead (pin)) ;

  printf ("+------+---------+--------+\n") ;
}


/*
 * doReadall:
 *	Read all the GPIO pins
 *	We also want to use this to read the state of pins on an externally
 *	connected device, so we need to do some fiddling with the internal
 *	wiringPi node structures - since the gpio command can only use
 *	one external device at a time, we'll use that to our advantage...
 *********************************************************************************
 */

static char *alts [] =
{
  "IN", "OUT", "ALT5", "ALT4", "ALT0", "ALT1", "ALT2", "ALT3"
} ;

//Beckert
// guenter anfang
static int physToWpi [64] =
{
  -1,        // 0
  
  -1,  -1,   // 1, 2
   8,  -1,   // 3, 4
   9,  -1,   // 5, 6
   7,  15,   // 7, 8
  -1,  16,   // 9, 10
   0,   1,   //11, 12
   2,  -1,   //13, 14
   3,   4,   //15, 16
  -1,   5,   //17, 18
  12,  -1,   //19, 20
  13,   6,   //21, 22
  14,  10,   //23, 24
  -1,  11,   //25, 26
  
  30,  -1,   //27, 28
  
  -1,  -1,   //29, 30
  -1,  -1,   //31, 32
  -1,  -1,   //33, 34
  -1,  -1,   //35, 36
  -1,  -1,   //37, 38
  -1,  -1,   //39, 40
   -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, //41-> 55
   -1, -1, -1, -1, -1, -1, -1, -1 // 56-> 63
} ;
//guenter ende
//Beckert

//Beckert
//guenter orange pi
static char *physNames [64] = 
{
      NULL,

 "    3.3v", "5v      ",
 "   SDA.0", "5V      ",
 "   SCL.0", "0v      ",
 "  GPIO.7", "TxD3    ",
 "      0v", "RxD3    ",
 "    RxD2", "GPIO.1  ",
 "    TxD2", "0v      ",
 "    CTS2", "GPIO.4  ",
 "    3.3v", "GPIO.5  ",
 "    MOSI", "0v      ",
 "    MISO", "RTS2    ",
 "    SCLK", "CE0     ",
 "      0v", "GPIO.11 ",
 
 "STAT-LED", "PWR-LED ",
 
       NULL, NULL,
       NULL, NULL,
       NULL, NULL,
       NULL, NULL,
       NULL, NULL,
       NULL, NULL,
       NULL, NULL,
       NULL, NULL,
       NULL, NULL,
       NULL, NULL,
       NULL, NULL,
       NULL, NULL,
       NULL, NULL,
	   NULL, NULL,
	   NULL, NULL,
	   NULL, NULL,
	   NULL, NULL,
	   NULL,
} ;
// guenter ende
//Beckert



/*
 * readallPhys:
 *	Given a physical pin output the data on it and the next pin:
 *| BCM | wPi |   Name  | Mode | Val| Physical |Val | Mode | Name    | wPi | BCM |
 *********************************************************************************
 */

static void readallPhys (int physPin)
{
  int pin ;

  if (physPinToGpio (physPin) == -1)
    printf ("|     |    ") ;
  else
    printf ("| %3d | %3d", physPinToGpio (physPin), physToWpi [physPin]) ;

  printf (" | %s", physNames [physPin]) ;

  if (physToWpi [physPin] == -1)
    printf (" |      |  ") ;
  else
  {
    /**/ if (wpMode == WPI_MODE_GPIO)
      pin = physPinToGpio (physPin) ;
    else if (wpMode == WPI_MODE_PHYS)
      pin = physPin ;
    else
      pin = physToWpi [physPin] ;

    printf (" | %4s", alts [getAlt (pin)]) ;
    printf (" | %d", digitalRead (pin)) ;
  }

// Pin numbers:

  printf (" | %2d", physPin) ;
  ++physPin ;
  printf (" || %-2d", physPin) ;

// Same, reversed

  if (physToWpi [physPin] == -1)
    printf (" |   |     ") ;
  else
  {
    /**/ if (wpMode == WPI_MODE_GPIO)
      pin = physPinToGpio (physPin) ;
    else if (wpMode == WPI_MODE_PHYS)
      pin = physPin ;
    else
      pin = physToWpi [physPin] ;

    printf (" | %d", digitalRead (pin)) ;
    printf (" | %-4s", alts [getAlt (pin)]) ;
  }

  printf (" | %-5s", physNames [physPin]) ;

  if (physToWpi     [physPin] == -1)
    printf (" |     |    ") ;
  else
    printf (" | %-3d | %-3d", physToWpi [physPin], physPinToGpio (physPin)) ;

  printf (" |\n") ;
}


void cmReadall (void)
{
  int pin ;

  printf ("+-----+------+-------+      +-----+------+-------+\n") ;
  printf ("| Pin | Mode | Value |      | Pin | Mode | Value |\n") ;
  printf ("+-----+------+-------+      +-----+------+-------+\n") ;

  for (pin = 0 ; pin < 28 ; ++pin)
  {
    printf ("| %3d ", pin) ;
    printf ("| %-4s ", alts [getAlt (pin)]) ;
    printf ("| %s  ", digitalRead (pin) == HIGH ? "High" : "Low ") ;
    printf ("|      ") ;
    printf ("| %3d ", pin + 28) ;
    printf ("| %-4s ", alts [getAlt (pin + 28)]) ;
    printf ("| %s  ", digitalRead (pin + 28) == HIGH ? "High" : "Low ") ;
    printf ("|\n") ;
  }

  printf ("+-----+------+-------+      +-----+------+-------+\n") ;
}


/*
 * abReadall:
 *	Read all the pins on the model A or B.
 *********************************************************************************
 */

void abReadall (int model, int rev)
{
  int pin ;
  char *type ;

  if (model == PI_MODEL_A)
    type = " A" ;
  else
    if (rev == PI_VERSION_2)
      type = "B2" ;
    else
      type = "B1" ;

  printf ("+-----+-----+---------+------+---+-Model %s-+---+------+---------+-----+-----+\n", type) ;
  printf ("| BCM | wPi |   Name  | Mode | V | Physical | V | Mode | Name    | wPi | BCM |\n") ;
  printf ("+-----+-----+---------+------+---+----++----+---+------+---------+-----+-----+\n") ;
  for (pin = 1 ; pin <= 26 ; pin += 2)
    readallPhys (pin) ;

  if (rev == PI_VERSION_2) // B version 2
  {
    printf ("+-----+-----+---------+------+---+----++----+---+------+---------+-----+-----+\n") ;
    for (pin = 51 ; pin <= 54 ; pin += 2)
      readallPhys (pin) ;
  }

  printf ("+-----+-----+---------+------+---+----++----+---+------+---------+-----+-----+\n") ;
  printf ("| BCM | wPi |   Name  | Mode | V | Physical | V | Mode | Name    | wPi | BCM |\n") ;
  printf ("+-----+-----+---------+------+---+-Model %s-+---+------+---------+-----+-----+\n", type) ;
}


/*
 * bPlusReadall:
 *	Read all the pins on the model B+
 *********************************************************************************
 */

void bPlusReadall (void)
{
  int pin ;

  printf ("+-----+-----+---------+------+---+--B Plus--+---+------+---------+-----+-----+\n") ;
  printf ("| BCM | wPi |   Name  | Mode | V | Physical | V | Mode | Name    | wPi | BCM |\n") ;
  printf ("+-----+-----+---------+------+---+----++----+---+------+---------+-----+-----+\n") ;
  for (pin = 1 ; pin <= 40 ; pin += 2)
    readallPhys (pin) ;
  printf ("+-----+-----+---------+------+---+----++----+---+------+---------+-----+-----+\n") ;
  printf ("| BCM | wPi |   Name  | Mode | V | Physical | V | Mode | Name    | wPi | BCM |\n") ;
  printf ("+-----+-----+---------+------+---+--B Plus--+---+------+---------+-----+-----+\n") ;
}

//add for BananaPro by lemaker team
void BPReadAll(void)
{
  int pin ;

  printf ("+-----+-----+---------+------+---+--Banana Pro--+---+------+---------+-----+-----+\n") ;
  printf ("| BCM | wPi |   Name  | Mode | V | Physical | V | Mode | Name    | wPi | BCM |\n") ;
  printf ("+-----+-----+---------+------+---+----++----+---+------+---------+-----+-----+\n") ;
  for (pin = 1 ; pin <= 40 ; pin += 2)
    readallPhys (pin) ;
  printf ("+-----+-----+---------+------+---+----++----+---+------+---------+-----+-----+\n") ;
  printf ("| BCM | wPi |   Name  | Mode | V | Physical | V | Mode | Name    | wPi | BCM |\n") ;
  printf ("+-----+-----+---------+------+---+--Banana Pro--+---+------+---------+-----+-----+\n") ;	
}
//end 2014.09.26

// changed by Christian Beckert
void OrangePiReadAll(void)
{
  int pin ;

  printf ("+-----+-----+----------+------+--Orange Pi Zero--+---+------+---------+-----+--+\n") ;
  printf ("| H2+ | wPi |   Name   | Mode | V | Physical | V | Mode | Name     | wPi | H2+ |\n") ;
  printf ("+-----+-----+----------+------+---+----++----+---+------+----------+-----+-----+\n") ;
  for (pin = 1 ; pin <= 26 ; pin += 2)
    readallPhys (pin) ;
  printf ("+-----+-----+----------+------+---+---LEDs---+---+------+----------+-----+-----+\n") ;	
  readallPhys (27) ;
  printf ("+-----+-----+----------+------+---+-----+----+---+------+----------+-----+-----+\n") ;
  printf ("| H2+ | wPi |   Name   | Mode | V | Physical | V | Mode | Name     | wPi | H2+ |\n") ;	
  printf ("+-----+-----+----------+------+--Orange Pi Zero--+---+------+---------+-----+--+\n") ;	
}
// changed by Christian Beckert

void doReadall (void)
{
  int model, rev, mem, maker, overVolted ;

  if (wiringPiNodes != NULL)	// External readall
  {
    doReadallExternal () ;
    return ;
  }

  piBoardId (&model, &rev, &mem, &maker, &overVolted) ;

  /**/ if ((model == PI_MODEL_A) || (model == PI_MODEL_B))
    abReadall (model, rev) ;
  else if (model == PI_MODEL_BP) 
    bPlusReadall () ;
  else if (model == PI_MODEL_CM)
    cmReadall () ;
  else if (model == PI_MODEL_OPZ) //add for BananaPro by lemaker team
     OrangePiReadAll();	//guenter / // changed by Christian Beckert 
  else
    printf ("Oops - unable to determine board type... model: %d\n", model) ;
}
