# WiringOP-Zero

WiringPi / WiringOP libary for the Orange Pi Zero with 26 pin GPIO header

This is a modified WiringPi for specially OrangePi Zero. 

GPIO is fully working, i2c and Spi not testet yet!


## Download
    git clone https://github.com/xpertsavenue/WiringOP-Zero.git
## Installation
    cd WiringOP-Zero
    chmod +x ./build
    sudo ./build

## Test
```    
christian@orangepizero:~/WiringOP-Zero$ gpio readall
 +-----+-----+----------+------+--Orange Pi Zero--+---+------+---------+-----+--+
 | H2+ | wPi |   Name   | Mode | V | Physical | V | Mode | Name     | wPi | H2+ |
 +-----+-----+----------+------+---+----++----+---+------+----------+-----+-----+
 |     |     |     3.3v |      |   |  1 || 2  |   |      | 5v       |     |     |
 |  12 |   8 |    SDA.0 | ALT5 | 0 |  3 || 4  |   |      | 5V       |     |     |
 |  11 |   9 |    SCL.0 | ALT5 | 0 |  5 || 6  |   |      | 0v       |     |     |
 |   6 |   7 |   GPIO.7 | ALT3 | 0 |  7 || 8  | 0 | ALT5 | TxD3     | 15  | 198 |
 |     |     |       0v |      |   |  9 || 10 | 0 | ALT5 | RxD3     | 16  | 199 |
 |   1 |   0 |     RxD2 | ALT5 | 0 | 11 || 12 | 0 | ALT3 | GPIO.1   | 1   | 7   |
 |   0 |   2 |     TxD2 | ALT5 | 0 | 13 || 14 |   |      | 0v       |     |     |
 |   3 |   3 |     CTS2 | ALT3 | 0 | 15 || 16 | 0 | ALT4 | GPIO.4   | 4   | 19  |
 |     |     |     3.3v |      |   | 17 || 18 | 0 | ALT4 | GPIO.5   | 5   | 18  |
 |  15 |  12 |     MOSI | ALT5 | 0 | 19 || 20 |   |      | 0v       |     |     |
 |  16 |  13 |     MISO | ALT5 | 0 | 21 || 22 | 0 | ALT3 | RTS2     | 6   | 2   |
 |  14 |  14 |     SCLK | ALT5 | 0 | 23 || 24 | 0 | ALT5 | CE0      | 10  | 13  |
 |     |     |       0v |      |   | 25 || 26 | 0 | ALT3 | GPIO.11  | 11  | 10  |
 +-----+-----+----------+------+---+---LEDs---+---+------+----------+-----+-----+
 |  17 |  30 | STAT-LED |  OUT | 0 | 27 || 28 |   |      | PWR-LED  |     |     |
 +-----+-----+----------+------+---+-----+----+---+------+----------+-----+-----+
 | H2+ | wPi |   Name   | Mode | V | Physical | V | Mode | Name     | wPi | H2+ |
 +-----+-----+----------+------+--Orange Pi Zero--+---+------+---------+-----+--+

```    
To control red LED on the board run for example this:
```
gpio write 30 0 
```
and to turn off this
```
gpio write 30 0 
```

Thanks to zhaolei, who created the base libary for this one!


