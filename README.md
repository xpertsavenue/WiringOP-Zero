# WiringOP-Zero

WiringPi / WiringOP libary for the Orange Pi Zero with 26 pin GPIO header

This is a modified WiringPi for specially OrangePi Zero. 



## Download
### For Orangepi Pi
    git clone https://github.com/xpertsavenue/WiringOP-Zero.git
## Installation
    cd WiringOP-Zero
    chmod +x ./build
    sudo ./build

```    
christian@orangepizero:~/WiringOP-Zero$ gpio readall
 +-----+-----+----------+------+Orange Pi Zero+---+---+------+---------+-----+--+
 | BCM | wPi |   Name   | Mode | V | Physical | V | Mode | Name     | wPi | BCM |
 +-----+-----+----------+------+---+----++----+---+------+----------+-----+-----+
 |     |     |     3.3v |      |   |  1 || 2  |   |      | 5v       |     |     |
 |  12 |   8 |    SDA.0 |  OUT | 1 |  3 || 4  |   |      | 5V       |     |     |
 |  11 |   9 |    SCL.0 |  OUT | 1 |  5 || 6  |   |      | 0v       |     |     |
 |   6 |   7 |   GPIO.7 |  OUT | 1 |  7 || 8  | 0 | OUT  | TxD3     | 15  | 198 |
 |     |     |       0v |      |   |  9 || 10 | 0 | OUT  | RxD3     | 16  | 199 |
 |   1 |   0 |     RxD2 | ALT5 | 0 | 11 || 12 | 0 | ALT3 | GPIO.1   | 1   | 7   |
 |   0 |   2 |     TxD2 | ALT5 | 0 | 13 || 14 |   |      | 0v       |     |     |
 |   3 |   3 |     CTS2 |  OUT | 0 | 15 || 16 | 0 | ALT4 | GPIO.4   | 4   | 19  |
 |     |     |     3.3v |      |   | 17 || 18 | 0 | ALT4 | GPIO.5   | 5   | 18  |
 |  15 |  12 |     MOSI | ALT3 | 0 | 19 || 20 |   |      | 0v       |     |     |
 |  16 |  13 |     MISO |  OUT | 1 | 21 || 22 | 0 | ALT3 | RTS2     | 6   | 2   |
 |  14 |  14 |     SCLK |  OUT | 1 | 23 || 24 | 0 | ALT3 | CE0      | 10  | 13  |
 |     |     |       0v |      |   | 25 || 26 | 0 | OUT  | GPIO.11  | 11  | 10  |
 +-----+-----+----------+------+---+----++----+---+------+----------+-----+-----+
 | BCM | wPi |   Name   | Mode | V | Physical | V | Mode | Name     | wPi | BCM |
 +-----+-----+----------+------+Orange Pi Zero+---+------+----------+-----+-----+


```    
Thanks to zhaolei, who created the base libary for this one!


