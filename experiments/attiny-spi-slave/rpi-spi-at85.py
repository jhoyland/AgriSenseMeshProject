#Experiment in running MCP3208 through Raspberry Pi SPI interface
#http://jeremyblythe.blogspot.com/2012/09/raspberry-pi-hardware-spi-analog-inputs.html
#https://www.raspberrypi-spy.co.uk/2013/10/analogue-sensors-on-the-raspberry-pi-using-an-mcp3008/

import spidev
import time
import math

spi = spidev.SpiDev()
spi.open(0,0)
spi.max_speed_hz = 131072  #From MCP3208 datasheet: Max speed at 5V=2MHz at 2.7V=1MHz

print spi.cshigh
print spi.lsbfirst
print spi.max_speed_hz
print spi.mode


def ReadATtiny(cmdcode):
    cnum = ord(cmdcode)
    rsp = spi.xfer2([cnum,1,1])
    print "Sent:" + cmdcode + "-" + str(cnum)  + " got:",  str(rsp[0]),  str(rsp[1]),  str(rsp[2])

#Read SPI Sensor input 0 to 7 for MCP3028



def ReadInput(Sensor):
    #We are using Fig6.1 timing sheet in which clock idles low
    #First byte contains start bit on bit 6 (MSB) followed by 1 for single ended and MSB of 3 bit channel selection
    #Second byte contains bits 1 and 0 of channel address in MSB position
    #Third byte NULL - we must send three bytes to get three back

    adc = spi.xfer2([6 | ((Sensor&4) >> 2),(Sensor&3)<<6,0])

    #Three returned bytes.
    #First byte ignored
    #Second byte only four least significant bits needed (adc[1]&15 << 8)
    #Third byte remainder of data

    data = ((adc[1]&15) << 8) + adc[2]
    return data


for i in range(0,3):
    ReadATtiny('A')
    time.sleep(2)
    ReadATtiny('B')
    time.sleep(2)
    ReadATtiny('C')
    time.sleep(2)
 
