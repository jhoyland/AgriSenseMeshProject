/**
 * mrf24j.c, James Hoyland, 2018, james@jamesmakesthings.ca 
 *
 * Dervide from:
 * mrf24j.cpp, Karl Palsson, 2011, karlp@tweak.net.au
 * modified bsd license / apache license
 */

#include "mrf24j.h"
#include "mrfpindefs.h"
#include "tinyspi.h"

#include <avr/interrupt.h>
#include <util/delay.h>

#ifndef MRF_RESET
#error No MRF_RESET pin defined for MRF24J module
#endif

#ifndef MRF_WAKE
#error No MRF_WAKE pin defined for MRF24J module
#endif

#ifndef MRF_CS
#error No chip select pin (MRF_CS) defined for MRF24J module
#endif

// aMaxPHYPacketSize = 127, from the 802.15.4-2006 standard.
static uint8_t rx_buf[aMaxPHYPacketSize];

// essential for obtaining the data frame only
// bytes_MHR = 2 Frame control + 1 sequence number + 2 panid + 2 shortAddr Destination + 2 shortAddr Source


static int ignoreBytes = 0; // bytes to ignore, some modules behaviour.

//static bool bufPHY = false; // flag to buffer all bytes in PHY Payload, or not

volatile uint8_t flag_got_rx;
volatile uint8_t flag_got_tx;

static rx_info_t rx_info;
static tx_info_t tx_info;

static uint8_t mrf_spi_buffer[3];

#define MRF_GOT_RX 1
#define MRF_GOT_TX 2
#define MRF_BUF_PHY 4


volatile uint8_t mrf_flags;


void mrf_reset(void) {
    MRF_RESET_PORT &= ~(1 << MRF_RESET);
    _delay_ms(10);  // just my gut
    MRF_RESET_PORT |=  (1 << MRF_RESET);
    _delay_ms(20);  // from manual
}

uint8_t mrf_read_short(uint8_t address) {
    // 0 top for short addressing, 0 bottom for read

    mrf_spi_buffer[0] = (address<<1) & 0b01111110;
    mrf_spi_buffer[1] = 0;

    spi_transfer_nbytes(mrf_spi_buffer,mrf_spi_buffer,2,MRF_CS);  

    return mrf_spi_buffer[1];
}

uint8_t mrf_read_long(uint16_t address) {
    mrf_spi_buffer[0] = 0x80 | (address >> 3);
    mrf_spi_buffer[1] = address << 5;
    mrf_spi_buffer[2] = 0;

    spi_transfer_nbytes(mrf_spi_buffer,mrf_spi_buffer,3,MRF_CS);  

    return mrf_spi_buffer[2];
}


void mrf_write_short(uint8_t address, uint8_t data) {
    // 0 for top short address, 1 bottom for write
    mrf_spi_buffer[0] = ((address<<1 & 0b01111110) | 0x01);
    mrf_spi_buffer[1] = data;

    spi_transfer_nbytes(mrf_spi_buffer,mrf_spi_buffer,2,MRF_CS);  
}

void mrf_write_long(uint16_t address, uint8_t data) {
    mrf_spi_buffer[0] = 0x80 | (address >> 3);
    mrf_spi_buffer[1] = (address << 5) | 0x10;
    mrf_spi_buffer[2] = data;

    spi_transfer_nbytes(mrf_spi_buffer,mrf_spi_buffer,3,MRF_CS); 
}

uint16_t mrf_get_pan(void) {
    uint8_t panh = mrf_read_short(MRF_PANIDH);
    return panh << 8 | mrf_read_short(MRF_PANIDL);
}

void mrf_set_pan(uint16_t panid) {
    mrf_write_short(MRF_PANIDH, panid >> 8);
    mrf_write_short(MRF_PANIDL, panid & 0xff);
}

void mrf_address16_write(uint16_t address16) {
    mrf_write_short(MRF_SADRH, address16 >> 8);
    mrf_write_short(MRF_SADRL, address16 & 0xff);
}

uint16_t mrf_address16_read(void) {
    uint8_t a16h = mrf_read_short(MRF_SADRH);
    return a16h << 8 | mrf_read_short(MRF_SADRL);
}

/**
 * Simple send 16, with acks, not much of anything.. assumes src16 and local pan only.
 * @param data
 */
void mrf_send16(uint16_t dest16, uint8_t * data, uint8_t len) {
    //uint8_t len = strlen(data); // get the length of the char* array
    int i = 0;
    mrf_write_long(i++, bytes_MHR); // header length
    // +ignoreBytes is because some module seems to ignore 2 bytes after the header?!.
    // default: ignoreBytes = 0;
    mrf_write_long(i++, bytes_MHR+ignoreBytes+len);
    
    // 0 | pan compression | ack | no security | no data pending | data frame[3 bits]
    mrf_write_long(i++, 0b01100001); // first uint8_t of Frame Control
    // 16 bit source, 802.15.4 (2003), 16 bit dest,
    mrf_write_long(i++, 0b10001000); // second uint8_t of frame control
    mrf_write_long(i++, 1);  // sequence number 1

    uint16_t panid = mrf_get_pan();

    mrf_write_long(i++, panid & 0xff);  // dest panid
    mrf_write_long(i++, panid >> 8);
    mrf_write_long(i++, dest16 & 0xff);  // dest16 low
    mrf_write_long(i++, dest16 >> 8); // dest16 high

    uint16_t src16 = mrf_address16_read();
    mrf_write_long(i++, src16 & 0xff); // src16 low
    mrf_write_long(i++, src16 >> 8); // src16 high

    // All testing seems to indicate that the next two bytes are ignored.
    //2 bytes on FCS appended by TXMAC
    i+=ignoreBytes;
    int q;
    for (q = 0; q < len; q++) {
        mrf_write_long(i++, data[q]);
    }
    // ack on, and go!
    mrf_write_short(MRF_TXNCON, (1<<MRF_TXNACKREQ | 1<<MRF_TXNTRIG));
}

void mrf_set_interrupts(void) {
    // interrupts for rx and tx normal complete
    mrf_write_short(MRF_INTCON, 0b11110110);
}

/** use the 802.15.4 channel numbers..
 */
void mrf_set_channel(uint8_t channel) {
    mrf_write_long(MRF_RFCON0, (((channel - 11) << 4) | 0x03));
}

void mrf_init(void) {
    /*
    // Seems a bit ridiculous when I use reset pin anyway
    mrf_write_short(MRF_SOFTRST, 0x7); // from manual
    while (read_short(MRF_SOFTRST) & 0x7 != 0) {
        ; // wait for soft reset to finish
    }
    */
    
    spi_set_data_direction(SPI_MSB);
    
    mrf_write_short(MRF_PACON2, 0x98); // – Initialize FIFOEN = 1 and TXONTS = 0x6.
    mrf_write_short(MRF_TXSTBL, 0x95); // – Initialize RFSTBL = 0x9.

    mrf_write_long(MRF_RFCON0, 0x03); // – Initialize RFOPT = 0x03.
    mrf_write_long(MRF_RFCON1, 0x01); // – Initialize VCOOPT = 0x02.
    mrf_write_long(MRF_RFCON2, 0x80); // – Enable PLL (PLLEN = 1).
    mrf_write_long(MRF_RFCON6, 0x90); // – Initialize TXFIL = 1 and 20MRECVR = 1.
    mrf_write_long(MRF_RFCON7, 0x80); // – Initialize SLPCLKSEL = 0x2 (100 kHz Internal oscillator).
    mrf_write_long(MRF_RFCON8, 0x10); // – Initialize RFVCO = 1.
    mrf_write_long(MRF_SLPCON1, 0x21); // – Initialize CLKOUTEN = 1 and SLPCLKDIV = 0x01.

    //  Configuration for nonbeacon-enabled devices (see Section 3.8 “Beacon-Enabled and
    //  Nonbeacon-Enabled Networks”):
    mrf_write_short(MRF_BBREG2, 0x80); // Set CCA mode to ED
    mrf_write_short(MRF_CCAEDTH, 0x60); // – Set CCA ED threshold.
    mrf_write_short(MRF_BBREG6, 0x40); // – Set appended RSSI value to RXFIFO.
    mrf_set_interrupts();
    mrf_set_channel(12);
    // max power is by default.. just leave it...
    // Set transmitter power - See “REGISTER 2-62: RF CONTROL 3 REGISTER (ADDRESS: 0x203)”.
    mrf_write_short(MRF_RFCTL, 0x04); //  – Reset RF state machine.
    mrf_write_short(MRF_RFCTL, 0x00); // part 2
    
    flag_got_rx = 0;
    flag_got_tx = 0;
    
    _delay_ms(1); // delay at least 192usec
}

/**
 * Call this from within an interrupt handler connected to the MRFs output
 * interrupt pin.  It handles reading in any data from the module, and letting it
 * continue working.
 * Only the most recent data is ever kept.
 */
void mrf_interrupt_handler(void) {
    uint8_t last_interrupt = mrf_read_short(MRF_INTSTAT);
    if (last_interrupt & MRF_I_RXIF) {
        int i = 0;
        flag_got_rx++;
        // read out the packet data...
        cli();
        mrf_rx_disable();
        // read start of rxfifo for, has 2 bytes more added by FCS. frame_length = m + n + 2
        uint8_t frame_length = mrf_read_long(0x300);

        // buffer all bytes in PHY Payload
        if(mrf_flags | MRF_BUF_PHY){
            int rb_ptr = 0;
            for (i = 0; i < frame_length; i++) { // from 0x301 to (0x301 + frame_length -1)
                rx_buf[rb_ptr++] = mrf_read_long(0x301 + i);
            }
        }

        // buffer data bytes
        int rd_ptr = 0;
        // from (0x301 + bytes_MHR) to (0x301 + frame_length - bytes_nodata - 1)
        for (i = 0; i < mrf_rx_datalength(); i++) {
            rx_info.rx_data[rd_ptr++] = mrf_read_long(0x301 + bytes_MHR + i);
        }

        rx_info.frame_length = frame_length;
        // same as datasheet 0x301 + (m + n + 2) <-- frame_length
        rx_info.lqi = mrf_read_long(0x301 + frame_length);
        // same as datasheet 0x301 + (m + n + 3) <-- frame_length + 1
        rx_info.rssi = mrf_read_long(0x301 + frame_length + 1);

        mrf_rx_enable();
        sei();
    }
    if (last_interrupt & MRF_I_TXNIF) {
        flag_got_tx++;
        uint8_t tmp = mrf_read_short(MRF_TXSTAT);
        // 1 means it failed, we want 1 to mean it worked.
        tx_info.tx_ok = !(tmp & ~(1 << TXNSTAT));
        tx_info.retries = tmp >> 6;
        tx_info.channel_busy = (tmp & (1 << CCAFAIL));
    }
}


/**
 * Call this function periodically, it will invoke your nominated handlers
 */
void mrf_check_flags(void (*rx_handler)(void), void (*tx_handler)(void)){
    // TODO - we could check whether the flags are > 1 here, indicating data was lost?
    if (flag_got_rx) {
        flag_got_rx = 0;
        rx_handler();
    }
    if (flag_got_tx) {
        flag_got_tx = 0;
        tx_handler();
    }
}

/**
 * Set RX mode to promiscuous, or normal
 */
void mrf_set_promiscuous(uint8_t enabled) {
    if (enabled) {
        mrf_write_short(MRF_RXMCR, 0x01);
    } else {
        mrf_write_short(MRF_RXMCR, 0x00);
    }
}

void mrf_get_rxinfo(rx_info_t** x) {
    *x = &rx_info;
}

void mrf_get_txinfo(tx_info_t** x) {
    *x = &tx_info;
}

uint8_t * mrf_get_rxbuf(void) {
    return rx_buf;
}

int mrf_rx_datalength(void) {
    return rx_info.frame_length - bytes_nodata;
}

void mrf_set_ignoreBytes(int ib) {
    // some modules behaviour
    ignoreBytes = ib;
}

/**
 * Set bufPHY flag to buffer all bytes in PHY Payload, or not
 */
void mrf_set_bufferPHY(uint8_t bp) {
    //bufPHY = bp;

    if(bp) {
        mrf_flags |= (1<<MRF_BUF_PHY);
    }else{
        mrf_flags &= ~(1<<MRF_BUF_PHY);
    } 
}

uint8_t mrf_get_bufferPHY(void) {
    return mrf_flags | MRF_BUF_PHY;
}

/**
 * Set PA/LNA external control
 */
void mrf_set_palna(uint8_t enabled) {
    if (enabled) {
        mrf_write_long(MRF_TESTMODE, 0x07); // Enable PA/LNA on MRF24J40MB module.
    }else{
        mrf_write_long(MRF_TESTMODE, 0x00); // Disable PA/LNA on MRF24J40MB module.
    }
}

void mrf_rx_flush(void) {
    mrf_write_short(MRF_RXFLUSH, 0x01);
}

void mrf_rx_disable(void) {
    mrf_write_short(MRF_BBREG1, 0x04);  // RXDECINV - disable receiver
}

void mrf_rx_enable(void) {
    mrf_write_short(MRF_BBREG1, 0x00);  // RXDECINV - enable receiver
}

uint8_t* mrf_get_rxdata()
{
	return rx_info.rx_data;
}


uint8_t mrf_tx_ok()
{
	return tx_info.tx_ok;
}
