/*
 * File:   mrf24j.h
 * copyright Karl Palsson, karlp@tweak.net.au, 2011
 * modified BSD License / apache license
 */

/*
    MRF24J interface specific commands. Ported to C from Karl's C++ code
*/

#ifndef LIB_MRF24J_H
#define LIB_MRF24J_H
#include <stdint.h>

#define aMaxPHYPacketSize 127


#define MRF_RXMCR 0x00
#define MRF_PANIDL 0x01
#define MRF_PANIDH 0x02
#define MRF_SADRL 0x03
#define MRF_SADRH 0x04
#define MRF_EADR0 0x05
#define MRF_EADR1 0x06
#define MRF_EADR2 0x07
#define MRF_EADR3 0x08
#define MRF_EADR4 0x09
#define MRF_EADR5 0x0A
#define MRF_EADR6 0x0B
#define MRF_EADR7 0x0C
#define MRF_RXFLUSH 0x0D
//#define MRF_Reserved 0x0E
//#define MRF_Reserved 0x0F
#define MRF_ORDER 0x10
#define MRF_TXMCR 0x11
#define MRF_ACKTMOUT 0x12
#define MRF_ESLOTG1 0x13
#define MRF_SYMTICKL 0x14
#define MRF_SYMTICKH 0x15
#define MRF_PACON0 0x16
#define MRF_PACON1 0x17
#define MRF_PACON2 0x18
//#define MRF_Reserved 0x19
#define MRF_TXBCON0 0x1A

// TXNCON: TRANSMIT NORMAL FIFO CONTROL REGISTER (ADDRESS: 0x1B)
#define MRF_TXNCON      0x1B
#define MRF_TXNTRIG     0
#define MRF_TXNSECEN    1
#define MRF_TXNACKREQ   2
#define MRF_INDIRECT    3
#define MRF_FPSTAT      4

#define MRF_TXG1CON 0x1C
#define MRF_TXG2CON 0x1D
#define MRF_ESLOTG23 0x1E
#define MRF_ESLOTG45 0x1F
#define MRF_ESLOTG67 0x20
#define MRF_TXPEND 0x21
#define MRF_WAKECON 0x22
#define MRF_FRMOFFSET 0x23
// TXSTAT: TX MAC STATUS REGISTER (ADDRESS: 0x24)
#define MRF_TXSTAT 0x24
#define TXNRETRY1       7
#define TXNRETRY0       6
#define CCAFAIL         5
#define TXG2FNT         4
#define TXG1FNT         3
#define TXG2STAT        2
#define TXG1STAT        1
#define TXNSTAT         0

#define MRF_TXBCON1 0x25
#define MRF_GATECLK 0x26
#define MRF_TXTIME 0x27
#define MRF_HSYMTMRL 0x28
#define MRF_HSYMTMRH 0x29
#define MRF_SOFTRST 0x2A
//#define MRF_Reserved 0x2B
#define MRF_SECCON0 0x2C
#define MRF_SECCON1 0x2D
#define MRF_TXSTBL 0x2E
//#define MRF_Reserved 0x2F
#define MRF_RXSR 0x30
#define MRF_INTSTAT 0x31
#define MRF_INTCON 0x32
#define MRF_GPIO 0x33
#define MRF_TRISGPIO 0x34
#define MRF_SLPACK 0x35
#define MRF_RFCTL 0x36
#define MRF_SECCR2 0x37
#define MRF_BBREG0 0x38
#define MRF_BBREG1 0x39
#define MRF_BBREG2 0x3A
#define MRF_BBREG3 0x3B
#define MRF_BBREG4 0x3C
//#define MRF_Reserved 0x3D
#define MRF_BBREG6 0x3E
#define MRF_CCAEDTH 0x3F

#define MRF_RFCON0 0x200
#define MRF_RFCON1 0x201
#define MRF_RFCON2 0x202
#define MRF_RFCON3 0x203
#define MRF_RFCON5 0x205
#define MRF_RFCON6 0x206
#define MRF_RFCON7 0x207
#define MRF_RFCON8 0x208
#define MRF_SLPCAL0 0x209
#define MRF_SLPCAL1 0x20A
#define MRF_SLPCAL2 0x20B
#define MRF_RSSI 0x210
#define MRF_SLPCON0 0x211
#define MRF_SLPCON1 0x220
#define MRF_WAKETIMEL 0x222
#define MRF_WAKETIMEH 0x223
#define MRF_REMCNTL 0x224
#define MRF_REMCNTH 0x225
#define MRF_MAINCNT0 0x226
#define MRF_MAINCNT1 0x227
#define MRF_MAINCNT2 0x228
#define MRF_MAINCNT3 0x229
#define MRF_TESTMODE 0x22F
#define MRF_ASSOEADR1 0x231
#define MRF_ASSOEADR2 0x232
#define MRF_ASSOEADR3 0x233
#define MRF_ASSOEADR4 0x234
#define MRF_ASSOEADR5 0x235
#define MRF_ASSOEADR6 0x236
#define MRF_ASSOEADR7 0x237
#define MRF_ASSOSADR0 0x238
#define MRF_ASSOSADR1 0x239
#define MRF_UPNONCE0 0x240
#define MRF_UPNONCE1 0x241
#define MRF_UPNONCE2 0x242
#define MRF_UPNONCE3 0x243
#define MRF_UPNONCE4 0x244
#define MRF_UPNONCE5 0x245
#define MRF_UPNONCE6 0x246
#define MRF_UPNONCE7 0x247
#define MRF_UPNONCE8 0x248
#define MRF_UPNONCE9 0x249
#define MRF_UPNONCE10 0x24A
#define MRF_UPNONCE11 0x24B
#define MRF_UPNONCE12 0x24C

#define MRF_I_RXIF  0b00001000
#define MRF_I_TXNIF 0b00000001

#define bytes_MHR 9
#define bytes_FCS 2 // FCS length = 2
//static const int bytes_nodata = bytes_MHR + bytes_FCS; // no_data bytes in PHY payload,  header length + FCS
#define bytes_nodata 11

typedef struct _mrf_hw_info_t{
    uint8_t cs;
    uint8_t reset_pin;
} hw_info_t;



typedef struct mrf_rx_info_t{
    uint8_t frame_length;
    uint8_t rx_data[116]; //max data length = (127 aMaxPHYPacketSize - 2 Frame control - 1 sequence number - 2 panid - 2 shortAddr Destination - 2 shortAddr Source - 2 FCS)
    uint8_t lqi;
    uint8_t rssi;
} rx_info_t;

/**
 * Based on the TXSTAT register, but "better"
 */
typedef struct mrf_tx_info_t{
    uint8_t tx_ok:1;
    uint8_t retries:2;
    uint8_t channel_busy:1;
} tx_info_t;



uint8_t mrf_reg_TXSTAT;

/*Interrupt handler runs in a separate thread which means other functions should not alter variables while it is running. This simple mutex method should prevent clashes */

uint8_t isr_lock;

#define GRAB_ISR_MUTEX while(isr_lock);isr_lock=1;
#define DROP_ISR_MUTEX isr_lock = 0;



/*class Mrf24j
{
    public:
        Mrf24j(int pin_reset, int pin_chip_select, int pin_interrupt);*/
        void mrf_reset(void);
        void mrf_init(void);

        uint8_t mrf_read_short(uint8_t address);
        uint8_t mrf_read_long(uint16_t address);

        void mrf_write_short(uint8_t address, uint8_t data);
        void mrf_write_long(uint16_t address, uint8_t data);

        uint16_t mrf_get_pan(void);
        void mrf_set_pan(uint16_t panid);

        void mrf_address16_write(uint16_t address16);
        uint16_t mrf_address16_read(void);

        void mrf_set_interrupts(void);

        void mrf_set_promiscuous(uint8_t enabled);

        /**
         * Set the channel, using 802.15.4 channel numbers (11..26)
         */
        void mrf_set_channel(uint8_t channel);

        void mrf_rx_enable(void);
        void mrf_rx_disable(void);

        /** If you want to throw away rx data */
        void mrf_rx_flush(void);

        void mrf_get_rxinfo(rx_info_t **);

        void mrf_get_txinfo(tx_info_t **);
        
        uint8_t * mrf_get_rxdata();
        uint8_t mrf_tx_ok();

        uint8_t * mrf_get_rxbuf(void);

        int mrf_rx_datalength(void);

        void mrf_set_ignoreBytes(int ib);

        /**
         * Set bufPHY flag to buffer all bytes in PHY Payload, or not
         */
        void mrf_set_bufferPHY(uint8_t bp);

        uint8_t mrf_get_bufferPHY(void);

        /**
         * Set PA/LNA external control
         */
        void mrf_set_palna(uint8_t enabled);

        void mrf_send16(uint16_t dest16, uint8_t * data, uint8_t len);

        void mrf_interrupt_handler(void);

        void mrf_check_flags(void (*rx_handler)(void), void (*tx_handler)(void));

   /* private:
        int _pin_reset;
        int _pin_cs;
        int _pin_int;
};*/

#endif  /* LIB_MRF24J_H */
