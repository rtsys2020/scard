/*
 * card_ll.c - Card Low Level. Here is a smart card low level driver working
 * as described in ISO7816-3 Protocol.
 * Main words: LPC11Uxx, ISO7816-3, Smartcard UART interface, USART in SmartCard mode
 *
 *  Created on: 10 апр. 2015 г.
 *      Author: RLeonov
 */

#include "card_ll.h"
#include "uart_iso.h"
#include "sw_cmd_uart.h"
#include "delay.h"

//#define HIGHSPEED

uint8_t HistoricalBytes[] = {
    0x80, 0x73, 0xC8, 0x21, 0x13, 0x66, 0x01, 0x06, 0x11, 0x59, 0x00, 0x01
};

void activation(ISO7816_SC* scard) {
    RST_LO();
    PWR_ON();
    CLK_ON();
}

void deactivation(ISO7816_SC* scard) {
    RST_LO();
    CLK_OFF();
    PWR_OFF();
    scard->State = scs_Off;
}

// stop everything then activate card again.
void scard_cold_rst(ISO7816_SC* scard) {
    deactivation(scard);
//    sleep_ms(4);
    _delay_ms(4);
    activation(scard);
//    sleep_ms(407);     // Wait more than 400 clock cycles
    _delay_ms(407);
    RST_HI();
}

// Reset by the HW reset pin
void scard_warm_rst() {
    RST_LO();
//    sleep_us(207);     // Wait more than 400 clock cycles
    _delay_ms(207);
    RST_HI();
}

static bool isValid(uint8_t *pBuf, uint8_t Length) {
    // Check TCK Here
    uint8_t TCK = 0;
    for(uint8_t i = 1; i < Length; i++)
        TCK ^= pBuf[i];
    return (TCK == 0)? true : false;
}

static bool getATR(ISO7816_SC* scard) {
    ENABLE_RX_IRQ();
    scard_cold_rst(scard);
    _delay_ms(99); // wait for ATR recieving
    DISABLE_RX_IRQ();
    if(scard->dLen == 0) {
        UartSW_Printf("sc absent\r");
        return false;
    }
    memcpy(scard->ATR, scard->dBuf, scard->dLen);
    scard->ATRLength = scard->dLen;
    return isValid(scard->ATR, scard->ATRLength);
}

static void parseATR(ISO7816_SC* scard) {
    uint8_t* p = scard->ATR;
    uint8_t TS = *p++;
    uint8_t T0 = *p++;
    uint8_t Yi = T0 & 0xF0;
    uint8_t K = T0 & 0x0F;
    uint8_t i = 1;
#ifdef ATR_CHANGE
//    UartSW_Printf("ATR: %A\r", scard->ATR, scard->ATRLength, ' ');
#endif
    if(TS == 0x3F) {
        UartSW_Printf("Inverse needed\r");
    }
    // Read TA, TB, TC, TD as needed
    while(Yi != 0) { // while TD is present
        if(Yi & TAi) { // TAi is present (bit 5 is set)
            Yi &= ~TAi;       // Clear Yi
            if(i == 1) scard->FindexDindex = *p++; // TA1 conveys F & D values
            if((i > 2) && (scard->ProtocolType == 1))
                    scard->IFSC = *p++;
        }
        if(Yi & TBi) { // TBi is present (bit 6 is set)
            Yi &= ~TBi;       // Clear Yi
            if(i > 2) scard->BWI_CWI_T1 = *p++;
        }
        if(Yi & TCi) {       // TCi is present (bit 7 is set)
            Yi &= ~TCi;       // Clear Yi
            if(i == 1) scard->N = *p++;
        }
        if(Yi & TDi) {       // TDi is present (bit 8 is set)
            Yi = *p & 0xF0;
            // Get protocol type
            if(i == 1) {
                scard->ProtocolType = *p++ & 0x0F;    // Mask bits representing protocol type
                if((scard->ProtocolType != 0) && (scard->ProtocolType != 1)) return; // Only T0 & T1 are supported here
            }
        }
        i++;
    }
#ifdef ATR_CHANGE
    memcpy(&scard->ATR[8], HistoricalBytes, sizeof(HistoricalBytes) - 1);
    scard->ATRLength = (scard->ATRLength - K) + sizeof(HistoricalBytes) - 1;
    K = sizeof(HistoricalBytes);
    scard->ATR[1] &= ~0x0F; // Clear the Historical byte length
    scard->ATR[1] |= (0x0F & K); // Write new value to it
    // count TCK
    uint8_t TCK = 0;
    for(i = 1; i < scard->ATRLength; i++) {
        TCK ^= scard->ATR[i];
    }
    scard->ATR[scard->ATRLength++] = TCK;
#endif
}

static bool pps_exchange() {
#define PPS0    0x11       // PPS1 present, protocol = 1
#ifdef HIGHSPEED
#define PPS1    0x08     // 0000{f=4; Fi=372}, 1000{Di=12}  => bitrate = 129032 bps
#else
#define PPS1    0x01       // 0000{f=4; Fi=372}, 0001{Di=1}  => bitrate = 10753 bps
#endif

    uint8_t pps_req[4];
    pps_req[0] = 0xFF;     // PPSS identifies the PPS request or response and is set to 'FF'
    pps_req[1] = PPS0;     // PPS0
    pps_req[2] = PPS1;     // PPS1
    pps_req[3] = pps_req[0] ^ \
                 pps_req[1] ^ \
                 pps_req[2]; // PCK
    if(card_lld_data_synch(pps_req, 4, 4) == 0) {
        return false;
    }
#ifdef HIGHSPEED
    card_switch_to_highspeed();
#endif
    UartSW_Printf("PPS ok\r");
    return true;
}

bool scard_power_on(ISO7816_SC* scard) {
    if (!getATR(scard)) {       // Try to get ATR
        UartSW_Printf("ATR err\r");
        return false;
    }
    parseATR(scard);            // Parse ATR ang replace the Historical bytes
    UartSW_Printf("ATR: %A\r", scard->ATR, scard->ATRLength, ' ');
    pps_exchange(scard);        // PPS exchange to default settings TODO: increase speed
    scard->State = scs_Idle;    // Switch card state
    return true;
}

void scard_power_off(ISO7816_SC* scard) {
    deactivation(scard);
    scard->State = scs_Off;
}

int scard_execute_cmd(ISO7816_SC* scard, const uint8_t* pInBuf, unsigned int inLength, uint8_t* pOutBuf, unsigned int* pOutLength) {
    int res = -1;
    if ((scard->State != scs_Idle) || (inLength > CARD_BUFFER_SIZE)) {
        return res;
    }
    scard->State = scs_Busy;
    UartSW_Printf("-->(%u) ", inLength);
    if(inLength) UartSW_Printf("%A ", pInBuf, inLength, ' ');
    UartSW_Printf("\r");
    memcpy(scard->dBuf, pInBuf, inLength); // copy data to CardBuf
    scard->dLen = inLength;
    if(card_lld_data_exchange()) {
        UartSW_Printf("<--(%u) ", scard->dLen);
        if(scard->dLen) UartSW_Printf("%A", scard->dBuf, scard->dLen, ' ');
        //        memcpy(pOutBuf, scard->dBuf, outLen);
        //        res = *pOutLength = outLen;
        UartSW_Printf("\r");
        res = scard->dLen;
        scard->State = scs_Idle;
    }
    else scard->State = scs_Error;
    return res;
}

void scard_init(ISO7816_SC* scard) {
    card_lld_init(scard);
    UartSW_Printf("sc init\r");
}
