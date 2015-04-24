/*
 * card_ll.c - Card Low Level. Here is a smart card low level driver working
 * as described in ISO7816-3 Protocol.
 * Main words: LPC11Uxx, ISO7816-3, Smartcard UART interface, USART in SmartCard mode
 *
 *  Created on: 10 апр. 2015 г.
 *      Author: RLeonov
 */

#include "card_ll.h"
#include "sw_cmd_uart.h"

//#define HIGHSPEED

const uint8_t HistoricalBytes[] = {
    0x80, 0x73, 0xC8, 0x21,
    0x13, 0x66, 0x01, 0x06,
    0x11, 0x59, 0x00, 0x01
};

static void activation(ISO7816_SC* scard) {
    RST_LO();
    PWR_ON();
    CLK_ON();
}

static void deactivation(ISO7816_SC* scard) {
    RST_LO();
    CLK_OFF();
    PWR_OFF();
    scard->State = scs_Off;
}

// stop everything then activate card again.
static void scard_cold_rst(ISO7816_SC* scard) {
    deactivation(scard);
    _delay_ms(51);
    activation(scard);
    _delay_ms(251);     // Wait more than 400 clock cycles
    RST_HI();
}

// Reset by the HW reset pin
//static void scard_warm_rst() {
//    RST_LO();
//    sleep_us(207);     // Wait more than 400 clock cycles
//    RST_HI();
//}

static bool isValid(uint8_t *pBuf, uint8_t Length) {
    // Check TCK Here
    uint8_t TCK = 0;
    uint8_t i = 1;
    for(; i < Length; i++)
        TCK ^= pBuf[i];
    return (TCK == 0)? true : false;
}

static bool getATR(ISO7816_SC* pScard) {
    ENABLE_RX_IRQ();
    scard_cold_rst(pScard);
    _delay_ms(99); // wait for ATR recieving
    DISABLE_RX_IRQ();
    if(pScard->DEx.Len == 0)
        return false;
    memcpy(pScard->ATR, pScard->DEx.Buf, pScard->DEx.Len);
    pScard->ATRLength = pScard->DEx.Len;
    return isValid(pScard->ATR, pScard->ATRLength);
}

static void parseATR(ISO7816_SC* pScard) {
    uint8_t* p = pScard->ATR;
    uint8_t TS = *p++;
    uint8_t T0 = *p++;
    uint8_t Yi = T0 & 0xF0;
    uint8_t K = T0 & 0x0F;
    uint8_t i = 1;
#ifdef ATR_CHANGE
//    swdbg_printfI("ATR: %A\r", pScard->ATR, pScard->ATRLength, ' ');
#endif
    if(TS == 0x3F) {
//        UartSW_Printf("Inverse needed\r");
    }
    // Read TA, TB, TC, TD as needed
    while(Yi != 0) { // while TD is present
        if(Yi & TAi) { // TAi is present (bit 5 is set)
            Yi &= ~TAi;       // Clear Yi
            if(i == 1) pScard->FindexDindex = *p++; // TA1 conveys F & D values
            if((i > 2) && (pScard->ProtocolType == 1))
                pScard->IFSC = *p++;
        }
        if(Yi & TBi) { // TBi is present (bit 6 is set)
            Yi &= ~TBi;       // Clear Yi
            if(i > 2) pScard->BWI_CWI_T1 = *p++;
        }
        if(Yi & TCi) {       // TCi is present (bit 7 is set)
            Yi &= ~TCi;       // Clear Yi
            if(i == 1) pScard->N = *p++;
        }
        if(Yi & TDi) {       // TDi is present (bit 8 is set)
            Yi = *p & 0xF0;
            // Get protocol type
            if(i == 1) {
                pScard->ProtocolType = *p++ & 0x0F;    // Mask bits representing protocol type
                if((pScard->ProtocolType != 0) && (pScard->ProtocolType != 1)) return; // Only T0 & T1 are supported here
            }
        }
        i++;
    }
#ifdef ATR_CHANGE
    memcpy(&pScard->ATR[8], HistoricalBytes, sizeof(HistoricalBytes));
    pScard->ATRLength = (pScard->ATRLength - K) + sizeof(HistoricalBytes) - 1; // -1 for TCK which be placed follow up
    K = sizeof(HistoricalBytes);
    pScard->ATR[1] &= ~0x0F; // Clear the Historical byte length
    pScard->ATR[1] |= (0x0F & K); // Write new value to it
    // count TCK
    uint8_t TCK = 0;
    for(i = 1; i < pScard->ATRLength; i++) {
        TCK ^= pScard->ATR[i];
    }
    pScard->ATR[pScard->ATRLength++] = TCK;
#ifdef ATR_VERBOSE
    UartSW_Printf("ATR: %A\r", pScard->ATR, pScard->ATRLength, ' ');
#endif

#endif
}

static bool pps_exchange(ISO7816_SC* pScard) {
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
    if(card_lld_data_pps(pScard, pps_req, 4, 4) == 0) {
        return false;
    }
#ifdef HIGHSPEED
    card_switch_to_highspeed();
#endif
//    swdbg_printfI("pps ok\r");
    return true;
}

bool scard_power_on(ISO7816_SC* pScard) {
    if (!getATR(pScard))       // Try to get ATR
        return false;
    parseATR(pScard);          // Parse ATR ang replace the Historical bytes if needed
    if(!pps_exchange(pScard))  // PPS exchange to default settings if no error
        return false;
    pScard->State = scs_Idle;    // Switch card state
    return true;
}

void scard_power_off(ISO7816_SC* pScard) {
    deactivation(pScard);
    pScard->State = scs_Off;
}

int scard_execute_cmd(ISO7816_SC* pScard, const uint8_t* pInBuf, unsigned int inLength, uint8_t* pOutBuf) {
    int res = -1;
    if ((pScard->State != scs_Idle) || (inLength > CARD_BUFFER_SIZE)) {
        return res;
    }
    pScard->State = scs_Busy;
    UartSW_Printf("-->(%u): %A \r", inLength, pInBuf, inLength, ' ');
    pScard->DEx.pTxByte = (uint8_t*)pInBuf;
    pScard->DEx.Len = inLength;
//    led_set_mode(app, LED_MODE_BLINK);
    if(card_lld_data_exchange_synch(pScard) != 0) {
        UartSW_Printf("cmd err\r");
        pScard->State = scs_Error;
        return res;
    }
    else if (pScard->DEx.Len != 0) {
        res = pScard->DEx.Len;
        pScard->State = scs_Idle;
        UartSW_Printf("<--(%u): %A \r", res, pScard->DEx.Buf, res, ' ');
        if(pOutBuf != NULL) memcpy(pOutBuf, pScard->DEx.Buf, res);
    }
    else UartSW_Printf("len err\r");
    return res;
}

bool scard_init(ISO7816_SC* pScard) {
    card_lld_init(&pScard->DEx);
    return true;
}
