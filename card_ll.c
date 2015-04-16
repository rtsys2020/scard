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
    scard_cold_rst(scard);
    _delay_ms(99); // wait for ATR recieving
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
    UartSW_Printf("ATR: %A\r", scard->ATR, scard->ATRLength, ' ');
#endif
    if(TS == 0x3F) {
        UartSW_Printf("Inverse needed\r");
    }
#ifdef ATR_VERBOSE
    UartSW_Printf("TS: %X\r", TS);
    UartSW_Printf("HisBLen: %X\r", K);
#endif
    // Read TA, TB, TC, TD as needed
    while(Yi != 0) { // while TD is present
        if(Yi & TAi) { // TAi is present (bit 5 is set)
            Yi &= ~TAi;       // Clear Yi
            if(i == 1) {
                scard->FindexDindex = *p++; // TA1 conveys F & D values
#ifdef ATR_VERBOSE
                UartSW_Printf("TA%u =%X\r", i, scard->FindexDindex);
#endif
            }
            if(i > 2) {
                if(scard->ProtocolType == 1) scard->IFSC = *p++;
#ifdef ATR_VERBOSE
                UartSW_Printf("TA%u =%X\r", i, scard->IFSC);
#endif
            }
        }
        if(Yi & TBi) { // TBi is present (bit 6 is set)
            Yi &= ~TBi;       // Clear Yi
            if(i > 2) {
                scard->BWI_CWI_T1 = *p++;
#ifdef ATR_VERBOSE
                UartSW_Printf("TB%u=%X\r", i, scard->BWI_CWI_T1);
#endif
            }
        }
        if(Yi & TCi) {       // TCi is present (bit 7 is set)
            Yi &= ~TCi;       // Clear Yi
            if(i == 1) scard->N = *p++;
#ifdef ATR_VERBOSE
            UartSW_Printf("TC%u=%X\r", i, scard->N);
#endif
        }
        if(Yi & TDi) {       // TDi is present (bit 8 is set)
            Yi = *p & 0xF0;
#ifdef ATR_VERBOSE
            UartSW_Printf("TD%u=%X\r", i, *p);
#endif
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

bool scard_power_on(ISO7816_SC* scard) {
    if (!getATR(scard))
        return false;
    parseATR(scard);
    //    scard_pps_req(scard); // PPS exchange
    UartSW_Printf("ATR: %A\r", scard->ATR, scard->ATRLength, ' ');
    scard->State = scs_Idle;
    return true;
}

void scard_power_off(ISO7816_SC* scard) {
    deactivation(scard);
    scard->State = scs_Off;
}

int scard_execute_cmd(ISO7816_SC* scard, const uint8_t* pInBuf, unsigned int inLength, uint8_t* pOutBuf, unsigned int pOutLength) {
    int res = -1;
    if (!scard->State != scs_Idle) return res;

    return res;
}

bool scard_init(ISO7816_SC* scard) {
    card_lld_init(scard);
    UartSW_Printf("sc init\r");
    return false; // Dummy
}
