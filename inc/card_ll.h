/*
 * card_ll.h
 *
 *  Created on: 10 апр. 2015 г.
 *      Author: RLeonov
 */

#ifndef CARD_LL_H_
#define CARD_LL_H_

#include <string.h>
#include "LPC11Uxx.h"
#include "rj_lib_LPC11Uxx.h"

//#define ATR_VERBOSE
#define ATR_CHANGE

#define TAi     0x10
#define TBi     0x20
#define TCi     0x40
#define TDi     0x80

#define MAX_ATR_SIZE        32

typedef enum {
    scs_Error, scs_Off, scs_Idle, scs_Busy
} CardState_t;

typedef struct {
    // internal usage
    uint8_t FindexDindex;
    uint8_t BWI_CWI_T1;
    uint8_t N;
    uint8_t ProtocolType;
    uint8_t IFSC;
    // external usage
    uint8_t ATR[MAX_ATR_SIZE];
    uint8_t ATRLength;
    uint8_t dBuf[256];
    uint32_t dLen;
    CardState_t State;
    bool TSreceived;
} ISO7816_SC;

bool scard_init(ISO7816_SC* scard);
bool scard_power_on(ISO7816_SC* scard);
void scard_power_off(ISO7816_SC* scard);
int scard_execute_cmd(ISO7816_SC* scard, const uint8_t* pInBuf, unsigned int inLength, uint8_t* pOutBuf, unsigned int pOutLength);
//void scard_pps_req(ISO7816_SC* scard, const uint8_t* pInBuf, uint32_t inLength);

#endif /* CARD_LL_H_ */
