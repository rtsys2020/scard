/*
 * rj_lib_LPC11Uxx.h
 *
 *  Created on: 06 апр. 2015 г.
 *      Author: RLeonov
 */

#ifndef RJ_LIB_RJ_LIB_LPC11UXX_H_
#define RJ_LIB_RJ_LIB_LPC11UXX_H_

#include "LPC11Uxx.h"


#define bool  _Bool
#define true    1
#define false   0

static inline void PinSet(uint8_t portNum, uint32_t pinNum) {
    LPC_GPIO->SET[portNum] |= (1 << portNum);
}

static inline void PinClear(uint8_t portNum, uint8_t pinNum) {
    LPC_GPIO->CLR[portNum] |= (1 << portNum);
}

#endif /* RJ_LIB_RJ_LIB_LPC11UXX_H_ */
