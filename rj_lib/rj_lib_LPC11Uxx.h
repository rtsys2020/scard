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

static inline void PinSetupOut(const uint8_t PGpioPort, const uint16_t APinNumber) {
    LPC_GPIO->DIR[PGpioPort] |= (uint32_t)(1<<APinNumber);
}

static inline void PinSet(uint8_t PGpioPort, uint32_t APinNumber) {
    LPC_GPIO->SET[PGpioPort] |= (uint32_t)(1 << APinNumber);
}

static inline void PinClear(uint8_t PGpioPort, uint8_t APinNumber) {
    LPC_GPIO->CLR[PGpioPort] |= (uint32_t)(1 << APinNumber);
}



#endif /* RJ_LIB_RJ_LIB_LPC11UXX_H_ */
