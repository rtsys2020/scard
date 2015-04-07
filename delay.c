/*
 * delay.c
 *
 *  Created on: 06 апр. 2015 г.
 *      Author: RLeonov
 */

#include "delay.h"

volatile uint32_t ITickCounter;
uint32_t SystemClock;

void Delay_Init() {
    SysTick->RVR = 0xBB7F;     // Program RELOAD VALUE
    SysTick->CVR = 0;           // Clear the SYST_CVR
    NVIC_SetPriority (SysTick_IRQn, (1<<__NVIC_PRIO_BITS) - 1);  // set Priority for Cortex-M0 System Interrupts
    SysTick->CSR = SysTick_CTRL_CLKSOURCE_Msk |
            SysTick_CTRL_TICKINT_Msk; // Program SYST_SCR with 0x7 to enable SysTick Timer
    NVIC_EnableIRQ(SysTick_IRQn);
    SysTick->CSR |=  SysTick_CTRL_ENABLE_Msk; // Enable timer
}

bool Delay_Elapsed(uint32_t *AVar, const uint32_t ADelay) {
    if ((uint32_t)(ITickCounter - (*AVar)) >= ADelay) {
        *AVar = ITickCounter; // Reset delay
        return true;
    }
    else return false;
}

static void Loop(volatile uint32_t ACounter) { while(ACounter--); }

void Delay_ms(uint32_t Ams) {
    uint32_t __ticks = (SystemCoreClock / 10000) * Ams;
    Loop(__ticks);
}

void SysTick_Handler() {
    ITickCounter++;
}
