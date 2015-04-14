/*
 * card_ll.c - Card Low Level
 *
 *  Created on: 10 апр. 2015 г.
 *      Author: RLeonov
 */

#include "card_ll.h"
#include "sw_cmd_uart.h"
#include "delay.h"

#define BLUE_LED    PIO0_23

uint8_t defaultATR[] = {
    0x3B, 0xDC, 0x18, 0xFF, 0x81, 0x91, 0xFE, 0x1F,
    0xC3, 0x80, 0x73, 0xC8, 0x21, 0x13, 0x66, 0x01,
    0x06, 0x11, 0x59, 0x00, 0x01, 0x28
};

/* Initially, UART speed is 10752.69 bit/s (f=4MHz, F=372, D=1)
 * "The etu initially used by the card shall be equal to 372 clock cycles (i.e., during the answer to reset, the values
 * of the transmission parameters are the default values Fd = 372 and Dd = 1)." Page 15.
 * 372 clock/symbol makes bitrate about 10753.  */
#define DEFAULT_BAUDRATE 2400
//#define DEFAULT_BAUDRATE    10753
#define SET_BAUD(A)         regVal = LPC_SYSCON->UARTCLKDIV; \
Fdiv = (((48000000/LPC_SYSCON->SYSAHBCLKDIV)/regVal)/16)/A ; \
CARD_UART->DLM = Fdiv / 256; \
CARD_UART->DLL = Fdiv % 256;

void scard_activation(ISO7816_SC* scard) {
    RST_LO();
    PWR_ON();
    CLK_ON();
}

void scard_deactivation(ISO7816_SC* scard) {
    RST_LO();
    CLK_OFF();
    PWR_OFF();
    scard->State = scs_Off;
}

// stop everything then activate card again.
void scard_ColdRst(ISO7816_SC* scard) {
    scard_deactivation(scard);
//    sleep_ms(4);
    _delay_ms(4);
    scard_activation(scard);
//    sleep_ms(407);     // Wait more than 400 clock cycles
    _delay_ms(407);
    RST_HI();
}

// Reset by the HW reset pin
void scard_WarmRst() {
    RST_LO();
//    sleep_us(207);     // Wait more than 400 clock cycles
    _delay_ms(207);
    RST_HI();
}

// Init pin as reset and power
void scard_gpio_init() {
//    gpio_enable_pin(CARD_PWR_PIN, GPIO_MODE_OUT);
    PinSetupOut(1, 22); // PIO0_22
    PWR_OFF();
//    gpio_enable_pin(CARD_RST_PIN, GPIO_MODE_OUT);
    PinSetupOut(1, 24);
    RST_LO();
}

void Uart_IrqHandler() {
//    uint8_t IIRValue, LSRValue;
//    uint8_t Dummy = Dummy;
//    IIRValue = LPC_USART->IIR;
//    UartSW_Printf("i\r");
}


// Enable Timer to provide 4 MHz clock source to the card
void scard_clock_init() {
    // To get the 3 MHz clock freq use PR = 3, MR3 = 3, MR1 = 2;
    // To get the 4 MHz clock freq use PR = 2, MR3 = 3, MR1 = 2;
    LPC_SYSCON->SYSAHBCLKCTRL |= (1 << 8); // Enable the CT16B1 timer peripherals
    CARD_CLK_TMR->TCR = 2; // Reset the timer counter and prepare to change settings
    CARD_CLK_TMR->PR = 2; // PreScaler value, at 48MHz, a PR value of 3 (PR + 1)! gives us a 12MHz clock.
    CARD_CLK_TMR->PWMC = 0x0002;    // Enable PWM mode for Match 1 output.
    CARD_CLK_TMR->MCR = (1 << 10); // Reset on MR3.
    CARD_CLK_TMR->MR3 = 3;     // Set the period, the timer will be reset to zero once it reaches this value from 0 to MR3
    CARD_CLK_TMR->MR1 = 2;     // Match Register 1 set to 2 counts, giving us 50% duty // Here output 4 MHz
    CARD_CLK_IO_CON &= ~0x07;
    CARD_CLK_IO_CON = (2 | (1 << 3)); // Port 0 pin 22
    CARD_CLK_TMR->TCR = 0;     // Take timer out of reset
    // Then the timer is started by executing macro CLK_ON() and swithced off bye the CLK_OFF() macro
}

void scard_dataIO_init() {
    uint32_t Fdiv;
    uint32_t regVal;

    NVIC_DisableIRQ(UART_IRQn);
    LPC_SYSCON->SYSAHBCLKCTRL &= ~(1<<12);   // Disable Uart Clock
//    CARD_UART_IO_CON &= ~0x07;  // Clean
//    CARD_UART_IO_CON |= 0x01;   // Port 0 Pin 19 is UART TXD

//    LPC_IOCON->PIO0_18 &= ~0x07;  // Clean
//    LPC_IOCON->PIO0_18 |= 0x01;   // Port 0 Pin 19 is UART RXD

    CARD_UART->HDEN = 0; // Disable HDEN
    CARD_UART->SYNCCTRL = 0; // Disable SyncCTRL
    LPC_SYSCON->UARTCLKDIV = 1;  // No divide UART_PCLK = 12MHz
    SET_BAUD(DEFAULT_BAUDRATE);  // here need to setup baudrate
//    CARD_UART->OSR = (uint32_t)(371 << 4); // Oversampling by 371 by the card is should to overample UART by the 372 clocking, but here it is not needed

    CARD_UART->LCR = 0x03; // 8 bit
    CARD_UART->LCR |= (1 << 2); // 1 stop bits
    CARD_UART->FCR = 0x07; // Reset FIFO for UART
    CARD_UART->LCR |= (0x01 << 4); // Even Parity
    CARD_UART->LCR |= (1 << 3); // Parity Enable

//    CARD_UART->SCICTRL = 0x01; // Enable SmartCard interface
    LPC_SYSCON->SYSAHBCLKCTRL |= (1<<12);   // Enable Uart Clock
    CARD_UART->IER = IER_RBR; // Enable Rx Irq
//    NVIC_EnableIRQ(UART_IRQn);
 }

bool scard_cmd_getATR(ISO7816_SC* scard) {
    // this is an jam
    memcpy(scard->ATR, defaultATR, sizeof(defaultATR));
//    printf();
    return true;
}

bool scard_power_on(ISO7816_SC* scard) {
//    scard_pps_req(scard); // PPS exchange
    if (!scard_cmd_getATR(scard))
        return false;
    scard->State = scs_Idle;
    return true;
}

void scard_power_off(ISO7816_SC* scard) {
    scard_deactivation(scard);
    scard->State = scs_Off;
}

int scard_execute_cmd(ISO7816_SC* scard, const uint8_t* pInBuf, unsigned int inLength, uint8_t* pOutBuf, unsigned int pOutLength) {
    int i;
    int res = -1;
    if (!scard->State != scs_Idle)
        return res;
    for (i = 0; i < 3; ++i) {// TODO: here must be defined NRetry
        // Try to ping card
        // Dummy
    }
    if (res < 0) {
        scard_power_on(scard);
    }
    return res;
}


bool scard_init(ISO7816_SC* scard) {
    scard_gpio_init();
//    scard_dataIO_init();
    scard_clock_init();
    // ColdReset
    scard_ColdRst(scard);
    UartSW_Printf("sc init\r");
    return false; // Dummy
}
