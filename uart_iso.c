/*
 * uart_iso.c
 *
 *  Created on: 16 апр. 2015 г.
 *      Author: RLeonov
 */

#include "uart_iso.h"
#include "sw_cmd_uart.h"
#include "delay.h"

ISO7816_SC* pScard;
cmd_TxState_t TxState;
cmd_RxState_t RxState;
uint8_t* pTxByte;
uint8_t TxCnt;

/* Initially, UART speed is 10752.69 bit/s (f=4MHz, F=372, D=1)
 * "The etu initially used by the card shall be equal to 372 clock cycles (i.e., during the answer to reset, the values
 * of the transmission parameters are the default values Fd = 372 and Dd = 1)." Page 15.
 * 372 clock/symbol makes bitrate about 10753.  */
#define DEFAULT_BAUDRATE    10753
#define HIGHSPEED_BAUDRATE  129036
#define SET_BAUD(A)         regVal = LPC_SYSCON->UARTCLKDIV; \
Fdiv = (((38000000/LPC_SYSCON->SYSAHBCLKDIV)/regVal)/16)/A ; \
CARD_UART->DLM = Fdiv / 256; \
CARD_UART->DLL = Fdiv % 256;

// Init GPIO for Power and Reset to the card HW
static void card_gpio_init() {
//    gpio_enable_pin(CARD_PWR_PIN, GPIO_MODE_OUT);
    PinSetupOut(1, 22); // PIO0_22
    PWR_OFF();
//    gpio_enable_pin(CARD_RST_PIN, GPIO_MODE_OUT);
    PinSetupOut(1, 24);
    RST_LO();
}

// Enable Timer to provide 4 MHz clock source to the card
static void card_clock_init() {
    // To get the 3 MHz clock freq use PR = 3, MR3 = 3, MR1 = 2;
    // To get the 4 MHz clock freq use PR = 2, MR3 = 3, MR1 = 2;
    // To get the ? MHz clock freq use PR = 2, MR3 = 2, MR1 = 1;
    LPC_SYSCON->SYSAHBCLKCTRL |= (1 << 8);  // Enable the CT16B1 timer peripherals
    CARD_CLK_TMR->TCR = 2;                  // Reset the timer counter and prepare to change settings
    CARD_CLK_TMR->PR = 2;                   // PreScaler value, at 48MHz, a PR value of 3 (PR + 1)! gives us a 24MHz clock.
    CARD_CLK_TMR->PWMC = 0x0002;            // Enable PWM mode for Match 1 output.
    CARD_CLK_TMR->MCR = (1 << 10);          // Reset on MR3.
    CARD_CLK_TMR->MR3 = 2;                  // Set the period, the timer will be reset to zero once it reaches this value from 0 to MR3
    CARD_CLK_TMR->MR1 = 1;                  // Match Register 1 set to 2 counts, giving us 50% duty // Here output 4 MHz
    CARD_CLK_IO_CON &= ~0x07;               // Clear the pin
    CARD_CLK_IO_CON = (2 | (1 << 3));       // Port 0 pin 22
    CARD_CLK_TMR->TCR = 0;                  // Take timer out of reset
    // Then the timer is started by executing macro CLK_ON() and swithced off bye the CLK_OFF() macros
}

static void card_data_init() {
    uint32_t Fdiv, regVal;
    NVIC_DisableIRQ(UART_IRQn);
    IO_RESET();
    IO_ENABLE();
    LPC_SYSCON->SYSAHBCLKCTRL |= (1<<12);   // Enable Uart Clock
    LPC_SYSCON->UARTCLKDIV = 1;             // not divided
    CARD_UART->LCR = 0x9B;                  // 8 bits, Even Parity, Parity Enable, 1 Stop bit
    SET_BAUD(DEFAULT_BAUDRATE);             // Setup baudrate
    CARD_UART->DLL = (uint8_t)(Fdiv & 0xFF);// setup divider
    CARD_UART->DLM = (uint8_t)(Fdiv >> 8);  // setup divider
    CARD_UART->LCR &= ~0x80;                // DLAB = 0
    CARD_UART->FCR = 0x07;                  // Enable and reset TX and RX FIFO
    CARD_UART->SCICTRL = 0x01;              // Enable Smartcard interface
    while (CARD_UART->LSR & LSR_RDR) {
      regVal = CARD_UART->RBR;              // Dump data from RX FIFO
    }
    DISABLE_RX_IRQ();
    DISABLE_TX_IRQ();
    CARD_UART->IER = IER_RBR;               // Enable UART interrupt
    NVIC_EnableIRQ(UART_IRQn);              // Enable IRQ
}

void card_switch_to_highspeed() {
    uint32_t Fdiv, regVal;
    CARD_UART->LCR |= 0x80;                 // DLAB = 1
    SET_BAUD(HIGHSPEED_BAUDRATE);           // Setup highspeed baudrate
    CARD_UART->DLL = (uint8_t)(Fdiv & 0xFF);// setup divider
    CARD_UART->DLM = (uint8_t)(Fdiv >> 8);  // setup divider
    CARD_UART->LCR &= ~0x80;                // DLAB = 0
}

void start_tx() {
    pScard->LRC ^= pScard->NAD;             // Count LRC checksum
    TxState = cmd_TxPCB;                    // Set state to transmit PCB
    ENABLE_TX_IRQ();                        // disable in the IRQ
    CARD_UART->THR = pScard->NAD;           // Transmit NAD
}

// Use for PPS exchange and so on
uint32_t card_lld_data_synch(uint8_t* pData, uint8_t inLen, uint8_t OutLength) {
    uint8_t retVal = 0;                     // return value
    pTxByte = pData;
    TxCnt = inLen;
    while (TxCnt != 0) {                    // while data present send it
          CARD_UART->THR = *pTxByte++;      // put byte to the THR
          while ( !(CARD_UART->LSR & LSR_THRE) );   // wait til tx done
          TxCnt--;                          // decrease count
    }
    pScard->dLen = 0;                       // dLen now is all transmitted
    ENABLE_RX_IRQ();                        // enable RX
    uint16_t Timeout = 999;                 // set the timeout to wait ack
    while(Timeout-- > 0) {
        if (OutLength == pScard->dLen) {
            retVal = pScard->dLen;          // return the outer length
            break;
        } // ok
        _delay_ms(9);
    } // while
    DISABLE_RX_IRQ();
    return retVal; // timeout
}

// Transmit IBlock
void TxIBlock(uint8_t *pBuf, uint8_t Len) {
    pScard->NAD = 0;
    pScard->PCB ^= I_NS_BIT;
    pScard->LEN = Len;
    pScard->LRC = 0;
    pTxByte = pBuf;
    TxCnt = Len;
    start_tx();
}

bool card_lld_data_exchange() {
    TxIBlock(pScard->dBuf, pScard->dLen);   // transmit IBlock
    while(TxState != cmd_TxOff);            // wait Tx complete in IRQ
    RxState = cmd_RxNAD;                    // change state to receive NAD byte
    ENABLE_RX_IRQ();                        // Next disable in IRQ
    while(RxState != cmd_RxOff);            // wait Rx complete in IRQ
    if(pScard->LRC) {                       // Check LRC, if not zero - error
        UartSW_Printf("LRC Err\r");
        return false;
    }
    return true;    // No error
}

void card_lld_init(ISO7816_SC* scard) {
    card_gpio_init();
    card_clock_init();
    card_data_init();
    pScard = scard;
    pScard->dLen = 0;
    pScard->TSreceived = false;
    TxState = cmd_TxOff;
    RxState = cmd_RxOff;
}

#if 1 // ========================== IRQ Handling ===============================
static inline void rx_on_irq() {
    uint8_t RxByte;
    RxByte = CARD_UART->RBR;
    switch (RxState) {
        case cmd_RxNAD:
            pScard->LRC = RxByte;
            pScard->NAD = RxByte;
            RxState = cmd_RxPCB;
            break;

        case cmd_RxPCB:
            pScard->LRC ^= RxByte;
            pScard->PCB = RxByte;
            RxState = cmd_RxLEN;
            break;
        case cmd_RxLEN:
            pScard->LRC ^= RxByte;
            pScard->LEN = RxByte;
            pScard->dLen = 0;
            if(pScard->LEN == 0) RxState = cmd_RxLRC;
            else RxState = cmd_RxINFO;
            break;
        case cmd_RxINFO:
            pScard->LRC ^= RxByte;
            pScard->dBuf[pScard->dLen++]= RxByte;
            pScard->LEN--; // decrease LEN
            if(pScard->LEN == 0) { // all data rx complete
                RxState = cmd_RxLRC;
            }
            break;
        case cmd_RxLRC:
            pScard->LRC ^= RxByte;
            RxState = cmd_RxOff;
            DISABLE_RX_IRQ();
            break;

        case cmd_RxOff:
            if(pScard->TSreceived) pScard->dBuf[pScard->dLen++] = RxByte;
            else {
                if((RxByte == 0x3B) || (RxByte == 0x3F)) { // Check TS
                    pScard->dBuf[pScard->dLen++] = RxByte;
                    pScard->TSreceived = !pScard->TSreceived;
                    return;
                } // TS correct
            } // // Receive data
        break;
    } // switch RxState
}

static inline void tx_on_irq() {
    switch (TxState) {
        case cmd_TxPCB:
            pScard->LRC ^= pScard->PCB;
            TxState = cmd_TxLEN;
            CARD_UART->THR = pScard->PCB;
            break;

        case cmd_TxLEN:
            pScard->LRC ^= pScard->LEN;
            TxState = cmd_TxINFO;
            CARD_UART->THR = pScard->LEN;
            break;

        case cmd_TxINFO:
            pScard->LRC ^= *pTxByte;
            CARD_UART->THR = *pTxByte++;
            TxCnt--;
            if(TxCnt == 0)
                TxState = cmd_TxLRC;
            break;

        case cmd_TxLRC:
            TxState = cmd_TxIdle;
            CARD_UART->THR = pScard->LRC;
            break;

        case cmd_TxIdle:
            TxState = cmd_TxOff;
            DISABLE_TX_IRQ();
            break;

        case cmd_TxOff:
        default:
            break;
    }
}

// IRQ wrapper
void UART_IRQHandler() {
    uint8_t IIRValue;
    uint8_t LSRValue;
    IIRValue = CARD_UART->IIR;
    IIRValue >>= 1;           /* skip pending bit in IIR */
    IIRValue &= 0x07;         /* check bit 1~3, interrupt identification */
    if (IIRValue == IIR_RDA) rx_on_irq();
    else if (IIRValue == IIR_THRE) {
        LSRValue = CARD_UART->LSR;      // Check status in the LSR to see if
        if (LSRValue & LSR_THRE) tx_on_irq();
    } // Tx Irq
    return;
}

#endif
