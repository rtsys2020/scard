/*
 * uart_iso.c
 *
 *  Created on: 16 апр. 2015 г.
 *      Author: RLeonov
 */

#include "sw_cmd_uart.h"
#include "uart_iso.h"
#include "delay.h"

DEx_t * pDExInt;

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

#if 1 // ========================= Low Level Init ==============================
// Init GPIO for Power and Reset to the card HW
static void card_gpio_init() {
//    gpio_enable_pin(CARD_PWR_PIN, GPIO_MODE_OUT);
    PinSetupOut(1, 22);
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

static void card_data_init(DEx_t* pDEx) {
    uint32_t Fdiv, regVal;
    NVIC_DisableIRQ(UART_IRQ_Channel);
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
//    irq_register(UART_IRQ_Channel, uart_irq_handler, (void*)pDEx); // register interrupt function
//    NVIC_SetPriority(UART_IRQ_Channel, 1);  // Set priority
    NVIC_EnableIRQ(UART_IRQ_Channel);       // Enable IRQ
}
#endif

#if 1 // ========================= Interface Relative ==========================
// Use for PPS exchange and so on wait
uint32_t card_lld_data_pps(ISO7816_SC* pScard, uint8_t* pData, uint8_t inLen, uint8_t OutLength) {
    DEx_t * pDex = &pScard->DEx;
    uint8_t retVal = 0;                             // return value
    uint8_t i = 0;
    for(; i < inLen; i++) {
        _delay_ms(2);                              // TODO: if there is not delay the bytes send wrong
        CARD_UART->THR = pData[i];                // put byte to the THR
        while (!(CARD_UART->LSR & LSR_THRE));     // wait til tx done
    }
    pDex->Len = 0;                                  // Len now is all transmitted
    ENABLE_RX_IRQ();                                // enable RX
    uint16_t Timeout = 999;                         // set the timeout to wait ack
    while(Timeout-- > 0) {
        if (OutLength == pDex->Len) {
            retVal = pDex->Len;                     // return the outer length
            break;
        } // ok
        _delay_ms(9);
    } // while
    DISABLE_RX_IRQ();
    return retVal; // timeout
}

void card_lld_data_exchange_asynch(ISO7816_SC* pScard) {
    // Prepare
    DEx_t * pDex = &pScard->DEx;
    pDex->PCB = pDex->TxPCB;              // prepare preamble PCB
    pDex->LEN = pDex->Len;              // len to tx
    pDex->TxCnt = pDex->Len;            // send data length
    // Start transmittion
    pDex->LRC = pDex->NAD;              // Init LRC checksum
    pDex->TxState = cmd_TxPCB;          // Set state to transmit PCB
    CARD_UART->THR = pDex->NAD;         // Transmit NAD
    ENABLE_TX_IRQ();                    // disable in the IRQ
}

uint8_t card_lld_data_exchange_synch(ISO7816_SC* pScard) {
    card_lld_data_exchange_asynch(pScard);
    DEx_t * pDex = &pScard->DEx;
    uint16_t Timeout = 30000;  // TODO: define timeout
    while (Timeout-- != 0) {
        if(pDex->RxState == cmd_RxFinal) {
            pDex->RxState = cmd_RxOff;
            if(pDex->LRC == 0) {
                if(pDex->PCB != pDex->TxPCB) {
                    UartSW_Printf("\rWRN!\r");
                }
                pDex->TxPCB ^= I_NS_BIT;
                return 0;
            }
            return 2; // error
        }
        _delay_ms(2);
    }
    return 1; // timeout
}
#endif

#if 1 // ============================ Init =====================================
void card_lld_init(DEx_t* pDex) {
    pDExInt = pDex;
    card_gpio_init();
    card_clock_init();
    card_data_init(pDex);
    // Flush struct
    pDex->NAD = 0;
    pDex->PCB = 0;
    pDex->TxPCB = 0;
    pDex->LEN = 0;
    pDex->LRC = 0;
    pDex->Len = 0;
    pDex->TxState = cmd_TxOff;
    pDex->RxState = cmd_RxOff;
    pDex->pTxByte = pDex->Buf;
    pDex->TxCnt = 0;
    pDex->TSreceived = false;
}
#endif

void card_switch_to_highspeed() {
    uint32_t Fdiv, regVal;
    CARD_UART->LCR |= 0x80;                 // DLAB = 1
    SET_BAUD(HIGHSPEED_BAUDRATE);           // Setup highspeed baudrate
    CARD_UART->DLL = (uint8_t)(Fdiv & 0xFF);// setup divider
    CARD_UART->DLM = (uint8_t)(Fdiv >> 8);  // setup divider
    CARD_UART->LCR &= ~0x80;                // DLAB = 0
}

#if 1 // ========================== IRQ Handling ===============================
static inline void rx_on_irq(DEx_t* pDex) {
    uint8_t RxByte;
    RxByte = CARD_UART->RBR;
    switch (pDex->RxState) {
        case cmd_RxNAD:
            pDex->LRC = RxByte;
            pDex->NAD = RxByte;
            pDex->RxState = cmd_RxPCB;
            break;

        case cmd_RxPCB:
            pDex->LRC ^= RxByte;
            pDex->PCB = RxByte;
            pDex->RxState = cmd_RxLEN;
            break;
        case cmd_RxLEN:
            pDex->LRC ^= RxByte;
            pDex->LEN = RxByte;
            pDex->Len = 0;
            if(pDex->LEN == 0) pDex->RxState = cmd_RxLRC;
            else pDex->RxState = cmd_RxINFO;
            break;
        case cmd_RxINFO:
            pDex->LRC ^= RxByte;
            pDex->Buf[pDex->Len++]= RxByte;
            pDex->LEN--; // decrease LEN
            if(pDex->LEN == 0) { // all data rx complete
                pDex->RxState = cmd_RxLRC;
            }
            break;
        case cmd_RxLRC:
            pDex->LRC ^= RxByte;
            pDex->RxState = cmd_RxFinal;
            DISABLE_RX_IRQ();
            break;

        case cmd_RxOff:
            if(pDex->TSreceived) pDex->Buf[pDex->Len++] = RxByte;
            else {
                if((RxByte == 0x3B) || (RxByte == 0x3F)) { // Check TS
                    pDex->Buf[pDex->Len++] = RxByte;
                    pDex->TSreceived = !pDex->TSreceived;
                    return;
                } // TS correct
            } // // Receive data
            break;

        case cmd_RxFinal:
        default:

        break;
    } // switch RxState
}

static inline void tx_on_irq(DEx_t* pDex) {
    switch (pDex->TxState) {
        case cmd_TxPCB:
            pDex->LRC ^= pDex->PCB;
            pDex->TxState = cmd_TxLEN;
            CARD_UART->THR = pDex->PCB;
        break;

        case cmd_TxLEN:
            pDex->LRC ^= pDex->LEN;
            pDex->TxState = cmd_TxINFO;
            CARD_UART->THR = pDex->LEN;
        break;

        case cmd_TxINFO:
            pDex->LRC ^= *pDex->pTxByte;
            CARD_UART->THR = *pDex->pTxByte;
            pDex->TxCnt--;
            if(pDex->TxCnt == 0)
                pDex->TxState = cmd_TxLRC;
            else pDex->pTxByte++;
        break;

        case cmd_TxLRC:
            pDex->TxState = cmd_TxIdle;
            CARD_UART->THR = pDex->LRC;
        break;

        case cmd_TxIdle:
            pDex->TxState = cmd_TxOff;
            DISABLE_TX_IRQ();
            pDex->RxState = cmd_RxNAD;
            ENABLE_RX_IRQ();
        break;

        case cmd_TxOff:
        default:
            break;
    }
    return;
}

// IRQ wrapper
void UART_IRQHandler() {
    uint8_t IIRValue;
    uint8_t LSRValue;
    IIRValue = CARD_UART->IIR;
    IIRValue >>= 1;           /* skip pending bit in IIR */
    IIRValue &= 0x07;         /* check bit 1~3, interrupt identification */
    if (IIRValue == IIR_RDA) rx_on_irq(pDExInt);
    else if (IIRValue == IIR_THRE) {
        LSRValue = CARD_UART->LSR;      // Check status in the LSR to see if
        if (LSRValue & LSR_THRE) tx_on_irq(pDExInt);
    } // Tx Irq
    return;
}
#endif
