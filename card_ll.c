/*
 * card_ll.c - Card Low Level
 *
 *  Created on: 10 ���. 2015 �.
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

ISO7816_SC* pScard;

/* Initially, UART speed is 10752.69 bit/s (f=4MHz, F=372, D=1)
 * "The etu initially used by the card shall be equal to 372 clock cycles (i.e., during the answer to reset, the values
 * of the transmission parameters are the default values Fd = 372 and Dd = 1)." Page 15.
 * 372 clock/symbol makes bitrate about 10753.  */
#define DEFAULT_BAUDRATE    10753
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
    UartSW_Printf("Cld Rst\r");
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

void UART_IRQHandler() {
//    uint8_t IIRValue, LSRValue;
//    uint8_t Dummy = Dummy;
//    IIRValue = LPC_USART->IIR;
//    IIRValue >>= 1;           /* skip pending bit in IIR */
//    IIRValue &= 0x07;         /* check bit 1~3, interrupt identification */
//
//    if (IIRValue == IIR_RLS) {
//        LSRValue = LPC_USART->LSR;
//        if (LSRValue & (LSR_OE | LSR_PE | LSR_FE | LSR_RXFE | LSR_BI)) {
//            pScard->dBuf[pScard->dLen++] = LPC_USART->RBR;
//            return;
//        }
//        if (LSRValue & LSR_RDR) {
//            pScard->dBuf[pScard->dLen++] = LPC_USART->RBR;
//        }
//    }
//    else if (IIRValue == IIR_RDA) {
//        pScard->dBuf[pScard->dLen++] = LPC_USART->RBR;
//    }
    uint8_t IIRValue, LSRValue;
    uint8_t Dummy = Dummy;
    IIRValue = LPC_USART->IIR;
    IIRValue >>= 1;           /* skip pending bit in IIR */
    IIRValue &= 0x07;         /* check bit 1~3, interrupt identification */
    if (IIRValue == IIR_RLS) {
      LSRValue = LPC_USART->LSR;
      /* Receive Line Status */
      if (LSRValue & (LSR_OE | LSR_PE | LSR_FE | LSR_RXFE | LSR_BI))
      {
        /* There are errors or break interrupt */
        /* Read LSR will clear the interrupt */
        Dummy = LPC_USART->RBR;   /* Dummy read on RX to clear
                                  interrupt, then bail out */
        return;
      }
      if (LSRValue & LSR_RDR) /* Receive Data Ready */
      {
        pScard->dBuf[pScard->dLen++] = LPC_USART->RBR;
      }
    }
    else if (IIRValue == IIR_RDA) /* Receive Data Available */
    {
        pScard->dBuf[pScard->dLen++] = LPC_USART->RBR;
    }
    return;
}

//static inline uint8_t read_byte(uint8_t *AByte) {
//   for(uint16_t i=0; i<459; i++) {
//       uint8_t IIRValue = LPC_USART->IIR;
//       IIRValue >>= 1;
////       UartSW_Printf("%X\r", LSRValue);
//        if (IIRValue & IIR_RDA) {
//            *AByte = LPC_USART->RBR;
//            return 0;
//        }
//        _delay_us(100);
//    }
//    return 1;
//}

// Enable Timer to provide 4 MHz clock source to the card
void scard_clock_init() {
    // To get the 3 MHz clock freq use PR = 3, MR3 = 3, MR1 = 2;
    // To get the 4 MHz clock freq use PR = 2, MR3 = 3, MR1 = 2;
    LPC_SYSCON->SYSAHBCLKCTRL |= (1 << 8); // Enable the CT16B1 timer peripherals
    CARD_CLK_TMR->TCR = 2; // Reset the timer counter and prepare to change settings
    CARD_CLK_TMR->PR = 2; // PreScaler value, at 48MHz, a PR value of 3 (PR + 1)! gives us a 24MHz clock.
    CARD_CLK_TMR->PWMC = 0x0002;    // Enable PWM mode for Match 1 output.
    CARD_CLK_TMR->MCR = (1 << 10); // Reset on MR3.
    CARD_CLK_TMR->MR3 = 2;     // Set the period, the timer will be reset to zero once it reaches this value from 0 to MR3
    CARD_CLK_TMR->MR1 = 1;     // Match Register 1 set to 2 counts, giving us 50% duty // Here output 4 MHz
    CARD_CLK_IO_CON &= ~0x07;
    CARD_CLK_IO_CON = (2 | (1 << 3)); // Port 0 pin 22
    CARD_CLK_TMR->TCR = 0;     // Take timer out of reset
    // Then the timer is started by executing macro CLK_ON() and swithced off bye the CLK_OFF() macro
}

void scard_dataIO_init() {
    uint32_t Fdiv;
    uint32_t regVal;
    NVIC_DisableIRQ(UART_IRQn);
    LPC_IOCON->PIO0_18 &= ~0x07;    /*  UART I/O config */
    LPC_IOCON->PIO0_19 &= ~0x07;
    TX_ON();
//    RX_ON();
    LPC_SYSCON->SYSAHBCLKCTRL |= (1<<12);
    LPC_SYSCON->UARTCLKDIV = 2;     /* divided by 1 */
//    CARD_UART->LCR = 0x9B;  /* 8 bits, Even Parity, Parity Enable, 1 Stop bit */
    CARD_UART->LCR = 0x80; // DLAB = 1;
    CARD_UART->LCR |= 0x03; // 8 bit
    CARD_UART->LCR |= (0 << 2); // 1 stop bit
    CARD_UART->LCR |= (1 << 3);  // Parity Enable
    CARD_UART->LCR |= (0x1 << 4); // Even Parity

    regVal = LPC_SYSCON->UARTCLKDIV;
    uint32_t UartPCLK = ((38000000/LPC_SYSCON->SYSAHBCLKDIV)/regVal);
    Fdiv = (UartPCLK / (16 * DEFAULT_BAUDRATE));
    CARD_UART->DLL = (uint8_t)(Fdiv & 0xFF);
    CARD_UART->DLM = (uint8_t)(Fdiv >> 8);
    CARD_UART->LCR &= ~0x80;      /* DLAB = 0 */
    CARD_UART->FCR = 0x07;        /* Enable and reset TX and RX FIFO. */
    CARD_UART->SCICTRL = 0x01; // Enable Smartcard interface
    regVal = CARD_UART->LSR;
    while (( CARD_UART->LSR & (LSR_THRE|LSR_TEMT)) != (LSR_THRE|LSR_TEMT) );
    while ( CARD_UART->LSR & LSR_RDR ) {
      regVal = CARD_UART->RBR;    /* Dump data from RX FIFO */
    }
    CARD_UART->IER = IER_RBR;   /* Enable UART interrupt */
    NVIC_EnableIRQ(UART_IRQn);
 }


bool scard_cmd_getATR(ISO7816_SC* scard) {
    scard_ColdRst(scard);
    _delay_ms(99); // wait for ATR recieving
    UartSW_Printf("(%u) %A\r", scard->dLen, scard->dBuf, scard->dLen, ' ');
//    uint8_t b = 0;
//    if(read_byte(&b)) {
//        UartSW_Printf("TS fail\r");
//        return false;
//    }
//    UartSW_Printf("TS: %X\r", b);

    // this is an jam
    memcpy(scard->ATR, scard->dBuf, scard->dLen);
    scard->ATRLength = scard->dLen;
    return true;
}

bool scard_power_on(ISO7816_SC* scard) {
    if (!scard_cmd_getATR(scard))
        return false;
    //    scard_pps_req(scard); // PPS exchange
    UartSW_Printf("ATR: %A\r", scard->ATR, scard->ATRLength, ' ');
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
    scard_dataIO_init();
    scard_clock_init();
    pScard = scard;
    pScard->dLen = 0;
    return false; // Dummy
}
