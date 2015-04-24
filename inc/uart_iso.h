/*
 * uart_iso.h
 *
 *  Created on: 16 апр. 2015 г.
 *      Author: RLeonov
 */

#ifndef INC_UART_ISO_H_
#define INC_UART_ISO_H_

#include <string.h>
#include "rj_lib_LPC11Uxx.h"
#include "LPC11Uxx.h"

#define CARD_PWR_PIN        PIO1_22
#define PWR_ON()            PinClear(1, 22);
#define PWR_OFF()           PinSet(1, 22);

#define CARD_IO_PORT        0
#define CARD_IO_PIN         19
#define CARD_UART           LPC_USART
#define CARD_UART_IO_CON    LPC_IOCON->PIO0_19

#define IER_RBR             0x01
#define IER_THRE            0x02
#define IER_RLS             0x04

#define IIR_PEND            0x01
#define IIR_RLS             0x03
#define IIR_RDA             0x02
#define IIR_CTI             0x06
#define IIR_THRE            0x01

#define LSR_RDR             0x01
#define LSR_OE              0x02
#define LSR_PE              0x04
#define LSR_FE              0x08
#define LSR_BI              0x10
#define LSR_THRE            0x20
#define LSR_TEMT            0x40
#define LSR_RXFE            0x80

#define UART_IRQ_Channel    UART_IRQn
#define ENABLE_RX_IRQ()     CARD_UART->IER |= IER_RBR   // Enable RX interrupt
#define ENABLE_TX_IRQ()     CARD_UART->IER |= IER_THRE   // Enable TX interrupt
#define DISABLE_RX_IRQ()    CARD_UART->IER &= ~IER_RBR   // Disable RX interrupt
#define DISABLE_TX_IRQ()    CARD_UART->IER &= ~IER_THRE   // Disable TX interrupt

#define CARD_CLK_TMR        LPC_CT16B1
#define CARD_CLK_PORT       0
#define CARD_CLK_PIN        22
#define CARD_CLK_IO_CON     LPC_IOCON->PIO0_22
#define CLK_ON()            CARD_CLK_TMR->TCR = 1
#define CLK_OFF()           CARD_CLK_TMR->TCR = 0

#define IO_RESET()          LPC_IOCON->PIO0_19 &= ~0x07
#define IO_ENABLE()         LPC_IOCON->PIO0_19 |= 0x01 // Port 0 Pin 19 is UART TXD
#define IO_DISABLE()        LPC_IOCON->PIO0_19 &= ~0x01

#define CARD_RST_PIN        PIO1_24
#define RST_HI()            PinSet(1, 24);
#define RST_LO()            PinClear(1, 24);

// T1's I-block
#define I_PCB       (uint8_t)0x00
#define I_NS_BIT    (uint8_t)0x40    // Bit 7 encodes the send-sequence number denoted N(S).
#define I_M_BIT     (uint8_t)0x20    // Bit 6 is the more-data bit denoted M-bit
// T1's R-block
#define R_PCB       (uint8_t)0x80
#define R_NR_BIT    (uint8_t)0x10
#define R_ERR_OK    (uint8_t)0x00
#define R_ERR_CRC   (uint8_t)0x01
#define R_ERR_OTHR  (uint8_t)0x02
// T1's S-block
#define S_PCB       (uint8_t)0xC0

#define MAX_ATR_SIZE        32
#define CARD_BUFFER_SIZE    256

typedef enum {
    cmd_TxOff, cmd_TxIdle, cmd_TxPCB, cmd_TxLEN, cmd_TxINFO, cmd_TxLRC
} cmd_TxState_t;

typedef enum {
    cmd_RxOff, cmd_RxNAD, cmd_RxPCB, cmd_RxLEN, cmd_RxINFO, cmd_RxLRC, cmd_RxFinal
} cmd_RxState_t;

typedef struct {
    // protocol using
    uint8_t TxPCB;
    uint8_t NAD, PCB, LEN;
    uint8_t LRC;
    uint8_t Buf[256];
    uint32_t Len;
    cmd_TxState_t TxState;
    cmd_RxState_t RxState;
    uint8_t* pTxByte;
    uint8_t TxCnt;
    bool TSreceived;
} DEx_t;

typedef enum {
    scs_Error, scs_Off, scs_Idle, scs_Busy
} CardState_t;

typedef struct {
    // low level driver
    DEx_t DEx; // data exchange
    // internal usage
    uint8_t FindexDindex;
    uint8_t BWI_CWI_T1;
    uint8_t N;
    uint8_t ProtocolType;
    uint8_t IFSC;
    // external usage
    uint8_t ATR[MAX_ATR_SIZE];
    uint8_t ATRLength;
    CardState_t State;
} ISO7816_SC;

void card_lld_init(DEx_t* pDex);
void card_switch_to_highspeed();
uint32_t card_lld_data_pps(ISO7816_SC* pScard, uint8_t* pData, uint8_t inLen, uint8_t OutLength);
void card_lld_data_exchange_asynch(ISO7816_SC* pScard);
uint8_t card_lld_data_exchange_synch(ISO7816_SC* pScard);


#endif /* INC_UART_ISO_H_ */
