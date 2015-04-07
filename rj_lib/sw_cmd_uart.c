/*
 * sw_cmd_uart.c
 *
 *  Created on: 07 апр. 2015 г.
 *      Author: RLeonov
 */

#include "sw_cmd_uart.h"
#include "sprintf.h"

#define TX_ACTIVE   1

static volatile uint8_t edgeCnt; //no of char edges
static volatile uint8_t edgeIdx; //edge index for output
static volatile int32_t Edge[11]; //array of edges
static volatile uint8_t lEdge, charEnd; //last two events index

//software UART Tx related definitions
static volatile uint32_t TxWrite, TxRead;
static volatile int32_t TxCnt;
static volatile uint32_t TxBuf[TXBUFF_LEN];
//definitions common for transmitting and receiving data
static volatile int32_t swu_status;
static uint32_t BitLength;

uint8_t OutBuf[SW_OUT_BIF_SZ];
uint8_t *pOut = OutBuf;

static inline void PinSet(uint32_t portNum, uint32_t bitPosi, uint32_t bitVal ) {
    if(bitVal) {
        LPC_GPIO->SET[portNum] |= (bitVal<<bitPosi);
    }
    else LPC_GPIO->CLR[portNum] |= (bitVal<<bitPosi);
}

static void UartSW_SetTx(void) {
    uint8_t bit, i;
    uint32_t ext_data, delta_edges, mask, reference;
    if (TxWrite != TxRead) //data to send, proceed
    {
        swu_status |= TX_ACTIVE; //sw uart tx is active
        TxRead++; //update the tx fifo ...
        if (TxRead == TXBUFF_LEN) //read index...
            TxRead = 0; //...
        ext_data = (unsigned long int) TxBuf[TxRead]; //read the data
        ext_data = 0xFFFFFE00 | (ext_data << 1); //prepare the pattern
        Edge[0] = BitLength; //at least 1 falling edge...
        edgeCnt = 1; //... because of the START bit
        bit = 1; //set the bit counter
        reference = 0x00000000; //init ref is 0 (start bit)
        mask = 1 << 1; //prepare the mask
        delta_edges = BitLength; //next edge at least 1 bit away
        while (bit != 10) { //until all bits are examined
            if ((ext_data & mask) == (reference & mask)) { //bit equal to the reference?
                delta_edges += BitLength; //bits identical=>update length
            } //...
            else { //bits are not the same:
                Edge[edgeCnt] = //store new...
                        Edge[edgeCnt - 1] + delta_edges; //... edge data
                reference = ~reference; //update the reference
                delta_edges = BitLength; //reset delta_ to 1 bit only
                edgeCnt++; //update the edges counter
            }
            mask = mask << 1; //update the mask
            bit++; //move on to the next bit
        }
        Edge[edgeCnt] = //add the stop bit end...
                Edge[edgeCnt - 1] + delta_edges; //... to the list
        edgeCnt++; //update the number of edges
        lEdge = edgeCnt - 2; //calculate the last edge index
        charEnd = edgeCnt - 1; //calc. the character end index

        edgeIdx = 0; //reset the edge index
        reference = SW_UART_TMR->TC + BitLength; //get the reference from TIMER0
        for (i = 0; i != edgeCnt; i++) //recalculate toggle points...
            Edge[i] = (Edge[i] + reference) //... an adjust for the...
                    & 0x3FFFFFFF; //... timer range
        SW_UART_TMR->MR3 = Edge[0]; //load MR3
        SW_UART_TMR->MCR = SW_UART_TMR->MCR | (1 << 9); //enable interrupt on MR3 match
        SW_UART_TMR->EMR = SW_UART_TMR->EMR | (3 << 10); //enable toggle on MR3 match
    }
    return; //return from the routine
}

static inline void FPut(char c) {
    *pOut++ = c;
    if(pOut >= &OutBuf[SW_OUT_BIF_SZ]) return;   // Block buffer
}

void UartSW_Printf(const char *format, ...) {
    uint32_t MaxLength = SW_OUT_BIF_SZ;
    va_list args;
    va_start(args, format);
    uint32_t Count = vsprintf(FPut, MaxLength, format, args);
    va_end(args);
    // Start transmission
    uint8_t i;
    for(i = 0; Count > 0; Count--, i++) {
        UartSW_PrintChar(OutBuf[i]);
    }
    pOut = OutBuf;
}

void UartSW_PrintStr(char* Str) {
    while (*Str != '\0') {
        UartSW_PrintChar(*Str++);
    }
}

void UartSW_PrintChar(char out_char) {
    while (TxCnt == TXBUFF_LEN); // wait if the tx FIFO is full
    TxWrite++; // update the write pointer
    if (TxWrite == TXBUFF_LEN)
        TxWrite = 0;
    TxBuf[TxWrite] = out_char; // put the char into the FIFO
    TxCnt++; // update no.of chrs in the FIFO
    if ((swu_status & TX_ACTIVE) == 0)
        UartSW_SetTx(); // start tx if tx is not active
    // write access to tx FIFO end
    return; // return from the routine
}

static inline void UartSW_IrqTx() {
    if ((SW_UART_TMR->IR & 0x08) != 0) {
        SW_UART_TMR->IR = 0x08; //clear the MAT3 flag
        if (edgeIdx == charEnd) { // if the end of the char?
            SW_UART_TMR->MCR &= ~(7 << 9); // MR3 impacts T0 ints no more
            TxCnt--; // update no. of chars
            if (TxWrite != TxRead) // if more data
                UartSW_SetTx(); // another transmission
            else
                swu_status &= ~TX_ACTIVE; // no data left - turn off the tx
        } else {
            if (edgeIdx == lEdge) // is this the last toggle?
                SW_UART_TMR->EMR = 0x000003FF; // no more toggle on MAT3
            edgeIdx++; // update the edge index
            SW_UART_TMR->MR3 = Edge[edgeIdx]; // prepare the next toggle event
        }
    }
}

void UartSW_Init(uint32_t Baudrate) {
    // setup the software uart
    TxCnt = 0; // no data in the swu tx FIFO
    TxWrite = 0; // last char written was on 0
    swu_status = 0; // neither tx nor rx active

    BitLength = SystemCoreClock/Baudrate;     // 48000000/9600 = 5000 PCLKs
    LPC_SYSCON->SYSAHBCLKCTRL |= (1 << 9);
    LPC_SYSCON->SYSAHBCLKCTRL |= (1 << 12);
    LPC_SYSCON->UARTCLKDIV = 0x1;     /* divided by 1 */

    LPC_IOCON->PIO1_27 &= ~0x07;
    LPC_IOCON->PIO1_27 |= 0x01; /* Timer0_32 MAT3 */

    //Timer0_32 setup
    SW_UART_TMR->TCR = 0x00; // stop TIMER0_32
    SW_UART_TMR->TCR = 0x02; // reset counter
    SW_UART_TMR->TCR = 0x00; // release the reset

    SW_UART_TMR->IR = 0x01F; // clear all TIMER0 flags
    SW_UART_TMR->PR = 0x00000000; // no prescaler
    SW_UART_TMR->MR0 = 0x3FFFFFFF; // TIMER0_32 counts 0 - 0x3FFFFFFF
    SW_UART_TMR->MCR = 2; // reset TIMER0_32 on MR0
    SW_UART_TMR->EMR = 0x0008; // drive MAT0.3 high
    SW_UART_TMR->TCR = 0x01; // let TIMER0_32 run
    NVIC_EnableIRQ(TIMER_32_0_IRQn); // Enable the TIMER0_32 Interrupt
    return;
}

void TIMER32_0_IRQHandler() {
    UartSW_IrqTx();
}

