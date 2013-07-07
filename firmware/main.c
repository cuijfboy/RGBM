/* Name: main.c
 * Author: <insert your name here>
 * Copyright: <insert your copyright message here>
 * License: <insert your license reference here>
 */

#include <avr/io.h>
#include <stdint.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include "frames.h"

#define FRAME_SIZE	24

#define RED	0
#define GRN	1
#define BLU	2

#define GRN_H_SEG_L_DP	PORTA
#define RED_DP			PORTB
#define SEG_H_GRN_L_DP	PORTC
#define BLU_UART_DP		PORTD
#define UART_RX_DP		PD0
#define UART_TX_DP		PD1

#define BAUD 9600UL
#define UBRRVAL (F_CPU / (BAUD * 16) - 1)

static uint8_t i, j, frameIndex = 0;
static uint8_t rgb[24] = {
	0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF,
	0xFF, 0xFF, 0xFF
};
//static uint8_t *framePtr;
uint8_t uart_flag, uart_rxBuf;

static void init(void);
static void mtx_dispFrame(void);
static void uart_putChar(uint8_t c);

int main(void) {
	init();
	while(1) ;
}

//---------------------------------

void init(void) {
	DDRA = DDRB = DDRC = DDRD = 0xFF;
	mtx_dispFrame();
	
	TCCR1B |= _BV(WGM12);
	TIMSK |= _BV(OCIE1A);
	TCNT1 = 0x0000;
	OCR1A = 0x009C;
	
	UBRRH = UBRRVAL >> 8;								// set baud rate
    UBRRL = UBRRVAL & 0xFF;
	UCSRB = _BV(RXCIE) | _BV(TXCIE);					// enable serial receiver and transmitter
	UCSRC = _BV(URSEL) | _BV(UCSZ1) | _BV(UCSZ0);		// set frame format: 8 bit, no parity, 1 bit
}

void mtx_dispFrame(void) {
	for(i=0; i<8; i++) {
		SEG_H_GRN_L_DP |= _BV(i) & 0xF0;
		GRN_H_SEG_L_DP |= _BV(i) & 0x0F;
		for(j=0; j<5; j++) {
			RED_DP = rgb[RED + i * 3];
			SEG_H_GRN_L_DP |= rgb[GRN + i * 3] & 0x0F;
			GRN_H_SEG_L_DP |= rgb[GRN + i * 3] & 0xF0;
			BLU_UART_DP = rgb[BLU + i * 3];
			_delay_us(20);
			RED_DP = 0xFF;
			_delay_us(20);
			SEG_H_GRN_L_DP |= 0x0F;
			GRN_H_SEG_L_DP |= 0xF0;
			_delay_us(20);
			BLU_UART_DP = 0xFF;
		}
	}
}

void uart_putChar(uint8_t c) {
    while(!(UCSRA & _BV(UDRE)))	// wait until transmit buffer is empty
		;
    UDR = c;						// send next byte
}

ISR(TIMER1_COMPA_vect) {
	UCSRB |= _BV(RXEN);
	UCSRB |= _BV(TXEN);
	mtx_dispFrame();
	UCSRB &= ~_BV(RXEN);
	UCSRB &= ~_BV(TXEN);
}

ISR(USART_RXC_vect) {
	uart_rxBuf = UDR;
	for(i=0; i<24; i++) {
		rgb[i] = pgm_read_byte(framesData + FRAME_SIZE * frameIndex + i);
		rgb[i] = pgm_read_byte(frameTable + i);
	}
	uart_putChar(uart_rxBuf);
}