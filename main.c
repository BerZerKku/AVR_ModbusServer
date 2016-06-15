/*
 * main.c
 *
 *  Created on: 09.06.2016
 *      Author: Shcheblykin
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include "define.h"
#include "modbus.h"

/// ����� ������ PORTD ��� ���������� ������������ �������� �� UART.
#define PIND_UART_CTRL 2
/// ����� ������ PORTD ��� �������� ����� 1.
#define PIND_TP1 6
/// ����� ������ PORTD ��� �������� ����� 2.
#define PIND_TP2 7
/// ������ ������ ������ ��� UART.
#define BUF_UART_MAX 15
/// �������� �������� ��������� ��� ����������� ��������� ������ ������ MODBUS
#define TIME_SYMBOLS_3_5 124

typedef enum {
	FSM_READ1_TX 	= 0,
	FSM_READ1_RX	= 1,
	FSM_READ2_TX	= 2,
	FSM_READ2_RX	= 3,
	FSM_WRITE1_TX	= 4,
	FSM_WRITE1_RX	= 5,
	FSM_ERROR		= 6
} FSM;


// ��������� ������������� ���������
void low_level_init() __attribute__((__naked__)) __attribute__((section(".init3")));
static void setTxAdm485();
static void setRxAdm485();
static uint8_t isLoopEnd();
static void startTimer(uint8_t ocr);

/// ����� ������ ��� �������� UART.
uint8_t bufUartTx[BUF_UART_MAX];
/// ����� ������ ��� ������ UART
uint8_t bufUartRx[BUF_UART_MAX];
/// ������� ������� � ������ UART.
volatile uint8_t cntBufUart = 0;
/// ���������� ���� ������ �� ��������.
volatile uint8_t lenTxUart = 0;
/// ������� ������� ����� ������� �� ������ UART � 0.1 ��
volatile uint8_t intervalRx = 0;

volatile uint8_t debug = 0;

/// ��������� ADM485 �� ��������.
static void setTxAdm485() {
	PORTD |= (1 << PIND_UART_CTRL);
}

/// ��������� ADM485 �� �����.
static void setRxAdm485() {
	PORTD &= ~(1 << PIND_UART_CTRL);
}

void setTP1(uint8_t value) {
	if (value == TRUE) {
		PORTD |= (1 << PIND_TP1);
	} else {
		PORTD &= ~(1 << PIND_TP1);
	}
}

void switchTP1() {
	PORTD ^= (1 << PIND_TP1);
}

/**	������ �������� ��������� ������� 3.5.
 *
 * 	��� �������� UART 9600, 8 ��� ������ � 2-� ����-����� �� 1 ������ ����������
 * 	11 ��� ������. ��������, ��� ���������� ��������� 3,5*11/9600 = 4��. ���
 * 	������� 8��� � �������� 256 ��������:
 * 	OCR = Period*(F_CPU/PRESCALER) - 1 = 0.004*(8000000/256) -1 = 124
 */
static void startTimer(uint8_t ocr) {
	TCNT0 = 0;
	OCR0 = ocr;
	TCCR0 = (1 << WGM01) | (0 << WGM00) |				// CTC
			(1 << CS02 ) | (0 << CS01)  | (0 << CS00);	// prescaler 256
	TIMSK = (1 << OCIE0);
}

/**	�������� ��������� ��������� �����.
 *
 * 	return TRUE ���� ���� ������, FALSE - � ��������� ������.
 */
static uint8_t isLoopEnd() {

	if (TIFR & (1 << OCF1A)) {
		TIFR |= (1 << OCF1A);
		return TRUE;
	}

	return FALSE;
}

static void trData(uint8_t len) {
	lenTxUart = len;
	cntBufUart = 0;

	UCSRB |= (1 << TXEN);
	UCSRB &= ~((1 << RXCIE) | (1 << RXEN));

	UDR = bufUartTx[cntBufUart++];

	UCSRB |= (1 << TXCIE) | (1 << UDRIE);
}

/// �������� ���������.
int main() {
	uint8_t step = FSM_READ1_TX;
	MODBUS_STATE_RX state = MODBUS_STATE_RX_ERROR_DATA;

	bufUartTx[0] = bufUartRx[0] = 0xFF;

	sei();

	while(1) {
		if (isLoopEnd()) {
			PORTD ^= (1 << PIND_TP2);

			switch(step) {
				case FSM_READ1_TX: {
					trData(MODBUS_readHoldingRegisters(bufUartTx, 1, 1, 3));
					step = FSM_READ1_RX;
				} break;

				case FSM_READ1_RX: {
					state = MODBUS_checkReadData(bufUartRx, bufUartTx, cntBufUart);
					debug = state;
					if (state == MODBUS_STATE_RX_OK) {
						step = FSM_READ2_TX;
					} else {
						step = FSM_ERROR;
					}
				} break;

				case FSM_READ2_TX: {
					trData(MODBUS_readHoldingRegisters(bufUartTx, 1, 4, 1));
					step = FSM_READ2_RX;
				} break;

				case FSM_READ2_RX: {
					state = MODBUS_checkReadData(bufUartRx, bufUartTx, cntBufUart);
					if (state == MODBUS_STATE_RX_OK) {
						step = FSM_WRITE1_TX;
					} else {
						step = FSM_ERROR;
					}
				} break;

				case FSM_WRITE1_TX: {
					trData(MODBUS_writeSingleRegister(bufUartTx, 1, 1, 11));
					step = FSM_WRITE1_RX;
				} break;

				case FSM_WRITE1_RX: {
					state = MODBUS_checkReadData(bufUartRx, bufUartTx, cntBufUart);

					debug = state;

					if (state == MODBUS_STATE_RX_OK) {
						step = FSM_READ1_TX;
					} else {
						step = FSM_ERROR;
					}
				} break;

				case FSM_ERROR: {
					step = FSM_READ1_TX;
				} break;
			}
		}

		PORTA = debug;
	}
}

/**	��������� �� ���������� ������ 0.
 *
 * 	���������� ������ ������ �� UART, �.�. ����������� ������ ��������� �
 * 	����������� ����������. ��� �� ��������������� � ��� ������ 0.
 */
ISR(TIMER0_COMP_vect) {
	UCSRB &= ~((1 << RXCIE) | (1 << RXEN));
	TCCR0 &= ~((1 << CS02) | (1 << CS01) | (1 << CS00));
}


/** ��������� �� ����������� ������ ����������� UART.
 *
 * 	���������� ��������� ���� ������ �� ������.
 * 	��� �������� ���������� ����������� ���������� �� ����������� ������ �
 * 	���������� ���������� �� ���������� ��������.
 */
ISR(USART_UDRE_vect) {
	if (cntBufUart < lenTxUart) {
		UDR = bufUartTx[cntBufUart++];
	} else {
		UCSRB &= ~(1 << UDRIE);
	}
}

/** ��������� �� ���������� �������� UART.
 *
 * 	��� ���������� �������� ���������� ����� ���������� �����������,����������
 * 	��������, ���������� ��������� �� ������, ���������� ������� �������� ����.
 *
 * 	���������� ������� ������� � ������ UART � ���-�� ���� �� ��������.
 */
ISR(USART_TXC_vect) {
	UCSRB &= ~((1 << TXCIE) | (1 << TXEN)| (1 << UDRIE));
	UCSRB |= (1 << RXCIE) | (1 << RXEN);
	cntBufUart = 0;
}

/** ��������� �� ������ ������ UART.
 *
 *	����������� ���������� �������� ������ � ��� ������� ����� ���������� �
 *	����� UART.
 */
ISR(USART_RXC_vect) {
	uint8_t state = UCSRA;
	uint8_t tmp = UDR;

	startTimer(TIME_SYMBOLS_3_5);

	switchTP1();

	// �������� ������ ������ �������
	if (!(state & ((1 << PE) | (1 << DOR) | (1 << FE)))) {
		// �������� ������������ ������
		if (cntBufUart < BUF_UART_MAX) {
			bufUartRx[cntBufUart++] = tmp;
		}
	}
}

/**	��������� ������������� ���������.
 *
 * 	��-��������� �� ����������� �� �������� ������ 8���.
 *
 */
void low_level_init() {

	// PORTA - DEBUG
	PORTA = 0;
	DDRA = 0xFF;

	// PORTD
	// PD.0	RXD		in_alt_z	���� UART
	// PD.1 TXD		out_alt_hi	����� UART
	// PD.2 CTRL 	out_low		����� ���������� ������������ �������� UART
	// PD.7 TP		out_low 	�������� �����
	PORTD = (0 << PIND_UART_CTRL) | (0 << PIND_TP1) | ( 0 << PIND_TP2);
	DDRD = (1 << PIND_UART_CTRL) | (1 << PIND_TP1) | ( 1 << PIND_TP2);

	// BAUD = 9600
	// U2X = 0 -> UBRR = F_CPU/(16*BAUD) - 1 = 8000000/(16*9600) - 1 = 51
	UBRRL = 51;
	UCSRA = (0 << U2X);
	UCSRB = (0 << UCSZ2);
	UCSRC = (1 << URSEL) |
			(0 << UMSEL) |					// asynchronous operation
			(0 << UPM1)  | (0 << UPM0) |	// parity disabled
			(1 << USBS)	 |					// 2 stop bits
			(1 << UCSZ1) | (1 << UCSZ0);	// 8-bits character size (UCSZ2 = 0)

	// Timer 0
	// CTC Mode, Period = 0.004s
	// OCR = Period*(F_CPU/PRESCALER) - 1 = 0.004*(8000000/256) -1 = 124
	//
//	OCR0 = 124;
//	TCNT0 = 0;
//	TCCR0 = (1 << WGM01) | (0 << WGM00);				// CTC
//			(1 << CS02 ) | (0 << CS01)  | (0 << CS00);	// prescaler 256
//	TIMSK = (1 << OCIE0);

	// Timer 1
	// CTC mode, period 100ms
	// OCR = Period*(F_CPU/PRESCALER) - 1 = 0.1*(8000000/64) -1 = 12499
	TCNT1 = 0;
	OCR1A = 12499*2 + 1;
	TCCR1A = (0 << WGM11) | (0 << WGM10);				// CTC
	TCCR1B = (0 << WGM13) | (1 << WGM12) |
			 (0 << CS12)  | (1 << CS11)  | (1 << CS10);	// prescaler 64
};
