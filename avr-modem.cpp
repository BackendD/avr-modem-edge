#include "avr-modem.h"
#include <avr/interrupt.h>

#define TCNT_PER_LOW_FREQ                    ((F_CPU / 8.0) / LOW_FREQ)
#define TCNT_PER_HIGH_FREQ                   ((F_CPU / 8.0) / HIGH_FREQ)

#define MIN_TCNT_PER_LOW_FREQ                (2 * (TCNT_PER_LOW_FREQ * TCNT_PER_HIGH_FREQ) / (TCNT_PER_LOW_FREQ + TCNT_PER_HIGH_FREQ))
#define MAX_TCNT_PER_LOW_FREQ                (2 * (TCNT_PER_LOW_FREQ * TCNT_PER_LOW_FREQ) / (TCNT_PER_LOW_FREQ + TCNT_PER_HIGH_FREQ))
#define MIN_TCNT_PER_HIGH_FREQ               (2 * (TCNT_PER_HIGH_FREQ * TCNT_PER_HIGH_FREQ) / (TCNT_PER_LOW_FREQ + TCNT_PER_HIGH_FREQ))
#define MAX_TCNT_PER_HIGH_FREQ               (2 * (TCNT_PER_LOW_FREQ * TCNT_PER_HIGH_FREQ) / (TCNT_PER_LOW_FREQ + TCNT_PER_HIGH_FREQ))

#define TCNT_PER_BAUD                        ((F_CPU / 8.0) / BAUD_RATE)

enum ReceiveStatus {
	START_BIT = 0, DATA_BIT = 8, STOP_BIT = 9, INACTIVE = 0xff
};

AvrModem::AvrModem() {
}

AvrModem::~AvrModem() {
}

AvrModem *AvrModem::activeObject = 0;

void AvrModem::listen(void (*onReceive)(char*)) {

	// calling this function after every receiving message
	_onReceive = onReceive;
	// adjust PB2 (AIN0) port for FSKIN (connected to the speaker), wave receive		~~ ==> |PB2(AIN0)
	DDRB &= ~(1 << 2);
	PORTB &= ~(1 << 2);
	// adjust PA2 (AIN1) port for static voltage (connected to the static voltage)		-- ==> |PA2(AIN1)
	DDRA &= ~(1 << 2);
	PORTA &= ~(1 << 2);

	AvrModem::activeObject = this;

	_receiveStatus = INACTIVE;
	_receiveBufferHead = _receiveBufferTail = 0;

	// timer1
	_lastTCNT = TCNT1;
	// low or high count += TCNT1 - _lastTCNT
	_lowCount = _highCount = 0;
	// adjust timer 1 mode to 0 (normal)
	// start from 0 to 0xffff and overflowing
	TCCR1A &= ~((1 << WGM11) | (1 << WGM10));               // TCCR1A   ------00
	TCCR1B &= ~((1 << WGM13) | (1 << WGM12));               // TCCR1B   ---00---
	// adjust timer 1 clock to 1/8 of microcontroller clock ==> 1/8 * 8000000 = 1000000
	TCCR1B &= ~((1 << CS12) | (1 << CS10));                 // TCCR1B   -----0-0
	TCCR1B |= (1 << CS11);                                  // TCCR1B   ------1-
	// adjust comparator interrupt on falling output edge
	ACSR |= (1 << ACIS1);                                   // ACSR     ------1-
	ACSR &= ~(1 << ACIS0);                                  // ACSR     -------0
	// enable analog comparator multiplexer
	SFIOR |= (1 << ACME);                                   // SFIOR    ----1---
	// select ADC2 (PA2) as negative input (AIN1)
	ADMUX |= ~((1 << MUX2) | (1 << MUX0));                  // ADMUX    -----0-0
	ADMUX |= (1 << MUX1);                                   // ADMUX    ------1-
	// enable analog comparator interrupt
	ACSR |= (1 << ACIE);									// ACSR		----1---
}

// analog comparator interrupt
ISR(ANA_COMP_vect) {
	AvrModem::activeObject->demodulate();
}

void AvrModem::demodulate(void) {

	// get a copy of the TCNT value right now
	uint16_t t = TCNT1;
	// calculate the amount of the difference with the previous value of TCNT
	uint16_t diff = t - _lastTCNT;

	// wave is too short
	if (diff < (uint16_t) MIN_TCNT_PER_HIGH_FREQ)
		return;
	_lastTCNT = t;
	// wave is too long
	if (diff > (uint16_t) MAX_TCNT_PER_LOW_FREQ) {
		_receiveStatus = INACTIVE;
		_lowCount = _highCount = 0;
		// disable OCR1A and TCNT1 compare match interrupt
		TIMSK &= ~(1 << OCIE1A);                    // TIMSK    ---0----
		return;
	}

	// zero's wave
	if (diff > (uint16_t) MIN_TCNT_PER_LOW_FREQ) {
		_lowCount += diff;
		if (_receiveStatus == INACTIVE)
			// Start bit detection
			if (_lowCount > (uint16_t) (TCNT_PER_BAUD * 0.5)) {
				_receiveStatus = START_BIT;
				_highCount = 0;

				// upon compare match (OCR1A == TCNT1) the micro will jump to the output compare A interrupt vetor
				OCR1A = t + (uint16_t) TCNT_PER_BAUD - _lowCount;
				// timer interrupt flag of OCR1A and TCNT1 compare match
				// will be 0 when try to set it 1
				TIFR |= (1 << OCF1A);               // TIFR     ---0----
				// enable OCR1A and TCNT1 compare match interrupt
				TIMSK |= (1 << OCIE1A);
			}
	}
	// one's wave
	else if (_receiveStatus == INACTIVE)
		_lowCount = _highCount = 0;
	else
		_highCount += diff;
}

// timer 1 compare match interrupt A
ISR(TIMER1_COMPA_vect) {
	OCR1A += (uint16_t) TCNT_PER_BAUD;
	AvrModem::activeObject->receive();
}

void AvrModem::receive(void) {
	uint8_t high;

	// Bit logic determination
	if (_highCount < _lowCount) {
		_lowCount = 0;
		high = 0x00;			//0b00000000
	} else {
		_highCount = 0;
		high = 0x80;			//0b10000000
	}

	// Start bit reception
	if (_receiveStatus == START_BIT && !high) {
		_receiveStatus++;
		return;
	}
	// Data bit reception
	if (_receiveStatus <= DATA_BIT) {
		_receiveBits >>= 1;
		_receiveBits |= high;
		_receiveStatus++;
		return;
	}
	// Stop bit reception
	if (_receiveStatus == STOP_BIT) {
		if (high) {
			// message character
			if (_receiveBits) {

				// store the received bite into the buffer
				*(_receiveBuffer + _receiveBufferTail) = _receiveBits;
				// move the receive buffer tail one step forward
				++_receiveBufferTail &= (RX_BUF_SIZE - 1);

				// if buffer has overflowed, ignore the last character
				// by moving buffer head one step forward
				if (_receiveBufferTail == _receiveBufferHead)
					++_receiveBufferHead &= (RX_BUF_SIZE - 1);
			}
			// end of transmission
			else {
				uint8_t msgSize = (_receiveBufferTail - _receiveBufferHead)
						& (RX_BUF_SIZE - 1);

				// plus 1 for setting 0 at the end of the message
				char msg[msgSize + 1];
				// full the message from the buffer
				for (uint8_t i = 0; i < msgSize; i++) {
					msg[i] = *(_receiveBuffer + _receiveBufferHead);
					++_receiveBufferHead &= (RX_BUF_SIZE - 1);
				}
				// setting 0 at the end of the message
				msg[msgSize] = 0;

				_onReceive(msg);
			}
		}
	}
	_receiveStatus = INACTIVE;
	TIMSK &= ~(1 << OCIE1A);						// TIMSK	---0----
}

void AvrModem::write(char *message, size_t size) {
}

void AvrModem::modulate(void) {
}
