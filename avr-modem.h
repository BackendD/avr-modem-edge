#ifndef AVR_MODEM_H_
#define AVR_MODEM_H_

#include <stdint-gcc.h>
#include <stdlib.h>

#define BAUD_RATE          (1225)     // boud/s
// low frequency for 0
#define LOW_FREQ           (4900)     // Hz
// high frequency for 1
#define HIGH_FREQ          (7350)     // Hz
// buffer size. Must be a power of 2 and greater than 8 {8, 16, 32, 64, 128,...}
#define RX_BUF_SIZE        (16)       // Byte

class AvrModem {
public:
	AvrModem();
	~AvrModem();
	static AvrModem *activeObject;
	virtual void listen(void (*onReceive)(char*));
	virtual void write(char *message, size_t size);
	void modulate(void);
	void demodulate(void);
	void receive(void);
private:
	uint8_t     _receiveStatus;
	char        _receiveBits;							// every bit of data will be stored in this byte
	uint8_t     _receiveBufferHead;
	uint8_t     _receiveBufferTail;
	char        _receiveBuffer[RX_BUF_SIZE];
	uint16_t    _lastTCNT;
	uint16_t    _lowCount;
	uint16_t    _highCount;
	void        (*_onReceive)(char*);
};

#endif /* AVR_MODEM_H_ */
