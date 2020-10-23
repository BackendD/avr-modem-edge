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
#define RX_BUF_SIZE        (32)       // Byte

class AvrModem {
public:
	AvrModem();
	virtual ~AvrModem();
	static AvrModem *activeObject;
	virtual int listen(void (*onReceive)(uint8_t *));
	virtual size_t write(uint8_t *message, size_t size);
	void modulate(void);
	void demodulate(void);
	void recv(void);
};

#endif /* AVR_MODEM_H_ */
