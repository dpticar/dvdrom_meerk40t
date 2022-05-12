#ifndef serial_h
#define serial_h

//#define BAUD_RATE 57600
//#define BAUD_RATE 250000
#define BAUD_RATE 500000
//#define BAUD_RATE   1000000

uint8_t oneWireCRC(uint8_t crc, uint8_t inbyte);
void serial_init();
void serial_write(uint8_t data);
uint8_t serial_read();
uint8_t serial_peek();
uint8_t serial_available();

void printString(const char *s);
void printChar(char c);
void printInt(uint32_t n);
void printInt(int32_t n);
void printInt(int16_t n);
void printInt(uint16_t n);
void printInt(int8_t n);
void printInt(uint8_t n);
void printFloat(float x, uint8_t dec=3);

#endif
