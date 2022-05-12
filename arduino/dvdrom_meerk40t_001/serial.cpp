#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
//#include <avr/wdt.h>
#include "serial.h"

#define RX_PACKET_SIZE 34
#define RX_DATA_SIZE 30
#define SERIAL_RX1
//#define SERIAL_RX_NOPCKT

#define EMPTY_BUFFER 0xFF

#define LED13_ON() PORTB = PORTB | (1 << 5)
#define LED13_OFF() PORTB = PORTB & (~(1 << 5))

#define TX_BUFFER_SIZE 17
uint8_t TX_buffer[TX_BUFFER_SIZE];
volatile uint8_t TX_buffer_head = 0;
volatile uint8_t TX_buffer_tail = 0;

extern uint8_t acrc;

#define reply_OK 206         // 0xCE
#define replay_BUSY 238      // 0xEE
#define replay_CRC_ERROR 207 // 0xCF

#define cmd_STATUS 0xA0
#define cmd_PACKET 0xA6
#define cmd_1 0xAF //


// in RAM
uint8_t crc_table[] = {
    0x00, 0x5E, 0xBC, 0xE2, 0x61, 0x3F, 0xDD, 0x83, 0xC2, 0x9C, 0x7E, 0x20, 0xA3, 0xFD, 0x1F, 0x41,
    0x9D, 0xC3, 0x21, 0x7F, 0xFC, 0xA2, 0x40, 0x1E, 0x5F, 0x01, 0xE3, 0xBD, 0x3E, 0x60, 0x82, 0xDC,
    0x23, 0x7D, 0x9F, 0xC1, 0x42, 0x1C, 0xFE, 0xA0, 0xE1, 0xBF, 0x5D, 0x03, 0x80, 0xDE, 0x3C, 0x62,
    0xBE, 0xE0, 0x02, 0x5C, 0xDF, 0x81, 0x63, 0x3D, 0x7C, 0x22, 0xC0, 0x9E, 0x1D, 0x43, 0xA1, 0xFF,
    0x46, 0x18, 0xFA, 0xA4, 0x27, 0x79, 0x9B, 0xC5, 0x84, 0xDA, 0x38, 0x66, 0xE5, 0xBB, 0x59, 0x07,
    0xDB, 0x85, 0x67, 0x39, 0xBA, 0xE4, 0x06, 0x58, 0x19, 0x47, 0xA5, 0xFB, 0x78, 0x26, 0xC4, 0x9A,
    0x65, 0x3B, 0xD9, 0x87, 0x04, 0x5A, 0xB8, 0xE6, 0xA7, 0xF9, 0x1B, 0x45, 0xC6, 0x98, 0x7A, 0x24,
    0xF8, 0xA6, 0x44, 0x1A, 0x99, 0xC7, 0x25, 0x7B, 0x3A, 0x64, 0x86, 0xD8, 0x5B, 0x05, 0xE7, 0xB9,
    0x8C, 0xD2, 0x30, 0x6E, 0xED, 0xB3, 0x51, 0x0F, 0x4E, 0x10, 0xF2, 0xAC, 0x2F, 0x71, 0x93, 0xCD,
    0x11, 0x4F, 0xAD, 0xF3, 0x70, 0x2E, 0xCC, 0x92, 0xD3, 0x8D, 0x6F, 0x31, 0xB2, 0xEC, 0x0E, 0x50,
    0xAF, 0xF1, 0x13, 0x4D, 0xCE, 0x90, 0x72, 0x2C, 0x6D, 0x33, 0xD1, 0x8F, 0x0C, 0x52, 0xB0, 0xEE,
    0x32, 0x6C, 0x8E, 0xD0, 0x53, 0x0D, 0xEF, 0xB1, 0xF0, 0xAE, 0x4C, 0x12, 0x91, 0xCF, 0x2D, 0x73,
    0xCA, 0x94, 0x76, 0x28, 0xAB, 0xF5, 0x17, 0x49, 0x08, 0x56, 0xB4, 0xEA, 0x69, 0x37, 0xD5, 0x8B,
    0x57, 0x09, 0xEB, 0xB5, 0x36, 0x68, 0x8A, 0xD4, 0x95, 0xCB, 0x29, 0x77, 0xF4, 0xAA, 0x48, 0x16,
    0xE9, 0xB7, 0x55, 0x0B, 0x88, 0xD6, 0x34, 0x6A, 0x2B, 0x75, 0x97, 0xC9, 0x4A, 0x14, 0xF6, 0xA8,
    0x74, 0x2A, 0xC8, 0x96, 0x15, 0x4B, 0xA9, 0xF7, 0xB6, 0xE8, 0x0A, 0x54, 0xD7, 0x89, 0x6B, 0x35};

// fast, but it's faster to do  crc = crc_table[(crc ^ inbyte)] in RX isr
uint8_t oneWireCRC(uint8_t crc, uint8_t inbyte)
{
  // crc = pgm_read_byte(crc_table + (crc ^ inbyte)); //slow
  crc = crc_table[(crc ^ inbyte)];
  return crc;
}

uint8_t crc = 0;
//------------------------------------------------RX1--------------------------------------------------------------
#if defined(SERIAL_RX1)

#define RX_BUFFER_SIZE 241

uint8_t crcError = 0;

uint8_t RX_buffer[RX_BUFFER_SIZE];
volatile uint8_t RX_buffer_head = 0;
volatile uint8_t RX_buffer_tail = 0;

uint8_t packetByte = 0;
uint8_t RX_buffer_head_unconfirmed = 0;

ISR(USART_RX_vect)
{
 
  uint8_t data = UDR0;

  uint8_t next_head_unconfirmed;
  next_head_unconfirmed = RX_buffer_head_unconfirmed + 1;

  // sei();

  if (next_head_unconfirmed == RX_BUFFER_SIZE)
    next_head_unconfirmed = 0;

  if (packetByte >= 2 && packetByte <= 31) // data
  {
    crc = crc_table[(crc ^ data)];                  //  CRC 
    RX_buffer[RX_buffer_head_unconfirmed] = data;
    RX_buffer_head_unconfirmed = next_head_unconfirmed;
    packetByte++;
    
    return;
  }

  if (data == cmd_STATUS && packetByte == 0)
  {
    if (crcError)
    {
      UDR0 = replay_CRC_ERROR;
      
      return;
    }

    uint8_t tail = RX_buffer_tail;
    uint8_t head = RX_buffer_head;
    uint8_t a = head - tail; // a -> serial_available()
    if (head < tail)
      a = RX_BUFFER_SIZE + a;

    if ((RX_BUFFER_SIZE - 1) - a < RX_DATA_SIZE)
    {
      UDR0 = replay_BUSY;
      
      return;
    }
    UDR0 = reply_OK;
    
    return;
  }

  if (data == cmd_PACKET && packetByte == 32) // 0xa6   end-data-block
  { 
    packetByte++;
    
    return;
  }

  if (data == cmd_PACKET && packetByte == 0)
  {
    crc = 0;
    packetByte = 1;
    
    return;
  }

  if (data == 0x00 && packetByte == 1)
  { // 0x00     start-data-block
    packetByte++;
   
    return;
  }

  if (packetByte == 33) // CRC
  {
    if (crc != data)
    {
      crcError = true;
    }
    else
    {
      RX_buffer_head = RX_buffer_head_unconfirmed;
      crcError = false;
    }
    crc = 0;
    packetByte = 0;
    
    return;
  }

  if (data == cmd_1 && packetByte == 0) // debug
  {
    serial_write(acrc);
    
    return;
  }
}

uint8_t serial_read()
{
  uint8_t tail = RX_buffer_tail;
  if (RX_buffer_head == tail)
  {
    return EMPTY_BUFFER;
  }
  else
  {
    uint8_t data = RX_buffer[tail];
    tail++;
    if (tail == RX_BUFFER_SIZE)
    {
      tail = 0;
    }

    cli();
    RX_buffer_tail = tail;
    sei();

    return data;
  }
}

uint8_t serial_peek()
{
  uint8_t tail = RX_buffer_tail;
  if (RX_buffer_head == tail)
    return EMPTY_BUFFER;
  else
    return RX_buffer[tail];
}

uint8_t serial_available()
{
  uint8_t tail = RX_buffer_tail;
  if (RX_buffer_head >= tail)
    return (RX_buffer_head - tail);
  return (RX_BUFFER_SIZE - tail + RX_buffer_head);
}
#endif

//----------------------------------------------------SERIAL_RX_NOPCKT-----------------------------------------------------
#if defined(SERIAL_RX_NOPCKT)

#define RX_BUFFER_SIZE 250

uint8_t RX_buffer[RX_BUFFER_SIZE];
volatile uint8_t RX_buffer_head = 0;
volatile uint8_t RX_buffer_tail = 0;

ISR(USART_RX_vect)
{
  uint8_t data = UDR0;
  uint8_t next_head;
  next_head = RX_buffer_head + 1;
  if (next_head == RX_BUFFER_SIZE)
    next_head = 0;

  if (next_head != RX_buffer_tail)
  {
    RX_buffer[RX_buffer_head] = data;
    RX_buffer_head = next_head;
  }
}

uint8_t serial_read()
{
  uint8_t tail = RX_buffer_tail;
  if (RX_buffer_head == tail)
    return EMPTY_BUFFER;
  else
  {
    uint8_t data = RX_buffer[tail];
    tail++;
    if (tail == RX_BUFFER_SIZE)
      tail = 0;
    RX_buffer_tail = tail;
    return data;
  }
}

uint8_t serial_peek()
{
  uint8_t tail = RX_buffer_tail;
  if (RX_buffer_head == tail)
    return EMPTY_BUFFER;
  else
    return RX_buffer[tail];
}

uint8_t serial_available()
{
  uint8_t tail = RX_buffer_tail;
  if (RX_buffer_head >= tail)
    return (RX_buffer_head - tail);
  return (RX_BUFFER_SIZE - tail + RX_buffer_head);
}

#endif

// --------------------------------------- TX ISR
ISR(USART_UDRE_vect)
{
  uint8_t tail = TX_buffer_tail;

  UDR0 = TX_buffer[tail];

  tail++;
  if (tail == TX_BUFFER_SIZE)
    tail = 0;

  TX_buffer_tail = tail;

  // ISR OFF
  if (tail == TX_buffer_head)
  {
    UCSR0B &= ~(1 << UDRIE0);
  }
}

void serial_init()
{
  crc = 0;

  uint16_t baud_setting = (F_CPU / 4L / BAUD_RATE - 1) / 2;
  UCSR0A = (1 << U2X0);
  UBRR0H = baud_setting >> 8;
  UBRR0L = baud_setting;

  // UCSR0C = (1<<UCSZ01) | (1<<UCSZ00);
  UCSR0B |= (1 << RXEN0 | 1 << TXEN0 | 1 << RXCIE0); // RX enable, TX enable, int. on RX enable
}

// ako je buffer prazan odmah pisi u UDR0 ! dodati
void serial_write(uint8_t data)
{
  uint8_t next_head = TX_buffer_head + 1;

  if (next_head == TX_BUFFER_SIZE)
    next_head = 0;

  // BUFFER FULL
  while (next_head == TX_buffer_tail);

  TX_buffer[TX_buffer_head] = data;
  TX_buffer_head = next_head;

  // ISR ENABLE
  UCSR0B |= (1 << UDRIE0);
}

void printString(const char *s)
{
  while (*s)
    serial_write(*s++);
}

void printChar(char c)
{
  serial_write(c);
}

void printInt(uint32_t n)
{
  if (n == 0)
  {
    serial_write('0');
    return;
  }

  char tmp[10];
  int8_t i = 0;
  while (n > 0)
  {
    tmp[i] = n % 10;
    n = n / 10;
    i++;
  }
  while (i)
    serial_write('0' + tmp[--i]);
}

void printInt(int32_t n)
{
  if (n < 0)
  {
    serial_write('-');
    printInt((uint32_t)-n);
  }
  else
  {
    printInt((uint32_t)n);
  }
}

void printInt(uint16_t n)
{
  printInt((uint32_t)n);
}

void printInt(int16_t n)
{
  printInt((int32_t)n);
}

void printInt(uint8_t n)
{
  printInt((uint32_t)n);
}

void printInt(int8_t n)
{
  printInt((int32_t)n);
}

void printFloat(float x, uint8_t dec) // no roundings
{
  if (x < 0)
  {
    printChar('-');
    x = x * (-1);
  }
  printInt((uint32_t)x);
  printChar('.');
  x = x - (uint32_t)x;
  for (int i = 0; i < dec; i++)
  {
    x = x * 10;
  }

  uint32_t d = x;

  char tmp[10];
  for (int i = 0; i < dec; i++)
  {
    tmp[i] = d % 10;
    d = d / 10;
  }
  while (dec)
    serial_write('0' + tmp[--dec]);
}
