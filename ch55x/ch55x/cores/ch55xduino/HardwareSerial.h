#ifndef HardwareSerial_h
#define HardwareSerial_h

#include <stdint.h>
#include <stdio.h>
#if defined(__CH559__)
 #include "include/ch559.h"
#else
  #include "include/ch554.h"
#endif

#define SERIAL0_TX_BUFFER_SIZE 16
#define SERIAL0_RX_BUFFER_SIZE 16
#define SERIAL1_TX_BUFFER_SIZE 16
#define SERIAL1_RX_BUFFER_SIZE 16

uint8_t Serial0(void);
void Serial0_begin(unsigned long baud);

uint8_t Serial0_available(void);
uint8_t Serial0_read(void);
uint8_t Serial0_write(uint8_t c);
void Serial0_flush(void);

void Serial0_end(void);

void uart0IntRxHandler();
void uart0IntTxHandler();

#endif