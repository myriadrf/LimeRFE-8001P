#define DATA_OUT_RESETn_PIN 4
#define DATA_OUT_CLK_PIN 5
#define DATA_SER_CLK_PIN 6
#define DATA_SER_RESETn_PIN 7
#define DATA_PIN 8

#define TXRX_1_PIN 21
#define TXRX_2_PIN 22

#include "Arduino.h"
#include <Wire.h>
#include <SPI.h>
#include <pico/stdio.h>
#include "pico/stdlib.h"

//void setupMCU(unsigned char value, unsigned char * currentState);
void setupMCU(unsigned char value);
void setRXTX(unsigned char value);

//milans 220419
void setDiodeState(char state);

/*******************************************************************************
 * Function Declarations
 */
void reg_write(spi_inst_t *spi,
               uint cs,
               uint8_t reg,
               uint8_t data);

int reg_read(spi_inst_t *spi,
             uint cs,
             uint8_t reg,
             uint8_t *buf,
             uint8_t nbytes);