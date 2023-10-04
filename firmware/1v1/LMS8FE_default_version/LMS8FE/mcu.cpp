#include "Arduino.h"
#include <Wire.h>
#include "consts.h"
#include "mcu.h"
#include "chain.h"

#include "pico/stdlib.h"
/*
// Registers
static const uint8_t REG_DEVID = 0x00;
static const uint8_t REG_POWER_CTL = 0x2D;
static const uint8_t REG_DATAX0 = 0x32;

// Other constants
static const uint8_t DEVID = 0xE5;
static const float SENSITIVITY_2G = 1.0 / 256;  // (g/LSB)
static const float EARTH_GRAVITY = 9.80665;     // Earth's gravity in [m/s^2]
*/

//void setupMCU(unsigned char value, unsigned char * currentState){
void setupMCU(unsigned char value){
  setRXTX(value);
//  memcpy(currentState + CHAIN_SIZE, &value, sizeof(value));
}

void setRXTX(unsigned char value){
  digitalWrite(TXRX_1_PIN, bitRead(value, MCU_TXRX_1_BIT));
  digitalWrite(TXRX_2_PIN, bitRead(value, MCU_TXRX_2_BIT));
}

//milans 220419
void setDiodeState( char state ) {
  if (state) {
//    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
    gpio_put(LED_BUILTIN, 1);
  } else {
//    digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
    gpio_put(LED_BUILTIN, 0);    
  }
}

/*******************************************************************************
 * Function Definitions
 */

// Write 1 byte to the specified register
void reg_write( spi_inst_t *spi, uint cs, uint8_t reg, uint8_t data) {

    uint8_t msg[2];
                
    // Construct message (set ~W bit low, MB bit low)
    msg[0] = 0x00 | reg;
    msg[1] = data;

    // Write to register
    gpio_put(cs, 0);
    spi_write_blocking(spi, msg, 2);
    gpio_put(cs, 1);
}

// Read byte(s) from specified register. If nbytes > 1, read from consecutive
// registers.
int reg_read( spi_inst_t *spi, uint cs, uint8_t reg, uint8_t *buf, uint8_t nbytes) {

    int num_bytes_read = 0;
    uint8_t mb = 0;

    // Determine if multiple byte (MB) bit should be set
    if (nbytes < 1) {
        return -1;
    } else if (nbytes == 1) {
        mb = 0;
    } else {
        mb = 1;
    }

    // Construct message (set ~W bit high)
    uint8_t msg = 0x80 | (mb << 6) | reg;

    // Read from register
    gpio_put(cs, 0);
    spi_write_blocking(spi, &msg, 1);
    num_bytes_read = spi_read_blocking(spi, 0, buf, nbytes);
    gpio_put(cs, 1);

    return num_bytes_read;
}
