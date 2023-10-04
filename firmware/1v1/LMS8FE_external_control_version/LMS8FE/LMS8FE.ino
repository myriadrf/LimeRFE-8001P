#include "mcu.h"
#include "consts.h"
#include "chain.h"
#include "cmd.h"
#include "errorCode.h"

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"

// milans 220616
#include "LMS64C_protocol.h"

#define SPIMASTER spi1
#define SPISLAVE spi0

#define SPIMASTER_RX 12
// #define SPIMASTER_CSN  13
#define SPIMASTER_CLK 14
#define SPIMASTER_TX 15

#define SPISLAVE_RX 0
#define SPISLAVE_CSN 1
#define SPISLAVE_CLK 2
#define SPISLAVE_TX 3

#define SC1905_ACK0 0x0F
#define SC1905_ACK1 0xF0

#define SC1905_RSR_READ_ATTEMPTS 200
#define SC1905_EEPROM_STATUS_ATTEMPTS 200

// milans 220615
// uint8_t activeBuffer[BUFFER_SIZE];
// uint8_t activeBufferSize = BUFFER_SIZE;
uint8_t activeBuffer[BUFFER_SIZE_LMS8001];
uint8_t activeBufferSize = BUFFER_SIZE_LMS8001;

// milans 220615
// uint8_t out_buf0[BUFFER_SIZE], in_buf0[BUFFER_SIZE];
// uint8_t out_buf1[BUFFER_SIZE], in_buf1[BUFFER_SIZE];
uint8_t out_buf0[BUFFER_SIZE_LMS8001], in_buf0[BUFFER_SIZE_LMS8001];
uint8_t out_buf1[BUFFER_SIZE_LMS8001], in_buf1[BUFFER_SIZE_LMS8001];

unsigned char activeState[STATE_SIZE];
unsigned char nextState[STATE_SIZE];

unsigned char activeChannel;

// int periphID;

unsigned char diodeState = 0;
/************   LMS8001 functions - Start   ************/
#define TRUE 1
#define FALSE 0

uint8_t activeBufferLMS8001tx[BUFFER_SIZE_LMS8001];

tLMS_Ctrl_Packet *LMS_Ctrl_Packet_Tx = (tLMS_Ctrl_Packet *)activeBufferLMS8001tx;
tLMS_Ctrl_Packet *LMS_Ctrl_Packet_Rx = (tLMS_Ctrl_Packet *)activeBuffer;

uint8_t block, cmd_errors;

uint16_t maddress = 0x00AA;
uint16_t maddress2 = 0x00AB;

// B.J.
uint8_t BuffLMS8FE[64];
uint8_t RegLMS8FE[2];
uint8_t RDbyteX[128]; // for debugging
int wrPtr = 0;        // for debugging
int rdPtr = 0;        // for debugging
uint8_t spi_state = 0;
uint16_t maddr = 0x00;
uint16_t faddr = 0x00;
uint16_t w_nr = 0x00;
uint8_t FlagX = 0;
uint8_t X10, X1 = 0;
uint16_t src = 0x0000;
uint16_t dst = 0x0000;
// end B.J.

uint8_t SPI1_transfer_byte(uint8_t cmd)
{
  uint8_t result;
  /*
      while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET); //Wait until the transmit buffer is empty
      SPI_I2S_SendData(SPI1, cmd);
      //while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_BSY) == SET); //wait finish sending

      while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET); // wait data received

      result = SPI_I2S_ReceiveData(SPI1);
  */

  spi_write_read_blocking(SPIMASTER, &cmd, &result, 1);

  return result;
}

/**  This function checks if all blocks could fit in data field.
  If blocks will not fit, function returns TRUE. */
unsigned char Check_many_blocks(unsigned char block_size)
{
  if (LMS_Ctrl_Packet_Rx->Header.Data_blocks > (sizeof(LMS_Ctrl_Packet_Tx->Data_field) / block_size))
  {
    LMS_Ctrl_Packet_Tx->Header.Status = STATUS_BLOCKS_ERROR_CMD;
    return TRUE;
  }
  else
    return FALSE;
  return FALSE;
}
/************   LMS8001 functions - End   ************/

void cs_setval(int periphID, uint8_t val)
{
  uint8_t targetByte;
  uint8_t targetBit;

  switch (periphID)
  {
  case PERIPH_SC1905:
    if (activeChannel == 0)
    {
      targetByte = SC1905_1_SSENn_BYTE;
      targetBit = SC1905_1_SSENn_BIT;
    }
    else
    {
      targetByte = SC1905_2_SSENn_BYTE;
      targetBit = SC1905_2_SSENn_BIT;
    }
    break;
  case PERIPH_LMS8001:
    if (activeChannel == 0)
    {
      targetByte = LMS8001_1_SSENn_BYTE;
      targetBit = LMS8001_1_SSENn_BIT;
    }
    else
    {
      targetByte = LMS8001_2_SSENn_BYTE;
      targetBit = LMS8001_2_SSENn_BIT;
    }
    break;
  case PERIPH_ADF4002:
    targetByte = EXT_PLL_SSENn_BYTE;
    targetBit = EXT_PLL_SSENn_BIT;
    break;
  default:
    // Implement error handling
    return;
  }
  setChainBit(targetByte, targetBit, val, activeState, nextState);
  setState(nextState);
}

void cs_select(int periphID)
{
  // Here implement the CSEN = 0 for the periphery provided as parameter
  // This is just a temporary solution
  //    asm volatile("nop \n nop \n nop");
  //    gpio_put(SPIMASTER_CSN, 0);  // Active low
  //    asm volatile("nop \n nop \n nop");
  cs_setval(periphID, 0);
}

void cs_deselect(int periphID)
{
  // Here implement the CSEN = 1 for the periphery provided as parameter
  // This is just a temporary solution
  //    asm volatile("nop \n nop \n nop");
  //    gpio_put(SPIMASTER_CSN, 1);
  //    asm volatile("nop \n nop \n nop");
  cs_setval(periphID, 1);
}

/*
  static inline void cs_select() {
    asm volatile("nop \n nop \n nop");
    gpio_put(SPIMASTER_CSN, 0);  // Active low
    asm volatile("nop \n nop \n nop");
  }

  static inline void cs_deselect() {
    asm volatile("nop \n nop \n nop");
    gpio_put(SPIMASTER_CSN, 1);
    asm volatile("nop \n nop \n nop");
  }
*/

int loadenbSC1905pin()
{
  if (activeChannel == 0)
    return LOADENB_SC1905_1_PIN;
  else
    return LOADENB_SC1905_2_PIN;
}

void test_irq_handler(uint gpio, uint32_t events)
{
  gpio_set_irq_enabled(29, GPIO_IRQ_EDGE_FALL, false);

  // B.J.
  FlagX = 1;

  gpio_set_irq_enabled(29, GPIO_IRQ_EDGE_FALL, true);
}

int sc1905_message_protocol(uint8_t *mrb, uint8_t *mrb_rcv)
{

  uint8_t lastRSR = 0x00;

  // RSR Read Command
  out_buf0[0] = 0xC8;
  out_buf0[1] = 0x00;
  out_buf0[2] = 0x28;
  out_buf0[3] = 0x00;

  cs_select(PERIPH_SC1905);
  spi_write_read_blocking(SPIMASTER, out_buf0, in_buf0, 4);
  cs_deselect(PERIPH_SC1905);
  sleep_ms(10);

  lastRSR = in_buf0[3];

  if (!((lastRSR == 0x00) || (lastRSR == SC1905_ACK0) || (lastRSR == SC1905_ACK1)))
  {
    return ERROR_SC1905_RSR;
  }

  //  uint8_t mrb[4];

  uint8_t bytesCode = 0x00;

  uint8_t chk = ~((mrb[0] + mrb[1] + mrb[2] + mrb[3]) % 256);

  // CHK Write Command
  clearBuffer(out_buf0);
  out_buf0[0] = 0xD5;
  out_buf0[1] = 0x81;
  out_buf0[2] = 0x20;
  out_buf0[3] = chk;

  cs_select(PERIPH_SC1905);
  spi_write_read_blocking(SPIMASTER, out_buf0, in_buf0, 4);
  cs_deselect(PERIPH_SC1905);
  sleep_ms(10);

  // MRB Write Command
  clearBuffer(out_buf0);
  out_buf0[0] = 0xF0;
  out_buf0[1] = 0x00;
  out_buf0[2] = 0x20;
  out_buf0[3] = mrb[0];
  out_buf0[4] = mrb[1];
  out_buf0[5] = mrb[2];
  out_buf0[6] = mrb[3];

  cs_select(PERIPH_SC1905);
  spi_write_read_blocking(SPIMASTER, out_buf0, in_buf0, 7);
  cs_deselect(PERIPH_SC1905);
  sleep_ms(10);

  int i = 0;

  // RSR Read Command
  clearBuffer(out_buf0);
  out_buf0[0] = 0xC8;
  out_buf0[1] = 0x00;
  out_buf0[2] = 0x28;

  bool success = false;
  while (i < SC1905_RSR_READ_ATTEMPTS)
  {
    cs_select(PERIPH_SC1905);
    spi_write_read_blocking(SPIMASTER, out_buf0, in_buf0, 4);
    cs_deselect(PERIPH_SC1905);

    uint8_t newRSR = in_buf0[3];

    if (((newRSR == SC1905_ACK0) || (newRSR == SC1905_ACK1)) && (newRSR != lastRSR))
    {
      lastRSR = newRSR;
      success = true;
      break;
    }

    sleep_ms(5);
    i++;
  }

  if (!success)
  {
    return ERROR_SC1905_RSR;
  }

  // MRB Read Command
  clearBuffer(out_buf0);
  out_buf0[0] = 0xF0;
  out_buf0[1] = 0x00;
  out_buf0[2] = 0x28;

  cs_select(PERIPH_SC1905);
  spi_write_read_blocking(SPIMASTER, out_buf0, in_buf0, 7);
  cs_deselect(PERIPH_SC1905);
  sleep_ms(10);

  mrb_rcv[0] = in_buf0[3];
  mrb_rcv[1] = in_buf0[4];
  mrb_rcv[2] = in_buf0[5];
  mrb_rcv[3] = in_buf0[6];

  // CHK Read Command
  clearBuffer(out_buf0);
  out_buf0[0] = 0xD5;
  out_buf0[1] = 0x81;
  out_buf0[2] = 0x28;

  cs_select(PERIPH_SC1905);
  spi_write_read_blocking(SPIMASTER, out_buf0, in_buf0, 4);
  cs_deselect(PERIPH_SC1905);
  sleep_ms(10);

  chk = in_buf0[3];

  uint8_t read_chk = ~((mrb_rcv[0] + mrb_rcv[1] + mrb_rcv[2] + mrb_rcv[3] + lastRSR) % 256);

  if (chk != read_chk)
  {
    return ERROR_SC1905_CHK;
  }

  return COMPLETED;
}

void sc1905_Reset()
{
  //  gpio_put(RESETN_SC1904_PIN, 0);
  //  sleep_ms(100);
  //  gpio_put(RESETN_SC1904_PIN, 1);

  uint8_t targetByte;
  uint8_t targetBit;

  if (activeChannel == 0)
  {
    targetByte = SC1905_1_RESETn_BYTE;
    targetBit = SC1905_1_RESETn_BIT;
  }
  else
  {
    targetByte = SC1905_2_RESETn_BYTE;
    targetBit = SC1905_2_RESETn_BIT;
  }

  setChainBit(targetByte, targetBit, 0, activeState, nextState);
  setState(nextState);
  sleep_ms(100);
  setChainBit(targetByte, targetBit, 1, activeState, nextState);
  setState(nextState);
}

int sc1905_EEPROM_Read(uint8_t *cmd_buf)
{
  uint8_t bytesNo = cmd_buf[4];
  clearBuffer(out_buf0);
  out_buf0[0] = 0x03;
  out_buf0[1] = cmd_buf[2];
  out_buf0[2] = cmd_buf[3];
  //    gpio_put(LOADENB_SC1904_1_PIN, 1);
  gpio_put(loadenbSC1905pin(), 1);

  cs_select(PERIPH_SC1905);
  spi_write_read_blocking(SPIMASTER, out_buf0, in_buf0, 3 + bytesNo);
  cs_deselect(PERIPH_SC1905);
  sleep_ms(10);

  //    gpio_put(LOADENB_SC1904_1_PIN, 0);
  gpio_put(loadenbSC1905pin(), 0);

  cmd_buf[1] = in_buf0[3];
  if (bytesNo == 2)
    cmd_buf[2] = in_buf0[4];

  return COMPLETED;
}

int sc1905_EEPROM(uint8_t *cmd_buf)
{
  uint8_t isRead = cmd_buf[1];

  if (isRead == 1)
  {
    // Read
    sc1905_EEPROM_Read(cmd_buf);
    /*
        uint8_t bytesNo = cmd_buf[4];
        clearBuffer(out_buf0);
        out_buf0[0] = 0x03;
        out_buf0[1] = cmd_buf[2];
        out_buf0[2] = cmd_buf[3];
    //    gpio_put(LOADENB_SC1904_1_PIN, 1);
        gpio_put(loadenbSC1905pin(), 1);

        cs_select(PERIPH_SC1905);
        spi_write_read_blocking(SPIMASTER, out_buf0, in_buf0, 3 + bytesNo);
        cs_deselect(PERIPH_SC1905);
        sleep_ms(10);

    //    gpio_put(LOADENB_SC1904_1_PIN, 0);
        gpio_put(loadenbSC1905pin(), 0);

        cmd_buf[1] = in_buf0[3];
        if (bytesNo == 2)
          cmd_buf[2] = in_buf0[4];
    */
  }
  else
  {
    // Write
    uint8_t tmp_buf[BUFFER_SIZE];
    // Read original values, will be needed for checksum calculation
    memcpy(tmp_buf, cmd_buf, sizeof(cmd_buf[0]) * BUFFER_SIZE);
    sc1905_EEPROM_Read(tmp_buf);
    uint8_t original_values[2];
    original_values[0] = tmp_buf[1];
    original_values[1] = tmp_buf[2];
    // Read original checksum, will be needed for checksum calculation
    tmp_buf[2] = 0xFF; // Checksum address is 0xFFFF
    tmp_buf[3] = 0xFF;
    tmp_buf[4] = 1; // Read single byte
    sc1905_EEPROM_Read(tmp_buf);
    uint8_t original_checksum = tmp_buf[1];

    // Write
    //    gpio_put(LOADENB_SC1904_1_PIN, 1);
    gpio_put(loadenbSC1905pin(), 1);

    //    clearBuffer(out_buf0);
    //    out_buf0[0] = 0x06; // WREN command to enable write operations to EEPROM
    //    out_buf0[1] = 0x01; // Write zero to STATUS register to unlock
    //    out_buf0[2] = 0x00;
    //    out_buf0[3] = 0x05; // Read STATUS register to make sure EEPROM is UNLOCKED

    //    cs_select();
    //    spi_write_read_blocking(SPIMASTER, out_buf0, in_buf0, 5);
    //    cs_deselect();
    //    sleep_ms(10);

    //    uint8_t statusRegister = in_buf0[4];

    clearBuffer(out_buf0);
    out_buf0[0] = 0x06; // WREN command to enable write operations to EEPROM

    cs_select(PERIPH_SC1905);
    spi_write_read_blocking(SPIMASTER, out_buf0, in_buf0, 1);
    cs_deselect(PERIPH_SC1905);
    sleep_ms(10);

    clearBuffer(out_buf0);
    out_buf0[0] = 0x01; // Write zero to STATUS register to unlock
    out_buf0[1] = 0x00;

    cs_select(PERIPH_SC1905);
    spi_write_read_blocking(SPIMASTER, out_buf0, in_buf0, 2);
    cs_deselect(PERIPH_SC1905);
    sleep_ms(10);

    // milans 220809 TMP TMP TMP TMP TMP
    //     sleep_ms(1000);

    clearBuffer(out_buf0);
    out_buf0[0] = 0x05; // Read STATUS register to make sure EEPROM is UNLOCKED

    cs_select(PERIPH_SC1905);
    spi_write_read_blocking(SPIMASTER, out_buf0, in_buf0, 2);
    cs_deselect(PERIPH_SC1905);
    sleep_ms(10);

    uint8_t statusRegister = in_buf0[1];

    uint8_t wip = statusRegister & 0x01;
    bool isBusy = (wip == 1);
    uint8_t bpi01 = statusRegister & 0x0c;
    bool isLocked = (bpi01 != 0);

    if (isLocked)
    {
      cmd_buf[1] = statusRegister;

      // milans 220809
      gpio_put(loadenbSC1905pin(), 0);

      return ERROR_SC1905_EEPROM_LOCKED;
    }

    if (isBusy)
    {
      cmd_buf[1] = statusRegister;

      // milans 220809
      gpio_put(loadenbSC1905pin(), 0);

      return ERROR_SC1905_EEPROM_BUSY;
    }

    // Write
    clearBuffer(out_buf0);
    out_buf0[0] = 0x06; // WREN command to enable write operations to EEPROM

    cs_select(PERIPH_SC1905);
    spi_write_read_blocking(SPIMASTER, out_buf0, in_buf0, 1);
    cs_deselect(PERIPH_SC1905);
    sleep_ms(10);

    clearBuffer(out_buf0);
    out_buf0[0] = 0x02;
    out_buf0[1] = cmd_buf[2]; // Address
    out_buf0[2] = cmd_buf[3];
    out_buf0[3] = cmd_buf[5]; // Value
    out_buf0[4] = cmd_buf[6];

    uint8_t bytesNo = cmd_buf[4];

    cs_select(PERIPH_SC1905);
    spi_write_read_blocking(SPIMASTER, out_buf0, in_buf0, 3 + bytesNo);
    cs_deselect(PERIPH_SC1905);
    sleep_ms(10);

    /************************ EEPROM Checksum ************************/
    // Calculate checksum
    uint8_t checksum;
    int int_checksum = original_checksum;

    int_checksum += ((256 - original_values[0]) + cmd_buf[5]);
    if (bytesNo == 2)
      int_checksum += ((256 - original_values[1]) + cmd_buf[6]);

    checksum = int_checksum % 256;

    // Write checksum
    clearBuffer(out_buf0);
    out_buf0[0] = 0x06; // WREN command to enable write operations to EEPROM

    cs_select(PERIPH_SC1905);
    spi_write_read_blocking(SPIMASTER, out_buf0, in_buf0, 1);
    cs_deselect(PERIPH_SC1905);
    sleep_ms(10);

    clearBuffer(out_buf0);
    out_buf0[0] = 0x02;
    out_buf0[1] = 0xFF; // Address
    out_buf0[2] = 0xFF;
    out_buf0[3] = checksum; // Value

    bytesNo = 1;

    cs_select(PERIPH_SC1905);
    spi_write_read_blocking(SPIMASTER, out_buf0, in_buf0, 3 + bytesNo);
    cs_deselect(PERIPH_SC1905);
    sleep_ms(10);

    /*****************************************************************/

    int i = 0;

    clearBuffer(out_buf0);
    out_buf0[0] = 0x05; // Read STATUS register to make sure EEPROM is LOCKED

    bool success = false;
    while (i < SC1905_RSR_READ_ATTEMPTS)
    {
      cs_select(PERIPH_SC1905);
      spi_write_read_blocking(SPIMASTER, out_buf0, in_buf0, 2);
      cs_deselect(PERIPH_SC1905);

      statusRegister = in_buf0[1];

      wip = statusRegister & 0x01;
      isBusy = (wip == 1);

      if (!isBusy)
      {
        success = true;
        break;
      }

      sleep_ms(5);
      i++;
    }

    if (!success)
    {
      cmd_buf[1] = 0x02;

      // milans 220809
      gpio_put(loadenbSC1905pin(), 0);

      return ERROR_SC1905_EEPROM_BUSY;
    }

    clearBuffer(out_buf0);
    out_buf0[0] = 0x06; // WREN command to enable write operations to EEPROM

    cs_select(PERIPH_SC1905);
    spi_write_read_blocking(SPIMASTER, out_buf0, in_buf0, 1);
    cs_deselect(PERIPH_SC1905);
    sleep_ms(10);

    clearBuffer(out_buf0);
    out_buf0[0] = 0x01; // Write "0C" to STATUS register to lock
    out_buf0[1] = 0x0C;

    cs_select(PERIPH_SC1905);
    spi_write_read_blocking(SPIMASTER, out_buf0, in_buf0, 2);
    cs_deselect(PERIPH_SC1905);
    sleep_ms(10);

    clearBuffer(out_buf0);
    out_buf0[0] = 0x05;

    cs_select(PERIPH_SC1905);
    spi_write_read_blocking(SPIMASTER, out_buf0, in_buf0, 2);
    cs_deselect(PERIPH_SC1905);
    sleep_ms(10);

    statusRegister = in_buf0[1];

    bpi01 = statusRegister & 0x0c;
    isLocked = (bpi01 != 0);

    if (!isLocked)
      return ERROR_SC1905_EEPROM_UNLOCKED;

    //    gpio_put(LOADENB_SC1904_1_PIN, 0);
    gpio_put(loadenbSC1905pin(), 0);

    sc1905_Reset();
  }

  return COMPLETED;
}

void setState(unsigned char *state)
{
  shiftData(state);
  setupMCU(state[MCU_BYTE]);
  activeChannel = bitRead(state[MISC_BYTE], MISC_CHANNEL_BIT);
  memcpy(activeState, state, sizeof(state[0]) * STATE_SIZE);
}

void resetState()
{
  memset(activeState, 0, sizeof(activeState[0]) * STATE_SIZE);
  // activeState[0] = 0xCE;
/*  
  activeState[0] = 0xCF;
  activeState[1] = 0x00;
  activeState[2] = 0x05; // Bypass AMP1 and AMP2
  activeState[3] = 0x00;
  activeState[4] = 0xfc; // Channel 1 Observation Path Attenuation max
  activeState[5] = 0xff; // Channel 2 Observation Path Attenuation max
  activeState[6] = 0x00;
  activeState[7] = 0x00;
*/
// milans 221229 - Version 1v1 update  
  activeState[0] = 0xCF;
  activeState[1] = 0x00;
  activeState[2] = 0x05; // Bypass AMP1 and AMP2
  activeState[3] = 0x00;
  activeState[4] = 0xFE; // Channel 1 Observation Path Attenuation max
  activeState[5] = 0xFE; // Channel 2 Observation Path Attenuation max
  activeState[6] = 0xFE; // Channel 1 TX Path Attenuation max
  activeState[7] = 0xFE; // Channel 2 TX Path Attenuation max
      
  setState(activeState);
  //  shiftData(activeState, activeState);
  //  setupMCU(activeState[MCU_BYTE]);
}

/*************************************************************************/
/*****************************   S E T U P   *****************************/
/*************************************************************************/

void setup()
{

  // Initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
  // Serial.begin(115200);

  // Configure pins
  gpio_init(LED_BUILTIN);
  gpio_set_dir(LED_BUILTIN, GPIO_OUT);

  // Temporarily solution, only for test
  // This pin is used as a reset pin for SC1905
  // The final LMS8FE will have this signal inside the shift registers
  //  gpio_init(RESETN_SC1904_PIN);
  //  gpio_set_dir(RESETN_SC1904_PIN, GPIO_OUT);
  //  gpio_put(RESETN_SC1904_PIN, 1);

  gpio_init(LOADENB_SC1905_1_PIN);
  gpio_set_dir(LOADENB_SC1905_1_PIN, GPIO_OUT);
  gpio_put(LOADENB_SC1905_1_PIN, 0);

  gpio_init(LOADENB_SC1905_2_PIN);
  gpio_set_dir(LOADENB_SC1905_2_PIN, GPIO_OUT);
  gpio_put(LOADENB_SC1905_2_PIN, 0);

  //  gpio_init(RESETN_LMS8001_PIN);
  //  gpio_set_dir(RESETN_LMS8001_PIN, GPIO_OUT);
  //  gpio_put(RESETN_LMS8001_PIN, 1);

  /***** Shift register control *****/

  gpio_init(DATA_OUT_RESETn_PIN);
  gpio_set_dir(DATA_OUT_RESETn_PIN, GPIO_OUT);
  gpio_put(DATA_OUT_RESETn_PIN, 1);

  gpio_init(DATA_OUT_CLK_PIN);
  gpio_set_dir(DATA_OUT_CLK_PIN, GPIO_OUT);
  gpio_put(DATA_OUT_CLK_PIN, 0);

  gpio_init(DATA_SER_CLK_PIN);
  gpio_set_dir(DATA_SER_CLK_PIN, GPIO_OUT);
  gpio_put(DATA_SER_CLK_PIN, 0);

  gpio_init(DATA_SER_RESETn_PIN);
  gpio_set_dir(DATA_SER_RESETn_PIN, GPIO_OUT);
  gpio_put(DATA_SER_RESETn_PIN, 1);

  gpio_init(DATA_PIN);
  gpio_set_dir(DATA_PIN, GPIO_OUT);
  gpio_put(DATA_PIN, 0);

  resetData();

  /***** TX/RX *****/

  /************************************************************************/
  // I M P O R T A N T
  // This is the control of the TX/RX from the MCU
  // In case that external control is to be provided (normal application)
  // then these pins should be configured as inputs
  /************************************************************************/
  /*gpio_init(TXRX_1_PIN);
  gpio_set_dir(TXRX_1_PIN, GPIO_OUT);
  gpio_put(TXRX_1_PIN, 0);

  gpio_init(TXRX_2_PIN);
  gpio_set_dir(TXRX_2_PIN, GPIO_OUT);
  gpio_put(TXRX_2_PIN, 0);*/

  //branko 2301045
  //this is when TXRX1 and TXRX2 pins are not GPIO pins
  gpio_pull_up(TXRX_1_PIN);
  gpio_pull_up(TXRX_2_PIN);

  /************************************************************************/
  ////milans 220714
  ////memset(activeState, 0, sizeof activeState);
  // memset(activeState, 0, sizeof(activeState[0])*CHAIN_SIZE);
  // shiftData(activeState, activeState);  // Shift Chain Data
  resetState();

  // setDiodeState(1);
  // delay(1000);
  // setDiodeState(0);

  spi_init(SPIMASTER, 1000 * 1000);
  //    spi_init(SPIMASTER, 10 * 1000);

  gpio_set_function(SPIMASTER_RX, GPIO_FUNC_SPI);
  //    gpio_set_function(SPIMASTER_CSN, GPIO_FUNC_SPI);
  gpio_set_function(SPIMASTER_CLK, GPIO_FUNC_SPI);
  gpio_set_function(SPIMASTER_TX, GPIO_FUNC_SPI);

  //  // Chip select is active-low, so we'll initialise it to a driven-high state
  //  gpio_init(SPIMASTER_CSN);
  //  gpio_set_dir(SPIMASTER_CSN, GPIO_OUT);
  //  gpio_put(SPIMASTER_CSN, 1);

  // B.J.  SPI slave configuration
  spi_init(SPISLAVE, 1250 * 1000);
  spi_set_slave(SPISLAVE, true);
  spi_set_format(SPISLAVE, 16, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);

  gpio_set_function(SPISLAVE_RX, GPIO_FUNC_SPI);
  gpio_set_function(SPISLAVE_CSN, GPIO_FUNC_SPI);
  gpio_set_function(SPISLAVE_CLK, GPIO_FUNC_SPI);
  gpio_set_function(SPISLAVE_TX, GPIO_FUNC_SPI);

  gpio_set_irq_enabled_with_callback(SPISLAVE_CSN, GPIO_IRQ_EDGE_FALL, true, &test_irq_handler);
  
  for (int i=0; i<64; i++) BuffLMS8FE[i] = 0x00;
  RegLMS8FE[0]=0x00;
  RegLMS8FE[1]=0x00;



}

/*************************************************************************/
/******************************   L O O P   ******************************/
/*************************************************************************/

void loop()
{
  if (Serial.available())
  {

    Serial.readBytes(activeBuffer, 1);
    if (activeBuffer[0] == CMD_LMS8FE_HELLO)
    {
      while (Serial.read() >= 0)
      {
      } // flush the receive buffer
      Serial.write((byte)CMD_LMS8FE_HELLO);
      //      setDiodeState(0);
      delay(100);
    }
    else
    {
      uint8_t *tx_buf;
      unsigned char command;
      command = activeBuffer[0];

      if (command >= LMS8001_CMD_MASK)
      {
        //        periphID = PERIPH_LMS8001_1; // Implement possibility to choose PERIPH_LMS8001_1
        cmd_errors = 0;
        activeBufferSize = BUFFER_SIZE_LMS8001;
        tx_buf = activeBufferLMS8001tx;
        LMS_Ctrl_Packet_Tx->Header.Command = LMS_Ctrl_Packet_Rx->Header.Command;
        LMS_Ctrl_Packet_Tx->Header.Data_blocks = LMS_Ctrl_Packet_Rx->Header.Data_blocks;
        LMS_Ctrl_Packet_Tx->Header.Status = STATUS_BUSY_CMD;
      }
      else
      {
        activeBufferSize = BUFFER_SIZE;
        tx_buf = activeBuffer;
      }

      wait_for_bytes(activeBufferSize - 1, 1000);
      Serial.readBytes(activeBuffer + 1, activeBufferSize - 1);
      activeBuffer[0] = doCommand(activeBuffer);
      //      activeBuffer[0] = command;

      //      Serial.write(activeBuffer,activeBufferSize);
      Serial.write(tx_buf, activeBufferSize);
    }
  }

   // B.J.
  if (FlagX == 1)
  {
    // interrupt on CSN happened, receive the 16-bit data from SPI
    FlagX = 0;
    MyFunction();
  }

  // B.J.:
  if (rdPtr != wrPtr)
  { // for debugging, does not happen in normal operation
    X10 = 0;
    X1 = 0;
    ConvertToAscii(RDbyteX[rdPtr], &X10, &X1);
    Serial.write(X10);
    Serial.write(X1);
    if (rdPtr < 127)
      rdPtr++;
    else
      rdPtr = 0;
    ConvertToAscii(RDbyteX[rdPtr], &X10, &X1);
    Serial.write(X10);
    Serial.write(X1);
    if (rdPtr < 127)
      rdPtr++;
    else
      rdPtr = 0;
    Serial.write('\n');
  }

  if (RegLMS8FE[0] == 1)
  { // flag set?

    RegLMS8FE[1] = 0x00;
    if (BuffLMS8FE[0] == CMD_LMS8FE_HELLO)
    {
      // BuffLMS8FE[0] = CMD_LMS8FE_HELLO; // not change
      RegLMS8FE[1] = 1; // packet length in bytes = 1
      //delay(100);
    }
    else
    {
      uint8_t command = 0x00;
      uint8_t m_bLMS8001 = 0x00;
      command = BuffLMS8FE[0];

      for (int i = 0; i< BUFFER_SIZE_LMS8001; i++)
           activeBuffer[i] = BuffLMS8FE[i];

      if (command >= LMS8001_CMD_MASK)
      {
        // periphID = PERIPH_LMS8001_1; // Implement possibility to choose PERIPH_LMS8001_1
        cmd_errors = 0; // ???
        // activeBufferSize = BUFFER_SIZE_LMS8001;
        // tx_buf = BuffLMS8FE;
        m_bLMS8001 = 0x01;      
        activeBufferSize = BUFFER_SIZE_LMS8001;
        RegLMS8FE[1] = BUFFER_SIZE_LMS8001; // packet length in bytes = 64
        LMS_Ctrl_Packet_Tx->Header.Command = LMS_Ctrl_Packet_Rx->Header.Command;
        LMS_Ctrl_Packet_Tx->Header.Data_blocks = LMS_Ctrl_Packet_Rx->Header.Data_blocks;
        LMS_Ctrl_Packet_Tx->Header.Status = STATUS_BUSY_CMD;
      }
      else
      {
        // activeBufferSize = BUFFER_SIZE;
        // tx_buf = BuffLMS8FE;
        activeBufferSize = BUFFER_SIZE;
        RegLMS8FE[1] = BUFFER_SIZE; // packet length in bytes = 16
      }
      activeBuffer[0] = doCommand(activeBuffer);

      // for transmit
      if (m_bLMS8001 == 0x01) {
        for (int i = 0; i< BUFFER_SIZE_LMS8001; i++)
            BuffLMS8FE[i] = activeBufferLMS8001tx[i];
      }
      else  {
        for (int i = 0; i< BUFFER_SIZE_LMS8001; i++)
            BuffLMS8FE[i] = activeBuffer[i];
      }
    }
    //RegLMS8FE[0] = 0; // not in progress now, clear flag
    RegLMS8FE[0] = 0xAA; // not in progress now, clear flag
    // start of data packet, required for synchronization
  }
}
// B.J.: for debugging, not happen in normal operation
void ConvertToAscii(uint8_t ch, uint8_t *pX10, uint8_t *pX1)
{

  uint8_t X10, X1 = 0;

  X10 = ((ch & 0xF0) >> 4);
  X1 = (ch & 0x0F);

  if (X10 > 9)
    X10 = X10 + 0x37; // A-F
  else
    X10 = X10 + 0x30; // 0-9

  if (X1 > 9)
    X1 = X1 + 0x37;
  else
    X1 = X1 + 0x30;

  *pX10 = X10;
  *pX1 = X1;
}

/*************************************************************************/
/************************** D O   C O M M A N D **************************/
/*************************************************************************/

int doCommand(uint8_t *cmd_buf)
{

  int result = COMPLETED;

  switch (cmd_buf[0])
  {

  case CMD_LMS8FE_GET_INFO:
  {
    clearBuffer(cmd_buf);
    //        cmd_buf[1] = 0x01; // FW_VER
    cmd_buf[1] = 0x02; // FW_VER
    cmd_buf[2] = 0x01; // HW_VER
    cmd_buf[3] = 1;    // Status
    cmd_buf[4] = 1;    // Status
    break;
  }

    //    case CMD_LMS8FE_DIODE: {
    //        setDiodeState(cmd_buf[1]);
    //        break;
    //      }

  // milans 220714
  case CMD_LMS8FE_RESET:
  {
    //        setDiodeState(cmd_buf[1]);
    //        clearBuffer();
    //        memset(activeState, 0, sizeof(activeState[0])*CHAIN_SIZE);
    //        shiftData(activeState, activeState);  // Shift Chain Data
    resetState();
    break;
  }
    // milans 221128
  case CMD_LMS8FE_LMS8_ENABLE:
  {
    uint8_t val = cmd_buf[1];
    //        setChainBit(LMS8001_1_RESETn_BYTE, LMS8001_1_RESETn_BIT, val, activeState, nextState);
    //        setChainBit(LMS8001_2_RESETn_BYTE, LMS8001_2_RESETn_BIT, val, nextState, nextState);
    //        setChainBit(LMS8001_1_RESETn_BYTE, LMS8001_1_RESETn_BIT, 0, activeState, nextState);
    //        setState(nextState);
    //        setChainBit(LMS8001_2_RESETn_BYTE, LMS8001_2_RESETn_BIT, 1, nextState, nextState);
    //        setChainBit(LMS8001_2_RESETn_BYTE, LMS8001_2_RESETn_BIT, 1, activeState, nextState);
    //        setState(nextState);

    setChainBit(LMS8001_1_RESETn_BYTE, LMS8001_1_RESETn_BIT, val, activeState, nextState);
    setChainBit(LMS8001_2_RESETn_BYTE, LMS8001_2_RESETn_BIT, val, nextState, nextState);
    setState(nextState);

    break;
  }
    // milans 221130
  case CMD_LMS8FE_SELECT_CHANNEL:
  {
    uint8_t val = cmd_buf[1];
    setChainBit(MISC_BYTE, MISC_CHANNEL_BIT, val, activeState, nextState);
    setState(nextState);
    break;
  }

    /*
      case CMD_LMS8FE_DIODESPI: {
          setDiodeState(1);
          delay(100);
          setDiodeState(0);
          delay(100);
          setDiodeState(1);
          delay(100);
          setDiodeState(0);

          out_buf0[0] = CMD_LMS8FE_DIODESPI;
          out_buf0[1] = cmd_buf[1];

          // Write the output buffer to MOSI, and at the same time read from MISO.
          spi_write_read_blocking(SPIMASTER, out_buf0, in_buf0, BUFFER_SIZE);

          break;
        }
  */
  case CMD_LMS8FE_SET_CONFIG_FULL:
  {
    //        clearBuffer();
    //        shiftData(cmd_buf + 1, activeState);  // Shift Chain Data
    setState(cmd_buf + 1);
    //        memcpy(activeState, cmd_buf + 1, sizeof(activeState[0])*CHAIN_SIZE);
    //        activeState[0] = cmd_buf[1];

    //        setupMCU(activeBuffer[CHAIN_SIZE + 1], activeState);  // Setup control signals directly from MCU
    //        setupMCU(cmd_buf[MCU_BYTE], activeState);  // Setup control signals directly from MCU
    //        setupMCU(cmd_buf[MCU_BYTE]);  // Setup control signals directly from MCU
    break;
  }

  case CMD_LMS8FE_GET_CONFIG_FULL:
  {
    clearBuffer(cmd_buf);
    memcpy(cmd_buf + 1, activeState, sizeof(activeState[0]) * STATE_SIZE);
    //        cmd_buf[1] = 0x89;
    break;
  }

  case CMD_SC1905_SPI_SPECIAL_COMMAND:
  {
    //      setDiodeState(1);
    //      delay(100);
    //      setDiodeState(0);

    uint8_t mrb[4];
    uint8_t mrb_rcv[4];

    uint8_t bytesCode = 0x00;

    mrb[0] = cmd_buf[1];
    mrb[1] = cmd_buf[2];
    mrb[2] = cmd_buf[3];
    mrb[3] = cmd_buf[4];

    result = sc1905_message_protocol(mrb, mrb_rcv);

    if (result != COMPLETED)
      return result;

    cmd_buf[1] = mrb_rcv[0];
    cmd_buf[2] = mrb_rcv[1];
    cmd_buf[3] = mrb_rcv[2];
    cmd_buf[4] = mrb_rcv[3];

    break;
  }

  case CMD_SC1905_SPI_MESSAGE_MEMORY:
  {
    //      setDiodeState(1);
    //      delay(100);
    //      setDiodeState(0);

    result = COMPLETED;

    uint8_t isEEPROM = cmd_buf[7];

    if (isEEPROM == 1)
    {
      result = sc1905_EEPROM(cmd_buf);
      if (result != COMPLETED)
        return result;
      break;
    }

    uint8_t mrb[4];
    uint8_t mrb_rcv[4];

    uint8_t bytesCode = 0x00;

    if (cmd_buf[1] == 1)
    { // read
      bytesCode = 0x04;
      if (cmd_buf[4] == 2)
        bytesCode = 0x06;
    }
    else
    { // write
      bytesCode = 0x00;
      if (cmd_buf[4] == 2)
        bytesCode = 0x02;
    }

    mrb[0] = (bytesCode << 4) | cmd_buf[2];
    mrb[1] = cmd_buf[3];

    if (cmd_buf[1] == 1)
    { // read
      mrb[2] = 0x00;
      mrb[3] = 0x00;
    }
    else
    { // write
      mrb[2] = cmd_buf[5];
      mrb[3] = cmd_buf[6];
    }

    result = sc1905_message_protocol(mrb, mrb_rcv);

    if (result != COMPLETED)
      return result;

    cmd_buf[1] = mrb_rcv[2];
    cmd_buf[2] = mrb_rcv[3];

    break;
  }

  case CMD_SC1905_RESET:
  {
    sc1905_Reset();
    break;
  }
  /*
  //milans 220614
    case (CMD_LMS_RST | LMS8001_CMD_MASK): {
      int pause = 100;
      setDiodeState(1);
      delay(pause);
      setDiodeState(0);
      delay(pause);
      setDiodeState(1);
      delay(pause);
      setDiodeState(0);
      delay(pause);
      setDiodeState(1);
      delay(pause);
      setDiodeState(0);

      break;
    }
*/
  /************   LMS8001 Commands   ***************/
  // milans 220615
  case (CMD_GET_INFO | LMS8001_CMD_MASK):
  {

    LMS_Ctrl_Packet_Tx->Data_field[0] = FW_VER;
    LMS_Ctrl_Packet_Tx->Data_field[1] = DEV_TYPE;
    LMS_Ctrl_Packet_Tx->Data_field[2] = LMS_PROTOCOL_VER;
    LMS_Ctrl_Packet_Tx->Data_field[3] = HW_VER;
    LMS_Ctrl_Packet_Tx->Data_field[4] = EXP_BOARD;

    LMS_Ctrl_Packet_Tx->Header.Status = STATUS_COMPLETED_CMD;

    break;
  }

  case (CMD_LMS_RST | LMS8001_CMD_MASK):
  {
    uint8_t targetByte;
    uint8_t targetBit;

    if (activeChannel == 0)
    {
      targetByte = LMS8001_1_RESETn_BYTE;
      targetBit = LMS8001_1_RESETn_BIT;
    }
    else
    {
      targetByte = LMS8001_2_RESETn_BYTE;
      targetBit = LMS8001_2_RESETn_BIT;
    }

    switch (LMS_Ctrl_Packet_Rx->Data_field[0])
    {
    case LMS_RST_DEACTIVATE:
      //          gpio_put(RESETN_LMS8001_PIN, 1);
      setChainBit(targetByte, targetBit, 1, activeState, nextState);
      setState(nextState);
      break;

    case LMS_RST_ACTIVATE:
      //          gpio_put(RESETN_LMS8001_PIN, 0);
      setChainBit(targetByte, targetBit, 0, activeState, nextState);
      setState(nextState);

      break;

    case LMS_RST_PULSE:
      //          gpio_put(RESETN_LMS8001_PIN, 0);
      //          asm volatile("nop");
      //          asm volatile("nop");
      //          asm volatile("nop");
      //          asm volatile("nop");
      //          gpio_put(RESETN_LMS8001_PIN, 1);
      setChainBit(targetByte, targetBit, 0, activeState, nextState);
      setState(nextState);
      setChainBit(targetByte, targetBit, 1, activeState, nextState);
      setState(nextState);

      break;

    default:
      cmd_errors++;
      break;
    }

    LMS_Ctrl_Packet_Tx->Header.Status = STATUS_COMPLETED_CMD;
    break;
  }

  case (CMD_LMS8001_WR | LMS8001_CMD_MASK):
  {
    if (Check_many_blocks(4))
      break;

    //      GPIO_ResetBits(SAEN_PORT, SAEN_PIN); //Enable LMS's SPI
    //      cs_select(periphID);
    cs_select(PERIPH_LMS8001);
    for (block = 0; block < LMS_Ctrl_Packet_Rx->Header.Data_blocks; block++)
    {
      // write reg addr
      unsigned char false_write_to_0x0000 = 0;

      if ((LMS_Ctrl_Packet_Rx->Header.Data_blocks == 4) &&
          (block != 0) &&
          (LMS_Ctrl_Packet_Rx->Data_field[0 + (block * 4)] == 0x00) &&
          (LMS_Ctrl_Packet_Rx->Data_field[1 + (block * 4)] == 0x00))
        false_write_to_0x0000 = 1;

      if (false_write_to_0x0000 == 0)
        sbi(LMS_Ctrl_Packet_Rx->Data_field[0 + (block * 4)], 7); // set write bit

      SPI1_transfer_byte(LMS_Ctrl_Packet_Rx->Data_field[0 + (block * 4)]); // reg addr MSB with write bit
      SPI1_transfer_byte(LMS_Ctrl_Packet_Rx->Data_field[1 + (block * 4)]); // reg addr LSB*/

      // write reg data
      SPI1_transfer_byte(LMS_Ctrl_Packet_Rx->Data_field[2 + (block * 4)]); // reg data MSB
      SPI1_transfer_byte(LMS_Ctrl_Packet_Rx->Data_field[3 + (block * 4)]); // reg data LSB
    }

    //          GPIO_SetBits(SAEN_PORT, SAEN_PIN); //Disable LMS's SPI
    //      cs_deselect(periphID);
    cs_deselect(PERIPH_LMS8001);

    LMS_Ctrl_Packet_Tx->Header.Status = STATUS_COMPLETED_CMD;
    break;
  }

  case (CMD_LMS8001_RD | LMS8001_CMD_MASK):
  {

    if (Check_many_blocks(4))
      break;
    // Reconfigure_SPI_for_LMS ();

    //      GPIO_ResetBits(SAEN_PORT, SAEN_PIN); //Enable LMS's SPI
    //      cs_select(periphID);
    cs_select(PERIPH_LMS8001);

    for (block = 0; block < LMS_Ctrl_Packet_Rx->Header.Data_blocks; block++)
    {
      // write reg addr
      cbi(LMS_Ctrl_Packet_Rx->Data_field[0 + (block * 2)], 7); // clear write bit

      SPI1_transfer_byte(LMS_Ctrl_Packet_Rx->Data_field[0 + (block * 2)]); // reg addr MSB
      SPI1_transfer_byte(LMS_Ctrl_Packet_Rx->Data_field[1 + (block * 2)]); // reg addr LSB

      LMS_Ctrl_Packet_Tx->Data_field[0 + (block * 4)] = LMS_Ctrl_Packet_Rx->Data_field[0 + (block * 2)];
      LMS_Ctrl_Packet_Tx->Data_field[1 + (block * 4)] = LMS_Ctrl_Packet_Rx->Data_field[1 + (block * 2)];

      // read reg data
      LMS_Ctrl_Packet_Tx->Data_field[2 + (block * 4)] = SPI1_transfer_byte(0x00); // reg data MSB
      LMS_Ctrl_Packet_Tx->Data_field[3 + (block * 4)] = SPI1_transfer_byte(0x00); // reg data LSB
    }

    //      GPIO_SetBits(SAEN_PORT, SAEN_PIN); //Disable LMS's SPI
    //      cs_deselect(periphID);
    cs_deselect(PERIPH_LMS8001);

    LMS_Ctrl_Packet_Tx->Header.Status = STATUS_COMPLETED_CMD;

    break;
  }
    // milans 220811
    // ovdi!!!
  case (CMD_ADF4002_WR | LMS8001_CMD_MASK):
  {
    if (Check_many_blocks(3))
      break;

    for (block = 0; block < LMS_Ctrl_Packet_Rx->Header.Data_blocks; block++)
    {
      cs_select(PERIPH_ADF4002);

      SPI1_transfer_byte(LMS_Ctrl_Packet_Rx->Data_field[0 + (block * 3)]);
      SPI1_transfer_byte(LMS_Ctrl_Packet_Rx->Data_field[1 + (block * 3)]);
      SPI1_transfer_byte(LMS_Ctrl_Packet_Rx->Data_field[2 + (block * 3)]);

      cs_deselect(PERIPH_ADF4002);
    }

    LMS_Ctrl_Packet_Tx->Header.Status = STATUS_COMPLETED_CMD;
    break;
    /*
         case CMD_ADF4002_WR:
          if(Check_many_blocks (3)) break;

          for(block = 0; block < LMS_Ctrl_Packet_Rx->Header.Data_blocks; block++)
          {
            GPIO_ResetBits(SBEN_PORT, SBEN_PIN); //Enable ADF's SPI

            SPI1_transfer_byte(LMS_Ctrl_Packet_Rx->Data_field[0 + (block * 3)]);
            SPI1_transfer_byte(LMS_Ctrl_Packet_Rx->Data_field[1 + (block * 3)]);
            SPI1_transfer_byte(LMS_Ctrl_Packet_Rx->Data_field[2 + (block * 3)]);

            GPIO_SetBits(SBEN_PORT, SBEN_PIN); //Disable ADF's SPI
          }

          LMS_Ctrl_Packet_Tx->Header.Status = STATUS_COMPLETED_CMD;
          break;
 */
  }
  default:
  {
    clearBuffer(cmd_buf);
  }
  }
  return COMPLETED;
}

void clearBuffer()
{
  for (int i = 0; i < BUFFER_SIZE; i++)
  {
    activeBuffer[i] = 0x0;
  }
}

void clearBuffer(uint8_t *buf)
{
  for (int i = 0; i < BUFFER_SIZE; i++)
  {
    buf[i] = 0x0;
  }
}

void wait_for_bytes(int num_bytes, unsigned long timeout)
{
  unsigned long startTime = millis();
  // Wait for incoming bytes or exit if timeout
  while ((Serial.available() < num_bytes) && (millis() - startTime < timeout))
  {
  }
}

// B.J.
// for debugging purpose
void AddRDbyteX(uint16_t dst)
{
  RDbyteX[wrPtr] = (uint8_t)((dst & 0xFF00) >> 8);
  if (wrPtr < 127)
    wrPtr++;
  else
    wrPtr = 0;
  RDbyteX[wrPtr] = (uint8_t)(dst & 0x00FF);
  if (wrPtr < 127)
    wrPtr++;
  else
    wrPtr = 0;
}

// B.J.
void MyFunction()
{

  while (spi_is_readable(SPISLAVE))
  {
    //sleep_us(8);
    switch (spi_state)
    {
    case 0:
      spi_write16_read16_blocking(SPISLAVE, &src, &dst, 1);
      maddr = (dst & 0x7FE0) >> 5;
      faddr = (dst & 0x001F);
      w_nr = (dst & 0x8000) >> 15;
      src = 0;

      if (maddr == maddress)
      {
        if (w_nr == 0) // READ
          src = (BuffLMS8FE[2 * faddr + 1] << 8) + BuffLMS8FE[2 * faddr];
        else
          src = 0;
        spi_state = 1;
      }
      else if ((maddr == maddress2) && (faddr == 0))
      {
        if (w_nr == 0) // READ
          src = (RegLMS8FE[1] << 8) + RegLMS8FE[0];
        else
          src = 0;
        spi_state = 1; // no error
      }
      else
      {
        src = 0;       // error
        spi_state = 0; // error
      }
      sleep_us(13); // MORA DELAY
      break;

    case 1:
      spi_write16_read16_blocking(SPISLAVE, &src, &dst, 1);
      if (w_nr == 1)
      {
        if (maddr == maddress)
        {
          BuffLMS8FE[2 * faddr + 1] = ((dst & 0xFF00) >> 8); // msb part
          BuffLMS8FE[2 * faddr] = (dst & 0x00FF);            // lsb part
        }
        else if ((maddr == maddress2) && (faddr == 0))
        {
          RegLMS8FE[1] = ((dst & 0xFF00) >> 8); // msb part
          RegLMS8FE[0] = (dst & 0x00FF);        // lsb part
        }
      }
      src = 0;
      spi_state = 0;
      sleep_us(13); // MORA DELAY
      break;

    default:
      spi_state = 0;
    }
  }
}
