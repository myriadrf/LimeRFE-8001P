#include "Arduino.h"
#include <Wire.h>
#include "consts.h"
#include "mcu.h"
#include "chain.h"
#include "cmd.h"

void setChainBit(uint8_t targetByte, uint8_t targetBit, uint8_t value, unsigned char* currentState, unsigned char* tmpState){
 //  memcpy(tmpState, currentState, sizeof(currentState[0])*CHAIN_SIZE);
 //milans 221202
  memcpy(tmpState, currentState, sizeof(currentState[0])*STATE_SIZE);
  if(value == 0)
    cbi(tmpState[targetByte], targetBit);
  else
    sbi(tmpState[targetByte], targetBit);    

//  shiftData(tmpState);
//  setupMCU(tmpState[MCU_BYTE]);
//  memcpy(currentState, tmpState, sizeof(tmpState[0])*CHAIN_SIZE);

//  Function: void setState(unsigned char* state) needs to be invoked for the modified chain data to be active
}

void resetData(){
  digitalWrite(DATA_SER_RESETn_PIN, LOW);
  delay(10); //time?
  digitalWrite(DATA_SER_RESETn_PIN, HIGH);  

  digitalWrite(DATA_OUT_RESETn_PIN, LOW);
  delay(10); //time?
  digitalWrite(DATA_OUT_RESETn_PIN, HIGH);
}

//void shiftData(unsigned char* data, unsigned char* currentState){
void shiftData(unsigned char* data){
  
//  disableLNAs();
//  disablePAs();
  
//  digitalWrite(DATA_RESET_PIN, DATA_RESET);     // Reset Registers Values
//  delay(10); // todo time?
//  digitalWrite(DATA_RESET_PIN, DATA_RESETn);    // Enable Register
// Do we need to reset before we shift data?
  myShiftOut(DATA_PIN, DATA_SER_CLK_PIN, MSBFIRST, data, CHAIN_SIZE);
//  memcpy(currentState, data, sizeof(data[0])*CHAIN_SIZE);
//  memcpy(currentState, data, sizeof(data[0])*STATE_SIZE);

  digitalWrite(DATA_OUT_CLK_PIN, HIGH);
  digitalWrite(DATA_OUT_CLK_PIN, LOW);  
}

void myShiftOut(uint8_t dataPin, uint8_t clockPin, uint8_t bitOrder, unsigned char* val, unsigned int numOfBytes){
//milans 220714  
//  for(int a = 0; a < numOfBytes; a++)  {
  for(int a = numOfBytes; a >= 0; a--)  {    
    for (int i = 0; i < 8; i++)  {
      if (bitOrder == LSBFIRST)
        digitalWrite(dataPin, !!(val[a] & (1 << i)));
      else
        digitalWrite(dataPin, !!(val[a] & (1 << (7 - i))));
      digitalWrite(clockPin, HIGH);
      digitalWrite(clockPin, LOW);  
    }
  }
}
