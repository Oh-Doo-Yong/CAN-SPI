//#include "Arduino.h"

#include "drv_canfdspi_api.h"
#include "drv_spi.h"
#include "CanAPI.h"

bool flush = true;

void setup() {
  Serial.begin(115200);
  DRV_SPI_Initialize();
  
  uint8_t error = 0;

  configure_can(CAN_1000K_8M);
  error = can_init_interrupts();
  can_init_filter(CAN_FILTER0, 0x400, 0xF00);
  can_init_interrupts();
  
  Serial.println(error);
}

void loop() {
//  if (can_test_ram_access())
//    Serial.println("error reading ram");
//  else
//    Serial.println("ram works fine\n");

//  if (can_test_register_access())
//    Serial.println("error reading registers");
//  else
//    Serial.println("register access OK");

  uint8_t error = 0;
  uint8_t *txPacket = malloc(8*sizeof(uint8_t));
  error = can_transmit(0x45A, txPacket, CAN_DLC_8);
  if(error)
    Serial.println("CAN send issue");
  else
    Serial.println("CAN send OK");

  delay(500);
  uint8_t rxLen;
  uint8_t rx[MAX_DATA_BYTES]; // same as  uint8_t *rx = malloc(8*sizeof(uint8_t));
  uint16_t rxSid;
  uint8_t *rxObj = can_receive(&rxSid, rx, &rxLen);
  if(rxObj){
    for(int i=0; i<rxLen; i++)
      {Serial.print(rx[i]); Serial.print(" ");}
    Serial.println(" UpShift");
  }
  else
    Serial.println("DownShift");

  delay(1000);
}
