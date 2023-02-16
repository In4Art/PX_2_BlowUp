#include <Arduino.h>
#include <SPI.h>

#ifdef ESP8266
 #include <ESP8266WiFi.h>
#else //ESP32
 #include <WiFi.h>
#endif

#include <ModbusIP_ESP8266.h>

#include "SPI_shiftreg.h"


#include "creds.h"
#include "WifiControl.h"
#include "ModeControl.h"

#include "PX_Bronchi.h"

#define PX_NUM 2
#define PX_REG 100 + (PX_NUM * 10) //base operational modbus register
#define PX_STATE_REG 200 + (PX_NUM * 10) // base status modbus register

#define SHIFT_REG_OE D4
#define SHIFT_REG_LATCH D2
#define SHIFT_REG_CLR D3

#define NUM_VALVE_BOARDS 1

#define ACTIVE_PX_BRONCHI 5

#define MSTR_BREATH_PERIOD 10000
uint32_t mstr_breath_t = 0;

//master inflate deflate states
enum{
  MSTR_INFLATE,
  MSTR_DEFLATE
};
uint8_t mstr_breath_state = MSTR_INFLATE;


uint8_t regDatas[NUM_VALVE_BOARDS];


void clearAll(void);
void shiftBytes(uint8_t *shiftData, uint8_t nBytes);
void shiftRegInit(void);

SPI_shiftreg pxReg(D2, D3, D4, 1);

//wifi setup
char ssid[] = SSID ;        // your network SSID (name)
char pass[] = PW;                    // your network password
WifiControl pxWifi(ssid, pass, PX_NUM);

enum {
  PX_ERR = -1,
  PX_OK
};

ModbusIP pxModbus;

int8_t px2State = 0;

PX_Bronchi *px_br;

void setState(int8_t state);


#define DEMO_SW_PIN 3
int8_t demoState = 0;
void demoCallback(uint32_t dTime, px_mode_t mode);
ModeControl pxMC(DEMO_SW_PIN, &demoCallback, 2 * DEFLATE_TIME, &pxWifi);

void setup() {
  
  pxReg.enable_output();

  Serial.begin(57600, SERIAL_8N1, SERIAL_TX_ONLY);
  delay(2000);

  //time out used for waiting for a connection to occur
  //useful to change during testing to speed things up when dev-ing with no C&C
  pxWifi.setTimeOut(30000);

  if(digitalRead(DEMO_SW_PIN) == HIGH){ //skip initial connection attempt to wifi if demo switch is active
    Serial.println("Connecting to C&C...");
    int8_t res = pxWifi.init();
    if(res == -1){
      Serial.println("No C&C found, starting up in demo mode!");
    }
  }

  pxModbus.server(502);
  pxModbus.addHreg(PX_REG, 0);
  pxModbus.addIreg(PX_REG, 0);
  pxModbus.addHreg(PX_STATE_REG, PX_OK);

  

  ////WARNING CURRENT IMPLEMENTATION OF PX_Bronchi DOES NOT DEAL WITH THE SPARE PINS YET!!!!!!
  /// THEREFOR ACTIVE_PX_BRONCHI IS DEFINED AS 4!!!
  uint8_t spare_pins[] = {D0, D1, 9, 10};
  px_br = new PX_Bronchi(ACTIVE_PX_BRONCHI, 1, &pxReg, spare_pins, 4 );
    
  //delay for 10 seconds to give the compressor time to pressurize the system
  delay(10000);
  pxMC.init(); //initialize modeControl

  //deflate all!
  for(uint8_t i = 0; i < ACTIVE_PX_BRONCHI; i++){
    px_br->deflate(i);
  }
  Serial.println("Setup complete");

}

void loop() {

  pxModbus.task();

  pxMC.run();

  if(pxModbus.Hreg(PX_REG) != pxModbus.Ireg(PX_REG)){
    pxModbus.Ireg(PX_REG, pxModbus.Hreg(PX_REG));
  }

  if(px2State != pxModbus.Ireg(PX_REG)){
    int8_t ireg = pxModbus.Ireg(PX_REG);
    if(ireg >= 0 && ireg <= 3){
      setState(ireg);
      px2State = ireg;
    }
  

  }

  if(millis() - mstr_breath_t > MSTR_BREATH_PERIOD){
    mstr_breath_t = millis();
    if(mstr_breath_state == MSTR_INFLATE){

      switch(px2State){
        case 0:
          break;
        case 1:
          px_br->inflate(2);
          break;
        case 2:
          px_br->inflate(1);
          px_br->inflate(2);
          px_br->inflate(3);
          break;
        case 3:
          for(uint8_t i = 0; i < ACTIVE_PX_BRONCHI; i++){
            px_br->inflate(i);
          }
          break;
        default:
          break;
      }

      mstr_breath_state = MSTR_DEFLATE;
    }else if(mstr_breath_state == MSTR_DEFLATE && px2State != 0){

      switch(px2State){
        case 0:
          break;
        case 1:
         
          px_br->deflate(2);
          break;
        case 2:
          px_br->deflate(1);
          px_br->deflate(2);
          px_br->deflate(3);
          break;
        case 3:
          for(uint8_t i = 0; i < ACTIVE_PX_BRONCHI; i++){
            px_br->deflate(i);
          }
          break;
        default:
          break;
      }
      mstr_breath_state = MSTR_INFLATE;
    }

  }

  px_br->run();

  //attempt to reconnect to C&C
  //do not attempt when any px_br is INFLATING
  //reConn() blocks for some time
  if(pxWifi.getStatus() != WL_CONNECTED){
    bool canReconnect = true;
    for(int8_t i = 0; i < ACTIVE_PX_BRONCHI; i++){
      if(px_br->get_state(i) == INFLATING){
        canReconnect = false;
      }
    }
    if(px2State != 0){
      canReconnect = false;
    }
    if(canReconnect){
      pxWifi.reConn();
    }
    
  }

}


void setState(int8_t state)
{
    switch(state){
        case 0:
          for(uint8_t i = 0; i < ACTIVE_PX_BRONCHI; i++){
            px_br->deflate(i);
          }
          break;
        case 1:
          px_br->inflate(2);
          px_br->deflate(0);
          px_br->deflate(1);
          px_br->deflate(3);
          px_br->deflate(4);
          break;
        case 2:
          px_br->inflate(1);
          px_br->inflate(2);
          px_br->inflate(3);
          px_br->deflate(0);
          px_br->deflate(4);
          break;
        case 3:
          for(uint8_t i = 0; i < ACTIVE_PX_BRONCHI; i++){
            px_br->inflate(i);
          }
          break;
        default:
          break;
      }
  
}

void demoCallback(uint32_t dTime, px_mode_t mode){

  if(mode == PX_DEMO_MODE){
  if(demoState > 7){
        demoState = 0;
        
      }
      if(demoState > 4){
        pxModbus.Hreg(PX_REG, 8 - demoState);
        Serial.print("Checking PX status reg in demo mode: ");
        Serial.println(pxModbus.Hreg(PX_REG));
      }else{
        pxModbus.Hreg(PX_REG, demoState);
        Serial.print("Checking PX status reg in demo mode: ");
        Serial.println(pxModbus.Hreg(PX_REG));
      }
      demoState++;
  }else if(mode == PX_CC_MODE){
    demoState = 0;
    pxModbus.Hreg(PX_REG, demoState);
  }
}


/* clears all data in shift reg by toggling SRCLR pin from HIGH->LOW->HIGH
* then toggling RCLK pin from LOW->HIGH->LOW
*/
void clearAll(void){
	digitalWrite(SHIFT_REG_CLR, LOW);//clear all data after 10secs
	digitalWrite(SHIFT_REG_LATCH, HIGH);
	delayMicroseconds(10);
	digitalWrite(SHIFT_REG_LATCH, LOW);

}

void shiftRegInit(void){
	for(uint8_t i = 0; i < NUM_VALVE_BOARDS; i++){
		regDatas[i] = 0x00;
	}
}

/*
 * writes an array of byte to the shift register
 */
void shiftBytes(uint8_t *shiftData, uint8_t nBytes){

	while(nBytes > 0){
		SPI.transfer(*(shiftData + (nBytes - 1)));
		nBytes--;
	}
	digitalWrite(SHIFT_REG_LATCH, HIGH);
	delayMicroseconds(10);
	digitalWrite(SHIFT_REG_LATCH, LOW);

}