/**************************************************************************
 @FILE:         LoRaX1.ino
 @AUTHOR:       Raimundo Alfonso
 @COMPANY:      Ray Ingeniería Electronica, S.L.
 @DESCRIPTION:  Ejemplo de uso para el nodo LoRaX1 y emoncms (https://emoncms.org)
                Example of use for the LoRaX1 node and emoncms (https://emoncms.org)
  
 @LICENCE DETAILS:
  Este sketch está basada en software libre. Tu puedes redistribuir
  y/o modificar esta sketch bajo los términos de licencia GNU.

  Esta programa se distribuye con la esperanza de que sea útil,
  pero SIN NINGUNA GARANTÍA, incluso sin la garantía implícita de
  COMERCIALIZACIÓN O PARA UN PROPÓSITO PARTICULAR.
  Consulte los términos de licencia GNU para más detalles:
                                                                       
  http://www.gnu.org/licenses/gpl-3.0.txt

  This sketch is based on free software. You can redistribute
  and/or modify this library under the terms of the GNU license.

  This software is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY, even without the implied warranty of
  MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU license terms for more details:   
  
  http://www.gnu.org/licenses/gpl-3.0.txt

 @VERSIONS:
  17-07-2019 - v1.00 : Primera versión
  
**************************************************************************/

#define  FIRMWARE_VERSION "1.00"
#define  HARDWARE_VERSION "190203"

#include <SPI.h>
#include <RHReliableDatagram.h>
#include <RH_RF95.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <EEPROM.h>
#include "ADE7753.h"

// Defines what sensors are available in the hardware...


// Structure of default configuration parameters. This parameters are stored in internal eeprom...
typedef struct {
  int     sendTime    = 5;      // [1...9999] Send time delay (seconds)
  byte    rfPower     = 14;     // [5...23]   RF power (5:min...23:max)
  byte    rfRetries   = 3;      // [0...20]   Data send retries (0 = no retries)
  byte    rfNode      = 11;     // [0...250]  Node id
  byte    rfPan       = 100;    // [0...250]  PAN id (Only nodes with this same number are visible)
} stConfig;
stConfig config;

// Identify the node
//#define DEVICE_ID      5      // 5 = LoRaSDL
//#define DEVICE_ID      6      // 6 = LoRaTH
#define DEVICE_ID      7      // 7 = LoRaX1
//#define DEVICE_ID      8      // 8 = LoRaHALL

// Hardware definitions...
#define SERVER_ADDRESS 250
#define RX        0
#define TX        1
#define RFM95_INT 2
#define RFM95_CS  4
#define BT_EN     5
#define LED_ST    6
#define BUZZER    A0
#define RELAY     A1
#define DIR_EEPROM_CFG  10

// Payload structure...  
typedef struct {
  byte        pan_id;
  byte        device_id = DEVICE_ID;
  int         status;         // bit 0:  Funcionando OK   
                              // ...
                              // bit 15: device timeout
  int         rssi;           // x1   - dB
  int         frequency;      // x10  - Hz
  int         v_rms;          // x1   - V
  int         v_peak;         // x1   - V
  int         i_rms;          // x100 - A
  int         p_act;          // x1   - W
  int         e_act;          // x1   - KWh
} Payload;
Payload theData;

// RFM95 transceiver configuration and instances...
#define RF95_FREQ 868.0
RH_RF95 rf95(RFM95_CS, RFM95_INT);
RHReliableDatagram manager(rf95, 0);

// ADE7753 instance...
ADE7753 ade;
void ADE7753_IRQ(void);

// Global variables...
boolean modoCMD = false;  // flag que indica cuando estás en modo comando de linea

/**************************************************************************
 * SETUP
 *************************************************************************/  
void setup(){
  pinMode(LED_ST, OUTPUT);
  pinMode(BUZZER, OUTPUT);
  pinMode(RELAY, OUTPUT);
   
  led_st_off();
  
  // Make a Beep to warn that ArduX1 has restarted...
  digitalWrite(BUZZER, HIGH);
  delay(500);
  digitalWrite(BUZZER, LOW);

  // Check if the eeprom is empty...
  if(EEPROM.read(0) == 123){
    // Read the configuration parameters...
    EEPROM.get(DIR_EEPROM_CFG, config);
  }else{
    // If it is empty, it saves the configuration parameters by default...
    EEPROM.write(0,123);
    EEPROM.put(DIR_EEPROM_CFG, config);
  }

  // Init serial port...
  while (!Serial);
  Serial.begin(9600);

  // Init payload struct...
  theData.pan_id   = config.rfPan;
  theData.frequency = 0;
  theData.v_rms = 0;
  theData.v_peak = 0;  
  theData.i_rms  = 0;
  theData.p_act = 0; 
  theData.e_act  = 0;
  theData.status |= 0x01;   

  // Init RFM95 module...
  radioInit();
  rf95.sleep();
  delay(10);  

  // Init ADE7753 chip...
  ade.begin();

  pinMode(RX, INPUT);
  delay(100);

 
  // Check the RX pin to see if you have to enter command line mode...
  if(digitalRead(RX)){  
    modoCMD = true;
    led_commandLine();
    commandLine();
    modoCMD = false;
  }
}


/**************************************************************************
 * LOOP
 *************************************************************************/ 
void loop(){
  // // Turn on status led. This led shows us how long the node is read and transmit data...
  led_st_on();
  
  // Read energy measure data...
  lee_ade();
      
  // Send payload to server...
  detachInterrupt(digitalPinToInterrupt(PIN_IRQ));
  send_to_server();
  attachInterrupt(digitalPinToInterrupt(PIN_IRQ), ADE7753_IRQ, LOW);

  // Turn of status led...
  led_st_off();

  // Wait x seconds...
  espera(config.sendTime);
}


/**************************************************************************
 * FUNCTIONS
 *************************************************************************/ 

void espera(int segundos){
  int m = segundos * 10;
  int n;
  for(n=0;n<m;n++){
    delay(100);
  }
}

boolean send_to_server(void){
  boolean r1;

  manager.setThisAddress(config.rfNode);
  r1 = manager.sendtoWait((const void*)(&theData), sizeof(theData), SERVER_ADDRESS);
  rf95.sleep(); 
  delay(10);

  SPI.setDataMode(SPI_MODE1);
  return(r1);
}

void lee_ade(void){ 
  theData.frequency = (int)(ade.freqRead() * 10.0);
  theData.v_rms     = (int)(ade.vrmsRead());
  theData.v_peak    = (int)(ade.vPeakRead());
  theData.i_rms     = (int)(ade.irmsRead() * 100.0);
  theData.p_act     = (int)(ade.pActiveRead());
  theData.e_act     = (int)(ade.eActiveRead());
  theData.status |= 0x01;   
}

void led_st_on(void){
  digitalWrite(LED_ST, HIGH);
}

void led_st_off(void){
  digitalWrite(LED_ST, LOW);
}

void led_commandLine(void){
  for(byte n=0;n<3;n++){
    led_st_on();
    delay(50);
    led_st_off();
    delay(50);
  }
}

void led_init(void){
    led_st_on();
    delay(50);
    led_st_off();
}


void radioInit(void){
  
  while (!manager.init()) {
    Serial.println(F("LoRa radio init failed"));
    while (1);
  }

  manager.setThisAddress(config.rfNode);
  manager.setRetries(config.rfRetries);
  manager.setTimeout(100);

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println(F("setFrequency failed"));
    while (1);
  }

  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on

  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then 
  // you can set transmitter powers from 5 to 23 dBm:
  rf95.setTxPower(config.rfPower, false);
}


int filterIIR (int data, byte f){
  static boolean ini = true;
  static long data_ant = 0;
  long data_now;
  long output;

  data_now = (long)data * 10L;

  // Captura la primera muestra...
  if(ini){
    data_ant = data_now;
    ini = false;
  }

  // Filtra...
  data_ant = ((data_now * (10L - f)) + (data_ant * (f))) / 10L;
  output = data_ant / 10L;

  return(output);
}

void i2DecimalPoint(int numero, byte decimales, char *cadena){
  byte len;
  int i;
  int x;
  char format[5];

  x = decimales - len + 1;
  sprintf(format,"%%0%dd", x);
   
  // Pasa el número a cadena...
  sprintf(cadena,format,numero);
  len = strlen(cadena);
  
  // Recorre toda la cadena hasta que encuentres el punto...
  for(i = len + 0; i>=0; i--){
    cadena[i+1] = cadena[i];
    if(i == len - decimales){
      cadena[i] = '.';
      break;
    }    
  }
}




