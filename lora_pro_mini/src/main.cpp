/*********
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/ttgo-lora32-sx1276-arduino-ide/
*********/

//Libraries for LoRa
#include <Arduino.h>
#include "LowPower.h"
#include <SPI.h>
#include <LoRa.h>
#include <LoRa.cpp>

#include <avr/sleep.h>
#include <avr/wdt.h>

/*******************
//define the pins used by the LoRa transceiver module (for esp32)
#define SCK 18
#define MISO 19
#define MOSI 23
#define SS 05
#define RST 14
#define DIO0 02
#define TXEN 25
#define RXEN 26
*****/

//define the pins used by the LoRa transceiver module (for pro mini)
#define SCK 13
#define MISO 12
#define MOSI 11
#define SS 10
#define RST 9
#define DIO0 2
#define TXEN 8
#define RXEN 7
//433E6 for Asia
//866E6 for Europe
//915E6 for North America


#define BAND 915E6


//packet counter
uint16_t counter = 1;


struct MsgStruct {
  uint8_t header[4]; // cabeçalho usado na biblioteca rh95.h
  uint16_t count;
  char  pangrama[44]; //= "the quick brown fox jump over the crazy dog";
};

union MsgUnion {
  char msg_array[sizeof(MsgStruct)];
  MsgStruct msg;
};
 


MsgUnion msg_tx;

// watchdog interrupt
ISR (WDT_vect) 
{
   wdt_disable();  // disable watchdog
}  // end of WDT_vect

void setup() {
  Serial.begin(9600);
  
  Serial.println("LoRa Sender Test");  
    // disable ADC
  ADCSRA = 0;  


  sprintf(msg_tx.msg.pangrama, "the quick brow fox jump over the crazy dog");

  //Serial.begin(9600);
  
  //Serial.println("LoRa Sender Test");

  // inicia pinos de controle TX RX do modulo E19
  pinMode(TXEN, OUTPUT);
  digitalWrite(TXEN, LOW);
  pinMode(RXEN, OUTPUT);
  digitalWrite(RXEN, LOW); 

  //SPI LoRa pins
 // SPI.begin(SCK, MISO, MOSI, SS);
  SPI.begin();
  LoRa.setSPI(SPI);
  //setup LoRa transceiver module
  LoRa.setPins(SS, RST, DIO0);
  
  delay(10);

  if (!LoRa.begin(BAND)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }


  LoRa.setTxPower(20);
  LoRa.setFrequency(915E6);
  LoRa.setSpreadingFactor(7);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(8);
  LoRa.setPreambleLength(5);
  LoRa.setSyncWord(0x12);
  LoRa.enableCrc();
  delay(10);

  msg_tx.msg.header[0]= 255;
  msg_tx.msg.header[1]= 255;
  msg_tx.msg.header[2]= 0;
  msg_tx.msg.header[3]= 0;
}

void loop() {

  digitalWrite(TXEN, HIGH);
 
 
  int i;
  Serial.print("Sending packet: ");

  msg_tx.msg.count= counter;
  Serial.print(msg_tx.msg.pangrama);
  Serial.println(msg_tx.msg.count);
  //Send LoRa packet to receiver
  LoRa.beginPacket();
  for(i=0;i<sizeof(msg_tx.msg_array);i++){
    LoRa.write(msg_tx.msg_array[i]);  
  }
   LoRa.endPacket();
//  LoRa.print(counter);

  counter++;
  digitalWrite(TXEN, LOW);
  LoRa.end();

//pinMode(SCK, INPUT);
//pinMode(MOSI, INPUT);
//pinMode(SS,INPUT);
pinMode(RST,INPUT_PULLUP);
//pinMode(DIO0, INPUT);
//pinMode(TXEN, INPUT);
//pinMode(RXEN, INPUT);

  



    // clear various "reset" flags
  MCUSR = 0;     
  // allow changes, disable reset
  WDTCSR = bit (WDCE) | bit (WDE);
  // set interrupt mode and an interval 
 // WDTCSR = bit (WDIE) | bit (WDP3) | bit (WDP0);    // set WDIE, and 8 seconds delay
  WDTCSR = bit (WDIE) | bit (WDP3);    // set WDIE, and 8 seconds delay
  wdt_reset();  // pat the dog

  
  set_sleep_mode (SLEEP_MODE_PWR_DOWN);  
  noInterrupts ();           // timed sequence follows
  sleep_enable();
 
  // turn off brown-out enable in software
  MCUCR = bit (BODS) | bit (BODSE);
  MCUCR = bit (BODS); 
  interrupts ();             // guarantees next instruction executed
  sleep_cpu ();  
  
  // cancel sleep as a precaution
  sleep_disable();

}




/*********
#include <Arduino.h>
#include "LowPower.h"
#include <SPI.h>
#include <LoRa.h>

#include <avr/sleep.h>
#include <avr/wdt.h>


  
// watchdog interrupt
ISR (WDT_vect) 
{
   wdt_disable();  // disable watchdog
}  // end of WDT_vect
 
void setup () { }

void loop () 
{
 
  pinMode (LED, OUTPUT);
  digitalWrite (LED, HIGH);
  delay (50);
  digitalWrite (LED, LOW);
  pinMode (LED, INPUT);
  
  // disable ADC
  ADCSRA = 0;  

  // clear various "reset" flags
  MCUSR = 0;     
  // allow changes, disable reset
  WDTCSR = bit (WDCE) | bit (WDE);
  // set interrupt mode and an interval 
  WDTCSR = bit (WDIE) | bit (WDP2) | bit (WDP1);    // set WDIE, and 1 second delay
  wdt_reset();  // pat the dog
  
  set_sleep_mode (SLEEP_MODE_PWR_DOWN);  
  noInterrupts ();           // timed sequence follows
  sleep_enable();
 
  // turn off brown-out enable in software
  MCUCR = bit (BODS) | bit (BODSE);
  MCUCR = bit (BODS); 
  interrupts ();             // guarantees next instruction executed
  sleep_cpu ();  
  
  // cancel sleep as a precaution
  sleep_disable();
  
  } // end of loop
**********************/