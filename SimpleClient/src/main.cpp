/**
 * @file main.cpp
 * @author Paulo Valim
 * @brief 
 * @version 0.1
 * @date 2021-12-18
 * 
 * @copyright Copyright (c) 2021
 * 
 * Exemplo de um cliente simples para as placas TTGO e Heltec
 * Para estas placas é necessário usar o driver SPI por software em função
 * da conexão dos pinos à placa LoRa (SX12??)
 * modificado a partir do mac
 */
 
#include <SPI.h>
#include <RH_RF95.h>
#include <RHSoftwareSPI.h>

/*****
// e19 com arduino pro mini
#define RFM95_CS 10
#define RFM95_RST 9
#define RFM95_INT 2
#define TXEN 8
#define RXEN 7
*****/ 

// ttgo
#define RFM95_CS 18
#define RFM95_RST 14
#define RFM95_INT 26

// definicoes SPI da placa Heltec
#define LoRa_MOSI 27
#define LoRa_MISO 19
#define LoRa_SCK 5

#define PIN_NUM_MISO 19 
#define PIN_NUM_MOSI 27 
#define PIN_NUM_CLK  5 
#define PIN_NUM_CS   18


struct MsgStruct {
  uint16_t count;
  char  pangrama[44]; //= "the quick brown fox jump over the crazy dog";
};

union MsgUnion {
  char msg_array[sizeof(MsgStruct)];
  MsgStruct msg;
};
 
union MsgUnion msg_rx;

// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 915.0
 
RHSoftwareSPI spi;

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT, spi);
 
void setup() 
{
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  sprintf(msg_rx.msg.pangrama, "the quick brow fox jump over the crazy dog");
 // pinMode(TXEN, OUTPUT);
//  digitalWrite(TXEN, LOW);

//    pinMode(RXEN, OUTPUT);
//  digitalWrite(RXEN, LOW);

 
  while (!Serial);
  Serial.begin(115200);
  delay(100);
 
  Serial.println("Arduino LoRa TX Test!");
 
  spi.setPins(LoRa_MISO, LoRa_MOSI, LoRa_SCK); // MISO 12, MOSI 11, SCK  13

  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);
 
  while (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    while (1);
  }
  Serial.println("LoRa radio init OK!");
 
  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }
  Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);
  
  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on
 
  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then 
  // you can set transmitter powers from 5 to 23 dBm:
  rf95.setTxPower(14, false);

//  digitalWrite(TXEN, HIGH);
}
 
unsigned long packetnum = 0;  // packet counter, we increment per xmission
 
void loop()
{
  Serial.println("Sending to rf95_server");
  // Send a message to rf95_server

 Serial.print("Sending "); Serial.println(msg_rx.msg.pangrama);
 
  packetnum++;
  msg_rx.msg.count= packetnum;
  Serial.println("Sending..."); delay(10);
//  if (rf95.send((uint8_t *)msg_rx.msg_array, 46)) 
  if (rf95.send((uint8_t *)"funciona caralho",16)) 
    Serial.println("envio com sucesso");
  else Serial.println("erro de envio");
 
  Serial.println("Waiting for packet to complete..."); delay(10);
  rf95.waitPacketSent(100);

  Serial.println("Packet sending complete...");

  /****
  // Now wait for a reply
  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);
 
  Serial.println("Waiting for reply..."); delay(10);
  if (rf95.waitAvailableTimeout(5000))
  { 
    // Should be a reply message for us now   
    if (rf95.recv(buf, &len))
   {
      Serial.print("Got reply: ");
      Serial.println((char*)buf);
      Serial.print("RSSI: ");
      Serial.println(rf95.lastRssi(), DEC);    
    }
    else
    {
      Serial.println("Receive failed");
    }
  }
  else
  {
    Serial.println("No reply, is there a listener around?");
  }

  ****/

 delay(5000);
}