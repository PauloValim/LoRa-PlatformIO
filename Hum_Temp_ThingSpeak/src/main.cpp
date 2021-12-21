#include <Arduino.h>
/***
 * cliente efetuando envio de dados de temperatura  e umidade para 
 * o ThingSpeak. Versão utilizando o pro mini + E19
 * 
 ***/

#include <SPI.h>
#include <LoRa.h>
#include <DHT.h>

#define DHTPIN 5  // pino de dados do DHT21
#define DHTTYPE DHT21   // AM3201
DHT dht(DHTPIN, DHTTYPE);  // cria instância do objeto dht
/****
//define the pins used by the LoRa transceiver module TTGO
#define SCK 5
#define MISO 19
#define MOSI 27
#define SS 18
#define RST 14
#define DIO0 26
****/
//define the pins used by the LoRa transceiver module (for pro mini)
#define SCK 13
#define MISO 12
#define MOSI 11
#define SS 10
#define RST 9
#define DIO0 2
#define TXEN 8
#define RXEN 7

#define BAND 915E6

// estrutura da mensagem que vai ser trocada
struct MsgStruct {
  uint8_t header[4]; // quatro bytes ?? diferença entre as bibliotecas RH_RF95.h e Lora.h
  uint8_t ID[3]; // id do nodo transmissor
  float temp; //= "the quick brown fox jump over the crazy dog";
  float hum;
  uint16_t CRC;
};

// permite acessar a mnensagem como um array de bytes.
union MsgUnion {
  uint8_t msg_array[sizeof(MsgStruct)];
  MsgStruct msg;
};
 
union MsgUnion msg_tx;

String LoRaData;  // Buffer para recepcao de mensagem de confirmação

long lastSendTime = 0;        // last send time
long interval = 60000;          // interval between sends
uint8_t ackOk = 0;

void setup() 
{

  // inicia pinos de controle TX RX do modulo E19
  pinMode(TXEN, OUTPUT);
  digitalWrite(TXEN, LOW);
  pinMode(RXEN, OUTPUT);
  digitalWrite(RXEN, LOW); 

//  sprintf(msg_tx.msg.pangrama, "the quick brow fox jump over the crazy dog");
 //initialize Serial Monitor
  Serial.begin(9600);

  Serial.println("LoRa Sender Test");

   dht.begin();
  
  //SPI LoRa pins
 // SPI.begin(SCK, MISO, MOSI, SS);
  SPI.begin();
  LoRa.setSPI(SPI);
  //setup LoRa transceiver module
  LoRa.setPins(SS, RST, DIO0);

  if (!LoRa.begin(BAND)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
  Serial.println("LoRa Initializing OK!");

// configurações da transmissão LoRa
  LoRa.setTxPower(20);
  LoRa.setFrequency(915E6);
  LoRa.setSpreadingFactor(7);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(5);
  LoRa.setPreambleLength(8);
  LoRa.setSyncWord(0x12);  // 0x12 = 34 em decimal
  LoRa.enableCrc();
 


// inicializacao do header  ?? não sei pra que serve
 msg_tx.msg.header[0]= 255;
 msg_tx.msg.header[1]= 255;
 msg_tx.msg.header[2]= 0;
 msg_tx.msg.header[3]= 0;

 // inicializa o ID
 msg_tx.msg.ID[0] = 1;
 msg_tx.msg.ID[1] = 1;
 msg_tx.msg.ID[2] = 1;
   
}


uint16_t calcByte(uint16_t crc, uint8_t b)
{
    uint32_t i;
    crc = crc ^ (uint32_t)b << 8;
    
    for ( i = 0; i < 8; i++)
    {
        if ((crc & 0x8000) == 0x8000)
            crc = crc << 1 ^ 0x1021;
        else
            crc = crc << 1;
    }
    return crc & 0xffff;
}

uint16_t CRC16(uint8_t *pBuffer,uint32_t length)
{
    uint16_t wCRC16=0;
    uint32_t i;
    if (( pBuffer==0 )||( length==0 ))
    {
      return 0;
    }
    for ( i = 0; i < length; i++)
    { 
      wCRC16 = calcByte(wCRC16, pBuffer[i]);
    }
    return wCRC16;
}



void sendMessage(){
  digitalWrite(TXEN, HIGH);
    Serial.println("Sending to rf95_server");
  // Send a message to rf95_server
 // uint8_t data[] = "Hello World!";

  LoRa.beginPacket();
  LoRa.write(msg_tx.msg_array, sizeof(msg_tx.msg_array));
  LoRa.endPacket();
  digitalWrite(TXEN, LOW);
}

void onReceive(int packetSize) {
  if (packetSize == 0) return;          // if there's no packet, return 0

  //received a packet
  Serial.print("Received packet of size: ");
  Serial.print(packetSize);
  Serial.print(": ");

  //read packet
  while (LoRa.available()) {
    LoRaData = LoRa.readString();
    Serial.println(LoRaData);
  }
  ackOk = 1; // indica que recebeu um pacote
}


void readDHT(){
  // char temp[6];
  // char hum[6];
    // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  float h = dht.readHumidity();
  // Read temperature as Celsius (the default)
  float t = dht.readTemperature();


  msg_tx.msg.temp = t;
  msg_tx.msg.hum = h;


  // dtostrf(t, 3, 1, temp);
  // dtostrf(h, 3, 1, hum);
  // Read temperature as Fahrenheit (isFahrenheit = true)
  float f = dht.readTemperature(true);

  // Check if any reads failed and exit early (to try again).
  if (isnan(h) || isnan(t) || isnan(f)) {
    Serial.println(F("Failed to read from DHT sensor!"));
    return;
  }

  // // Compute heat index in Fahrenheit (the default)
  // float hif = dht.computeHeatIndex(f, h);
  // // Compute heat index in Celsius (isFahreheit = false)
  // float hic = dht.computeHeatIndex(t, h, false);


//  sprintf(msg_tx.msg.pangrama, "Humidity: %s / Temperature %s°C", hum, temp);
//  Serial.println(msg_tx.msg.pangrama);
//  Serial.print(F("Humidity: "));
//  Serial.print(h);
//  Serial.print(F("%  Temperature: "));
//  Serial.print(t);
//  Serial.print(F("°C "));
//  Serial.print(f);
//  Serial.print(F("°F  Heat index: "));
//  Serial.print(hic);
//  Serial.print(F("°C "));
//  Serial.print(hif);
//  Serial.println(F("°F"));
}

void loop()
{

  if (millis() - lastSendTime > interval) { // efetua a transmissão a cada "interval" (ms)

 //   Serial.println(msg_tx.msg.pangrama);
    lastSendTime = millis();            // timestamp the message
    readDHT();

    msg_tx.msg.CRC = CRC16(msg_tx.msg.ID,sizeof(msg_tx)-6);
    Serial.print("CRC envio: ");
    Serial.println(msg_tx.msg.CRC, HEX);

    sendMessage();
    ackOk= 0; // indica que aguarda reconhecimento desta mensagem
    Serial.print("Sending ");
    for (int i= 0; i< sizeof(msg_tx); i++){
      Serial.print(msg_tx.msg_array[i], HEX);
    }
    Serial.println("");

 //   interval = random(2000) + 1000;    // 2-3 seconds
  }

  
  // parse for a packet, and call onReceive with the result:
  digitalWrite(RXEN, HIGH);
  onReceive(LoRa.parsePacket());
  digitalWrite(RXEN, LOW);
}


  