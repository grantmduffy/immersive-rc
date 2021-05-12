// Run on nano

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <printf.h>

#define IN_PACKET_SIZE 8
#define OUT_PACKET_SIZE 12

#define CE_PIN 7
#define CS_PIN 8

#define BAUDRATE 115200
//#define DEBUG 0

RF24 radio(CE_PIN, CS_PIN);

const uint64_t rx_addr = 0x0000000126;
const uint64_t tx_addr = 0x0000000621;

byte in_packet[IN_PACKET_SIZE];
byte out_packet[OUT_PACKET_SIZE];

void setup() {
  
  printf_begin();
  Serial.begin(BAUDRATE);

  #ifdef DEBUG
  while (!Serial.available()){}
  #endif
  
  radio.begin();
  radio.openReadingPipe(1, rx_addr);
  radio.openWritingPipe(tx_addr);
  radio.setPALevel(RF24_PA_MAX);
  radio.enableDynamicPayloads();
  radio.enableAckPayload();

  #ifdef DEBUG
  radio.printDetails();
  #endif
  
  radio.stopListening();
}

void loop() {
  if (Serial.available() >= IN_PACKET_SIZE){
    Serial.readBytes(in_packet, sizeof(in_packet));
    radio.write(&in_packet, sizeof(in_packet));

    #ifdef DEBUG
    Serial.print("SENT:<");
    Serial.write(in_packet, sizeof(in_packet));
    Serial.println(">");
    #endif
  }
  if (radio.available()){
    
    #ifdef DEBUG
    Serial.print("RECIEVED:<");
    #endif
    
    radio.read(&out_packet, sizeof(out_packet));
    Serial.write(out_packet, sizeof(out_packet));
    
    #ifdef DEBUG
    Serial.println(">");
    #endif
    
  }
  
}
