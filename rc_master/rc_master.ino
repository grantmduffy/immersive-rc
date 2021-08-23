// Run on nano

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <printf.h>

#define IN_PACKET_SIZE 8
#define OUT_PACKET_SIZE 16

#define CE_PIN 7
#define CS_PIN 8
#define IR_PIN 2
#define IR_INDICATOR_PIN 3

#define MIN_LAP 2000

#define BAUDRATE 115200
//#define DEBUG 0

RF24 radio(CE_PIN, CS_PIN);

const uint64_t rx_addr = 0x0000000126;
const uint64_t tx_addr = 0x0000000621;

unsigned long t0;

union {
  struct {
    char header[4];
    unsigned long t;
  } message;
  byte buffer[OUT_PACKET_SIZE];
} timing_packet;

byte in_packet[IN_PACKET_SIZE];
byte out_packet[OUT_PACKET_SIZE];

void setup() {

  
  printf_begin();
  Serial.begin(BAUDRATE);

  #ifdef DEBUG
  while (!Serial.available()){}
  #endif

  pinMode(IR_PIN, INPUT);
  pinMode(IR_INDICATOR_PIN, OUTPUT);
  char header[] = {'T', 'I', 'M', 'E'};
  for (int i = 0; i < 4; i++){
    timing_packet.message.header[i] = header[i];
  }
  t0 = millis();
//  attachInterrupt(digitalPinToInterrupt(IR_PIN), lap, RISING);
  
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
  digitalWrite(IR_INDICATOR_PIN, digitalRead(IR_PIN));
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
    noInterrupts();
    Serial.write(out_packet, sizeof(out_packet));
    interrupts();
    #ifdef DEBUG
    Serial.println(">");
    #endif
    
  }
  
}

void lap() {
  noInterrupts();
  unsigned long t = millis();
  if (t - t0 > MIN_LAP){
//    digitalWrite(IR_INDICATOR_PIN, HIGH);
    timing_packet.message.t = t - t0;
    Serial.write(timing_packet.buffer, sizeof(timing_packet.buffer));
    t0 = t;
//    digitalWrite(IR_INDICATOR_PIN, LOW);
  }
  interrupts();
}
