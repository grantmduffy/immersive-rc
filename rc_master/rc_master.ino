// Run on nano

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <printf.h>

#define BUFF_SIZE 32

RF24 radio(7, 8); // CE, CSN

const uint64_t rx_addr = 0x0000000126;
const uint64_t tx_addr = 0x0000000621;

char message[BUFF_SIZE] = "Hello world!";
char rx_message[BUFF_SIZE] = "";
char *next_char = message;
uint8_t i = 0;

void setup() {
  Serial.begin(112500);
  printf_begin();
  while (!Serial.available()){}
  radio.begin();
  radio.openReadingPipe(1, rx_addr);
  radio.openWritingPipe(tx_addr);
  radio.setPALevel(RF24_PA_MAX);
  radio.enableDynamicPayloads();
  radio.enableAckPayload();
  radio.stopListening();
  radio.printDetails();
  Serial.println("Nano Ready");
}

void loop() {
  if (Serial.available()){
    char ch = Serial.read();
    if (ch == '\r' || next_char == message + BUFF_SIZE - 1){
      next_char = message;
      Serial.print("Sent: ");
      Serial.println(message);
      radio.write(&message, strlen(message));
    } else {
      *next_char++ = ch;
      *next_char = NULL;
      Serial.print(message);
      Serial.print('\r');
    }
  }
  if (radio.available()){
    radio.read(&message, sizeof(message));
    Serial.print("Ack: ");
    Serial.println(message);
  }
  
}
