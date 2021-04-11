// Run on Teensy 4.0

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <printf.h>

#define BUFF_SIZE 32

RF24 radio(9, 10); // CE, CSN

const uint64_t rx_addr = 0x0000000621;
const uint64_t tx_addr = 0x0000000126;

char message[BUFF_SIZE] = "Hello world!";
uint8_t i = 0;

void setup() {
  Serial.begin(115200);
  printf_begin();
  while (!Serial.available()){}
  radio.begin();
  radio.openReadingPipe(1, rx_addr);
  radio.openWritingPipe(tx_addr);
  radio.setPALevel(RF24_PA_MAX);
  radio.enableDynamicPayloads();
  radio.enableAckPayload();
  radio.startListening();
  radio.printDetails();
  Serial.println("Teensy Ready");
}

void loop() {
  if (radio.available()){
    radio.read(&message, sizeof(message));
    Serial.print("Recieved: ");
    Serial.println(message);
    radio.writeAckPayload(1, &message, strlen(message));
  }
}
