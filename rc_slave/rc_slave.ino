// Run on Teensy 4.0

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <printf.h>
#define CE_PIN 9
#define CS_PIN 10

#define BATTERY_LEVEL_PIN 14
#define STEERING_IN_PIN 15
#define STEERING_A_PIN 5
#define STEERING_B_PIN 6
#define STEERING_PWM_PIN 7
#define THROTTLE_PWM_PIN 8
#define STEERING_PWM_FREQ 36621.09
#define THROTTLE_PWM_FREQ 10000.0

#define dt 0.01

// Control system stuff
unsigned long t;
float x = 0.5;
float err, err_1, f, f_1, y, y_1, y_2, u, u_1;
float z = 30.0, p = 300.0, K = 10.0;
float K_sys = 200.0;
float lp_a = 0.2;

// Radio stuff
RF24 radio(CE_PIN, CS_PIN);
const uint64_t rx_addr = 0x0000000621;
const uint64_t tx_addr = 0x0000000126;

// Communication packets
union {
  struct {
    float steering;
    float throttle;
  } message;
  byte buffer[8];
} in_packet;

union {
  struct {
    float force_feedback;
    uint16_t steering_position;
    uint16_t battery_level;
    float steering_force;
  } message;
  byte buffer[12];
} out_packet;


void setup() {

  printf_begin();
  Serial.begin(115200);
//  while (!Serial.available()){}
//  while (!Serial){}
  delay(2000);
  Serial.println("READY");

  //  Setup pins/motors
  pinMode(BATTERY_LEVEL_PIN, INPUT);
  pinMode(STEERING_IN_PIN, INPUT);
  pinMode(STEERING_A_PIN, OUTPUT);
  pinMode(STEERING_B_PIN, OUTPUT);
  pinMode(STEERING_PWM_PIN, OUTPUT);
  pinMode(THROTTLE_PWM_PIN, OUTPUT);
  analogWriteFrequency(STEERING_PWM_PIN, STEERING_PWM_FREQ);
  analogWriteFrequency(THROTTLE_PWM_PIN, THROTTLE_PWM_FREQ);
  analogWriteResolution(12);
  analogReadRes(12);
  setSteeringForce(0.0);
  setThrottle(0.0);

  // Setup radio
  radio.begin();
  radio.openReadingPipe(1, rx_addr);
  radio.openWritingPipe(tx_addr);
  radio.setPALevel(RF24_PA_MAX);
  radio.enableDynamicPayloads();
  radio.enableAckPayload();
  radio.printDetails();
  radio.startListening();

  t = millis();
}

void loop() {

  // Read from radio, send ack
  if (radio.available()){
    radio.read(&in_packet.buffer, sizeof(in_packet.buffer));
  }

  // Sample  
  if (millis() - t >= dt * 1000) {
    t = millis();
    
    x = in_packet.message.steering;
  
    y_2 = y_1;
    y_1 = y;
    y = getSteeringPosition();
    err_1 = err;
    err = x - y;
    f_1 = f;
    f = (K * p * (z + 1.0 / dt) * err - K * p * err_1 / dt + z * f_1 / dt) / (z * (p + 1.0 / dt));
    
    setSteeringForce(f);
    setThrottle(in_packet.message.throttle);
  
    u_1 = u;
    u = (y - 2 * y_1 + y_2) / (dt * dt * K_sys) - f;
    u = lp_a * u + (1 - lp_a) * u_1;
  
    out_packet.message.force_feedback = u;
    out_packet.message.steering_force = f;
    out_packet.message.steering_position = analogRead(STEERING_IN_PIN);
    out_packet.message.battery_level = analogRead(BATTERY_LEVEL_PIN);
    radio.writeAckPayload(1, &out_packet.buffer, sizeof(out_packet.buffer));
  }
  
}



void setThrottle(float throttle){
  throttle = max(min(throttle, 1.0), 0);
  analogWrite(THROTTLE_PWM_PIN, round(4095.0 * throttle));
}

void setSteeringForce(float force){  // force: [-1 -> 1]
  force = min(force, 1);
  force = max(force, -1);
  int val = round(force * 4095);
  if (val > 0){
    digitalWrite(STEERING_A_PIN, LOW);
    digitalWrite(STEERING_B_PIN, HIGH);
  } else {
    digitalWrite(STEERING_A_PIN, HIGH);
    digitalWrite(STEERING_B_PIN, LOW);
  }
  analogWrite(STEERING_PWM_PIN, abs(val));
}

float getSteeringPosition(){
  return (float) analogRead(STEERING_IN_PIN) / 4095;
}
