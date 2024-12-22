#define NUM_US 1
#define WINDOW_SIZE 20

const uint32_t US_TRIG[NUM_US] = { 32 };
const uint32_t US_ECHO[NUM_US] = { 31 };

void setup() {
  Serial.begin(115200);
  for (uint32_t ch = 0; ch < NUM_US; ++ch) {
    pinMode(US_TRIG[ch], OUTPUT);
    pinMode(US_ECHO[ch], INPUT);
  }
}

void loop() {
  float us = usToInches(readUltrasonicRaw(0));
  Serial.println(us);
  delay(100);
}

inline float usToInches(uint32_t us) {
  return us / 58.0f;
}

uint32_t avgAccs[NUM_US] = {};
uint32_t avgIdxs[NUM_US] = {};
uint32_t avgBufs[NUM_US][WINDOW_SIZE] = {};

uint32_t movingAverage(uint32_t value, uint32_t *buffer, uint32_t *accumulator, uint32_t *index) {
  *accumulator -= buffer[*index];         // Remove the oldest entry from the sum
  buffer[*index] = value;                 // Add the newest reading to the window
  *accumulator += value;                  // Add the newest reading to the sum
  *index = (*index + 1) % WINDOW_SIZE;    // Increment the index, and wrap to 0 if it exceeds the window size
  return *accumulator / WINDOW_SIZE; 
}

uint32_t readUltrasonicAveraged(uint32_t ch) {
  uint32_t us = readUltrasonicRaw(ch);
  return movingAverage(us, avgBufs[ch], &avgAccs[ch], &avgIdxs[ch]);
}

uint32_t readUltrasonicRaw(uint32_t ch) {
  digitalWrite(US_TRIG[ch], LOW);
  delayMicroseconds(2);  /* Ensure pin is reset */
  digitalWrite(US_TRIG[ch], HIGH);
  delayMicroseconds(10);  /* 10us delay to start module */
  digitalWrite(US_TRIG[ch], LOW);
  return pulseIn(US_ECHO[ch], HIGH, 1000000L);
}