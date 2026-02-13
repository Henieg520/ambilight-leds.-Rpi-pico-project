#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

#define NUM_LEDS_A 100
#define NUM_LEDS_B 25

#define PIN_A 18
#define PIN_B 19

#define START_A 0xAA
#define START_B 0xBB

#define BAUDRATE 1000000

Adafruit_NeoPixel stripA(NUM_LEDS_A, PIN_A, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel stripB(NUM_LEDS_B, PIN_B, NEO_GRB + NEO_KHZ800);

enum ParserState {
  WAIT_START,
  WAIT_LEN1,
  WAIT_LEN2,
  WAIT_PAYLOAD,
  WAIT_CRC
};

struct FrameParser {
  ParserState state = WAIT_START;
  uint16_t length = 0;
  uint16_t index = 0;
  uint8_t *payload;
  uint16_t max_len;
  uint8_t start_byte;
};

uint8_t payloadA[NUM_LEDS_A * 3];
uint8_t payloadB[NUM_LEDS_B * 3];

FrameParser parserA;
FrameParser parserB;

uint8_t crc8(uint8_t *data, uint16_t len) {
  uint8_t crc = 0x00;
  for (uint16_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t j = 0; j < 8; j++) {
      crc = (crc & 0x80) ? (crc << 1) ^ 0x07 : (crc << 1);
    }
  }
  return crc;
}

void renderA(uint8_t *data, uint16_t len) {
  uint16_t leds = min(len / 3, NUM_LEDS_A);
  for (uint16_t i = 0; i < leds; i++) {
    stripA.setPixelColor(i,
      stripA.Color(data[i*3], data[i*3+1], data[i*3+2]));
  }
  stripA.show();
}

void renderB(uint8_t *data, uint16_t len) {
  uint16_t leds = min(len / 3, NUM_LEDS_B);
  for (uint16_t i = 0; i < leds; i++) {
    stripB.setPixelColor(i,
      stripB.Color(data[i*3], data[i*3+1], data[i*3+2]));
  }
  stripB.show();
}

void handle_parser(FrameParser &p, uint8_t byte) {
  switch (p.state) {

    case WAIT_START:
      if (byte == p.start_byte) {
        p.state = WAIT_LEN1;
      }
      break;

    case WAIT_LEN1:
      p.length = byte;
      p.state = WAIT_LEN2;
      break;

    case WAIT_LEN2:
      p.length |= (byte << 8);
      if (p.length > p.max_len) {
        p.state = WAIT_START;
      } else {
        p.index = 0;
        p.state = WAIT_PAYLOAD;
      }
      break;

    case WAIT_PAYLOAD:
      p.payload[p.index++] = byte;
      if (p.index >= p.length) {
        p.state = WAIT_CRC;
      }
      break;

    case WAIT_CRC:
      if (crc8(p.payload, p.length) == byte) {
        if (p.start_byte == START_A)
          renderA(p.payload, p.length);
        else
          renderB(p.payload, p.length);
      }
      p.state = WAIT_START;
      break;
  }
}

void setup() {
  Serial.begin(BAUDRATE);

  stripA.begin();
  stripB.begin();
  stripA.show();
  stripB.show();

  parserA.payload = payloadA;
  parserA.max_len = sizeof(payloadA);
  parserA.start_byte = START_A;

  parserB.payload = payloadB;
  parserB.max_len = sizeof(payloadB);
  parserB.start_byte = START_B;
}

void loop() {
  while (Serial.available()) {
    uint8_t b = Serial.read();
    handle_parser(parserA, b);
    handle_parser(parserB, b);
  }
}
