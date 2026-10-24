#include <Arduino.h>

#include "ModbusRTUMaster.h"

using ModbusRTU::Function;

// Pins (adjust according to hardware)
constexpr int8_t PIN_RX  = 18;
constexpr int8_t PIN_TX  = 17;
constexpr int8_t PIN_DE  = 8; // DE and /RE of the RS485 transceiver, tied together
constexpr int8_t PIN_LED = 2;

constexpr uint32_t BAUDRATE  = 9600;
constexpr uint32_t PERIOD_MS = 200; // time between requests

// Requests sent in turn to three slaves
const ModbusRTU::Request requests[] = {
  {1, Function::ReadHoldingRegisters, 0, 1},
  {2, Function::ReadHoldingRegisters, 0, 1},
  {3, Function::ReadInputRegisters,   4, 2},
};
constexpr uint8_t REQUEST_COUNT = sizeof(requests) / sizeof(requests[0]);

ModbusRTU::Master master(Serial1, PIN_DE);

void setup() {
  Serial.begin(115200);

  // Wait up to 3 s for the USB serial monitor, so the board also runs without a PC attached
  while (!Serial && millis() < 3000) {
  }

  pinMode(PIN_LED, OUTPUT);

  Serial1.begin(BAUDRATE, SERIAL_8N1, PIN_RX, PIN_TX);
  master.begin(BAUDRATE);
  master.setTimeout(300);
}

void loop() {
  static uint8_t next       = 0;
  static uint32_t last_sent = 0;

  // Start the next request when the bus is free and the period elapsed
  if (!master.isBusy() && millis() - last_sent >= PERIOD_MS) {
    if (master.sendRequest(requests[next])) {
      last_sent = millis();
    }
    next = (next + 1) % REQUEST_COUNT;
  }

  // poll() returns true once the pending transaction finished, whatever the result
  ModbusRTU::Response r;
  if (master.poll(r)) {
    if (r.ok()) {
      for (uint16_t i = 0; i < r.request.quantity; i++) {
        Serial.printf("Slave %u, register %u: %u\n",
          r.request.address,
          r.request.reg + i,
          r.getRegister(i));
      }
    } else {
      Serial.printf("Slave %u: %s\n", r.request.address, ModbusRTU::toString(r.error));
    }
  }

  // The loop never blocks waiting for a slave, so the rest of the program keeps running
  static uint32_t last_blink = 0;
  if (millis() - last_blink >= 500) {
    last_blink = millis();
    digitalWrite(PIN_LED, !digitalRead(PIN_LED));
  }
}
