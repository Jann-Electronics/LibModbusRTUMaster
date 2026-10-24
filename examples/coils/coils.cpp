#include <Arduino.h>

#include "ModbusRTUMaster.h"

using ModbusRTU::Function;

// Pins (adjust according to hardware)
constexpr int8_t PIN_RX = 18;
constexpr int8_t PIN_TX = 17;
constexpr int8_t PIN_DE = 8; // DE and /RE of the RS485 transceiver, tied together

constexpr uint32_t BAUDRATE     = 9600;
constexpr uint8_t SLAVE_ADDRESS = 1;
constexpr uint16_t RELAY_COUNT  = 8;

ModbusRTU::Master master(Serial1, PIN_DE);

void setup() {
  Serial.begin(115200);

  // Wait up to 3 s for the USB serial monitor, so the board also runs without a PC attached
  while (!Serial && millis() < 3000) {
  }

  Serial1.begin(BAUDRATE, SERIAL_8N1, PIN_RX, PIN_TX);
  master.begin(BAUDRATE);
}

void loop() {
  static uint8_t step = 0;

  // Coils are packed 16 per word: coil i is bit (i % 16) of values[i / 16]. Here, a walking bit
  // over 8 relays, starting at coil 0
  const uint16_t pattern[] = {static_cast<uint16_t>(1 << step)};
  ModbusRTU::Response r =
    master.transaction({SLAVE_ADDRESS, Function::WriteMultipleCoils, 0, RELAY_COUNT}, pattern);
  if (!r.ok()) Serial.printf("[Modbus] Write coils failed: %s\n", ModbusRTU::toString(r.error));

  step = (step + 1) % RELAY_COUNT;

  // Read them back
  r = master.transaction({SLAVE_ADDRESS, Function::ReadCoils, 0, RELAY_COUNT});
  if (r.ok()) {
    Serial.print("Relays: ");
    for (uint16_t i = 0; i < RELAY_COUNT; i++) {
      Serial.print(r.getCoil(i) ? '1' : '0');
    }
    Serial.println();
  } else {
    Serial.printf("[Modbus] Read coils failed: %s\n", ModbusRTU::toString(r.error));
  }

  // Discrete inputs are read the same way
  r = master.transaction({SLAVE_ADDRESS, Function::ReadDiscreteInputs, 0, 4});
  if (r.ok()) Serial.printf("Input 0: %s\n", r.getCoil(0) ? "ON" : "OFF");

  // A single coil: any non-zero value turns it on
  const uint16_t off = 0;
  master.transaction({SLAVE_ADDRESS, Function::WriteSingleCoil, 15, 1}, &off);

  delay(500);
}
