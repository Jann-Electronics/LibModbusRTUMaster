#include <Arduino.h>

#include "ModbusRTUMaster.h"

using ModbusRTU::Function;

// Pins (adjust according to hardware)
constexpr int8_t PIN_RX = 18;
constexpr int8_t PIN_TX = 17;
constexpr int8_t PIN_DE = 8; // DE and /RE of the RS485 transceiver, tied together

constexpr uint32_t BAUDRATE     = 9600;
constexpr uint8_t SLAVE_ADDRESS = 1;

ModbusRTU::Master master(Serial1, PIN_DE);

void setup() {
  Serial.begin(115200);

  // Wait up to 3 s for the USB serial monitor, so the board also runs without a PC attached
  while (!Serial && millis() < 3000) {
  }

  // The serial port is started by the application. Baud rate and parity must match the slave
  Serial1.begin(BAUDRATE, SERIAL_8N1, PIN_RX, PIN_TX);
  master.begin(BAUDRATE);
  master.setTimeout(500);

  Serial.printf("[Modbus] Master started, T3.5 = %u us\n", master.getT35());
}

void loop() {
  // Read 4 holding registers starting at address 0
  ModbusRTU::Response r = master.transaction({SLAVE_ADDRESS, Function::ReadHoldingRegisters, 0, 4});

  if (r.ok()) {
    for (uint8_t i = 0; i < 4; i++) {
      // Registers are raw 16-bit words: cast to int16_t if the slave stores signed values
      Serial.printf("  HR[%u] = %u (%d)\n",
        r.request.reg + i,
        r.getRegister(i),
        static_cast<int16_t>(r.getRegister(i)));
    }
  } else {
    Serial.printf("[Modbus] Read failed: %s\n", ModbusRTU::toString(r.error));
  }

  // Write a single register
  const uint16_t setpoint = 250;
  r = master.transaction({SLAVE_ADDRESS, Function::WriteSingleRegister, 10, 1}, &setpoint);
  if (!r.ok()) Serial.printf("[Modbus] Write failed: %s\n", ModbusRTU::toString(r.error));

  // Write 3 consecutive registers
  const uint16_t values[] = {100, 200, 300};
  r = master.transaction({SLAVE_ADDRESS, Function::WriteMultipleRegisters, 20, 3}, values);
  if (!r.ok()) Serial.printf("[Modbus] Write failed: %s\n", ModbusRTU::toString(r.error));

  delay(1000);
}
