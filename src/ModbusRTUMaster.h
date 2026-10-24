#pragma once

#include <Arduino.h>

// Size of the frame buffer in bytes, from 16 to 256 (the Modbus RTU maximum). It sets how many
// registers or coils fit in one request: 27 registers / 432 coils with the default of 64, 123
// registers / 1968 coils with 256. Define it in build_flags, e.g. -DMODBUS_RTU_BUFFER_SIZE=256
#ifndef MODBUS_RTU_BUFFER_SIZE
#define MODBUS_RTU_BUFFER_SIZE 64
#endif

namespace ModbusRTU {

/* ---------------------------------------------------------------------------------------------- */
/*                                             Limits                                             */
/* ---------------------------------------------------------------------------------------------- */

constexpr uint16_t BUFFER_SIZE = MODBUS_RTU_BUFFER_SIZE;
static_assert(BUFFER_SIZE >= 16 && BUFFER_SIZE <= 256,
  "MODBUS_RTU_BUFFER_SIZE must be between 16 and 256");

// The largest frame is a Write Multiple request: 9 bytes of address, function, start, quantity,
// byte count and CRC, plus the data
constexpr uint8_t MAX_REGISTERS = (BUFFER_SIZE - 9) / 2;
constexpr uint16_t MAX_COILS    = MAX_REGISTERS * 16;

constexpr uint16_t DEFAULT_TIMEOUT_MS = 1000;

/* ---------------------------------------------------------------------------------------------- */
/*                                             Enums                                              */
/* ---------------------------------------------------------------------------------------------- */

// Supported function codes
enum class Function : uint8_t {
  ReadCoils              = 0x01,
  ReadDiscreteInputs     = 0x02,
  ReadHoldingRegisters   = 0x03,
  ReadInputRegisters     = 0x04,
  WriteSingleCoil        = 0x05,
  WriteSingleRegister    = 0x06,
  WriteMultipleCoils     = 0x0F,
  WriteMultipleRegisters = 0x10,
};

// Result of a transaction. Codes 0x01 to 0x0B are exceptions returned by the slave; codes from 0xE0
// are detected by the master
enum class Error : uint8_t {
  None = 0x00,

  // Exceptions returned by the slave
  IllegalFunction         = 0x01,
  IllegalDataAddress      = 0x02,
  IllegalDataValue        = 0x03,
  SlaveDeviceFailure      = 0x04,
  Acknowledge             = 0x05,
  SlaveDeviceBusy         = 0x06,
  NegativeAcknowledge     = 0x07,
  MemoryParityError       = 0x08,
  GatewayPathUnavailable  = 0x0A,
  GatewayTargetNoResponse = 0x0B,

  // Detected by the master
  InvalidDeviceId  = 0xE0, // response from another slave address
  InvalidFunction  = 0xE1, // response function code does not match the request
  InvalidCrc       = 0xE2, // CRC mismatch: noise on the bus or a baud rate / parity mismatch
  InvalidFrameSize = 0xE3, // response length does not match the function
  BufferOverflow   = 0xE4, // response longer than MODBUS_RTU_BUFFER_SIZE
  Timeout          = 0xE5, // no response within the timeout
  NotReady         = 0xE6, // begin() not called, or another request is still pending
  InvalidRequest   = 0xE7, // invalid address, quantity or function, or missing write values
  InvalidResponse  = 0xE8, // well-formed response with a byte count or echo that does not match
};

/* ---------------------------------------------------------------------------------------------- */
/*                                       Request / Response                                       */
/* ---------------------------------------------------------------------------------------------- */

struct Request {
  uint8_t address;   // slave address, 1 to 247
  Function function; // function code
  uint16_t reg;      // first register or coil, as a 0-based protocol address
  uint16_t quantity; // registers or coils to read or write. Ignored by the single write functions
};

struct Response {
  Request request;    // request this response answers
  Error error;        // Error::None on success
  uint8_t data_bytes; // data bytes received (read functions only)

  // Read registers, or read coils packed 16 per word: coil i is bit (i % 16) of values[i / 16].
  // Prefer getRegister() / getCoil(), which check the index against the quantity requested
  uint16_t values[MAX_REGISTERS];

  bool ok() const { return error == Error::None; }

  // Register `index` of the response, counted from request.reg. 0 if out of range
  uint16_t getRegister(uint16_t index) const;

  // Coil or discrete input `index` of the response, counted from request.reg. false if out of range
  bool getCoil(uint16_t index) const;
};

/* ---------------------------------------------------------------------------------------------- */
/*                                            Helpers                                             */
/* ---------------------------------------------------------------------------------------------- */

const char* toString(Error error);
const char* toString(Function function);

// true for the functions that read (0x01 to 0x04)
bool isReadFunction(Function function);

// true for the functions that access registers, false for the ones that access coils / inputs
bool isRegisterFunction(Function function);

// Largest quantity a single request of this function can carry with the current buffer size.
// 1 for the single write functions, 0 for an unsupported function code
uint16_t maxQuantity(Function function);

// Modbus CRC-16 (polynomial 0xA001, initial value 0xFFFF). The frame carries it low byte first
uint16_t crc16(const uint8_t* data, size_t length);

// Silent interval that ends a frame (3.5 characters), in microseconds. Fixed at 1750 us above
// 19200 baud, as the Modbus specification requires
uint32_t t35Micros(uint32_t baudrate);

/* ---------------------------------------------------------------------------------------------- */
/*                                             Master                                             */
/* ---------------------------------------------------------------------------------------------- */

// Called right before a request is transmitted and right after its last bit leaves the UART, to
// drive the DE / RE pins of the RS485 transceiver when a plain GPIO is not enough
using TransmitCallback = void (*)();

// Modbus RTU master over any Stream (HardwareSerial, SoftwareSerial, ...).
//
// Transactions run in one of two ways:
//   - Blocking:     transaction() sends the request and returns the response.
//   - Non-blocking: sendRequest() sends the request and poll(), called from loop(), returns true
//                   once the response arrived, failed or timed out.
// Only one transaction can be pending at a time. Not thread-safe: guard it externally if several
// tasks use the same master.
class Master {
  public:
  // serial: port started by the application with serial.begin()
  // de_pin: transceiver driver enable (DE and /RE tied together), high while transmitting. -1 if
  //         not used: the transceiver is driven by callbacks, by the UART itself or not at all
  explicit Master(Stream& serial, int8_t de_pin = -1);

  // Configures the DE pin and the frame timing. baudrate must match the one passed to
  // serial.begin(). Returns false (and changes nothing) if it is 0
  bool begin(uint32_t baudrate);

  // ----- Configuration -----

  // Time the slave has to start answering, counted from the end of the request (default 1000 ms)
  void setTimeout(uint16_t timeout_ms);
  uint16_t getTimeout() const;

  // Silent interval that ends a frame at the current baud rate, in microseconds
  uint32_t getT35() const;

  void onBeforeTransmit(TransmitCallback callback);
  void onAfterTransmit(TransmitCallback callback);

  // ----- Blocking -----

  // Sends a request and waits for the response, for up to the timeout plus the time the response
  // takes to arrive. values holds the data of the write functions (see sendRequest())
  Response transaction(const Request& request, const uint16_t* values = nullptr);

  // ----- Non-blocking -----

  // Sends a request and returns without waiting for the response. values holds the data of the
  // write functions: the register values, or the coils packed 16 per word (coil i is bit (i % 16)
  // of values[i / 16]). For Write Single Coil, any non-zero values[0] turns the coil on.
  // Returns false (and sends nothing) if begin() was not called, another request is pending, or
  // the request is invalid
  bool sendRequest(const Request& request, const uint16_t* values = nullptr);

  // Advances the pending transaction. Returns true once it finishes, with the result in response;
  // false while it is still running or if nothing is pending. Call it often: it detects the end of
  // the response by the 3.5 character silence that follows it
  bool poll(Response& response);

  // true from sendRequest() until poll() returns the response
  bool isBusy() const;

  // Drops the pending transaction. A response that arrives later is discarded by the next request
  void cancel();

  const Request& getPendingRequest() const;

  private:
  enum class State : uint8_t {
    Idle,
    WaitingResponse, // request sent, no byte of the response yet
    Receiving,       // receiving the response, waiting for the silence that ends it
  };

  Stream& _serial;
  int8_t _de_pin;
  TransmitCallback _before_transmit = nullptr;
  TransmitCallback _after_transmit  = nullptr;

  // ----- Configuration -----
  bool _started          = false;
  uint16_t _timeout_ms   = DEFAULT_TIMEOUT_MS;
  uint32_t _t35_us       = 0;
  uint32_t _max_frame_us = 0;

  // ----- Pending transaction -----
  State _state             = State::Idle;
  Request _pending_request = {};
  uint8_t _echo[6]         = {}; // first bytes of the request, echoed by the write responses
  uint32_t _sent_ms        = 0;
  uint32_t _rx_start_us    = 0;
  uint32_t _rx_last_us     = 0;
  uint16_t _rx_length      = 0;
  bool _rx_overflow        = false;
  uint8_t _buffer[BUFFER_SIZE];

  Error startRequest(const Request& request, const uint16_t* values);
  uint16_t buildFrame(const Request& request, const uint16_t* values);
  Error parseFrame(Response& response) const;
  void receive();
  void flushInput();
};

} // namespace ModbusRTU
