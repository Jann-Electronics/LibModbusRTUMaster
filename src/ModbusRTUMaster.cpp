#include "ModbusRTUMaster.h"

namespace ModbusRTU {

namespace {
constexpr uint8_t BITS_PER_CHAR    = 11; // start + 8 data + parity + stop (or 2 stop bits)
constexpr uint8_t MAX_SLAVE        = 247;
constexpr uint8_t EXCEPTION_FLAG   = 0x80; // set in the function code of an exception response
constexpr uint8_t MIN_FRAME_SIZE   = 5;    // exception response: address, function, code, CRC
constexpr uint8_t WRITE_RESPONSE   = 8;    // echo of address, function, start, quantity / value
constexpr uint16_t COIL_ON         = 0xFF00;
constexpr uint32_t T35_CHAR_BIT_US = 38500000UL; // 3.5 characters x 11 bits x 1000000 us

// Above 19200 baud the specification stops scaling T3.5 with the baud rate and fixes it at
// 1750 us: shorter silences would not give a slave time to rearm its receiver between frames
constexpr uint32_t T35_FIXED_ABOVE_BAUD = 19200;
constexpr uint32_t T35_FIXED_US         = 1750;

bool isSingleWrite(Function function) {
  return function == Function::WriteSingleCoil || function == Function::WriteSingleRegister;
}
} // namespace

/* ---------------------------------------------------------------------------------------------- */
/*                                            Response                                            */
/* ---------------------------------------------------------------------------------------------- */
uint16_t Response::getRegister(uint16_t index) const {
  if (index >= request.quantity || index >= MAX_REGISTERS) return 0;
  return values[index];
}

bool Response::getCoil(uint16_t index) const {
  if (index >= request.quantity || index >= MAX_COILS) return false;
  return (values[index / 16] >> (index % 16)) & 0x01;
}

/* ---------------------------------------------------------------------------------------------- */
/*                                            Helpers                                             */
/* ---------------------------------------------------------------------------------------------- */
const char* toString(Error error) {
  switch (error) {
    case Error::None: return "None";
    case Error::IllegalFunction: return "Illegal function";
    case Error::IllegalDataAddress: return "Illegal data address";
    case Error::IllegalDataValue: return "Illegal data value";
    case Error::SlaveDeviceFailure: return "Slave device failure";
    case Error::Acknowledge: return "Acknowledge";
    case Error::SlaveDeviceBusy: return "Slave device busy";
    case Error::NegativeAcknowledge: return "Negative acknowledge";
    case Error::MemoryParityError: return "Memory parity error";
    case Error::GatewayPathUnavailable: return "Gateway path unavailable";
    case Error::GatewayTargetNoResponse: return "Gateway target failed to respond";
    case Error::InvalidDeviceId: return "Invalid device ID";
    case Error::InvalidFunction: return "Invalid function";
    case Error::InvalidCrc: return "Invalid CRC";
    case Error::InvalidFrameSize: return "Invalid frame size";
    case Error::BufferOverflow: return "Buffer overflow";
    case Error::Timeout: return "Timeout";
    case Error::NotReady: return "Not ready";
    case Error::InvalidRequest: return "Invalid request";
    case Error::InvalidResponse: return "Invalid response";
    default: return "Unknown exception";
  }
}

const char* toString(Function function) {
  switch (function) {
    case Function::ReadCoils: return "Read Coils";
    case Function::ReadDiscreteInputs: return "Read Discrete Inputs";
    case Function::ReadHoldingRegisters: return "Read Holding Registers";
    case Function::ReadInputRegisters: return "Read Input Registers";
    case Function::WriteSingleCoil: return "Write Single Coil";
    case Function::WriteSingleRegister: return "Write Single Register";
    case Function::WriteMultipleCoils: return "Write Multiple Coils";
    case Function::WriteMultipleRegisters: return "Write Multiple Registers";
    default: return "Unknown";
  }
}

bool isReadFunction(Function function) {
  switch (function) {
    case Function::ReadCoils:
    case Function::ReadDiscreteInputs:
    case Function::ReadHoldingRegisters:
    case Function::ReadInputRegisters: return true;
    default: return false;
  }
}

bool isRegisterFunction(Function function) {
  switch (function) {
    case Function::ReadHoldingRegisters:
    case Function::ReadInputRegisters:
    case Function::WriteSingleRegister:
    case Function::WriteMultipleRegisters: return true;
    default: return false;
  }
}

uint16_t maxQuantity(Function function) {
  switch (function) {
    case Function::ReadCoils:
    case Function::ReadDiscreteInputs:
    case Function::WriteMultipleCoils: return MAX_COILS;
    case Function::ReadHoldingRegisters:
    case Function::ReadInputRegisters:
    case Function::WriteMultipleRegisters: return MAX_REGISTERS;
    case Function::WriteSingleCoil:
    case Function::WriteSingleRegister: return 1;
    default: return 0;
  }
}

uint16_t crc16(const uint8_t* data, size_t length) {
  uint16_t crc = 0xFFFF;

  for (size_t i = 0; i < length; i++) {
    crc ^= data[i];
    for (uint8_t bit = 0; bit < 8; bit++) {
      crc = (crc & 0x0001) ? (crc >> 1) ^ 0xA001 : crc >> 1;
    }
  }

  return crc;
}

uint32_t t35Micros(uint32_t baudrate) {
  if (baudrate == 0 || baudrate > T35_FIXED_ABOVE_BAUD) return T35_FIXED_US;
  return T35_CHAR_BIT_US / baudrate;
}

/* ---------------------------------------------------------------------------------------------- */
/*                                             Master                                             */
/* ---------------------------------------------------------------------------------------------- */
Master::Master(Stream& serial, int8_t de_pin)
    : _serial(serial)
    , _de_pin(de_pin) {}

bool Master::begin(uint32_t baudrate) {
  if (baudrate == 0) return false;

  _t35_us = t35Micros(baudrate);

  // Upper bound for one response: a full buffer at this baud rate plus T3.5. Without it, a slave
  // that never goes quiet would keep the transaction open forever
  _max_frame_us =
    (static_cast<uint32_t>(BUFFER_SIZE) * BITS_PER_CHAR * 1000000UL) / baudrate + _t35_us;

  if (_de_pin >= 0) {
    digitalWrite(_de_pin, LOW);
    pinMode(_de_pin, OUTPUT);
  }

  _state   = State::Idle;
  _started = true;
  return true;
}

void Master::setTimeout(uint16_t timeout_ms) { _timeout_ms = timeout_ms; }

uint16_t Master::getTimeout() const { return _timeout_ms; }

uint32_t Master::getT35() const { return _t35_us; }

void Master::onBeforeTransmit(TransmitCallback callback) { _before_transmit = callback; }

void Master::onAfterTransmit(TransmitCallback callback) { _after_transmit = callback; }

// ----- Blocking -----

Response Master::transaction(const Request& request, const uint16_t* values) {
  Response response = {};
  response.request  = request;
  response.error    = startRequest(request, values);
  if (response.error != Error::None) return response;

  while (!poll(response)) {
    yield();
  }

  return response;
}

// ----- Non-blocking -----

bool Master::sendRequest(const Request& request, const uint16_t* values) {
  return startRequest(request, values) == Error::None;
}

bool Master::poll(Response& response) {
  if (_state == State::Idle) return false;

  receive();

  const uint32_t now_us = micros();
  Error error;

  if (_state == State::WaitingResponse) {
    if (millis() - _sent_ms < _timeout_ms) return false;
    error = Error::Timeout;
  } else {
    // T3.5 is the silence between frames, not the time a frame takes to arrive: the response is
    // complete only once T3.5 passes without a new byte
    if (now_us - _rx_last_us < _t35_us && now_us - _rx_start_us < _max_frame_us) return false;
    error = _rx_overflow ? Error::BufferOverflow : Error::None;
  }

  response         = {};
  response.request = _pending_request;
  response.error   = (error == Error::None) ? parseFrame(response) : error;

  _state = State::Idle;
  return true;
}

bool Master::isBusy() const { return _state != State::Idle; }

void Master::cancel() { _state = State::Idle; }

const Request& Master::getPendingRequest() const { return _pending_request; }

// ----- Private -----

Error Master::startRequest(const Request& request, const uint16_t* values) {
  if (!_started || _state != State::Idle) return Error::NotReady;
  if (request.address == 0 || request.address > MAX_SLAVE) return Error::InvalidRequest;

  const uint16_t size = buildFrame(request, values);
  if (size == 0) return Error::InvalidRequest;

  memcpy(_echo, _buffer, sizeof(_echo));

  // Drop leftovers, such as a late response to a request that timed out
  flushInput();

  if (_de_pin >= 0) digitalWrite(_de_pin, HIGH);
  if (_before_transmit) _before_transmit();
  _serial.write(_buffer, size);
  _serial.flush();
  if (_after_transmit) _after_transmit();
  if (_de_pin >= 0) digitalWrite(_de_pin, LOW);

  // Drop the echo of a half-duplex transceiver that keeps its receiver enabled. flush() returns
  // once the last bit is on the wire, so the echo is already in the buffer: waiting any longer
  // would eat the start of the response of a fast slave
  flushInput();

  _pending_request = request;
  _rx_length       = 0;
  _rx_overflow     = false;
  _sent_ms         = millis();
  _state           = State::WaitingResponse;
  return Error::None;
}

uint16_t Master::buildFrame(const Request& request, const uint16_t* values) {
  const uint16_t max_quantity = maxQuantity(request.function);
  if (max_quantity == 0) return 0;

  const uint16_t quantity = isSingleWrite(request.function) ? 1 : request.quantity;
  if (quantity == 0 || quantity > max_quantity) return 0;
  if (!isReadFunction(request.function) && values == nullptr) return 0;

  uint16_t size   = 0;
  _buffer[size++] = request.address;
  _buffer[size++] = static_cast<uint8_t>(request.function);
  _buffer[size++] = highByte(request.reg);
  _buffer[size++] = lowByte(request.reg);

  switch (request.function) {
    case Function::ReadCoils:
    case Function::ReadDiscreteInputs:
    case Function::ReadHoldingRegisters:
    case Function::ReadInputRegisters:
    {
      _buffer[size++] = highByte(quantity);
      _buffer[size++] = lowByte(quantity);
    } break;

    case Function::WriteSingleCoil:
    {
      const uint16_t value = values[0] ? COIL_ON : 0x0000;
      _buffer[size++]      = highByte(value);
      _buffer[size++]      = lowByte(value);
    } break;

    case Function::WriteSingleRegister:
    {
      _buffer[size++] = highByte(values[0]);
      _buffer[size++] = lowByte(values[0]);
    } break;

    case Function::WriteMultipleCoils:
    {
      const uint8_t byte_count = (quantity + 7) / 8;
      _buffer[size++]          = highByte(quantity);
      _buffer[size++]          = lowByte(quantity);
      _buffer[size++]          = byte_count;

      // Coils go out 8 per byte, first coil in the LSB. Packing them from the words by shifts
      // keeps the frame independent of the CPU byte order
      for (uint8_t i = 0; i < byte_count; i++) {
        const uint16_t word = values[i / 2];
        _buffer[size++]     = (i % 2) ? highByte(word) : lowByte(word);
      }

      // The specification asks for the unused bits of the last byte to be zero
      if (quantity % 8) _buffer[size - 1] &= (1 << (quantity % 8)) - 1;
    } break;

    case Function::WriteMultipleRegisters:
    {
      _buffer[size++] = highByte(quantity);
      _buffer[size++] = lowByte(quantity);
      _buffer[size++] = static_cast<uint8_t>(quantity * 2);

      for (uint8_t i = 0; i < quantity; i++) {
        _buffer[size++] = highByte(values[i]);
        _buffer[size++] = lowByte(values[i]);
      }
    } break;

    default: return 0;
  }

  const uint16_t crc = crc16(_buffer, size);
  _buffer[size++]    = lowByte(crc);
  _buffer[size++]    = highByte(crc);

  return size;
}

Error Master::parseFrame(Response& response) const {
  const uint16_t length = _rx_length;
  if (length < MIN_FRAME_SIZE) return Error::InvalidFrameSize;

  // CRC first: until it matches, no other byte of the frame can be trusted
  const uint16_t crc = crc16(_buffer, length - 2);
  if (_buffer[length - 2] != lowByte(crc) || _buffer[length - 1] != highByte(crc)) {
    return Error::InvalidCrc;
  }

  if (_buffer[0] != _pending_request.address) return Error::InvalidDeviceId;

  const uint8_t function = static_cast<uint8_t>(_pending_request.function);
  if ((_buffer[1] & ~EXCEPTION_FLAG) != function) return Error::InvalidFunction;

  if (_buffer[1] & EXCEPTION_FLAG) {
    if (length != MIN_FRAME_SIZE) return Error::InvalidFrameSize;
    // Exception code 0 does not exist: reporting it as is would read as Error::None
    return (_buffer[2] == 0) ? Error::InvalidResponse : static_cast<Error>(_buffer[2]);
  }

  switch (_pending_request.function) {
    case Function::ReadCoils:
    case Function::ReadDiscreteInputs:
    case Function::ReadHoldingRegisters:
    case Function::ReadInputRegisters:
    {
      const uint16_t byte_count = _buffer[2];
      if (length != MIN_FRAME_SIZE + byte_count) return Error::InvalidFrameSize;

      const bool registers = isRegisterFunction(_pending_request.function);
      const uint16_t expect =
        registers ? _pending_request.quantity * 2 : (_pending_request.quantity + 7) / 8;
      if (byte_count != expect) return Error::InvalidResponse;

      const uint8_t* data = &_buffer[3];
      if (registers) {
        for (uint8_t i = 0; i < _pending_request.quantity; i++) {
          response.values[i] = (static_cast<uint16_t>(data[2 * i]) << 8) | data[2 * i + 1];
        }
      } else {
        // 8 coils per byte, first coil in the LSB; 2 bytes per word, first byte in the low half
        for (uint8_t i = 0; i < byte_count; i++) {
          if (i % 2) {
            response.values[i / 2] |= static_cast<uint16_t>(data[i]) << 8;
          } else {
            response.values[i / 2] = data[i];
          }
        }
      }

      response.data_bytes = static_cast<uint8_t>(byte_count);
    } break;

    default:
    {
      // Write responses echo the start address and the quantity (or the value written)
      if (length != WRITE_RESPONSE) return Error::InvalidFrameSize;
      if (memcmp(_buffer, _echo, sizeof(_echo)) != 0) return Error::InvalidResponse;
    } break;
  }

  return Error::None;
}

void Master::receive() {
  while (_serial.available() > 0) {
    const int value = _serial.read();
    if (value < 0) break;

    if (_state == State::WaitingResponse) {
      // A 0x00 before the response is the glitch of the bus when DE is released: without bias
      // resistors the line drops and the UART reads a zero byte. No slave answers from address 0,
      // and keeping it would make the response look like a frame from the wrong device
      if (value == 0x00) continue;

      _state       = State::Receiving;
      _rx_start_us = micros();
    }

    if (_rx_length < BUFFER_SIZE) {
      _buffer[_rx_length++] = static_cast<uint8_t>(value);
    } else {
      _rx_overflow = true;
    }

    _rx_last_us = micros();
  }
}

void Master::flushInput() {
  while (_serial.available() > 0) {
    _serial.read();
  }
}

} // namespace ModbusRTU
