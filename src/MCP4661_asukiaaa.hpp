#pragma once

#include <wire_asukiaaa.h>

#include <bits_asukiaaa.hpp>

namespace MCP4661_asukiaaa {

namespace DeviceAddress {
const uint8_t A0H_A1H_A2H = 0b0101111;
const uint8_t A0L_A1H_A2H = 0b0101110;
const uint8_t A0H_A1L_A2H = 0b0101101;
const uint8_t A0L_A1L_A2H = 0b0101100;
const uint8_t A0H_A1H_A2L = 0b0101011;
const uint8_t A0L_A1H_A2L = 0b0101010;
const uint8_t A0H_A1L_A2L = 0b0101001;
const uint8_t A0L_A1L_A2L = 0b0101000;
}  // namespace DeviceAddress

namespace Register {
const uint8_t Wiper0RAM = 0x00;
const uint8_t Wiper1RAM = 0x01;
const uint8_t Wiper0EEPROM = 0x02;
const uint8_t Wiper1EEPROM = 0x03;
const uint8_t Tcon = 0x04;
const uint8_t Status = 0x05;
}  // namespace Register

namespace Command {
const uint8_t Write = 0b0;
const uint8_t Increment = 0b01;
const uint8_t Decrement = 0b10;
const uint8_t Read = 0b11;
}  // namespace Command

namespace Error {
uint8_t undefinedChannel = 10;
}

const uint16_t MAX_WIPER_VAL = 0x100;

class ClassCommon {
 public:
  int readResult = -1;
  virtual uint16_t toU16() const = 0;
  virtual void updateFromU16(const uint16_t) = 0;
  virtual void print(Stream* serial) const = 0;
  void println(Stream* serial) const {
    print(serial);
    serial->println();
  }
};

class Status : public ClassCommon {
 public:
  bool EEWA, WL0, WL1, WP;

  void updateFromU16(const uint16_t byte) {
    using bits_asukiaaa::isBitTrue;
    EEWA = isBitTrue(byte, 3);
    WL1 = isBitTrue(byte, 2);
    WL0 = isBitTrue(byte, 1);
    WP = isBitTrue(byte, 0);
  }

  uint16_t toU16() const {
    uint8_t byte = 0;
    using bits_asukiaaa::setBitTrue;
    if (EEWA) setBitTrue(&byte, 3);
    if (WL1) setBitTrue(&byte, 2);
    if (WL0) setBitTrue(&byte, 1);
    if (WP) setBitTrue(&byte, 0);
    return 0b1111110000 | byte;
  }

  void print(Stream* serial) const {
    serial->print("EEWA " + String(EEWA));
    serial->print(", WL1 " + String(WL1));
    serial->print(", WL0 " + String(WL0));
    serial->print(", WP " + String(WP));
  }
};

class TCON : public ClassCommon {
 public:
  bool GCEN, R1HW, R1A, R1W, R1B, R0HW, R0A, R0W, R0B;

  void updateFromU16(const uint16_t byte) {
    using bits_asukiaaa::isBitTrue;
    GCEN = isBitTrue(byte >> 8, 0);
    R1HW = isBitTrue(byte, 7);
    R1A = isBitTrue(byte, 6);
    R1W = isBitTrue(byte, 5);
    R1B = isBitTrue(byte, 4);
    R0HW = isBitTrue(byte, 3);
    R0A = isBitTrue(byte, 2);
    R0W = isBitTrue(byte, 1);
    R0B = isBitTrue(byte, 0);
  }

  uint16_t toU16() const {
    uint8_t byteHigher, byteLower;
    using bits_asukiaaa::setBitTrue;
    if (GCEN) setBitTrue(&byteHigher, 0);
    if (R1HW) setBitTrue(&byteLower, 7);
    if (R1A) setBitTrue(&byteLower, 6);
    if (R1W) setBitTrue(&byteLower, 5);
    if (R1B) setBitTrue(&byteLower, 4);
    if (R0HW) setBitTrue(&byteLower, 3);
    if (R0A) setBitTrue(&byteLower, 2);
    if (R0W) setBitTrue(&byteLower, 1);
    if (R0B) setBitTrue(&byteLower, 0);
    return ((uint16_t)byteHigher << 8) | byteLower;
  }

  void print(Stream* serial) const {
    serial->print("GCEN " + String(GCEN));
    serial->print(", R1HW " + String(R1HW));
    serial->print(", R1A " + String(R1A));
    serial->print(", R1W " + String(R1W));
    serial->print(", R1B " + String(R1B));
    serial->print(", R0HW " + String(R0HW));
    serial->print(", R0A " + String(R0A));
    serial->print(", R0W " + String(R0W));
    serial->print(", R0B " + String(R0B));
  }
};

class Core {
 public:
  TwoWire* wire;
  const uint8_t address;
  Core(TwoWire* wire, uint8_t address) : wire(wire), address(address) {}

  int writeWiperRAM(uint8_t channel, uint16_t val) {
    if (channel >= 2) {
      return Error::undefinedChannel;
    }
    return writeToRegister(Register::Wiper0RAM + channel, val);
  }

  int writeWiperEEPROM(uint8_t channel, uint16_t val) {
    if (channel >= 2) {
      return Error::undefinedChannel;
    }
    auto reg = Register::Wiper0EEPROM + channel;
    return writeToRegister(reg, val);
  }

  int readWiperRAM(uint8_t channel, uint16_t* val) {
    if (channel >= 2) {
      return Error::undefinedChannel;
    }
    return readFromRegister(Register::Wiper0RAM + channel, val);
  }

  int readWiperEEPROM(uint8_t channel, uint16_t* val) {
    if (channel >= 2) {
      return Error::undefinedChannel;
    }
    return readFromRegister(Register::Wiper0EEPROM + channel, val);
  }

  int readTCON(TCON* tcon) { return readClass(Register::Tcon, tcon); }
  int readStatus(Status* status) { return readClass(Register::Status, status); }
  int writeTCON(const TCON& tcon) { return writeClass(Register::Tcon, tcon); }
  int writeStatus(const Status& status) {
    return writeClass(Register::Status, status);
  }

 private:
  int readClass(uint8_t regAddress, ClassCommon* instance) {
    uint16_t val;
    auto result = readFromRegister(regAddress, &val);
    instance->readResult = result;
    if (result != 0) {
      return result;
    }
    instance->updateFromU16(val);
    return result;
  }

  int writeClass(uint8_t regAddress, const ClassCommon& instance) {
    return writeToRegister(regAddress, instance.toU16());
  }

  int writeToRegister(uint8_t registerAddress, uint16_t val,
                      uint8_t command = Command::Write) {
    wire->beginTransmission(address);
    wire->write(buildFirstByte(registerAddress, command, val));
    wire->write(val & 0xFF);
    return wire->endTransmission();
  }

  int readFromRegister(uint8_t registerAddress, uint16_t* val,
                       uint8_t command = Command::Read) {
    uint8_t bytes[2];
    auto result = wire_asukiaaa::readBytes(
        wire, address, buildFirstByte(registerAddress, command, 0), bytes, 2);
    if (result != 0) {
      return result;
    }
    *val = ((uint16_t)(bytes[0] & 0b1) << 8) | bytes[1];
    return 0;
  }

  static uint8_t buildFirstByte(uint8_t address, uint8_t command,
                                uint16_t data) {
    return (address << 4) | (command << 2) | ((data >> 8) & 0b11);
  }
};

}  // namespace MCP4661_asukiaaa
