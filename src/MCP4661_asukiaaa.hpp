#pragma once

#include <wire_asukiaaa.h>

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
const uint8_t Write = 0;
const uint8_t Increment = 1;
const uint8_t Decrement = 2;
const uint8_t Read = 3;
}  // namespace Command

const uint16_t MAX_WIPER_VAL = 0x100;

class Core {
 public:
  TwoWire* wire;
  const uint8_t address;
  Core(TwoWire* wire, uint8_t address) : wire(wire), address(address) {}

  int writeWiper(uint8_t channel, uint16_t val) {
    if (channel >= 2) {
      return 10;
    }
    wire->beginTransmission(address);
    wire->write(
        buildFirstByte(Register::Wiper0RAM + channel, Command::Write, val));
    wire->write(val & 0xFF);
    return wire->endTransmission();
  }

  int readWiper(uint8_t channel, uint16_t* val) {
    if (channel >= 2) {
      return 10;
    }
    uint8_t bytes[2];
    auto result = wire_asukiaaa::readBytes(
        wire, address,
        buildFirstByte(Register::Wiper0RAM + channel, Command::Read, 0), bytes,
        2);
    if (result != 0) {
      return result;
    }
    *val = ((uint16_t)(bytes[0] & 0b1) << 8) | bytes[1];
    return 0;
  }

 private:
  static uint8_t buildFirstByte(uint8_t address, uint8_t command,
                                uint16_t data) {
    return (address << 4) | (command << 2) | ((data >> 8) & 0b11);
  }
};

}  // namespace MCP4661_asukiaaa
