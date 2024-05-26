#include <MCP4661_asukiaaa.hpp>

MCP4661_asukiaaa::Core potentio(&Wire,
                                MCP4661_asukiaaa::DeviceAddress::A0H_A1H_A2H);

void setup() {
  Wire.begin();
  Serial.begin(115200);
}

void printVal(Stream* serial, String target, uint8_t channel, int result,
              uint16_t val) {
  serial->print("read " + target + " of wiper" + String(channel) + " ");
  if (result != 0) {
    serial->println("failed error: " + String(result));
  } else {
    serial->println("got " + String(val));
  }
}

void loop() {
  uint16_t val;
  int result;
  for (uint8_t targetChannel = 0; targetChannel < 2; ++targetChannel) {
    result = potentio.readWiperEEPROM(targetChannel, &val);
    printVal(&Serial, "EEPROM", targetChannel, result, val);
    result = potentio.readWiperRAM(targetChannel, &val);
    printVal(&Serial, "RAM", targetChannel, result, val);
  }

  MCP4661_asukiaaa::TCON tcon;
  potentio.readTCON(&tcon);
  Serial.print("read TCON ");
  if (tcon.readResult == 0) {
    tcon.println(&Serial);
  } else {
    Serial.println("failed error: " + String(tcon.readResult));
  }

  MCP4661_asukiaaa::Status statusPotentio;
  potentio.readStatus(&statusPotentio);
  Serial.print("read status ");
  if (statusPotentio.readResult == 0) {
    statusPotentio.println(&Serial);
  } else {
    Serial.println("failed error: " + String(statusPotentio.readResult));
  }

  Serial.println("at " + String(millis()));
  delay(1000);
}
