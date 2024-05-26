#include <MCP4661_asukiaaa.hpp>

MCP4661_asukiaaa::Core potentio(&Wire,
                                MCP4661_asukiaaa::DeviceAddress::A0H_A1H_A2H);

int resultWiper[2];
uint16_t valForWiper[] = {90, 210};

void setup() {
  Wire.begin();
  Serial.begin(115200);
  MCP4661_asukiaaa::Status status;
  potentio.readStatus(&status);
  if (status.readResult == 0) {
    if (status.WP) {
      status.WP = false;
      potentio.writeStatus(status);
    }
  }
  resultWiper[0] = potentio.writeWiperEEPROM(0, valForWiper[0]);
  delay(5);
  resultWiper[1] = potentio.writeWiperEEPROM(1, valForWiper[1]);
  delay(5);
}

void printVal(Stream* serial, String strAction, uint8_t channel, int result,
              uint16_t val) {
  serial->print(strAction + " EEPROM of wiper" + String(channel) + " ");
  if (result != 0) {
    serial->println("but failed error: " + String(result));
  } else {
    serial->println("as " + String(val));
  }
}

void loop() {
  for (uint8_t targetChannel = 0; targetChannel < 2; ++targetChannel) {
    Serial.print("in setup function ");
    printVal(&Serial, "wrote", targetChannel, resultWiper[targetChannel],
             valForWiper[targetChannel]);
    uint16_t val;
    auto result = potentio.readWiperEEPROM(targetChannel, &val);
    printVal(&Serial, "read", targetChannel, result, val);
  }
  Serial.println("at " + String(millis()));
  delay(1000);
}
