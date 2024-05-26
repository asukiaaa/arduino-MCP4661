#include <MCP4661_asukiaaa.hpp>

MCP4661_asukiaaa::Core potentio(&Wire,
                                MCP4661_asukiaaa::DeviceAddress::A0H_A1H_A2H);

void setup() {
  Wire.begin();
  Serial.begin(115200);
}

uint16_t val = 240;

void printLogAboutWrite(Stream* serial, uint8_t channel, uint16_t valWrote,
                        int result) {
  serial->print("write " + String(valWrote) + " for wiper" + String(channel) +
                " ");
  serial->println(result == 0 ? "succeeded"
                              : "failed by error " + String(result));
}

void printLogAboutRead(Stream* serial, uint8_t channel, uint16_t valRead,
                       int result) {
  serial->print("read from wiper" + String(channel) + " ");
  if (result == 0) {
    serial->println("got " + String(valRead));
  } else {
    serial->println("failed by error " + String(result));
  }
}

void loop() {
  int result;
  result = potentio.writeWiperRAM(0, val);
  printLogAboutWrite(&Serial, 0, val, result);
  auto val1 = MCP4661_asukiaaa::MAX_WIPER_VAL - val;
  result = potentio.writeWiperRAM(1, val1);
  printLogAboutWrite(&Serial, 1, val1, result);
  ++val;
  if (val > MCP4661_asukiaaa::MAX_WIPER_VAL) {
    val = 0;
  }
  uint16_t valRead;
  result = potentio.readWiperRAM(0, &valRead);
  printLogAboutRead(&Serial, 0, valRead, result);
  result = potentio.readWiperRAM(1, &valRead);
  printLogAboutRead(&Serial, 1, valRead, result);
  Serial.println("at " + String(millis()));
  delay(1000);
}
