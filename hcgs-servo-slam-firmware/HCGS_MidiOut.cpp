#include "Arduino.h"
#include "HCGS_MidiOut.h"


HcgsMidiOut::HcgsMidiOut(Stream *midiStream) {
  _midiStream = midiStream;
}

void HcgsMidiOut::sendNoteOn(unsigned note, unsigned velocity, unsigned channel) {
}

void HcgsMidiOut::sendNoteOff() {
}

// rawLen maximum value is 21, because encoding buffer size is 24 (encoding inflates data by 1 byte for every 7 input)
void HcgsMidiOut::sendSysEx(uint8_t deviceId, uint8_t modelId, uint8_t instructionId, const byte* rawData, uint8_t rawLen) {

  unsigned long acc;

  byte encodedData[24];
  unsigned encodedLen = midi::encodeSysEx(rawData, encodedData, rawLen);

  // sysex begin
  _midiStream->write(0xF0);

  // Grumpenspiel manufacturer ID
  _midiStream->write(0x1E);
  acc += 0x1E;

  // device ID
  _midiStream->write(deviceId);
  acc += deviceId;

  // model ID
  _midiStream->write(modelId);
  acc += modelId;

  // instruction ID
  _midiStream->write(instructionId);
  acc += instructionId;

  // encoded data
  for (uint8_t i = 0; i < encodedLen; i++) {
    _midiStream->write(encodedData[i]);
    acc += encodedData[i];
  }

  // checksum
  uint8_t checksum = (uint8_t)(128 - (acc % 128));
  _midiStream->write(checksum);

  // sysex end
  _midiStream->write(0xF7);
}
