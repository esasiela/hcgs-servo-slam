#ifndef __HCGS_MIDI_OUT__
#define __HCGS_MIDI_OUT__

#include "Arduino.h"
#include <MIDI.h>


class HcgsMidiOut {

  public:
    HcgsMidiOut(Stream *midiStream);
    void sendNoteOn(unsigned note, unsigned velocity, unsigned channel);
    void sendNoteOff();
    void sendSysEx(uint8_t deviceId, uint8_t modelId, uint8_t instructionId, const byte* rawData, uint8_t rawLen);

  private:
    Stream *_midiStream;
  
};



#endif
