#pragma once
#include <Arduino.h>

namespace UsbMidi {

  // Start USB MIDI automatically when USB power is detected at boot.
  void init();

  // Drain incoming packets so the TinyUSB stack stays serviced.
  void service();

  // Mirror pad note events to USB MIDI when the stack is active.
  void noteOn(uint8_t noteIndex);
  void noteOff(uint8_t noteIndex);

  // Send all-notes-off and clear the local note cache.
  void allNotesOff();

} // namespace UsbMidi
