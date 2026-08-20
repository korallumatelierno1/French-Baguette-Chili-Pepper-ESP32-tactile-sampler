#pragma once
#include <Arduino.h>

namespace Input {

  // Touch edges are reported as bitmasks over the mapped note pads.
  struct TouchEvents {
    uint32_t rising;   // Set bits are newly touched note pads.
    uint32_t falling;  // Set bits are newly released note pads.
  };

  // Initialize I2C, both MPR121 controllers, and interrupt routing.
  void init();

  // Read the latest touch state and return note-pad edge events.
  TouchEvents pollTouch();

  // Register the current FreeRTOS task as the touch interrupt target.
  void attachCurrentTaskAsWorker();

  // Wait for a touch interrupt or for the timeout used by fallback polling.
  bool waitForTouchSignal(uint32_t timeoutMs);

  // Send note events to the audio engine and the MIDI layer.
  void dispatchTouchEvents(const TouchEvents& ev);

  // True while any physical pad is touched.
  bool  isAnyPadActive();

}
