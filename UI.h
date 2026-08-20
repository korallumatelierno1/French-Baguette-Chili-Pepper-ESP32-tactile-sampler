#pragma once

#include <Arduino.h>

namespace UI {

  // Headless UI service used by the physical controls.
  void init();

  // Loop timing is stored for diagnostics even when no display is present.
  void setLoopPeriodUs(uint32_t periodUs);
  uint32_t getLoopPeriodUs();
  void setLoopTimingUs(uint32_t pollUs, uint32_t uiUs, uint32_t audioUs,
                       uint32_t otherUs, uint32_t totalUs);

  // Control helpers called by the touch input layer.
  void nudgeVolume(float delta);
  void nudgeOctave(int delta);

  // Print a short status line to the serial log.
  void showToast(const char* msg);

} // namespace UI
