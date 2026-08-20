#include "UI.h"

#include <Arduino.h>

#include "AudioEngine.h"
#include "Logger.h"

#define Serial LogSerial

namespace UI {
  // This headless UI keeps only values controlled by the physical pads.
  static float gVolume = 0.70f;
  static uint32_t gLoopPeriodUs = 0;

  // Clamp normalized control values.
  static inline float clamp01(float v) {
    if (v < 0.0f) return 0.0f;
    if (v > 1.0f) return 1.0f;
    return v;
  }

  // Limit octave shifts to a small playable range.
  static inline int clampOctave(int v) {
    if (v < -2) return -2;
    if (v > 2) return 2;
    return v;
  }

  // Initialize the headless control state.
  void init() {
    Audio::setVolume(gVolume);
    Serial.println("[UI] headless controls ready");
  }

  // Store the main loop period for diagnostics.
  void setLoopPeriodUs(uint32_t periodUs) {
    gLoopPeriodUs = periodUs;
  }

  // Return the last measured main loop period.
  uint32_t getLoopPeriodUs() {
    return gLoopPeriodUs;
  }

  // Accept timing data from the main loop; currently stored nowhere.
  void setLoopTimingUs(uint32_t pollUs, uint32_t uiUs, uint32_t audioUs,
                       uint32_t otherUs, uint32_t totalUs) {
    (void)pollUs;
    (void)uiUs;
    (void)audioUs;
    (void)otherUs;
    (void)totalUs;
  }

  // Apply a relative volume change from the touch controls.
  void nudgeVolume(float delta) {
    gVolume = clamp01(gVolume + delta);
    Audio::setVolume(gVolume);
    Serial.printf("[UI] volume %.2f\n", gVolume);
  }

  // Apply a relative octave change from the touch controls.
  void nudgeOctave(int delta) {
    int octave = clampOctave(Audio::getScaleOctaveOffset() + delta);
    Audio::setScaleOctaveOffset(octave);
    Serial.printf("[UI] octave %+d\n", octave);
  }

  // In headless mode, toast messages are serial log lines.
  void showToast(const char* msg) {
    if (msg && msg[0]) {
      Serial.printf("[UI] %s\n", msg);
    }
  }
}
