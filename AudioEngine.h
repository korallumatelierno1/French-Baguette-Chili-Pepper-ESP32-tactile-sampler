#pragma once

#include <Arduino.h>

namespace Audio {

  // Four available internal synth engines.
  enum OscillatorEngine : uint8_t {
    OSC_ENGINE_WORMY = 0,
    OSC_ENGINE_LIGHT,
    OSC_ENGINE_AURORA_LIGHT,
    OSC_ENGINE_ORGAN,
    OSC_ENGINE_COUNT
  };

  // Space effects add ambience after the dry engine signal.
  enum SpaceFxMode : uint8_t {
    SPACE_FX_OFF = 0,
    SPACE_FX_WARM_REVERB,
    SPACE_FX_COUNT
  };

  // Initialize audio state and the I2S output.
  void init();

  // Render one audio block and write it to I2S. Called by the audio task.
  void update();

  // Low-priority service hook kept for future non-audio work.
  void service();

  // Voice triggers come from touch input and are mirrored to USB MIDI.
  void noteOn(uint8_t noteIndex);
  void noteOff(uint8_t noteIndex);
  uint8_t midiNoteFromNoteIndex(uint8_t noteIndex);

  // Toggle between the internal synth and RAM-loaded SD samples.
  bool initSamplerStorage();
  bool toggleSamplerMode();
  bool isSamplerMode();

  // Master output level, clamped to 0..1.
  void setVolume(float vol);

  // Select the current engine for new notes.
  void setOscillatorEngine(OscillatorEngine engine);
  OscillatorEngine getOscillatorEngine();
  bool computeAllowsOscillatorEngine(OscillatorEngine engine);

  // Space effect controls.
  void setSpaceFxMode(SpaceFxMode mode);
  SpaceFxMode getSpaceFxMode();
  void setReverbAmount(float amount);
  float getReverbAmount();

  // Disable every effect in one call.
  void clearAllEffects();

  // Runtime octave transpose added on top of PANORYTHE_SCALE.
  void setScaleOctaveOffset(int offset);
  int getScaleOctaveOffset();

} // namespace Audio
