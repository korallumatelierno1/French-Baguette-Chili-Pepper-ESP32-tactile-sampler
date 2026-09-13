#pragma once

#include <stdint.h>

// All calls belong to the audio task. Input/output are mono, at 44100 Hz.
namespace GranularFx {
void reset();
void configure(float mix, float feedback, uint16_t bpm,
               float pitchRandom = 0.75f, float timeRandom = 0.60f);
// Seed once at startup; reset() preserves the random sequence between activations.
void setRandomSeed(uint32_t seed);
void noteOn();
float process(float input);
}  // namespace GranularFx
