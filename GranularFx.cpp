#include "GranularFx.h"

#include <cmath>

namespace GranularFx {
namespace {

// One 64 KiB recording at half the output sample rate; three readers maximum.
// Allocation, buffer clearing and transcendental functions never enter process().
constexpr uint32_t WET_SAMPLE_RATE = 22050u;
constexpr uint32_t RING_FRAMES = 32768u;
constexpr uint32_t RING_MASK = RING_FRAMES - 1u;
constexpr uint32_t READ_GUARD = 8u;
constexpr uint32_t START_HISTORY = WET_SAMPLE_RATE / 10u;
constexpr uint32_t IDLE_RESET_FRAMES = WET_SAMPLE_RATE / 2u;
constexpr uint32_t PAD_HOP_MAX = WET_SAMPLE_RATE / 2u;
constexpr uint32_t SLICE_MAX = WET_SAMPLE_RATE * 3u / 10u;
constexpr uint32_t MIN_GRAIN_FRAMES = 128u;
constexpr float PCM_SCALE = 32767.0f;
constexpr float PCM_RECIPROCAL = 1.0f / PCM_SCALE;
constexpr float ACTIVITY_THRESHOLD = 0.00015f;
constexpr uint8_t GRAIN_COUNT = 3u;

struct Grain {
  bool active;
  uint32_t readSerial;
  uint32_t elapsed;
  uint32_t duration;
  uint32_t attackFrames;
  uint32_t releaseFrames;
  float attackReciprocal;
  float releaseReciprocal;
  float gain;
  int16_t rateQ8;
  uint8_t fraction;
};

struct Slice {
  uint32_t firstSerial;
  uint32_t frames;
  uint8_t repeatsLeft;
};

static int16_t gRecording[RING_FRAMES];
static Grain gGrains[GRAIN_COUNT];
static Slice gSlice;
// Unsigned serial differences stay correct through the ~54 hour counter wrap.
// Fractional read positions are local Q8 values, never growing float counters.
static uint32_t gWriteSerial = 0u;
static uint32_t gValidFrames = 0u;
static uint32_t gRhythmCountdown = START_HISTORY;
static uint32_t gPadCountdown = START_HISTORY;
static uint32_t gIdleFrames = IDLE_RESET_FRAMES;
static uint32_t gStepFrames = WET_SAMPLE_RATE / 4u;
static uint32_t gPadHop = PAD_HOP_MAX;
static uint8_t gRhythmStep = 0u;
static uint8_t gPadStep = 0u;
static uint8_t gDownsamplePhase = 0u;
static float gMix = 0.45f;
static float gFeedback = 0.30f;
static float gPitchRandom = 0.75f;
static float gTimeRandom = 0.60f;
static uint32_t gRandomState = 0x6D2B79F5u;
static float gInputLowpassA = 0.0f;
static float gInputLowpassB = 0.0f;
static float gPreviousInput = 0.0f;
static float gInputHighpass = 0.0f;
static float gRecordedEnvelope = 0.0f;
static float gFeedbackLowpass = 0.0f;
static float gWetInterpolated = 0.0f;
static float gWetInterpolationStep = 0.0f;
static float gOutputLowpass = 0.0f;

static float clamp(float value, float low, float high) {
  return value < low ? low : (value > high ? high : value);
}

static uint32_t minimum(uint32_t a, uint32_t b) {
  return a < b ? a : b;
}

// Small PRNG, used only when scheduling grains, never for every audio sample.
static uint32_t randomWord() {
  gRandomState ^= gRandomState << 13;
  gRandomState ^= gRandomState >> 17;
  gRandomState ^= gRandomState << 5;
  return gRandomState;
}

static float randomUnit() {
  return static_cast<float>(randomWord() >> 8) * (1.0f / 16777216.0f);
}

static int16_t chooseRate(int16_t patternedRate, bool pad) {
  if (gPitchRandom <= 0.0f || randomUnit() >= gPitchRandom) return patternedRate;
  // Harmonically related speeds; direction changes happen only at grain starts.
  static const int16_t RHYTHM_RATES[8] = {128, 192, 256, 384, 512, -128, -256, -384};
  static const int16_t PAD_RATES[8] = {128, 128, 192, 256, 384, -128, -192, -256};
  const uint8_t choice = static_cast<uint8_t>(randomWord() & 7u);
  return pad ? PAD_RATES[choice] : RHYTHM_RATES[choice];
}

static uint32_t varyInterval(uint32_t base) {
  if (gTimeRandom <= 0.0f) return base;
  // Uneven subdivisions and occasional longer gaps, plus small timing drift.
  static const float RATIOS[8] = {0.50f, 0.75f, 0.75f, 1.0f, 1.0f, 1.25f, 1.5f, 2.0f};
  const float ratio = RATIOS[randomWord() & 7u] + (randomUnit() - 0.5f) * 0.20f;
  const float varied = static_cast<float>(base) * (1.0f + (ratio - 1.0f) * gTimeRandom);
  return static_cast<uint32_t>(clamp(varied, static_cast<float>(base) * 0.5f,
                                    static_cast<float>(base) * 2.0f));
}

static float smoothWindow(float phase) {
  return phase * phase * (3.0f - 2.0f * phase);
}

// A future read must remain inside initialized history AND ahead of overwrites
// for the grain's entire lifetime, including interpolation's second sample.
static bool canRead(uint32_t serial, uint32_t duration, int16_t rateQ8) {
  if (duration < MIN_GRAIN_FRAMES || gValidFrames < READ_GUARD + 2u) return false;
  const uint32_t age = gWriteSerial - serial;
  if (age < 2u || age > gValidFrames || age >= RING_FRAMES - 1u) return false;
  const int32_t travelQ8 = static_cast<int32_t>(duration - 1u) * rateQ8;
  int32_t travel = travelQ8 / 256;
  if (travelQ8 < 0 && travelQ8 % 256 != 0) --travel;
  const int32_t finalAge = static_cast<int32_t>(age + duration - 1u) - travel;
  const int32_t finalInitialAge = static_cast<int32_t>(age) - travel;
  return finalAge >= 2 && finalAge < static_cast<int32_t>(RING_FRAMES - 1u)
      && finalInitialAge <= static_cast<int32_t>(gValidFrames);
}

static bool startGrain(uint8_t slot, uint32_t serial, uint32_t duration,
                       int16_t rateQ8, bool pad) {
  Grain& grain = gGrains[slot];
  if (grain.active || !canRead(serial, duration, rateQ8)) return false;
  grain.active = true;
  grain.readSerial = serial;
  grain.elapsed = 0u;
  grain.duration = duration;
  grain.rateQ8 = rateQ8;
  grain.fraction = 0u;
  grain.gain = pad ? 0.25f : 0.50f;
  // Rounded windows reach zero at both ends. The pad windows extend across
  // almost the entire grain; two pads overlap without replacing active readers.
  grain.attackFrames = pad ? duration / 2u : minimum(220u, duration / 5u);
  grain.releaseFrames = pad ? duration / 2u : minimum(662u, duration / 3u);
  grain.attackReciprocal = 1.0f / static_cast<float>(grain.attackFrames);
  grain.releaseReciprocal = 1.0f / static_cast<float>(grain.releaseFrames);
  return true;
}

static bool captureSlice() {
  if (gValidFrames <= START_HISTORY || gRecordedEnvelope < ACTIVITY_THRESHOLD) return false;
  const uint32_t available = gValidFrames - READ_GUARD;
  uint32_t length = minimum(gStepFrames * 3u / 4u, SLICE_MAX);
  length = minimum(length, available);
  const uint32_t delayRoom = minimum(available - length, WET_SAMPLE_RATE / 2u);
  const uint32_t delay = gTimeRandom > 0.0f
      ? static_cast<uint32_t>(randomUnit() * gTimeRandom * delayRoom) : 0u;
  gSlice.firstSerial = gWriteSerial - READ_GUARD - length - delay;
  gSlice.frames = length;
  // Repeat a captured fragment; recapture early if history would be overwritten.
  gSlice.repeatsLeft = 3u;
  if (gTimeRandom > 0.0f && randomUnit() < gTimeRandom) {
    gSlice.repeatsLeft = static_cast<uint8_t>(1u + randomWord() % 5u);
  }
  return true;
}

static bool startSlice(int16_t rateQ8, uint32_t interval) {
  if (gSlice.frames < MIN_GRAIN_FRAMES) return false;
  const uint32_t rateMagnitude = static_cast<uint32_t>(rateQ8 < 0 ? -rateQ8 : rateQ8);
  const uint32_t availableDuration = (gSlice.frames - 2u) * 256u / rateMagnitude;
  const uint32_t duration = minimum(interval * 4u / 5u, availableDuration);
  const uint32_t serial = rateQ8 < 0
      ? gSlice.firstSerial + gSlice.frames - 2u : gSlice.firstSerial;
  return startGrain(0u, serial, duration, rateQ8, false);
}

static void triggerRhythm(uint32_t interval) {
  static const int16_t RATES_Q8[8] = {256, 256, 384, -256, 128, 512, 256, -128};
  const int16_t rate = chooseRate(RATES_Q8[gRhythmStep], false);
  gRhythmStep = (gRhythmStep + 1u) & 7u;
  if (gSlice.repeatsLeft == 0u && !captureSlice()) return;
  if (startSlice(rate, interval)) {
    --gSlice.repeatsLeft;
  } else {
    // A reader is never allowed to reach an overwritten or unrecorded sample.
    gSlice.repeatsLeft = 0u;
    if (captureSlice() && startSlice(rate, interval)) --gSlice.repeatsLeft;
  }
}

static void triggerPad() {
  if (gValidFrames <= START_HISTORY || gRecordedEnvelope < ACTIVITY_THRESHOLD) return;
  const uint8_t slot = !gGrains[1].active ? 1u : (!gGrains[2].active ? 2u : 0u);
  if (slot == 0u) return;
  static const int16_t RATES_Q8[4] = {128, -128, 256, 128};
  const int16_t rate = chooseRate(RATES_Q8[gPadStep], true);
  gPadStep = (gPadStep + 1u) & 3u;
  const uint32_t magnitude = static_cast<uint32_t>(rate < 0 ? -rate : rate);
  const uint32_t available = gValidFrames - READ_GUARD - 2u;
  const uint32_t duration = minimum(gPadHop * 9u / 5u, available * 256u / magnitude);
  const uint32_t sourceFrames = (duration * magnitude + 255u) / 256u;
  uint32_t serial = rate < 0 ? gWriteSerial - READ_GUARD - 2u
      : gWriteSerial - READ_GUARD - sourceFrames - 1u;
  if (gTimeRandom > 0.0f) {
    const uint32_t delay = static_cast<uint32_t>(randomUnit() * gTimeRandom * (WET_SAMPLE_RATE / 2u));
    // One bounded attempt; keep the recent source if the delayed grain is unsafe.
    if (canRead(serial - delay, duration, rate)) serial -= delay;
  }
  startGrain(slot, serial, duration, rate, true);
}

static float renderGrain(Grain& grain) {
  if (!grain.active) return 0.0f;
  const uint32_t age = gWriteSerial - grain.readSerial;
  // The spawn check proves this for every scheduled sample. Keep the guard
  // here as well so even counter wrap or later parameter changes cannot read
  // stale data. No buffer-wide work is needed on reset.
  if (age < 2u || age > gValidFrames || age >= RING_FRAMES - 1u) {
    grain.active = false;
    return 0.0f;
  }
  const uint32_t index = grain.readSerial & RING_MASK;
  const float a = static_cast<float>(gRecording[index]);
  const float b = static_cast<float>(gRecording[(index + 1u) & RING_MASK]);
  const float sample = (a + (b - a) * (static_cast<float>(grain.fraction) / 256.0f))
      * PCM_RECIPROCAL;
  const uint32_t remaining = grain.duration - 1u - grain.elapsed;
  float window = 1.0f;
  if (grain.elapsed < grain.attackFrames) {
    window = smoothWindow(static_cast<float>(grain.elapsed) * grain.attackReciprocal);
  } else if (remaining < grain.releaseFrames) {
    window = smoothWindow(static_cast<float>(remaining) * grain.releaseReciprocal);
  }
  // The positive offset gives floor division for reverse rates as well, using
  // shifts and masks instead of floating-point floor/fmod in the sample loop.
  const int32_t phase = static_cast<int32_t>(grain.fraction) + grain.rateQ8 + 1024;
  grain.readSerial += static_cast<uint32_t>((phase >> 8) - 4);
  grain.fraction = static_cast<uint8_t>(phase & 255);
  if (++grain.elapsed >= grain.duration) grain.active = false;
  return sample * window * grain.gain;
}

static float processWet(float input) {
  // Remove DC before recording. The filtered feedback has a hard gain ceiling
  // and is sourced from the normalized sum, so repeated fragments decay.
  gInputHighpass = input - gPreviousInput + 0.995f * gInputHighpass;
  gPreviousInput = input;
  const float recorded = clamp(gInputHighpass + gFeedback * gFeedbackLowpass, -0.98f, 0.98f);
  gRecording[gWriteSerial & RING_MASK] = static_cast<int16_t>(recorded * PCM_SCALE);
  ++gWriteSerial;
  if (gValidFrames < RING_FRAMES) ++gValidFrames;
  const float magnitude = std::fabs(recorded);
  gRecordedEnvelope *= 0.9998f;
  if (magnitude > gRecordedEnvelope) gRecordedEnvelope = magnitude;
  if (std::fabs(input) > ACTIVITY_THRESHOLD) gIdleFrames = 0u;
  else if (gIdleFrames < IDLE_RESET_FRAMES) ++gIdleFrames;

  if (gRhythmCountdown == 0u) {
    const uint32_t interval = varyInterval(gStepFrames);
    triggerRhythm(interval);
    gRhythmCountdown = interval;
  }
  --gRhythmCountdown;
  if (gPadCountdown == 0u) {
    triggerPad();
    gPadCountdown = varyInterval(gPadHop);
  }
  --gPadCountdown;

  float wet = 0.0f;
  for (uint8_t i = 0u; i < GRAIN_COUNT; ++i) wet += renderGrain(gGrains[i]);
  gFeedbackLowpass += 0.30f * (wet - gFeedbackLowpass);
  return wet;
}

}  // namespace

void configure(float mix, float feedback, uint16_t bpm, float pitchRandom, float timeRandom) {
  gMix = std::isfinite(mix) ? clamp(mix, 0.0f, 1.0f) : 0.0f;
  gFeedback = std::isfinite(feedback) ? clamp(feedback, 0.0f, 0.65f) : 0.0f;
  gPitchRandom = std::isfinite(pitchRandom) ? clamp(pitchRandom, 0.0f, 1.0f) : 0.0f;
  gTimeRandom = std::isfinite(timeRandom) ? clamp(timeRandom, 0.0f, 1.0f) : 0.0f;
  if (bpm < 40u) bpm = 40u;
  if (bpm > 240u) bpm = 240u;
  gStepFrames = WET_SAMPLE_RATE * 30u / bpm;  // Eighth-note rhythm.
  gPadHop = minimum(gStepFrames * 2u, PAD_HOP_MAX);
}

void setRandomSeed(uint32_t seed) {
  gRandomState = seed ? seed : 0x6D2B79F5u;
}

void reset() {
  for (uint8_t i = 0u; i < GRAIN_COUNT; ++i) gGrains[i] = Grain{};
  gSlice = Slice{};
  gWriteSerial = 0u;
  gValidFrames = 0u;
  gRhythmCountdown = START_HISTORY;
  gPadCountdown = START_HISTORY;
  gIdleFrames = IDLE_RESET_FRAMES;
  gRhythmStep = 0u;
  gPadStep = 0u;
  gDownsamplePhase = 0u;
  gInputLowpassA = 0.0f;
  gInputLowpassB = 0.0f;
  gPreviousInput = 0.0f;
  gInputHighpass = 0.0f;
  gRecordedEnvelope = 0.0f;
  gFeedbackLowpass = 0.0f;
  gWetInterpolated = 0.0f;
  gWetInterpolationStep = 0.0f;
  gOutputLowpass = 0.0f;
}

void noteOn() {
  // Chords and rapid note flurries leave the ongoing rhythm and pad tails alone.
  if (gIdleFrames < IDLE_RESET_FRAMES) return;
  gRhythmCountdown = varyInterval(START_HISTORY);
  gPadCountdown = varyInterval(START_HISTORY);
  gRhythmStep = 0u;
  gSlice.repeatsLeft = 0u;
  gIdleFrames = 0u;
}

float process(float input) {
  if (!std::isfinite(input)) input = 0.0f;
  // Two inexpensive poles before 2:1 decimation give the fragments a warm tone
  // while reducing high-frequency aliasing from the reduced processing rate.
  const float boundedInput = clamp(input, -1.0f, 1.0f);
  gInputLowpassA += 0.45f * (boundedInput - gInputLowpassA);
  gInputLowpassB += 0.45f * (gInputLowpassA - gInputLowpassB);
  gDownsamplePhase ^= 1u;
  if (gDownsamplePhase == 0u) {
    const float wet = processWet(gInputLowpassB);
    gWetInterpolationStep = 0.5f * (wet - gWetInterpolated);
  }
  gWetInterpolated += gWetInterpolationStep;
  gOutputLowpass += 0.50f * (gWetInterpolated - gOutputLowpass);
  // Reader gains sum to 1.0 at most; an ordinary crossfade and slight wet trim
  // avoid the volume jump of adding three grains on top of the full dry signal.
  return input * (1.0f - gMix) + gOutputLowpass * (0.95f * gMix);
}

}  // namespace GranularFx
