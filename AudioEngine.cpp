#include "AudioEngine.h"

#include <Arduino.h>
#include <FS.h>
#include <SD.h>
#include <SPI.h>
#include <atomic>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "Config.h"
#include "GranularFx.h"
#include "Logger.h"
#include "driver/i2s.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#define Serial LogSerial

namespace Audio {
namespace {

static SPIClass gSampleSpi(FSPI);
static bool gSampleSpiReady = false;

// Audio is rendered in fixed-size blocks to keep the I2S write cadence stable.
constexpr uint32_t SAMPLE_RATE = 44100;
constexpr size_t CHUNK_SAMPLES = 128;
constexpr uint16_t I2S_DMA_BUF_COUNT = 2;
constexpr uint16_t I2S_DMA_BUF_LEN = 256;
constexpr uint8_t MAX_VOICES = SYNTH_POLYPHONY;
static_assert(MAX_VOICES >= 1 && MAX_VOICES <= 24, "SYNTH_POLYPHONY must be 1..24");
static_assert(SYNTH_VOICE_STEAL_FADE_MS > 0, "Voice steal fade must be positive");
constexpr uint32_t STEAL_SAMPLES = SAMPLE_RATE * SYNTH_VOICE_STEAL_FADE_MS / 1000u;
constexpr uint8_t MAX_SAMPLE_VOICES = 10;
constexpr uint8_t SAMPLE_PAD_COUNT = 10;
constexpr uint8_t MAX_NOTE_INDEX = 24;
constexpr uint8_t AUDIO_EVENT_QUEUE_LEN = 48;
constexpr size_t SINE_TABLE_SIZE = 1024;
constexpr uint32_t REVERB_A_SIZE = 2048u;
constexpr uint32_t REVERB_B_SIZE = 4096u;
constexpr uint32_t REVERB_A_MASK = REVERB_A_SIZE - 1u;
constexpr uint32_t REVERB_B_MASK = REVERB_B_SIZE - 1u;
constexpr uint32_t ECHOLOOP_BUFFER_SIZE = 32768u;
constexpr uint32_t ECHOLOOP_BUFFER_MASK = ECHOLOOP_BUFFER_SIZE - 1u;
constexpr uint8_t ECHOLOOP_VOICE_COUNT = 2;
constexpr uint8_t ECHOLOOP_NOTE_HISTORY = 16;
constexpr float OUTPUT_PCM_SCALE = 22000.0f;
constexpr float PI_F = 3.14159265358979323846f;
constexpr float TWO_PI_F = 6.28318530717958647692f;
constexpr uint16_t WAV_FORMAT_PCM = 0x0001;
constexpr uint16_t WAV_FORMAT_IEEE_FLOAT = 0x0003;
constexpr uint16_t WAV_FORMAT_EXTENSIBLE = 0xFFFE;
constexpr const char* SAMPLE_ROOT_DIR = "/sample";

static const char* SAMPLE_FILE_BASES[6] = {
  "pigeon_pad",
  "Eiffel_pad",
  "cloud_pad",
  "flower_pad",
  "bottle_pad",
  "cheese_pad"
};

static const float FLUTE_AIR_FORMANTS[3] = {820.0f, 1680.0f, 3120.0f};
static const float FLUTE_AIR_GAINS[3] = {0.050f, 0.032f, 0.018f};
static const float ACCORDION_BODY_FORMANTS[3] = {290.0f, 760.0f, 1950.0f};
static const float ACCORDION_BODY_GAINS[3] = {0.24f, 0.15f, 0.08f};

enum EnvState : uint8_t {
  ENV_IDLE = 0,
  ENV_ATTACK,
  ENV_SUSTAIN,
  ENV_RELEASE
};

enum AudioEventType : uint8_t {
  AUDIO_EVENT_NOTE_ON = 1,
  AUDIO_EVENT_NOTE_OFF,
  AUDIO_EVENT_SET_SAMPLER_MODE
};

struct AudioEvent {
  AudioEventType type;
  uint8_t noteIndex;
  bool samplerMode;
};

// Minimal state for one state-variable formant filter.
struct FormantState {
  float low;
  float band;
};

// One active musical voice. The same struct carries state for all four engines.
struct Voice {
  bool active;
  uint8_t noteIndex;
  OscillatorEngine engine;
  EnvState envState;
  float env;
  float attackStep;
  float releaseStep;
  float freq;
  float phaseA;
  float phaseB;
  float phaseC;
  float phaseD;
  float lp;
  float pluckEnv;
  float noiseEnv;
  uint32_t rng;
  uint32_t startedAt;
  float formantCoeff[3];
  float formantDamping[3];
  FormantState formant[3];
};

struct SampleSlot {
  int16_t* data;
  uint32_t frames;
  uint32_t capacityFrames;
  uint32_t sampleRate;
  bool loaded;
};

struct SampleVoice {
  bool active;
  uint8_t noteIndex;
  uint8_t slotIndex;
  float position;
  float increment;
  uint32_t startedAt;
};

struct EchoLoopVoice {
  bool active;
  uint16_t readIndex;
  uint16_t remaining;
  uint16_t length;
  uint16_t fadeSamples;
  float fadeScale;
  float level;
};

// Global audio state. New notes use gEngine; existing voices keep their engine.
static Voice gVoices[MAX_VOICES];
static Voice gPendingVoices[MAX_VOICES];
static uint32_t gStealRemaining[MAX_VOICES];
static SampleVoice gSampleVoices[MAX_SAMPLE_VOICES];
static SampleSlot gSampleSlots[SAMPLE_PAD_COUNT];
static uint32_t gVoiceSeq = 1;
static float gVolume = 0.70f;
static OscillatorEngine gEngine = OSC_ENGINE_OMNICHORD;
static volatile bool gSamplerModeRequested = false;
static bool gSamplerMode = false;
static bool gSampleSdReady = false;
static bool gSampleStorageReady = false;
static uint8_t gLoadedSampleCount = 0;
static uint32_t gSampleBytesPerPadLimit = 0;
static uint32_t gSampleStorageRetryMs = 0;
static bool gScaleMinor = true;
static int gScaleBaseMidiNote = 48;
static int gScaleOctaveOffset = 0;
struct ScaleRoot { const char* name; uint8_t semitone; };
// One spelling per pitch class so every combo changes the sounding key.
// PANORYTHE_SCALE still accepts enharmonic spellings such as Db or Eb.
static const ScaleRoot SCALE_ROOTS[] = {
  {"C", 0}, {"C#", 1}, {"D", 2}, {"D#", 3}, {"E", 4}, {"F", 5},
  {"F#", 6}, {"G", 7}, {"G#", 8}, {"A", 9}, {"A#", 10}, {"B", 11}
};
static constexpr int SCALE_ROOT_COUNT = sizeof(SCALE_ROOTS) / sizeof(SCALE_ROOTS[0]);
static int gScaleRootIndex = 0;

static SpaceFxMode gSpaceFxMode = SPACE_FX_OFF;
// Input controls publish requests; only the audio task changes effect buffers.
static std::atomic<SpaceFxMode> gSpaceFxModeRequested{SPACE_FX_OFF};
static SpaceFxMode gNextSpaceFxMode = SPACE_FX_OFF;
static float gSpaceFxBlend = 0.0f;
constexpr float SPACE_FX_BLEND_STEP = 1.0f / (0.020f * SAMPLE_RATE);
static float gReverbAmount = 0.0f;
static std::atomic<float> gReverbAmountRequested{0.0f};
static bool gI2sReady = false;
static std::atomic<uint32_t> gRenderPeakUs{0};
static int16_t gOut[CHUNK_SAMPLES * 2];

static float gReverbA[REVERB_A_SIZE];
static float gReverbB[REVERB_B_SIZE];
static uint32_t gReverbIndexA = 0;
static uint32_t gReverbIndexB = 0;
static int16_t gEchoLoopBuffer[ECHOLOOP_BUFFER_SIZE];
static EchoLoopVoice gEchoLoopVoices[ECHOLOOP_VOICE_COUNT];
static uint16_t gEchoLoopWriteIndex = 0;
static uint16_t gEchoLoopCountdown = 0;
static uint32_t gEchoLoopSampleCounter = 0;
static uint32_t gEchoLoopNoteStarts[ECHOLOOP_NOTE_HISTORY];
static uint8_t gEchoLoopNoteWrite = 0;
static uint8_t gEchoLoopNoteCount = 0;
static uint32_t gEchoLoopRng = 0xC0DEC0DEu;
static StaticQueue_t gAudioEventQueueState;
static uint8_t gAudioEventQueueStorage[AUDIO_EVENT_QUEUE_LEN * sizeof(AudioEvent)];
static QueueHandle_t gAudioEventQueue = nullptr;
static bool gAudioEventOverflowLogged = false;
static float gSineTable[SINE_TABLE_SIZE];
static bool gSineTableReady = false;

// Keep normalized control values inside the expected 0..1 range.
static inline float clamp01(float v) {
  if (v < 0.0f) return 0.0f;
  if (v > 1.0f) return 1.0f;
  return v;
}

// Generic clamp for oscillator and filter parameters.
static inline float clampf(float v, float lo, float hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

// Oscillator phases are stored as normalized 0..1 values.
static inline float wrap01(float p) {
  while (p >= 1.0f) p -= 1.0f;
  while (p < 0.0f) p += 1.0f;
  return p;
}

// Fast saturator used instead of tanhf in the real-time audio path.
static inline float fastTanh(float x) {
  x = clampf(x, -3.0f, 3.0f);
  float x2 = x * x;
  return x * (27.0f + x2) / (27.0f + 9.0f * x2);
}

// Replace invalid DSP values with silence before they can poison delay buffers.
static inline float finiteOrZero(float x) {
  return isfinite(x) ? x : 0.0f;
}

// Build the sine table once at startup.
static void buildSineTable() {
  for (size_t i = 0; i < SINE_TABLE_SIZE; ++i) {
    gSineTable[i] = sinf(TWO_PI_F * ((float)i / (float)SINE_TABLE_SIZE));
  }
  gSineTableReady = true;
}

// Sine lookup helper; interpolation keeps oscillators smooth without per-sample sinf.
static inline float sine01(float phase) {
  if (!gSineTableReady) return sinf(TWO_PI_F * phase);
  if (phase >= 1.0f) phase -= (int)phase;
  if (phase < 0.0f) phase += 1.0f;

  float pos = phase * (float)SINE_TABLE_SIZE;
  int idx0 = (int)pos;
  if (idx0 >= (int)SINE_TABLE_SIZE) idx0 = 0;
  int idx1 = idx0 + 1;
  if (idx1 >= (int)SINE_TABLE_SIZE) idx1 = 0;
  float frac = pos - (float)idx0;
  return gSineTable[idx0] + (gSineTable[idx1] - gSineTable[idx0]) * frac;
}

// Cheap triangle wave used to add reed/string edge.
static inline float triangle01(float phase) {
  return (phase < 0.5f) ? (phase * 4.0f - 1.0f) : (3.0f - phase * 4.0f);
}

// Small deterministic noise generator used by breath and strike transients.
static uint32_t xorshift(uint32_t& state) {
  uint32_t x = state ? state : 0x9E3779B9u;
  x ^= x << 13;
  x ^= x >> 17;
  x ^= x << 5;
  state = x ? x : 0x9E3779B9u;
  return state;
}

// Convert xorshift output to a bipolar floating-point noise signal.
static inline float whiteNoise(uint32_t& state) {
  return ((int32_t)xorshift(state)) / 2147483648.0f;
}

static uint32_t randomRange(uint32_t& state, uint32_t maxExclusive) {
  if (maxExclusive == 0) return 0;
  return xorshift(state) % maxExclusive;
}

// Standard equal-tempered MIDI note conversion.
static float midiToFrequency(uint8_t note) {
  return 440.0f * powf(2.0f, ((float)note - 69.0f) / 12.0f);
}

// Convert an ASCII letter to lowercase without pulling in locale behavior.
static char lowerAscii(char c) {
  if (c >= 'A' && c <= 'Z') return (char)(c - 'A' + 'a');
  return c;
}

// Return true when s begins with prefix, ignoring ASCII case.
static bool startsWithNoCase(const char* s, const char* prefix) {
  if (!s || !prefix) return false;
  while (*prefix) {
    if (lowerAscii(*s) != lowerAscii(*prefix)) return false;
    ++s;
    ++prefix;
  }
  return true;
}

static bool endsWithNoCase(const String& s, const char* suffix) {
  if (!suffix) return false;
  int len = s.length();
  int suffixLen = (int)strlen(suffix);
  if (len < suffixLen) return false;
  for (int i = 0; i < suffixLen; ++i) {
    if (lowerAscii(s[len - suffixLen + i]) != lowerAscii(suffix[i])) {
      return false;
    }
  }
  return true;
}

static void beginSampleSpiBus() {
  if (gSampleSpiReady) return;

  Serial.println("[Sampler] init SD SPI bus (FSPI)");
  gSampleSpi.begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN);
  pinMode(SD_CS_PIN, OUTPUT);
  digitalWrite(SD_CS_PIN, HIGH);
  gSampleSpiReady = true;
}

static bool mountSampleSdAt(uint32_t hz) {
  for (uint8_t attempt = 0; attempt < 3; ++attempt) {
    if (SD.begin(SD_CS_PIN, gSampleSpi, hz)) {
      Serial.printf("[Sampler] SD mounted at %lu Hz\n", (unsigned long)hz);
      return true;
    }
    SD.end();
    digitalWrite(SD_CS_PIN, HIGH);
    delay(120);
  }
  return false;
}

static bool beginSampleSd() {
  if (gSampleSdReady) return true;

  beginSampleSpiBus();
  gSampleSdReady =
    mountSampleSdAt(SD_SPI_READY_FREQ_HZ) ||
    mountSampleSdAt(SD_SPI_RETRY_FREQ_HZ);
  if (!gSampleSdReady) {
    Serial.println("[Sampler] SD init failed");
    SD.end();
    digitalWrite(SD_CS_PIN, HIGH);
  } else {
    uint64_t cardMb = SD.cardSize() / (1024ULL * 1024ULL);
    uint64_t totalMb = SD.totalBytes() / (1024ULL * 1024ULL);
    uint64_t usedMb = SD.usedBytes() / (1024ULL * 1024ULL);
    Serial.printf("[Sampler] SD card=%lluMB fs=%lluMB used=%lluMB\n",
                  (unsigned long long)cardMb,
                  (unsigned long long)totalMb,
                  (unsigned long long)usedMb);
  }
  return gSampleSdReady;
}

static bool pathIsDirectory(const String& path) {
  File f = SD.open(path.c_str());
  if (!f) return false;
  bool isDir = f.isDirectory();
  f.close();
  return isDir;
}

static bool ensureDirectory(const String& path) {
  if (SD.exists(path.c_str())) {
    File existing = SD.open(path.c_str());
    bool isDir = existing && existing.isDirectory();
    size_t size = existing ? existing.size() : 0;
    if (existing) existing.close();
    if (isDir) return true;

    if (size == 0 && SD.remove(path.c_str())) {
      Serial.printf("[Sampler] replaced empty file with dir %s\n", path.c_str());
    } else {
      Serial.printf("[Sampler] path exists but is not a dir: %s\n", path.c_str());
      return false;
    }
  }

  if (SD.mkdir(path.c_str())) {
    Serial.printf("[Sampler] created dir %s\n", path.c_str());
    return true;
  }
  Serial.printf("[Sampler] mkdir failed: %s\n", path.c_str());
  return false;
}

static bool ensureSampleStorage() {
  if (gSampleStorageReady) return true;
  if (!beginSampleSd()) return false;

  if (!ensureDirectory(SAMPLE_ROOT_DIR)) {
    gSampleStorageReady = false;
    return false;
  }

  for (uint8_t i = 0; i < SAMPLE_PAD_COUNT; ++i) {
    String dir = String(SAMPLE_ROOT_DIR) + "/pad_" + String(i + 1);
    ensureDirectory(dir);
  }
  for (uint8_t i = 0; i < (uint8_t)(sizeof(SAMPLE_FILE_BASES) / sizeof(SAMPLE_FILE_BASES[0])); ++i) {
    String path = String(SAMPLE_ROOT_DIR) + "/" + SAMPLE_FILE_BASES[i];
    ensureDirectory(path);
  }

  gSampleStorageReady = true;
  Serial.println("[Sampler] SD sample structure ready");
  return true;
}

static bool existingPath(const String& path, String& outPath) {
  if (!SD.exists(path.c_str())) return false;
  if (pathIsDirectory(path)) return false;
  outPath = path;
  return true;
}

static bool loadWavIntoSlot(const String& path, SampleSlot& slot);

static bool findFirstWavInDirectory(const String& dirPath, String& outPath) {
  File dir = SD.open(dirPath.c_str());
  if (!dir || !dir.isDirectory()) {
    if (dir) dir.close();
    return false;
  }

  File entry = dir.openNextFile();
  while (entry) {
    if (!entry.isDirectory()) {
      String name = entry.name();
      if (endsWithNoCase(name, ".wav")) {
        outPath = name.startsWith("/") ? name : dirPath + "/" + name;
        entry.close();
        dir.close();
        return true;
      }
    }
    entry.close();
    entry = dir.openNextFile();
  }

  dir.close();
  return false;
}

static bool loadValidWavInDirectory(const String& dirPath,
                                    SampleSlot& slot,
                                    uint8_t padIndex) {
  File dir = SD.open(dirPath.c_str());
  if (!dir || !dir.isDirectory()) {
    if (dir) dir.close();
    return false;
  }

  bool sawWav = false;
  File entry = dir.openNextFile();
  while (entry) {
    if (!entry.isDirectory()) {
      String name = entry.name();
      if (endsWithNoCase(name, ".wav")) {
        sawWav = true;
        String path = name.startsWith("/") ? name : dirPath + "/" + name;
        entry.close();
        if (loadWavIntoSlot(path, slot)) {
          Serial.printf("[Sampler] pad_%u mapped to %s\n",
                        (unsigned)(padIndex + 1),
                        path.c_str());
          dir.close();
          return true;
        }
        Serial.printf("[Sampler] ignored invalid sample for pad_%u: %s\n",
                      (unsigned)(padIndex + 1),
                      path.c_str());
        entry = dir.openNextFile();
        continue;
      }
    }
    entry.close();
    entry = dir.openNextFile();
  }

  dir.close();
  if (sawWav) {
    Serial.printf("[Sampler] no valid WAV in %s for pad_%u\n",
                  dirPath.c_str(),
                  (unsigned)(padIndex + 1));
  }
  return false;
}

static bool loadCandidateFile(const String& path,
                              SampleSlot& slot,
                              uint8_t padIndex) {
  String existing;
  if (!existingPath(path, existing)) return false;
  if (loadWavIntoSlot(existing, slot)) {
    Serial.printf("[Sampler] pad_%u mapped to %s\n",
                  (unsigned)(padIndex + 1),
                  existing.c_str());
    return true;
  }
  Serial.printf("[Sampler] ignored invalid sample for pad_%u: %s\n",
                (unsigned)(padIndex + 1),
                existing.c_str());
  return false;
}

static bool loadSampleForPad(uint8_t padIndex, SampleSlot& slot) {
  if (padIndex < (uint8_t)(sizeof(SAMPLE_FILE_BASES) / sizeof(SAMPLE_FILE_BASES[0]))) {
    String namedBase = String(SAMPLE_ROOT_DIR) + "/" + SAMPLE_FILE_BASES[padIndex];
    if (loadValidWavInDirectory(namedBase, slot, padIndex)) return true;
    if (loadCandidateFile(namedBase + ".wav", slot, padIndex)) return true;
    if (loadCandidateFile(namedBase + ".WAV", slot, padIndex)) return true;
  }

  String padDir = String(SAMPLE_ROOT_DIR) + "/pad_" + String(padIndex + 1);
  if (loadValidWavInDirectory(padDir, slot, padIndex)) return true;

  String padBase = String(SAMPLE_ROOT_DIR) + "/pad_" + String(padIndex + 1);
  if (loadCandidateFile(padBase + ".wav", slot, padIndex)) return true;
  if (loadCandidateFile(padBase + ".WAV", slot, padIndex)) return true;

  return false;
}

static void resetSampleSlot(SampleSlot& slot) {
  slot.frames = 0;
  slot.sampleRate = SAMPLE_RATE;
  slot.loaded = false;
}

static void freeSampleBuffer(SampleSlot& slot) {
  if (slot.data) {
    free(slot.data);
  }
  memset(&slot, 0, sizeof(slot));
  slot.sampleRate = SAMPLE_RATE;
}

static uint32_t computeSampleBytesPerPadLimit() {
  uint32_t available = 0;
#if defined(ESP32) || defined(ARDUINO_ARCH_ESP32)
  available = (uint32_t)heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
  if (available == 0) {
    available = ESP.getFreeHeap();
  }
#endif
  if (available == 0) return 0;

  uint32_t usable = available;
  if (usable > PANORYTHE_SAMPLE_PSRAM_RESERVE_BYTES) {
    usable -= PANORYTHE_SAMPLE_PSRAM_RESERVE_BYTES;
  } else {
    usable /= 2;
  }

  uint32_t perPad = usable / SAMPLE_PAD_COUNT;
  uint32_t minTarget = PANORYTHE_SAMPLE_MIN_SECONDS_PER_PAD *
                       SAMPLE_RATE *
                       sizeof(int16_t);
  if (perPad < minTarget) {
    Serial.printf("[Sampler] sample buffer below %us target: %lu bytes/pad\n",
                  (unsigned)PANORYTHE_SAMPLE_MIN_SECONDS_PER_PAD,
                  (unsigned long)perPad);
  }
  return perPad;
}

static int16_t* allocateSampleFrames(uint32_t frames) {
  size_t bytes = (size_t)frames * sizeof(int16_t);
  if (bytes == 0) return nullptr;

  void* p = nullptr;
#if defined(ESP32) || defined(ARDUINO_ARCH_ESP32)
  if (psramFound()) {
    p = heap_caps_malloc(bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  }
#endif
  if (!p) {
    p = malloc(bytes);
  }
  return (int16_t*)p;
}

static bool ensureSampleBuffers() {
  if (gSampleBytesPerPadLimit == 0) {
    gSampleBytesPerPadLimit = computeSampleBytesPerPadLimit();
  }

  uint32_t capacityFrames = gSampleBytesPerPadLimit / sizeof(int16_t);
  if (capacityFrames == 0) {
    Serial.println("[Sampler] no RAM available for sample buffers");
    return false;
  }

  bool alreadyReady = true;
  for (uint8_t i = 0; i < SAMPLE_PAD_COUNT; ++i) {
    if (!gSampleSlots[i].data ||
        gSampleSlots[i].capacityFrames != capacityFrames) {
      alreadyReady = false;
      break;
    }
  }
  if (alreadyReady) return true;

  for (uint8_t i = 0; i < SAMPLE_PAD_COUNT; ++i) {
    freeSampleBuffer(gSampleSlots[i]);
  }

  for (uint8_t i = 0; i < SAMPLE_PAD_COUNT; ++i) {
    gSampleSlots[i].data = allocateSampleFrames(capacityFrames);
    if (!gSampleSlots[i].data) {
      Serial.printf("[Sampler] sample buffer alloc failed for pad_%u (%lu bytes)\n",
                    (unsigned)(i + 1),
                    (unsigned long)(capacityFrames * sizeof(int16_t)));
      for (uint8_t j = 0; j < SAMPLE_PAD_COUNT; ++j) {
        freeSampleBuffer(gSampleSlots[j]);
      }
      return false;
    }
    gSampleSlots[i].capacityFrames = capacityFrames;
    resetSampleSlot(gSampleSlots[i]);
  }

  Serial.printf("[Sampler] sample buffers ready: %lu bytes/pad\n",
                (unsigned long)(capacityFrames * sizeof(int16_t)));
  return true;
}

static bool readExact(File& file, void* dst, size_t len) {
  uint8_t* out = (uint8_t*)dst;
  size_t total = 0;
  while (total < len) {
    int got = file.read(out + total, len - total);
    if (got <= 0) return false;
    total += (size_t)got;
  }
  return true;
}

static bool readLe16(File& file, uint16_t& value) {
  uint8_t b[2];
  if (!readExact(file, b, sizeof(b))) return false;
  value = (uint16_t)b[0] | ((uint16_t)b[1] << 8);
  return true;
}

static bool readLe32(File& file, uint32_t& value) {
  uint8_t b[4];
  if (!readExact(file, b, sizeof(b))) return false;
  value = (uint32_t)b[0] |
          ((uint32_t)b[1] << 8) |
          ((uint32_t)b[2] << 16) |
          ((uint32_t)b[3] << 24);
  return true;
}

static bool readPcmSampleAs16(File& file, uint16_t bitsPerSample, int16_t& out) {
  if (bitsPerSample == 8) {
    uint8_t raw = 0;
    if (!readExact(file, &raw, 1)) return false;
    out = (int16_t)(((int32_t)raw - 128) << 8);
    return true;
  }

  if (bitsPerSample == 16) {
    uint16_t raw = 0;
    if (!readLe16(file, raw)) return false;
    out = (int16_t)raw;
    return true;
  }

  if (bitsPerSample == 24) {
    uint8_t b[3];
    if (!readExact(file, b, sizeof(b))) return false;
    int32_t raw = (int32_t)b[0] |
                  ((int32_t)b[1] << 8) |
                  ((int32_t)b[2] << 16);
    if (raw & 0x00800000) raw |= 0xFF000000;
    out = (int16_t)(raw >> 8);
    return true;
  }

  if (bitsPerSample == 32) {
    uint32_t raw = 0;
    if (!readLe32(file, raw)) return false;
    out = (int16_t)(((int32_t)raw) >> 16);
    return true;
  }

  return false;
}

static bool readFloatSampleAs16(File& file, uint16_t bitsPerSample, int16_t& out) {
  if (bitsPerSample != 32) return false;

  uint32_t raw = 0;
  if (!readLe32(file, raw)) return false;

  float value = 0.0f;
  memcpy(&value, &raw, sizeof(value));
  if (!isfinite(value)) value = 0.0f;
  value = clampf(value, -1.0f, 1.0f);
  out = (int16_t)(value * 32767.0f);
  return true;
}

static bool loadWavIntoSlot(const String& path, SampleSlot& slot) {
  resetSampleSlot(slot);
  if (!slot.data || slot.capacityFrames == 0) {
    Serial.printf("[Sampler] no buffer for %s\n", path.c_str());
    return false;
  }

  File file = SD.open(path.c_str(), FILE_READ);
  if (!file) {
    Serial.printf("[Sampler] open failed: %s\n", path.c_str());
    return false;
  }

  char riff[4];
  char wave[4];
  uint32_t riffSize = 0;
  if (!readExact(file, riff, sizeof(riff)) ||
      !readLe32(file, riffSize) ||
      !readExact(file, wave, sizeof(wave)) ||
      memcmp(riff, "RIFF", 4) != 0 ||
      memcmp(wave, "WAVE", 4) != 0) {
    Serial.printf("[Sampler] not a WAV file: %s\n", path.c_str());
    file.close();
    return false;
  }
  (void)riffSize;

  bool haveFmt = false;
  bool haveData = false;
  uint16_t audioFormat = 0;
  uint16_t channels = 0;
  uint32_t sampleRate = 0;
  uint16_t bitsPerSample = 0;
  uint16_t validBitsPerSample = 0;
  uint32_t dataOffset = 0;
  uint32_t dataBytes = 0;

  while (file.available()) {
    char chunkId[4];
    uint32_t chunkSize = 0;
    if (!readExact(file, chunkId, sizeof(chunkId)) ||
        !readLe32(file, chunkSize)) {
      break;
    }

    uint32_t chunkStart = (uint32_t)file.position();
    uint32_t nextChunk = chunkStart + chunkSize + (chunkSize & 1u);

    if (memcmp(chunkId, "fmt ", 4) == 0) {
      uint32_t byteRate = 0;
      uint16_t blockAlign = 0;
      if (chunkSize < 16 ||
          !readLe16(file, audioFormat) ||
          !readLe16(file, channels) ||
          !readLe32(file, sampleRate) ||
          !readLe32(file, byteRate) ||
          !readLe16(file, blockAlign) ||
          !readLe16(file, bitsPerSample)) {
        break;
      }
      (void)byteRate;
      (void)blockAlign;

      if (audioFormat == WAV_FORMAT_EXTENSIBLE && chunkSize >= 40) {
        uint16_t extraSize = 0;
        uint32_t channelMask = 0;
        uint8_t subFormatGuid[16];
        if (!readLe16(file, extraSize) ||
            !readLe16(file, validBitsPerSample) ||
            !readLe32(file, channelMask) ||
            !readExact(file, subFormatGuid, sizeof(subFormatGuid))) {
          break;
        }
        (void)extraSize;
        (void)channelMask;
        audioFormat = (uint16_t)subFormatGuid[0] |
                      ((uint16_t)subFormatGuid[1] << 8);
      }

      haveFmt = true;
      if (!file.seek(nextChunk)) break;
    } else if (memcmp(chunkId, "data", 4) == 0) {
      dataOffset = chunkStart;
      dataBytes = chunkSize;
      haveData = true;
      if (haveFmt) break;
      if (!file.seek(nextChunk)) break;
    } else {
      if (!file.seek(nextChunk)) break;
    }
  }

  bool isPcm = (audioFormat == WAV_FORMAT_PCM);
  bool isFloat = (audioFormat == WAV_FORMAT_IEEE_FLOAT);
  bool bitsOk = (isPcm &&
                 (bitsPerSample == 8 ||
                  bitsPerSample == 16 ||
                  bitsPerSample == 24 ||
                  bitsPerSample == 32)) ||
                (isFloat && bitsPerSample == 32);

  if (!haveFmt || !haveData ||
      (!isPcm && !isFloat) ||
      (channels != 1 && channels != 2) ||
      !bitsOk ||
      sampleRate == 0) {
    Serial.printf("[Sampler] unsupported WAV: %s\n", path.c_str());
    file.close();
    return false;
  }
  (void)validBitsPerSample;

  uint32_t bytesPerSample = bitsPerSample / 8;
  uint32_t bytesPerFrame = bytesPerSample * channels;
  uint32_t frames = dataBytes / bytesPerFrame;
  if (frames > slot.capacityFrames) {
    Serial.printf("[Sampler] truncating %s to %lu frames\n",
                  path.c_str(), (unsigned long)slot.capacityFrames);
    frames = slot.capacityFrames;
  }
  if (frames == 0) {
    Serial.printf("[Sampler] empty sample: %s\n", path.c_str());
    file.close();
    return false;
  }

  if (!file.seek(dataOffset)) {
    file.close();
    return false;
  }

  uint32_t loadedFrames = 0;
  for (; loadedFrames < frames; ++loadedFrames) {
    int32_t acc = 0;
    bool ok = true;
    for (uint16_t ch = 0; ch < channels; ++ch) {
      int16_t sample = 0;
      ok = isFloat
        ? readFloatSampleAs16(file, bitsPerSample, sample)
        : readPcmSampleAs16(file, bitsPerSample, sample);
      acc += sample;
      if (!ok) break;
    }
    if (!ok) break;
    slot.data[loadedFrames] = (int16_t)(acc / (int32_t)channels);
  }

  file.close();
  if (loadedFrames == 0) {
    return false;
  }

  slot.frames = loadedFrames;
  slot.sampleRate = sampleRate;
  slot.loaded = true;

  Serial.printf("[Sampler] loaded %s (%lu frames @ %lu Hz)\n",
                path.c_str(),
                (unsigned long)slot.frames,
                (unsigned long)slot.sampleRate);
  return true;
}

static bool loadAllSamplesFromSd() {
  if (!ensureSampleStorage()) return false;
  if (!ensureSampleBuffers()) return false;

  gLoadedSampleCount = 0;
  for (uint8_t i = 0; i < SAMPLE_PAD_COUNT; ++i) {
    resetSampleSlot(gSampleSlots[i]);

    if (!loadSampleForPad(i, gSampleSlots[i])) {
      Serial.printf("[Sampler] no sample for pad_%u\n", (unsigned)(i + 1));
      continue;
    }
    gLoadedSampleCount++;
  }

  Serial.printf("[Sampler] %u/%u samples loaded in RAM\n",
                (unsigned)gLoadedSampleCount,
                (unsigned)SAMPLE_PAD_COUNT);
  return true;
}

// Parse root names accepted by PANORYTHE_SCALE.
static bool parseRootSemitone(const char*& p, int& rootSemi) {
  if (!p || !*p) return false;

  switch (lowerAscii(*p++)) {
    case 'c': rootSemi = 0; break;
    case 'd': rootSemi = 2; break;
    case 'e': rootSemi = 4; break;
    case 'f': rootSemi = 5; break;
    case 'g': rootSemi = 7; break;
    case 'a': rootSemi = 9; break;
    case 'b': rootSemi = 11; break;
    default: return false;
  }

  char accidental = lowerAscii(*p);
  if (accidental == '#' || accidental == 's') {
    rootSemi++;
    p++;
  } else if (accidental == 'b') {
    rootSemi--;
    p++;
  }

  while (rootSemi < 0) rootSemi += 12;
  rootSemi %= 12;
  return true;
}

// Parse a signed integer octave.
static bool parseOctave(const char*& p, int& octave) {
  if (!p || !*p) return false;
  int sign = 1;
  if (*p == '-') {
    sign = -1;
    p++;
  }
  if (*p < '0' || *p > '9') return false;

  int value = 0;
  while (*p >= '0' && *p <= '9') {
    value = value * 10 + (*p - '0');
    p++;
  }
  octave = value * sign;
  return true;
}

// Read PANORYTHE_SCALE and cache the base mode/root/octave for note mapping.
static void configureScaleFromCode() {
  const char* p = PANORYTHE_SCALE;
  bool minor = true;

  if (startsWithNoCase(p, "min")) {
    minor = true;
    p += 3;
  } else if (startsWithNoCase(p, "maj")) {
    minor = false;
    p += 3;
  } else {
    p = nullptr;
  }

  int rootSemi = 0;
  int octave = 3;
  bool ok = p &&
            parseRootSemitone(p, rootSemi) &&
            parseOctave(p, octave) &&
            *p == '\0';

  int baseMidi = (octave + 1) * 12 + rootSemi;
  if (!ok || baseMidi < 0 || baseMidi > 127) {
    minor = true;
    baseMidi = 48;
    Serial.println("[Audio] invalid PANORYTHE_SCALE, using minC3.");
  }

  gScaleMinor = minor;
  gScaleBaseMidiNote = baseMidi;
  gScaleRootIndex = baseMidi % SCALE_ROOT_COUNT;
  Serial.printf("[Audio] scale %s, base MIDI note %d\n",
                gScaleMinor ? "minor" : "major",
                gScaleBaseMidiNote);
}

// Map note indices to a two-octave major or natural minor scale.
static int scaleStepForPad(uint8_t noteIndex) {
  static const int8_t majorSteps[16] = {
    0, 2, 4, 5, 7, 9, 11, 12,
    14, 16, 17, 19, 21, 23, 24, 26
  };
  static const int8_t minorSteps[16] = {
    0, 2, 3, 5, 7, 8, 10, 12,
    14, 15, 17, 19, 20, 22, 24, 26
  };
  uint8_t idx = noteIndex;
  if (idx >= 16) idx = 15;
  return gScaleMinor ? minorSteps[idx] : majorSteps[idx];
}

// Engine-specific attack times keep percussive sounds quick and breath/reed sounds playable.
static float attackSecondsFor(OscillatorEngine engine) {
  switch (engine) {
    case OSC_ENGINE_STEEL_DRUM: return 0.002f;
    case OSC_ENGINE_VAPOR_FLUTE: return 0.135f;
    case OSC_ENGINE_ACCORDION: return 0.038f;
    case OSC_ENGINE_OMNICHORD:
    default: return 0.018f;
  }
}

// Engine-specific release times define each sound tail.
static float releaseSecondsFor(OscillatorEngine engine) {
  switch (engine) {
    case OSC_ENGINE_STEEL_DRUM: return 0.18f;
    case OSC_ENGINE_VAPOR_FLUTE: return 1.55f;
    case OSC_ENGINE_ACCORDION: return 0.32f;
    case OSC_ENGINE_OMNICHORD:
    default: return 1.05f;
  }
}

// Convert envelope time in seconds to a per-sample linear increment.
static float stepFromSeconds(float seconds) {
  if (seconds <= 0.0f) return 1.0f;
  return 1.0f / (seconds * (float)SAMPLE_RATE);
}

// Reset a voice slot to a silent reusable state.
static void clearVoice(Voice& v) {
  memset(&v, 0, sizeof(v));
  v.envState = ENV_IDLE;
}

static void clearSampleVoice(SampleVoice& v) {
  memset(&v, 0, sizeof(v));
}

static void clearAllSynthVoices() {
  for (uint8_t i = 0; i < MAX_VOICES; ++i) {
    clearVoice(gVoices[i]);
    clearVoice(gPendingVoices[i]);
    gStealRemaining[i] = 0;
  }
}

static void clearAllSampleVoices() {
  for (uint8_t i = 0; i < MAX_SAMPLE_VOICES; ++i) {
    clearSampleVoice(gSampleVoices[i]);
  }
}

static void resetReverbState() {
  memset(gReverbA, 0, sizeof(gReverbA));
  memset(gReverbB, 0, sizeof(gReverbB));
  gReverbIndexA = 0;
  gReverbIndexB = 0;
}

static void resetEchoLoopPlayback() {
  memset(gEchoLoopVoices, 0, sizeof(gEchoLoopVoices));
  gEchoLoopCountdown = 0;
  gEchoLoopRng ^= gEchoLoopSampleCounter + 0x9E3779B9u;
}

static void resetEchoLoopHistory() {
  memset(gEchoLoopBuffer, 0, sizeof(gEchoLoopBuffer));
  memset(gEchoLoopNoteStarts, 0, sizeof(gEchoLoopNoteStarts));
  gEchoLoopWriteIndex = 0;
  gEchoLoopSampleCounter = 0;
  gEchoLoopNoteWrite = 0;
  gEchoLoopNoteCount = 0;
  resetEchoLoopPlayback();
}

// Change algorithms only after fading to dry, at an audio block boundary.
static void serviceSpaceFxRequest() {
  gNextSpaceFxMode = gSpaceFxModeRequested.load(std::memory_order_relaxed);
  // Preserve the old reverb level while its output fades away.
  if (gNextSpaceFxMode == gSpaceFxMode || gNextSpaceFxMode == SPACE_FX_WARM_REVERB) {
    gReverbAmount = gReverbAmountRequested.load(std::memory_order_relaxed);
  }
  if (gNextSpaceFxMode == gSpaceFxMode || gSpaceFxBlend > 0.0f) return;

  if (gNextSpaceFxMode == SPACE_FX_WARM_REVERB) resetReverbState();
  resetEchoLoopPlayback();
  if (gSpaceFxMode == SPACE_FX_MIETTES || gNextSpaceFxMode == SPACE_FX_MIETTES) {
    GranularFx::reset();
  }
  gSpaceFxMode = gNextSpaceFxMode;
}

static void rememberEchoLoopNoteStart() {
  gEchoLoopNoteStarts[gEchoLoopNoteWrite] = gEchoLoopSampleCounter;
  gEchoLoopNoteWrite = (uint8_t)((gEchoLoopNoteWrite + 1u) % ECHOLOOP_NOTE_HISTORY);
  if (gEchoLoopNoteCount < ECHOLOOP_NOTE_HISTORY) {
    gEchoLoopNoteCount++;
  }
}

// Find a free voice, then prefer released voices, then steal the oldest voice.
static int allocateVoice() {
  int oldest = 0;
  uint32_t oldestSeq = UINT32_MAX;
  float quietestRelease = 2.0f;
  int releaseVoice = -1;

  for (uint8_t i = 0; i < MAX_VOICES; ++i) {
    if (!gVoices[i].active || gVoices[i].envState == ENV_IDLE) return i;
    if (!gStealRemaining[i] && gVoices[i].envState == ENV_RELEASE && gVoices[i].env < quietestRelease) {
      quietestRelease = gVoices[i].env;
      releaseVoice = i;
    }
    uint32_t sequence = gStealRemaining[i] ? gPendingVoices[i].startedAt : gVoices[i].startedAt;
    if (sequence < oldestSeq) {
      oldestSeq = sequence;
      oldest = i;
    }
  }
  return (releaseVoice >= 0) ? releaseVoice : oldest;
}

static int allocateSampleVoice() {
  int oldest = 0;
  uint32_t oldestSeq = UINT32_MAX;
  for (uint8_t i = 0; i < MAX_SAMPLE_VOICES; ++i) {
    if (!gSampleVoices[i].active) return i;
    if (gSampleVoices[i].startedAt < oldestSeq) {
      oldestSeq = gSampleVoices[i].startedAt;
      oldest = i;
    }
  }
  return oldest;
}

static void configureBodyResonators(Voice& v,
                                    const float* frequencies,
                                    float dampingBase,
                                    float dampingStep) {
  if (!frequencies) return;

  for (uint8_t i = 0; i < 3; ++i) {
    float hz = clampf(frequencies[i], 40.0f, 8000.0f);
    v.formantCoeff[i] = 2.0f * sinf(PI_F * hz / (float)SAMPLE_RATE);
    v.formantDamping[i] = dampingBase + dampingStep * (float)i;
  }
}

// Initialize all per-voice state for a new note.
static void configureVoice(Voice& v, uint8_t noteIndex) {
  clearVoice(v);
  v.active = true;
  v.noteIndex = noteIndex;
  v.engine = gEngine;
  v.envState = ENV_ATTACK;
  v.attackStep = stepFromSeconds(attackSecondsFor(v.engine));
  v.releaseStep = stepFromSeconds(releaseSecondsFor(v.engine));
  v.freq = midiToFrequency(midiNoteFromNoteIndex(noteIndex));
  v.phaseA = (float)(noteIndex % 7) * 0.071f;
  v.phaseB = 0.25f + (float)(noteIndex % 5) * 0.041f;
  v.phaseC = 0.50f + (float)(noteIndex % 3) * 0.053f;
  v.phaseD = 0.75f + (float)(noteIndex % 11) * 0.019f;
  v.pluckEnv = 1.0f;
  v.noiseEnv = 1.0f;
  v.rng = 0xA341316Cu ^ ((uint32_t)noteIndex * 0x45D9F3Bu) ^ gVoiceSeq;
  v.startedAt = gVoiceSeq++;

  switch (v.engine) {
    case OSC_ENGINE_VAPOR_FLUTE:
      configureBodyResonators(v, FLUTE_AIR_FORMANTS, 0.18f, 0.045f);
      break;
    case OSC_ENGINE_ACCORDION:
      configureBodyResonators(v, ACCORDION_BODY_FORMANTS, 0.10f, 0.030f);
      break;
    default:
      break;
  }
}

// Move a held voice into its release stage.
static void startRelease(Voice& v) {
  if (!v.active || v.envState == ENV_IDLE || v.envState == ENV_RELEASE) return;
  v.envState = ENV_RELEASE;
  v.releaseStep = stepFromSeconds(releaseSecondsFor(v.engine));
}

// Advance the simple attack/sustain/release envelope by one sample.
static void updateEnvelope(Voice& v) {
  if (v.envState == ENV_ATTACK) {
    v.env += v.attackStep;
    if (v.env >= 1.0f) {
      v.env = 1.0f;
      v.envState = ENV_SUSTAIN;
    }
  } else if (v.envState == ENV_RELEASE) {
    v.env -= v.releaseStep;
    if (v.env <= 0.0f) {
      clearVoice(v);
    }
  }
}

// Process one state-variable band-pass formant stage.
static float processFormant(FormantState& st, float input, float coeff, float damping) {
  float high = input - st.low - damping * st.band;
  st.band += coeff * high;
  st.low += coeff * st.band;
  return st.band;
}

// Steel drum: fast mallet strike, inharmonic partials, and an automatic tail.
static float renderSteelDrum(Voice& v) {
  float inc = v.freq / (float)SAMPLE_RATE;
  v.phaseA = wrap01(v.phaseA + inc);
  v.phaseB = wrap01(v.phaseB + inc * 2.012f);
  v.phaseC = wrap01(v.phaseC + inc * 3.874f);
  v.phaseD = wrap01(v.phaseD + inc * 5.392f);

  v.pluckEnv *= 0.99983f;
  v.noiseEnv *= 0.9925f;

  float bright = v.pluckEnv * v.pluckEnv;
  float s = 0.58f * sine01(v.phaseA);
  s += bright * (0.34f * sine01(v.phaseB) +
                 0.23f * sine01(v.phaseC) +
                 0.13f * sine01(v.phaseD));
  s *= v.pluckEnv;
  s += whiteNoise(v.rng) * (0.055f * v.noiseEnv);

  v.lp += (s - v.lp) * 0.22f;
  if (v.pluckEnv < 0.00045f && v.noiseEnv < 0.00045f) {
    clearVoice(v);
  }
  return fastTanh((s * 0.78f + v.lp * 0.22f) * 1.35f) * 1.02f;
}

// Vapor flute: soft sine body with breath noise through lightweight resonators.
static float renderVaporFlute(Voice& v) {
  float inc = v.freq / (float)SAMPLE_RATE;
  v.phaseC = wrap01(v.phaseC + 0.19f / (float)SAMPLE_RATE);
  v.phaseD = wrap01(v.phaseD + 5.15f / (float)SAMPLE_RATE);

  float vibrato = sine01(v.phaseD) * 0.0038f;
  float pitchInc = inc * (1.0f + vibrato);
  v.phaseA = wrap01(v.phaseA + pitchInc);
  v.phaseB = wrap01(v.phaseB + pitchInc * 2.003f);

  v.noiseEnv += (0.22f - v.noiseEnv) * 0.00035f;
  float noise = whiteNoise(v.rng);
  v.lp += (noise - v.lp) * 0.020f;
  float air = noise - v.lp;

  float resonantAir = 0.0f;
  for (uint8_t i = 0; i < 3; ++i) {
    resonantAir += FLUTE_AIR_GAINS[i] * processFormant(v.formant[i],
                                                       air,
                                                       v.formantCoeff[i],
                                                       v.formantDamping[i]);
  }

  float movement = 0.92f + 0.08f * sine01(v.phaseC);
  float tone = 0.72f * sine01(v.phaseA) + 0.12f * sine01(v.phaseB);
  tone *= movement;
  tone += resonantAir * (0.52f + v.noiseEnv);
  return fastTanh(tone * 0.95f) * 0.72f;
}

// Accordion: detuned free reeds with box resonances and a short key transient.
static float renderAccordion(Voice& v) {
  float inc = v.freq / (float)SAMPLE_RATE;
  v.phaseA = wrap01(v.phaseA + inc * 0.9965f);
  v.phaseB = wrap01(v.phaseB + inc * 1.0045f);
  v.phaseC = wrap01(v.phaseC + inc * 2.000f);
  v.phaseD = wrap01(v.phaseD + inc * 3.000f);

  v.noiseEnv *= 0.9950f;
  float reed = 0.39f * sine01(v.phaseA);
  reed += 0.39f * sine01(v.phaseB);
  reed += 0.14f * triangle01(v.phaseA);
  reed += 0.11f * triangle01(v.phaseB);
  reed += 0.13f * sine01(v.phaseC);
  reed += 0.055f * sine01(v.phaseD);

  float body = 0.0f;
  for (uint8_t i = 0; i < 3; ++i) {
    body += ACCORDION_BODY_GAINS[i] * processFormant(v.formant[i],
                                                     reed,
                                                     v.formantCoeff[i],
                                                     v.formantDamping[i]);
  }
  v.lp += (reed - v.lp) * 0.085f;

  float keyNoise = whiteNoise(v.rng) * (0.015f * v.noiseEnv);
  float s = reed * 0.68f + body + v.lp * 0.22f + keyNoise;
  return fastTanh(s * 1.25f) * 0.78f;
}

// Omnichord: chorused chord-machine voice with a slow internal shimmer.
static float renderOmnichord(Voice& v) {
  float inc = v.freq / (float)SAMPLE_RATE;
  v.phaseD = wrap01(v.phaseD + 0.32f / (float)SAMPLE_RATE);
  float move = sine01(v.phaseD);
  float shimmer = sine01(v.phaseD + 0.25f);

  v.phaseA = wrap01(v.phaseA + inc * (0.9985f + 0.0010f * move));
  v.phaseB = wrap01(v.phaseB + inc * 2.002f);
  v.phaseC = wrap01(v.phaseC + inc * 1.498f);

  v.pluckEnv += (0.38f - v.pluckEnv) * 0.00018f;

  float chord = 0.42f * triangle01(v.phaseA);
  chord += 0.28f * sine01(v.phaseB);
  chord += (0.17f + 0.045f * shimmer) * sine01(v.phaseC);
  chord += 0.08f * triangle01(v.phaseB);

  float filterStep = 0.032f + 0.022f * (0.5f + 0.5f * move);
  v.lp += (chord - v.lp) * filterStep;

  float pulse = 0.86f + 0.10f * shimmer;
  float pick = 0.82f + 0.18f * v.pluckEnv;
  return fastTanh((v.lp * 0.64f + chord * 0.36f) * 1.05f) * pulse * pick * 0.82f;
}

// Render one voice sample after envelope processing.
static float renderVoice(Voice& v) {
  updateEnvelope(v);
  if (!v.active) return 0.0f;

  float s = 0.0f;
  switch (v.engine) {
    case OSC_ENGINE_STEEL_DRUM:
      s = renderSteelDrum(v);
      break;
    case OSC_ENGINE_VAPOR_FLUTE:
      s = renderVaporFlute(v);
      break;
    case OSC_ENGINE_ACCORDION:
      s = renderAccordion(v);
      break;
    case OSC_ENGINE_OMNICHORD:
    default:
      s = renderOmnichord(v);
      break;
  }
  return s * v.env;
}

static float renderSampleVoice(SampleVoice& v) {
  if (!v.active || v.slotIndex >= SAMPLE_PAD_COUNT) return 0.0f;

  SampleSlot& slot = gSampleSlots[v.slotIndex];
  if (!slot.loaded || !slot.data || slot.frames == 0) {
    clearSampleVoice(v);
    return 0.0f;
  }

  uint32_t idx = (uint32_t)v.position;
  if (idx >= slot.frames) {
    clearSampleVoice(v);
    return 0.0f;
  }

  uint32_t nextIdx = idx + 1;
  if (nextIdx >= slot.frames) nextIdx = idx;
  float frac = v.position - (float)idx;
  float a = (float)slot.data[idx] / 32768.0f;
  float b = (float)slot.data[nextIdx] / 32768.0f;
  float s = a + (b - a) * frac;

  v.position += v.increment;
  if ((uint32_t)v.position >= slot.frames) {
    clearSampleVoice(v);
  }

  return s * 0.95f;
}

static inline uint16_t echoLoopIndex(uint32_t sampleIndex) {
  return (uint16_t)(sampleIndex & ECHOLOOP_BUFFER_MASK);
}

static void writeEchoLoopHistory(float x) {
  float sample = clampf(finiteOrZero(x), -1.0f, 1.0f);
  gEchoLoopBuffer[gEchoLoopWriteIndex] = (int16_t)(sample * 32767.0f);
  gEchoLoopWriteIndex = echoLoopIndex((uint32_t)gEchoLoopWriteIndex + 1u);
  gEchoLoopSampleCounter++;
}

static int allocateEchoLoopVoice() {
  for (uint8_t i = 0; i < ECHOLOOP_VOICE_COUNT; ++i) {
    if (!gEchoLoopVoices[i].active) return i;
  }
  return -1;
}

static void scheduleNextEchoLoopGrain(bool fastRetry) {
  uint32_t base = fastRetry ? 700u : 1800u;
  uint32_t span = fastRetry ? 1800u : 8500u;
  gEchoLoopCountdown = (uint16_t)(base + randomRange(gEchoLoopRng, span));
}

static bool chooseEchoLoopReadIndex(uint16_t length, uint16_t& readIndex) {
  uint32_t available = gEchoLoopSampleCounter;
  if (available > ECHOLOOP_BUFFER_SIZE) available = ECHOLOOP_BUFFER_SIZE;
  if (available <= (uint32_t)length + 512u) return false;

  if (gEchoLoopNoteCount > 0) {
    for (uint8_t attempt = 0; attempt < 8; ++attempt) {
      uint8_t idx = (uint8_t)randomRange(gEchoLoopRng, gEchoLoopNoteCount);
      uint32_t startSample = gEchoLoopNoteStarts[idx];
      uint32_t age = gEchoLoopSampleCounter - startSample;
      if (age <= (uint32_t)length + 512u ||
          age >= ECHOLOOP_BUFFER_SIZE - 256u ||
          age > available) {
        continue;
      }

      uint32_t maxJitter = age - (uint32_t)length - 256u;
      if (maxJitter > 1800u) maxJitter = 1800u;
      readIndex = echoLoopIndex(startSample + randomRange(gEchoLoopRng, maxJitter));
      return true;
    }
  }

  uint32_t maxDelay = available - 256u;
  if (maxDelay <= (uint32_t)length + 256u) return false;
  uint32_t span = maxDelay - (uint32_t)length - 256u;
  uint32_t delay = (uint32_t)length + 256u + randomRange(gEchoLoopRng, span);
  readIndex = echoLoopIndex(gEchoLoopSampleCounter - delay);
  return true;
}

static bool startEchoLoopGrain() {
  int slot = allocateEchoLoopVoice();
  if (slot < 0) return false;

  uint16_t length = (uint16_t)(1900u + randomRange(gEchoLoopRng, 6400u));
  uint16_t readIndex = 0;
  if (!chooseEchoLoopReadIndex(length, readIndex)) return false;

  uint16_t fadeSamples = (uint16_t)(length / 6u);
  if (fadeSamples < 192u) fadeSamples = 192u;
  if (fadeSamples > 768u) fadeSamples = 768u;

  EchoLoopVoice& voice = gEchoLoopVoices[slot];
  voice.active = true;
  voice.readIndex = readIndex;
  voice.remaining = length;
  voice.length = length;
  voice.fadeSamples = fadeSamples;
  voice.fadeScale = 1.0f / (float)fadeSamples;
  voice.level = 0.34f + 0.16f * ((float)randomRange(gEchoLoopRng, 256u) / 255.0f);
  return true;
}

// Lighter warm reverb: no integer modulo or nested saturators in the sample path.
static float processReverb(float x) {
  if (gReverbAmount <= 0.001f) return x;

  float amount = gReverbAmount;
  float input = finiteOrZero(x) * 0.72f;
  float a = gReverbA[gReverbIndexA];
  float b = gReverbB[gReverbIndexB];
  float wet = 0.58f * a + 0.43f * b;
  float feedback = 0.48f + 0.18f * amount;

  gReverbA[gReverbIndexA] = clampf(input + b * feedback, -1.2f, 1.2f);
  gReverbB[gReverbIndexB] = clampf(input * 0.64f + a * (feedback * 0.84f), -1.2f, 1.2f);

  gReverbIndexA = (gReverbIndexA + 1u) & REVERB_A_MASK;
  gReverbIndexB = (gReverbIndexB + 1u) & REVERB_B_MASK;

  return x * (1.0f - amount * 0.16f) + wet * (amount * 0.70f);
}

// Aggressive distortion with fixed output compensation before the final limiter.
static float processDrive(float x) {
  static_assert(DRIVE_OUTPUT_GAIN >= 0.0f && DRIVE_OUTPUT_GAIN <= 1.0f,
                "DRIVE_OUTPUT_GAIN must be 0..1");
  float input = finiteOrZero(x);
  float stageA = fastTanh(input * 8.5f);
  float stageB = fastTanh((stageA * 1.55f + input * 0.42f) * 2.7f);
  float out = stageB * 0.52f + stageA * 0.11f + input * 0.05f;
  return clampf(finiteOrZero(out), -0.72f, 0.72f) * DRIVE_OUTPUT_GAIN;
}

// Random micro-loop echo from recent audio note starts and short history grains.
static float processEchoLoop(float x) {
  if (gEchoLoopCountdown > 0) {
    gEchoLoopCountdown--;
  } else {
    bool started = startEchoLoopGrain();
    scheduleNextEchoLoopGrain(!started);
  }

  float wet = 0.0f;
  for (uint8_t i = 0; i < ECHOLOOP_VOICE_COUNT; ++i) {
    EchoLoopVoice& voice = gEchoLoopVoices[i];
    if (!voice.active) continue;

    uint16_t played = (uint16_t)(voice.length - voice.remaining);
    float env = 1.0f;
    if (played < voice.fadeSamples) {
      env = (float)(played + 1u) * voice.fadeScale;
    }
    if (voice.remaining < voice.fadeSamples) {
      float tail = (float)voice.remaining * voice.fadeScale;
      if (tail < env) env = tail;
    }

    wet += ((float)gEchoLoopBuffer[voice.readIndex] / 32768.0f) * env * voice.level;
    voice.readIndex = echoLoopIndex((uint32_t)voice.readIndex + 1u);
    if (voice.remaining > 0) voice.remaining--;
    if (voice.remaining == 0) voice.active = false;
  }

  return clampf(x * 0.78f + wet * 0.86f, -1.2f, 1.2f);
}

static float processSpaceFx(float x) {
  writeEchoLoopHistory(x);

  float effected = x;
  switch (gSpaceFxMode) {
    case SPACE_FX_WARM_REVERB:
      effected = processReverb(x);
      break;
    case SPACE_FX_DRIVE:
      effected = processDrive(x);
      break;
    case SPACE_FX_ECHOLOOP:
      effected = processEchoLoop(x);
      break;
    case SPACE_FX_MIETTES:
      effected = GranularFx::process(x);
      break;
    case SPACE_FX_OFF:
    default:
      return x;
  }
  float target = (gNextSpaceFxMode == gSpaceFxMode) ? 1.0f : 0.0f;
  if (gSpaceFxBlend < target) {
    gSpaceFxBlend = clamp01(gSpaceFxBlend + SPACE_FX_BLEND_STEP);
  } else if (gSpaceFxBlend > target) {
    gSpaceFxBlend = clamp01(gSpaceFxBlend - SPACE_FX_BLEND_STEP);
  }
  return x + (effected - x) * gSpaceFxBlend;
}

// Final soft limiter before converting to 16-bit PCM.
static float softLimit(float x) {
  return fastTanh(finiteOrZero(x) * 1.15f) * 0.90f;
}

// Configure I2S for the PCM5102 output stage.
static void i2sInit() {
  if (gI2sReady) return;

  i2s_config_t cfg{};
  cfg.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX);
  cfg.sample_rate = SAMPLE_RATE;
  cfg.bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT;
  cfg.channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT;
  cfg.communication_format = I2S_COMM_FORMAT_STAND_MSB;
  cfg.intr_alloc_flags = 0;
  cfg.dma_buf_count = I2S_DMA_BUF_COUNT;
  cfg.dma_buf_len = I2S_DMA_BUF_LEN;
  cfg.use_apll = false;
  cfg.tx_desc_auto_clear = true;
  cfg.fixed_mclk = 0;

  i2s_pin_config_t pins{};
  pins.bck_io_num = I2S_BCLK;
  pins.ws_io_num = I2S_LRCK;
  pins.data_out_num = I2S_DOUT;
  pins.data_in_num = I2S_PIN_NO_CHANGE;

  esp_err_t installErr = i2s_driver_install(I2S_NUM_0, &cfg, 0, nullptr);
  esp_err_t pinErr = ESP_FAIL;
  if (installErr == ESP_OK) {
    pinErr = i2s_set_pin(I2S_NUM_0, &pins);
    if (pinErr == ESP_OK) {
      i2s_zero_dma_buffer(I2S_NUM_0);
    }
  }
  gI2sReady = (installErr == ESP_OK && pinErr == ESP_OK);
  if (gI2sReady) {
    Serial.println("[Audio] I2S ready");
  } else {
    Serial.printf("[Audio] I2S init failed install=%d pin=%d\n",
                  (int)installErr, (int)pinErr);
  }
}

// Render one stereo block into gOut. Both channels currently carry the same mix.
static void renderChunk() {
  uint8_t activeCount = 0;
  if (gSamplerMode) {
    for (uint8_t i = 0; i < MAX_SAMPLE_VOICES; ++i) {
      if (gSampleVoices[i].active) activeCount++;
    }
  } else {
    for (uint8_t i = 0; i < MAX_VOICES; ++i) {
      if (gVoices[i].active) activeCount++;
    }
  }
  float polyTrim = 1.0f;
  if (activeCount > 1) {
    polyTrim = 1.0f / sqrtf((float)activeCount);
  }

  static float smoothedPolyTrim = 1.0f;
  for (size_t i = 0; i < CHUNK_SAMPLES; ++i) {
    smoothedPolyTrim += (polyTrim - smoothedPolyTrim) * (1.0f / 441.0f);
    float mix = 0.0f;
    if (gSamplerMode) {
      for (uint8_t v = 0; v < MAX_SAMPLE_VOICES; ++v) {
        if (!gSampleVoices[v].active) continue;
        mix += renderSampleVoice(gSampleVoices[v]);
      }
    } else {
      for (uint8_t v = 0; v < MAX_VOICES; ++v) {
        if (!gVoices[v].active) continue;
        float sample = renderVoice(gVoices[v]);
        if (gStealRemaining[v]) {
          // Raised-cosine fade reaches silence before reusing this DSP slot.
          float progress = (float)(STEAL_SAMPLES - gStealRemaining[v] + 1u) / STEAL_SAMPLES;
          sample *= 0.5f + 0.5f * sine01(0.25f + 0.5f * progress);
          if (--gStealRemaining[v] == 0 || !gVoices[v].active) {
            gVoices[v] = gPendingVoices[v];
            clearVoice(gPendingVoices[v]);
            gStealRemaining[v] = 0;
          }
        }
        mix += sample;
      }
    }
    mix *= smoothedPolyTrim;
    mix = softLimit(mix);
    mix = processSpaceFx(mix);
    mix = softLimit(mix * gVolume);

    float pcm = finiteOrZero(mix) * OUTPUT_PCM_SCALE;
    if (pcm > 32767.0f) pcm = 32767.0f;
    if (pcm < -32768.0f) pcm = -32768.0f;
    int16_t s = (int16_t)pcm;
    gOut[i * 2] = s;
    gOut[i * 2 + 1] = s;
  }
}

static void startQueuedSample(uint8_t noteIndex) {
  if (noteIndex >= SAMPLE_PAD_COUNT) return;
  SampleSlot& sample = gSampleSlots[noteIndex];
  if (!sample.loaded || !sample.data || sample.frames == 0) return;

  rememberEchoLoopNoteStart();

  int slot = allocateSampleVoice();
  SampleVoice& v = gSampleVoices[slot];
  clearSampleVoice(v);
  v.active = true;
  v.noteIndex = noteIndex;
  v.slotIndex = noteIndex;
  v.position = 0.0f;
  v.increment = (float)sample.sampleRate / (float)SAMPLE_RATE;
  if (v.increment <= 0.0f) v.increment = 1.0f;
  v.startedAt = gVoiceSeq++;
}

static void releaseQueuedSample(uint8_t noteIndex) {
  (void)noteIndex;
  // Samples are one-shot and stop naturally at the end of the loaded buffer.
}

// Start a note on the current engine. Called only by the audio task.
static void startQueuedNote(uint8_t noteIndex) {
  if (noteIndex >= MAX_NOTE_INDEX) return;
  if (gSpaceFxMode == SPACE_FX_MIETTES) GranularFx::noteOn();
  if (gSamplerMode) {
    startQueuedSample(noteIndex);
    return;
  }
  rememberEchoLoopNoteStart();
  int slot = allocateVoice();
  if (gVoices[slot].active) {
    configureVoice(gPendingVoices[slot], noteIndex);
    if (!gStealRemaining[slot]) gStealRemaining[slot] = STEAL_SAMPLES;
  } else {
    configureVoice(gVoices[slot], noteIndex);
  }
}

// Release every active voice that belongs to this pad. Called only by the audio task.
static void releaseQueuedNote(uint8_t noteIndex) {
  if (gSamplerMode) {
    releaseQueuedSample(noteIndex);
    return;
  }
  for (uint8_t i = 0; i < MAX_VOICES; ++i) {
    if (gPendingVoices[i].active && gPendingVoices[i].noteIndex == noteIndex) {
      startRelease(gPendingVoices[i]);
    }
    if (gVoices[i].active && gVoices[i].noteIndex == noteIndex) {
      startRelease(gVoices[i]);
    }
  }
}

static void setSamplerModeOnAudioTask(bool enabled) {
  clearAllSynthVoices();
  clearAllSampleVoices();
  gSamplerMode = enabled;
  Serial.printf("[Audio] mode %s\n", enabled ? "sampler" : "synth");
}

// Consume input events at block boundaries so the input task never edits voices mid-render.
static void processAudioEvents() {
  if (!gAudioEventQueue) return;

  AudioEvent ev{};
  while (xQueueReceive(gAudioEventQueue, &ev, 0) == pdTRUE) {
    if (ev.type == AUDIO_EVENT_NOTE_ON) {
      startQueuedNote(ev.noteIndex);
    } else if (ev.type == AUDIO_EVENT_NOTE_OFF) {
      releaseQueuedNote(ev.noteIndex);
    } else if (ev.type == AUDIO_EVENT_SET_SAMPLER_MODE) {
      setSamplerModeOnAudioTask(ev.samplerMode);
    }
  }
}

static bool pushAudioEvent(const AudioEvent& ev) {
  if (!gAudioEventQueue) return false;

  bool queued = xQueueSend(gAudioEventQueue, &ev, 0) == pdTRUE;
  if (!queued && !gAudioEventOverflowLogged) {
    gAudioEventOverflowLogged = true;
    Serial.println("[Audio] event queue full");
  }
  return queued;
}

// Queue a note event from the input task without blocking the real-time system.
static void queueAudioEvent(AudioEventType type, uint8_t noteIndex) {
  if (noteIndex >= MAX_NOTE_INDEX) return;

  AudioEvent ev{};
  ev.type = type;
  ev.noteIndex = noteIndex;
  ev.samplerMode = false;
  pushAudioEvent(ev);
}

static bool queueSamplerModeEvent(bool enabled) {
  AudioEvent ev{};
  ev.type = AUDIO_EVENT_SET_SAMPLER_MODE;
  ev.noteIndex = 0;
  ev.samplerMode = enabled;
  return pushAudioEvent(ev);
}

} // namespace

// Public setup: clear DSP state and start I2S.
void init() {
  configureScaleFromCode();
  buildSineTable();
  if (!gAudioEventQueue) {
    gAudioEventQueue = xQueueCreateStatic(
      AUDIO_EVENT_QUEUE_LEN,
      sizeof(AudioEvent),
      gAudioEventQueueStorage,
      &gAudioEventQueueState
    );
  } else {
    xQueueReset(gAudioEventQueue);
  }
  gAudioEventOverflowLogged = false;

  gSamplerModeRequested = false;
  gSamplerMode = false;
  clearAllSynthVoices();
  clearAllSampleVoices();
  ensureSampleBuffers();
  ensureSampleStorage();

  resetReverbState();
  resetEchoLoopHistory();
  static_assert(MIETTES_MIX >= 0.0f && MIETTES_MIX <= 1.0f, "MIETTES_MIX must be 0..1");
  static_assert(MIETTES_FEEDBACK >= 0.0f && MIETTES_FEEDBACK <= 0.65f,
                "MIETTES_FEEDBACK must be 0..0.65");
  static_assert(MIETTES_BPM >= 40 && MIETTES_BPM <= 240, "MIETTES_BPM must be 40..240");
  static_assert(MIETTES_PITCH_RANDOM >= 0.0f && MIETTES_PITCH_RANDOM <= 1.0f,
                "MIETTES_PITCH_RANDOM must be 0..1");
  static_assert(MIETTES_TIME_RANDOM >= 0.0f && MIETTES_TIME_RANDOM <= 1.0f,
                "MIETTES_TIME_RANDOM must be 0..1");
  GranularFx::configure(MIETTES_MIX, MIETTES_FEEDBACK, MIETTES_BPM,
                       MIETTES_PITCH_RANDOM, MIETTES_TIME_RANDOM);
  GranularFx::setRandomSeed(micros());
  GranularFx::reset();
  gSpaceFxMode = SPACE_FX_OFF;
  gNextSpaceFxMode = SPACE_FX_OFF;
  gSpaceFxBlend = 0.0f;
  gSpaceFxModeRequested.store(SPACE_FX_OFF, std::memory_order_relaxed);
  i2sInit();
  Serial.println("[Audio] engine ready");
}

// Audio task entry point: render one block and block until I2S accepts it.
void update() {
  if (!gI2sReady) {
    delay(1);
    return;
  }

  uint32_t renderStartUs = AUDIO_REPORT_RENDER_LOAD ? micros() : 0;
  serviceSpaceFxRequest();
  processAudioEvents();
  renderChunk();
  if (AUDIO_REPORT_RENDER_LOAD) {
    static uint32_t peakUs = 0;
    static uint16_t measuredBlocks = 0;
    uint32_t elapsedUs = micros() - renderStartUs;
    if (elapsedUs > peakUs) peakUs = elapsedUs;
    if (++measuredBlocks == 1024) {
      gRenderPeakUs.store(peakUs, std::memory_order_relaxed);
      measuredBlocks = 0;
      peakUs = 0;
    }
  }
  size_t written = 0;
  const size_t expected = sizeof(gOut);
  esp_err_t err = i2s_write(I2S_NUM_0, (const char*)gOut, expected,
                            &written, portMAX_DELAY);
  if (err != ESP_OK || written != expected) {
    Serial.printf("[Audio] I2S write error err=%d bytes=%u/%u\n",
                  (int)err, (unsigned)written, (unsigned)expected);
  }
}

// Reserved low-priority service hook.
void service() {
  if (AUDIO_REPORT_RENDER_LOAD) {
    static uint32_t lastReportMs = 0;
    uint32_t nowMs = millis();
    if (nowMs - lastReportMs >= 3000u) {
      lastReportMs = nowMs;
      Serial.printf("[Audio] DSP peak=%lu us, block budget=%lu us\n",
                    (unsigned long)gRenderPeakUs.load(std::memory_order_relaxed),
                    (unsigned long)(CHUNK_SAMPLES * 1000000u / SAMPLE_RATE));
    }
  }
  if (gSampleStorageReady) return;

  uint32_t nowMs = millis();
  if (gSampleStorageRetryMs != 0 &&
      nowMs - gSampleStorageRetryMs < 15000u) {
    return;
  }

  gSampleStorageRetryMs = nowMs;
  ensureSampleStorage();
}

// Queue a note-on for the audio task.
void noteOn(uint8_t noteIndex) {
  queueAudioEvent(AUDIO_EVENT_NOTE_ON, noteIndex);
}

// Queue a note-off for the audio task.
void noteOff(uint8_t noteIndex) {
  queueAudioEvent(AUDIO_EVENT_NOTE_OFF, noteIndex);
}

bool initSamplerStorage() {
  return ensureSampleStorage();
}

bool toggleSamplerMode() {
  bool target = !gSamplerModeRequested;
  if (target && !loadAllSamplesFromSd()) {
    Serial.println("[Sampler] staying in synth mode");
    gSamplerModeRequested = false;
    queueSamplerModeEvent(false);
    return false;
  }

  if (!queueSamplerModeEvent(target)) {
    return gSamplerModeRequested;
  }

  gSamplerModeRequested = target;
  Serial.printf("[Sampler] requested mode %s\n", target ? "sampler" : "synth");
  return target;
}

bool isSamplerMode() {
  return gSamplerModeRequested;
}

// Convert pad index to the MIDI note defined by PANORYTHE_SCALE plus transpose.
uint8_t midiNoteFromNoteIndex(uint8_t noteIndex) {
  int note = gScaleBaseMidiNote + scaleStepForPad(noteIndex) + gScaleOctaveOffset * 12;
  if (note < 0) note = 0;
  if (note > 127) note = 127;
  return (uint8_t)note;
}

// Set master volume for future blocks.
void setVolume(float vol) {
  gVolume = clamp01(vol);
}

// Select the engine used by subsequent note-ons.
void setOscillatorEngine(OscillatorEngine engine) {
  if (engine >= OSC_ENGINE_COUNT) engine = OSC_ENGINE_OMNICHORD;
  gEngine = engine;
}

// Return the current engine selection.
OscillatorEngine getOscillatorEngine() {
  return gEngine;
}

// All enum values are allowed because the firmware only exposes four engines.
bool computeAllowsOscillatorEngine(OscillatorEngine engine) {
  return engine < OSC_ENGINE_COUNT;
}

// Select the active space effect.
void setSpaceFxMode(SpaceFxMode mode) {
  if (mode >= SPACE_FX_COUNT) mode = SPACE_FX_OFF;
  gSpaceFxModeRequested.store(mode, std::memory_order_relaxed);
}

// Return the active space effect.
SpaceFxMode getSpaceFxMode() {
  return gSpaceFxModeRequested.load(std::memory_order_relaxed);
}

// Set warm reverb depth.
void setReverbAmount(float amount) {
  gReverbAmountRequested.store(clamp01(amount), std::memory_order_relaxed);
}

// Return warm reverb depth.
float getReverbAmount() {
  return gReverbAmountRequested.load(std::memory_order_relaxed);
}

// Fade all effects to dry; keep their settings until the audio task finishes.
void clearAllEffects() {
  setSpaceFxMode(SPACE_FX_OFF);
}

// Move to the next distinct pitch class, preserving mode and base octave.
void nudgeScale(int direction) {
  if (!direction) return;
  gScaleRootIndex = (gScaleRootIndex + (direction > 0 ? 1 : SCALE_ROOT_COUNT - 1)) % SCALE_ROOT_COUNT;
  int octave = gScaleBaseMidiNote / 12 - 1;
  gScaleBaseMidiNote = (octave + 1) * 12 + SCALE_ROOTS[gScaleRootIndex].semitone;
  Serial.printf("[Audio] scale %s%s%d\n", gScaleMinor ? "min" : "maj",
                SCALE_ROOTS[gScaleRootIndex].name, octave);
}

// Shift the pad map by whole octaves.
void setScaleOctaveOffset(int offset) {
  if (offset < -2) offset = -2;
  if (offset > 2) offset = 2;
  gScaleOctaveOffset = offset;
}

// Return the current octave offset.
int getScaleOctaveOffset() {
  return gScaleOctaveOffset;
}

} // namespace Audio
