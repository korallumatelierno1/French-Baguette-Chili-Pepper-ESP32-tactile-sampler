#include "AudioEngine.h"

#include <Arduino.h>
#include <FS.h>
#include <SD.h>
#include <SPI.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "Config.h"
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
constexpr uint8_t MAX_VOICES = 6;
constexpr uint8_t MAX_SAMPLE_VOICES = 10;
constexpr uint8_t SAMPLE_PAD_COUNT = 10;
constexpr uint8_t MAX_NOTE_INDEX = 24;
constexpr uint8_t AUDIO_EVENT_QUEUE_LEN = 48;
constexpr size_t SINE_TABLE_SIZE = 1024;
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

static const float WORMY_FORMANTS[5][3] = {
  {740.0f, 1180.0f, 2550.0f},
  {500.0f, 1860.0f, 2500.0f},
  {310.0f, 2220.0f, 2920.0f},
  {510.0f,  880.0f, 2380.0f},
  {360.0f,  720.0f, 2180.0f}
};

static const float WORMY_FORMANT_GAINS[3] = {0.85f, 0.58f, 0.32f};

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
  uint8_t vowelIndex;
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

// Global audio state. New notes use gEngine; existing voices keep their engine.
static Voice gVoices[MAX_VOICES];
static SampleVoice gSampleVoices[MAX_SAMPLE_VOICES];
static SampleSlot gSampleSlots[SAMPLE_PAD_COUNT];
static uint32_t gVoiceSeq = 1;
static float gVolume = 0.70f;
static OscillatorEngine gEngine = OSC_ENGINE_LIGHT;
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
static SpaceFxMode gSpaceFxMode = SPACE_FX_OFF;
static float gReverbAmount = 0.0f;
static bool gI2sReady = false;
static int16_t gOut[CHUNK_SAMPLES * 2];

static float gReverbA[2048];
static float gReverbB[3072];
static uint16_t gReverbIndexA = 0;
static uint16_t gReverbIndexB = 0;
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

// Cheap triangle wave used to add edge to the Light engine.
static inline float triangle01(float phase) {
  return (phase < 0.5f) ? (phase * 4.0f - 1.0f) : (3.0f - phase * 4.0f);
}

// Small deterministic noise generator used by Aurora Light and Wormy.
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

// Engine-specific attack times keep plucked sounds quick and vocal sounds softer.
static float attackSecondsFor(OscillatorEngine engine) {
  switch (engine) {
    case OSC_ENGINE_WORMY: return 0.030f;
    case OSC_ENGINE_AURORA_LIGHT: return 0.004f;
    case OSC_ENGINE_ORGAN: return 0.012f;
    case OSC_ENGINE_LIGHT:
    default: return 0.014f;
  }
}

// Engine-specific release times define each sound tail.
static float releaseSecondsFor(OscillatorEngine engine) {
  switch (engine) {
    case OSC_ENGINE_WORMY: return 0.34f;
    case OSC_ENGINE_AURORA_LIGHT: return 1.45f;
    case OSC_ENGINE_ORGAN: return 0.26f;
    case OSC_ENGINE_LIGHT:
    default: return 0.42f;
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
  }
}

static void clearAllSampleVoices() {
  for (uint8_t i = 0; i < MAX_SAMPLE_VOICES; ++i) {
    clearSampleVoice(gSampleVoices[i]);
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
    if (gVoices[i].envState == ENV_RELEASE && gVoices[i].env < quietestRelease) {
      quietestRelease = gVoices[i].env;
      releaseVoice = i;
    }
    if (gVoices[i].startedAt < oldestSeq) {
      oldestSeq = gVoices[i].startedAt;
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
  v.vowelIndex = (uint8_t)(noteIndex % 5);

  float shift = 0.90f + 0.035f * (float)(v.noteIndex % 7);
  uint8_t vowel = v.vowelIndex;
  if (vowel > 4) vowel = 0;
  for (uint8_t i = 0; i < 3; ++i) {
    float hz = clampf(WORMY_FORMANTS[vowel][i] * shift, 40.0f, 8000.0f);
    v.formantCoeff[i] = 2.0f * sinf(PI_F * hz / (float)SAMPLE_RATE);
    v.formantDamping[i] = 0.10f + 0.025f * (float)i;
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

// Light: smooth dual-oscillator tone with a filtered edge component.
static float renderLight(Voice& v) {
  float inc = v.freq / (float)SAMPLE_RATE;
  float incB = (v.freq * 1.006f) / (float)SAMPLE_RATE;
  v.phaseA = wrap01(v.phaseA + inc);
  v.phaseB = wrap01(v.phaseB + incB);
  float body = 0.58f * sine01(v.phaseA) + 0.28f * sine01(v.phaseB);
  float edge = 0.18f * triangle01(v.phaseA);
  v.lp += (body + edge - v.lp) * 0.055f;
  return v.lp * 0.85f;
}

// Aurora Light: bell-like pluck with decaying harmonics and a small noise strike.
static float renderAuroraLight(Voice& v) {
  float inc = v.freq / (float)SAMPLE_RATE;
  v.phaseA = wrap01(v.phaseA + inc);
  v.phaseB = wrap01(v.phaseB + inc * 2.006f);
  v.phaseC = wrap01(v.phaseC + inc * 3.014f);
  v.pluckEnv *= 0.99935f;
  v.noiseEnv *= 0.9965f;
  float bell = sine01(v.phaseA) * 0.52f;
  bell += sine01(v.phaseB) * (0.38f * v.pluckEnv);
  bell += sine01(v.phaseC) * (0.20f * v.pluckEnv);
  bell += whiteNoise(v.rng) * (0.045f * v.noiseEnv);
  return bell * 0.95f;
}

// Organ: additive drawbar-style tone with soft saturation.
static float renderOrgan(Voice& v) {
  float inc = v.freq / (float)SAMPLE_RATE;
  v.phaseA = wrap01(v.phaseA + inc);
  v.phaseB = wrap01(v.phaseB + inc * 2.0f);
  v.phaseC = wrap01(v.phaseC + inc * 3.0f);
  v.phaseD = wrap01(v.phaseD + inc * 4.0f);
  float s = 0.62f * sine01(v.phaseA);
  s += 0.23f * sine01(v.phaseB);
  s += 0.14f * sine01(v.phaseC);
  s += 0.08f * sine01(v.phaseD);
  return fastTanh(s * 1.15f) * 0.82f;
}

// Process one state-variable band-pass formant stage.
static float processFormant(FormantState& st, float input, float coeff, float damping) {
  float high = input - st.low - damping * st.band;
  st.band += coeff * high;
  st.low += coeff * st.band;
  return st.band;
}

// Wormy: compact vocal engine built from one glottal source and three formants.
static float renderWormy(Voice& v) {
  float inc = v.freq / (float)SAMPLE_RATE;
  v.phaseA = wrap01(v.phaseA + inc);
  v.phaseB = wrap01(v.phaseB + inc * 0.50f);
  float glottal = (v.phaseA < 0.42f) ? 0.92f : -0.58f;
  glottal += 0.18f * sine01(v.phaseA);
  glottal += 0.05f * whiteNoise(v.rng);

  float out = 0.0f;
  for (uint8_t i = 0; i < 3; ++i) {
    out += WORMY_FORMANT_GAINS[i] * processFormant(v.formant[i],
                                                   glottal,
                                                   v.formantCoeff[i],
                                                   v.formantDamping[i]);
  }
  out += 0.10f * sine01(v.phaseB);
  return fastTanh(out * 1.7f) * 0.72f;
}

// Render one voice sample after envelope processing.
static float renderVoice(Voice& v) {
  updateEnvelope(v);
  if (!v.active) return 0.0f;

  float s = 0.0f;
  switch (v.engine) {
    case OSC_ENGINE_WORMY:
      s = renderWormy(v);
      break;
    case OSC_ENGINE_AURORA_LIGHT:
      s = renderAuroraLight(v);
      break;
    case OSC_ENGINE_ORGAN:
      s = renderOrgan(v);
      break;
    case OSC_ENGINE_LIGHT:
    default:
      s = renderLight(v);
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

// Warm reverb: two feedback delay lines with modest damping.
static float processReverb(float x) {
  if (gSpaceFxMode != SPACE_FX_WARM_REVERB || gReverbAmount <= 0.001f) {
    return x;
  }

  float amount = clamp01(gReverbAmount);
  float input = fastTanh(finiteOrZero(x) * 0.90f) * 0.85f;
  float a = gReverbA[gReverbIndexA];
  float b = gReverbB[gReverbIndexB];
  float wet = fastTanh(0.58f * a + 0.42f * b);
  float feedback = 0.36f + 0.18f * amount;
  gReverbA[gReverbIndexA] = fastTanh(input + b * feedback);
  gReverbB[gReverbIndexB] = fastTanh(input * 0.70f + a * (feedback * 0.82f));

  gReverbIndexA = (uint16_t)((gReverbIndexA + 1u) % 2048u);
  gReverbIndexB = (uint16_t)((gReverbIndexB + 1u) % 3072u);

  return x * (1.0f - amount * 0.30f) + wet * (amount * 0.50f);
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

  for (size_t i = 0; i < CHUNK_SAMPLES; ++i) {
    float mix = 0.0f;
    if (gSamplerMode) {
      for (uint8_t v = 0; v < MAX_SAMPLE_VOICES; ++v) {
        if (!gSampleVoices[v].active) continue;
        mix += renderSampleVoice(gSampleVoices[v]);
      }
    } else {
      for (uint8_t v = 0; v < MAX_VOICES; ++v) {
        if (!gVoices[v].active) continue;
        mix += renderVoice(gVoices[v]);
      }
    }
    mix *= polyTrim;
    mix = softLimit(mix);
    mix = processReverb(mix);
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
  if (gSamplerMode) {
    startQueuedSample(noteIndex);
    return;
  }
  int slot = allocateVoice();
  configureVoice(gVoices[slot], noteIndex);
}

// Release every active voice that belongs to this pad. Called only by the audio task.
static void releaseQueuedNote(uint8_t noteIndex) {
  if (gSamplerMode) {
    releaseQueuedSample(noteIndex);
    return;
  }
  for (uint8_t i = 0; i < MAX_VOICES; ++i) {
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

  memset(gReverbA, 0, sizeof(gReverbA));
  memset(gReverbB, 0, sizeof(gReverbB));
  gReverbIndexA = 0;
  gReverbIndexB = 0;
  i2sInit();
  Serial.println("[Audio] engine ready");
}

// Audio task entry point: render one block and block until I2S accepts it.
void update() {
  if (!gI2sReady) {
    delay(1);
    return;
  }

  processAudioEvents();
  renderChunk();
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
  if (engine >= OSC_ENGINE_COUNT) engine = OSC_ENGINE_LIGHT;
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
  gSpaceFxMode = mode;
}

// Return the active space effect.
SpaceFxMode getSpaceFxMode() {
  return gSpaceFxMode;
}

// Set warm reverb depth.
void setReverbAmount(float amount) {
  gReverbAmount = clamp01(amount);
}

// Return warm reverb depth.
float getReverbAmount() {
  return gReverbAmount;
}

// Reset all effects to a dry state.
void clearAllEffects() {
  gSpaceFxMode = SPACE_FX_OFF;
  gReverbAmount = 0.0f;
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
