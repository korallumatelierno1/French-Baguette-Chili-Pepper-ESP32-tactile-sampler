#include "Input.h"

#include <Wire.h>
#include <Adafruit_MPR121.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "Config.h"
#include "AudioEngine.h"
#include "UI.h"
#include "UsbMidi.h"
#include "Logger.h"

#define Serial LogSerial

namespace Input {
  // Raw MPR121 thresholds. Dynamic filtering below adds extra protection.
  const uint8_t  TOUCH_THRESH   = 7;
  const uint8_t  RELEASE_THRESH = 4;
  const float    DYN_PCT        = 0.006f;
  const uint16_t DEBOUNCE_MS    = 4;

  static const float PAD_SENS_MIN = 0.005f;
  static const float PAD_SENS_MAX = 120.0f;
  static float gPadSensitivityNorm = 0.85f;

  // Convert the normalized sensitivity control to a baseline-relative threshold.
  static inline float padSensitivityScale() {
    float inv = 1.0f - gPadSensitivityNorm;
    return PAD_SENS_MIN + inv * (PAD_SENS_MAX - PAD_SENS_MIN);
  }

  // Physical pad layout: ten note pads plus six control pads.
  static const uint8_t ELECTRODES_PER_MPR = 12;
  static const uint8_t TOTAL_ELECTRODES   = 2 * ELECTRODES_PER_MPR;
  static const uint8_t NOTE_PAD_COUNT     = 10;
  static const uint8_t NOTE_PADS[NOTE_PAD_COUNT] = {
    15, 14, 13, 12, 11, 4, 3, 2, 1, 0
  };
  static const uint32_t PHYSICAL_NOTE_PAD_MASK =
    (1u << 15) |
    (1u << 14) |
    (1u << 13) |
    (1u << 12) |
    (1u << 11) |
    (1u << 4) |
    (1u << 3) |
    (1u << 2) |
    (1u << 1) |
    (1u << 0);

  static const uint8_t PAD_FX             = 6;
  static const uint8_t PAD_ENGINE         = 8;
  static const uint8_t PAD_VOLUME_DOWN    = 10;
  static const uint8_t PAD_VOLUME_UP      = 9;
  static const uint8_t PAD_OCTAVE_DOWN    = 16;
  static const uint8_t PAD_OCTAVE_UP      = 7;
  static const uint32_t NOTE_PAD_MASK     = (1u << NOTE_PAD_COUNT) - 1u;
  static const uint32_t FX_PAD_BIT        = (1u << PAD_FX);
  static const uint32_t ENGINE_PAD_BIT    = (1u << PAD_ENGINE);
  static const uint32_t OCTAVE_DOWN_PAD_BIT = (1u << PAD_OCTAVE_DOWN);
  static const uint32_t OCTAVE_UP_PAD_BIT   = (1u << PAD_OCTAVE_UP);
  static const uint32_t OCTAVE_PAD_MASK   =
    OCTAVE_DOWN_PAD_BIT | OCTAVE_UP_PAD_BIT;
  static const uint32_t CONTROL_PAD_MASK  =
    FX_PAD_BIT |
    ENGINE_PAD_BIT |
    (1u << PAD_VOLUME_DOWN) |
    (1u << PAD_VOLUME_UP) |
    OCTAVE_PAD_MASK;
  // Sampler slots 0..5 match AudioEngine's named folders:
  // pigeon, Eiffel, cloud, flower, bottle, cheese.
  static const uint8_t SAMPLER_NOTE_PADS[NOTE_PAD_COUNT] = {
    8, 16, 10, 9, 7, 5, 15, 14, 13, 12
  };
  static const uint32_t SAMPLER_NOTE_PAD_MASK =
    (1u << 8) |
    (1u << 16) |
    (1u << 10) |
    (1u << 9) |
    (1u << 7) |
    (1u << 5) |
    (1u << 15) |
    (1u << 14) |
    (1u << 13) |
    (1u << 12);
  static const uint32_t ACTIVE_PAD_MASK   =
    PHYSICAL_NOTE_PAD_MASK | CONTROL_PAD_MASK | SAMPLER_NOTE_PAD_MASK;
  static const uint8_t  NO_CONTROL_PAD    = 255;
  static const uint16_t SAMPLE_MODE_COMBO_WINDOW_MS = 90;

  // Internal actions assigned to each physical pad.
  enum PadAction : uint8_t {
    ACTION_NONE = 0,
    ACTION_NOTE,
    ACTION_FX_CYCLE,
    ACTION_ENGINE,
    ACTION_VOLUME_DOWN,
    ACTION_VOLUME_UP,
    ACTION_OCTAVE_DOWN,
    ACTION_OCTAVE_UP
  };

  // Engine list shown by the engine pad. Only these four engines exist.
  struct EngineItem {
    Audio::OscillatorEngine engine;
    const char* label;
  };

  static const EngineItem HEADLESS_ENGINES[] = {
    {Audio::OSC_ENGINE_STEEL_DRUM,  "Steel Drum"},
    {Audio::OSC_ENGINE_VAPOR_FLUTE, "Vapor Flute"},
    {Audio::OSC_ENGINE_ACCORDION,   "Accordion"},
    {Audio::OSC_ENGINE_OMNICHORD,   "Omnichord"}
  };
  static const int HEADLESS_ENGINE_COUNT =
    (int)(sizeof(HEADLESS_ENGINES) / sizeof(HEADLESS_ENGINES[0]));

  // Two touch controllers are used to cover all note and control pads.
  static Adafruit_MPR121 cap1;
  static Adafruit_MPR121 cap2;
  static bool hasCap1 = false;
  static bool hasCap2 = false;
  static volatile bool gMprIrq1 = false;
  static volatile bool gMprIrq2 = false;
  static TaskHandle_t gInputWorkerTask = nullptr;

  // Previous touch state and debounce tracking.
  static uint32_t prevTouch = 0;
  static uint32_t prevTouchRaw = 0;
  static unsigned long lastTrigPerPad[TOTAL_ELECTRODES] = {0};
  static bool gAnyTouch = false;
  static uint32_t gIrqFallbackLastMs = 0;
  static uint32_t gIrqMissLogMs = 0;

  static uint8_t  gFxStage = 0; // 0: off, 1: reverb, 2: drive, 3: echoloop.
  static int      gCurrentEngineIndex = 3;

  // Control pads are filtered so one strong control press does not trigger neighbors.
  static bool     gControlGroupActive = false;
  static uint8_t  gDominantControlPad = NO_CONTROL_PAD;
  static bool     gEnginePadPressed = false;
  static bool     gSampleModeComboHeld = false;
  static bool     gPendingOctaveActive = false;
  static uint8_t  gPendingOctavePad = NO_CONTROL_PAD;
  static PadAction gPendingOctaveAction = ACTION_NONE;
  static uint32_t gPendingOctaveMs = 0;

  #ifndef MPR_ADDR2
  #define MPR_ADDR2 0x5B
  #endif

  // Convert a physical note pad into its musical rank in NOTE_PADS.
  static inline bool mapNotePadIndex(uint8_t globalIndex, uint8_t &noteIndex) {
    for (uint8_t i = 0; i < NOTE_PAD_COUNT; ++i) {
      if (NOTE_PADS[i] == globalIndex) {
        noteIndex = i;
        return true;
      }
    }
    return false;
  }

  static inline bool mapSamplerPadIndex(uint8_t globalIndex, uint8_t &noteIndex) {
    for (uint8_t i = 0; i < NOTE_PAD_COUNT; ++i) {
      if (SAMPLER_NOTE_PADS[i] == globalIndex) {
        noteIndex = i;
        return true;
      }
    }
    return false;
  }

  // Convert a physical electrode index into a note index or a control action.
  static inline PadAction mapElectrodeAction(uint8_t globalIndex, uint8_t &noteIndex) {
    if (Audio::isSamplerMode() && mapSamplerPadIndex(globalIndex, noteIndex)) {
      return ACTION_NOTE;
    }

    switch (globalIndex) {
      case PAD_FX:          return ACTION_FX_CYCLE;
      case PAD_ENGINE:      return ACTION_ENGINE;
      case PAD_VOLUME_DOWN: return ACTION_VOLUME_DOWN;
      case PAD_VOLUME_UP:   return ACTION_VOLUME_UP;
      case PAD_OCTAVE_DOWN: return ACTION_OCTAVE_DOWN;
      case PAD_OCTAVE_UP:   return ACTION_OCTAVE_UP;
      default:              break;
    }
    if (mapNotePadIndex(globalIndex, noteIndex)) return ACTION_NOTE;
    return ACTION_NONE;
  }

  // Every mapped pad must pass the dynamic sensitivity filter.
  static inline bool actionNeedsPadSenseFilter(PadAction action) {
    return action != ACTION_NONE;
  }

  // Control pads are processed separately from note pads.
  static inline bool isControlAction(PadAction action) {
    return action == ACTION_FX_CYCLE ||
           action == ACTION_ENGINE ||
           action == ACTION_VOLUME_DOWN ||
           action == ACTION_VOLUME_UP ||
           action == ACTION_OCTAVE_DOWN ||
           action == ACTION_OCTAVE_UP;
  }

  static uint32_t currentControlTouchMask(uint32_t touchMask) {
    uint32_t controlMask = 0;
    for (uint8_t i = 0; i < TOTAL_ELECTRODES; ++i) {
      uint32_t bit = (1u << i);
      if (!(touchMask & bit)) continue;

      uint8_t noteIndex = 0;
      if (isControlAction(mapElectrodeAction(i, noteIndex))) {
        controlMask |= bit;
      }
    }
    return controlMask;
  }

  // The first MPR121 covers electrodes 0..11; the second covers 12..23.
  static inline bool isOnCap1(uint8_t globalIndex) {
    return globalIndex < ELECTRODES_PER_MPR;
  }

  // Convert a global electrode index to the local index used by each MPR121.
  static inline uint8_t localIndex(uint8_t globalIndex) {
    return (uint8_t)(globalIndex % ELECTRODES_PER_MPR);
  }

  // Read baseline and filtered values for dynamic threshold decisions.
  static bool readBaselineAndFilteredDyn(uint8_t globalIndex,
                                         uint16_t &b, uint16_t &f) {
    uint8_t loc = localIndex(globalIndex);
    if (isOnCap1(globalIndex)) {
      if (!hasCap1) return false;
      b = cap1.baselineData(loc);
      f = cap1.filteredData(loc);
      return true;
    }
    if (!hasCap2) return false;
    b = cap2.baselineData(loc);
    f = cap2.filteredData(loc);
    return true;
  }

  // Pick the strongest touched control pad when several controls are touched.
  static uint8_t strongestControlPad(uint32_t controlMask,
                                     const int16_t controlDelta[]) {
    uint8_t bestPad = NO_CONTROL_PAD;
    int16_t bestDelta = -1;
    for (uint8_t i = 0; i < TOTAL_ELECTRODES; ++i) {
      uint32_t bit = (1u << i);
      if (!(controlMask & bit)) continue;

      uint8_t noteIndex = 0;
      PadAction action = mapElectrodeAction(i, noteIndex);
      if (!isControlAction(action)) continue;

      int16_t d = controlDelta[i];
      if (d > bestDelta) {
        bestDelta = d;
        bestPad = i;
      }
    }
    return bestPad;
  }

  // Keep only the dominant control pad when several controls are touched.
  static uint32_t filterControlPadsToDominant(uint32_t touchMask,
                                              const int16_t controlDelta[]) {
    uint32_t controlsNow = currentControlTouchMask(touchMask);
    if (!controlsNow) {
      gControlGroupActive = false;
      gDominantControlPad = NO_CONTROL_PAD;
      return touchMask;
    }

    if (!gControlGroupActive) {
      gDominantControlPad = strongestControlPad(controlsNow, controlDelta);
      gControlGroupActive = true;
    }

    uint32_t keep = 0;
    if (gDominantControlPad != NO_CONTROL_PAD) {
      uint32_t dominantBit = (1u << gDominantControlPad);
      if (controlsNow & dominantBit) {
        keep = dominantBit;
      }
    }

    return (touchMask & ~controlsNow) | keep;
  }

  // Apply stable MPR121 settings after each controller is detected.
  static void configureCap(Adafruit_MPR121 &cap) {
    cap.writeRegister(MPR121_ECR, 0x00);
    cap.writeRegister(MPR121_SOFTRESET, 0x63);
    delay(5);
    cap.writeRegister(MPR121_DEBOUNCE, 0x22);
    cap.writeRegister(MPR121_CONFIG1, 0x10);
    cap.writeRegister(MPR121_CONFIG2, 0x22);
    cap.setThresholds(TOUCH_THRESH, RELEASE_THRESH);
    cap.writeRegister(MPR121_ECR, 0x8F);
    delay(50);
  }

  // Interrupt handlers wake the input task without doing I2C work in the ISR.
  static void IRAM_ATTR onMpr1Irq() {
    gMprIrq1 = true;
    BaseType_t higherPriorityTaskWoken = pdFALSE;
    if (gInputWorkerTask) {
      vTaskNotifyGiveFromISR(gInputWorkerTask, &higherPriorityTaskWoken);
    }
    if (higherPriorityTaskWoken == pdTRUE) {
      portYIELD_FROM_ISR();
    }
  }

  // Interrupt handler for the second MPR121.
  static void IRAM_ATTR onMpr2Irq() {
    gMprIrq2 = true;
    BaseType_t higherPriorityTaskWoken = pdFALSE;
    if (gInputWorkerTask) {
      vTaskNotifyGiveFromISR(gInputWorkerTask, &higherPriorityTaskWoken);
    }
    if (higherPriorityTaskWoken == pdTRUE) {
      portYIELD_FROM_ISR();
    }
  }

  // Find the index of an engine in the physical engine cycle list.
  static int findEngineIndex(Audio::OscillatorEngine engine) {
    for (int i = 0; i < HEADLESS_ENGINE_COUNT; ++i) {
      if (HEADLESS_ENGINES[i].engine == engine) return i;
    }
    return -1;
  }

  // Apply one of the headless FX states.
  static void applyHeadlessFxStage(uint8_t stage) {
    Audio::clearAllEffects();
    switch (stage) {
      case 1:
        Audio::setSpaceFxMode(Audio::SPACE_FX_WARM_REVERB);
        Audio::setReverbAmount(0.78f);
        UI::showToast("FX: Reverb");
        break;
      case 2:
        Audio::setSpaceFxMode(Audio::SPACE_FX_DRIVE);
        UI::showToast("FX: drive");
        break;
      case 3:
        Audio::setSpaceFxMode(Audio::SPACE_FX_ECHOLOOP);
        UI::showToast("FX: echoloop");
        break;
      default:
        UI::showToast("FX: Off");
        break;
    }
  }

  // Cycle through off, reverb, drive, and echoloop.
  static void cycleFxPad() {
    gFxStage = (uint8_t)((gFxStage + 1u) % 4u);
    applyHeadlessFxStage(gFxStage);
  }

  // Cycle to the next allowed engine.
  static void cycleEnginePad() {
    int current = findEngineIndex(Audio::getOscillatorEngine());
    if (current >= 0) gCurrentEngineIndex = current;

    for (int step = 1; step <= HEADLESS_ENGINE_COUNT; ++step) {
      int idx = (gCurrentEngineIndex + step) % HEADLESS_ENGINE_COUNT;
      Audio::OscillatorEngine candidate = HEADLESS_ENGINES[idx].engine;
      if (!Audio::computeAllowsOscillatorEngine(candidate)) continue;

      gCurrentEngineIndex = idx;
      Audio::setOscillatorEngine(candidate);
      Serial.printf("[Headless Input] engine: %s\n", HEADLESS_ENGINES[idx].label);
      return;
    }
  }

  // Execute a debounced control action.
  static void handleControlPress(uint8_t pad, PadAction action, uint32_t nowMs) {
    if (nowMs - lastTrigPerPad[pad] <= DEBOUNCE_MS) return;
    lastTrigPerPad[pad] = nowMs;

    switch (action) {
      case ACTION_FX_CYCLE:
        cycleFxPad();
        break;
      case ACTION_VOLUME_DOWN:
        UI::nudgeVolume(-0.05f);
        break;
      case ACTION_VOLUME_UP:
        UI::nudgeVolume(0.05f);
        break;
      case ACTION_OCTAVE_DOWN:
        UI::nudgeOctave(-1);
        break;
      case ACTION_OCTAVE_UP:
        UI::nudgeOctave(1);
        break;
      default:
        break;
    }
  }

  static inline bool isOctaveAction(PadAction action) {
    return action == ACTION_OCTAVE_DOWN || action == ACTION_OCTAVE_UP;
  }

  static void clearPendingOctave() {
    gPendingOctaveActive = false;
    gPendingOctavePad = NO_CONTROL_PAD;
    gPendingOctaveAction = ACTION_NONE;
    gPendingOctaveMs = 0;
  }

  static void schedulePendingOctave(uint8_t pad, PadAction action, uint32_t nowMs) {
    if (!isOctaveAction(action) || gSampleModeComboHeld) return;
    gPendingOctaveActive = true;
    gPendingOctavePad = pad;
    gPendingOctaveAction = action;
    gPendingOctaveMs = nowMs;
  }

  static void servicePendingOctave(uint32_t nowMs) {
    if (!gPendingOctaveActive || gSampleModeComboHeld) return;
    if (nowMs - gPendingOctaveMs < SAMPLE_MODE_COMBO_WINDOW_MS) return;

    handleControlPress(gPendingOctavePad, gPendingOctaveAction, nowMs);
    clearPendingOctave();
  }

  static uint32_t processSampleModeCombo(uint32_t touchMask) {
    bool octaveUp = (touchMask & OCTAVE_UP_PAD_BIT) != 0;
    bool octaveDown = (touchMask & OCTAVE_DOWN_PAD_BIT) != 0;

    if (gSampleModeComboHeld) {
      if (!octaveUp && !octaveDown) {
        gSampleModeComboHeld = false;
      }
      return touchMask & ~OCTAVE_PAD_MASK;
    }

    if (octaveUp && octaveDown) {
      clearPendingOctave();
      gSampleModeComboHeld = true;
      bool samplerOn = Audio::toggleSamplerMode();
      UI::showToast(samplerOn ? "Mode: Sampler" : "Mode: Synth");
      return touchMask & ~OCTAVE_PAD_MASK;
    }

    return touchMask;
  }

  // Initialize touch hardware and reset all input state.
  void init() {
    Wire.begin(SDA_PIN, SCL_PIN, 400000);

    #ifdef SDA2_PIN
    #ifdef SCL2_PIN
    Wire1.begin(SDA2_PIN, SCL2_PIN, 400000);
    #endif
    #endif

    Serial.println("[Input] init headless MPR121");

    hasCap1 = cap1.begin(MPR_ADDR, &Wire);
    if (!hasCap1) {
      Serial.println("[Input] MPR121 #1 not detected");
    } else {
      configureCap(cap1);
      Serial.println("[Input] MPR121 #1 ready, electrodes E0..E11");
      if (MPR1_IRQ_PIN >= 0) {
        pinMode(MPR1_IRQ_PIN, INPUT_PULLUP);
        attachInterrupt(digitalPinToInterrupt(MPR1_IRQ_PIN), onMpr1Irq, FALLING);
      }
    }

    #ifdef SDA2_PIN
    #ifdef SCL2_PIN
    hasCap2 = cap2.begin(MPR_ADDR2, &Wire1);
    if (!hasCap2) {
      Serial.println("[Input] MPR121 #2 not detected");
    } else {
      configureCap(cap2);
      Serial.println("[Input] MPR121 #2 ready, electrodes E12..E23");
      if (MPR2_IRQ_PIN >= 0) {
        pinMode(MPR2_IRQ_PIN, INPUT_PULLUP);
        attachInterrupt(digitalPinToInterrupt(MPR2_IRQ_PIN), onMpr2Irq, FALLING);
      }
    }
    #else
    hasCap2 = false;
    #endif
    #else
    hasCap2 = false;
    #endif

    if (!hasCap1 && !hasCap2) {
      Serial.println("[Input] no MPR121 detected, touch disabled");
      return;
    }

    for (uint8_t i = 0; i < TOTAL_ELECTRODES; ++i) {
      lastTrigPerPad[i] = 0;
    }

    prevTouch = 0;
    prevTouchRaw = 0;
    gAnyTouch = false;
    gControlGroupActive = false;
    gDominantControlPad = NO_CONTROL_PAD;
    gEnginePadPressed = false;
    gSampleModeComboHeld = false;
    clearPendingOctave();
  }

  // Poll the touch controllers and return note-edge events.
  TouchEvents pollTouch() {
    TouchEvents ev{0, 0};
    if (!hasCap1 && !hasCap2) return ev;

    uint32_t nowMs = millis();

    bool irq1 = false;
    bool irq2 = false;
    noInterrupts();
    irq1 = gMprIrq1;
    irq2 = gMprIrq2;
    gMprIrq1 = false;
    gMprIrq2 = false;
    interrupts();

    // If no interrupt arrives while idle, poll occasionally to recover missed IRQs.
    bool fallbackPoll = false;
    if (!irq1 && !irq2 && prevTouchRaw == 0) {
      if ((nowMs - gIrqFallbackLastMs) >= 40u) {
        fallbackPoll = true;
        gIrqFallbackLastMs = nowMs;
      }
    }

    bool needRead = irq1 || irq2 || (prevTouchRaw != 0) || fallbackPoll;
    uint32_t tRaw = prevTouchRaw;
    if (needRead) {
      uint16_t t1 = hasCap1 ? cap1.touched() : 0;
      uint16_t t2 = hasCap2 ? cap2.touched() : 0;
      tRaw = (uint32_t)t1 | ((uint32_t)t2 << ELECTRODES_PER_MPR);
      if (fallbackPoll && tRaw != 0 && (nowMs - gIrqMissLogMs) > 1000u) {
        Serial.println("[Input] IRQ fallback saw touch");
        gIrqMissLogMs = nowMs;
      }
    }

    // Filter out unused electrodes and weak touches before action handling.
    tRaw &= ACTIVE_PAD_MASK;
    uint32_t tFiltered = tRaw;
    int16_t controlDelta[TOTAL_ELECTRODES] = {0};
    for (uint8_t i = 0; i < TOTAL_ELECTRODES; ++i) {
      uint32_t bit = (1u << i);
      if (!(tRaw & bit)) continue;

      uint8_t noteIndex = 0;
      PadAction action = mapElectrodeAction(i, noteIndex);
      if (!actionNeedsPadSenseFilter(action)) {
        tFiltered &= ~bit;
        continue;
      }
      if (action == ACTION_NOTE && (prevTouch & bit)) continue;

      uint16_t b = 0;
      uint16_t f = 0;
      if (!readBaselineAndFilteredDyn(i, b, f) || !b) {
        tFiltered &= ~bit;
        continue;
      }
      int16_t d = (int16_t)b - (int16_t)f;
      if (d < 0) d = 0;
      controlDelta[i] = d;
      float dynScaled = (float)b * DYN_PCT * padSensitivityScale();
      int16_t dyn = (int16_t)((dynScaled > (float)TOUCH_THRESH) ? dynScaled : (float)TOUCH_THRESH);
      if (d < dyn) {
        tFiltered &= ~bit;
      }
    }

    // Resolve the two-pad sampler combo before control-pad ambiguity filtering.
    tFiltered = processSampleModeCombo(tFiltered);

    // Resolve control-pad ambiguity before computing edges.
    uint32_t t = filterControlPadsToDominant(tFiltered, controlDelta);
    uint32_t rising = t & ~prevTouch;
    uint32_t falling = (~t) & prevTouch;
    gAnyTouch = (tRaw != 0);

    unsigned long now = millis();

    // Rising edges trigger notes or queue control actions.
    for (uint8_t i = 0; i < TOTAL_ELECTRODES; ++i) {
      uint32_t bit = (1u << i);
      if (!(rising & bit)) continue;

      uint8_t noteIndex = 0;
      PadAction action = mapElectrodeAction(i, noteIndex);
      if (action == ACTION_NOTE) {
        uint16_t b = 0;
        uint16_t f = 0;
        if (!readBaselineAndFilteredDyn(i, b, f) || !b) continue;

        int16_t d = (int16_t)b - (int16_t)f;
        float dynScaled = (float)b * DYN_PCT * padSensitivityScale();
        int16_t dyn = (int16_t)((dynScaled > (float)TOUCH_THRESH) ? dynScaled : (float)TOUCH_THRESH);
        if (d >= dyn && (now - lastTrigPerPad[i] > DEBOUNCE_MS)) {
          ev.rising |= (1u << noteIndex);
          lastTrigPerPad[i] = now;
        }
      } else if (action == ACTION_ENGINE) {
        if (nowMs - lastTrigPerPad[i] > DEBOUNCE_MS) {
          gEnginePadPressed = true;
          lastTrigPerPad[i] = nowMs;
        }
      } else if (isOctaveAction(action)) {
        schedulePendingOctave(i, action, nowMs);
      } else {
        handleControlPress(i, action, nowMs);
      }
    }

    if (falling & ENGINE_PAD_BIT) {
      if (gEnginePadPressed) {
        cycleEnginePad();
      }
      gEnginePadPressed = false;
    }

    // Falling note edges release voices.
    for (uint8_t i = 0; i < TOTAL_ELECTRODES; ++i) {
      uint32_t bit = (1u << i);
      if (!(falling & bit)) continue;

      uint8_t noteIndex = 0;
      PadAction action = mapElectrodeAction(i, noteIndex);
      if (action == ACTION_NOTE) {
        ev.falling |= (1u << noteIndex);
      }
    }

    servicePendingOctave(nowMs);

    prevTouch = t;
    prevTouchRaw = tRaw;

    return ev;
  }

  // The input task calls this once so ISRs can wake it directly.
  void attachCurrentTaskAsWorker() {
    gInputWorkerTask = xTaskGetCurrentTaskHandle();
    ulTaskNotifyTake(pdTRUE, 0);
  }

  // Sleep until an interrupt arrives, with a timeout for fallback polling.
  bool waitForTouchSignal(uint32_t timeoutMs) {
    TickType_t timeoutTicks = (timeoutMs == 0) ? 0 : pdMS_TO_TICKS(timeoutMs);
    return ulTaskNotifyTake(pdTRUE, timeoutTicks) > 0;
  }

  // Forward note edges to both USB MIDI and the internal audio engine.
  void dispatchTouchEvents(const TouchEvents& ev) {
    if (ev.rising) {
      uint32_t mask = ev.rising & NOTE_PAD_MASK;
      for (uint8_t idx = 0; idx < NOTE_PAD_COUNT; ++idx) {
        uint32_t bit = (1u << idx);
        if (mask & bit) {
          UsbMidi::noteOn(idx);
          Audio::noteOn(idx);
        }
      }
    }

    if (ev.falling) {
      uint32_t mask = ev.falling & NOTE_PAD_MASK;
      for (uint8_t idx = 0; idx < NOTE_PAD_COUNT; ++idx) {
        uint32_t bit = (1u << idx);
        if (mask & bit) {
          UsbMidi::noteOff(idx);
          Audio::noteOff(idx);
        }
      }
    }
  }

  // Used by the input task to choose a short or long wait interval.
  bool isAnyPadActive() {
    return gAnyTouch;
  }
}
