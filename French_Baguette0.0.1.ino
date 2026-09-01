#include <Arduino.h>
#include "sdkconfig.h"
#include "soc/soc_caps.h"

#include "AudioEngine.h"
#include "Config.h"
#include "Input.h"
#include "Logger.h"
#include "UsbMidi.h"
#include "UI.h"

#define Serial LogSerial

TaskHandle_t audioTaskHandle = nullptr;
TaskHandle_t inputTaskHandle = nullptr;

// Highest-priority task: render audio continuously and let i2s_write pace it.
void audioTask(void *param) {
  (void)param;
  for (;;) {
    Audio::update();
  }
}

// Touch task: wakes from MPR121 interrupts, with fallback polling while idle.
void inputTask(void *param) {
  (void)param;
  Input::attachCurrentTaskAsWorker();
  for (;;) {
    uint32_t waitMs = Input::isAnyPadActive() ? 1u : 40u;
    Input::waitForTouchSignal(waitMs);
    Input::TouchEvents ev = Input::pollTouch();
    Input::dispatchTouchEvents(ev);
  }
}

// Bring up serial, MIDI, touch, audio, UI, then start the real-time tasks.
void setup() {
  Serial.begin(115200);
  delay(300);

  Serial.println("[Boot] Panorythe headless");
  UsbMidi::init();
  Audio::initSamplerStorage();
  Input::init();
  Audio::init();
  UI::init();

  xTaskCreatePinnedToCore(
    audioTask,
    "AudioTask",
    4096,
    nullptr,
    3,
    &audioTaskHandle,
    0
  );

  xTaskCreatePinnedToCore(
    inputTask,
    "InputTask",
    4096,
    nullptr,
    2,
    &inputTaskHandle,
    1
  );

  Serial.println("[Boot] ready");
}

// Low-priority loop: service non-real-time hooks and USB MIDI.
void loop() {
  static uint32_t lastLoopUs = 0;
  uint32_t nowUs = micros();
  if (lastLoopUs != 0) {
    UI::setLoopPeriodUs(nowUs - lastLoopUs);
  }
  lastLoopUs = nowUs;

  uint32_t audioStartUs = micros();
  Audio::service();
  uint32_t audioUs = micros() - audioStartUs;

  UsbMidi::service();
  UI::setLoopTimingUs(0, 0, audioUs, 0, micros() - nowUs);
  vTaskDelay(1);
}
