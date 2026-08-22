#include "UsbMidi.h"

#include <Arduino.h>
#include <string.h>
#include "sdkconfig.h"
#include "soc/soc_caps.h"

#include "AudioEngine.h"
#include "Config.h"
#include "Logger.h"

#define Serial LogSerial

#ifndef ARDUINO_USB_MODE
#define ARDUINO_USB_MODE 1
#endif

// USB MIDI is available only when the board is built in USB-OTG TinyUSB mode.
#if SOC_USB_OTG_SUPPORTED && CONFIG_TINYUSB_ENABLED && CONFIG_TINYUSB_MIDI_ENABLED && !ARDUINO_USB_MODE
#include "USB.h"
#include "esp32-hal-tinyusb.h"
#define USB_MIDI_AVAILABLE 1
#else
#define USB_MIDI_AVAILABLE 0
#endif

#ifndef ARDUINO_USB_ON_BOOT
#define ARDUINO_USB_ON_BOOT 0
#endif

namespace UsbMidi {
namespace {
  static const uint8_t USB_MIDI_MAX_NOTES = 24;

  // Runtime state for the USB MIDI endpoint and per-pad note cache.
  bool gActive = false;
  bool gStackStarted = false;
  bool gUnsupportedLogged = false;
  bool gPadNoteActive[USB_MIDI_MAX_NOTES] = {false};
  uint8_t gPadMidiNote[USB_MIDI_MAX_NOTES] = {0};

#if USB_MIDI_AVAILABLE
  bool gDescriptorLoaded = false;
  bool gInterfaceEnabled = false;

  // Build the USB MIDI descriptor that the ESP32 TinyUSB stack requests.
  static uint16_t loadMidiDescriptor(uint8_t *dst, uint8_t *itf) {
    if (gDescriptorLoaded) return 0;
    gDescriptorLoaded = true;

    uint8_t strIndex = tinyusb_add_string_descriptor("USB MIDI");
    uint8_t epIn = tinyusb_get_free_in_endpoint();
    uint8_t epOut = tinyusb_get_free_out_endpoint();
    if (!epIn || !epOut) return 0;

    uint8_t descriptor[TUD_MIDI_DESC_LEN] = {
      TUD_MIDI_DESCRIPTOR(
        *itf,
        strIndex,
        epOut,
        (uint8_t)(0x80 | epIn),
        CFG_TUD_ENDPOINT_SIZE
      )
    };

    *itf += 2;
    memcpy(dst, descriptor, sizeof(descriptor));
    return sizeof(descriptor);
  }

  // Register the MIDI interface before USB.begin() starts enumeration.
  static bool enableMidiInterface() {
    if (gInterfaceEnabled) return true;

    esp_err_t err = tinyusb_enable_interface(
      USB_INTERFACE_MIDI,
      TUD_MIDI_DESC_LEN,
      loadMidiDescriptor
    );
    if (err != ESP_OK) {
      Serial.printf("[USB MIDI] interface registration failed: %d\n", (int)err);
      return false;
    }

    gInterfaceEnabled = true;
    return true;
  }

  // Start the TinyUSB stack and expose the MIDI device.
  static bool beginUsbStack() {
    if (!enableMidiInterface()) return false;

    gStackStarted = USB.begin();
    if (gStackStarted) {
      Serial.println("[USB MIDI] active.");
    } else {
      Serial.println("[USB MIDI] USB.begin() failed.");
    }
    return gStackStarted;
  }

  // Convert the configured MIDI channel to the 0..15 value used in status bytes.
  static uint8_t midiChannelIndex() {
    uint8_t channel = USB_MIDI_CHANNEL;
    if (channel < 1) channel = 1;
    if (channel > 16) channel = 16;
    return (uint8_t)(channel - 1);
  }

  // Write one USB MIDI event packet on virtual cable 0.
  static bool sendMidiPacket(uint8_t codeIndex, uint8_t status, uint8_t data1, uint8_t data2) {
    uint8_t packet[4] = {
      codeIndex,
      status,
      data1,
      data2
    };
    return tud_midi_packet_write(packet);
  }

  // Send a channel voice event on the configured MIDI channel.
  static bool sendChannelPacket(uint8_t codeIndex,
                                uint8_t statusBase,
                                uint8_t data1,
                                uint8_t data2) {
    uint8_t status = (uint8_t)(statusBase | midiChannelIndex());
    return sendMidiPacket(codeIndex, status, data1, data2);
  }
#endif

  // The same analog input is used as a simple USB power detector.
  static float readUsbSenseVoltage() {
    if (PIN_BATTERY_SENSE < 0) return 0.0f;

    pinMode(PIN_BATTERY_SENSE, INPUT);
    analogReadResolution(12);
#if defined(ARDUINO_ARCH_ESP32)
    analogSetPinAttenuation(PIN_BATTERY_SENSE, ADC_11db);
#endif
    delay(2);
    (void)analogRead(PIN_BATTERY_SENSE);

    uint32_t rawSum = 0;
    static const uint8_t readings = 6;
    for (uint8_t i = 0; i < readings; ++i) {
      rawSum += (uint32_t)analogRead(PIN_BATTERY_SENSE);
      delay(1);
    }

    float raw = (float)rawSum / (float)readings;
    float pinVolts = raw * (BATTERY_ADC_FULL_SCALE_VOLTAGE / 4095.0f);
    float divider = (BATTERY_DIVIDER_TOP_OHMS + BATTERY_DIVIDER_BOTTOM_OHMS) /
                    BATTERY_DIVIDER_BOTTOM_OHMS;
    return pinVolts * divider;
  }
}

// Auto-start MIDI only when USB power is present at boot.
void init() {
  float usbSenseVoltage = readUsbSenseVoltage();
#if USB_MIDI_AVAILABLE
  if (usbSenseVoltage > BATTERY_USB_POWER_V) {
    gActive = true;
    (void)beginUsbStack();
  }
#else
  if (usbSenseVoltage > BATTERY_USB_POWER_V && !gUnsupportedLogged) {
    gUnsupportedLogged = true;
    Serial.println("[USB MIDI] unavailable: build the board with USB-OTG TinyUSB MIDI support.");
  }
#endif
}

// Drain incoming packets so the USB stack remains healthy.
void service() {
#if USB_MIDI_AVAILABLE
  if (!gActive || !gStackStarted) return;
  uint8_t packet[4] = {0, 0, 0, 0};
  while (tud_midi_packet_read(packet)) {
  }
#endif
}

// Send a MIDI note-on for the current scale mapping.
void noteOn(uint8_t noteIndex) {
#if USB_MIDI_AVAILABLE
  if (!gActive || !gStackStarted) return;
  if (noteIndex >= USB_MIDI_MAX_NOTES) return;

  uint8_t midiNote = Audio::midiNoteFromNoteIndex(noteIndex);
  gPadNoteActive[noteIndex] = true;
  gPadMidiNote[noteIndex] = midiNote;
  sendChannelPacket(MIDI_CIN_NOTE_ON, 0x90, midiNote, USB_MIDI_NOTE_VELOCITY);
#else
  (void)noteIndex;
#endif
}

// Send the matching note-off, using the note cached at note-on time.
void noteOff(uint8_t noteIndex) {
#if USB_MIDI_AVAILABLE
  if (!gActive || !gStackStarted) return;
  if (noteIndex >= USB_MIDI_MAX_NOTES) return;

  uint8_t midiNote = gPadNoteActive[noteIndex]
                     ? gPadMidiNote[noteIndex]
                     : Audio::midiNoteFromNoteIndex(noteIndex);
  gPadNoteActive[noteIndex] = false;
  sendChannelPacket(MIDI_CIN_NOTE_OFF, 0x80, midiNote, 0);
#else
  (void)noteIndex;
#endif
}

// Clear all locally tracked notes on the host side.
void allNotesOff() {
#if USB_MIDI_AVAILABLE
  if (!gActive || !gStackStarted) return;
  for (uint8_t i = 0; i < USB_MIDI_MAX_NOTES; ++i) {
    if (!gPadNoteActive[i]) continue;
    sendChannelPacket(MIDI_CIN_NOTE_OFF, 0x80, gPadMidiNote[i], 0);
    gPadNoteActive[i] = false;
  }
  sendChannelPacket(MIDI_CIN_CONTROL_CHANGE, 0xB0, 123, 0);
#endif
}

} // namespace UsbMidi
