#pragma once

#include <Arduino.h>

// Analog input used to detect USB power and battery voltage.
#define PIN_BATTERY_SENSE              2
#define BATTERY_DIVIDER_TOP_OHMS       100000.0f
#define BATTERY_DIVIDER_BOTTOM_OHMS    200000.0f
#define BATTERY_ADC_FULL_SCALE_VOLTAGE 3.333333f
#define BATTERY_USB_POWER_V            4.9f

// USB MIDI defaults.
#define USB_MIDI_CHANNEL               1
#define USB_MIDI_NOTE_VELOCITY         100

// SD card over the same dedicated SPI wiring as the working WPCB sketch.
#define SD_CS_PIN                       5
#define SD_SCK_PIN                      12
#define SD_MISO_PIN                     13
#define SD_MOSI_PIN                     14
#define SD_SPI_READY_FREQ_HZ            25000000u
#define SD_SPI_RETRY_FREQ_HZ            4000000u

// Keep a margin for runtime allocations before splitting PSRAM across sample pads.
#define PANORYTHE_SAMPLE_PSRAM_RESERVE_BYTES (256u * 1024u)
#define PANORYTHE_SAMPLE_MIN_SECONDS_PER_PAD 4u

// Synth voice count (1..24) and voice-steal fade duration (milliseconds).
// A stolen slot fades to silence before its replacement starts.
static constexpr uint8_t SYNTH_POLYPHONY = 2;
static constexpr uint16_t SYNTH_VOICE_STEAL_FADE_MS = 20;

// Overdrive output compensation after saturation (0..1; lower = quieter).
static constexpr float DRIVE_OUTPUT_GAIN = 0.35f;

// Miettes: rhythmic fragments, pitch shifts and ambient grain tails.
// Wet processing runs at 22.05 kHz with at most three simultaneous grains.
static constexpr float MIETTES_MIX = 0.50f;       // 0..1, dry to wet.
static constexpr float MIETTES_FEEDBACK = 0.45f;  // 0..0.65, longer ambient tails.
static constexpr uint16_t MIETTES_BPM = 120;      // 40..240, internal rhythmic grid.
static constexpr float MIETTES_PITCH_RANDOM = 0.75f; // 0..1, chance of a random transposition.
static constexpr float MIETTES_TIME_RANDOM = 0.60f;  // 0..1, timing, delay and repeat variation.
// Optional serial timing report to check DSP headroom on the actual board.
static constexpr bool AUDIO_REPORT_RENDER_LOAD = false;

// Initial pad scale.
// Edit this string to change the default musical layout at compile time.
//
// Format:
//   "<mode><root><octave>"
//   Every combination of one valid mode, one valid root, and one valid octave
//   works as long as the generated MIDI notes stay inside 0..127.
//
// Valid modes:
//   "maj" = major scale
//   "min" = natural minor scale
//
// Valid root syntax:
//   Natural note: C, D, E, F, G, A, B
//   Optional sharp: "#", or "s" for filenames/keyboards that avoid symbols
//   Optional flat: "b"
//
// Accepted root examples covering all pitch classes:
//   C
//   Cs, C#, Db
//   D
//   Ds, D#, Eb
//   E, Fb
//   F, E#
//   Fs, F#, Gb
//   G
//   Gs, G#, Ab
//   A
//   As, A#, Bb
//   B, Cb
//
// Valid octaves:
//   Any octave that keeps the resulting MIDI notes inside 0..127.
//
// Examples:
//   "majC3", "minC3", "majF#2", "minBb4", "majA3"
// Synth combos: Eiffel + cloud = previous root, Eiffel + flower = next root.
// Circular order, skipping enharmonic duplicates (mode and octave stay unchanged):
// C, C#, D, D#, E, F, F#, G, G#, A, A#, B.
// Flat/enharmonic spellings remain valid below, e.g. "majEb3" starts on D#.
static constexpr const char* PANORYTHE_SCALE = "majD#3";

// Two MPR121 touch controllers on separate I2C buses to ensure stable use.
#define SDA_PIN                        8
#define SCL_PIN                        9
#define SDA2_PIN                       21
#define SCL2_PIN                       38

#define MPR_ADDR                       0x5A
#define MPR_ADDR2                      0x5C
#define MPR1_IRQ_PIN                   41
#define MPR2_IRQ_PIN                   42

// PCM5102 I2S output pins.
#define I2S_BCLK                       40
#define I2S_LRCK                       17
#define I2S_DOUT                       18
