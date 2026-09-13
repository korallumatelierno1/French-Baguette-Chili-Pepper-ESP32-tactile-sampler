# French Baguette — quick guide

French Baguette is a touch instrument with four synthesizer sounds,
an SD card sample player, effects, and USB MIDI output.
This guide describes the controls in the firmware in this folder.

## Getting started

1. Connect the power supply and connect the audio output to a powered speaker or an audio input.
2. To use MIDI, connect a USB data cable to your computer **before powering on**.
3. Wait for initialization to finish, then touch the note pads to play.

Startup settings: **synth mode, Omnichord sound, `majD#3` scale, 70% volume, FX Off**.
Changes to the sound, volume, scale, octave, and effects are not saved
when the instrument is powered off or RESET is pressed.

## Controls in synthesizer mode

| Pad | Action |
| --- | --- |
| Note pads | Play a note; release the pad to let the note fade out. |
| Pigeon | Select the next sound **when released**. |
| Cloud | Lower the volume by 5 percentage points. |
| Flower | Raise the volume by 5 percentage points. |
| Eiffel Tower | Shift down one octave. |
| Bottle | Shift up one octave. |
| Cheese | No action in synth mode; used in sampler mode. |
| FX pad | Select the next effect. |

Volume ranges from 0 to 100%. Octave transposition ranges from −2 to +2.
Release and touch again to repeat a command: holding a pad does not repeat it.

The synth plays **up to two voices** by default. A new note first replaces a
voice that is already fading out, otherwise the oldest voice. The replaced
voice fades out over 20 ms before the new note starts.

The Pigeon pad cycles through these sounds, starting with the startup sound:

**Omnichord → Steel Drum → Vapor Flute → Accordion → Omnichord**.

## All touch combinations

Touch both pads almost simultaneously, then **release both** before trying
again. Aim for less than 90 ms between touches to avoid triggering an individual
pad action first. Holding a combination triggers it only once.

| Combination | Action | Mode |
| --- | --- | --- |
| **Eiffel Tower + Cloud** | Select the previous scale root. | Synth |
| **Eiffel Tower + Flower** | Select the next scale root. | Synth |
| **Eiffel Tower + Bottle** | Switch between **Synth ↔ Sampler**. | Both |

Scale combinations move through the twelve distinct chromatic roots while
preserving the major/minor mode and octave. The list wraps in both directions:

`C → C# → D → D# → E → F → F# → G → G# → A → A# → B → C`

From the default `majD#3`, **Eiffel Tower + Flower** selects `majE3`;
**Eiffel Tower + Cloud** takes `majE3` back to `majD#3`.
From `majD#3`, **Eiffel Tower + Cloud** selects `majD3`.

Navigation uses sharp names and automatically skips equivalent spellings, so
there is no separate step from D# to Eb. Flats and other enharmonic spellings
are still accepted in the initial `PANORYTHE_SCALE` setting in
[Config.h](Config.h). Choose `maj` or `min` there, then upload the firmware
to apply the change.

## Effects

The FX pad cycles through **Off → Reverb → Drive → Echoloop → Miettes → Off**.
One effect is selected at a time, and it processes both synth sounds and samples.

| Effect | Sound |
| --- | --- |
| Off | Dry sound. |
| Reverb | Warm reverberation. |
| Drive | Distortion with output level compensation. |
| Echoloop | Repeats short fragments of recent audio. |
| Miettes | Repeated, pitch-shifted or reversed fragments, irregular patterns, and ambient textures. |

Miettes evolves automatically as you play and while sound tails decay.
It uses an internal tempo; it does not follow your playing tempo or a MIDI clock.
The `MIETTES_MIX`, `MIETTES_FEEDBACK`, `MIETTES_BPM`,
`MIETTES_PITCH_RANDOM`, and `MIETTES_TIME_RANDOM` settings are in [Config.h](Config.h).
Both randomness settings range from 0 (regular patterns) to 1 (maximum variation).

## Sampler mode and the SD card

Prepare the card on your computer, insert it before powering on, then touch
**Eiffel Tower + Bottle**. Files are loaded into memory each time you enter
sampler mode; let loading finish before playing.

To get started, put one **16-bit PCM WAV file, mono, 44.1 kHz** in each folder
you want to use. Example: `/sample/pigeon_pad/son.wav`.

| Pad in sampler mode | Primary folder on the SD card | Alternative folder |
| --- | --- | --- |
| Pigeon | `/sample/pigeon_pad/` | `/sample/pad_1/` |
| Eiffel Tower | `/sample/Eiffel_pad/` | `/sample/pad_2/` |
| Cloud | `/sample/cloud_pad/` | `/sample/pad_3/` |
| Flower | `/sample/flower_pad/` | `/sample/pad_4/` |
| Bottle | `/sample/bottle_pad/` | `/sample/pad_5/` |
| Cheese | `/sample/cheese_pad/` | `/sample/pad_6/` |
| Note pads 1–4 | `/sample/pad_7/` through `/sample/pad_10/` | — |

Note pads 5–10 also trigger samples 5–10. These pad numbers follow their
note order in synth mode.

Primary folders take priority over the alternatives. You can also place a file
directly in `/sample/`, for example `pigeon_pad.wav` or `pad_7.wav`.
Use the exact names, including the capital E in `Eiffel_pad`.

Each sample plays to the end, even after you release the pad; touch it again to
retrigger it. Up to ten sample voices can overlap. Missing samples remain silent.
Files that are too long are truncated to fit the available memory.
The player also accepts 8/24/32-bit PCM WAV and 32-bit float WAV, in mono or stereo;
stereo is mixed down to mono. PSRAM allows longer samples to be loaded.

In sampler mode, the illustrated pads trigger samples: set the volume, octave,
and synth sound **before** entering this mode. The FX pad remains available.
To return to the synth, touch **Eiffel Tower + Bottle** again.
Scale and octave settings change synth and MIDI notes, but do not change WAV playback pitch.

## USB MIDI

Connect a USB data cable to the ESP32-S3's native USB port, then power on
or press **RESET on its own**. In your music software, select the instrument's
MIDI input. The firmware uses **channel 1**, with a fixed velocity of **100**
by default.

Touching and releasing note pads sends MIDI notes alongside the internal sound.
Effects process only the internal audio. This firmware sends MIDI;
it does not play incoming MIDI notes or send audio over USB.
USB power is detected only at startup.

## BOOT / RESET: entering the bootloader

Use the **physical BOOT and RESET buttons** on the ESP32-S3 board.
RESET may be labeled **RST** or **EN**, and BOOT may be labeled **FLASH**.

1. Connect the board to your computer with a USB data cable.
2. **Press and hold BOOT.**
3. While holding BOOT, **press and release RESET**.
4. **Release BOOT.**
5. Wait for the upload port to appear and select it in the IDE.
6. Start the upload.

Reminder: **hold BOOT → press and release RESET → release BOOT**.
This sequence starts the ESP32-S3's download mode; it does not erase the
firmware by itself. It relies on BOOT being held during reset, as described in
the [Espressif documentation](https://docs.espressif.com/projects/esptool/en/latest/esp32s3/advanced-topics/boot-mode-selection.html).

After uploading, **press RESET on its own**, without holding BOOT, if the
program does not restart automatically. The USB port may change between the
bootloader and the firmware: select the port that appears again.
See also [Espressif's USB instructions](https://docs.espressif.com/projects/arduino-esp32/en/latest/tutorials/cdc_dfu_flash.html).

## Uploading from Arduino IDE

1. Open [French_Baguette0.0.1.ino](French_Baguette0.0.1.ino), keeping all project files in the same folder.
2. Install the **esp32 by Espressif Systems** board package and the **Adafruit MPR121** library, including its dependencies.
3. Select your ESP32-S3 board and its actual flash size and PSRAM type.
4. For MIDI, select **USB Mode: USB-OTG (TinyUSB)**. With the current logger, keep **USB CDC On Boot: Disabled**: logging uses the hardware serial port.
5. Enter the bootloader using the sequence above if needed, select the port, and upload.

Memory settings depend on the installed module: do not enable PSRAM if your
module does not have it. Changes to [Config.h](Config.h) take effect after
recompiling and uploading. The installed ESP32 core used to build this project
is **3.3.2**.

## Quick troubleshooting

| Symptom | What to check |
| --- | --- |
| No sound | Check the audio connection and volume, then return to synth mode with Eiffel Tower + Bottle. |
| Combination not recognized | Touch only the two relevant pads almost simultaneously, and release both before trying again. |
| Sampler is silent or will not activate | Check the SD card, WAV paths, and available memory; sampler mode can be active but empty if no valid WAV files were found. |
| No MIDI | Check the data cable, native USB port, and TinyUSB build setting, then press RESET with USB already connected. |
| No upload port | Repeat BOOT/RESET and look for the new port; try another USB data cable. |
| Instrument does not start after an upload | Release BOOT, then press RESET on its own. |

Diagnostic messages are sent at **115200 baud** over the hardware serial port
(through the USB–UART bridge, if the board has one). They report the mode,
sound, effects, and SD read errors, among other information. Host DSP tests are
described in [tests/README.md](tests/README.md).
