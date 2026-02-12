# `ESP32 Signal Generator`

A simpleembedded waveform generator built on the ESP32-WROOM-32.
The project was intentionally redesigned from a larger experimental version to focus on deterministic timing, hardware control, and clear architecture.

The generator produces analog waveforms using the ESP32 DAC and hardware PWM and allows real-time control through physical buttons.

---

## Features

* Sine, Triangle, Sawtooth and Square wave output
* 5-button hardware control interface

  * Waveform select
  * Frequency increase / decrease
  * Amplitude increase / decrease
* Timer-driven waveform generation (DDS-style phase accumulator)
* DAC output for analog waveforms (GPIO25)
* Hardware PWM (LEDC) for stable high-frequency square waves

---

## Why this project exists

Many microcontroller “signal generator” projects use `delay()` loops to output samples.
This produces unstable frequencies because execution time varies.

This implementation instead uses a hardware timer interrupt:

The timer fires at a fixed sampling rate →
each interrupt outputs exactly one sample →
the phase accumulator determines the waveform frequency.

This makes frequency depend only on math, not CPU speed.

---

## Architecture

The generator is essentially a simplified Direct Digital Synthesis (DDS) system.

**Core components:**

1. Lookup Tables (LUTs)

   * Precomputed 8-bit samples for each waveform
   * Prevents expensive math inside interrupts

2. Hardware Timer (`esp_timer`)

   * Generates a constant sample rate
   * Drives waveform output

3. Phase Accumulator

   * Controls frequency
   * Larger phase step → higher frequency

4. DAC / PWM Output

   * DAC → sine/triangle/sawtooth
   * LEDC PWM → square wave

```
Timer Interrupt → Phase Accumulator → LUT → DAC Output
```

---

## Frequency Control

Frequency is controlled by changing the phase step, not the timer period.

Output frequency = sample rate × phase step / LUT size

This allows smooth frequency changes without restarting the timer.

---

## Hardware

**Board:** ESP32-WROOM-32

| Component         | Purpose                   |
| ----------------- | ------------------------- |
| GPIO25            | DAC waveform output       |
| LEDC PWM pin      | Square wave output        |
| 5 push buttons    | User control              |


Buttons:

* Wave select
* F+
* F-
* A+
* A-

---

## How to Run

1. Open `main.ino` in Arduino IDE / PlatformIO
2. Select **ESP32 Dev Module**
3. Upload to ESP32

The generator starts immediately on boot.

---

## Earlier Version

An earlier experimental version with FFT spectrum visualization and WiFi interface is preserved in the:

```
spectrum-version
```

branch of this repository.

The current `main` branch is a simplified implementation.

---

## What I learned

* Timer interrupts vs software delays
* Deterministic embedded timing
* Direct Digital Synthesis (DDS)
* DAC quantization limits
* Hardware PWM vs DAC tradeoffs
