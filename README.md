# `ESP32 Signal Generator`

A simple embedded waveform generator built on the ESP32-WROOM-32.
The project was intentionally redesigned from a larger experimental version to ocus on timer-driven waveform generation, hardware control, and clear architecture.

The generator produces analog waveforms using the ESP32 DAC and hardware PWM and allows real-time control through physical buttons.

---

## Features

* Sine, Triangle, Sawtooth and Square wave output
* 5-button hardware control interface

  * Waveform select
  * Frequency increase / decrease
  * Amplitude increase / decrease
* Timer-driven waveform generation 
* DAC output for analog waveforms (GPIO25)
* Hardware PWM (LEDC) for stable high-frequency square waves

---

## Why this project exists

Many simple microcontroller waveform generators use `delay()` loops to control
sample timing. Execution-time variation can make this approach unsuitable when
more predictable waveform timing is required.

This implementation instead uses a periodic `esp_timer` callback.

The timer period is calculated from the desired waveform frequency and LUT size.
Each callback outputs one sample from the waveform lookup table and advances
the sample index.

This separates waveform timing from the main program loop and allows the
processor to handle user input independently.
---

## Architecture

The generator uses lookup-table-based waveform synthesis.

**Core components:**

1. Lookup Tables (LUTs)

   * 256 precomputed 8-bit samples for sine, triangle and sawtooth waveforms
   * Avoids expensive waveform calculations during output

2. Periodic Timer (`esp_timer`)

   * Schedules DAC sample updates
   * Timer period is derived from the requested waveform frequency

3. Sample Index

   * Advances through the waveform LUT on each timer callback
   * Wraps after 256 samples

4. DAC / PWM Output

   * ESP32 DAC → sine / triangle / sawtooth
   * LEDC hardware PWM → square wave

```
Requested Frequency → Timer Period → LUT Index → DAC Output
```

## Frequency Control

For LUT-driven waveforms, the timer callback period is calculated from the
requested output frequency and the 256-sample waveform table.

sample rate = output frequency × LUT size

timer period = 1 / sample rate

Square waves use the ESP32 LEDC hardware PWM peripheral instead of the DAC
sampling path.

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

1. Open `main_v1.ino` in Arduino IDE / PlatformIO
2. Select **ESP32 Dev Module**
3. Upload to ESP32

The generator starts immediately on boot.

---

## Earlier Version

An earlier experimental version with ADC sampling, FFT-based spectrum analysis
and OLED visualization is preserved in the:

`spectrum-version`

branch of this repository.

The current `main` branch was simplified to focus on waveform generation,
timing and hardware control.

---

## What I learned

* Periodic timer callbacks vs software delays
* Deterministic embedded timing
* DAC quantization limits
* Hardware PWM vs DAC tradeoffs
