#pragma once
#include <Arduino.h>

enum class Waveform : uint8_t { Sine=0, Tri=1, Saw=2, Square=3 };

struct Settings {
  Waveform wave = Waveform::Sine;
  float targetHz = 1000.0f;
  float actualHz  = 1000.0f;
  uint8_t amp = 255; // 0..255 for DAC modes; ignored for square (PWM is 0/3.3V)
};

struct Spectrum {
  float peakHz = 0.0f;
  uint8_t bins[32] = {0}; // 0..63 display level
};
