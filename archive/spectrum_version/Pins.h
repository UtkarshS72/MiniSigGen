#pragma once
#include <Arduino.h>

// ---- Outputs ----
static constexpr uint8_t PIN_DAC_OUT = 25;   // DAC1 for sine, triangle and sawtooth
static constexpr uint8_t PIN_PWM_OUT = 26;   // LEDC PWM for square

// ---- FFT / ADC ----
static constexpr uint8_t PIN_ADC_IN  = 34;   // ADC1 input-only

// ---- Buttons (active-low, INPUT_PULLUP) ----
static constexpr uint8_t PIN_BTN_FREQ_UP   = 32;
static constexpr uint8_t PIN_BTN_FREQ_DOWN = 33;
static constexpr uint8_t PIN_BTN_AMP_UP    = 27;
static constexpr uint8_t PIN_BTN_AMP_DOWN  = 4;
static constexpr uint8_t PIN_BTN_WAVE_NEXT = 14;

// ---- OLED ----
static constexpr uint8_t OLED_ADDR = 0x3C;
