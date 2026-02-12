#include "FFTProc.h"
#include "Pins.h"

#include <Arduino.h>
#include "arduinoFFT.h"

static constexpr uint16_t N = 256;
static constexpr uint32_t FS = 10000;   // 10 kHz
static constexpr uint32_t UPDATE_MS = 200; 
static constexpr float F_MIN = 100.0f;
static constexpr float F_MAX = 4500.0f;

static double vReal[N];
static double vImag[N];
static ArduinoFFT<double> FFT(vReal, vImag, N, FS);

static uint32_t lastMs = 0;

static void acquire_blocking() {
  const uint32_t Ts = 1000000UL / FS;
  uint32_t t = micros();

  for (uint16_t i = 0; i < N; i++) {
    while ((uint32_t)(micros() - t) < Ts) { /* wait */ }
    t += Ts;

    int raw = analogRead(PIN_ADC_IN); // 0..4095
    vReal[i] = (double)raw;
    vImag[i] = 0.0;
  }

  // DC remove
  double mean = 0.0;
  for (uint16_t i = 0; i < N; i++) mean += vReal[i];
  mean /= (double)N;
  for (uint16_t i = 0; i < N; i++) vReal[i] -= mean;
}

void fft_init() {
  analogReadResolution(12);
  analogSetPinAttenuation(PIN_ADC_IN, ADC_11db);
  lastMs = 0;
}

bool fft_poll(Spectrum& out) {
  uint32_t now = millis();
  if (now - lastMs < UPDATE_MS) return false;
  lastMs = now;

  acquire_blocking();
  FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);
  FFT.compute(FFTDirection::Forward);
  FFT.complexToMagnitude();

  // Peak find in band
  uint16_t binMin = (uint16_t)(F_MIN * (float)N / (float)FS);
  uint16_t binMax = (uint16_t)(F_MAX * (float)N / (float)FS);
  if (binMin < 1) binMin = 1;
  if (binMax > (N/2 - 1)) binMax = (N/2 - 1);

  uint16_t peakIdx = binMin;
  double peakVal = 0.0;
  for (uint16_t i = binMin; i <= binMax; i++) {
    if (vReal[i] > peakVal) { peakVal = vReal[i]; peakIdx = i; }
  }
  out.peakHz = (float)peakIdx * (float)FS / (float)N;

  // Normalize + downsample into 32 bins from 1..(N/2-1)
  double maxMag = 1.0;
  for (uint16_t i = 1; i < N/2; i++) if (vReal[i] > maxMag) maxMag = vReal[i];

  // Map 127 bins (1..127) -> 32 bars (about 4 bins per bar)
  for (uint8_t b = 0; b < 32; b++) {
    uint16_t start = 1 + (uint16_t)((b * (N/2 - 1)) / 32.0);
    uint16_t end   = 1 + (uint16_t)(((b + 1) * (N/2 - 1)) / 32.0);
    if (end <= start) end = start + 1;
    if (end > (N/2)) end = (N/2);

    double acc = 0.0;
    uint16_t cnt = 0;
    for (uint16_t i = start; i < end; i++) { acc += vReal[i]; cnt++; }
    double avg = (cnt ? acc / (double)cnt : 0.0);
    double norm = avg / maxMag; // 0..1

    if (norm < 0.0) norm = 0.0;
    if (norm > 1.0) norm = 1.0;

    out.bins[b] = (uint8_t)(norm * 63.0); // 0..63 pixels
  }

  return true;
}
