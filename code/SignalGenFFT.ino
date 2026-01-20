#include "Pins.h"
#include "Types.h"
#include "WaveGen.h"
#include "FFTProc.h"
#include "UI.h"

static Settings st;
static Spectrum sp;

void setup() {
  Serial.begin(115200);

  ui_init();
  wavegen_init();
  fft_init();

  // apply initial settings
  wavegen_apply(st);
  sp = Spectrum{};
}

void loop() {
  // inputs 
  if (ui_poll(st)) {
    wavegen_apply(st);
  }

  // refresh settings (gets actualHz updates)
  st = wavegen_get();

  // FFT
  if (fft_poll(sp)) {
    // updated spectrum
  }

  // OLED redraw consistently
  static uint32_t lastDraw = 0;
  uint32_t now = millis();
  if (now - lastDraw >= 33) { // ~30 fps
    lastDraw = now;
    ui_draw(st, sp);
  }
}
