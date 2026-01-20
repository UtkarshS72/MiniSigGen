#include "UI.h"
#include "Pins.h"
#include "WaveGen.h"

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

static constexpr uint8_t W = 128;
static constexpr uint8_t H = 64;

static Adafruit_SSD1306 display(W, H, &Wire, -1);

struct DebBtn {
  uint8_t pin;
  uint8_t stable;     
  uint8_t lastRead;
  uint32_t lastChangeMs;
};

static DebBtn bFU{PIN_BTN_FREQ_UP,   HIGH, HIGH, 0};
static DebBtn bFD{PIN_BTN_FREQ_DOWN, HIGH, HIGH, 0};
static DebBtn bAU{PIN_BTN_AMP_UP,    HIGH, HIGH, 0};
static DebBtn bAD{PIN_BTN_AMP_DOWN,  HIGH, HIGH, 0};
static DebBtn bWN{PIN_BTN_WAVE_NEXT, HIGH, HIGH, 0};

static constexpr uint32_t DEBOUNCE_MS = 25;

static bool deb_pressed_edge(DebBtn& b) {
  uint8_t r = digitalRead(b.pin);
  uint32_t now = millis();

  if (r != b.lastRead) {
    b.lastRead = r;
    b.lastChangeMs = now;
  }

  if ((now - b.lastChangeMs) >= DEBOUNCE_MS && b.stable != b.lastRead) {
    uint8_t prev = b.stable;
    b.stable = b.lastRead;
    // active-low press edge: HIGH -> LOW
    return (prev == HIGH && b.stable == LOW);
  }
  return false;
}

void ui_init() {
  pinMode(PIN_BTN_FREQ_UP,   INPUT_PULLUP);
  pinMode(PIN_BTN_FREQ_DOWN, INPUT_PULLUP);
  pinMode(PIN_BTN_AMP_UP,    INPUT_PULLUP);
  pinMode(PIN_BTN_AMP_DOWN,  INPUT_PULLUP);
  pinMode(PIN_BTN_WAVE_NEXT, INPUT_PULLUP);

  Wire.begin(); // SDA=21 SCL=22 default
  display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("SignalGen + FFT (minimal)");
  display.display();
}

bool ui_poll(Settings& s) {
  bool changed = false;

  if (deb_pressed_edge(bFU)) { s.targetHz *= 1.10f; changed = true; }
  if (deb_pressed_edge(bFD)) { s.targetHz *= 0.90f; changed = true; }

  if (deb_pressed_edge(bAU)) {
    if (s.amp <= 223) s.amp += 32;
    else s.amp = 255;
    changed = true;
  }
  if (deb_pressed_edge(bAD)) {
    if (s.amp >= 32) s.amp -= 32;
    else s.amp = 0;
    changed = true;
  }

  if (deb_pressed_edge(bWN)) {
    uint8_t w = (uint8_t)s.wave;
    w = (uint8_t)((w + 1) & 3);
    s.wave = (Waveform)w;
    changed = true;
  }

  // reasonable boundaries
  if (s.targetHz < 1.0f) s.targetHz = 1.0f;
  if (s.targetHz > 10000000.0f) s.targetHz = 10000000.0f;

  return changed;
}

void ui_draw(const Settings& s, const Spectrum& sp) {
  display.clearDisplay();

  // header
  display.setCursor(0, 0);
  display.print(wavegen_name(s.wave));
  display.print("  ");

  display.print((int)s.targetHz);
  display.print("Hz");

  display.setCursor(80, 0);
  display.print("Pk ");
  display.print((int)sp.peakHz);

  // amp line
  display.setCursor(0, 10);
  display.print("Amp ");
  display.print((uint16_t)s.amp * 100 / 255);
  display.print("%");

  display.setCursor(64, 10);
  display.print("Act ");
  display.print((int)s.actualHz);

  // spectrum bars: 32 bars across bottom 52px height
  const uint8_t top = 20;
  const uint8_t bottom = 63;
  const uint8_t height = bottom - top;

  for (uint8_t i = 0; i < 32; i++) {
    uint8_t x = (uint8_t)(i * 4);  // 32 bars * 4px = 128
    uint8_t h = sp.bins[i];
    if (h > height) h = height;
    uint8_t y0 = bottom;
    uint8_t y1 = bottom - h;
    // draw 3px wide bar
    display.drawLine(x,   y0, x,   y1, SSD1306_WHITE);
    display.drawLine(x+1, y0, x+1, y1, SSD1306_WHITE);
    display.drawLine(x+2, y0, x+2, y1, SSD1306_WHITE);
  }

  display.display();
}
