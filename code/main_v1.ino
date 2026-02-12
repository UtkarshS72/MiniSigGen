#include <Arduino.h>
#include "driver/dac.h"

// ------------------------- User config -------------------------

// Output pins
static constexpr int DAC_PIN = 25;          // DAC1 = GPIO25
static constexpr dac_channel_t DAC_CH = DAC_CHANNEL_1;

static constexpr int SQUARE_PIN = 27;       // LEDC output pin (square wave)
static constexpr int LEDC_CH = 0;
static constexpr int LEDC_RES_BITS = 10;    // 10-bit resolution
static constexpr int LEDC_DUTY_50 = (1 << LEDC_RES_BITS) / 2;

// Button pins (active-low buttons to GND)
static constexpr int BTN_MODE = 32;
static constexpr int BTN_FUP  = 33;
static constexpr int BTN_FDN  = 18;
static constexpr int BTN_AUP  = 19;
static constexpr int BTN_ADN  = 21;

// Debounce
static constexpr uint32_t DEBOUNCE_MS = 35;

// Wave / LUT
static constexpr uint16_t LUT_SIZE = 256;

// Frequency limits (for LUT-driven DAC waves)
static constexpr float FREQ_MIN = 1.0f;
static constexpr float FREQ_MAX = 5000.0f;     // practical limit due to timer rate & ISR/task overhead

// Amplitude steps (0..255 DAC code scale)
static constexpr int AMP_STEP = 16;            // each press changes ~6% amplitude
static constexpr int AMP_MIN  = 0;
static constexpr int AMP_MAX  = 255;

// Frequency step (Hz)
static constexpr float FREQ_STEP = 25.0f;

// ------------------------- State -------------------------

enum WaveMode : uint8_t { MODE_SINE, MODE_TRI, MODE_SAW, MODE_SQUARE };
static WaveMode mode = MODE_SINE;

static float frequencyHz = 500.0f;
static int amplitude = 160;          // 0..255 peak-to-peak scale-ish (see mapping below)

static uint8_t lutSine[LUT_SIZE];
static uint8_t lutTri[LUT_SIZE];
static uint8_t lutSaw[LUT_SIZE];

static esp_timer_handle_t waveTimer = nullptr;
static volatile uint16_t phaseIdx = 0;

// Timer period in microseconds
static volatile uint32_t timerPeriodUs = 20;   // updated from frequency
static volatile bool timerActive = false;

// ------------------------- Helpers -------------------------

static float clampf(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

static int clampi(int x, int lo, int hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

static const char* modeName(WaveMode m) {
  switch (m) {
    case MODE_SINE:   return "SINE";
    case MODE_TRI:    return "TRI";
    case MODE_SAW:    return "SAW";
    case MODE_SQUARE: return "SQUARE";
    default:          return "?";
  }
}

// Build LUTs 
static void buildLUTs() {
  // Sine: 0..255
  for (uint16_t i = 0; i < LUT_SIZE; i++) {
    float x = (2.0f * PI * i) / LUT_SIZE;
    float s = sinf(x);                 // -1..+1
    float u = (s * 0.5f) + 0.5f;       // 0..1
    lutSine[i] = (uint8_t)lroundf(u * 255.0f);
  }

  // Triangle: 0..255 (up then down)
  for (uint16_t i = 0; i < LUT_SIZE; i++) {
    float p = (float)i / (LUT_SIZE - 1);  // 0..1
    float tri = (p < 0.5f) ? (2.0f * p) : (2.0f * (1.0f - p)); // 0..1..0
    lutTri[i] = (uint8_t)lroundf(tri * 255.0f);
  }

  // Saw: 0..255
  for (uint16_t i = 0; i < LUT_SIZE; i++) {
    lutSaw[i] = (uint8_t)((i * 255) / (LUT_SIZE - 1));
  }
}

// Convert base LUT sample (0..255) to amplitude-scaled sample centered around midscale.
// amplitude is 0..255 meaning "how wide around 128" we allow the wave to go.
static inline uint8_t applyAmplitude(uint8_t base) {
  // base: 0..255, centered at 128
  int centered = (int)base - 128;                  // -128..+127
  int scaled = (centered * amplitude) / 255;       // scale around 0
  int out = 128 + scaled;                          // back to 0..255-ish
  return (uint8_t)clampi(out, 0, 255);
}

// For LUT-driven waves: need sample-rate = frequency * LUT_SIZE
static uint32_t calcTimerPeriodUs(float fHz) {
  float sampleRate = fHz * (float)LUT_SIZE;     // samples/sec
  if (sampleRate < 1.0f) sampleRate = 1.0f;
  float periodUs = 1000000.0f / sampleRate;
  // Prevent absurdly small periods (esp_timer is not a hard-realtime ISR; keep safe)
  if (periodUs < 20.0f) periodUs = 20.0f;       // 20us => 50 kHz timer callback rate
  if (periodUs > 50000.0f) periodUs = 50000.0f; // 50ms => very low freq
  return (uint32_t)lroundf(periodUs);
}

// ------------------------- Wave engine -------------------------

static void stopTimerWave() {
  if (waveTimer && timerActive) {
    esp_timer_stop(waveTimer);
    timerActive = false;
  }
}

static void startTimerWave() {
  if (!waveTimer) return;
  phaseIdx = 0;
  esp_timer_start_periodic(waveTimer, timerPeriodUs);
  timerActive = true;
}

static void setSquareWave(float fHz) {
  // LEDC handles the timing in hardware (more stable than software)
  ledcSetup(LEDC_CH, (uint32_t)lroundf(fHz), LEDC_RES_BITS);
  ledcAttachPin(SQUARE_PIN, LEDC_CH);
  ledcWrite(LEDC_CH, LEDC_DUTY_50);
}

static void stopSquareWave() {
  ledcDetachPin(SQUARE_PIN);
  pinMode(SQUARE_PIN, OUTPUT);
  digitalWrite(SQUARE_PIN, LOW);
}

static void applyMode() {
  // Stop everything first
  stopTimerWave();
  stopSquareWave();
  dac_output_disable(DAC_CH);

  if (mode == MODE_SQUARE) {
    setSquareWave(frequencyHz);
  } else {
    dac_output_enable(DAC_CH);
    timerPeriodUs = calcTimerPeriodUs(frequencyHz);
    startTimerWave();
  }

  Serial.printf("[MODE=%s] f=%.1f Hz  amp=%d  timerPeriod=%uus\n",
                modeName(mode), frequencyHz, amplitude, (unsigned)timerPeriodUs);
}

// Timer callback: outputs one sample per tick to DAC.
// We keep it tiny and deterministic.
static void IRAM_ATTR waveTimerCb(void* arg) {
  (void)arg;

  uint8_t base;
  switch (mode) {
    case MODE_SINE: base = lutSine[phaseIdx & 0xFF]; break;
    case MODE_TRI:  base = lutTri [phaseIdx & 0xFF]; break;
    case MODE_SAW:  base = lutSaw [phaseIdx & 0xFF]; break;
    default:        base = 128; break;
  }

  uint8_t out = applyAmplitude(base);
  dac_output_voltage(DAC_CH, out);

  phaseIdx = (phaseIdx + 1) & 0xFF;
}

// ------------------------- Button handling -------------------------

struct Btn {
  int pin;
  bool last;
  uint32_t lastChangeMs;
};

static Btn bMode{BTN_MODE, true, 0};
static Btn bFup {BTN_FUP,  true, 0};
static Btn bFdn {BTN_FDN,  true, 0};
static Btn bAup {BTN_AUP,  true, 0};
static Btn bAdn {BTN_ADN,  true, 0};

static bool readPressed(Btn &b) {
  bool now = digitalRead(b.pin); // with pullups: HIGH=idle, LOW=pressed
  uint32_t t = millis();

  if (now != b.last) {
    b.last = now;
    b.lastChangeMs = t;
  }

  // stable long enough + is pressed?
  if ((t - b.lastChangeMs) >= DEBOUNCE_MS && now == LOW) {
    // latch until release (simple: wait for release in main loop is enough)
    return true;
  }
  return false;
}

static void waitRelease(int pin) {
  while (digitalRead(pin) == LOW) delay(1);
  delay(DEBOUNCE_MS);
}

// ------------------------- Arduino -------------------------

void setup() {
  Serial.begin(115200);
  Serial.println("\n=== ESP32 Minimal Signal Generator ===");

  // Buttons
  pinMode(BTN_MODE, INPUT_PULLUP);
  pinMode(BTN_FUP,  INPUT_PULLUP);
  pinMode(BTN_FDN,  INPUT_PULLUP);
  pinMode(BTN_AUP,  INPUT_PULLUP);
  pinMode(BTN_ADN,  INPUT_PULLUP);

  // Output init
  pinMode(SQUARE_PIN, OUTPUT);
  digitalWrite(SQUARE_PIN, LOW);

  buildLUTs();

  // Timer init
  esp_timer_create_args_t args = {};
  args.callback = &waveTimerCb;
  args.arg = nullptr;
  args.dispatch_method = ESP_TIMER_TASK; // runs in timer task, not a true ISR; safe and simple
  args.name = "wave";

  esp_err_t err = esp_timer_create(&args, &waveTimer);
  if (err != ESP_OK) {
    Serial.printf("esp_timer_create failed: %d\n", (int)err);
    while (true) delay(1000);
  }

  applyMode();
}

void loop() {
  // MODE
  if (readPressed(bMode)) {
    waitRelease(BTN_MODE);
    mode = (WaveMode)((mode + 1) % 4);
    applyMode();
  }

  // Frequency+
  if (readPressed(bFup)) {
    waitRelease(BTN_FUP);
    frequencyHz = clampf(frequencyHz + FREQ_STEP, FREQ_MIN, FREQ_MAX);
    applyMode();
  }

  // Frequency-
  if (readPressed(bFdn)) {
    waitRelease(BTN_FDN);
    frequencyHz = clampf(frequencyHz - FREQ_STEP, FREQ_MIN, FREQ_MAX);
    applyMode();
  }

  // Amplitude+
  if (readPressed(bAup)) {
    waitRelease(BTN_AUP);
    amplitude = clampi(amplitude + AMP_STEP, AMP_MIN, AMP_MAX);
    applyMode();
  }

  // Amplitude-
  if (readPressed(bAdn)) {
    waitRelease(BTN_ADN);
    amplitude = clampi(amplitude - AMP_STEP, AMP_MIN, AMP_MAX);
    applyMode();
  }

  delay(1);
}
