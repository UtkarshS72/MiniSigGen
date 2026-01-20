#include "WaveGen.h"
#include "Pins.h"

#include <driver/dac.h>
#include <esp_timer.h>
#include <math.h>

static Settings g;

static constexpr uint16_t LUT_N = 256;
static uint8_t lut_sin[LUT_N];
static uint8_t lut_tri[LUT_N];
static uint8_t lut_saw[LUT_N];

static volatile uint16_t phase = 0;
static volatile uint16_t phaseStep = 1;      // Q0.0 integer step in LUT indices
static volatile uint8_t  ampScale = 255;     // 0..255
static volatile Waveform wf = Waveform::Sine;

static esp_timer_handle_t tmr = nullptr;
static volatile uint32_t periodUs = 50;      // timer period

// PWM config (square)
static constexpr int PWM_CH = 0;
static constexpr int PWM_RES_BITS = 8;       
static constexpr uint32_t PWM_MAX_HZ = 10000000; 
static constexpr float DAC_MAX_HZ = 20000.0f;    // limitation

static inline uint8_t lut_get_sample(Waveform w, uint16_t idx) {
  switch (w) {
    case Waveform::Sine:   return lut_sin[idx];
    case Waveform::Tri:    return lut_tri[idx];
    case Waveform::Saw:    return lut_saw[idx];
    default:               return 0;
  }
}

static void IRAM_ATTR on_timer(void*) {
  if (wf == Waveform::Square) return; // DAC timer unused in square mode

  uint8_t raw = lut_get_sample(wf, phase & (LUT_N - 1));
  // scale amplitude in 0..255 (fast integer scaling)
  uint8_t out = (uint16_t(raw) * ampScale) >> 8;
  dac_output_voltage(DAC_CHANNEL_1, out); // GPIO25 is DAC1

  phase = (phase + phaseStep) & (LUT_N - 1);
}

static void build_luts_once() {
  for (uint16_t i = 0; i < LUT_N; i++) {
    float t = (float)i / (float)LUT_N;
    lut_sin[i] = (uint8_t)(127.5f + 127.5f * sinf(2.0f * (float)M_PI * t) + 0.5f);
    lut_tri[i] = (i < 128) ? (uint8_t)(i * 2) : (uint8_t)((255 - i) * 2);
    lut_saw[i] = (uint8_t)i;
  }
}

static void dac_timer_start(uint32_t perUs) {
  if (tmr) {
    esp_timer_stop(tmr);
    esp_timer_delete(tmr);
    tmr = nullptr;
  }
  esp_timer_create_args_t args = {
    .callback = &on_timer,
    .arg = nullptr,
    .dispatch_method = ESP_TIMER_TASK,
    .name = "dac_wave"
  };
  esp_timer_create(&args, &tmr);
  esp_timer_start_periodic(tmr, perUs);
}

static void dac_timer_stop() {
  if (tmr) {
    esp_timer_stop(tmr);
    esp_timer_delete(tmr);
    tmr = nullptr;
  }
}

static float configure_dac_wave(float targetHz) {
  // Keep target in a reasonable range
  if (targetHz < 1.0f) targetHz = 1.0f;
  if (targetHz > DAC_MAX_HZ) targetHz = DAC_MAX_HZ;

  // sampleRate = LUT_N * targetHz.
  // But esp_timer has a practical min period (~50us stable-ish). We use phaseStep to go faster.
  // Let periodUs = 50us always, choose phaseStep such that:
  // actualHz = (1e6 / periodUs) * (phaseStep / LUT_N)
  periodUs = 50;
  float fs = 1000000.0f / (float)periodUs;
  float idealStep = (targetHz * LUT_N) / fs;

  uint16_t st = (uint16_t)(idealStep + 0.5f);
  if (st < 1) st = 1;
  if (st > 64) st = 64; // prevent extreme coarseness

  phaseStep = st;
  float actual = fs * ((float)phaseStep / (float)LUT_N);

  dac_output_enable(DAC_CHANNEL_1);
  dac_timer_start(periodUs);
  return actual;
}

static void configure_square(float targetHz) {
  if (targetHz < 1.0f) targetHz = 1.0f;
  if (targetHz > (float)PWM_MAX_HZ) targetHz = (float)PWM_MAX_HZ;

  dac_timer_stop();
  dac_output_disable(DAC_CHANNEL_1);

  ledcDetachPin(PIN_PWM_OUT);
  ledcSetup(PWM_CH, (uint32_t)targetHz, PWM_RES_BITS);
  ledcAttachPin(PIN_PWM_OUT, PWM_CH);
  // fixed 50% duty
  ledcWrite(PWM_CH, 128);
}

void wavegen_init() {
  build_luts_once();
  pinMode(PIN_PWM_OUT, OUTPUT);
  dac_output_enable(DAC_CHANNEL_1);
  wavegen_apply(g);
}

void wavegen_apply(const Settings& s) {
  g = s;

  // clamp amplitude
  if (g.amp > 255) g.amp = 255;
  ampScale = g.amp;
  wf = g.wave;

  if (g.wave == Waveform::Square) {
    configure_square(g.targetHz);
    g.actualHz = g.targetHz; 
  } else {
    g.actualHz = configure_dac_wave(g.targetHz);
  }
}

Settings wavegen_get() { return g; }

const char* wavegen_name(Waveform w) {
  switch (w) {
    case Waveform::Sine:   return "SIN";
    case Waveform::Tri:    return "TRI";
    case Waveform::Saw:    return "SAW";
    case Waveform::Square: return "SQR";
    default:               return "?";
  }
}
