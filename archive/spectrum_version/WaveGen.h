#pragma once
#include "Types.h"

void wavegen_init();
void wavegen_apply(const Settings& s);     // update settings
Settings wavegen_get();                   // current (includes actualHz)

const char* wavegen_name(Waveform w);
