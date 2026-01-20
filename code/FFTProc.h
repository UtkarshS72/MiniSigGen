#pragma once
#include "Types.h"

void fft_init();
bool fft_poll(Spectrum& out);   // returns true when updated (throttled)
