#pragma once
#include "Types.h"

void ui_init();
bool ui_poll(Settings& s);           // returns true if settings changed
void ui_draw(const Settings& s, const Spectrum& sp);
