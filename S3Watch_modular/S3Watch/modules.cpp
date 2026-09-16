#include "modules.h"

// ตัวอย่างการเพิ่มโมดูลใหม่:
//
//   static void stepsInit()                    { ... }
//   static void stepsMinute(const struct tm &t){ ... }
//
//   static const Module MODULES[] = {
//     { "steps", stepsInit, stepsMinute, nullptr },
//   };

static const Module MODULES[] = {
  { "none", nullptr, nullptr, nullptr },
};
static const size_t MODULES_N = sizeof(MODULES) / sizeof(MODULES[0]);

void modulesInit() {
  for (size_t i = 0; i < MODULES_N; i++) if (MODULES[i].init) MODULES[i].init();
}

void modulesMinute(const struct tm &t) {
  for (size_t i = 0; i < MODULES_N; i++) if (MODULES[i].onMinute) MODULES[i].onMinute(t);
}

void modulesRaise() {
  for (size_t i = 0; i < MODULES_N; i++) if (MODULES[i].onRaise) MODULES[i].onRaise();
}
