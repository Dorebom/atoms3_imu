#pragma once

#if defined(USE_M5UNIFIED) && !defined(USE_M5STACK_OFFICIAL)
// clang-format off
#include <M5Unified.h>
#if defined(USE_FASTLED)
#define FASTLED_INTERNAL
#include <FastLED.h>
#endif
// clang-format on
inline void M5_BEGIN(void) {
    M5.begin();
}
inline void M5_BEGIN(m5::M5Unified::config_t& cfg) {
    M5.begin(cfg);
}
#elif defined(ARDUINO_M5STACK_ATOMS3)
#include <M5AtomS3.h>
inline void M5_BEGIN(bool LCDEnable = true, bool SerialEnable = true,
                     bool I2CEnable = true, bool LEDEnable = false) {
    M5.begin(LCDEnable, SerialEnable, I2CEnable, LEDEnable);
}
#endif

inline void M5_UPDATE(void) {
#if defined(ARDUINO_M5STACK_CORES3) || \
    (defined(ARDUINO_M5STACK_STAMPS3) && !defined(ARDUINO_M5STACK_DIAL))
#else
    M5.update();
#endif
}
