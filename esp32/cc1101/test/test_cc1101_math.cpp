// Host-side tests for cc1101_math.h - no Arduino toolchain needed.
// Build & run: g++ -std=c++17 -Wall -Wextra -I.. test_cc1101_math.cpp -o /tmp/test_cc1101_math && /tmp/test_cc1101_math
#include "../cc1101_math.h"
#include <cstdio>
#include <cmath>

static int failures = 0;

#define CHECK(cond, msg)                                                          \
  do {                                                                           \
    if (!(cond)) {                                                               \
      std::fprintf(stderr, "FAIL (%s:%d): %s\n", __FILE__, __LINE__, msg);       \
      failures++;                                                                \
    }                                                                            \
  } while (0)

// computeFreqRegisters() og registersToMHz() skal være hverandres inverser
// over hele det støttede båndet (300-6000 MHz).
static void test_roundtrip() {
  for (float mhz = 300.0f; mhz <= 6000.0f; mhz += 17.3f) {
    uint8_t f2, f1, f0;
    cc1101math::computeFreqRegisters(mhz, f2, f1, f0);
    float back = cc1101math::registersToMHz(f2, f1, f0);
    // Registeroppløsning er f_xosc / 2^16 ~= 397 Hz => godt under 0.001 MHz.
    CHECK(std::fabs(back - mhz) < 0.001f, "round-trip frequency mismatch");
  }
}

// Kjente register-verdier, uavhengig beregnet (Python, double presisjon)
// for f_xosc = 26 MHz: freq_reg = round(freq_hz * 2^16 / f_xosc).
static void test_known_values() {
  struct Case { float mhz; uint8_t f2, f1, f0; };
  const Case cases[] = {
      {433.92f, 0x10, 0xB0, 0x71},
      {868.0f,  0x21, 0x62, 0x76},
      {300.0f,  0x0B, 0x89, 0xD9},
      {6000.0f, 0xE6, 0xC4, 0xEC},
      {315.0f,  0x0C, 0x1D, 0x8A},
      {915.0f,  0x23, 0x31, 0x3B},
  };
  for (const auto &c : cases) {
    uint8_t f2, f1, f0;
    cc1101math::computeFreqRegisters(c.mhz, f2, f1, f0);
    CHECK(f2 == c.f2 && f1 == c.f1 && f0 == c.f0, "known-value register mismatch");
  }
}

static void test_zero_registers_decode_to_zero_mhz() {
  float mhz = cc1101math::registersToMHz(0, 0, 0);
  CHECK(mhz == 0.0f, "zero registers should decode to 0 MHz");
}

int main() {
  test_roundtrip();
  test_known_values();
  test_zero_registers_decode_to_zero_mhz();

  if (failures == 0) {
    std::printf("cc1101_math: all tests passed\n");
    return 0;
  }
  std::printf("cc1101_math: %d test(s) failed\n", failures);
  return 1;
}
