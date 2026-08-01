#pragma once
// Pure frequency <-> register math for the CC1101 sub-GHz transceiver.
// No Arduino/hardware dependencies - compiles and runs on host for testing.
#include <cstdint>
#include <cmath>

namespace cc1101math {

// CC1101 crystal oscillator frequency (standard on nesten alle CC1101-moduler)
constexpr double F_XOSC_HZ = 26000000.0;
// FREQ-register er 24-bit: freq_reg = round(freq_hz * 2^16 / f_xosc)
constexpr double FREQ_MULT = 65536.0; // 2^16

// Regner ut FREQ2/FREQ1/FREQ0 fra ønsket frekvens i MHz.
inline void computeFreqRegisters(float mhz, uint8_t &freq2, uint8_t &freq1, uint8_t &freq0) {
  double freqHz = (double)mhz * 1.0e6;
  uint32_t freqReg = (uint32_t)llround(freqHz * FREQ_MULT / F_XOSC_HZ);

  freq2 = (uint8_t)((freqReg >> 16) & 0xFF);
  freq1 = (uint8_t)((freqReg >> 8) & 0xFF);
  freq0 = (uint8_t)(freqReg & 0xFF);
}

// Regner tilbake fra registerverdier til MHz (nyttig for verifisering/lesing).
inline float registersToMHz(uint8_t freq2, uint8_t freq1, uint8_t freq0) {
  uint32_t freqReg = ((uint32_t)freq2 << 16) | ((uint32_t)freq1 << 8) | freq0;
  double freqHz = (double)freqReg * F_XOSC_HZ / FREQ_MULT;
  return (float)(freqHz / 1.0e6);
}

} // namespace cc1101math
