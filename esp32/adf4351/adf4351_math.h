#pragma once
// Pure ADF4351 PLL register math (INT/FRAC/MOD/R-divider + register
// bit-packing). No Arduino/hardware dependencies - compiles and runs on
// host for testing.
#include <cstdint>
#include <cmath>

namespace adf4351math {

struct RefConfig {
  double refFrequencyHz;
  uint16_t rCounter;
  bool refDoubler;
  bool refDiv2;
};

struct Params {
  bool ok = false;
  uint16_t intVal = 0;
  uint16_t fracVal = 0;
  uint16_t modVal = 0;
  uint8_t rfDivSelect = 0;
};

// Velger RF-divider (og dermed VCO-frekvens i gyldig område 2.2-4.4 GHz).
// vcoFreqHzOut er uint64_t fordi VCO-frekvensen kan naa 4.4 GHz, som ikke
// faar plass i en uint32_t (maks ~4.295 GHz).
inline uint8_t selectRfDivider(float freqMHz, uint64_t &vcoFreqHzOut) {
  uint8_t divSelect = 0; // 0=/1,1=/2,2=/4,3=/8,4=/16,5=/32,6=/64
  double vco = (double)freqMHz * 1.0e6;
  while (vco < 2.2e9 && divSelect < 6) {
    vco *= 2.0;
    divSelect++;
  }
  vcoFreqHzOut = (uint64_t)llround(vco);
  return divSelect;
}

// Beregner INT, FRAC, MOD og RF-divider for ønsket frekvens.
// MOD settes til et fast, praktisk tall (4000) med god oppløsning.
inline Params computeParams(float freqMHz, const RefConfig &ref) {
  Params p;
  uint64_t vcoHz;
  p.rfDivSelect = selectRfDivider(freqMHz, vcoHz);

  double pfdHz = ref.refFrequencyHz * (ref.refDoubler ? 2.0 : 1.0)
               / (double)ref.rCounter / (ref.refDiv2 ? 2.0 : 1.0);
  if (pfdHz <= 0) {
    return p; // p.ok stays false
  }

  p.modVal = 4000;

  double nTotal = (double)vcoHz / pfdHz;
  uint32_t nInt = (uint32_t)nTotal;
  double remainder = nTotal - (double)nInt;
  uint32_t frac = (uint32_t)llround(remainder * p.modVal);

  if (frac >= p.modVal) { // avrunding kan i sjeldne tilfeller rulle over
    frac = 0;
    nInt += 1;
  }

  // Prescaler 4/5 krever N >= 23. Vi bruker alltid 4/5.
  if (nInt < 23 || nInt > 65535) {
    return p; // p.ok stays false
  }

  p.intVal = (uint16_t)nInt;
  p.fracVal = (uint16_t)frac;
  p.ok = true;
  return p;
}

// Bygger de 6 registrene som faktisk sendes til chipen (iht. ADF4351
// datablad, Rev. A).
inline void buildRegisters(uint16_t intVal, uint16_t fracVal, uint16_t modVal,
                            uint8_t rfDivSelect, uint16_t rCounter,
                            bool rfOutEnabled, uint8_t outputPowerIdx,
                            double refFrequencyHz, uint32_t reg[6]) {
  // Reg0: INT[30:15], FRAC[14:3], adresse[2:0]=0
  reg[0] = ((uint32_t)intVal << 15) | ((uint32_t)fracVal << 3) | 0x0;

  // Reg1: prescaler[27]=0 (4/5), phase[26:15]=1, MOD[14:3], adresse[2:0]=1
  reg[1] = (0UL << 27) | (1UL << 15) | ((uint32_t)modVal << 3) | 0x1;

  // Reg2: MUXOUT=digital lock detect(110), R-counter[23:14], double-buffer,
  //       charge pump current ~2.5mA (idx2), LDF=frac-N, LDP=10ns, adresse=2
  uint32_t muxout = 0b110;
  reg[2] = (muxout << 26)
          | ((uint32_t)rCounter << 14)
          | (2UL << 9)   // charge pump current setting
          | (0UL << 8)   // LDF = fractional-N
          | (0UL << 7)   // LDP = 10 ns
          | (0UL << 6)   // PD polarity positive
          | 0x2;

  // Reg3: band select clock mode = 1 (raskere lock, anbefalt for ADF4351), adresse=3
  reg[3] = (1UL << 23) | 0x3;

  // Reg4: RF-divider select[22:20], band select clock divider[19:12],
  //       VCO power on, RF output A enable + effekt, adresse=4
  uint32_t bandSelClkDiv = (uint32_t)ceil((refFrequencyHz / (double)rCounter) / 500000.0);
  if (bandSelClkDiv < 1) bandSelClkDiv = 1;
  if (bandSelClkDiv > 255) bandSelClkDiv = 255;

  reg[4] = ((uint32_t)rfDivSelect << 20)
          | (bandSelClkDiv << 12)
          | (1UL << 11)  // VCO power on
          | ((rfOutEnabled ? 1UL : 0UL) << 5)  // RF output A enable
          | ((uint32_t)outputPowerIdx << 3)     // output power A
          | 0x4;

  // Reg5: LD pin mode = digital lock detect (must be 0b11 per anbefaling), adresse=5
  reg[5] = (0b11UL << 22) | 0x5;
}

} // namespace adf4351math
