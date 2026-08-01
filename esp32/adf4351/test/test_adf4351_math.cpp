// Host-side tests for adf4351_math.h - no Arduino toolchain needed.
// Build & run: g++ -std=c++17 -Wall -Wextra -I.. test_adf4351_math.cpp -o /tmp/test_adf4351_math && /tmp/test_adf4351_math
#include "../adf4351_math.h"
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

static const adf4351math::RefConfig STANDARD_REF{25000000.0, 1, false, false};

// Kjente INT/FRAC/MOD-verdier, uavhengig beregnet (Python, double presisjon)
// for standard 25 MHz referanse, R=1, ingen doubler/div2.
static void test_known_values() {
  struct Case { float mhz; uint8_t rfDivSelect; uint16_t intVal; uint16_t fracVal; };
  const Case cases[] = {
      {433.92f, 3, 138, 3418},
      {900.0f,  2, 144, 0},
      {2400.0f, 0, 96,  0},
      {35.0f,   6, 89,  2400},
      {4400.0f, 0, 176, 0},
      {137.5f,  4, 88,  0},
  };
  for (const auto &c : cases) {
    adf4351math::Params p = adf4351math::computeParams(c.mhz, STANDARD_REF);
    CHECK(p.ok, "expected valid params for known-value case");
    CHECK(p.rfDivSelect == c.rfDivSelect, "rfDivSelect mismatch");
    CHECK(p.intVal == c.intVal, "intVal mismatch");
    CHECK(p.fracVal == c.fracVal, "fracVal mismatch");
    CHECK(p.modVal == 4000, "modVal should always be the fixed 4000");
  }
}

// FRAC skal aldri rulle over/lik MOD - hvis avrunding ville gitt det,
// skal koden bære overflowet inn i INT i stedet (frac=0, nInt+1).
static void test_frac_never_reaches_mod() {
  for (float mhz = 35.0f; mhz <= 4400.0f; mhz += 3.7f) {
    adf4351math::Params p = adf4351math::computeParams(mhz, STANDARD_REF);
    if (p.ok) {
      CHECK(p.fracVal < p.modVal, "fracVal must stay below modVal");
    }
  }
}

// N utenfor det gyldige prescaler-området (23..65535) skal avvises.
static void test_rejects_out_of_range_n() {
  // For lav PFD (stor R-counter) -> N blir for stort.
  adf4351math::RefConfig hugeRDivider{25000000.0, 1000, false, false};
  adf4351math::Params tooLarge = adf4351math::computeParams(4400.0f, hugeRDivider);
  CHECK(!tooLarge.ok, "N above 65535 should be rejected");

  // Svaert hoy referanse -> PFD stor nok til at N blir for lite.
  adf4351math::RefConfig hugeRef{100000000.0, 1, false, false};
  adf4351math::Params tooSmall = adf4351math::computeParams(35.0f, hugeRef);
  CHECK(!tooSmall.ok, "N below 23 should be rejected");
}

// selectRfDivider skal alltid loefte VCO til 2.2-4.4 GHz naar det er mulig,
// og stoppe ved /64 (divSelect=6) selv om det ikke er nok for veldig lave frekvenser.
static void test_select_rf_divider_bounds() {
  uint64_t vcoHz;
  uint8_t div = adf4351math::selectRfDivider(2400.0f, vcoHz);
  CHECK(div == 0, "2400 MHz is already in VCO range, no divider needed");
  CHECK(vcoHz == 2400000000ull, "VCO frequency should equal input when no doubling needed");

  div = adf4351math::selectRfDivider(137.5f, vcoHz);
  CHECK(div == 4, "137.5 MHz needs /16 to reach VCO range");
  CHECK(vcoHz >= 2200000000ull && vcoHz <= 4400000000ull, "VCO must land in 2.2-4.4 GHz range");
}

// Regresjonstest for en tidligere overflow-bug: VCO-frekvensen kan naa
// 4.4 GHz, som IKKE faar plass i en uint32_t (maks ~4.295 GHz). Med feil
// type ville dette rullet over og gitt feil INT/FRAC ved baandets topp.
static void test_max_frequency_does_not_overflow() {
  uint64_t vcoHz;
  adf4351math::selectRfDivider(4400.0f, vcoHz);
  CHECK(vcoHz == 4400000000ull, "VCO frequency at 4400 MHz must not overflow");

  adf4351math::Params p = adf4351math::computeParams(4400.0f, STANDARD_REF);
  CHECK(p.ok, "4400 MHz (max supported frequency) must produce valid params");
  CHECK(p.intVal == 176, "4400 MHz intVal mismatch");
}

// buildRegisters() er ren bit-pakking - sjekk adresse-bitene (de 3 laveste
// bitene identifiserer registeret) og noen kjente felt eksplisitt.
static void test_build_registers_addresses_and_fields() {
  uint32_t reg[6];
  adf4351math::buildRegisters(/*intVal=*/138, /*fracVal=*/3418, /*modVal=*/4000,
                               /*rfDivSelect=*/3, /*rCounter=*/1,
                               /*rfOutEnabled=*/true, /*outputPowerIdx=*/3,
                               /*refFrequencyHz=*/25000000.0, reg);

  for (int i = 0; i < 6; ++i) {
    CHECK((reg[i] & 0x7) == (uint32_t)i, "register address bits must match index");
  }

  CHECK(((reg[0] >> 15) & 0x7FFF) == 138, "reg0 INT field mismatch");
  CHECK(((reg[0] >> 3) & 0xFFF) == 3418, "reg0 FRAC field mismatch");
  CHECK(((reg[1] >> 3) & 0xFFF) == 4000, "reg1 MOD field mismatch");
  CHECK(((reg[4] >> 20) & 0x7) == 3, "reg4 RF-divider-select field mismatch");
  CHECK(((reg[4] >> 5) & 0x1) == 1, "reg4 RF output enable bit mismatch (expected ON)");

  // RF output disabled -> bit 5 i reg4 skal vaere 0.
  adf4351math::buildRegisters(138, 3418, 4000, 3, 1, /*rfOutEnabled=*/false, 3, 25000000.0, reg);
  CHECK(((reg[4] >> 5) & 0x1) == 0, "reg4 RF output enable bit mismatch (expected OFF)");
}

int main() {
  test_known_values();
  test_frac_never_reaches_mod();
  test_rejects_out_of_range_n();
  test_select_rf_divider_bounds();
  test_max_frequency_does_not_overflow();
  test_build_registers_addresses_and_fields();

  if (failures == 0) {
    std::printf("adf4351_math: all tests passed\n");
    return 0;
  }
  std::printf("adf4351_math: %d test(s) failed\n", failures);
  return 1;
}
