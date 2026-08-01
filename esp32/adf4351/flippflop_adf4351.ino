#include <WiFi.h>
#include <WebServer.h>
#include <uri/UriBraces.h>
#include <SPI.h>
#include <math.h>

const char* AP_SSID = "ESP32-ADF4351";
const char* AP_PASSWORD = "88888888";

// --- Pinner ----------------------------------------------------------------
// ADF4351 har KUN 3-tråds skriv-grensesnitt: CLK, DATA (=MOSI), LE.
// MUXOUT/LD (lock detect) kobles til en digital inngang for statusavlesning.
constexpr uint8_t LED_PIN     = 8;
constexpr uint8_t SPI_SCK     = 4;   // -> CLK på ADF4351
constexpr uint8_t SPI_MOSI    = 6;   // -> DATA på ADF4351
constexpr uint8_t ADF_LE      = 7;   // -> LE  på ADF4351 (Load Enable)
constexpr uint8_t ADF_MUXOUT  = 10;  // <- MUXOUT/LD (lock detect, digital in)

constexpr float MIN_FREQUENCY_MHZ = 35.0f;
constexpr float MAX_FREQUENCY_MHZ = 4400.0f;
constexpr bool ENABLE_WIFI = true;

WebServer server(80);

// --- ADF4351 konfigurasjon ---------------------------------------------
struct Adf4351State {
  bool present = false;           // vi kan ikke "detektere" chipen over SPI (skriv-only),
                                   // så dette settes true etter vellykket INIT-sekvens
  float frequencyMHz = 433.92f;
  double refFrequencyHz = 25000000.0; // din modul: 25 MHz aktiv krystall
  uint16_t rCounter = 1;
  bool refDoubler = false;
  bool refDiv2 = false;
  bool rfOutEnabled = true;
  uint8_t outputPowerIdx = 3;     // 0=-4dBm,1=-1dBm,2=+2dBm,3=+5dBm (maks)
  bool locked = false;
  char lastAction[48] = "idle";

  // Sweep-parametre. NB: ADF4351 er en SENDER (signalgenerator), ikke en
  // mottaker som CC1101 - den kan ikke "finne" et eksternt signal. Sweep her
  // steg-vis tuner gjennom et bånd og logger om PLL faktisk låser på hvert
  // steg. Nyttig for å verifisere tuning-rekkevidde/registerkorrekthet, ikke
  // for spektrumskanning.
  float sweepStartMHz = 400.0f;
  float sweepStopMHz = 1000.0f;
  float sweepStepMHz = 10.0f;
  uint16_t sweepDwellMs = 15;
  uint16_t sweepPointsTotal = 0;
  uint16_t sweepPointsLocked = 0;
  bool sweepRunning = false;
};

Adf4351State pll;

// De 6 registrene som faktisk sendes til chipen (bygges på nytt for hver TUNE)
uint32_t reg[6] = {0, 0, 0, 0, 0, 0};

// --- Frekvensberegning -------------------------------------------------
// Velger RF-divider (og dermed VCO-frekvens i gyldig område 2.2-4.4 GHz)
uint8_t selectRfDivider(float freqMHz, uint32_t &vcoFreqHzOut) {
  uint8_t divSelect = 0; // 0=/1,1=/2,2=/4,3=/8,4=/16,5=/32,6=/64
  double vco = (double)freqMHz * 1.0e6;
  while (vco < 2.2e9 && divSelect < 6) {
    vco *= 2.0;
    divSelect++;
  }
  vcoFreqHzOut = (uint32_t)llround(vco);
  return divSelect;
}

// Beregner INT, FRAC, MOD og RF-divider for ønsket frekvens.
// MOD settes til et fast, praktisk tall (4000) med god oppløsning (~ få kHz steg ved 25 MHz PFD).
bool cc4351ComputeParams(float freqMHz, uint16_t &intVal, uint16_t &fracVal, uint16_t &modVal, uint8_t &rfDivSelect) {
  uint32_t vcoHz;
  rfDivSelect = selectRfDivider(freqMHz, vcoHz);

  double pfdHz = pll.refFrequencyHz * (pll.refDoubler ? 2.0 : 1.0) / (double)pll.rCounter / (pll.refDiv2 ? 2.0 : 1.0);
  if (pfdHz <= 0) return false;

  modVal = 4000; // fast modulus -> kanalavstand = pfdHz / 4000 (typisk noen kHz)

  double nTotal = (double)vcoHz / pfdHz;
  uint32_t nInt = (uint32_t)nTotal;
  double remainder = nTotal - (double)nInt;
  uint32_t frac = (uint32_t)llround(remainder * modVal);

  if (frac >= modVal) { // avrunding kan i sjeldne tilfeller rulle over
    frac = 0;
    nInt += 1;
  }

  // Prescaler 4/5 krever N >= 23. Vi bruker alltid 4/5 (dekker det meste av båndet greit).
  if (nInt < 23 || nInt > 65535) return false;

  intVal = (uint16_t)nInt;
  fracVal = (uint16_t)frac;
  return true;
}

// --- Register-bygging (iht. ADF4351 datablad, Rev. A) -------------------
void adf4351BuildRegisters(uint16_t intVal, uint16_t fracVal, uint16_t modVal, uint8_t rfDivSelect) {
  // Reg0: INT[30:15], FRAC[14:3], adresse[2:0]=0
  reg[0] = ((uint32_t)intVal << 15) | ((uint32_t)fracVal << 3) | 0x0;

  // Reg1: prescaler[27]=0 (4/5), phase[26:15]=1, MOD[14:3], adresse[2:0]=1
  reg[1] = (0UL << 27) | (1UL << 15) | ((uint32_t)modVal << 3) | 0x1;

  // Reg2: MUXOUT=digital lock detect(110), R-counter[23:14], double-buffer,
  //       charge pump current ~2.5mA (idx2), LDF=frac-N, LDP=10ns, adresse=2
  uint32_t muxout = 0b110;
  reg[2] = (muxout << 26)
          | ((uint32_t)pll.rCounter << 14)
          | (2UL << 9)   // charge pump current setting
          | (0UL << 8)   // LDF = fractional-N
          | (0UL << 7)   // LDP = 10 ns
          | (0UL << 6)   // PD polarity positive
          | 0x2;

  // Reg3: band select clock mode = 1 (raskere lock, anbefalt for ADF4351), adresse=3
  reg[3] = (1UL << 23) | 0x3;

  // Reg4: RF-divider select[22:20], band select clock divider[19:12],
  //       VCO power on, RF output A enable + effekt, adresse=4
  uint32_t bandSelClkDiv = (uint32_t)ceil((pll.refFrequencyHz / (double)pll.rCounter) / 500000.0);
  if (bandSelClkDiv < 1) bandSelClkDiv = 1;
  if (bandSelClkDiv > 255) bandSelClkDiv = 255;

  reg[4] = ((uint32_t)rfDivSelect << 20)
          | (bandSelClkDiv << 12)
          | (1UL << 11)  // VCO power on
          | ((pll.rfOutEnabled ? 1UL : 0UL) << 5)  // RF output A enable
          | ((uint32_t)pll.outputPowerIdx << 3)     // output power A
          | 0x4;

  // Reg5: LD pin mode = digital lock detect (must be 0b11 per anbefaling), adresse=5
  reg[5] = (0b11UL << 22) | 0x5;
}

// --- SPI-skriving --------------------------------------------------------
// ADF4351 vil ha registrene skrevet MSB-først, R5 -> R0 rekkefølge ved
// full re-programmering (viktig, ellers oppstår ikke lås korrekt).
void adf4351WriteRegister(uint32_t value) {
  digitalWrite(ADF_LE, LOW);
  SPI.transfer((value >> 24) & 0xFF);
  SPI.transfer((value >> 16) & 0xFF);
  SPI.transfer((value >> 8) & 0xFF);
  SPI.transfer(value & 0xFF);
  digitalWrite(ADF_LE, HIGH);
  delayMicroseconds(5);
  digitalWrite(ADF_LE, LOW);
}

void adf4351WriteAllRegisters() {
  for (int i = 5; i >= 0; --i) {
    adf4351WriteRegister(reg[i]);
  }
}

bool adf4351ReadLockDetect() {
  // MUXOUT går høy når PLL har låst (med reg2 muxout=digital lock detect)
  return digitalRead(ADF_MUXOUT) == HIGH;
}

// --- Høynivå-funksjoner ----------------------------------------------------
bool adf4351Init() {
  pinMode(ADF_LE, OUTPUT);
  pinMode(ADF_MUXOUT, INPUT);
  digitalWrite(ADF_LE, LOW);

  SPI.begin(SPI_SCK, -1 /* MISO ikke i bruk */, SPI_MOSI, ADF_LE);

  bool ok = adf4351SetFrequency(pll.frequencyMHz);
  pll.present = ok;
  strncpy(pll.lastAction, ok ? "init ok" : "init feilet (frekvens ugyldig)", sizeof(pll.lastAction) - 1);
  return ok;
}

bool adf4351SetFrequency(float mhz) {
  if (mhz < MIN_FREQUENCY_MHZ || mhz > MAX_FREQUENCY_MHZ) {
    return false;
  }

  uint16_t intVal, fracVal, modVal;
  uint8_t rfDivSelect;
  if (!cc4351ComputeParams(mhz, intVal, fracVal, modVal, rfDivSelect)) {
    snprintf(pll.lastAction, sizeof(pll.lastAction), "tune feilet %.2f MHz", mhz);
    return false;
  }

  adf4351BuildRegisters(intVal, fracVal, modVal, rfDivSelect);
  adf4351WriteAllRegisters();

  pll.frequencyMHz = mhz;
  delay(10); // gi PLL litt tid til å låse før vi sjekker MUXOUT
  pll.locked = adf4351ReadLockDetect();
  snprintf(pll.lastAction, sizeof(pll.lastAction), "tuned %.2f MHz (%s)", mhz, pll.locked ? "locked" : "unlocked");
  return true;
}

void adf4351SetRfOutput(bool enabled) {
  pll.rfOutEnabled = enabled;
  adf4351SetFrequency(pll.frequencyMHz); // reprogrammer med ny output-enable
  snprintf(pll.lastAction, sizeof(pll.lastAction), enabled ? "RF output ON" : "RF output OFF");
}

void adf4351SetPower(uint8_t idx) {
  if (idx > 3) idx = 3;
  pll.outputPowerIdx = idx;
  adf4351SetFrequency(pll.frequencyMHz);
  snprintf(pll.lastAction, sizeof(pll.lastAction), "power idx %u", idx);
}

void adf4351Sweep(float startMHz, float stopMHz, float stepMHz, uint16_t dwellMs) {
  if (startMHz > stopMHz || stepMHz <= 0.0f) {
    snprintf(pll.lastAction, sizeof(pll.lastAction), "sweep ERROR: start>stop/step<=0");
    return;
  }
  if (startMHz < MIN_FREQUENCY_MHZ) startMHz = MIN_FREQUENCY_MHZ;
  if (stopMHz > MAX_FREQUENCY_MHZ) stopMHz = MAX_FREQUENCY_MHZ;

  uint32_t estimatedPoints = (uint32_t)((stopMHz - startMHz) / stepMHz) + 1;
  if (estimatedPoints > 2000) {
    snprintf(pll.lastAction, sizeof(pll.lastAction), "sweep ERROR: for mange steg (%lu)", (unsigned long)estimatedPoints);
    return;
  }

  pll.sweepRunning = true;
  pll.sweepPointsTotal = 0;
  pll.sweepPointsLocked = 0;

  for (float freq = startMHz; freq <= stopMHz; freq += stepMHz) {
    bool ok = adf4351SetFrequency(freq);
    pll.sweepPointsTotal++;
    if (ok && pll.locked) {
      pll.sweepPointsLocked++;
    }
    delay(dwellMs);

    // Gi WiFi-stacken litt tid ved lange sweeps, ellers kan watchdog trigge
    if (pll.sweepPointsTotal % 20 == 0) {
      yield();
    }
  }

  pll.sweepRunning = false;
  pll.sweepStartMHz = startMHz;
  pll.sweepStopMHz = stopMHz;
  pll.sweepStepMHz = stepMHz;
  snprintf(pll.lastAction, sizeof(pll.lastAction), "sweep ferdig: %u/%u locked",
           pll.sweepPointsLocked, pll.sweepPointsTotal);
}

// --- Seriekommandoer -------------------------------------------------------
void processSerialCommand(String command) {
  command.trim();
  command.toUpperCase();

  int colonIndex = command.indexOf(':');
  String cmd = (colonIndex > 0) ? command.substring(0, colonIndex) : command;
  String args = (colonIndex > 0) ? command.substring(colonIndex + 1) : "";

  Serial.print("> ");
  Serial.println(cmd);

  if (cmd == "INIT") {
    bool ok = adf4351Init();
    Serial.println(ok ? "OK:ADF4351_INITIALIZED" : "ERROR:ADF4351_INIT_FAILED");
  }
  else if (cmd == "TUNE") {
    float freq = args.toFloat();
    if (!adf4351SetFrequency(freq)) {
      Serial.print("ERROR:TUNE_FAILED:");
      Serial.println(freq);
    } else {
      Serial.print("OK:TUNED:");
      Serial.print(freq);
      Serial.print(":LOCKED:");
      Serial.println(pll.locked ? "YES" : "NO");
    }
  }
  else if (cmd == "RFOUT") {
    if (args == "ON") { adf4351SetRfOutput(true); Serial.println("OK:RFOUT_ON"); }
    else if (args == "OFF") { adf4351SetRfOutput(false); Serial.println("OK:RFOUT_OFF"); }
    else Serial.println("ERROR:INVALID_RFOUT_STATE");
  }
  else if (cmd == "POWER") {
    adf4351SetPower((uint8_t)args.toInt());
    Serial.print("OK:POWER:");
    Serial.println(pll.outputPowerIdx);
  }
  else if (cmd == "SWEEP") {
    // Format: SWEEP:start:stop:step  (MHz). Eksempel: SWEEP:400:1000:10
    float start = pll.sweepStartMHz, stop = pll.sweepStopMHz, step = pll.sweepStepMHz;
    int firstColon = args.indexOf(':');
    int secondColon = args.indexOf(':', firstColon + 1);
    if (firstColon > 0 && secondColon > firstColon) {
      start = args.substring(0, firstColon).toFloat();
      stop = args.substring(firstColon + 1, secondColon).toFloat();
      step = args.substring(secondColon + 1).toFloat();
    }
    adf4351Sweep(start, stop, step, pll.sweepDwellMs);
    Serial.print("OK:SWEEP_DONE:");
    Serial.print(pll.sweepPointsLocked);
    Serial.print("/");
    Serial.println(pll.sweepPointsTotal);
  }
  else if (cmd == "LOCK") {
    pll.locked = adf4351ReadLockDetect();
    Serial.print("OK:LOCKED:");
    Serial.println(pll.locked ? "YES" : "NO");
  }
  else if (cmd == "STATUS") {
    Serial.print("PRESENT:");
    Serial.print(pll.present ? "YES" : "NO");
    Serial.print("|FREQ:");
    Serial.print(pll.frequencyMHz);
    Serial.print("|LOCKED:");
    Serial.print(pll.locked ? "YES" : "NO");
    Serial.print("|RFOUT:");
    Serial.print(pll.rfOutEnabled ? "ON" : "OFF");
    Serial.print("|POWER:");
    Serial.print(pll.outputPowerIdx);
    Serial.print("|SWEEP:");
    Serial.print(pll.sweepPointsLocked);
    Serial.print("/");
    Serial.print(pll.sweepPointsTotal);
    Serial.print("|LAST:");
    Serial.println(pll.lastAction);
  }
  else if (cmd == "HELP") {
    Serial.println("=== ADF4351 Serial Commands ===");
    Serial.println("INIT - Initialiser PLL og skriv startfrekvens");
    Serial.println("TUNE:freq - Tune til frekvens (MHz, 35-4400)");
    Serial.println("RFOUT:ON|OFF - Slå RF-utgang av/pa");
    Serial.println("POWER:0-3 - Sett utgangseffekt (0=-4dBm ... 3=+5dBm)");
    Serial.println("SWEEP:start:stop:step - Sweep gjennom band, sjekker lock per steg");
    Serial.println("LOCK - Les lock-detect status (MUXOUT)");
    Serial.println("STATUS - Vis naavaerende status");
    Serial.println("HELP - Vis denne meldingen");
  }
  else {
    Serial.println("ERROR:UNKNOWN_COMMAND");
    Serial.println("Type HELP for kommandoliste");
  }
}

// --- Web-grensesnitt ---------------------------------------------------
String buildHtml() {
  String html = R"HTML(
<!DOCTYPE html>
<html>
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1" />
  <title>ESP32 ADF4351 Controller</title>
  <style>
    body { font-family: Arial, sans-serif; background: #0f172a; color: #e2e8f0; margin: 0; padding: 20px; }
    .card { background: #111827; padding: 16px; border-radius: 12px; margin: 12px auto; max-width: 520px; }
    button { padding: 10px 14px; border-radius: 8px; border: none; margin: 4px; cursor: pointer; }
    .green { background: #16a34a; color: white; }
    .blue { background: #2563eb; color: white; }
    .red { background: #dc2626; color: white; }
    .gray { background: #475569; color: white; }
    input { padding: 8px; border-radius: 8px; border: 1px solid #64748b; color: #0f172a; }
  </style>
</head>
<body>
  <div class="card">
    <h2>ESP32 ADF4351 Wi-Fi Control</h2>
  </div>
  <div class="card">
    <h3>Frekvens</h3>
    <form action="/rf/tune/" method="get" style="margin-bottom: 8px;">
      <input name="freq" type="number" step="0.01" min="35" max="4400" placeholder="Frekvens MHz" style="width: 180px;" />
      <button type="submit" class="blue" style="padding: 8px 12px;">Tune</button>
    </form>
    <a href="/rf/tune/433.92"><button class="blue">433.92 MHz</button></a>
    <a href="/rf/tune/868.00"><button class="blue">868.00 MHz</button></a>
    <a href="/rf/tune/1000.00"><button class="blue">1000 MHz</button></a>
  </div>
  <div class="card">
    <h3>Utgang</h3>
    <a href="/rf/rfout/on"><button class="green">RF Output ON</button></a>
    <a href="/rf/rfout/off"><button class="red">RF Output OFF</button></a>
    <form action="/rf/power" method="get" style="margin-top: 8px;">
      <input name="idx" type="number" min="0" max="3" value="3" style="width: 100px;" />
      <button type="submit" class="gray" style="padding: 8px 12px;">Sett effekt (0-3)</button>
    </form>
  </div>
  <div class="card">
    <h3>Status</h3>
    <p>Frekvens: <strong>)HTML";
  html += String(pll.frequencyMHz, 2);
  html += R"HTML( MHz</strong></p>
    <p>Lock: <strong>)HTML";
  html += pll.locked ? "LOCKED" : "UNLOCKED";
  html += R"HTML(</strong></p>
    <p>RF Output: <strong>)HTML";
  html += pll.rfOutEnabled ? "ON" : "OFF";
  html += R"HTML(</strong></p>
    <p>Effekt-indeks: <strong>)HTML";
  html += String(pll.outputPowerIdx);
  html += R"HTML(</strong></p>
    <p>Siste handling: <strong>)HTML";
  html += String(pll.lastAction);
  html += R"HTML(</strong></p>
  </div>
</body>
</html>
)HTML";
  return html;
}

void handleRoot() {
  server.send(200, "text/html", buildHtml());
}

void handleTune() {
  String freqValue = server.pathArg(0);
  if (freqValue.length() == 0) {
    freqValue = server.arg("freq");
  }

  float freq = freqValue.toFloat();
  if (!adf4351SetFrequency(freq)) {
    String payload = "{\"ok\":false,\"error\":\"Frequency must be " + String(MIN_FREQUENCY_MHZ, 0) + "-" + String(MAX_FREQUENCY_MHZ, 0) + " MHz\"}";
    server.send(400, "application/json", payload);
    return;
  }

  String payload = "{\"ok\":true,\"frequency\":" + String(freq, 2) + ",\"locked\":" + (pll.locked ? "true" : "false") + "}";
  server.send(200, "application/json", payload);
}

void handleRfOutOn() {
  adf4351SetRfOutput(true);
  server.send(200, "application/json", "{\"rfout\":true}");
}

void handleRfOutOff() {
  adf4351SetRfOutput(false);
  server.send(200, "application/json", "{\"rfout\":false}");
}

void handlePower() {
  uint8_t idx = (uint8_t)server.arg("idx").toInt();
  adf4351SetPower(idx);
  String payload = "{\"power\":" + String(pll.outputPowerIdx) + "}";
  server.send(200, "application/json", payload);
}

void handleSweep() {
  float start = server.hasArg("start") ? server.arg("start").toFloat() : pll.sweepStartMHz;
  float stop = server.hasArg("stop") ? server.arg("stop").toFloat() : pll.sweepStopMHz;
  float step = server.hasArg("step") ? server.arg("step").toFloat() : pll.sweepStepMHz;

  adf4351Sweep(start, stop, step, pll.sweepDwellMs);

  String payload = "{\"pointsTotal\":" + String(pll.sweepPointsTotal) +
                    ",\"pointsLocked\":" + String(pll.sweepPointsLocked) +
                    ",\"lastFreq\":" + String(pll.frequencyMHz, 2) + "}";
  server.send(200, "application/json", payload);
}

void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println("\n\n==============================================");
  Serial.println("       FLIPPFLOP - ADF4351 RF Controller");
  Serial.println("==============================================");

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  bool pllOk = adf4351Init();
  if (pllOk) {
    Serial.println("[SUCCESS] ADF4351 programmert");
  } else {
    Serial.println("[WARNING] ADF4351 init feilet");
  }

  if (ENABLE_WIFI) {
    Serial.println("[INFO] Starter WiFi Access Point...");
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASSWORD);
    IPAddress ip = WiFi.softAPIP();

    Serial.print("[INFO] AP SSID: ");
    Serial.println(AP_SSID);
    Serial.print("[INFO] AP IP: ");
    Serial.println(ip);

    server.on("/", HTTP_GET, handleRoot);
    server.on("/rf/tune/", HTTP_GET, handleTune);
    server.on(UriBraces("/rf/tune/{}"), HTTP_GET, handleTune);
    server.on("/rf/rfout/on", HTTP_GET, handleRfOutOn);
    server.on("/rf/rfout/off", HTTP_GET, handleRfOutOff);
    server.on("/rf/power", HTTP_GET, handlePower);

    server.begin();
    Serial.println("[SUCCESS] Web-server startet");
  } else {
    Serial.println("[INFO] WiFi deaktivert - venter pa seriekommandoer");
    Serial.println("[INFO] Skriv 'HELP' for kommandoliste");
  }

  Serial.println("==============================================\n");
}

void loop() {
  if (ENABLE_WIFI) {
    server.handleClient();
  }

  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    if (command.length() > 0) {
      processSerialCommand(command);
    }
  }
}
