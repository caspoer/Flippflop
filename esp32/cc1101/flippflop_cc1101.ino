#include <WiFi.h>
#include <WebServer.h>
#include <uri/UriBraces.h>
#include <SPI.h>
#include "cc1101_math.h"

const char* AP_SSID = "ESP32-C3-CC1101";
const char* AP_PASSWORD = "88888888";

constexpr uint8_t LED_PIN = 8;
constexpr uint8_t SPI_SCK = 4;
constexpr uint8_t SPI_MOSI = 6;
constexpr uint8_t SPI_MISO = 5;
constexpr uint8_t CC1101_CS = 7;
constexpr uint8_t CC1101_GDO0 = 10;
constexpr float MIN_FREQUENCY_MHZ = 300.0f;
constexpr float MAX_FREQUENCY_MHZ = 6000.0f;

constexpr uint8_t CC1101_REG_PARTNUM = 0x30;
constexpr uint8_t CC1101_REG_FREQ0 = 0x0D;
constexpr uint8_t CC1101_REG_FREQ1 = 0x0E;
constexpr uint8_t CC1101_REG_FREQ2 = 0x0F;
constexpr uint8_t CC1101_READ_FLAG = 0x80;
constexpr uint16_t MAX_SWEEP_ITERATIONS = 10000;
constexpr float MIN_SWEEP_STEP = 0.01f;
constexpr bool ENABLE_WIFI = true;

WebServer server(80);

struct Cc1101State {
  bool present = false;
  float frequencyMHz = 433.92f;
  int8_t rssi = -128;
  uint32_t packetsReceived = 0;
  uint32_t packetsTransmitted = 0;
  char lastPacket[48] = "idle";
  bool monitorMode = false;
  bool carrierOn = false;
  bool jamActive = false;
  uint16_t jamCount = 0;
  uint16_t jamSent = 0;
  float sweepStartMHz = MIN_FREQUENCY_MHZ;
  float sweepStopMHz = MAX_FREQUENCY_MHZ;
  float sweepStepMHz = 1.0f;
  float sweepBestMHz = 0.0f;
  int8_t sweepBestRssi = -128;
};

Cc1101State radio;

// --- FREQ-beregning -------------------------------------------------------
// Ren matte er flyttet til cc1101_math.h (host-testbar, ingen Arduino-avhengigheter).
void cc1101ComputeFreqRegisters(float mhz, uint8_t &freq2, uint8_t &freq1, uint8_t &freq0) {
  cc1101math::computeFreqRegisters(mhz, freq2, freq1, freq0);
}

float cc1101RegistersToMHz(uint8_t freq2, uint8_t freq1, uint8_t freq0) {
  return cc1101math::registersToMHz(freq2, freq1, freq0);
}

void processSerialCommand(String command) {
  command.trim();
  command.toUpperCase();
  
  int colonIndex = command.indexOf(':');
  String cmd = (colonIndex > 0) ? command.substring(0, colonIndex) : command;
  String args = (colonIndex > 0) ? command.substring(colonIndex + 1) : "";
  
  Serial.print("> ");
  Serial.println(cmd);
  
  if (cmd == "INIT") {
    bool ok = cc1101Init();
    Serial.println(ok ? "OK:CC1101_INITIALIZED" : "ERROR:CC1101_NOT_DETECTED");
  }
  else if (cmd == "TUNE") {
    float freq = args.toFloat();
    if (freq < MIN_FREQUENCY_MHZ || freq > MAX_FREQUENCY_MHZ) {
      Serial.print("ERROR:FREQUENCY_OUT_OF_RANGE:");
      Serial.print(MIN_FREQUENCY_MHZ);
      Serial.print("-");
      Serial.println(MAX_FREQUENCY_MHZ);
    } else {
      cc1101SetFrequency(freq);
      Serial.print("OK:TUNED:");
      Serial.println(freq);
    }
  }
  else if (cmd == "SEND") {
    cc1101SendPing();
    Serial.println("OK:PING_SENT");
  }
  else if (cmd == "READ") {
    cc1101ReadPacket();
    Serial.println("OK:PACKET_READ");
  }
  else if (cmd == "RSSI") {
    int8_t rssi = cc1101ReadRssi();
    Serial.print("OK:RSSI:");
    Serial.println(rssi);
  }
  else if (cmd == "SWEEP") {
    cc1101SweepFrequencies();
    Serial.print("OK:SWEEP_BEST:");
    Serial.print(radio.sweepBestMHz);
    Serial.print(":");
    Serial.println(radio.sweepBestRssi);
  }
  else if (cmd == "CARRIER") {
    if (args == "ON") {
      cc1101StartCarrier();
      Serial.println("OK:CARRIER_ON");
    } else if (args == "OFF") {
      cc1101StopCarrier();
      Serial.println("OK:CARRIER_OFF");
    } else {
      Serial.println("ERROR:INVALID_CARRIER_STATE");
    }
  }
  else if (cmd == "JAM") {
    uint16_t count = args.toInt();
    if (count < 1 || count > 20) count = 5;
    cc1101JamSignal(count);
    Serial.print("OK:JAM_SENT:");
    Serial.println(count);
  }
  else if (cmd == "MONITOR") {
    if (args == "ON") {
      radio.monitorMode = true;
      Serial.println("OK:MONITOR_ON");
    } else if (args == "OFF") {
      radio.monitorMode = false;
      Serial.println("OK:MONITOR_OFF");
    } else {
      Serial.println("ERROR:INVALID_MONITOR_STATE");
    }
  }
  else if (cmd == "STATUS") {
    Serial.print("PRESENT:");
    Serial.print(radio.present ? "YES" : "NO");
    Serial.print("|FREQ:");
    Serial.print(radio.frequencyMHz);
    Serial.print("|RSSI:");
    Serial.print(radio.rssi);
    Serial.print("|RX:");
    Serial.print(radio.packetsReceived);
    Serial.print("|TX:");
    Serial.print(radio.packetsTransmitted);
    Serial.print("|LAST:");
    Serial.println(radio.lastPacket);
  }
  else if (cmd == "HELP") {
    Serial.println("=== Flippflop Serial Commands ===");
    Serial.println("INIT - Initialize CC1101");
    Serial.println("TUNE:freq - Tune to frequency (MHz)");
    Serial.println("SEND - Send ping packet");
    Serial.println("READ - Read packet");
    Serial.println("RSSI - Read signal strength");
    Serial.println("SWEEP - Sweep band and find best signal");
    Serial.println("CARRIER:ON|OFF - Enable/disable carrier");
    Serial.println("JAM:count - Send jam signal (1-20)");
    Serial.println("MONITOR:ON|OFF - Enable/disable monitor mode");
    Serial.println("STATUS - Display current status");
    Serial.println("HELP - Show this help message");
  }
  else {
    Serial.println("ERROR:UNKNOWN_COMMAND");
    Serial.println("Type HELP for command list");
  }
}

void cc1101WriteRegister(uint8_t reg, uint8_t value) {
  digitalWrite(CC1101_CS, LOW);
  SPI.transfer(reg);
  SPI.transfer(value);
  digitalWrite(CC1101_CS, HIGH);
}

uint8_t cc1101ReadRegister(uint8_t reg) {
  digitalWrite(CC1101_CS, LOW);
  SPI.transfer(CC1101_READ_FLAG | reg);
  uint8_t value = SPI.transfer(0x00);
  digitalWrite(CC1101_CS, HIGH);
  return value;
}

bool cc1101Init() {
  pinMode(CC1101_CS, OUTPUT);
  pinMode(CC1101_GDO0, INPUT);
  digitalWrite(CC1101_CS, HIGH);
  
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI, CC1101_CS);
  delay(20);
  
  uint8_t partNumber = cc1101ReadRegister(CC1101_REG_PARTNUM);
  radio.present = (partNumber != 0xFF);
  
  if (radio.present) {
    // Sett faktiske FREQ-registre for standard 433.92 MHz istedenfor hardkodede byte
    cc1101SetFrequency(radio.frequencyMHz);
    radio.rssi = -55;
    strncpy(radio.lastPacket, "ready", sizeof(radio.lastPacket) - 1);
    radio.lastPacket[sizeof(radio.lastPacket) - 1] = '\0';
    return true;
  }
  
  return false;
}

void cc1101SetFrequency(float mhz) {
  radio.frequencyMHz = mhz;
  if (radio.present) {
    uint8_t freq2, freq1, freq0;
    cc1101ComputeFreqRegisters(mhz, freq2, freq1, freq0);
    cc1101WriteRegister(CC1101_REG_FREQ2, freq2);
    cc1101WriteRegister(CC1101_REG_FREQ1, freq1);
    cc1101WriteRegister(CC1101_REG_FREQ0, freq0);
    radio.rssi = -60;
    snprintf(radio.lastPacket, sizeof(radio.lastPacket), "tuned %.2f", mhz);
  }
}

int8_t cc1101ReadRssi() {
  if (!radio.present) {
    return -128;
  }
  
  radio.rssi = (int8_t)(-55 - (random(0, 20)));
  return radio.rssi;
}

void cc1101SendPing() {
  if (!radio.present) {
    return;
  }
  
  radio.packetsTransmitted += 1;
  radio.rssi = -42;
  snprintf(radio.lastPacket, sizeof(radio.lastPacket), "TX %.2f MHz", radio.frequencyMHz);
}

void cc1101SendPayload(const String& payload) {
  if (!radio.present) {
    return;
  }
  
  radio.packetsTransmitted += 1;
  radio.rssi = -44;
  snprintf(radio.lastPacket, sizeof(radio.lastPacket), "msg:%s", payload.c_str());
}

void cc1101ReadPacket() {
  if (!radio.present) {
    return;
  }
  
  radio.packetsReceived += 1;
  radio.rssi = -58;
  snprintf(radio.lastPacket, sizeof(radio.lastPacket), "RX %.2f MHz", radio.frequencyMHz);
}

void cc1101StartCarrier() {
  radio.carrierOn = true;
  snprintf(radio.lastPacket, sizeof(radio.lastPacket), "carrier ON %.2f", radio.frequencyMHz);
}

void cc1101StopCarrier() {
  radio.carrierOn = false;
  snprintf(radio.lastPacket, sizeof(radio.lastPacket), "carrier OFF");
}

void cc1101JamSignal(uint16_t count) {
  if (!radio.present || count == 0) {
    return;
  }
  
  radio.jamActive = true;
  radio.jamCount = count;
  radio.jamSent = 0;
  
  for (uint16_t i = 0; i < count; ++i) {
    radio.packetsTransmitted += 1;
    radio.jamSent += 1;
    radio.rssi = -40;
    delay(15);
  }
  
  radio.jamActive = false;
  snprintf(radio.lastPacket, sizeof(radio.lastPacket), "JAM %u @%.2f", count, radio.frequencyMHz);
}

void cc1101SweepFrequencies() {
  radio.sweepBestRssi = -128;
  radio.sweepBestMHz = 0.0f;
  
  if (radio.sweepStartMHz > radio.sweepStopMHz) {
    snprintf(radio.lastPacket, sizeof(radio.lastPacket), "sweep ERROR: start > stop");
    return;
  }
  
  if (radio.sweepStepMHz < MIN_SWEEP_STEP) {
    radio.sweepStepMHz = MIN_SWEEP_STEP;
  }
  
  uint16_t estimatedIterations = (uint16_t)((radio.sweepStopMHz - radio.sweepStartMHz) / radio.sweepStepMHz) + 1;
  if (estimatedIterations > MAX_SWEEP_ITERATIONS) {
    snprintf(radio.lastPacket, sizeof(radio.lastPacket), "sweep ERROR: too many iterations");
    return;
  }
  
  for (float freq = radio.sweepStartMHz; freq <= radio.sweepStopMHz; freq += radio.sweepStepMHz) {
    cc1101SetFrequency(freq);
    int8_t sampleRssi = cc1101ReadRssi();
    if (sampleRssi > radio.sweepBestRssi) {
      radio.sweepBestRssi = sampleRssi;
      radio.sweepBestMHz = freq;
    }
    delay(20);
  }
  
  snprintf(radio.lastPacket, sizeof(radio.lastPacket), "sweep best %.2f", radio.sweepBestMHz);
}

String buildHtml() {
  String html = R"HTML(
<!DOCTYPE html>
<html>
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width, initial-scale=1" />
  <title>ESP32-C3 CC1101 Controller</title>
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
    <h2>ESP32-C3 CC1101 Wi-Fi Control</h2>
    <p>LED: <strong>ON</strong> or <strong>OFF</strong></p>
    <a href="/led/on"><button class="green">LED ON</button></a>
    <a href="/led/off"><button class="red">LED OFF</button></a>
  </div>
  <div class="card">
    <h3>RF Controls</h3>
    <form action="/rf/tune/" method="get" style="margin-bottom: 8px;">
      <input name="freq" type="number" step="0.01" min="300" max="6000" placeholder="Frequency MHz" style="width: 180px;" />
      <button type="submit" class="blue" style="padding: 8px 12px;">Tune</button>
    </form>
    <a href="/rf/tune/433.92"><button class="blue">Tune 433.92 MHz</button></a>
    <a href="/rf/tune/868.00"><button class="blue">Tune 868.00 MHz</button></a>
    <a href="/rf/send"><button class="green">Send Ping</button></a>
    <a href="/rf/read"><button class="blue">Read Packet</button></a>
  </div>
  <div class="card">
    <h3>SDR Style Extras</h3>
    <a href="/rf/rssi"><button class="gray">Read RSSI</button></a>
    <a href="/rf/sweep"><button class="blue">Sweep Band</button></a>
    <a href="/rf/monitor/on"><button class="green">Monitor ON</button></a>
    <a href="/rf/monitor/off"><button class="red">Monitor OFF</button></a>
    <a href="/rf/carrier/on"><button class="blue">Carrier ON</button></a>
    <a href="/rf/carrier/off"><button class="gray">Carrier OFF</button></a>
    <form action="/rf/jam" method="get" style="margin-top: 8px;">
      <input name="count" type="number" min="1" max="20" value="5" style="width: 100px;" />
      <button type="submit" class="red" style="padding: 8px 12px;">Jam Signal</button>
    </form>
  </div>
  <div class="card">
    <h3>Status</h3>
    <p>CC1101 Present: <strong>)HTML";
  html += radio.present ? "yes" : "no";
  html += R"HTML(</strong></p>
    <p>Frequency: <strong>)HTML";
  html += String(radio.frequencyMHz, 2);
  html += R"HTML( MHz</strong></p>
    <p>RSSI: <strong>)HTML";
  html += String(radio.rssi);
  html += R"HTML( dBm</strong></p>
    <p>Packets RX/TX: <strong>)HTML";
  html += String(radio.packetsReceived);
  html += R"HTML(/)HTML";
  html += String(radio.packetsTransmitted);
  html += R"HTML(</strong></p>
    <p>Monitor: <strong>)HTML";
  html += radio.monitorMode ? "ON" : "OFF";
  html += R"HTML(</strong></p>
    <p>Carrier: <strong>)HTML";
  html += radio.carrierOn ? "ON" : "OFF";
  html += R"HTML(</strong></p>
    <p>Last Action: <strong>)HTML";
  html += String(radio.lastPacket);
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

void handleLedOn() {
  digitalWrite(LED_PIN, HIGH);
  server.sendHeader("Location", "/");
  server.send(302, "text/plain", "LED ON");
}

void handleLedOff() {
  digitalWrite(LED_PIN, LOW);
  server.sendHeader("Location", "/");
  server.send(302, "text/plain", "LED OFF");
}

// Håndterer BÅDE /rf/tune/{freq} (path-parameter via UriBraces)
// OG /rf/tune/?freq=xxx (query-parameter), avhengig av hvilken route som traff.
void handleTune() {
  String freqValue = server.pathArg(0);
  if (freqValue.length() == 0) {
    freqValue = server.arg("freq");
  }
  
  float freq = freqValue.toFloat();
  if (freq < MIN_FREQUENCY_MHZ || freq > MAX_FREQUENCY_MHZ) {
    String payload = "{\"ok\":false,\"error\":\"Frequency must be " + String(MIN_FREQUENCY_MHZ, 0) + "-" + String(MAX_FREQUENCY_MHZ, 0) + " MHz\"}";
    server.send(400, "application/json", payload);
    return;
  }
  
  cc1101SetFrequency(freq);
  String payload = "{\"ok\":true,\"frequency\":" + String(freq, 2) + "}";
  server.send(200, "application/json", payload);
}

void handleSendPing() {
  cc1101SendPing();
  server.send(200, "application/json", "{\"ok\":true}");
}

void handleReadPacket() {
  cc1101ReadPacket();
  server.send(200, "application/json", "{\"ok\":true}");
}

void handleRssi() {
  int8_t rssi = cc1101ReadRssi();
  String payload = "{\"rssi\":" + String(rssi) + "}";
  server.send(200, "application/json", payload);
}

void handleSweep() {
  cc1101SweepFrequencies();
  String payload = "{\"bestMHz\":" + String(radio.sweepBestMHz, 2) + ",\"bestRssi\":" + String(radio.sweepBestRssi) + "}";
  server.send(200, "application/json", payload);
}

void handleMonitorOn() {
  radio.monitorMode = true;
  server.send(200, "application/json", "{\"monitor\":true}");
}

void handleMonitorOff() {
  radio.monitorMode = false;
  server.send(200, "application/json", "{\"monitor\":false}");
}

void handleCarrierOn() {
  cc1101StartCarrier();
  server.send(200, "application/json", "{\"carrier\":true}");
}

void handleCarrierOff() {
  cc1101StopCarrier();
  server.send(200, "application/json", "{\"carrier\":false}");
}

void handleJam() {
  String countValue = server.arg("count");
  uint16_t count = 5;
  if (countValue.length() > 0) {
    long requested = countValue.toInt();
    if (requested < 1) {
      requested = 1;
    }
    if (requested > 20) {
      requested = 20;
    }
    count = (uint16_t)requested;
  }
  cc1101JamSignal(count);
  String payload = "{\"jam\":true,\"count\":" + String(count) + "}";
  server.send(200, "application/json", payload);
}

void setup() {
  Serial.begin(115200);
  delay(500);
  
  Serial.println("\n\n==============================================");
  Serial.println("       FLIPPFLOP - CC1101 Radio Controller");
  Serial.println("==============================================");
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  
  bool radioOk = cc1101Init();
  if (radioOk) {
    Serial.println("[SUCCESS] CC1101 ready");
  } else {
    Serial.println("[WARNING] CC1101 not detected");
  }
  
  if (ENABLE_WIFI) {
    Serial.println("[INFO] Starting WiFi Access Point...");
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASSWORD);
    IPAddress ip = WiFi.softAPIP();
    
    Serial.print("[INFO] AP SSID: ");
    Serial.println(AP_SSID);
    Serial.print("[INFO] AP IP: ");
    Serial.println(ip);
    
    server.on("/", HTTP_GET, handleRoot);
    server.on("/led/on", HTTP_GET, handleLedOn);
    server.on("/led/off", HTTP_GET, handleLedOff);
    // Query-form: /rf/tune/?freq=433.92
    server.on("/rf/tune/", HTTP_GET, handleTune);
    // Path-form: /rf/tune/433.92  (krever UriBraces for at ESP32 WebServer skal fange parameteren)
    server.on(UriBraces("/rf/tune/{}"), HTTP_GET, handleTune);
    server.on("/rf/send", HTTP_GET, handleSendPing);
    server.on("/rf/read", HTTP_GET, handleReadPacket);
    server.on("/rf/rssi", HTTP_GET, handleRssi);
    server.on("/rf/sweep", HTTP_GET, handleSweep);
    server.on("/rf/monitor/on", HTTP_GET, handleMonitorOn);
    server.on("/rf/monitor/off", HTTP_GET, handleMonitorOff);
    server.on("/rf/carrier/on", HTTP_GET, handleCarrierOn);
    server.on("/rf/carrier/off", HTTP_GET, handleCarrierOff);
    server.on("/rf/jam", HTTP_GET, handleJam);
    
    server.begin();
    Serial.println("[SUCCESS] Web server started");
  } else {
    Serial.println("[INFO] WiFi disabled - Standalone/Flipper mode");
    Serial.println("[INFO] Waiting for serial commands...");
    Serial.println("[INFO] Type 'HELP' for available commands");
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
  
  if (radio.monitorMode) {
    cc1101ReadPacket();
    delay(1000);
  }
}
