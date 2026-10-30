// Mini Hydroponics - ESP32 final code (v3)
// v3: clean OLED layout (3 pages) + gauge web dashboard
// Parts: OLED (0x3C), DHT11, water level, LDR, TDS sensor, pH sensor, relay (pump)
// Libraries: Adafruit SSD1306 (+GFX), DHT sensor library (Adafruit)

#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <DHT.h>

// ---------- WIFI (2.4 GHz only) ----------
const char* WIFI_SSID_1 = "IOT LAB";
const char* WIFI_PASS_1 = "hungrysun358";

// ---------- PINS ----------
#define DHT_PIN     4
#define LEVEL_PIN   32
#define LDR_PIN     33
#define PH_PIN      34
#define TDS_PIN     35
#define RELAY_PUMP  27      // Pump = relay 2 (IN2 -> D27). Pump relay 1 var asel tar 26 kara.
#define SDA_PIN     21
#define SCL_PIN     22

#define RELAY_ON  LOW      // relay ulta chalat asel tar HIGH kara
#define RELAY_OFF HIGH

// ---------- SETTINGS ----------
const bool LEVEL_ENABLED = true;   // level sensor jodla nasel tar false
const bool TDS_ENABLED   = true;   // TDS sensor jodla nasel tar false (manual entry chalel)
const bool PH_ENABLED    = true;   // pH jodla nasel tar false
const int  OLED_ROTATION = 0;      // OLED ulta distoy tar 2 kara (0 = normal, 2 = 180 degree)

// Water level: korda 142 mV, pani madhe 2165 mV
const int LEVEL_MIN_MV = 1000;     // hyachya khali gelyavar pump band
const int LEVEL_DRY_MV = 142;      // gauge 0 %
const int LEVEL_WET_MV = 2165;     // gauge 100 %

// LDR: andhar 2080 mV, ujed 250 mV (ujed = lahan number)
const int LDR_DARK_MV    = 1500;   // hyapeksha jaast = DARK
const int LDR_BRIGHT_MV  = 700;    // hyapeksha kami = BRIGHT
const int LDR_DARK_REF   = 2080;   // gauge 0 %
const int LDR_BRIGHT_REF = 250;    // gauge 100 %

// TDS: handheld meter shi match karnya sathi hi value badla
// (handheld ppm / sensor ppm). Suruvat la 1.0 thevaa
const float TDS_CAL = 1.0;

// pH calibration (PH-4502C)
// IMPORTANT: PH_DIV = 2.0 assumes a 1:1 voltage divider is installed
// between module PO and ESP32 GPIO34. Do NOT connect PO directly if it
// can exceed the ESP32 input limit.
// Replace PH7_VOLT and PH4_VOLT with measured module-output voltages
// while the probe is in pH 7.00 and pH 4.00 buffer solutions.
const float PH_MIN = 5.5, PH_MAX = 6.5;
const float PH_DIV = 2.0;
const float PH7_VOLT = 2.50;  // EXAMPLE ONLY: calibrate with pH 7.00 buffer
const float PH4_VOLT = 3.00;  // EXAMPLE ONLY: calibrate with pH 4.00 buffer

const unsigned long PUMP_ON_MS  = 120000UL;  // Pump ON for 2 minutes
const unsigned long PUMP_OFF_MS = 10000UL;   // Pump OFF for 10 seconds

// ---------- OBJECTS ----------
Adafruit_SSD1306 oled(128, 64, &Wire, -1);
DHT dht(DHT_PIN, DHT11);
WebServer server(80);
bool oledOk = false;

// ---------- STATE ----------
float airTemp = 25, airHum = 0, phVal = 7, tdsVal = 0;
int levelMv = 0, ldrMv = 0;
int manualTds = 0;                 // 0 = sensor vaapra, >0 = manual value
bool waterLow = false, alertOn = false;
bool pumpOn = false;
int pumpMode = 0;                  // 0 = AUTO, 1 = ON, 2 = OFF
unsigned long pumpTimer = 0, sensorTimer = 0, oledTimer = 0;
byte page = 0;

// ---------- HELPERS ----------
float readMv(int pin) {
  long sum = 0;
  for (int i = 0; i < 20; i++) { sum += analogReadMilliVolts(pin); delay(2); }
  return sum / 20.0;
}

float readTds() {
  float v = readMv(TDS_PIN) / 1000.0;
  float comp = 1.0 + 0.02 * (airTemp - 25.0);   // temperature compensation
  float cv = v / comp;
  float ppm = (133.42 * cv * cv * cv - 255.86 * cv * cv + 857.39 * cv) * 0.5;
  return ppm * TDS_CAL;
}

int tdsShown() {
  if (manualTds > 0) return manualTds;
  return TDS_ENABLED ? (int)tdsVal : 0;
}

int levelPct() {
  long p = ((long)(levelMv - LEVEL_DRY_MV) * 100L) / (LEVEL_WET_MV - LEVEL_DRY_MV);
  return constrain((int)p, 0, 100);
}

int ldrPct() {   // 100 = khoop ujed, 0 = andhar
  long p = ((long)(LDR_DARK_REF - ldrMv) * 100L) / (LDR_DARK_REF - LDR_BRIGHT_REF);
  return constrain((int)p, 0, 100);
}

const char* ldrText() {
  if (ldrMv > LDR_DARK_MV) return "DARK";
  if (ldrMv < LDR_BRIGHT_MV) return "BRIGHT";
  return "NORMAL";
}

void readSensors() {
  float t = dht.readTemperature();
  float h = dht.readHumidity();
  if (!isnan(t)) airTemp = t;
  if (!isnan(h)) airHum = h;

  levelMv = (int)readMv(LEVEL_PIN);
  ldrMv = (int)readMv(LDR_PIN);
  if (TDS_ENABLED) tdsVal = readTds();

  if (PH_ENABLED) {
    // Average ADC voltage at GPIO34, then reconstruct module PO voltage.
    float pinVoltage = readMv(PH_PIN) / 1000.0;
    float sensorVoltage = pinVoltage * PH_DIV;

    // Two-point calibration using pH 7.00 and pH 4.00 buffers.
    // The example calibration voltages above must be replaced with
    // your measured values for an accurate pH result.
    float slope = (7.0 - 4.0) / (PH7_VOLT - PH4_VOLT);
    phVal = 7.0 + slope * (sensorVoltage - PH7_VOLT);

    // Limit displayed value to the standard 0-14 pH scale.
    phVal = constrain(phVal, 0.0f, 14.0f);
  }

  waterLow = LEVEL_ENABLED && (levelMv < LEVEL_MIN_MV);
  alertOn = waterLow || (PH_ENABLED && (phVal < PH_MIN || phVal > PH_MAX));
}

// Relay control. 5V relay module + 3.3V ESP32 pin: HIGH (3.3V) kahi modules la poora OFF karat nahi.
// Mhanun OFF = pin sodun dya (INPUT), ON = pin LOW kara. Hyane relay nehmi barobar OFF hoto.
const bool RELAY_FLOAT_OFF = true;   // true = OFF sathi pin sodaa (recommended)
void relayWrite(int pin, bool on) {
  if (on) {
    digitalWrite(pin, RELAY_ON);
    pinMode(pin, OUTPUT);
  } else if (RELAY_FLOAT_OFF) {
    pinMode(pin, INPUT);
  } else {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, RELAY_OFF);
  }
}

void setPump(bool on) { pumpOn = on; relayWrite(RELAY_PUMP, on); }

void updatePump() {
  if (waterLow) { setPump(false); return; }      // SAFETY: pani nasel tar pump band
  if (pumpMode == 1) { setPump(true);  return; }
  if (pumpMode == 2) { setPump(false); return; }
  unsigned long now = millis();
  if (pumpOn && now - pumpTimer >= PUMP_ON_MS)        { setPump(false); pumpTimer = now; }
  else if (!pumpOn && now - pumpTimer >= PUMP_OFF_MS) { setPump(true);  pumpTimer = now; }
}

const char* modeText(int m) { return m == 0 ? "AUTO" : (m == 1 ? "ON" : "OFF"); }

// ---------- OLED (clean layout) ----------
// Header bar (white) + 4 rows. Label left, value right.
void oledHeader(const char* title, int pg) {
  oled.fillRect(0, 0, 128, 12, SSD1306_WHITE);
  oled.setTextSize(1);
  oled.setTextColor(SSD1306_BLACK);
  oled.setCursor(3, 2);
  oled.print(title);
  char buf[8];
  snprintf(buf, sizeof(buf), "%d/3", pg);
  int16_t x1, y1; uint16_t w, h;
  oled.getTextBounds(buf, 0, 0, &x1, &y1, &w, &h);
  oled.setCursor(125 - w, 2);
  oled.print(buf);
  oled.setTextColor(SSD1306_WHITE);
}

void oledRow(int y, const char* label, const String& value) {
  oled.setTextSize(1);
  oled.setTextColor(SSD1306_WHITE);
  oled.setCursor(2, y);
  oled.print(label);
  int16_t x1, y1; uint16_t w, h;
  oled.getTextBounds(value, 0, 0, &x1, &y1, &w, &h);
  oled.setCursor(126 - w, y);
  oled.print(value);
}

void updateOLED() {
  if (!oledOk || millis() - oledTimer < 3000) return;
  oledTimer = millis();
  oled.clearDisplay();

  if (page == 0) {
    oledHeader("AIR & TANK", 1);
    String t = String(airTemp, 1); t += (char)247; t += "C";
    oledRow(16, "Temp", t);
    oledRow(28, "Humidity", String(airHum, 0) + " %");
    oledRow(40, "Tank", waterLow ? "LOW!" : "OK");
    oledRow(52, "Light", ldrText());
  } else if (page == 1) {
    oledHeader("WATER", 2);
    oledRow(16, "TDS", String(tdsShown()) + " ppm");
    oledRow(28, "pH", PH_ENABLED ? String(phVal, 1) : String("n/a"));
    oledRow(40, "Pump", pumpOn ? "ON" : "OFF");
    oledRow(52, "Mode", modeText(pumpMode));
  } else {
    oledHeader("STATUS", 3);
    bool wifiOk = (WiFi.status() == WL_CONNECTED);
    oledRow(16, "Status", alertOn ? "ALERT" : "All OK");
    oledRow(28, "WiFi", wifiOk ? "Connected" : "Offline");
    oledRow(40, "IP", wifiOk ? WiFi.localIP().toString() : String("--"));
    oledRow(52, "Water", String(levelPct()) + " %");
  }
  oled.display();
  page = (page + 1) % 3;
}

// ---------- WEB DASHBOARD (gauges) ----------
const char PAGE_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Hydroponics</title>
<style>
body{margin:0 auto;max-width:760px;padding:14px;background:#0d1712;color:#e8f5e9;font-family:system-ui,-apple-system,Segoe UI,Roboto,sans-serif}
h1{font-size:20px;margin:0}
.top{display:flex;justify-content:space-between;align-items:center;margin-bottom:12px}
.sub{color:#8fa89a;font-size:12px}
.dot{width:10px;height:10px;border-radius:50%;background:#ff5c5c;display:inline-block;margin-right:6px}
.dot.on{background:#3ddc84}
.grid{display:grid;grid-template-columns:repeat(2,1fr);gap:10px}
@media(min-width:640px){.grid{grid-template-columns:repeat(3,1fr)}}
.card{background:#16241c;border:1px solid #24382c;border-radius:14px;padding:10px 8px 8px;text-align:center}
.card svg{width:100%;height:auto;display:block}
.trk{fill:none;stroke:#24382c;stroke-width:9;stroke-linecap:round}
.val{fill:none;stroke:#3ddc84;stroke-width:9;stroke-linecap:round;transition:stroke-dashoffset .6s,stroke .3s}
.gv{font-size:20px;font-weight:700;fill:#e8f5e9;text-anchor:middle}
.gu{font-size:8px;fill:#8fa89a;text-anchor:middle}
.gm{font-size:7px;fill:#8fa89a}
.gn{font-size:13px;color:#cfe3d6;margin-top:2px}
.chips{display:grid;grid-template-columns:repeat(auto-fit,minmax(140px,1fr));gap:10px;margin:12px 0}
.chip{background:#16241c;border:1px solid #24382c;border-radius:14px;padding:10px 12px}
.chip small{display:block;color:#8fa89a;font-size:12px}
.chip b{font-size:18px}
.bad{border-color:#ff5c5c;background:#2a1717}
.ctl{background:#16241c;border:1px solid #24382c;border-radius:14px;padding:12px;margin-bottom:10px}
.ctl h3{margin:0 0 8px;font-size:14px;color:#cfe3d6}
.seg{display:flex;gap:8px}
.seg button{flex:1;padding:12px 0;border:1px solid #2f4a3a;border-radius:10px;background:#0f1a14;color:#e8f5e9;font-size:15px}
.seg button.act{background:#3ddc84;color:#06210f;border-color:#3ddc84;font-weight:700}
.row{display:flex;gap:8px}
input{flex:1;min-width:0;padding:12px;border-radius:10px;border:1px solid #2f4a3a;background:#0f1a14;color:#e8f5e9;font-size:15px}
.row button{padding:12px 18px;border:0;border-radius:10px;background:#3ddc84;color:#06210f;font-weight:700}
</style></head><body>
<div class="top">
<div><h1>Hydroponics</h1><div class="sub" id="st">Connecting...</div></div>
<div><span class="dot" id="dot"></span><span class="sub" id="up"></span></div>
</div>
<div class="grid" id="gauges"></div>
<div class="chips">
<div class="chip" id="cTank"><small>Tank</small><b id="vTank">--</b></div>
<div class="chip" id="cAlert"><small>System</small><b id="vAlert">--</b></div>
<div class="chip"><small>Pump</small><b id="vPump">--</b></div>
</div>
<div class="ctl"><h3>Pump</h3><div class="seg" id="sp">
<button data-m="auto">Auto</button><button data-m="on">ON</button><button data-m="off">OFF</button></div></div>
<div class="ctl"><h3>TDS from handheld meter (ppm, 0 = use sensor)</h3>
<div class="row"><input id="tdsIn" type="number" min="0" inputmode="numeric" placeholder="e.g. 650"><button id="tdsBtn">Save</button></div></div>
<script>
const G=[
{k:'t',n:'Temperature',u:'\u00b0C',min:0,max:50,d:1,c:v=>v<18?'#4fc3f7':v<=30?'#3ddc84':v<=35?'#ffb74d':'#ff5c5c'},
{k:'h',n:'Humidity',u:'%',min:0,max:100,d:0,c:v=>(v<40||v>80)?'#ffb74d':'#3ddc84'},
{k:'lvl',n:'Water level',u:'%',min:0,max:100,d:0,c:v=>v<30?'#ff5c5c':v<50?'#ffb74d':'#3ddc84'},
{k:'ldrp',n:'Light',u:'%',min:0,max:100,d:0,c:v=>v<20?'#ffb74d':'#3ddc84'},
{k:'tds',n:'TDS',u:'ppm',min:0,max:1500,d:0,c:v=>v<200?'#ffb74d':v<=1000?'#3ddc84':v<=1300?'#ffb74d':'#ff5c5c'},
{k:'ph',n:'pH',u:'',min:0,max:14,d:1,c:v=>(v>=5.5&&v<=6.5)?'#3ddc84':(v>=5&&v<=7)?'#ffb74d':'#ff5c5c'}
];
const $=id=>document.getElementById(id);
const L=141.4;
$('gauges').innerHTML=G.map(g=>'<div class="card"><svg viewBox="0 0 120 82">'
+'<path class="trk" d="M15 62A45 45 0 0 1 105 62"/>'
+'<path class="val" id="a_'+g.k+'" d="M15 62A45 45 0 0 1 105 62" stroke-dasharray="'+L+'" stroke-dashoffset="'+L+'"/>'
+'<text class="gv" id="v_'+g.k+'" x="60" y="56">--</text>'
+'<text class="gu" x="60" y="68">'+g.u+'</text>'
+'<text class="gm" x="15" y="77" text-anchor="middle">'+g.min+'</text>'
+'<text class="gm" x="105" y="77" text-anchor="middle">'+g.max+'</text>'
+'</svg><div class="gn">'+g.n+'</div></div>').join('');
function setG(g,v){
  const a=$('a_'+g.k),t=$('v_'+g.k);
  if(v===null||v===undefined||isNaN(v)){t.textContent='--';a.style.strokeDashoffset=L;a.style.stroke='#24382c';return;}
  let f=(v-g.min)/(g.max-g.min);f=Math.max(0,Math.min(1,f));
  a.style.strokeDashoffset=L*(1-f);
  a.style.stroke=g.c(v);
  t.textContent=Number(v).toFixed(g.d);
}
function seg(id,cur){document.querySelectorAll('#'+id+' button').forEach(b=>b.classList.toggle('act',b.dataset.m===String(cur).toLowerCase()))}
function fmtUp(s){const h=Math.floor(s/3600),m=Math.floor(s%3600/60);return h+'h '+m+'m'}
async function load(){
  try{
    const r=await fetch('/data',{cache:'no-store'});
    const d=await r.json();
    G.forEach(g=>setG(g,d[g.k]));
    $('vTank').textContent=d.low?'LOW':'OK';
    $('cTank').classList.toggle('bad',d.low);
    $('vAlert').textContent=d.alert?'ALERT':'All OK';
    $('cAlert').classList.toggle('bad',d.alert);
    $('vPump').textContent=(d.pump?'ON':'OFF')+' ('+d.pm+')';
    seg('sp',d.pm);
    $('dot').className='dot on';
    $('st').textContent='Live \u00b7 WiFi '+d.rssi+' dBm';
    $('up').textContent='Up '+fmtUp(d.up);
  }catch(e){
    $('dot').className='dot';
    $('st').textContent='Offline - reconnecting...';
  }
}
function send(u){fetch(u,{cache:'no-store'}).then(load).catch(()=>{})}
document.querySelectorAll('#sp button').forEach(b=>b.onclick=()=>send('/pump?mode='+b.dataset.m));
$('tdsBtn').onclick=()=>{const v=parseInt($('tdsIn').value||'0',10);send('/tds?v='+(isNaN(v)?0:v))};
load();setInterval(load,2000);
</script></body></html>
)rawliteral";

void handleRoot() {
  server.sendHeader("Cache-Control", "no-store");
  server.send_P(200, "text/html", PAGE_HTML);
}

void handleData() {
  String j;
  j.reserve(380);
  j = "{";
  j += "\"t\":" + String(airTemp, 1);
  j += ",\"h\":" + String(airHum, 0);
  j += ",\"lvl\":" + String(levelPct());
  j += ",\"lvlmv\":" + String(levelMv);
  j += ",\"ldrp\":" + String(ldrPct());
  j += ",\"ldrmv\":" + String(ldrMv);
  j += ",\"ldrt\":\"" + String(ldrText()) + "\"";
  j += ",\"tds\":" + String(tdsShown());
  j += ",\"ph\":";
  j += PH_ENABLED ? String(phVal, 2) : String("null");
  j += ",\"low\":" + String(waterLow ? "true" : "false");
  j += ",\"alert\":" + String(alertOn ? "true" : "false");
  j += ",\"pump\":" + String(pumpOn ? "true" : "false");
  j += ",\"pm\":\"" + String(modeText(pumpMode)) + "\"";
  j += ",\"rssi\":" + String(WiFi.RSSI());
  j += ",\"up\":" + String(millis() / 1000);
  j += "}";
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", j);
}

int parseMode(const String& m) { return m == "on" ? 1 : (m == "off" ? 2 : 0); }

void handlePump() {
  pumpMode = parseMode(server.arg("mode"));
  if (pumpMode == 0) pumpTimer = millis();
  server.send(200, "text/plain", "ok");
}

void handleTds() {
  manualTds = server.arg("v").toInt();
  if (manualTds < 0) manualTds = 0;
  server.send(200, "text/plain", "ok");
}

// ---------- WIFI HELPERS ----------
bool tryWiFi(const char* ssid, const char* pass, unsigned long timeoutMs) {
  Serial.print("Trying WiFi: "); Serial.println(ssid);
  if (oledOk) {
    oled.clearDisplay();
    oled.setTextSize(1);
    oled.setTextColor(SSD1306_WHITE);
    oled.setCursor(0, 0);
    oled.println("Connecting WiFi...");
    oled.println(ssid);
    oled.display();
  }
  WiFi.disconnect(true);
  delay(200);
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(ssid, pass);
  unsigned long st = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - st < timeoutMs) delay(250);
  return WiFi.status() == WL_CONNECTED;
}

bool connectWiFi() {
  return tryWiFi(WIFI_SSID_1, WIFI_PASS_1, 12000);
}

void scanAndPrint() {
  Serial.println("WiFi networks ESP32 la distat ahet:");
  int n = WiFi.scanNetworks();
  if (n <= 0) Serial.println("  (kahich distat nahi)");
  for (int i = 0; i < n; i++) {
    Serial.printf("  %s  (%d dBm, channel %d)\n", WiFi.SSID(i).c_str(), WiFi.RSSI(i), WiFi.channel(i));
  }
}

// ---------- MAIN ----------
void setup() {
  Serial.begin(115200);

  // Relay: suruvat la OFF (pin sodlele)
  relayWrite(RELAY_PUMP, false);
  analogSetAttenuation(ADC_11db);

  Wire.begin(SDA_PIN, SCL_PIN);
  oledOk = oled.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  if (!oledOk) oledOk = oled.begin(SSD1306_SWITCHCAPVCC, 0x3D);   // kahi screens 0x3D vaaprat
  if (!oledOk) Serial.println("OLED not found - wiring tapasaa (VDD=3V3, GND, SDA=D21, SCK=D22)");
  else {
    oled.setRotation(OLED_ROTATION);
    oled.clearDisplay();
    oled.setTextSize(2);
    oled.setTextColor(SSD1306_WHITE);
    oled.setCursor(0, 8);
    oled.println("Hydroponics");
    oled.setTextSize(1);
    oled.setCursor(0, 36);
    oled.println("Starting...");
    oled.display();
  }

  dht.begin();

  WiFi.setAutoReconnect(true);
  bool wifiUp = connectWiFi();
  if (wifiUp) {
    Serial.print("WiFi connected to: "); Serial.println(WiFi.SSID());
    if (oledOk) {
      oled.clearDisplay();
      oled.setTextSize(1);
      oled.setTextColor(SSD1306_WHITE);
      oled.setCursor(0, 0);
      oled.println("WiFi connected");
      oled.setCursor(0, 20);
      oled.println("Open in browser:");
      oled.setCursor(0, 34);
      oled.print("http://");
      oled.println(WiFi.localIP());
      oled.display();
    }
    delay(4000);
  } else {
    Serial.println("WiFi FAILED - hotspot ON ahe ka, 2.4 GHz ahe ka, naav/password barobar ahe ka tapasaa");
    scanAndPrint();
    if (oledOk) {
      oled.clearDisplay();
      oled.setTextSize(1);
      oled.setTextColor(SSD1306_WHITE);
      oled.setCursor(0, 0);
      oled.println("WiFi FAILED");
      oled.println("Hotspot ON? 2.4 GHz?");
      oled.println("Retrying...");
      oled.display();
    }
    delay(3000);
  }

  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.on("/pump", handlePump);
  server.on("/tds", handleTds);
  server.onNotFound([]() { server.send(404, "text/plain", "Not found"); });
  server.begin();

  Serial.print("Open in browser: http://"); Serial.println(WiFi.localIP());
  delay(1500);
  readSensors();

  pumpTimer = millis();

  // Start pump immediately if the water level is safe
  setPump(!waterLow);
}

void loop() {
  server.handleClient();

  if (millis() - sensorTimer >= 2000) {
    sensorTimer = millis();
    readSensors();
    Serial.printf("T=%.1f H=%.0f level=%dmV(%d%%) ldr=%dmV(%s) tds=%d ph=%.2f pump=%d low=%d\n",
                  airTemp, airHum, levelMv, levelPct(), ldrMv, ldrText(), tdsShown(), phVal, pumpOn, waterLow);
  }

  updatePump();
  updateOLED();

  // IP nehmi disava mhanun dar 10 second la print
  static unsigned long lastIp = 0;
  if (millis() - lastIp > 10000) {
    lastIp = millis();
    if (WiFi.status() == WL_CONNECTED) { Serial.print("Dashboard: http://"); Serial.println(WiFi.localIP()); }
    else Serial.println("WiFi not connected - IP nahi");
  }

  static bool wasUp = false;
  bool up = (WiFi.status() == WL_CONNECTED);
  if (up && !wasUp) { Serial.print("WiFi connected! Dashboard: http://"); Serial.println(WiFi.localIP()); }
  wasUp = up;

  // WiFi gela tar parat try (blocking nahi)
  if (!up) {
    static unsigned long lastTry = 0;
    if (millis() - lastTry > 20000) {
      lastTry = millis();
      WiFi.disconnect();
      WiFi.begin(WIFI_SSID_1, WIFI_PASS_1);
    }
  }
}
