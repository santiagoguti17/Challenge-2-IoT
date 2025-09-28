#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <MPU6500_WE.h>
#include <LiquidCrystal.h>
#include <math.h>

// ===== Pines (fila de abajo) =====
// Sensores/actuadores
#define PIN_HUM    32     // ADC humedad (A0 del módulo resistivo)
#define PIN_RAIN   15    // MH-RD D0 (activo LOW con pull-up)
#define PIN_LED    23    // LED
#define PIN_BUZZ   5     // Buzzer

// I2C IMU dedicado (NO usar 21/22)
#define I2C_SDA    19
#define I2C_SCL    18

// LCD 16x2 (paralelo 4-bit)  RS, E, D4, D5, D6, D7
#define LCD_RS     22
#define LCD_E      21
#define LCD_D4     2
#define LCD_D5     16     // RX2
#define LCD_D6     17     // TX2
#define LCD_D7     3      // RX0

LiquidCrystal lcd(LCD_RS, LCD_E, LCD_D4, LCD_D5, LCD_D6, LCD_D7);

// ===== Config lluvia =====
#define RAIN_ACTIVE_LOW   1
#define RAIN_CONFIRM_MS   1000

// ===== IMU =====
MPU6500_WE mpu(0x68);     // AD0 a GND => 0x68

// ===== Calibraciones (ajusta con tus lecturas reales) =====
int   humDry     = 3000;  // ADC en seco real
int   humWet     = 1200;  // ADC en suelo saturado real
float rmsBase    = 0.02f; // g
float rmsSigma   = 0.01f; // g
float THR_V_PEAK = 0.20f; // g pico

// ===== Estado lluvia =====
volatile bool rainRaw    = false;
volatile bool rainActive = false;
volatile unsigned long rainSince = 0;

// ===== Silenciar alarmas (toggle desde web) =====
volatile bool alarmSilenced = false;

// ===== Histórico en RAM =====
#define HISTORY_SIZE 60
struct Sample {
  unsigned long t;
  float H;          // 0..1
  bool rainRaw;
  bool rainActive;
  float rms;        // g
  float peak;       // g
  float Score;      // 0..1
  int level;        // 0/1/2
};
Sample history[HISTORY_SIZE];
int histIndex = 0;
int histCount = 0;

// ===== Últimos valores para UI/LCD =====
volatile float lastH = 0.0f;
volatile float lastRMS = 0.0f;
volatile float lastPeak = 0.0f;
volatile float lastScore = 0.0f;
volatile int   lastLevel = 0;

// ===== NUEVO: Inclinación =====
volatile float lastPitch = 0.0f; // grados
volatile float lastRoll  = 0.0f; // grados
static inline float lowpass(float prev, float now, float alpha=0.2f){
  return prev + alpha*(now-prev);
}

// ===== Utils =====
static inline float clamp01(float x){ return x<0?0:(x>1?1:x); }

// ---------- Lluvia (debounce temporal) ----------
void updateRainState_raw() {
  int val = digitalRead(PIN_RAIN);
  bool nowWet = RAIN_ACTIVE_LOW ? (val == LOW) : (val == HIGH);
  unsigned long t = millis();
  if (nowWet) {
    if (!rainRaw) rainSince = t;                     // flanco a mojado
    rainActive = (t - rainSince >= RAIN_CONFIRM_MS); // confirma
  } else {
    rainActive = false;
  }
  rainRaw = nowWet;
}

// ---------- Humedad (mediana de 3 + mapeo seco/mojado) ----------
float readHumidityNorm_raw() {
  // Mejor rango ADC para 0..3.3V
  // (hazlo una sola vez en setup también; repetir no daña)
  analogSetPinAttenuation(PIN_HUM, ADC_11db);

  int a1 = analogRead(PIN_HUM);
  int a2 = analogRead(PIN_HUM);
  int a3 = analogRead(PIN_HUM);
  // mediana
  int lo = min(a1, min(a2, a3));
  int hi = max(a1, max(a2, a3));
  int adc = a1 + a2 + a3 - lo - hi;

  // clamps duros para evitar 100% fijo si el sensor satura
  if (adc >= humDry) return 0.0f;   // seco
  if (adc <= humWet) return 1.0f;   // mojado

  int span = humDry - humWet;       // >0 si humDry>humWet
  float h = (span == 0) ? 0.0f : (float)(humDry - adc) / (float)span;
  return clamp01(h);
}

// ---------- Vibración (RMS y pico en ~1 s) ----------
float readIMURMS_1s(float &peakOut) {
  unsigned long t0 = millis();
  float sum = 0.0f, peak = 0.0f;
  int n = 0;
  while (millis() - t0 < 1000) {
    xyzFloat a = mpu.getGValues();          // en g
    float mag_g = sqrtf(a.x*a.x + a.y*a.y + a.z*a.z);
    float dyn_g = mag_g - 1.0f;             // remueve gravedad
    sum += dyn_g * dyn_g;
    float absdyn = fabsf(dyn_g);
    if (absdyn > peak) peak = absdyn;
    n++;
    delay(10); // ~100 Hz
  }
  peakOut = peak;
  return (n>0) ? sqrtf(sum / (float)n) : 0.0f;
}

// ====== FreeRTOS task: sensado + Score + actuadores + histórico ======
TaskHandle_t sensorTaskHandle = NULL;
void sensorTask(void *pvParameters) {
  (void) pvParameters;
  for (;;) {
    // 1) Lluvia
    updateRainState_raw();

    // 2) Humedad normalizada (0..1)
    float H = readHumidityNorm_raw();

    // 3) IMU (1 s)
    float peak = 0.0f;
    float rms  = readIMURMS_1s(peak);

    // ===== NUEVO: cálculo Pitch/Roll desde acelerómetro =====
    xyzFloat aAng = mpu.getGValues();
    float ax = aAng.x, ay = aAng.y, az = aAng.z;
    float rollDeg  = atan2f(ay, az) * 180.0f / PI;                     // rot. sobre X
    float pitchDeg = atan2f(-ax, sqrtf(ay*ay + az*az)) * 180.0f / PI;  // rot. sobre Y
    lastRoll  = lowpass(lastRoll,  rollDeg);
    lastPitch = lowpass(lastPitch, pitchDeg);

    // 4) Ponderaciones
    float Sv = 0.0f;
    if (peak >= THR_V_PEAK || rms >= (rmsBase + 6.0f*rmsSigma)) Sv = 1.0f;
    else if (rms >= (rmsBase + 3.0f*rmsSigma)) Sv = 0.6f;

    float Sr = 0.0f;
    if (rainActive) Sr = 0.5f;
    else if (rainRaw) Sr = 0.2f;

    float Sh = 0.0f;
    if (H > 0.80f) Sh = 1.0f;
    else if (H > 0.60f) Sh = 0.6f;

    float Score = 0.45f*Sv + 0.35f*Sh + 0.20f*Sr;

    int level = 0; // 0=normal, 1=amarillo, 2=rojo
    if (Score >= 0.7f || (Sv == 1.0f && Sh >= 0.6f)) level = 2;
    else if (Score >= 0.4f) level = 1;

    // 5) Publicar a variables "último estado"
    lastH     = H;
    lastRMS   = rms;
    lastPeak  = peak;
    lastScore = Score;
    lastLevel = level;

    // 6) Histórico circular
    Sample s;
    s.t = millis();
    s.H = H; s.rainRaw = rainRaw; s.rainActive = rainActive;
    s.rms = rms; s.peak = peak; s.Score = Score; s.level = level;
    history[histIndex] = s;
    histIndex = (histIndex + 1) % HISTORY_SIZE;
    if (histCount < HISTORY_SIZE) histCount++;

    // 7) Actuadores (respetando silenciado)
    if (!alarmSilenced) {
      if (level == 2) {
        digitalWrite(PIN_LED, HIGH);
        tone(PIN_BUZZ, 2000);
      } else if (level == 1) {
        digitalWrite(PIN_LED, (millis()/300)%2);
        tone(PIN_BUZZ, 1200, 150);
      } else {
        digitalWrite(PIN_LED, LOW);
        noTone(PIN_BUZZ);
      }
    } else {
      digitalWrite(PIN_LED, LOW);
      noTone(PIN_BUZZ);
    }

    // 8) Log corto (añadí pitch/roll al log)
    Serial.printf("[SAMPLE] H=%.0f%% rainRaw=%d conf=%d RMS=%.3f Pk=%.3f Score=%.2f Lvl=%d Pitch=%.1f Roll=%.1f sil=%d\n",
                  H*100.0f, rainRaw?1:0, rainActive?1:0, rms, peak, Score, level, lastPitch, lastRoll, alarmSilenced?1:0);

    // Repite (IMU ya consumió ~1 s)
    delay(50);
  }
}

// ====== Servidor Web (AP) ======
WebServer server(80);

// HTML + JS embebido (AÑADÍ Pitch y Roll en la tarjeta de KPIs)
const char INDEX_HTML[] PROGMEM = R"HTML(
<!doctype html>
<html>
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Nodo SAT Talud</title>
<style>
  body{font-family:system-ui,-apple-system,Segoe UI,Roboto,Arial;margin:12px;background:#f7f7f7}
  .card{background:#fff;border-radius:10px;padding:14px;margin-bottom:12px;box-shadow:0 1px 4px rgba(0,0,0,.12)}
  h1{margin:0 0 8px;font-size:18px}
  .state{font-size:20px;font-weight:700}
  .small{color:#666;font-size:12px}
  .row{display:flex;gap:8px;flex-wrap:wrap}
  .kv{flex:1 1 140px;background:#fafafa;border:1px solid #eee;border-radius:8px;padding:10px}
  .btn{padding:10px 14px;border:0;border-radius:8px;cursor:pointer}
  .btn-red{background:#d32f2f;color:#fff}
  .btn-green{background:#2e7d32;color:#fff}
  pre{white-space:pre-wrap;font-size:12px}
</style>
</head>
<body>
  <h1>Nodo SAT - Talud</h1>
  <div class="card">
    <div id="state" class="state">Cargando...</div>
    <div id="leveldesc" class="small">—</div>
  </div>

  <div class="card">
    <div class="row">
      <div class="kv"><b>Humedad</b><br><span id="hum">--</span></div>
      <div class="kv"><b>Lluvia</b><br><span id="lluv">--</span></div>
      <div class="kv"><b>Vibración (RMS)</b><br><span id="rms">--</span> g</div>
      <div class="kv"><b>Pico</b><br><span id="peak">--</span> g</div>
      <div class="kv"><b>Score</b><br><span id="score">--</span></div>
      <div class="kv"><b>Silenciado</b><br><span id="sil">NO</span></div>
      <div class="kv"><b>Pitch</b><br><span id="pitch">--</span> °</div>
      <div class="kv"><b>Roll</b><br><span id="roll">--</span> °</div>
    </div>
  </div>

  <div class="card">
    <button id="toggleBtn" class="btn btn-red">Silenciar alarmas</button>
    <div class="small">El silenciado apaga LED y buzzer, pero las mediciones siguen.</div>
  </div>

  <div class="card">
    <b>Histórico reciente</b>
    <pre id="hist"></pre>
  </div>

<script>
async function fetchStatus(){
  const r = await fetch('/status'); const j = await r.json();
  document.getElementById('hum').innerText  = Math.round(j.H*100) + '%';
  document.getElementById('lluv').innerText = j.rainConf ? 'SI' : (j.rainRaw ? 'POSIBLE' : 'NO');
  document.getElementById('rms').innerText  = j.rms.toFixed(3);
  document.getElementById('peak').innerText = j.peak.toFixed(3);
  document.getElementById('score').innerText= j.Score.toFixed(2);
  document.getElementById('sil').innerText  = j.silenced ? 'SI' : 'NO';
  // NUEVO: Pitch/Roll
  document.getElementById('pitch').innerText = j.pitch.toFixed(1);
  document.getElementById('roll').innerText  = j.roll.toFixed(1);

  const state = document.getElementById('state');
  const desc  = document.getElementById('leveldesc');
  const btn   = document.getElementById('toggleBtn');

  if (j.level == 2){
    state.textContent = 'PELIGRO ALTO';
    desc.textContent  = 'Nivel 2 — Actuar inmediatamente';
    btn.className = 'btn btn-red';
    btn.textContent = j.silenced ? 'Reactivar alarmas' : 'Silenciar alarmas';
  } else if (j.level == 1){
    state.textContent = 'PRECAUCIÓN';
    desc.textContent  = 'Nivel 1 — Vigilar la zona';
    btn.className = 'btn btn-red';
    btn.textContent = j.silenced ? 'Reactivar alarmas' : 'Silenciar alarmas';
  } else {
    state.textContent = 'TODO TRANQUILO';
    desc.textContent  = 'Nivel 0 — Operación normal';
    btn.className = 'btn btn-green';
    btn.textContent = j.silenced ? 'Reactivar alarmas' : 'Silenciar alarmas';
  }
}

async function fetchHistory(){
  const r = await fetch('/history'); const arr = await r.json();
  let txt = '';
  for (let i=0;i<arr.length;i++){
    const s = arr[i];
    const ts = (s.t/1000).toFixed(0) + 's';
    txt += ${ts} | H:${Math.round(s.H*100)}%  L:${s.rainConf?'SI':(s.rainRaw?'POS':'NO')}  RMS:${s.rms.toFixed(3)}  Pk:${s.peak.toFixed(2)}  Sc:${s.Score.toFixed(2)}  N:${s.level}\n;
  }
  document.getElementById('hist').textContent = txt;
}

document.getElementById('toggleBtn').addEventListener('click', async ()=>{
  const r = await fetch('/silence', {method:'POST'});
  const j = await r.json();
  await fetchStatus();
  await fetchHistory();
});

setInterval(()=>{ fetchStatus(); fetchHistory(); }, 2000);
window.onload = ()=>{ fetchStatus(); fetchHistory(); };
</script>
</body>
</html>
)HTML";

// ===== Endpoints JSON =====
String jsonStatus() {
  String s = "{";
  s += "\"H\":" + String(lastH,3) + ",";
  s += "\"rms\":" + String(lastRMS,4) + ",";
  s += "\"peak\":" + String(lastPeak,4) + ",";
  s += "\"Score\":" + String(lastScore,4) + ",";
  s += "\"level\":" + String(lastLevel) + ",";
  s += "\"rainRaw\":" + String(rainRaw?1:0) + ",";
  s += "\"rainConf\":" + String(rainActive?1:0) + ",";
  s += "\"silenced\":" + String(alarmSilenced?1:0) + ",";
  // ===== NUEVO: Pitch/Roll a la API
  s += "\"pitch\":" + String(lastPitch,1) + ",";
  s += "\"roll\":"  + String(lastRoll,1);
  s += "}";
  return s;
}

void handleRoot() { server.send_P(200, "text/html", INDEX_HTML); }
void handleStatus() { server.send(200, "application/json", jsonStatus()); }

void handleHistory() {
  String out = "[";
  int cnt = histCount;
  for (int i=0;i<cnt;i++){
    int idx = (histIndex - 1 - i + HISTORY_SIZE) % HISTORY_SIZE;
    Sample &s = history[idx];
    if (s.t == 0) continue;
    out += "{";
    out += "\"t\":" + String((unsigned long)s.t) + ",";
    out += "\"H\":" + String(s.H,3) + ",";
    out += "\"rms\":" + String(s.rms,4) + ",";
    out += "\"peak\":" + String(s.peak,4) + ",";
    out += "\"Score\":" + String(s.Score,4) + ",";
    out += "\"level\":" + String(s.level) + ",";
    out += "\"rainRaw\":" + String(s.rainRaw?1:0) + ",";
    out += "\"rainConf\":" + String(s.rainActive?1:0);
    out += "}";
    if (i < cnt-1) out += ",";
  }
  out += "]";
  server.send(200, "application/json", out);
}

void handleSilenceToggle() {
  alarmSilenced = !alarmSilenced;
  String j = "{\"silenced\":"+String(alarmSilenced?1:0)+"}";
  server.send(200, "application/json", j);
}

// ===== UI LCD (mensajes claros y banner en cambio de nivel) =====
void showLCD(float H, bool rainNow, bool rainConf, float score, int level){
  static uint32_t lastPageTs = 0;
  static uint8_t  page = 0;
  static int      lastLevelLCD = -1;
  static uint32_t levelChangeTs = 0;

  const int dwellMs  = 3000;  // alternancia pantallas
  const int bannerMs = 2000;  // banner al cambiar de nivel

  if (level != lastLevelLCD){
    lastLevelLCD = level;
    levelChangeTs = millis();
    lcd.clear();
    lcd.setCursor(0,0); lcd.print("CAMBIO DE NIVEL");
    lcd.setCursor(0,1);
    if (level == 2)      lcd.print("-> PELIGRO ALTO");
    else if (level == 1) lcd.print("-> PRECAUCION  ");
    else                 lcd.print("-> NORMAL      ");
    return;
  }
  if (millis() - levelChangeTs < (uint32_t)bannerMs) return;

  // de 2 pantallas a 3 pantallas (para inclinación)
  if (millis() - lastPageTs > (uint32_t)dwellMs) {
    page = (page + 1) % 3;
    lastPageTs = millis();
    lcd.clear();
  }

  int humPct  = (int)lround(H * 100.0f);
  int riskPct = (int)lround(score * 100.0f);
  bool llueve = (rainConf || rainNow);

  if (page == 0){
    lcd.setCursor(0,0);
    if (level == 2)      lcd.print("PELIGRO ALTO   ");
    else if (level == 1) lcd.print("PRECAUCION     ");
    else                 lcd.print("NORMAL         ");
    lcd.setCursor(0,1);
    char line2[17];
    snprintf(line2, sizeof(line2), "H:%2d%% Lluv:%s", humPct, llueve?"SI":"NO");
    lcd.print(line2);
  } 
  else if (page == 1) {
    lcd.setCursor(0,0);
    if (level == 0) lcd.print("Terreno Estable");
    else            lcd.print("Terreno Inestab");
    lcd.setCursor(0,1);
    char line2[17];
    snprintf(line2, sizeof(line2), "Riesgo:%2d%% N:%d", riskPct, level);
    lcd.print(line2);
  }
  else { // page == 2 (NUEVO)
    lcd.setCursor(0,0); lcd.print("Incl: Pitch Roll");
    lcd.setCursor(0,1);
    char line2[17];
    snprintf(line2, sizeof(line2), "%5.1f %5.1f  ", lastPitch, lastRoll);
    lcd.print(line2);
  }
}

// ====== SETUP ======
void setup() {
  Serial.begin(115200);
  delay(200);

  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_BUZZ, OUTPUT);
  pinMode(PIN_RAIN, INPUT_PULLUP);

  // IMU en 19/18
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000);

  // LCD 16x2
  lcd.begin(16, 2);
  lcd.clear();
  lcd.setCursor(0,0); lcd.print("Inicializando...");
  lcd.setCursor(0,1); lcd.print("IMU/HUM/LLUVIA  ");

  // IMU init
  if (!mpu.init()) {
    Serial.println("[IMU] MPU6500 no encontrado.");
    lcd.clear();
    lcd.setCursor(0,0); lcd.print("IMU FAIL");
    lcd.setCursor(0,1); lcd.print("Revise cableado");
  } else {
    mpu.setAccRange(MPU6500_ACC_RANGE_4G);
    mpu.setGyrRange(MPU6500_GYRO_RANGE_500);
    Serial.println("[IMU] OK");
  }

  // AP WiFi (servidor local)
  const char* apSSID = "AlcaldiaSAT";
  WiFi.mode(WIFI_AP);
  bool ok = WiFi.softAP(apSSID, "12345678");
  IPAddress ip = WiFi.softAPIP();
  Serial.printf("AP started: %s  IP: %s  ok=%d\n", apSSID, ip.toString().c_str(), ok);

  // HTTP server
  server.on("/", HTTP_GET, handleRoot);
  server.on("/status", HTTP_GET, handleStatus);
  server.on("/history", HTTP_GET, handleHistory);
  server.on("/silence", HTTP_POST, handleSilenceToggle);
  server.begin();
  Serial.println("HTTP server started.");

  // Task de sensado en el core 1
  xTaskCreatePinnedToCore(sensorTask, "SensorTask", 8192, NULL, 1, &sensorTaskHandle, 1);

  Serial.println("=== Sistema listo (IMU + Humedad + Lluvia + LCD + Web) ===");
}

// ====== LOOP (ligero: servidor + LCD) ======
void loop() {
  server.handleClient();

  // Refresca LCD con últimos valores (no bloquea)
  showLCD(lastH, rainRaw, rainActive, lastScore, lastLevel);

  delay(10);
}