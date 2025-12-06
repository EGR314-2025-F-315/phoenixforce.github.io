// TerraGuard rover firmware (ESP32-WROOM-32U)
#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include "bme_module.h"

// Pin map
#define GPS_RX_PIN 35
#define GPS_TX_PIN 36
#define MOTOR_L_IN1 1
#define MOTOR_L_IN2 2
#define MOTOR_R_IN1 3
#define MOTOR_R_IN2 4
#define MQ2_A_PIN 5
#define MQ2_D_PIN 21

// Motion timing
#define TIME_PER_UNIT 500
#define TIME_PER_TURN 400

// Network
const char* AP_SSID = "TerraGuard-AP";
const char* AP_PASS = "phoenixforce";
WebServer server(80);

// Sensors
BmeReading bme;
bool bmeReady = false;
const float MQ2_RL = 5.0f;
float mq2Ro = 0.0f;
const int MQ2_PREHEAT_MS = 20000;
const int MQ2_SAMPLE_COUNT = 50;
float cleanAirBaseline = 0.0f;

// GPS state
String gpsLat = "No Fix";
String gpsLon = "No Fix";
String gpsSats = "0";

// Motor helpers
void stopMotors() {
  digitalWrite(MOTOR_L_IN1, LOW);
  digitalWrite(MOTOR_L_IN2, LOW);
  digitalWrite(MOTOR_R_IN1, LOW);
  digitalWrite(MOTOR_R_IN2, LOW);
}

void driveForward(int duration) {
  digitalWrite(MOTOR_L_IN1, HIGH);
  digitalWrite(MOTOR_L_IN2, LOW);
  digitalWrite(MOTOR_R_IN1, HIGH);
  digitalWrite(MOTOR_R_IN2, LOW);
  delay(duration);
  stopMotors();
}

void driveBackward(int duration) {
  digitalWrite(MOTOR_L_IN1, LOW);
  digitalWrite(MOTOR_L_IN2, HIGH);
  digitalWrite(MOTOR_R_IN1, LOW);
  digitalWrite(MOTOR_R_IN2, HIGH);
  delay(duration);
  stopMotors();
}

void turnLeft(int duration) {
  digitalWrite(MOTOR_L_IN1, LOW);
  digitalWrite(MOTOR_L_IN2, HIGH);
  digitalWrite(MOTOR_R_IN1, HIGH);
  digitalWrite(MOTOR_R_IN2, LOW);
  delay(duration);
  stopMotors();
}

void turnRight(int duration) {
  digitalWrite(MOTOR_L_IN1, HIGH);
  digitalWrite(MOTOR_L_IN2, LOW);
  digitalWrite(MOTOR_R_IN1, LOW);
  digitalWrite(MOTOR_R_IN2, HIGH);
  delay(duration);
  stopMotors();
}

// Command parsing with remapped physical directions
void executeSegment(String cmd) {
  int magnitude = 1;
  String action = cmd;

  if (isDigit(cmd.charAt(0))) {
    int spaceIndex = cmd.indexOf(' ');
    if (spaceIndex != -1) {
      magnitude = cmd.substring(0, spaceIndex).toInt();
      action = cmd.substring(spaceIndex + 1);
    }
  }

  action.trim();
  action.toLowerCase();
  Serial.print("CMD: ");
  Serial.print(magnitude);
  Serial.print(" x ");
  Serial.println(action);

  if (action.startsWith("forward") || action.startsWith("up")) {
    turnLeft(magnitude * TIME_PER_UNIT);
  } else if (action.startsWith("back") || action.startsWith("down")) {
    turnRight(magnitude * TIME_PER_UNIT);
  } else if (action.startsWith("left")) {
    driveForward(magnitude * TIME_PER_TURN);
  } else if (action.startsWith("right")) {
    driveBackward(magnitude * TIME_PER_TURN);
  }

  delay(200);
}

void processCommandString(String rawCmd) {
  rawCmd.toLowerCase();
  int start = 0;
  int end = rawCmd.indexOf(',');

  while (end != -1) {
    String segment = rawCmd.substring(start, end);
    segment.trim();
    executeSegment(segment);
    start = end + 1;
    end = rawCmd.indexOf(',', start);
  }
  String segment = rawCmd.substring(start);
  segment.trim();
  executeSegment(segment);
}

// GPS parsing (GGA only)
void updateGPS() {
  while (Serial2.available() > 0) {
    String line = Serial2.readStringUntil('\n');
    if (line.startsWith("$GPGGA") || line.startsWith("$GNGGA")) {
      int i = 0, field = 0;
      String chunk = "";
      while (i < line.length()) {
        char c = line.charAt(i);
        if (c == ',') {
          if (field == 2) gpsLat = chunk;
          if (field == 3) gpsLat += chunk;
          if (field == 4) gpsLon = chunk;
          if (field == 5) gpsLon += chunk;
          if (field == 7) gpsSats = chunk;
          chunk = "";
          field++;
        } else {
          chunk += c;
        }
        i++;
      }
    }
  }
}

// MQ-2 helpers
float mq2ReadRs() {
  int sensorValue = analogRead(MQ2_A_PIN);
  float sensorVoltage = sensorValue * (3.3f / 4095.0f);
  if (sensorVoltage <= 0.01f) return 1000.0f;
  float Rs = (3.3f - sensorVoltage) * MQ2_RL / sensorVoltage;
  return Rs;
}

float mq2Calibrate() {
  float RsSum = 0.0f;
  for (int i = 0; i < MQ2_SAMPLE_COUNT; i++) {
    RsSum += mq2ReadRs();
    delay(100);
  }
  float RsAvg = RsSum / MQ2_SAMPLE_COUNT;
  return RsAvg / 9.83f;
}

// HTTP endpoints
void handleMetrics() {
  bme_read(bme);

  float mq2Rs = mq2ReadRs();
  float ratio = (mq2Ro > 0.0f) ? (mq2Rs / mq2Ro) : 0.0f;
  int mq2D = digitalRead(MQ2_D_PIN);

  String gasType = "Clean Air";
  if (mq2D == LOW) {
    gasType = "DANGER! (MQ2 D0 TRIPPED)";
  } else {
    if (mq2Rs > 25.0f) gasType = "Clean Air";
    else if (mq2Rs > 20.0f) gasType = "Slightly Polluted";
    else if (mq2Rs >= 4.0f) gasType = "Methane/H2";
    else if (mq2Rs > 0.0f) gasType = "HIGH Concentration";
    else gasType = "Sensor Error";
  }

  String airQuality = "Unknown";
  float gas = mq2Rs;
  if (gas > 25.0f) airQuality = "Excellent";
  else if (gas > 20.0f) airQuality = "Good";
  else if (gas > 15.0f) airQuality = "Moderate";
  else if (gas > 10.0f) airQuality = "Poor";
  else if (gas > 0.0f) airQuality = "Hazardous";
  else airQuality = "Sensor Error";

  String json = "{";
  json += "\"temp_c\":" + String(bme.tempC);
  json += ",\"humidity\":" + String(bme.humidity);
  json += ",\"gas_kohms\":" + String(mq2Rs);
  json += ",\"gas_type\":\"" + gasType + "\"";
  json += ",\"air_quality\":\"" + airQuality + "\"";
  json += "}";

  server.send(200, "application/json", json);
}

void handleCoords() {
  String json = "{";
  json += "\"lat\":\"" + gpsLat + "\"";
  json += ",\"lon\":\"" + gpsLon + "\"";
  json += ",\"sats\":\"" + gpsSats + "\"";
  json += "}";
  server.send(200, "application/json", json);
}

void handleControl() {
  if (server.hasArg("plain")) {
    String cmd = server.arg("plain");
    server.send(200, "text/plain", "OK");
    processCommandString(cmd);
  } else {
    server.send(400, "text/plain", "Empty");
  }
}

const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no">
  <title>TerraGuard Commander</title>
  <style>
    :root {
      --bg-color: #fcfaf8;
      --card-bg: #ffffff;
      --text-main: #2c2c2c;
      --text-muted: #848484;
      --accent: #C8553D;
      --shadow: 0 10px 30px -10px rgba(0,0,0,0.08);
    }
    body {
      font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, Helvetica, Arial, sans-serif;
      background-color: var(--bg-color);
      color: var(--text-main);
      margin: 0;
      padding: 20px;
      text-align: center;
    }
    h1 {
      font-family: "Georgia", "Times New Roman", serif;
      font-weight: 900;
      font-size: 2rem;
      letter-spacing: -0.5px;
      margin-bottom: 5px;
      color: var(--text-main);
    }
    .subtitle {
      font-family: "Georgia", "Times New Roman", serif;
      color: var(--text-muted);
      font-style: italic;
      margin-bottom: 30px;
      font-size: 0.9rem;
    }
    .card-title {
      text-transform: uppercase;
      letter-spacing: 1px;
      font-size: 0.75rem;
      font-weight: 700;
      color: var(--text-muted);
      margin-bottom: 20px;
      display: block;
      text-align: left;
    }
    .card {
      background: var(--card-bg);
      border-radius: 24px;
      padding: 25px;
      margin-bottom: 20px;
      box-shadow: var(--shadow);
      border: 1px solid rgba(0,0,0,0.03);
      max-width: 400px;
      margin-left: auto;
      margin-right: auto;
    }
    .data-grid { display: grid; grid-template-columns: 1fr; gap: 15px; }
    .data-row {
      display: flex;
      justify-content: space-between;
      align-items: center;
      padding-bottom: 10px;
      border-bottom: 1px solid #f0f0f0;
    }
    .data-row:last-child { border-bottom: none; }
    .label { font-size: 0.9rem; font-weight: 600; color: #555; }
    .val { 
      font-family: "Menlo", "Monaco", monospace; 
      font-size: 1.1rem; 
      color: var(--accent); 
      font-weight: bold; 
      background: #fdf3f0;
      padding: 4px 8px;
      border-radius: 6px;
    }

    .pad { 
      display: grid; 
      grid-template-columns: 1fr 1fr 1fr; 
      gap: 12px; 
    }
    .btn { 
      background: #f4f4f4; 
      color: var(--text-main); 
      padding: 20px 0; 
      font-size: 14px; 
      font-weight: 700;
      border-radius: 16px; 
      cursor: pointer; 
      user-select: none; 
      transition: all 0.2s ease;
      box-shadow: 0 2px 5px rgba(0,0,0,0.05);
      display: flex;
      align-items: center;
      justify-content: center;
    }
    .btn:active { 
      background: var(--accent); 
      color: white; 
      transform: scale(0.95);
      box-shadow: inset 0 2px 4px rgba(0,0,0,0.1);
    }
    
    .wide { grid-column: span 3; display: flex; gap: 8px; margin-top: 15px; }
    input { 
      flex-grow: 1; 
      padding: 14px; 
      border-radius: 12px; 
      border: 2px solid #eee; 
      background: #fafafa;
      font-family: inherit;
      outline: none;
      transition: border 0.3s;
    }
    input:focus { border-color: var(--accent); background: white; }
    button.go { 
      padding: 0 24px; 
      background: var(--text-main); 
      color: white; 
      border: none; 
      border-radius: 12px; 
      font-weight: bold; 
      cursor: pointer;
    }
    button.go:active { opacity: 0.8; }
    
    .spacer { height: 100%; width: 100%; }
  </style>
</head>
<body>

  <div>
    <h1>Phoenix Force</h1>
    <div class="subtitle">TerraGuard Commander</div>
  </div>

  <div class="card">
    <span class="card-title">DRIVE CONTROL</span>
    <div class="pad">
      <div class="spacer"></div>
      <div class="btn" onclick="post('1 forward')">▲</div>
      <div class="spacer"></div>
      
      <div class="btn" onclick="post('left')">◀</div>
      <div class="btn" onclick="post('stop')" style="font-size:10px; text-transform:uppercase; letter-spacing:1px; color:var(--accent);">Stop</div>
      <div class="btn" onclick="post('right')">▶</div>
      
      <div class="spacer"></div>
      <div class="btn" onclick="post('1 back')">▼</div>
      <div class="spacer"></div>
      
      <div class="wide">
        <input type="text" id="custom" placeholder="CMD: 2 forward, left...">
        <button class="go" onclick="sendCustom()">RUN</button>
      </div>
    </div>
  </div>

  <div class="card">
    <span class="card-title">LIVE TELEMETRY</span>
    <div class="data-grid">
      <div class="data-row"><span class="label">Temperature</span><span class="val" id="tc">--</span></div>
      <div class="data-row"><span class="label">Humidity</span><span class="val" id="hum">--</span></div>
      <div class="data-row"><span class="label">Gas Resistance</span><span class="val" id="gas">--</span></div>
      <div class="data-row"><span class="label">Latitude</span><span class="val" id="lat">--</span></div>
      <div class="data-row"><span class="label">Longitude</span><span class="val" id="lon">--</span></div>
    </div>
  </div>

  <script>
    function post(cmd) { fetch('/control', {method:'POST', body:cmd}); }
    function sendCustom() { const val=document.getElementById('custom').value; if(val) post(val); }

    // Update loop
    setInterval(() => {
      fetch('/metrics').then(r=>r.json()).then(d=>{
        document.getElementById('tc').innerText = d.temp_c.toFixed(1) + '°C';
        document.getElementById('hum').innerText = d.humidity.toFixed(0) + '%';
        document.getElementById('gas').innerText = (d.gas_kohms).toFixed(0);
      }).catch(e=>{});

      fetch('/coords').then(r=>r.json()).then(d=>{
        document.getElementById('lat').innerText = d.lat;
        document.getElementById('lon').innerText = d.lon;
      }).catch(e=>{});
    }, 2000);
  </script>
</body>
</html>
)rawliteral";

void handleRoot() { server.send(200, "text/html", index_html); }

// ==========================================
// 7. SETUP
// ==========================================
void setup() {
  Serial.begin(115200);
  delay(2000);

  // Init Motors
  pinMode(MOTOR_L_IN1, OUTPUT); pinMode(MOTOR_L_IN2, OUTPUT);
  pinMode(MOTOR_R_IN1, OUTPUT); pinMode(MOTOR_R_IN2, OUTPUT);
  stopMotors();

  // Init GPS
  Serial2.begin(115200, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

  // Init MQ-2
pinMode(MQ2_A_PIN, INPUT);
pinMode(MQ2_D_PIN, INPUT);

Serial.println("MQ-2: Preheating...");
delay(MQ2_PREHEAT_MS);
mq2Ro = mq2Calibrate();
cleanAirBaseline = mq2Ro;
Serial.print("MQ-2 calibrated Ro = ");
Serial.println(mq2Ro, 2);

  // Init WiFi (LOW POWER)
  WiFi.mode(WIFI_AP);
  WiFi.setTxPower(WIFI_POWER_8_5dBm);
  WiFi.softAP(AP_SSID, AP_PASS);
  Serial.println(WiFi.softAPIP());

  // Init BME
  if(bme_begin()) {
    delay(100);
    if(bme_read(bme)) {
      bmeReady = true;
    }
  }

  server.on("/", handleRoot);
  server.on("/metrics", handleMetrics);
  server.on("/coords", handleCoords);
  server.on("/control", HTTP_POST, handleControl);
  server.begin();
}

void loop() {
  server.handleClient();
  updateGPS();
}
