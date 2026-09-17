#define BLYNK_TEMPLATE_ID "TMPL3RURiy3lw"
#define BLYNK_TEMPLATE_NAME "Smart Energy Meter"
#define BLYNK_AUTH_TOKEN "xPhWcatdukKAlqGT_DIMf5Xe3559Az0a"

#include <Wire.h>
#include <WiFi.h>
#include <LiquidCrystal_I2C.h>
#include <BlynkSimpleEsp32.h>

char ssid[] = "Wokwi-GUEST";
char pass[] = "";

LiquidCrystal_I2C lcd(0x27, 16, 2);

// Potentiometer Pins (Battery Cells)
const int cell1Pin = 35;
const int cell2Pin = 34;
const int cell3Pin = 33;
const int cell4Pin = 32;

const int buzzerPin = 25;
const int relayPin = 26;

// Battery Variables
float cell1, cell2, cell3, cell4;
float averageVoltage;
float highestVoltage;
float lowestVoltage;
float imbalance;
String batteryHealth;
float imbalancePercent = 0.0;

int weakestCell = 1;
int strongestCell = 1;

float weakestVoltage = 0.0;
float strongestVoltage = 0.0;

String faultStatus="Normal";

bool relayState = LOW;
unsigned long lastRelayChangeTime = 0;

const unsigned long RELAY_MIN_INTERVAL = 3000;

// ===== LCD Diagnostic Screens =====
unsigned long lastLCDUpdate = 0;
const unsigned long LCD_INTERVAL = 2500;

int lcdScreen = 0;
const int TOTAL_LCD_SCREENS = 4;

// Timer (No delay())
unsigned long previousMillis = 0;

// Operating modes
enum SystemMode {
  NORMAL,
  DEGRADED,
  FAILSAFE,
  SHUTDOWN
};

SystemMode systemMode = NORMAL;

// Fault information
String activeFault = "None";
unsigned long faultTimestamp = 0;

// Fault counters
int sensorFaultCount = 0;
int invalidReadingCount = 0;
int frozenADCCount = 0;
int relayMismatchCount = 0;

// ===== Relay Feedback Monitoring =====
bool expectedRelayState = LOW;
unsigned long lastRelayCheck = 0;

const unsigned long RELAY_CHECK_INTERVAL = 2000;
const int MAX_RELAY_MISMATCH = 3;

// Recovery variables
unsigned long recoveryStartTime = 0;
bool recoveryInProgress = false;

// Previous ADC values for frozen-value detection
int previousADC1 = -1;
int previousADC2 = -1;
int previousADC3 = -1;
int previousADC4 = -1;

// Number of consecutive identical ADC readings
int frozenCount1 = 0;
int frozenCount2 = 0;
int frozenCount3 = 0;
int frozenCount4 = 0;

bool validVoltage(float v) {
  return (v >= 2.5 && v <= 4.3);
}

void checkSensorHealth() {

  bool invalidCell =
    !validVoltage(cell1) ||
    !validVoltage(cell2) ||
    !validVoltage(cell3) ||
    !validVoltage(cell4);

  if (invalidCell) {
    invalidReadingCount++;
    logFault("Invalid cell voltage");

    if (invalidReadingCount >= 5) {
      changeMode(FAILSAFE);
    }
    else {
      changeMode(DEGRADED);
    }
  }
  else {
    invalidReadingCount = 0;
  }
}


// ---------- Frozen ADC Detection ----------

void checkFrozenADC() {

  int adc1 = analogRead(cell1Pin);
  int adc2 = analogRead(cell2Pin);
  int adc3 = analogRead(cell3Pin);
  int adc4 = analogRead(cell4Pin);

  // Ignore first reading
  if (previousADC1 == -1) {
    previousADC1 = adc1;
    previousADC2 = adc2;
    previousADC3 = adc3;
    previousADC4 = adc4;
    return;
  }

  // Check whether ADC values remain exactly unchanged
  if (adc1 == previousADC1)
    frozenCount1++;
  else
    frozenCount1 = 0;

  if (adc2 == previousADC2)
    frozenCount2++;
  else
    frozenCount2 = 0;

  if (adc3 == previousADC3)
    frozenCount3++;
  else
    frozenCount3 = 0;

  if (adc4 == previousADC4)
    frozenCount4++;
  else
    frozenCount4 = 0;

  previousADC1 = adc1;
  previousADC2 = adc2;
  previousADC3 = adc3;
  previousADC4 = adc4;

  // Require a longer stable period before declaring a fault
  if (frozenCount1 >= 60 ||
      frozenCount2 >= 60 ||
      frozenCount3 >= 60 ||
      frozenCount4 >= 60) {

    if (activeFault != "Frozen ADC detected") {
      frozenADCCount++;
      logFault("Frozen ADC detected");
      changeMode(DEGRADED);
    }
  }
}

void setRelayState(bool newState) {

  // Change relay only if required
  if (newState == relayState) {
    return;
  }

  // Prevent rapid ON/OFF switching
  if (millis() - lastRelayChangeTime < RELAY_MIN_INTERVAL) {
    return;
  }

  relayState = newState;
  digitalWrite(relayPin, relayState);

  lastRelayChangeTime = millis();

  Serial.print("[RELAY] State changed: ");
  Serial.println(relayState ? "ON" : "OFF");
}

// ===== Relay Mismatch Detection =====
void checkRelayMismatch() {

  if (millis() - lastRelayCheck < RELAY_CHECK_INTERVAL) {
    return;
  }

  lastRelayCheck = millis();

  int actualRelayState = digitalRead(relayPin);

  if (actualRelayState != expectedRelayState) {

    relayMismatchCount++;

    Serial.print("[RELAY] Mismatch detected: ");
    Serial.println(relayMismatchCount);

    if (relayMismatchCount >= MAX_RELAY_MISMATCH) {

      activeFault = "Relay Mismatch";
      logFault("Relay mismatch detected");
      changeMode(FAILSAFE);

      digitalWrite(relayPin, HIGH);
    }

  } else {

    relayMismatchCount = 0;
  }
}

// ---------- Safety State Management ----------

void updateSafetyState() {

  if (batteryHealth == "Failure") {

    logFault("Battery failure");
    changeMode(SHUTDOWN);

    expectedRelayState = HIGH;
    setRelayState(HIGH);
    digitalWrite(buzzerPin, HIGH);

    return;
  }

  if (batteryHealth == "Critical") {

    logFault("Critical battery condition");
    changeMode(FAILSAFE);

    expectedRelayState = HIGH;
    setRelayState(HIGH);
    digitalWrite(buzzerPin, HIGH);

    return;
  }

  if (batteryHealth == "Minor") {

    changeMode(DEGRADED);

    expectedRelayState = LOW;
    setRelayState(LOW);
    digitalWrite(buzzerPin, LOW);

    return;
  }

  // All readings healthy
  if (batteryHealth == "Healthy") {

    changeMode(NORMAL);

    expectedRelayState = LOW;
    setRelayState(LOW);
    digitalWrite(buzzerPin, LOW);

    activeFault = "None";
  }
}


// ---------- Recovery ----------

void checkRecovery() {

  if (systemMode == DEGRADED ||
      systemMode == FAILSAFE) {

    if (batteryHealth == "Healthy" &&
        validVoltage(cell1) &&
        validVoltage(cell2) &&
        validVoltage(cell3) &&
        validVoltage(cell4)) {

      if (!recoveryInProgress) {
        recoveryInProgress = true;
        recoveryStartTime = millis();

        Serial.println("[RECOVERY] Stability check started");
      }

      if (millis() - recoveryStartTime >= 5000) {

        changeMode(NORMAL);
        activeFault = "None";
        recoveryInProgress = false;

        Serial.println("[RECOVERY] System recovered");
      }

    }
    else {
      recoveryInProgress = false;
    }
  }
}

// Fault logging function
void logFault(String fault) {
  activeFault = fault;
  faultTimestamp = millis();

  Serial.print("[FAULT] Time: ");
  Serial.print(faultTimestamp);
  Serial.print(" ms | Fault: ");
  Serial.println(fault);
}

// Change system operating mode
void changeMode(SystemMode newMode) {
  if (systemMode != newMode) {
    systemMode = newMode;

    Serial.print("[MODE] System changed to: ");

    if (systemMode == NORMAL)
      Serial.println("NORMAL");
    else if (systemMode == DEGRADED)
      Serial.println("DEGRADED");
    else if (systemMode == FAILSAFE)
      Serial.println("FAILSAFE");
    else if (systemMode == SHUTDOWN)
      Serial.println("SHUTDOWN");
  }
}

// Display current system mode
String getModeName() {
  if (systemMode == NORMAL)
    return "NORMAL";
  else if (systemMode == DEGRADED)
    return "DEGRADED";
  else if (systemMode == FAILSAFE)
    return "FAILSAFE";
  else
    return "SHUTDOWN";
}
const long interval = 2000;
void readBattery() {

  cell1 = analogRead(cell1Pin) * 4.2 / 4095.0;
  cell2 = analogRead(cell2Pin) * 4.2 / 4095.0;
  cell3 = analogRead(cell3Pin) * 4.2 / 4095.0;
  cell4 = analogRead(cell4Pin) * 4.2 / 4095.0;

  averageVoltage = (cell1 + cell2 + cell3 + cell4) / 4.0;

  highestVoltage = max(max(cell1, cell2), max(cell3, cell4));
  lowestVoltage = min(min(cell1, cell2), min(cell3, cell4));

  imbalance = highestVoltage - lowestVoltage;
  imbalancePercent=0.0;

if (averageVoltage > 0) {
  imbalancePercent = (imbalance / averageVoltage) * 100.0;
}

// Weakest cell
weakestVoltage = cell1;
weakestCell = 1;

if (cell2 < weakestVoltage) {
  weakestVoltage = cell2;
  weakestCell = 2;
}

if (cell3 < weakestVoltage) {
  weakestVoltage = cell3;
  weakestCell = 3;
}

if (cell4 < weakestVoltage) {
  weakestVoltage = cell4;
  weakestCell = 4;
}

// Strongest cell
strongestVoltage = cell1;
strongestCell = 1;

if (cell2 > strongestVoltage) {
  strongestVoltage = cell2;
  strongestCell = 2;
}

if (cell3 > strongestVoltage) {
  strongestVoltage = cell3;
  strongestCell = 3;
}

if (cell4 > strongestVoltage) {
  strongestVoltage = cell4;
  strongestCell = 4;
}
  if (imbalance < 0.10)
    {batteryHealth = "Healthy";
     faultStatus = "No Fault";}
  else if (imbalance < 0.30)
    {batteryHealth = "Minor";
     faultStatus = "Minor Imbalance";}
  else if (imbalance < 0.60)
    {batteryHealth = "Critical";
     faultStatus = "Critical";}
  else
    {batteryHealth = "Failure";
     faultStatus = "Pack Failure";}
}

// ===== Protection Limit Detection =====
void checkProtectionLimits() {

  // Weak cell detection
  if (weakestVoltage < 3.00) {
    activeFault = "Weak Cell";
    logFault("Weak cell detected");
    changeMode(DEGRADED);
    return;
  }

  // Over-voltage detection
  if (highestVoltage > 4.20) {
    activeFault = "Overvoltage";
    logFault("Overvoltage detected");
    changeMode(FAILSAFE);
    return;
  }
}

// ===== Rapid Voltage Fluctuation Detection =====
void checkRapidVoltageChange() {

  static float previousAverage = 0.0;
  static bool firstReading = true;

  if (firstReading) {
    previousAverage = averageVoltage;
    firstReading = false;
    return;
  }

  float voltageChange = abs(averageVoltage - previousAverage);

  // Rapid change threshold = 0.20 V
  if (voltageChange >= 0.20) {
    activeFault = "Rapid Voltage Change";
    logFault("Rapid voltage fluctuation detected");
    changeMode(DEGRADED);
  }

  previousAverage = averageVoltage;
}

// ===== Automatic Rotating LCD =====
void updateLCD() {

  unsigned long currentMillis = millis();

  // Update LCD only after fixed interval
  if (currentMillis - lastLCDUpdate < LCD_INTERVAL) {
    return;
  }

  lastLCDUpdate = currentMillis;

  // Fault has priority
  if (activeFault != "None" && activeFault != "") {

    lcd.clear();

    lcd.setCursor(0, 0);
    lcd.print("FAULT:");

    lcd.setCursor(0, 1);
    lcd.print(activeFault.substring(0, 16));

    return;
  }

  // Normal diagnostic screens
  lcd.clear();

  if (lcdScreen == 0) {

    lcd.setCursor(0, 0);
    lcd.print("Smart Energy");

    lcd.setCursor(0, 1);
    lcd.print("System ");
    lcd.print(getModeName());

  }

  else if (lcdScreen == 1) {

    lcd.setCursor(0, 0);
    lcd.print("C1:");
    lcd.print(cell1, 2);
    lcd.print(" C2:");
    lcd.print(cell2, 2);

    lcd.setCursor(0, 1);
    lcd.print("C3:");
    lcd.print(cell3, 2);
    lcd.print(" C4:");
    lcd.print(cell4, 2);

  }

  else if (lcdScreen == 2) {

    lcd.setCursor(0, 0);
    lcd.print("Avg:");
    lcd.print(averageVoltage, 2);
    lcd.print("V");

    lcd.setCursor(0, 1);
    lcd.print("Imbal:");
    lcd.print(imbalancePercent, 1);
    lcd.print("%");

  }

  else if (lcdScreen == 3) {

    lcd.setCursor(0, 0);
    lcd.print("Health:");

    lcd.setCursor(0, 1);
    lcd.print(batteryHealth.substring(0, 16));

  }

  // Move to next screen
  lcdScreen++;

  if (lcdScreen >= TOTAL_LCD_SCREENS) {
    lcdScreen = 0;
  }
}
void setup() {
  Serial.begin(115200);
  Serial.println("Starting WiFI...");
  WiFi.begin(ssid,pass);
  Blynk.config(BLYNK_AUTH_TOKEN);
  Serial.println("WiFi/Blynk initialization done");

  Wire.begin(21, 22);
  lcd.init();
  lcd.backlight();

  pinMode(cell1Pin, INPUT);
  pinMode(cell2Pin, INPUT);
  pinMode(cell3Pin, INPUT);
  pinMode(cell4Pin, INPUT);

  pinMode(buzzerPin, OUTPUT);
  pinMode(relayPin, OUTPUT);

  digitalWrite(buzzerPin, LOW);
  digitalWrite(relayPin, LOW);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Smart Energy");
  lcd.setCursor(0, 1);
  lcd.print("Initializing");

}

// ===== Network Status Monitoring =====
String lastNetworkStatus = "";

void checkNetworkStatus() {

  String currentStatus;

  if (WiFi.status() != WL_CONNECTED) {
    currentStatus = "WiFi Offline";
  }
  else if (!Blynk.connected()) {
    currentStatus = "Cloud Offline";
  }
  else {
    currentStatus = "Cloud Online";
  }

  if (currentStatus != lastNetworkStatus) {

    lastNetworkStatus = currentStatus;

    Serial.print("[NETWORK] ");
    Serial.println(currentStatus);

    addEvent(currentStatus);
  }
}

void handleNetwork() {
  static unsigned long lastReconnectAttempt = 0;

  if (WiFi.status() != WL_CONNECTED) {
    if (millis() - lastReconnectAttempt >= 5000) {
      lastReconnectAttempt = millis();

      Serial.println("[WIFI] Reconnecting...");
      //WiFi.disconnect();
      WiFi.begin(ssid, pass);
    }
    return;
  }

  if (!Blynk.connected()) {
    if (millis() - lastReconnectAttempt >= 5000) {
      lastReconnectAttempt = millis();

      Serial.println("[BLYNK] Reconnecting...");
      Blynk.connect(3000);
      Serial.print("[BLYNK] Status: ");
      Serial.println(Blynk.connected() ? "CONNECTED" : "FAILED"); 
    }
  }
}
void updateWiFiRSSI() {
  if (WiFi.status() == WL_CONNECTED && Blynk.connected()) {
    int rssi = WiFi.RSSI();
    Blynk.virtualWrite(V8, rssi);

    Serial.print("[WIFI] RSSI: ");
    Serial.print(rssi);
    Serial.println(" dBm");
  }
}
// ================= EVENT QUEUE =================

const int MAX_EVENTS = 10;

String eventQueue[MAX_EVENTS];
int queueHead = 0;
int queueTail = 0;
int queueCount = 0;

void addEvent(String eventText) {

  if (queueCount < MAX_EVENTS) {

    eventQueue[queueTail] = eventText;
    queueTail = (queueTail + 1) % MAX_EVENTS;
    queueCount++;

    Serial.print("[QUEUE] Event stored: ");
    Serial.println(eventText);
  }
  else {
    Serial.println("[QUEUE] Queue full - oldest event overwritten");

    eventQueue[queueTail] = eventText;
    queueTail = (queueTail + 1) % MAX_EVENTS;
    queueHead = (queueHead + 1) % MAX_EVENTS;
  }
}


void processEventQueue() {

  if (!Blynk.connected())
    return;

  while (queueCount > 0) {

    String eventText = eventQueue[queueHead];

    Serial.print("[QUEUE] Sending: ");
    Serial.println(eventText);

    // Send event information to Blynk
    Blynk.virtualWrite(V6, eventText);

    queueHead = (queueHead + 1) % MAX_EVENTS;
    queueCount--;

    Serial.println("[QUEUE] Event synchronized");
  }
}
void monitorWiFiSignal() {

  static unsigned long lastSignalCheck = 0;

  if (millis() - lastSignalCheck < 10000)
    return;

  lastSignalCheck = millis();

  if (WiFi.status() == WL_CONNECTED) {

    int rssi = WiFi.RSSI();

    Serial.print("[WIFI] RSSI: ");
    Serial.print(rssi);
    Serial.println(" dBm");

    String signalQuality;

    if (rssi >= -60) {
      signalQuality = "GOOD";
    }
    else if (rssi >= -75) {
      signalQuality = "FAIR";
    }
    else {
      signalQuality = "POOR";
    }

    Serial.print("[WIFI] Signal: ");
    Serial.println(signalQuality);

    // Send signal quality to Blynk
    if (Blynk.connected()) {
      Blynk.virtualWrite(V8, rssi);
    }
  }
}
String lastSentHealth = "";
String lastSentFault = "";
String lastSentMode = "";

float lastSentAverage = -1;

void sendTelemetryEvent() {

  bool healthChanged = (batteryHealth != lastSentHealth);
  bool faultChanged = (activeFault != lastSentFault);
  bool modeChanged = (getModeName() != lastSentMode);

  bool voltageChanged =
    (lastSentAverage < 0 ||
     abs(averageVoltage - lastSentAverage) >= 0.10);

  if (!healthChanged &&
      !faultChanged &&
      !modeChanged &&
      !voltageChanged) {
    return;
  }

  // -------------------------------
  // BLYNK DISCONNECTED
  // -------------------------------
  if (!Blynk.connected()) {

    String eventText =
      getModeName() + " | " + activeFault;

    addEvent(eventText);

    // Remember this event so it is not
    // added repeatedly every 2 seconds
    lastSentHealth = batteryHealth;
    lastSentFault = activeFault;
    lastSentMode = getModeName();
    lastSentAverage = averageVoltage;

    Serial.println("[CLOUD] Blynk offline - event queued");

    return;
  }

  // -------------------------------
  // BLYNK CONNECTED
  // -------------------------------

  Blynk.virtualWrite(V0, cell1);
  Blynk.virtualWrite(V1, cell2);
  Blynk.virtualWrite(V2, cell3);
  Blynk.virtualWrite(V3, cell4);
  Blynk.virtualWrite(V4, averageVoltage);
  Blynk.virtualWrite(V5, batteryHealth);
  Blynk.virtualWrite(V6, activeFault);
  Blynk.virtualWrite(V7, digitalRead(relayPin));
  String riskLevel;

if (batteryHealth == "Healthy") {
  riskLevel = "LOW RISK";
}
else if (batteryHealth == "Minor") {
  riskLevel = "MEDIUM RISK";
}
else {
  riskLevel = "HIGH RISK";
}

Blynk.virtualWrite(V9, riskLevel);

  lastSentHealth = batteryHealth;
  lastSentFault = activeFault;
  lastSentMode = getModeName();
  lastSentAverage = averageVoltage;

  Serial.println("[CLOUD] Telemetry event sent");
}
void loop() {
  handleNetwork();
  checkNetworkStatus();
  static unsigned long lastRSSI = 0;

if (millis() - lastRSSI >= 10000) {
  lastRSSI = millis();
  updateWiFiRSSI();
}
  Blynk.run();
  processEventQueue();
  monitorWiFiSignal();
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
  previousMillis = currentMillis;
  readBattery();

  checkSensorHealth();
  checkFrozenADC();

  checkProtectionLimits();
  checkRapidVoltageChange();
  
  updateSafetyState();
  checkRelayMismatch();
  checkRecovery();
  sendTelemetryEvent();
  updateLCD();
  if (batteryHealth == "Healthy") {
  setRelayState(LOW);
  noTone(buzzerPin);
}
else if (batteryHealth == "Minor") {
  setRelayState(LOW);
  noTone(buzzerPin);
}
else if (batteryHealth == "Critical") {
  setRelayState(LOW);
  tone(buzzerPin,1000);
}
else if (batteryHealth == "Failure") {
  setRelayState(HIGH);
  tone(buzzerPin,2000);
}

  Serial.print("Cell1: ");
  Serial.print(cell1);

  Serial.print(" Cell2: ");
  Serial.print(cell2);

  Serial.print(" Cell3: ");
  Serial.print(cell3);

  Serial.print(" Cell4: ");
  Serial.println(cell4);

  //noTone(buzzerPin);
}
}
