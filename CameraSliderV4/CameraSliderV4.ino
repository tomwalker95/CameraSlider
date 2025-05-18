// === Includes ===
#include <WiFiNINA.h>
#include <WebSocketServer.h>
#include <SAMD21turboPWM.h>
#include <Arduino_LSM6DS3.h>
#include <Base64.h>
#include <RTCZero.h>
#include <FlashStorage.h>
#include "wifi_secrets.h"
#include <AccelStepper.h>
#include <TMC2209.h>
#include "wiring_private.h"
#include <SAMDTimerInterrupt.h>
#include <SAMDTimerInterrupt.hpp>

// === Debug Macros ===
#define DEBUG 1
#ifdef DEBUG
  #define Sprint(a) Serial.print(a)
  #define Sprintln(a) Serial.println(a)
#else
  #define Sprint(a)
  #define Sprintln(a)
#endif

// === Constants and Pins ===
const int ledPin       = 12;
const int statusLed    = 13;
#define STEP_PIN_X       3
#define DIR_PIN_X        2
#define STEP_PIN_Z       5
#define DIR_PIN_Z        4
#define ENABLE_X_PIN     8
#define ENABLE_Z_PIN     9
#define LIMIT_SWITCH_PIN 7    // Normally-closed X-axis limit switch

#define X_SERIAL Serial1
#define Z_SERIAL Serial2

const int webPort    = 80;
const int socketPort = 8080;

// === Globals ===
RTCZero rtc;
TurboPWM pwm;
WiFiServer webServer(webPort);
WiFiServer socketServer(socketPort);
WebSocketServer webSocketServer;
WiFiClient webClient;
WiFiClient socketClient;

TMC2209 driverX, driverZ;
AccelStepper stepperX(AccelStepper::DRIVER, STEP_PIN_X, DIR_PIN_X);
AccelStepper stepperZ(AccelStepper::DRIVER, STEP_PIN_Z, DIR_PIN_Z);

boolean on                = false;
float volume              = 0;

typedef struct {
  boolean settingsSaved_F;
  boolean on_F;
  float   volume_F;
  int     cowbell_F;
} Settings;

FlashStorage(flashStore, Settings);
Settings settings;

// Fault‐tolerance flags
volatile bool clientDisconnectedFlag = false;
volatile bool xLimitTriggered        = false;
volatile bool xLimitPrevState        = true;

// Timer for checking client status
SAMDTimer ITimer(TIMER_TC3);

// === Interrupt and Timer Handlers ===
void handleXLimitInterrupt() {
  bool currentState = digitalRead(LIMIT_SWITCH_PIN);
  // detect rising edge: switch opened (NC → open)
  if (currentState == HIGH && xLimitPrevState == true) {
    xLimitTriggered = true;
    // stop and disable X axis
    digitalWrite(ENABLE_X_PIN, HIGH);
    stepperX.stop();
    Sprintln("--X limit switch triggered: X axis disabled");
  }
  xLimitPrevState = currentState;
}

// <-- CHANGED: timer callback must be `void` with no args -->
void onTimer() {
  if (!socketClient.connected()) {
    clientDisconnectedFlag = true;
  }
}

// === Setup ===
void setup() {
  #ifdef DEBUG
    Serial.begin(115200);
    while (!Serial);
  #endif

  // --- X axis setup ---
  X_SERIAL.begin(115200);
  driverX.setup(X_SERIAL);
  driverX.enable();
  driverX.setRunCurrent(75);
  Sprintln("Driver X ready");

  stepperX.setMaxSpeed(800);
  stepperX.setAcceleration(400);
  pinMode(ENABLE_X_PIN, OUTPUT);
  digitalWrite(ENABLE_X_PIN, LOW);

  // --- Z axis setup ---
  Z_SERIAL.begin(115200);
  driverZ.setup(Z_SERIAL);
  driverZ.enable();
  driverZ.setRunCurrent(75);
  Sprintln("Driver Z ready");

  stepperZ.setMaxSpeed(800);
  stepperZ.setAcceleration(400);
  pinMode(ENABLE_Z_PIN, OUTPUT);
  digitalWrite(ENABLE_Z_PIN, LOW);

  // --- Limit switch interrupt ---
  pinMode(LIMIT_SWITCH_PIN, INPUT_PULLUP);
  xLimitPrevState = digitalRead(LIMIT_SWITCH_PIN);
  attachInterrupt(digitalPinToInterrupt(LIMIT_SWITCH_PIN),
                  handleXLimitInterrupt, CHANGE);

  // --- Timer to monitor WebSocket client ---
  // <-- PASS onTimer, not a bool-returning function -->
  if (ITimer.attachInterruptInterval(200000, onTimer)) {
    Sprintln("--Client monitor timer started");
  } else {
    Sprintln("--Failed to start client monitor timer");
  }

  Sprintln("--Webinterface (Access Point version) started");
  pinMode(ledPin, OUTPUT);

  pwm.setClockDivider(1, false);
  pwm.timer(2, 64, 0xFFFF, false);
  pwm.enable(2, true);

  IMU.begin();
  rtc.begin();

  settings = flashStore.read();
  if (settings.settingsSaved_F) {
    on = settings.on_F;
  }

  const char* ssid      = SECRET_SSID;
  const char* pass      = SECRET_PASS;
  const IPAddress apIp(192,168,2,1);
  const int apChannel   = 13;

  Sprint("\n--Setting up Access Point "); Sprint(ssid); Sprintln(" ...");
  WiFi.setHostname("nano");
  WiFi.config(apIp, apIp, apIp, IPAddress(255,255,255,0));
  int apStatus = WiFi.beginAP(ssid, pass, apChannel);
  if (apStatus != WL_AP_LISTENING) {
    Sprint("\n--Setting up Access Point "); Sprint(ssid); Sprintln(" failed");
    pwm.enable(0, false);
    return;
  }

  webServer.begin();
  socketServer.begin();

  #ifdef DEBUG
    printWifiStatus();
  #endif

  Sprint("--Access Point "); Sprint(ssid); Sprintln(" set up");
  pwm.analogWrite(statusLed, 1000);
}

// === Main Loop ===
void loop() {
  // [HTTP + WebSocket handling unchanged…]
  // … your existing HTTP and WS code …

  // === Fault checks before running motors ===
  if (clientDisconnectedFlag) {
    digitalWrite(ENABLE_X_PIN, HIGH);
    digitalWrite(ENABLE_Z_PIN, HIGH);
    stepperX.stop();
    stepperZ.stop();
    clientDisconnectedFlag = false;
    Sprintln("--Client disconnected: motors stopped and disabled");
  }

  if (xLimitTriggered) {
    // Prevent positive motion on X-axis when limit is active
    if (stepperX.speed() > 0) {
      stepperX.setSpeed(0);
      stepperX.stop();
      Sprintln("--X axis forward motion blocked by limit switch");
    }
  }

  stepperX.run();
  stepperZ.run();
}

// === Debug Status Output ===
#ifdef DEBUG
void printWifiStatus() {
  Sprint("Access point SSID: "); Sprintln(WiFi.SSID());
  Sprint("IP address: ");      Sprintln(WiFi.localIP());
  Sprint("Gateway: ");         Sprintln(WiFi.gatewayIP());
  Sprint("Netmask: ");         Sprintln(WiFi.subnetMask());
  Sprint("Webserver at http://"); Sprint(WiFi.localIP()); Sprint(":"); Sprint(webPort); Sprintln("/");
  Sprint("Websocket at ws://"); Sprint(WiFi.localIP()); Sprint(":"); Sprint(socketPort); Sprintln("/");
}
#endif
