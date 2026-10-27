
#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <MycilaPZEM.h>
#include <math.h>

// =====================================================
// SMARTVOLT V2 - POWER ANOMALY DETECTOR
// Experimental monitoring only; not certified protection.
// =====================================================

// ---------- PINS ----------
#define SDA_PIN       21
#define SCL_PIN       22
#define PZEM_RX_PIN   16
#define PZEM_TX_PIN   17
#define RELAY_PIN     26
#define BUZZER_PIN    25  // Active buzzer signal pin; use a driver if required by the buzzer current
#define LED_GREEN_PIN 18  // Normal, armed operation
#define LED_YELLOW_PIN 19 // Learning, startup, warning, or not ready
#define LED_ALERT_PIN 27  // 2-pin LED: trip indicator; its actual color/behavior must be verified

#define RELAY_ON      LOW
#define RELAY_OFF     HIGH

#define OLED_ADDRESS  0x3C
#define ADS_ADDRESS   0x48

// ---------- DETECTOR SETTINGS ----------
const uint16_t LEARNING_SAMPLES = 30;
const uint8_t CONFIRM_SAMPLES = 3;
const uint8_t RECOVERY_SAMPLES = 3;

const float ANOMALY_SIGMA = 3.0f;
const float MIN_ANOMALY_DEVIATION_W = 5.0f;
const float LOAD_OFF_POWER_W = 1.0f;

// DEMONSTRATION MODE: an abnormality confirmation commands the relay OFF.
// This is a prototype demo, not certified electrical protection.
const bool AUTO_TRIP_ON_ANOMALY =true; // Only the intentional demo command may trip the relay.

// Experimental CT residual-current trip. These values are for controlled
// bench demonstration only; calibration/noise are not yet validated.
const bool AUTO_TRIP_ON_CT_DIFFERENCE = true; // CT readings are diagnostic only until validated.
const float CT_RESIDUAL_THRESHOLD_A = 0.030f; // 30 mA indicated difference
const uint8_t CT_CONFIRM_WINDOWS = 5;          // require 5 consecutive clean windows

const unsigned long MIN_SAMPLE_INTERVAL_MS = 1000;
const unsigned long SENSOR_TIMEOUT_MS = 5000;
const unsigned long RELAY_SETTLE_MS = 3000;

const unsigned long DISPLAY_INTERVAL_MS = 500;
const unsigned long CT_INTERVAL_MS = 1000;
const unsigned long REPORT_INTERVAL_MS = 2000;
const unsigned long PAGE_INTERVAL_MS = 4000;

// ---------- BUZZER ----------
const unsigned long ANOMALY_BEEP_MS = 180;
unsigned long buzzerBeepUntilMs = 0;
bool buzzerOutputOn = false;

// ---------- HARDWARE ----------
Adafruit_SSD1306 display(128, 64, &Wire, -1);
Adafruit_ADS1115 ads;
Mycila::PZEM pzem;

// ---------- PZEM ----------
volatile bool pzemUpdated = false;
Mycila::PZEM::Data pzemData;

bool pzemDataAvailable = false;
unsigned long lastValidPzemMs = 0;
unsigned long lastLearningSampleMs = 0;

// ---------- RELAY ----------
bool relayOn = false;
unsigned long lastRelayChangeMs = 0;

// ---------- CT SENSING: UNCALIBRATED / EXPERIMENTAL ----------
const float CT1_AMPS_PER_VRMS = 101.1f;
const float CT2_AMPS_PER_VRMS = 107.1f;
const float ADS_VOLTS_PER_COUNT = 0.000125f;
const int SAMPLE_COUNT = 300;  // Longer window for a steadier RMS estimate

struct CTResult {
  float biasVoltage;
  float acVrms;
  float currentA;
  float rmsCounts;
  int16_t minRaw;
  int16_t maxRaw;
  uint16_t rejectedSamples;
  uint16_t acceptedSamples;
  bool spikeDetected;
};

CTResult ctIn = {};
CTResult ctOut = {};

unsigned long lastCTRead = 0;
unsigned long lastDisplay = 0;
unsigned long lastReport = 0;

// ---------- DETECTOR STATES ----------
enum DetectorState {
  LEARNING,
  NORMAL,
  ANOMALY,
  LOAD_OFF,
  TRANSITION,
  SENSOR_ERROR
};

DetectorState detectorState = LEARNING;

uint16_t learningCount = 0;
double powerMean = 0.0;
double powerM2 = 0.0;
double powerVariance = 0.0;
double powerStdDev = 0.0;

float latestPowerW = 0.0f;
float anomalyThresholdW = MIN_ANOMALY_DEVIATION_W;

bool modelReady = false;

uint8_t anomalyCandidateCount = 0;
uint8_t recoveryCount = 0;

bool anomalyEpisodeActive = false;
bool tripLatched = false;
bool demoAnomalyInjected = false;
bool ctAnomalyTrip = false;
uint8_t ctResidualCandidateCount = 0;
float latestCTDifferenceA = NAN;
unsigned long anomalyCount = 0;

// =====================================================
// BUZZER CONTROL (non-blocking)
// =====================================================

void beepForAnomaly() {
  // A short beep is requested for each out-of-range power reading.
  // If the trip is latched, updateBuzzer() keeps the buzzer continuously ON.
  if (!tripLatched) {
    buzzerBeepUntilMs = millis() + ANOMALY_BEEP_MS;
  }
}

void updateBuzzer() {
  bool shouldBeOn = tripLatched ||
                    (buzzerBeepUntilMs != 0 &&
                     (long)(buzzerBeepUntilMs - millis()) > 0);

  if (shouldBeOn != buzzerOutputOn) {
    buzzerOutputOn = shouldBeOn;
    digitalWrite(BUZZER_PIN, buzzerOutputOn ? HIGH : LOW);
  }

  if (!tripLatched && buzzerBeepUntilMs != 0 &&
      (long)(millis() - buzzerBeepUntilMs) >= 0) {
    buzzerBeepUntilMs = 0;
  }
}

// =====================================================
// STATUS LEDS (one LED at a time)
// =====================================================
void updateStatusLEDs() {
  bool alertOn = tripLatched;
  bool greenOn = !tripLatched && relayOn && modelReady &&
                 detectorState == NORMAL &&
                 lastValidPzemMs != 0 &&
                 (millis() - lastValidPzemMs <= SENSOR_TIMEOUT_MS) &&
                 (millis() - lastRelayChangeMs >= RELAY_SETTLE_MS);
  // Yellow is the safe default whenever the system is not confirmed normal.
  bool yellowOn = !alertOn && !greenOn;

  digitalWrite(LED_ALERT_PIN, alertOn ? HIGH : LOW);
  digitalWrite(LED_GREEN_PIN, greenOn ? HIGH : LOW);
  digitalWrite(LED_YELLOW_PIN, yellowOn ? HIGH : LOW);
}

// =====================================================
// STATE TEXT
// =====================================================

const char* getStateText() {
  if (tripLatched) {
    if (demoAnomalyInjected) return "DEMO TRIP LATCHED";
    if (ctAnomalyTrip) return "CT TRIP LATCHED";
    return "POWER TRIP LATCHED";
  }

  // During relay settling, report TRANSITION rather than immediately
  // labeling the transient PZEM response as a persistent sensor fault.
  if (millis() - lastRelayChangeMs < RELAY_SETTLE_MS) {
    return "TRANSITION";
  }

  // Be explicit when the relay is commanded OFF but the PZEM data is
  // invalid/stale. This does NOT assume that the PZEM is wired downstream;
  // if it is upstream, its voltage should normally remain measurable.
  if (lastValidPzemMs == 0 ||
      millis() - lastValidPzemMs > SENSOR_TIMEOUT_MS) {
    return relayOn ? "SENSOR ERROR" : "OFF / PZEM INVALID";
  }

  switch (detectorState) {
    case LEARNING:     return "LEARNING";
    case NORMAL:       return "NORMAL";
    case ANOMALY:      return "ANOMALY";
    case LOAD_OFF:     return "LOAD OFF";
    case TRANSITION:   return "TRANSITION";
    case SENSOR_ERROR: return "SENSOR ERROR";
    default:           return "UNKNOWN";
  }
}

// =====================================================
// CT / ADS1115 DIAGNOSTICS
// =====================================================

CTResult readCT(int channel, float calibration) {
  CTResult result = {};
  double sum = 0.0;
  double sumSquares = 0.0;
  int16_t previousRaw = 0;
  bool havePrevious = false;

  result.minRaw = 32767;
  result.maxRaw = -32768;

  // Read one continuous window. Reject only abrupt, implausible single-step
  // jumps. CT measurements are diagnostic only while calibration is unverified.
  // 200 ADC counts is intentionally conservative relative to the observed
  // normal signal (~5 RMS counts); verify this threshold with real logs.
  const int16_t SPIKE_DELTA_COUNTS = 200;

  for (int i = 0; i < SAMPLE_COUNT; i++) {
    int16_t raw = ads.readADC_SingleEnded(channel);

    if (havePrevious &&
        abs((int)raw - (int)previousRaw) > SPIKE_DELTA_COUNTS) {
      result.rejectedSamples++;
      result.spikeDetected = true;
      previousRaw = raw;
      continue;
    }

    previousRaw = raw;
    havePrevious = true;

    double sample = (double)raw;
    sum += sample;
    sumSquares += sample * sample;
    result.acceptedSamples++;

    if (raw < result.minRaw) result.minRaw = raw;
    if (raw > result.maxRaw) result.maxRaw = raw;
  }

  if (result.acceptedSamples < (SAMPLE_COUNT * 9) / 10) {
    // Too many rejected samples: mark this window invalid rather than
    // presenting a potentially misleading current estimate.
    result.biasVoltage = NAN;
    result.rmsCounts = NAN;
    result.acVrms = NAN;
    result.currentA = NAN;
    return result;
  }

  double mean = sum / result.acceptedSamples;
  double meanSquare = sumSquares / result.acceptedSamples;
  double variance = meanSquare - (mean * mean);

  if (variance < 0.0) variance = 0.0;

  result.biasVoltage = mean * ADS_VOLTS_PER_COUNT;
  result.rmsCounts = sqrt(variance);
  result.acVrms = result.rmsCounts * ADS_VOLTS_PER_COUNT;
  result.currentA = result.acVrms * calibration;

  return result;
}

// =====================================================
// PZEM CALLBACK
// =====================================================

void onPzemEvent(
  const Mycila::PZEM::EventType event,
  const Mycila::PZEM::Data &data
) {
  if (event == Mycila::PZEM::EventType::EVT_READ) {
    pzemData = data;
    pzemUpdated = true;
  }
}

// =====================================================
// PZEM VALIDATION
// =====================================================

bool isValidPzemReading(const Mycila::PZEM::Data &data) {
  if (!isfinite(data.voltage) ||
      !isfinite(data.current) ||
      !isfinite(data.activePower)) {
    return false;
  }

  // Broad plausibility limits for this prototype.
  if (data.voltage < 180.0f || data.voltage > 260.0f) {
    return false;
  }

  if (data.current < 0.0f || data.current > 100.0f) {
    return false;
  }

  if (data.activePower < 0.0f ||
      data.activePower > 25000.0f) {
    return false;
  }

  return true;
}

// =====================================================
// RESET LEARNING
// =====================================================

void resetLearning() {
  learningCount = 0;

  powerMean = 0.0;
  powerM2 = 0.0;
  powerVariance = 0.0;
  powerStdDev = 0.0;

  anomalyThresholdW = MIN_ANOMALY_DEVIATION_W;

  modelReady = false;

  anomalyCandidateCount = 0;
  recoveryCount = 0;
  anomalyEpisodeActive = false;
  anomalyCount = 0;

  lastLearningSampleMs = 0;
  detectorState = LEARNING;

  Serial.println(
    "[LEARNING] Reset. Keep one steady appliance operating."
  );
}

// =====================================================
// ONLINE LEARNING: WELFORD ALGORITHM
// =====================================================

void addLearningSample(float powerW) {
  learningCount++;

  double delta = powerW - powerMean;
  powerMean += delta / learningCount;

  double delta2 = powerW - powerMean;
  powerM2 += delta * delta2;

  if (learningCount > 1) {
    powerVariance = powerM2 / (learningCount - 1);

    if (powerVariance < 0.0) {
      powerVariance = 0.0;
    }

    powerStdDev = sqrt(powerVariance);
  }

  Serial.printf(
    "[LEARNING] %u/%u | Power: %.2f W | Mean: %.2f W\n",
    learningCount,
    LEARNING_SAMPLES,
    powerW,
    powerMean
  );

  if (learningCount >= LEARNING_SAMPLES) {
    modelReady = true;
    detectorState = NORMAL;

    Serial.printf(
      "[LEARNING] Baseline ready: %.2f W, SD %.2f W\n",
      powerMean,
      powerStdDev
    );
  }
}

// =====================================================
// DEMO AUTO-TRIP
// =====================================================

void tripRelayForAnomaly(bool simulated, bool ctTriggered = false) {
  if (tripLatched) return;

  tripLatched = true;
  demoAnomalyInjected = simulated;
  ctAnomalyTrip = ctTriggered;

  Serial.println();
  if (simulated) {
    Serial.println("[DEMO] Synthetic abnormal power injected for presentation.");
  } else if (ctTriggered) {
    Serial.printf("[CT TRIP] Residual-current estimate %.3f A exceeded %.3f A for %u clean windows.\n",
                  latestCTDifferenceA, CT_RESIDUAL_THRESHOLD_A, CT_CONFIRM_WINDOWS);
  } else {
    Serial.println("[TRIP] Abnormal power confirmed by detector.");
  }
  Serial.println("[TRIP] Commanding relay OFF.");

  // This commands the relay module output OFF. It is demonstration logic,
  // not a certified or fail-safe protective device.
  setRelay(false);

  // setRelay() marks the transition and resets episode counters; restore the
  // latched event state so the OLED/serial report visibly shows the trip.
  detectorState = ANOMALY;
  anomalyEpisodeActive = true;
  anomalyCandidateCount = CONFIRM_SAMPLES;
  recoveryCount = 0;
}

void processPowerReading(float powerW) {
  latestPowerW = powerW;

  if (tripLatched) {
    detectorState = ANOMALY;
    return;
  }

  // Ignore measurements while the relay/load is settling.
  if (millis() - lastRelayChangeMs < RELAY_SETTLE_MS) {
    detectorState = TRANSITION;
    anomalyCandidateCount = 0;
    recoveryCount = 0;
    return;
  }

  // A valid voltage with almost no active power indicates
  // that the measured load is not consuming meaningful power.
  // It is not automatically a fault or a sensor error.
  if (powerW <= LOAD_OFF_POWER_W) {
    detectorState = LOAD_OFF;

    anomalyCandidateCount = 0;
    recoveryCount = 0;
    anomalyEpisodeActive = false;

    Serial.println("[LOAD] No significant active power.");
    return;
  }

  // Collect only nonzero, valid readings during initial learning.
  if (!modelReady) {
    detectorState = LEARNING;
    addLearningSample(powerW);
    return;
  }

  // Freeze the learned model after initial training.
  // The current reading is compared against the fixed baseline.
  double deviation = fabs((double)powerW - powerMean);

  anomalyThresholdW = fmax(
    MIN_ANOMALY_DEVIATION_W,
    ANOMALY_SIGMA * powerStdDev
  );

  bool outsideThreshold = deviation > anomalyThresholdW;

  if (outsideThreshold) {
    beepForAnomaly();
    recoveryCount = 0;

    if (anomalyCandidateCount < CONFIRM_SAMPLES) {
      anomalyCandidateCount++;
    }

    if (anomalyCandidateCount >= CONFIRM_SAMPLES) {
      detectorState = ANOMALY;

      if (!anomalyEpisodeActive) {
        anomalyEpisodeActive = true;
        anomalyCount++;

        Serial.printf(
          "[ANOMALY CONFIRMED] Power: %.2f W | "
          "Baseline: %.2f W | Deviation: %.2f W | "
          "Threshold: %.2f W | Event: %lu\n",
          powerW,
          powerMean,
          deviation,
          anomalyThresholdW,
          anomalyCount
        );

        if (AUTO_TRIP_ON_ANOMALY) {
          tripRelayForAnomaly(false);
          return;
        }
      }
    } else {
      detectorState = NORMAL;

      Serial.printf(
        "[ANOMALY CANDIDATE] %u/%u | "
        "Power: %.2f W | Deviation: %.2f W\n",
        anomalyCandidateCount,
        CONFIRM_SAMPLES,
        powerW,
        deviation
      );
    }

    // Never train on a reading outside the threshold.
    return;
  }

  // Reading is back inside the normal range.
  anomalyCandidateCount = 0;

  if (anomalyEpisodeActive) {
    recoveryCount++;

    if (recoveryCount < RECOVERY_SAMPLES) {
      detectorState = ANOMALY;
      return;
    }

    anomalyEpisodeActive = false;
    recoveryCount = 0;

    Serial.println("[ANOMALY CLEARED] Power returned to baseline.");
  }

  detectorState = NORMAL;

  // Baseline remains frozen after training.
}

// =====================================================
// CT RESIDUAL-CURRENT DEMO DETECTOR
// =====================================================

void processCTResidual() {
  latestCTDifferenceA = NAN;

  if (tripLatched || !relayOn || !modelReady ||
      millis() - lastRelayChangeMs < RELAY_SETTLE_MS) {
    ctResidualCandidateCount = 0;
    return;
  }

  // Reject windows containing detected spikes or too many discarded samples.
  // This is a basic demo quality gate, not a substitute for calibrated sensing.
  bool inValid = isfinite(ctIn.currentA) && isfinite(ctIn.rmsCounts) &&
                 ctIn.acceptedSamples >= (SAMPLE_COUNT * 9) / 10 &&
                 !ctIn.spikeDetected;
  bool outValid = isfinite(ctOut.currentA) && isfinite(ctOut.rmsCounts) &&
                  ctOut.acceptedSamples >= (SAMPLE_COUNT * 9) / 10 &&
                  !ctOut.spikeDetected;

  if (!inValid || !outValid) {
    ctResidualCandidateCount = 0;
    return;
  }

  latestCTDifferenceA = fabsf(ctIn.currentA - ctOut.currentA);

  if (latestCTDifferenceA >= CT_RESIDUAL_THRESHOLD_A) {
    if (ctResidualCandidateCount < CT_CONFIRM_WINDOWS) {
      ctResidualCandidateCount++;
    }

    Serial.printf("[CT CANDIDATE] %u/%u | Estimated residual: %.3f A | Threshold: %.3f A\n",
                  ctResidualCandidateCount, CT_CONFIRM_WINDOWS,
                  latestCTDifferenceA, CT_RESIDUAL_THRESHOLD_A);

    if (AUTO_TRIP_ON_CT_DIFFERENCE &&
        ctResidualCandidateCount >= CT_CONFIRM_WINDOWS) {
      tripRelayForAnomaly(false, true);
    }
  } else {
    if (ctResidualCandidateCount > 0) {
      Serial.println("[CT] Residual estimate returned below threshold; candidate counter reset.");
    }
    ctResidualCandidateCount = 0;
  }
}

// =====================================================
// PROCESS PZEM EVENTS
// =====================================================

void processPzemUpdate() {
  if (!pzemUpdated) return;

  // This preserves the callback/data handoff used by your
  // supplied sketch. If callbacks run on another task, use
  // a mutex or a critical section in a hardened version.
  Mycila::PZEM::Data snapshot = pzemData;
  pzemUpdated = false;

  if (!isValidPzemReading(snapshot)) {
    // Never use invalid data for learning or anomaly decisions.
    // Keep transition status during relay settling; afterwards preserve
    // a distinct OFF/invalid diagnostic if the relay is commanded OFF.
    if (millis() - lastRelayChangeMs < RELAY_SETTLE_MS) {
      detectorState = TRANSITION;
    } else if (relayOn) {
      detectorState = SENSOR_ERROR;
    } else {
      detectorState = LOAD_OFF;
    }
    Serial.println("[PZEM] Invalid reading rejected; detector model unchanged.");
    return;
  }

  lastValidPzemMs = millis();

  // Ignore readings arriving too close together.
  if (lastLearningSampleMs != 0 &&
      millis() - lastLearningSampleMs < MIN_SAMPLE_INTERVAL_MS) {
    return;
  }

  lastLearningSampleMs = millis();

  processPowerReading(snapshot.activePower);
}

// =====================================================
// RELAY CONTROL
// =====================================================

void setRelay(bool turnOn) {
  if (relayOn != turnOn) {
    relayOn = turnOn;

    digitalWrite(
      RELAY_PIN,
      relayOn ? RELAY_ON : RELAY_OFF
    );

    lastRelayChangeMs = millis();

    anomalyCandidateCount = 0;
    recoveryCount = 0;
    anomalyEpisodeActive = false;

    detectorState = TRANSITION;

    Serial.print("[RELAY] Command: ");
    Serial.println(relayOn ? "ON" : "OFF");
  } else {
    digitalWrite(
      RELAY_PIN,
      relayOn ? RELAY_ON : RELAY_OFF
    );
  }
}

// =====================================================
// SERIAL COMMANDS
// =====================================================

void handleSerial() {
  while (Serial.available()) {
    char c = Serial.read();

    if (c == '1') {
      // Manual re-arm after a trip.
      tripLatched = false;
      demoAnomalyInjected = false;
      ctAnomalyTrip = false;
      ctResidualCandidateCount = 0;
      anomalyCandidateCount = 0;
      recoveryCount = 0;
      anomalyEpisodeActive = false;
      setRelay(true);
      Serial.println("[REARM] Relay commanded ON; detector resumes after settling.");
    } else if (c == '0') {
      setRelay(false);
    } else if (c == 'd' || c == 'D') {
      // Safe software-only demo: no mains wiring changes or sensor manipulation.
      if (relayOn) {
        Serial.println("[DEMO] Intentional software anomaly requested by user.");
        tripRelayForAnomaly(true, false);
      } else {
        Serial.println("[DEMO] Relay is already OFF. Send 1 to re-arm, wait 3 seconds, then send d.");
      }
    } else if (c == 'r' || c == 'R') {
      resetLearning();
    } else if (c == 's' || c == 'S') {
      Serial.println("\n--- SMARTVOLT STATUS ---");
      Serial.printf("State: %s\n", getStateText());
      Serial.printf("Relay command: %s\n", relayOn ? "ON" : "OFF");
      Serial.printf("Model ready: %s\n", modelReady ? "YES" : "NO");
      Serial.printf("Learning samples: %u\n", learningCount);
      Serial.printf("Baseline: %.2f W\n", powerMean);
      Serial.printf("Std deviation: %.3f W\n", powerStdDev);
      Serial.printf("Threshold: %.2f W\n", anomalyThresholdW);
      Serial.printf("Confirmed events: %lu\n", anomalyCount);
      Serial.println("-----------------------");
    }
  }
}

// =====================================================
// OLED PAGE 1: POWER
// =====================================================

void drawPowerPage() {
  display.setCursor(0, 0);

  display.println("SmartVolt: Power");
  display.println("----------------");

  display.print("Voltage: ");
  display.print(pzemData.voltage, 1);
  display.println(" V");

  display.print("Current: ");
  display.print(pzemData.current, 3);
  display.println(" A");

  display.print("Power: ");
  display.print(pzemData.activePower, 1);
  display.println(" W");

  display.print("Energy: ");
  display.print(pzemData.activeEnergy);
  display.println(" Wh");

  display.print("Relay: ");
  display.println(relayOn ? "ON" : "OFF");
}

// =====================================================
// OLED PAGE 2: ANOMALY DETECTOR
// =====================================================

void drawLearningPage() {
  display.setCursor(0, 0);

  display.println("SmartVolt: Detector");
  display.println("----------------");

  display.print("State: ");
  display.println(getStateText());

  display.print("Power: ");
  display.print(latestPowerW, 1);
  display.println(" W");

  display.print("Mean: ");
  display.print(powerMean, 2);
  display.println(" W");

  display.print("SD: ");
  display.print(powerStdDev, 2);
  display.println(" W");

  display.print("Samples: ");
  display.print(learningCount);
  display.print("/");
  display.println(LEARNING_SAMPLES);

  display.print("Events: ");
  display.println((unsigned long)anomalyCount);
}

// =====================================================
// OLED PAGE 3: DIAGNOSTICS
// =====================================================

void drawDiagnosticsPage() {
  display.setCursor(0, 0);

  display.println("SmartVolt: System");
  display.println("----------------");

  display.print("PZEM: ");
  display.println(
    lastValidPzemMs != 0 &&
    millis() - lastValidPzemMs <= SENSOR_TIMEOUT_MS
      ? "VALID"
      : "ERROR"
  );

  display.println("ADS1115: INIT");

  display.print("IN CT*: ");
  display.print(ctIn.currentA, 3);
  display.println(" A");

  display.print("OUT CT*: ");
  display.print(ctOut.currentA, 3);
  display.println(" A");

  display.print("Diff*: ");
  display.print(fabs(ctIn.currentA - ctOut.currentA), 3);
  display.println(" A");

  display.println("*Experimental only");
  display.print("Relay: ");
  display.println(relayOn ? "ON" : "OFF");
}

// =====================================================
// OLED UPDATE
// =====================================================

void updateDisplay() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);

  unsigned long page = (millis() / PAGE_INTERVAL_MS) % 3;

  if (tripLatched) {
    display.setCursor(0, 0);
    display.println("SmartVolt: TRIP");
    display.println("----------------");
    if (demoAnomalyInjected) display.println("SIMULATED ANOMALY");
    else if (ctAnomalyTrip) display.println("CT IMBALANCE TRIP");
    else display.println("POWER ANOMALY");
    display.println("Relay command: OFF");
    display.println("Trip is latched");
    display.println("Send 1 to re-arm");
  } else if (lastValidPzemMs == 0 ||
      millis() - lastValidPzemMs > SENSOR_TIMEOUT_MS) {
    display.setCursor(0, 0);
    display.println("SmartVolt");
    display.println("----------------");
    display.println("PZEM DATA ERROR");
    display.println("Check sensor");
  } else if (page == 0) {
    drawPowerPage();
  } else if (page == 1) {
    drawLearningPage();
  } else {
    drawDiagnosticsPage();
  }

  display.display();
}

// =====================================================
// SERIAL REPORT
// =====================================================

void printReport() {
  Serial.println("\n========== SMARTVOLT ==========");

  Serial.println("--- PZEM ---");
  Serial.printf("Voltage:       %.2f V\n", pzemData.voltage);
  Serial.printf("Current:       %.3f A\n", pzemData.current);
  Serial.printf("Active Power:  %.2f W\n", pzemData.activePower);
  Serial.printf("Energy:        %lu Wh\n",
                (unsigned long)pzemData.activeEnergy);
  Serial.printf("Frequency:     %.2f Hz\n", pzemData.frequency);
  Serial.printf("Power Factor:  %.2f\n", pzemData.powerFactor);

  Serial.println("--- POWER DETECTOR ---");
  Serial.printf("State:         %s\n", getStateText());
  Serial.printf("Model ready:   %s\n", modelReady ? "YES" : "NO");
  Serial.printf("Samples:       %u\n", learningCount);
  Serial.printf("Mean:          %.2f W\n", powerMean);
  Serial.printf("Std deviation: %.3f W\n", powerStdDev);
  Serial.printf("Threshold:     %.2f W\n", anomalyThresholdW);
  Serial.printf("Candidates:    %u/%u\n",
                anomalyCandidateCount, CONFIRM_SAMPLES);
  Serial.printf("Events:        %lu\n", anomalyCount);

  Serial.println("--- CT DIAGNOSTICS ---");
  Serial.printf(
    "IN bias: %.4f V | AC variation: %.6f V | "
    "RMS counts: %.2f | I estimate: %.3f A\n",
    ctIn.biasVoltage, ctIn.acVrms, ctIn.rmsCounts, ctIn.currentA
  );

  Serial.printf(
    "OUT bias: %.4f V | AC variation: %.6f V | "
    "RMS counts: %.2f | I estimate: %.3f A\n",
    ctOut.biasVoltage, ctOut.acVrms, ctOut.rmsCounts, ctOut.currentA
  );

  Serial.printf(
    "CT rejected samples: IN %u/%d, OUT %u/%d | Spike flag: IN %s, OUT %s\n",
    ctIn.rejectedSamples, SAMPLE_COUNT, ctOut.rejectedSamples, SAMPLE_COUNT,
    ctIn.spikeDetected ? "YES" : "NO",
    ctOut.spikeDetected ? "YES" : "NO"
  );

  Serial.printf(
    "Estimated difference: %.3f A\n",
    fabs(ctIn.currentA - ctOut.currentA)
  );
  Serial.printf("CT trip candidates: %u/%u | Threshold: %.3f A | Auto-trip: %s\n",
                ctResidualCandidateCount, CT_CONFIRM_WINDOWS,
                CT_RESIDUAL_THRESHOLD_A,
                AUTO_TRIP_ON_CT_DIFFERENCE ? "ENABLED (EXPERIMENTAL)" : "DISABLED");

  Serial.printf("Relay command: %s\n", relayOn ? "ON" : "OFF");

  Serial.println(
    "CT difference is experimental, NOT a safety measurement."
  );
  Serial.printf("CT sample window: %d samples/channel; calibration factors unverified.\n", SAMPLE_COUNT);
  Serial.println("===============================");
}

// =====================================================
// SETUP
// =====================================================

void setup() {
  Serial.begin(115200);
  delay(1000);

  // Start with relay and buzzer outputs in known states.
  digitalWrite(RELAY_PIN, RELAY_OFF);
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  pinMode(BUZZER_PIN, OUTPUT);

  // Status LEDs: each LED must have its own series resistor.
  pinMode(LED_GREEN_PIN, OUTPUT);
  pinMode(LED_YELLOW_PIN, OUTPUT);
  pinMode(LED_ALERT_PIN, OUTPUT);
  digitalWrite(LED_GREEN_PIN, LOW);
  digitalWrite(LED_YELLOW_PIN, HIGH); // Startup/not-ready indicator
  digitalWrite(LED_ALERT_PIN, LOW);

  relayOn = false;
  lastRelayChangeMs = millis();

  Wire.begin(SDA_PIN, SCL_PIN);

  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS)) {
    Serial.println("[OLED] Not detected.");
  } else {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("SmartVolt starting...");
    display.display();
  }

  if (!ads.begin(ADS_ADDRESS)) {
    Serial.println("[ADS1115] Not detected at 0x48.");

    digitalWrite(RELAY_PIN, RELAY_OFF);
    relayOn = false;

    while (true) {
      delay(1000);
    }
  }

  ads.setGain(GAIN_ONE);
  ads.setDataRate(RATE_ADS1115_860SPS);

  // Preserve the MycilaPZEM API from your working sketch.
  pzem.begin(
    Serial2,
    PZEM_RX_PIN,
    PZEM_TX_PIN,
    0xF8,
    true
  );

  pzem.setCallback(onPzemEvent);

  Serial.println("\nSmartVolt V2 started.");
  Serial.println("Commands: 1=relay ON/re-arm, 0=relay OFF, d=DEMO TRIP, s=status, r=reset learning");
  Serial.println("Initial training requires 30 valid nonzero power readings.");
  Serial.println("Relay transitions are ignored for 3 seconds.");
  Serial.println("Anomaly confirmation requires 3 consecutive readings.");
  Serial.println("Automatic power-anomaly trip: DISABLED.");
  Serial.println("CT auto-trip: DISABLED (diagnostics only).");
  Serial.println("Buzzer: short beep for each out-of-range power reading; continuous tone while trip is latched.");
  Serial.println("LEDs: GREEN=confirmed normal, YELLOW=learning/not ready/warning, 2-pin LED=trip latched.");
  Serial.println("LED pins: GREEN GPIO18, YELLOW GPIO19, 2-pin alert LED GPIO27 (each bare LED needs its own series resistor).");
  Serial.println("Note: the 2-pin alert LED color-changing behavior is not yet verified; it is only a trip indicator.");
  Serial.println("Only command d intentionally simulates an anomaly and trips the relay.");
  Serial.printf("CT residual auto-trip: %s (diagnostics remain active; CT calibration is unverified).\\n",
                AUTO_TRIP_ON_CT_DIFFERENCE ? "ENABLED" : "DISABLED");
  Serial.println("CT trip is experimental; current estimates/calibration are NOT validated.");
  Serial.println("Do NOT test by separating live and neutral mains connections.");
  Serial.println("WARNING: prototype/demo only; do not rely on it for shock/fire protection.");
}

// =====================================================
// MAIN LOOP
// =====================================================

void loop() {
  handleSerial();
  updateBuzzer();
  updateStatusLEDs();

  processPzemUpdate();

  // If fresh valid readings disappear, report sensor error.
  if (lastValidPzemMs == 0 ||
      millis() - lastValidPzemMs > SENSOR_TIMEOUT_MS) {
    // An OFF relay with no valid PZEM reading is not automatically a sensor
    // failure. getStateText() reports OFF / PZEM INVALID in this condition.
    detectorState = relayOn ? SENSOR_ERROR : LOAD_OFF;
  }

  // Read CT channels and evaluate the experimental residual-current demo detector.
  if (millis() - lastCTRead >= CT_INTERVAL_MS) {
    lastCTRead = millis();

    ctIn = readCT(0, CT1_AMPS_PER_VRMS);
    ctOut = readCT(1, CT2_AMPS_PER_VRMS);
    processCTResidual();
  }

  if (millis() - lastDisplay >= DISPLAY_INTERVAL_MS) {
    lastDisplay = millis();
    updateDisplay();
  }

  if (millis() - lastReport >= REPORT_INTERVAL_MS) {
    lastReport = millis();
    printReport();
  }

  // Keep indicators and buzzer responsive even while other work runs.
  updateBuzzer();
  updateStatusLEDs();
}
