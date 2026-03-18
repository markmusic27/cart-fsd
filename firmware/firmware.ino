// Cart FSD — Arduino Mega 2560 Firmware
//
// PD closed-loop control for gas and brake actuators.
// Open-loop direction control for steering (closed-loop in Phase 2).
// Serial protocol v2: receives targets, reports state at ~100 Hz.
// Safety: hardware e-stop (pin 21), watchdog (250 ms timeout).
//
// Encoder: BMQ-A38L6 / 600 PPR on steering column via 3D-printed gear.

// =====================================================================
// Pin assignments
// =====================================================================

// Steering motor (BTS7960 #1 — 24 V)
const int STR_R_EN  = 8;
const int STR_R_PWM = 2;
const int STR_L_EN  = 9;
const int STR_L_PWM = 3;

// Gas actuator (BTS7960 #2 — 12 V)
const int GAS_R_EN  = 6;
const int GAS_R_PWM = 4;
const int GAS_L_EN  = 7;
const int GAS_L_PWM = 5;
const int GAS_POT   = A4;

// Brake actuator (BTS7960 #3 — 12 V)
const int BRK_R_EN  = 12;
const int BRK_R_PWM = 10;
const int BRK_L_EN  = 13;
const int BRK_L_PWM = 11;
const int BRK_POT   = A0;

// Rotary encoder
const int ENC_A = 19;  // Green — INT4
const int ENC_B = 18;  // White — INT5

// Hardware e-stop (NC mushroom-head latching button)
const int ESTOP_PIN = 21;  // INT2, internal pull-up

// =====================================================================
// Encoder constants
// =====================================================================

const float ENCODER_PPR       = 600.0;
const float ENCODER_WHEEL_DIA = 30.8;   // mm
const float STEERING_COL_DIA  = 90.0;   // mm
const float ENC_GEAR_RATIO    = ENCODER_WHEEL_DIA / STEERING_COL_DIA;

volatile long encoderCount = 0;

// =====================================================================
// Pedal calibration (normalized ADC: 0.0–1.0)
// =====================================================================

const float GAS_POS_MIN = 0.05;
const float GAS_POS_MAX = 0.72;

const float BRK_POS_MIN = 0.47;
const float BRK_POS_MAX = 0.90;

// =====================================================================
// PD controller defaults (overridable via C commands)
// =====================================================================

float gasKp  = 1200.0;
float gasKd  = 70.0;
float brkKp  = 1000.0;
float brkKd  = 60.0;
float strKp  = 2.0;
float strKd  = 0.9;

int   gasMaxPwm = 255;
int   brkMaxPwm = 255;
int   strMaxPwm = 255;

float gasDeadband = 0.05;
float brkDeadband = 0.05;
float strDeadband = 10.0;   // degrees

const int PEDAL_MIN_PWM = 110;
const int STEER_MIN_PWM = 65;
const int STEER_MID_MIN_PWM = 50;
const int STEER_PWM_SLEW_PER_LOOP = 7;  // ~105 ms from 0 to full at 200 Hz
const float STR_REENGAGE_DEADBAND = 15.0;
const float STR_SMALL_ERROR_BAND = 18.0;
const float STR_LARGE_ERROR_BAND = 35.0;
const float STR_OUTPUT_DEADBAND = 18.0;

// =====================================================================
// State
// =====================================================================

// Targets: -1.0 means "inactive / no target" for pedals
float gasTarget = -1.0;
float brkTarget = -1.0;
float strTarget =  0.0;   // degrees at steering column
bool  strActive = false;
float strCommand = 0.0;   // normalized open-loop command [-1, 1]
bool  strOpenLoop = false;
int   strAppliedPwm = 0;

// Previous positions for derivative (velocity) calculation
float gasPrevPos = 0.0;
float brkPrevPos = 0.0;
float strPrevAngle = 0.0;

// Timing
unsigned long prevLoopMicros = 0;
const unsigned long LOOP_INTERVAL_US = 5000;  // 5 ms → 200 Hz

// Watchdog
unsigned long lastCommandTime = 0;
const unsigned long WATCHDOG_TIMEOUT_MS = 250;
bool watchdogTripped = false;

// Hardware e-stop
volatile bool eStopActive = false;
bool safetyEnabled = true;

// State report throttle (send every N loops to avoid flooding serial)
const int STATE_REPORT_INTERVAL = 4;  // every 4th loop → ~50 Hz at 200 Hz loop
int stateReportCounter = 0;

// =====================================================================
// Motor helpers
// =====================================================================

void gasEnable()  { digitalWrite(GAS_R_EN, HIGH); digitalWrite(GAS_L_EN, HIGH); }
void gasDisable() { analogWrite(GAS_R_PWM, 0); analogWrite(GAS_L_PWM, 0);
                    digitalWrite(GAS_R_EN, LOW); digitalWrite(GAS_L_EN, LOW); }
void gasDrive(int pwm) {
  gasEnable();
  if (pwm > 0) { analogWrite(GAS_R_PWM, 0);        analogWrite(GAS_L_PWM, pwm); }
  else         { analogWrite(GAS_L_PWM, 0);        analogWrite(GAS_R_PWM, -pwm); }
}

void brkEnable()  { digitalWrite(BRK_R_EN, HIGH); digitalWrite(BRK_L_EN, HIGH); }
void brkDisable() { analogWrite(BRK_R_PWM, 0); analogWrite(BRK_L_PWM, 0);
                    digitalWrite(BRK_R_EN, LOW); digitalWrite(BRK_L_EN, LOW); }
void brkDrive(int pwm) {
  brkEnable();
  if (pwm > 0) { analogWrite(BRK_R_PWM, 0);        analogWrite(BRK_L_PWM, pwm); }
  else         { analogWrite(BRK_L_PWM, 0);        analogWrite(BRK_R_PWM, -pwm); }
}

void strEnable()  { digitalWrite(STR_R_EN, HIGH); digitalWrite(STR_L_EN, HIGH); }
void strDisable() { analogWrite(STR_R_PWM, 0); analogWrite(STR_L_PWM, 0);
                    digitalWrite(STR_R_EN, LOW); digitalWrite(STR_L_EN, LOW); }
void strDrive(int pwm) {
  strEnable();
  if (pwm > 0) { analogWrite(STR_L_PWM, 0);        analogWrite(STR_R_PWM, pwm); }
  else         { analogWrite(STR_R_PWM, 0);        analogWrite(STR_L_PWM, -pwm); }
}

void allDisable() { gasDisable(); brkDisable(); strDisable(); }

int slewPwmToward(int currentPwm, int targetPwm, int maxStep) {
  if (targetPwm > currentPwm + maxStep) return currentPwm + maxStep;
  if (targetPwm < currentPwm - maxStep) return currentPwm - maxStep;
  return targetPwm;
}

// =====================================================================
// Sensor reads
// =====================================================================

float gasReadPos() { return analogRead(GAS_POT) / 1023.0; }
float brkReadPos() { return analogRead(BRK_POT) / 1023.0; }

float getSteeringAngle() {
  return (encoderCount / ENCODER_PPR) * 360.0 * ENC_GEAR_RATIO;
}

// Map user input (0–1) to raw pot position
float gasUserToPos(float u) { return GAS_POS_MIN + u * (GAS_POS_MAX - GAS_POS_MIN); }
float brkUserToPos(float u) { return BRK_POS_MIN + u * (BRK_POS_MAX - BRK_POS_MIN); }

// Map raw pot position back to user (0–1)
float gasPosToUser(float pos) { return constrain((pos - GAS_POS_MIN) / (GAS_POS_MAX - GAS_POS_MIN), 0.0, 1.0); }
float brkPosToUser(float pos) { return constrain((pos - BRK_POS_MIN) / (BRK_POS_MAX - BRK_POS_MIN), 0.0, 1.0); }

// =====================================================================
// Encoder ISR
// =====================================================================

void readEncoder() {
  if (digitalRead(ENC_B) == LOW) encoderCount--;
  else                           encoderCount++;
}

// =====================================================================
// E-Stop ISR
// =====================================================================

void eStopISR() {
  eStopActive = true;
}

// =====================================================================
// PD controller for a single pedal actuator
// =====================================================================

void updatePedalPD(
    float target,          // raw pot target position
    float currentPos,      // raw pot current position
    float &prevPos,        // previous position (updated in place)
    float dt,              // seconds since last update
    float kp, float kd,
    int maxPwm,
    float deadband,
    void (*drive)(int),
    void (*disable)(),
    bool &settled
) {
  float error = target - currentPos;

  if (abs(error) < deadband) {
    disable();
    settled = true;
    prevPos = currentPos;
    return;
  }
  settled = false;

  float velocity = (dt > 0.0001) ? (currentPos - prevPos) / dt : 0.0;
  float output = kp * error - kd * velocity;

  // Clamp to configured max
  output = constrain(output, (float)-maxPwm, (float)maxPwm);

  // Enforce minimum PWM to overcome static friction
  if (abs(output) < PEDAL_MIN_PWM) {
    output = (output > 0) ? PEDAL_MIN_PWM : -PEDAL_MIN_PWM;
  }

  drive((int)output);
  prevPos = currentPos;
}

// =====================================================================
// Control loop updates
// =====================================================================

bool gasSettled  = true;
bool brkSettled  = true;
bool strSettled  = true;

void updateGas(float dt) {
  if (gasTarget < 0) { gasDisable(); gasSettled = true; return; }
  float pos = gasReadPos();
  updatePedalPD(gasTarget, pos, gasPrevPos, dt,
                gasKp, gasKd, gasMaxPwm, gasDeadband,
                gasDrive, gasDisable, gasSettled);
}

void updateBrake(float dt) {
  if (brkTarget < 0) { brkDisable(); brkSettled = true; return; }
  float pos = brkReadPos();
  updatePedalPD(brkTarget, pos, brkPrevPos, dt,
                brkKp, brkKd, brkMaxPwm, brkDeadband,
                brkDrive, brkDisable, brkSettled);
}

void updateSteering(float dt) {
  if (strOpenLoop) {
    float currentAngle = getSteeringAngle();
    if (abs(strCommand) < 0.05) {
      strDisable();
      strAppliedPwm = 0;
      strSettled = true;
      strPrevAngle = currentAngle;
      return;
    }

    strSettled = false;
    int targetPwm = (int)(abs(strCommand) * strMaxPwm);
    if (targetPwm < STEER_MID_MIN_PWM) targetPwm = STEER_MID_MIN_PWM;
    targetPwm = constrain(targetPwm, 0, strMaxPwm);
    if (strCommand < 0) targetPwm = -targetPwm;

    strAppliedPwm = slewPwmToward(strAppliedPwm, targetPwm, STEER_PWM_SLEW_PER_LOOP);
    strDrive(strAppliedPwm);

    strPrevAngle = currentAngle;
    return;
  }

  if (!strActive) { strDisable(); strAppliedPwm = 0; strSettled = true; return; }

  float currentAngle = getSteeringAngle();
  float error = strTarget - currentAngle;
  float absError = abs(error);

  if (strSettled && absError < STR_REENGAGE_DEADBAND) {
    strDisable();
    strAppliedPwm = 0;
    strPrevAngle = currentAngle;
    return;
  }

  if (absError < strDeadband) {
    strDisable();
    strAppliedPwm = 0;
    strSettled = true;
    strPrevAngle = currentAngle;
    return;
  }
  strSettled = false;

  float velocity = (dt > 0.0001) ? (currentAngle - strPrevAngle) / dt : 0.0;
  float output = strKp * error - strKd * velocity;
  output = constrain(output, (float)-strMaxPwm, (float)strMaxPwm);

  if (abs(output) < STR_OUTPUT_DEADBAND) {
    strDisable();
    strAppliedPwm = 0;
    strPrevAngle = currentAngle;
    return;
  }

  int steerMinPwm = 0;
  if (absError > STR_LARGE_ERROR_BAND) {
    steerMinPwm = STEER_MIN_PWM;
  } else if (absError > STR_SMALL_ERROR_BAND) {
    steerMinPwm = STEER_MID_MIN_PWM;
  }

  if (steerMinPwm > 0 && abs(output) < steerMinPwm) {
    output = (output > 0) ? steerMinPwm : -steerMinPwm;
  }

  strAppliedPwm = slewPwmToward(strAppliedPwm, (int)output, STEER_PWM_SLEW_PER_LOOP);
  strDrive(strAppliedPwm);

  strPrevAngle = currentAngle;
}

// =====================================================================
// Apply safe state (e-stop or watchdog)
// =====================================================================

void applySafeState() {
  gasTarget = gasUserToPos(0.0);   // retract gas
  brkTarget = brkUserToPos(1.0);   // full brake
  strActive = false;               // hold steering
  strOpenLoop = false;
  strCommand = 0.0;
}

// =====================================================================
// Serial protocol — command parsing
// =====================================================================

void parseCommand(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;

  lastCommandTime = millis();
  watchdogTripped = false;

  char first = cmd.charAt(0);

  // E — software e-stop
  if (first == 'E') {
    applySafeState();
    Serial.println("ESTOP");
    return;
  }

  // Z — zero encoder
  if (first == 'Z') {
    encoderCount = 0;
    Serial.println("ZEROED");
    return;
  }

  // C — config update: "C <KEY> <VALUE>"
  if (first == 'C') {
    int sp1 = cmd.indexOf(' ', 2);
    if (sp1 < 0) return;
    String key = cmd.substring(2, sp1);
    float val = cmd.substring(sp1 + 1).toFloat();

    if      (key == "SAFE") safetyEnabled = ((int)val != 0);
    else if (key == "GKP")  gasKp = val;
    else if (key == "GKD")  gasKd = val;
    else if (key == "BKP")  brkKp = val;
    else if (key == "BKD")  brkKd = val;
    else if (key == "SKP")  strKp = val;
    else if (key == "SKD")  strKd = val;
    else if (key == "GMAX") gasMaxPwm = (int)val;
    else if (key == "BMAX") brkMaxPwm = (int)val;
    else if (key == "SMAX") strMaxPwm = (int)val;
    else if (key == "GDB")  gasDeadband = val;
    else if (key == "BDB")  brkDeadband = val;
    else if (key == "SDB")  strDeadband = val;
    return;
  }

  // T — set targets: "T G<val> B<val> A<angle> [S<command>]"
  if (first == 'T') {
    // If hardware e-stop is active, ignore target commands
    if (eStopActive) return;

    int gi = cmd.indexOf('G');
    int bi = cmd.indexOf('B');
    int ai = cmd.indexOf('A');
    int si = cmd.indexOf('S');

    if (gi >= 0) {
      int endIdx = cmd.length();
      if (bi >= 0 && bi < endIdx) endIdx = bi;
      if (ai >= 0 && ai < endIdx) endIdx = ai;
      if (si >= 0 && si < endIdx) endIdx = si;
      float val = constrain(cmd.substring(gi + 1, endIdx).toFloat(), 0.0, 1.0);
      gasTarget = gasUserToPos(val);
    }

    if (bi >= 0) {
      int endIdx = cmd.length();
      if (ai >= 0 && ai < endIdx) endIdx = ai;
      if (si >= 0 && si < endIdx) endIdx = si;
      float val = constrain(cmd.substring(bi + 1, endIdx).toFloat(), 0.0, 1.0);
      brkTarget = brkUserToPos(val);
    }

    if (ai >= 0) {
      int endIdx = cmd.length();
      if (si >= 0 && si < endIdx) endIdx = si;
      strTarget = cmd.substring(ai + 1, endIdx).toFloat();
      strActive = true;
    }

    if (si >= 0) {
      strCommand = constrain(cmd.substring(si + 1).toFloat(), -1.0, 1.0);
      strOpenLoop = true;
      strActive = false;
    } else {
      strCommand = 0.0;
      strOpenLoop = false;
    }
    return;
  }

  // STATUS — legacy compatibility
  if (cmd == "STATUS") {
    Serial.print("GAS:");   Serial.print(gasReadPos(), 3);
    Serial.print(" BRK:");  Serial.print(brkReadPos(), 3);
    Serial.print(" ENC:");  Serial.print(encoderCount);
    Serial.print(" ANG:");  Serial.print(getSteeringAngle(), 1);
    Serial.println();
    return;
  }

  // STOP — legacy compatibility
  if (cmd == "STOP") {
    applySafeState();
    Serial.println("ALL STOPPED");
    return;
  }
}

// =====================================================================
// State report
// =====================================================================

void sendStateReport() {
  float gasPos   = gasPosToUser(gasReadPos());
  float brkPos   = brkPosToUser(brkReadPos());
  float strAngle = getSteeringAngle();

  Serial.print("S G");  Serial.print(gasPos, 3);
  Serial.print(" B");   Serial.print(brkPos, 3);
  Serial.print(" A");   Serial.print(strAngle, 1);
  Serial.print(" GS");  Serial.print(gasSettled ? 1 : 0);
  Serial.print(" BS");  Serial.print(brkSettled ? 1 : 0);
  Serial.print(" AS");  Serial.print(strSettled ? 1 : 0);
  Serial.print(" ES");  Serial.print(eStopActive ? 1 : 0);
  Serial.println();
}

// =====================================================================
// Setup
// =====================================================================

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(2);

  // Encoder
  pinMode(ENC_A, INPUT);
  pinMode(ENC_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENC_A), readEncoder, RISING);

  // E-stop (NC button between pin 21 and GND, internal pull-up)
  pinMode(ESTOP_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ESTOP_PIN), eStopISR, RISING);
  // Check initial state — if button is already pressed at boot
  if (digitalRead(ESTOP_PIN) == HIGH) {
    eStopActive = true;
  }

  // Gas
  pinMode(GAS_R_PWM, OUTPUT); pinMode(GAS_L_PWM, OUTPUT);
  pinMode(GAS_R_EN, OUTPUT);  pinMode(GAS_L_EN, OUTPUT);
  gasDisable();

  // Brake
  pinMode(BRK_R_PWM, OUTPUT); pinMode(BRK_L_PWM, OUTPUT);
  pinMode(BRK_R_EN, OUTPUT);  pinMode(BRK_L_EN, OUTPUT);
  brkDisable();

  // Steering
  pinMode(STR_R_PWM, OUTPUT); pinMode(STR_L_PWM, OUTPUT);
  pinMode(STR_R_EN, OUTPUT);  pinMode(STR_L_EN, OUTPUT);
  strDisable();

  // Initialize previous positions
  gasPrevPos  = gasReadPos();
  brkPrevPos  = brkReadPos();
  strPrevAngle = getSteeringAngle();

  lastCommandTime = millis();
  prevLoopMicros = micros();

  Serial.println("CART_FSD READY");
}

// =====================================================================
// Main loop — 100 Hz fixed-rate
// =====================================================================

void loop() {
  unsigned long now = micros();
  if (now - prevLoopMicros < LOOP_INTERVAL_US) return;
  float dt = (now - prevLoopMicros) / 1000000.0;
  prevLoopMicros = now;

  // --- Safety checks ---

  // Hardware e-stop: check if button has been released (pin LOW = NC closed)
  if (eStopActive && digitalRead(ESTOP_PIN) == LOW) {
    // Button released, but we stay in e-stop until a new command arrives.
    // The next T command will clear it (eStopActive is reset on valid T
    // only if the pin is LOW, i.e., button physically released).
  }

  if (eStopActive) {
    applySafeState();
  }

  // Watchdog: no command received within timeout
  if (safetyEnabled && !eStopActive && (millis() - lastCommandTime > WATCHDOG_TIMEOUT_MS)) {
    if (!watchdogTripped) {
      watchdogTripped = true;
      applySafeState();
    }
  }

  // --- Read serial commands ---
  while (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    parseCommand(input);

    // If hardware e-stop was released and a new command arrived, clear it
    if (eStopActive && digitalRead(ESTOP_PIN) == LOW) {
      eStopActive = false;
    }
  }

  // --- PD control updates ---
  updateGas(dt);
  updateBrake(dt);
  updateSteering(dt);

  // --- State report (throttled) ---
  stateReportCounter++;
  if (stateReportCounter >= STATE_REPORT_INTERVAL) {
    stateReportCounter = 0;
    sendStateReport();
  }
}
