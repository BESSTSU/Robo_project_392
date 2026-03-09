#include <Arduino.h>

// =========================
// Motor pins (MDD10A)
// =========================
#define RR_PWM 26
#define RR_DIR 25
#define RL_PWM 19
#define RL_DIR 18
#define FR_PWM 14
#define FR_DIR 27
#define FL_PWM 17
#define FL_DIR 16

// =========================
// Encoder pins (2 channels)
// =========================
#define ENC_L_A 22
#define ENC_L_B 23
#define ENC_R_A 32
#define ENC_R_B 33

// 400 PPR, 4x decode
constexpr int32_t COUNTS_PER_REV = 1600;
constexpr float WHEEL_DIAMETER_INCH = 5.0f;
constexpr float INCH_TO_CM = 2.54f;
constexpr float WHEEL_CIRC_CM = WHEEL_DIAMETER_INCH * INCH_TO_CM * PI;

constexpr int PWM_FREQ = 20000;
constexpr int PWM_BITS = 10;
constexpr int SPEED_MAX = 1023;
constexpr int ACC_STEP = 40;
constexpr int LOOP_DT_MS = 10;
constexpr uint32_t CMD_TIMEOUT_MS = 500;
constexpr uint32_t TELEMETRY_PERIOD_MS = 200;
constexpr int DRV_PROTO_VER = 2;

// If direction is flipped on your chassis, change signs.
constexpr int LEFT_ENCODER_SIGN = +1;
constexpr int RIGHT_ENCODER_SIGN = +1;

class EncoderChannel {
public:
  EncoderChannel(uint8_t pinA, uint8_t pinB): _pinA(pinA), _pinB(pinB) {}

  void begin() {
    pinMode(_pinA, INPUT_PULLUP);
    pinMode(_pinB, INPUT_PULLUP);
    _oldState = (digitalRead(_pinA) << 1) | digitalRead(_pinB);
  }

  void updateFromISR() {
    uint8_t newState = (digitalRead(_pinA) << 1) | digitalRead(_pinB);
    _count += QEM[(_oldState << 2) | newState];
    _oldState = newState;
  }

  int32_t readCount() const {
    return _count;
  }

  void reset() {
    _count = 0;
  }

private:
  uint8_t _pinA;
  uint8_t _pinB;
  volatile int32_t _count = 0;
  volatile uint8_t _oldState = 0;

  static const int8_t QEM[16];
};

const int8_t EncoderChannel::QEM[16] = {
  0, -1, 1, 0,
  1, 0, 0, -1,
  -1, 0, 0, 1,
  0, 1, -1, 0
};

EncoderChannel leftEnc(ENC_L_A, ENC_L_B);
EncoderChannel rightEnc(ENC_R_A, ENC_R_B);

void IRAM_ATTR isrLeftA() { leftEnc.updateFromISR(); }
void IRAM_ATTR isrLeftB() { leftEnc.updateFromISR(); }
void IRAM_ATTR isrRightA() { rightEnc.updateFromISR(); }
void IRAM_ATTR isrRightB() { rightEnc.updateFromISR(); }

class DriveBase {
public:
  void begin() {
    pinMode(FR_DIR, OUTPUT);
    pinMode(FL_DIR, OUTPUT);
    pinMode(RR_DIR, OUTPUT);
    pinMode(RL_DIR, OUTPUT);

    ledcSetup(0, PWM_FREQ, PWM_BITS);
    ledcAttachPin(FR_PWM, 0);
    ledcSetup(1, PWM_FREQ, PWM_BITS);
    ledcAttachPin(FL_PWM, 1);
    ledcSetup(2, PWM_FREQ, PWM_BITS);
    ledcAttachPin(RR_PWM, 2);
    ledcSetup(3, PWM_FREQ, PWM_BITS);
    ledcAttachPin(RL_PWM, 3);

    setTarget(0, 0);
    curL = curR = 0;
  }

  void setTarget(int left, int right) {
    tgtL = constrain(left, -SPEED_MAX, SPEED_MAX);
    tgtR = constrain(right, -SPEED_MAX, SPEED_MAX);
  }

  void stop() {
    setTarget(0, 0);
  }

  void tick() {
    curL = smooth(curL, tgtL);
    curR = smooth(curR, tgtR);

    int fl = -curL;
    int rl = -curL;
    int fr = +curR;
    int rr = +curR;

    digitalWrite(FL_DIR, fl >= 0 ? HIGH : LOW);
    digitalWrite(FR_DIR, fr >= 0 ? HIGH : LOW);
    digitalWrite(RL_DIR, rl >= 0 ? HIGH : LOW);
    digitalWrite(RR_DIR, rr >= 0 ? HIGH : LOW);

    ledcWrite(1, abs(fl));
    ledcWrite(0, abs(fr));
    ledcWrite(3, abs(rl));
    ledcWrite(2, abs(rr));
  }

private:
  int curL = 0;
  int curR = 0;
  int tgtL = 0;
  int tgtR = 0;

  int smooth(int cur, int tgt) {
    if (cur < tgt) {
      cur += ACC_STEP;
      if (cur > tgt) cur = tgt;
    } else if (cur > tgt) {
      cur -= ACC_STEP;
      if (cur < tgt) cur = tgt;
    }
    return cur;
  }
};

DriveBase drive;
String inbuf;
uint32_t lastCmdMs = 0;
uint32_t lastTelemMs = 0;
int32_t lastCountL = 0;
int32_t lastCountR = 0;

void emitTelemetry(bool forceEmit);

void emitProto() {
  Serial.printf("DRV_PROTO VER:%d CAPS:VEL,ODO_RESET,STOP,STATUS,PROTO_VER,HELP\n", DRV_PROTO_VER);
}

void emitHelp() {
  Serial.println("Commands: VEL L:<int> R:<int>, ODO RESET, STOP, STATUS, PROTO_VER, HELP");
}

void parseCommand(String cmd) {
  cmd.trim();
  cmd.toUpperCase();
  if (cmd.length() == 0) return;

  if (cmd.startsWith("VEL")) {
    int idxL = cmd.indexOf("L:");
    int idxR = cmd.indexOf("R:");
    if (idxL >= 0 && idxR >= 0) {
      int endL = cmd.indexOf(' ', idxL);
      if (endL < 0) endL = cmd.length();
      int left = cmd.substring(idxL + 2, endL).toInt();
      int right = cmd.substring(idxR + 2).toInt();
      drive.setTarget(left, right);
      lastCmdMs = millis();
      Serial.printf("DRV_ACK VEL L:%d R:%d\n", left, right);
      return;
    }
  }

  if (cmd == "ODO RESET") {
    noInterrupts();
    leftEnc.reset();
    rightEnc.reset();
    interrupts();
    lastCountL = 0;
    lastCountR = 0;
    Serial.println("DRV_ACK ODO_RESET");
    return;
  }

  if (cmd == "STOP") {
    drive.stop();
    lastCmdMs = millis();
    Serial.println("DRV_ACK STOP");
    return;
  }

  if (cmd == "STATUS") {
    emitTelemetry(true);
    Serial.println("DRV_ACK STATUS");
    return;
  }

  if (cmd == "PROTO_VER") {
    emitProto();
    return;
  }

  if (cmd == "HELP") {
    emitHelp();
    return;
  }

  Serial.print("DRV_ERR UNKNOWN:");
  Serial.println(cmd);
}

void emitTelemetry(bool forceEmit) {
  uint32_t now = millis();
  uint32_t dt = now - lastTelemMs;
  if (!forceEmit && dt < TELEMETRY_PERIOD_MS) return;
  if (dt == 0) dt = 1;

  int32_t countL;
  int32_t countR;
  noInterrupts();
  countL = leftEnc.readCount();
  countR = rightEnc.readCount();
  interrupts();

  int32_t dL = (countL - lastCountL) * LEFT_ENCODER_SIGN;
  int32_t dR = (countR - lastCountR) * RIGHT_ENCODER_SIGN;

  float rpmL = (dL * 60000.0f) / (COUNTS_PER_REV * dt);
  float rpmR = (dR * 60000.0f) / (COUNTS_PER_REV * dt);

  float distL = (countL * LEFT_ENCODER_SIGN) / (float)COUNTS_PER_REV * WHEEL_CIRC_CM;
  float distR = (countR * RIGHT_ENCODER_SIGN) / (float)COUNTS_PER_REV * WHEEL_CIRC_CM;
  float distAvg = (distL + distR) * 0.5f;

  Serial.printf(
    "DRV_STAT L_RPM:%.2f R_RPM:%.2f DIST_CM:%.2f L_COUNT:%ld R_COUNT:%ld\n",
    rpmL, rpmR, distAvg, (long)countL, (long)countR
  );

  lastCountL = countL;
  lastCountR = countR;
  lastTelemMs = now;
}

void setup() {
  Serial.begin(115200);

  drive.begin();

  leftEnc.begin();
  rightEnc.begin();
  attachInterrupt(digitalPinToInterrupt(ENC_L_A), isrLeftA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_L_B), isrLeftB, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), isrRightA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_R_B), isrRightB, CHANGE);

  lastCmdMs = millis();
  lastTelemMs = millis();

  Serial.println("DRV_READY");
  emitProto();
  emitHelp();
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (inbuf.length() > 0) {
        parseCommand(inbuf);
        inbuf = "";
      }
    } else {
      inbuf += c;
    }
  }

  if (millis() - lastCmdMs > CMD_TIMEOUT_MS) {
    drive.stop();
  }

  drive.tick();
  emitTelemetry(false);

  delay(LOOP_DT_MS);
}
