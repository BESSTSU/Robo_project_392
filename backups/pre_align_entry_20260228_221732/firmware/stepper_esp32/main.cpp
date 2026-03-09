#include <Arduino.h>
#include <AccelStepper.h>
#include <ESP32Servo.h>

// =========================
// Stepper pins
// =========================
#define FEED_STEP_PIN 25
#define FEED_DIR_PIN  26
#define FEED_EN_PIN   27

#define PLANT_STEP_PIN 18
#define PLANT_DIR_PIN  19
#define PLANT_EN_PIN   23

// =========================
// Limit pins for plant axis
// =========================
#define PLANT_LIMIT_TOP_PIN    34
#define PLANT_LIMIT_BOTTOM_PIN 35

// =========================
// Servo pins
// =========================
#define SERVO_CAM_PIN   32
#define SERVO_PLANT_PIN 33

constexpr int SERVO_MIN_US = 500;
constexpr int SERVO_MAX_US = 2400;
constexpr int SERVO_MIN_DEG = 0;
constexpr int SERVO_MAX_DEG = 180;
constexpr int ACT_PROTO_VER = 2;

constexpr float STEPS_PER_REV = 420.0f;
constexpr float RUN_LIMIT_SPEED = 800.0f;
constexpr uint32_t LIMIT_DEBOUNCE_MS = 20;

AccelStepper feedMotor(AccelStepper::DRIVER, FEED_STEP_PIN, FEED_DIR_PIN);
AccelStepper plantMotor(AccelStepper::DRIVER, PLANT_STEP_PIN, PLANT_DIR_PIN);

Servo servoCam;
Servo servoPlant;

struct DebouncedLimit {
  uint8_t pin;
  bool raw;
  bool stable;
  uint32_t lastChange;
};

DebouncedLimit topLimit{PLANT_LIMIT_TOP_PIN, false, false, 0};
DebouncedLimit bottomLimit{PLANT_LIMIT_BOTTOM_PIN, false, false, 0};

enum PlantLimitMode {
  PLANT_IDLE,
  PLANT_RUN_UP_LIMIT,
  PLANT_RUN_DOWN_LIMIT
};

PlantLimitMode plantMode = PLANT_IDLE;
String inbuf;
int servoCamDeg = 120;
int servoPlantDeg = 10;

long degToSteps(float deg) {
  return (long)(deg * STEPS_PER_REV / 360.0f);
}

bool limitPressed(DebouncedLimit &sw) {
  bool rawPressed = (digitalRead(sw.pin) == LOW);
  uint32_t now = millis();
  if (rawPressed != sw.raw) {
    sw.raw = rawPressed;
    sw.lastChange = now;
  }
  if ((now - sw.lastChange) >= LIMIT_DEBOUNCE_MS) {
    sw.stable = sw.raw;
  }
  return sw.stable;
}

void enableDrivers() {
  feedMotor.enableOutputs();
  plantMotor.enableOutputs();
}

void stopAll() {
  feedMotor.stop();
  plantMotor.stop();
  plantMotor.setSpeed(0);
  plantMode = PLANT_IDLE;
  Serial.println("ACT_ACK STOP");
}

void setServoSafe(Servo &s, int &holder, int deg) {
  holder = constrain(deg, SERVO_MIN_DEG, SERVO_MAX_DEG);
  s.write(holder);
}

void startPlantLimitRun(bool up) {
  plantMotor.stop();
  plantMode = up ? PLANT_RUN_UP_LIMIT : PLANT_RUN_DOWN_LIMIT;
  plantMotor.setSpeed(up ? RUN_LIMIT_SPEED : -RUN_LIMIT_SPEED);
  Serial.println(up ? "ACT_ACK PLANT_UP_LIMIT_START" : "ACT_ACK PLANT_DOWN_LIMIT_START");
}

void emitProto() {
  Serial.printf(
    "ACT_PROTO VER:%d CAPS:INIT_HOME,SERVO_CAM,SERVO_PLANT,FEED_STEP,PLANT_DOWN_LIMIT,PLANT_UP_LIMIT,PLANT_UP_STEP,PLANT_DOWN_STEP,ACT_STATUS,ACT_STOP,PROTO_VER,HELP\n",
    ACT_PROTO_VER
  );
}

void emitHelp() {
  Serial.println(
    "Commands: INIT_HOME CAM:x PLANT:y, SERVO CAM x, SERVO PLANT x, FEED STEP x, "
    "PLANT DOWN_LIMIT, PLANT UP_LIMIT, PLANT UP_STEP x, PLANT DOWN_STEP x, ACT_STATUS, ACT_STOP, PROTO_VER, HELP"
  );
}

void parseCommand(String cmd) {
  cmd.trim();
  cmd.toUpperCase();
  if (cmd.length() == 0) return;

  if (cmd.startsWith("INIT_HOME")) {
    int idxCam = cmd.indexOf("CAM:");
    int idxPlant = cmd.indexOf("PLANT:");
    if (idxCam >= 0) {
      int endCam = cmd.indexOf(' ', idxCam);
      if (endCam < 0) endCam = cmd.length();
      int camDeg = cmd.substring(idxCam + 4, endCam).toInt();
      setServoSafe(servoCam, servoCamDeg, camDeg);
    }
    if (idxPlant >= 0) {
      int plantDeg = cmd.substring(idxPlant + 6).toInt();
      setServoSafe(servoPlant, servoPlantDeg, plantDeg);
    }
    startPlantLimitRun(true);
    Serial.println("ACT_ACK INIT_HOME");
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

  if (cmd.startsWith("SERVO CAM")) {
    int deg = cmd.substring(String("SERVO CAM").length()).toInt();
    setServoSafe(servoCam, servoCamDeg, deg);
    Serial.printf("ACT_ACK SERVO_CAM %d\n", servoCamDeg);
    return;
  }

  if (cmd.startsWith("SERVO PLANT")) {
    int deg = cmd.substring(String("SERVO PLANT").length()).toInt();
    setServoSafe(servoPlant, servoPlantDeg, deg);
    Serial.printf("ACT_ACK SERVO_PLANT %d\n", servoPlantDeg);
    return;
  }

  if (cmd.startsWith("FEED STEP")) {
    int deg = cmd.substring(String("FEED STEP").length()).toInt();
    long steps = degToSteps(abs(deg));
    if (deg >= 0) feedMotor.move(steps);
    else feedMotor.move(-steps);
    Serial.printf("ACT_ACK FEED_STEP %ld\n", steps);
    return;
  }

  if (cmd == "PLANT DOWN_LIMIT") {
    startPlantLimitRun(false);
    return;
  }

  if (cmd == "PLANT UP_LIMIT") {
    startPlantLimitRun(true);
    return;
  }

  if (cmd.startsWith("PLANT UP_STEP")) {
    int deg = cmd.substring(String("PLANT UP_STEP").length()).toInt();
    long steps = degToSteps(abs(deg));
    plantMode = PLANT_IDLE;
    plantMotor.move(steps);
    Serial.printf("ACT_ACK PLANT_UP_STEP %ld\n", steps);
    return;
  }

  if (cmd.startsWith("PLANT DOWN_STEP")) {
    int deg = cmd.substring(String("PLANT DOWN_STEP").length()).toInt();
    long steps = degToSteps(abs(deg));
    plantMode = PLANT_IDLE;
    plantMotor.move(-steps);
    Serial.printf("ACT_ACK PLANT_DOWN_STEP %ld\n", steps);
    return;
  }

  if (cmd == "ACT_STOP") {
    stopAll();
    return;
  }

  if (cmd == "ACT_STATUS") {
    long feedPos = feedMotor.currentPosition();
    long plantPos = plantMotor.currentPosition();
    Serial.printf(
      "ACT_STAT CAM:%d PLANT_SERVO:%d FEED_POS:%ld PLANT_POS:%ld TOP:%d BOTTOM:%d APRIL_OK:0 TAG_ID:0 AB_CM:0 C_CM:0 DE_CM:0\n",
      servoCamDeg,
      servoPlantDeg,
      feedPos,
      plantPos,
      limitPressed(topLimit) ? 1 : 0,
      limitPressed(bottomLimit) ? 1 : 0
    );
    return;
  }

  Serial.print("ACT_ERR UNKNOWN:");
  Serial.println(cmd);
}

void setup() {
  Serial.begin(115200);

  pinMode(PLANT_LIMIT_TOP_PIN, INPUT_PULLUP);
  pinMode(PLANT_LIMIT_BOTTOM_PIN, INPUT_PULLUP);
  topLimit.raw = (digitalRead(PLANT_LIMIT_TOP_PIN) == LOW);
  topLimit.stable = topLimit.raw;
  topLimit.lastChange = millis();
  bottomLimit.raw = (digitalRead(PLANT_LIMIT_BOTTOM_PIN) == LOW);
  bottomLimit.stable = bottomLimit.raw;
  bottomLimit.lastChange = millis();

  pinMode(FEED_EN_PIN, OUTPUT);
  pinMode(PLANT_EN_PIN, OUTPUT);

  feedMotor.setEnablePin(FEED_EN_PIN);
  feedMotor.setPinsInverted(false, false, true);
  feedMotor.setMaxSpeed(2000);
  feedMotor.setAcceleration(1000);
  feedMotor.setMinPulseWidth(5);

  plantMotor.setEnablePin(PLANT_EN_PIN);
  plantMotor.setPinsInverted(true, false, true);
  plantMotor.setMaxSpeed(2000);
  plantMotor.setAcceleration(1000);
  plantMotor.setMinPulseWidth(5);

  enableDrivers();

  servoCam.setPeriodHertz(50);
  servoPlant.setPeriodHertz(50);
  servoCam.attach(SERVO_CAM_PIN, SERVO_MIN_US, SERVO_MAX_US);
  servoPlant.attach(SERVO_PLANT_PIN, SERVO_MIN_US, SERVO_MAX_US);
  setServoSafe(servoCam, servoCamDeg, servoCamDeg);
  setServoSafe(servoPlant, servoPlantDeg, servoPlantDeg);

  Serial.println("ACT_READY");
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

  bool topPressed = limitPressed(topLimit);
  bool bottomPressed = limitPressed(bottomLimit);

  if (plantMode == PLANT_RUN_UP_LIMIT) {
    if (topPressed) {
      plantMode = PLANT_IDLE;
      plantMotor.setSpeed(0);
      plantMotor.stop();
      plantMotor.setCurrentPosition(0);
      Serial.println("ACT_EVENT PLANT_HIT_TOP");
    } else {
      plantMotor.runSpeed();
    }
  } else if (plantMode == PLANT_RUN_DOWN_LIMIT) {
    if (bottomPressed) {
      plantMode = PLANT_IDLE;
      plantMotor.setSpeed(0);
      plantMotor.stop();
      Serial.println("ACT_EVENT PLANT_HIT_BOTTOM");
    } else {
      plantMotor.runSpeed();
    }
  } else {
    plantMotor.run();
  }

  feedMotor.run();
  delayMicroseconds(100);
}
