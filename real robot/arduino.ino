
// ─── CONFIGURATION ───────────────────────────────────────────────────────────
const bool USE_ENCODERS = false; // Set to true to use ticks, false to use milliseconds
const int MOVE_SPEED = 230;      // Speed for driving forward/backward (0-255)
const int TURN_SPEED = 190;      // Speed for turning left/right (0-255)
// ─────────────────────────────────────────────────────────────────────────────

const int DIR1 = 6;
const int PWM1 = 10;
const int DIR2 = 5;
const int PWM2 = 9;
const int L_ENC_A = 3;

volatile long left_ticks = 0;
long target_ticks = 0;

unsigned long move_start_time = 0;
unsigned long move_duration = 0;
bool is_moving = false;

void encoderISR() {
  left_ticks++;
}

void setup() {
  Serial.begin(115200);
  pinMode(DIR1, OUTPUT);
  pinMode(PWM1, OUTPUT);
  pinMode(DIR2, OUTPUT);
  pinMode(PWM2, OUTPUT);
  pinMode(L_ENC_A, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(L_ENC_A), encoderISR, RISING);
  stopRobot();
}

void loop() {
  // 1. Monitor Non-Blocking Serial Inputs
  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd.length() > 0) {
      char type = toupper(cmd.charAt(0));
      long val = cmd.length() > 1 ? cmd.substring(1).toInt() : 0;

      if (type == 'S') {
        stopRobot();
        Serial.println("DONE");
      } else if (val > 0) {
        left_ticks = 0;
        target_ticks = val;
        move_duration = val;
        move_start_time = millis();
        is_moving = true;

        if (type == 'F') {
          digitalWrite(DIR1, LOW);  digitalWrite(DIR2, LOW);
          analogWrite(PWM1, MOVE_SPEED); analogWrite(PWM2, MOVE_SPEED);
        } else if (type == 'B') {
          digitalWrite(DIR1, HIGH); digitalWrite(DIR2, HIGH);
          analogWrite(PWM1, MOVE_SPEED); analogWrite(PWM2, MOVE_SPEED);
        } else if (type == 'R') {
          digitalWrite(DIR1, LOW);  digitalWrite(DIR2, HIGH);
          analogWrite(PWM1, TURN_SPEED); analogWrite(PWM2, TURN_SPEED);
        } else if (type == 'L') {
          digitalWrite(DIR1, HIGH); digitalWrite(DIR2, LOW);
          analogWrite(PWM1, TURN_SPEED); analogWrite(PWM2, TURN_SPEED);
        }
      }
    }
  }

  // 2. Continuous Non-Blocking Target Distance/Time Monitor
  if (is_moving) {
    if (USE_ENCODERS) {
      if (left_ticks >= target_ticks) {
        stopRobot();
        Serial.println("DONE");
      }
    } else {
      if (millis() - move_start_time >= move_duration) {
        stopRobot();
        Serial.println("DONE");
      }
    }
  }
}

void stopRobot() {
  is_moving = false;
  analogWrite(PWM1, 0);   analogWrite(PWM2, 0);
  digitalWrite(DIR1, LOW); digitalWrite(DIR2, LOW);
}