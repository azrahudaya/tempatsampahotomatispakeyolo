// File: arduino_controller.ino
// Arduino code for Smart Trash Bin actuator control with limit switches and safety

#include <Servo.h>

// DC motor pins (lid mechanism)
#define LID_MOTOR_PIN1 PINXX
#define LID_MOTOR_PIN2 PINXX
#define LID_MOTOR_STOP_PIN PINXX  // optional lid closed limit switch

// Servo motor pin (to open chamber slot)
#define SERVO_PIN PINXX
Servo chamberServo;

// Stepper motor pins (connected to driver like DM542)
#define STEPPER_STEP_PIN PINXX
#define STEPPER_DIR_PIN PINXX

// Limit switch pins for stepper safety
#define LIMIT_LEFT_PIN PINXX   // Leftmost limit switch (position 0)
#define LIMIT_RIGHT_PIN PINXX  // Rightmost limit switch (last chamber)

// Position mapping (e.g., in steps)
const int positions[4] = {0, 1000, 2000, 3000};
int currentPosition = 0;

void setup() {
  Serial.begin(9600);
  pinMode(LID_MOTOR_PIN1, OUTPUT);
  pinMode(LID_MOTOR_PIN2, OUTPUT);
  pinMode(STEPPER_STEP_PIN, OUTPUT);
  pinMode(STEPPER_DIR_PIN, OUTPUT);

  pinMode(LIMIT_LEFT_PIN, INPUT_PULLUP);
  pinMode(LIMIT_RIGHT_PIN, INPUT_PULLUP);
  pinMode(LID_MOTOR_STOP_PIN, INPUT_PULLUP);

  chamberServo.attach(SERVO_PIN);
  chamberServo.write(0); // closed position
}

void loop() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();

    if (command == "OPEN_LID") {
      openLid();
    } else if (command == "CLOSE_LID") {
      closeLid();
    } else if (command.startsWith("GOTO:")) {
      int target = command.substring(5).toInt();
      moveToChamber(target);
    } else if (command == "DROP") {
      releaseTrash();
    } else if (command == "HOME") {
      moveToChamber(0);
    } else {
      // perintah tidak dikenal
      Serial.println("ERR:perintah tidak dikenal");
    }
  }
}

void openLid() {
  // buka tutup, kasih timeout biar motor ga nyangkut
  digitalWrite(LID_MOTOR_PIN1, HIGH);
  digitalWrite(LID_MOTOR_PIN2, LOW);
  unsigned long startTime = millis();
  while (millis() - startTime < 2000) {
    if (digitalRead(LID_MOTOR_STOP_PIN) == LOW) {
      // tutup sudah kebuka penuh
      stopLid();
      Serial.println("OK:tutup sudah kebuka");
      return;
    }
  }
  stopLid();
  Serial.println("ERR:timeout buka tutup");
}

void closeLid() {
  // tutup tutup, ada timeout juga
  digitalWrite(LID_MOTOR_PIN1, LOW);
  digitalWrite(LID_MOTOR_PIN2, HIGH);
  unsigned long startTime = millis();
  while (digitalRead(LID_MOTOR_STOP_PIN) == HIGH && millis() - startTime < 3000) {
    // nunggu sampai tutup beneran nutup atau timeout
  }
  stopLid();
  if (digitalRead(LID_MOTOR_STOP_PIN) == LOW) {
    Serial.println("OK:tutup sudah nutup");
  } else {
    Serial.println("ERR:timeout tutup tutup");
  }
}

void stopLid() {
  // matiin motor tutup
  digitalWrite(LID_MOTOR_PIN1, LOW);
  digitalWrite(LID_MOTOR_PIN2, LOW);
}

void moveToChamber(int targetIndex) {
  // pindah ke chamber sesuai target, ada timeout biar aman
  if (targetIndex < 0 || targetIndex > 3) {
    Serial.println("ERR:chamber tidak ada");
    return;
  }

  int targetPos = positions[targetIndex];
  int steps = abs(targetPos - currentPosition);
  bool dir = targetPos > currentPosition;

  digitalWrite(STEPPER_DIR_PIN, dir);
  unsigned long startTime = millis();
  for (int i = 0; i < steps; i++) {
    // cek limit switch biar ga nabrak
    if (dir && digitalRead(LIMIT_RIGHT_PIN) == LOW) {
      Serial.println("ERR:limit kanan aktif");
      break;
    }
    if (!dir && digitalRead(LIMIT_LEFT_PIN) == LOW) {
      Serial.println("ERR:limit kiri aktif");
      break;
    }
    if (millis() - startTime > 5000) {
      Serial.println("ERR:timeout stepper");
      break;
    }
    digitalWrite(STEPPER_STEP_PIN, HIGH);
    delayMicroseconds(800);
    digitalWrite(STEPPER_STEP_PIN, LOW);
    delayMicroseconds(800);
  }
  currentPosition = targetPos;
  Serial.print("OK:chamber ke-");
  Serial.println(targetIndex);
}

void releaseTrash() {
  // buka chamber bawah pake servo
  chamberServo.write(90);  // buka
  delay(1500);
  chamberServo.write(0);   // tutup lagi
  delay(500);
  Serial.println("OK:chamber sudah dibuka");
}
