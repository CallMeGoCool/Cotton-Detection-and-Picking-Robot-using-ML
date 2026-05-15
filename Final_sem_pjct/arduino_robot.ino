#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Servo.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
Servo gripper;

// -------- PCA9685 SETTINGS --------
#define SERVOMIN 120
#define SERVOMAX 650
#define BASE        0
#define SHOULDER    2
#define ELBOW       4
#define WRIST_R     6
#define WRIST_V     8

// -------- GRIPPER PIN --------
#define GRIP_PIN 9

// =====================================================
// FIXED POSITIONS — same for ALL grids
// =====================================================
int neutral[5]     = {90, 90, 100, 30, 30};
int place_above[5] = {80, 95, 145, 20, 20};
int place_down[5]  = {80, 95, 150, 20, 20};
int mid1[5]        = {60, 80, 100, 20, 20};
int mid2[5]        = {75, 85, 105, 25, 20};

// =====================================================
// DYNAMIC POSITIONS — set per grid
// =====================================================
int pick_above[5]  = {10, 25, 110, 0, 0};
int pick_down[5]   = {7,  23, 110, 0, 0};
int gripCloseVal   = 40;
int gripOpenVal    = 120;

// =====================================================
// GRID CENTERS — averaged from all your data points
// =====================================================
struct GridCenter {
  char  name;
  float cx, cy, cz;
};

GridCenter grids[] = {
  { 'A',  1.05,  3.47, 31.05 },
  { 'B', -2.19,  0.96, 32.48 },
  { 'C', -6.38, -2.09, 36.06 },
  { 'D',  4.71,  0.88, 34.47 },
  { 'E',  0.50, -2.10, 39.25 },
  { 'F', -3.44, -5.56, 39.56 },
  { 'G',  7.50, -2.50, 37.67 },
  { 'H',  3.87, -5.52, 41.35 },
  { 'I',  0.00, -8.17, 43.83 }
};

#define NUM_GRIDS 9

// Half-diagonal of 7.5cm grid = 5.3cm
// +2cm sensor noise buffer = 7.5 max
#define MAX_DIST 7.5

// -------- HELPERS --------
int angleToPWM(float angle) {
  return map(angle, 0, 180, SERVOMIN, SERVOMAX);
}

float euclideanDist(float x, float y, float z,
                    float cx, float cy, float cz) {
  float dx = x - cx;
  float dy = y - cy;
  float dz = z - cz;
  return sqrt(dx*dx + dy*dy + dz*dz);
}

// Returns closest grid name or '?' if too far
char detectGrid(float x, float y, float z) {
  float minDist = 9999.0;
  char  best    = '?';

  Serial.println("--- Distance Check ---");
  for (int i = 0; i < NUM_GRIDS; i++) {
    float d = euclideanDist(x, y, z,
                            grids[i].cx,
                            grids[i].cy,
                            grids[i].cz);
    Serial.print("  Grid ");
    Serial.print(grids[i].name);
    Serial.print(": ");
    Serial.println(d, 2);

    if (d < minDist) {
      minDist = d;
      best    = grids[i].name;
    }
  }

  Serial.print("Closest: ");
  Serial.print(best);
  Serial.print(" (dist=");
  Serial.print(minDist, 2);
  Serial.println(")");

  if (minDist > MAX_DIST) {
    Serial.println("Rejected — too far, staying neutral");
    return '?';
  }

  return best;
}

// =====================================================
// SET PICK ANGLES + GRIPPER VALUES PER GRID
// =====================================================
void setGridConfig(int pa0, int pa1, int pa2,
                   int pd0, int pd1, int pd2,
                   int gClose, int gOpen) {
  pick_above[0] = pa0; pick_above[1] = pa1;
  pick_above[2] = pa2; pick_above[3] = 0; pick_above[4] = 0;

  pick_down[0]  = pd0; pick_down[1]  = pd1;
  pick_down[2]  = pd2; pick_down[3]  = 0; pick_down[4]  = 0;

  gripCloseVal  = gClose;
  gripOpenVal   = gOpen;
}

// -------- MOVEMENT --------
void moveSmooth(int target[5]) {
  static float current[5] = {90, 90, 90, 90, 90};
  int steps = 60;
  float step[5];

  for (int i = 0; i < 5; i++)
    step[i] = (target[i] - current[i]) / (float)steps;

  for (int s = 0; s < steps; s++) {
    for (int i = 0; i < 5; i++)
      current[i] += step[i];

    pwm.setPWM(BASE,     0, angleToPWM(current[0]));
    pwm.setPWM(SHOULDER, 0, angleToPWM(current[1]));
    pwm.setPWM(ELBOW,    0, angleToPWM(current[2]));
    pwm.setPWM(WRIST_R,  0, angleToPWM(current[3]));
    pwm.setPWM(WRIST_V,  0, angleToPWM(current[4]));
    delay(15);
  }

  for (int i = 0; i < 5; i++) current[i] = target[i];
}

// -------- GRIPPER --------
void gripClose() {
  for (int pos = gripOpenVal; pos >= gripCloseVal; pos--) {
    gripper.write(pos);
    delay(10);
  }
  delay(2000);
}

void gripOpen() {
  for (int pos = gripCloseVal; pos <= gripOpenVal; pos++) {
    gripper.write(pos);
    delay(10);
  }
  delay(1500);
}

// -------- PICK & PLACE --------
void executePickPlace() {
  moveSmooth(neutral);
  gripOpen();

  moveSmooth(mid1);
  moveSmooth(pick_above);
  moveSmooth(pick_down);

  gripClose();

  moveSmooth(pick_above);
  moveSmooth(mid2);

  moveSmooth(place_above);
  moveSmooth(place_down);

  gripOpen();

  moveSmooth(place_above);
  moveSmooth(neutral);
}

// -------- SETUP --------
void setup() {
  pwm.begin();
  pwm.setPWMFreq(50);
  gripper.attach(GRIP_PIN);
  Serial.begin(9600);
  moveSmooth(neutral);  // start at neutral, wait for commands
}

// -------- LOOP --------
void loop() {

  if (!Serial.available()) return;  // stay in neutral, do nothing

  String data = Serial.readStringUntil('\n');
  data.trim();

  // Parse x,y,z
  int c1 = data.indexOf(',');
  int c2 = data.indexOf(',', c1 + 1);
  if (c1 == -1 || c2 == -1) {
    Serial.println("Bad format — skipping");
    return;
  }

  float x = data.substring(0, c1).toFloat();
  float y = data.substring(c1 + 1, c2).toFloat();
  float z = data.substring(c2 + 1).toFloat();

  Serial.print("\nReceived → X:"); Serial.print(x);
  Serial.print(" Y:"); Serial.print(y);
  Serial.print(" Z:"); Serial.println(z);

  // Detect grid using Euclidean distance
  char grid = detectGrid(x, y, z);

  // =====================================================
  // ASSIGN ANGLES PER GRID
  // setGridConfig(pick_above[0,1,2], pick_down[0,1,2],
  //               gripClose, gripOpen)
  // =====================================================

  if (grid == 'A') {
    Serial.println("Grid: A — not calibrated yet, skipping");
    // setGridConfig(?, ?, ?,  ?, ?, ?,  ?, ?);
    return;
  }

  else if (grid == 'B') {
    Serial.println("Grid: B");
    //              pa0  pa1  pa2   pd0  pd1  pd2   close open
    setGridConfig(  0,   45,  140,   0,   40,  140,   20,  70 );
  }

  else if (grid == 'C') {
    Serial.println("Grid: C");
    setGridConfig(  15,  45,  140,   15,  40,  140,   20,  70 );
  }

  else if (grid == 'D') {
    Serial.println("Grid: D — not calibrated yet, skipping");
    // setGridConfig(?, ?, ?,  ?, ?, ?,  ?, ?);
    return;
  }

  else if (grid == 'E') {
    Serial.println("Grid: E");
    setGridConfig(  10,  25,  110,   7,   23,  110,   40,  120 );
  }

  else if (grid == 'F') {
    Serial.println("Grid: F");
    setGridConfig(  20,  25,  110,   20,  23,  110,   40,  80 );
  }

  else if (grid == 'G') {
    Serial.println("Grid: G");
    setGridConfig(  0,   12,  80,    0,   9,   80,    30,  80 );
  }

  else if (grid == 'H') {
    Serial.println("Grid: H");
    setGridConfig(  20,  12,  80,    20,  9,   80,    20,  70 );
  }

  else if (grid == 'I') {
    Serial.println("Grid: I");
    setGridConfig(  0,   45,  140,   0,   40,  140,   20,  70 );
  }

  else {
    // '?' — no grid matched or too far
    // Arm stays at neutral, nothing happens
    Serial.println("No cotton detected — staying neutral");
    return;
  }

  // Execute pick and place
  executePickPlace();
  delay(5000);  // wait for camera to update next frame
}