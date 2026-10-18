// Robotics arena sketch, split into separate parts:
//
//   PHASE_PICKUP       -> runPickup()      (search for the cube, grab it, drive onto SEP)
//   PHASE_LINE_FOLLOW  -> runLineFollow()  (line following from the end of SEP to EXIT)
//   PHASE_DONE         -> motors stopped (the claw is opened at the end of the course)
//
// All hardware (motors, IR sensors, 2 ultrasonic sensors, servo) is set up once in setup().
// To test the line following on its own, set START_PHASE to PHASE_LINE_FOLLOW.
//
// ASSUMPTIONS FOR THE PICKUP PART (see the settings block below)
//   - Robot starts at the START baseline, bottom-left, facing straight up the arena,
//     roughly in the lane of O1 (x = 5..25 cm), so it will have to dodge O1.
//   - The ABOVE ultrasonic sensor sees obstacles/walls; the BELOW one sees obstacles AND the cube.
//     Cube = "below sees something close, above sees nothing close or much farther".
//   - The gripper holds the cube when it is 1.5 in (3.81 cm) in front of the BELOW sensor.

#include <Servo.h>

// =====================================================================
//  PINS
// =====================================================================
// IR line sensors
#define IR_SENSOR_RIGHT   11
#define IR_SENSOR_LEFT    12
#define IR_SENSOR_MIDDLE  A2     // analog pins work as normal digital pins

// Ultrasonic sensors (HC-SR04 style: TRIG output, ECHO input)
#define US_ABOVE_TRIG     3
#define US_ABOVE_ECHO     A1     // analog pins work as normal digital pins; keeps pin 0 (RX) free for uploading
#define US_BELOW_TRIG     2
#define US_BELOW_ECHO     A0

// Servo
#define SERVO_PIN         4

// Motors
int enableRightMotor = 6;
int rightMotorPin1   = 7;
int rightMotorPin2   = 8;

int enableLeftMotor = 5;
int leftMotorPin1   = 9;
int leftMotorPin2   = 10;

// =====================================================================
//  SETTINGS
// =====================================================================
#define MOTOR_SPEED       200
#define SOFT_TURN_SPEED   65     // inner wheel speed for gentle steering (about 1/3 of MOTOR_SPEED)

// The robot drifts right (uneven weight), so the RIGHT motor runs a little faster everywhere.
// +10 means the right motor gets 210 when the left gets 200 (all right-motor speeds are scaled by 210/200).
// Still drifts right -> raise it (try 14). Now drifts left -> lower it (try 6).
#define RIGHT_MOTOR_TRIM  10
#define LINE_COLOR        HIGH   // IR output over the black line (flip to LOW if inverted)

// Servo angles (adjust to your gripper)
// The gripper starts open. If it closes the wrong way, swap OPEN and CLOSED.
// If the servo buzzes or stalls before the cube is gripped, move CLOSED closer to OPEN.
#define SERVO_START_ANGLE  90
#define SERVO_OPEN_ANGLE   90
#define SERVO_CLOSED_ANGLE 35

// Line-following timing (measure your robot)
#define SPEED_CM_PER_S    23     // distance per second at MOTOR_SPEED
#define LOST_TRAVEL_CM    30     // blind distance before deciding the line has ended
#define SIDE_MEMORY_MS    120    // a side sensor this recent + lost line = corner
#define PIVOT_MAX_MS      1500   // give up searching for the line after this long

const unsigned long LOST_STOP_MS = (unsigned long)LOST_TRAVEL_CM * 1000UL / SPEED_CM_PER_S;

// ---- Pickup settings -------------------------------------------------
// Detection
#define CUBE_DETECT_CM     35    // bottom sensor must see something closer than this to call it a cube
#define CUBE_ABOVE_GAP_CM  10    // ...and the above sensor must read at least this much farther
#define SAME_OBJECT_CM      8    // |above - below| under this = both sensors see the same obstacle
#define OBSTACLE_CM        25    // react to an obstacle closer than this
#define CUBE_GRAB_CM     3.81    // 1.5 in. Readings are float now, so no rounding tricks needed
#define SENSOR_GAP_MS      50    // pause between the two sensors so they don't hear each other

// Approach
#define APPROACH_SPEED    140    // PWM while far from the cube
#define APPROACH_STEP_MS  150    // drive time per step while far
#define CREEP_SPEED       130    // PWM for the last few cm
#define CREEP_MS           60    // drive time per creep pulse (about 1 cm)
#define FAR_CM             14    // above this distance use APPROACH_*, below it use CREEP_*
#define GRIP_SETTLE_MS    700    // wait for the servo to close

// Turning (open loop: these depend on battery and floor, so they are estimates)
#define TURN_SPEED        200    // PWM for scan steps
#define TURN_STEP_MS       55    // one scan step (roughly 6 to 12 degrees)
#define SCAN_SETTLE_MS     50    // let the robot stop before reading
#define SWEEP_STEPS         6    // scan this many steps left AND right of straight ahead
#define TURN_90_MS        405    // pivot about 90 degrees at MOTOR_SPEED (was 450 at 180, scaled; re-measure)

// Where to search (distances estimated from drive time at MOTOR_SPEED)
#define SEARCH_ZONE_CM    300    // start sweeping after driving this far
#define SWEEP_EVERY_CM     20    // then sweep every this many cm
#define WALL_ZONE_CM      330    // after this much driving, a close obstacle is the far wall, not O1/O2
#define MAX_DRIVE_CM      335    // after this, stop driving straight and sweep in place instead
#define ENDZONE_STEP_CM     8    // if the in-place sweep fails, move forward this much and try again
#define WALL_BACKOFF_CM    15    // when the far wall is close ahead, back off this much first

// Obstacle detour: sidestep right, pass, come back to the original lane.
// 40 cm to the side clears O1 and O2 from any lane they could block. The pass is long on purpose:
// if a pivot over-rotates, the return leg cuts back toward the obstacle and needs the extra room.
#define BACKUP_MS         250    // back up first (about 6 cm)
#define DETOUR_SIDE_CM     40
#define DETOUR_PASS_CM    100    // ~30 to reach the obstacle after backing up + 20 depth + robot length + margin

// Straight-line trim (the one value worth checking with a 2 m test drive). It is added to the RIGHT
// wheel's PWM while driving straight in the pickup phase. Robot veers LEFT -> make it negative (try -10);
// robot veers RIGHT -> make it positive (try +10). Leave at 0 if it already drives straight.
#define PICKUP_TRIM         0

// Safety timeouts
#define PICKUP_TIMEOUT_MS   90000UL
#define PAST_SEP_TIMEOUT_MS  2500UL

#define CM_TO_MS(cm) ((unsigned long)(cm) * 1000UL / SPEED_CM_PER_S)

// =====================================================================
//  PHASES
// =====================================================================
enum Phase { PHASE_PICKUP, PHASE_LINE_FOLLOW, PHASE_DONE };

#define START_PHASE PHASE_PICKUP    // use PHASE_LINE_FOLLOW to test line following alone

Phase phase = START_PHASE;

Servo myServo;

// Line-following state
int  lastSide = 0;                // -1 line last seen on left, +1 on right
unsigned long lastSideMs = 0;
bool lost = false;
bool pivoting = false;
unsigned long lostSince = 0;

// Add previous sensor snapshot so we can detect WBW->WBB and WBW->BBW transitions
int prevLeftSensor = 0;
int prevMiddleSensor = 0;
int prevRightSensor = 0;

// =====================================================================
//  SETUP: initialise everything
// =====================================================================
void setup()
{
  // Leave TCCR0B untouched so millis() and pulseIn() timing stay correct.

  // Motors
  pinMode(enableRightMotor, OUTPUT);
  pinMode(rightMotorPin1, OUTPUT);
  pinMode(rightMotorPin2, OUTPUT);
  pinMode(enableLeftMotor, OUTPUT);
  pinMode(leftMotorPin1, OUTPUT);
  pinMode(leftMotorPin2, OUTPUT);

  // IR sensors
  pinMode(IR_SENSOR_RIGHT, INPUT);
  pinMode(IR_SENSOR_LEFT, INPUT);
  pinMode(IR_SENSOR_MIDDLE, INPUT);

  // Ultrasonic sensors
  pinMode(US_ABOVE_TRIG, OUTPUT);
  pinMode(US_ABOVE_ECHO, INPUT);
  pinMode(US_BELOW_TRIG, OUTPUT);
  pinMode(US_BELOW_ECHO, INPUT);

  // Servo
  myServo.attach(SERVO_PIN);
  myServo.write(SERVO_START_ANGLE);

  rotateMotor(0, 0);

  if (phase == PHASE_LINE_FOLLOW)
  {
    startLineFollow();
  }
}

// =====================================================================
//  MAIN LOOP: just runs whichever phase we are in
// =====================================================================
void loop()
{
  switch (phase)
  {
    case PHASE_PICKUP:
      runPickup();
      startLineFollow();             // pickup finished -> hand over to line following
      phase = PHASE_LINE_FOLLOW;
      break;

    case PHASE_LINE_FOLLOW:
      runLineFollow();
      break;

    case PHASE_DONE:
      rotateMotor(0, 0);
      break;
  }
}

// =====================================================================
//  PART 1: PICKUP
// =====================================================================
// Called once. Returns when the cube is gripped and the robot is on the SEP block,
// facing the start of the line.
//
// Plan:
//   1. Drive straight up the arena. If both ultrasonic sensors see the same thing
//      close ahead, it is an obstacle: detour around it (avoidObstacle).
//   2. If only the BELOW sensor sees something close, it is the cube: approach it
//      (approachCube) and close the gripper at 3.81 cm.
//   3. Near the pickup area, stop every SWEEP_EVERY_CM and sweep left/right looking
//      for the cube in case it is not straight ahead (sweepForCube).
//   4. With the cube gripped, turn right toward SEP and drive until an IR sensor
//      sees it, then hand over to line following.

// ---------- sensing helpers ----------
// Median of 3 readings. readCM() returns 0 when there is no echo,
// so 0 is treated as "far away" (999).
float readClean(int trigPin, int echoPin)
{
  float a = readCM(trigPin, echoPin); delay(10);
  float b = readCM(trigPin, echoPin); delay(10);
  float c = readCM(trigPin, echoPin);
  if (a <= 0) a = 999;
  if (b <= 0) b = 999;
  if (c <= 0) c = 999;
  if (a > b) { float t = a; a = b; b = t; }
  if (b > c) { float t = b; b = c; c = t; }
  if (a > b) { float t = a; a = b; b = t; }
  return b;
}

// Read both sensors, with a gap between them so they don't hear each other's ping
void readBoth(float &below, float &above)
{
  below = readClean(US_BELOW_TRIG, US_BELOW_ECHO);
  delay(SENSOR_GAP_MS);
  above = readClean(US_ABOVE_TRIG, US_ABOVE_ECHO);
}

// Cube: bottom sensor sees it, top sensor looks over it
bool isCube(float below, float above)
{
  return below < CUBE_DETECT_CM && (above - below) > CUBE_ABOVE_GAP_CM;
}

// Obstacle: both sensors see the same thing at about the same distance
bool isObstacle(float below, float above)
{
  return below < OBSTACLE_CM && abs(above - below) < SAME_OBJECT_CM;
}

// Used while scanning. One quick reading first, full check only if something is close.
bool cubeVisible()
{
  delay(SCAN_SETTLE_MS);
  float quick = readCM(US_BELOW_TRIG, US_BELOW_ECHO);
  if (quick <= 0 || quick >= CUBE_DETECT_CM) return false;

  float below, above;
  readBoth(below, above);
  return isCube(below, above);
}

// ---------- movement helpers ----------
// dir = +1 pivots right, -1 pivots left (same sense as the line-follow pivots)
void turnStep(int dir)
{
  rotateMotor(-dir * TURN_SPEED, dir * TURN_SPEED);
  delay(TURN_STEP_MS);
  rotateMotor(0, 0);
}

void pivotMs(int dir, unsigned long ms)
{
  rotateMotor(-dir * MOTOR_SPEED, dir * MOTOR_SPEED);
  delay(ms);
  rotateMotor(0, 0);
}

// Forward at MOTOR_SPEED. PICKUP_TRIM compensates for one motor being stronger than the other.
void driveStraight()
{
  rotateMotor(constrain(MOTOR_SPEED + PICKUP_TRIM, 0, 255), MOTOR_SPEED);
}

void driveMs(unsigned long ms)
{
  driveStraight();
  delay(ms);
  rotateMotor(0, 0);
}

void driveBackMs(unsigned long ms)
{
  rotateMotor(-MOTOR_SPEED, -MOTOR_SPEED);
  delay(ms);
  rotateMotor(0, 0);
}

// Back up, sidestep right, pass the obstacle, come back to the original lane.
// The turns go right, left, left, right, so any constant turning error cancels out.
void avoidObstacle()
{
  rotateMotor(-MOTOR_SPEED, -MOTOR_SPEED);
  delay(BACKUP_MS);
  rotateMotor(0, 0);

  pivotMs(+1, TURN_90_MS);  driveMs(CM_TO_MS(DETOUR_SIDE_CM));   // sidestep right
  pivotMs(-1, TURN_90_MS);  driveMs(CM_TO_MS(DETOUR_PASS_CM));   // pass the obstacle
  pivotMs(-1, TURN_90_MS);  driveMs(CM_TO_MS(DETOUR_SIDE_CM));   // back to the lane
  pivotMs(+1, TURN_90_MS);                                       // face forward again
}

// ---------- search ----------
// Turn in direction d until the cube leaves view. Returns how many steps it stayed visible.
int turnUntilLost(int d, int maxSteps)
{
  int n = 0;
  while (n < maxSteps)
  {
    turnStep(d);
    if (!cubeVisible()) break;
    n++;
  }
  return n;
}

// Sweep left, then right, then back to centre. If the cube shows up, keep turning until it
// leaves view, then step back to the middle of it and return true (robot now faces the cube).
// If nothing is found the robot ends up facing straight ahead again and returns false.
bool sweepForCube()
{
  int offset = 0;                                   // steps from straight ahead, + = right
  int targets[3] = { -SWEEP_STEPS, SWEEP_STEPS, 0 };

  for (int t = 0; t < 3; t++)
  {
    while (offset != targets[t])
    {
      int d = (targets[t] > offset) ? +1 : -1;
      turnStep(d);
      offset += d;

      if (t < 2 && cubeVisible())                   // only scan on the two sweep legs
      {
        int seen = 1 + turnUntilLost(d, 2 * SWEEP_STEPS);
        for (int i = 0; i < (seen + 1) / 2; i++) turnStep(-d);   // centre on the cube
        return true;
      }
    }
  }
  return false;
}

// The cube is in view but maybe off to one side (the ultrasonic beam is wide).
// Find both edges of the beam window and aim at the middle.
void centerOnCube()
{
  turnUntilLost(-1, 2 * SWEEP_STEPS);               // turn left until it disappears
  int guard = 0;
  bool back = false;
  while (!back && guard++ < 2 * SWEEP_STEPS)        // step right until it reappears
  {
    turnStep(+1);
    back = cubeVisible();
  }
  if (!back) return;                                // lost it completely: caller will handle it
  int w = 1 + turnUntilLost(+1, 2 * SWEEP_STEPS);   // width of the window, in steps
  for (int i = 0; i < (w + 1) / 2; i++) turnStep(-1);
}

// Drive toward the cube, creeping for the last stretch. Returns true once it is gripped.
bool approachCube()
{
  centerOnCube();

  int misses = 0;
  bool recentred = false;
  for (int n = 0; n < 80; n++)
  {
    float below, above;
    readBoth(below, above);

    if (!isCube(below, above))
    {
      misses++;
      if (misses >= 2)
      {
        if (!sweepForCube()) return false;          // cube really gone: back to searching
        misses = 0;
      }
      continue;
    }
    misses = 0;

    if (below <= CUBE_GRAB_CM)
    {
      rotateMotor(0, 0);
      myServo.write(SERVO_CLOSED_ANGLE);
      delay(GRIP_SETTLE_MS);
      return true;
    }

    if (below <= FAR_CM && !recentred)              // aim again once we are close
    {
      centerOnCube();
      recentred = true;
      continue;
    }

    if (below > FAR_CM)
    {
      rotateMotor(APPROACH_SPEED, APPROACH_SPEED);
      delay(APPROACH_STEP_MS);
    }
    else
    {
      rotateMotor(CREEP_SPEED, CREEP_SPEED);
      delay(CREEP_MS);
    }
    rotateMotor(0, 0);
  }
  return false;
}

// ---------- the pickup routine ----------
void runPickup()
{
  myServo.write(SERVO_OPEN_ANGLE);

  const unsigned long searchZoneMs = CM_TO_MS(SEARCH_ZONE_CM);
  const unsigned long maxDriveMs   = CM_TO_MS(MAX_DRIVE_CM);
  const unsigned long wallZoneMs   = CM_TO_MS(WALL_ZONE_CM);
  const unsigned long sweepEveryMs = CM_TO_MS(SWEEP_EVERY_CM);

  unsigned long startMs = millis();
  unsigned long lastMs = startMs;
  unsigned long drivenMs = 0;          // estimated forward travel, as drive time
  unsigned long lastSweepAt = 0;
  bool gotCube = false;

  while (millis() - startMs < PICKUP_TIMEOUT_MS)
  {
    unsigned long now = millis();
    unsigned long dt = now - lastMs;
    lastMs = now;

    float below, above;
    readBoth(below, above);

    // 1) Cube in view: stop and re-check while standing still (the two readings above were
    //    taken a moment apart on the move and can disagree at the edge of the beam), then go get it
    if (isCube(below, above))
    {
      rotateMotor(0, 0);
      if (cubeVisible())
      {
        if (approachCube()) { gotCube = true; break; }
      }
      lastMs = millis();
      continue;
    }

    // 2) Something close ahead seen by both sensors
    if (isObstacle(below, above))
    {
      rotateMotor(0, 0);
      if (drivenMs < wallZoneMs)
      {
        avoidObstacle();                                        // O1 / O2: go around
        drivenMs += CM_TO_MS(DETOUR_PASS_CM) - BACKUP_MS;       // net forward progress of the detour
      }
      else
      {
        driveBackMs(CM_TO_MS(WALL_BACKOFF_CM));                 // far wall: back off and search here
        if (drivenMs < maxDriveMs) drivenMs = maxDriveMs;
      }
      lastMs = millis();
      continue;
    }

    // 3) End of the drive: sweep in place, nudging forward if nothing is found.
    //    (Step 2 catches the far wall, so this can't run into it.)
    if (drivenMs >= maxDriveMs)
    {
      rotateMotor(0, 0);
      if (sweepForCube() && approachCube()) { gotCube = true; break; }
      driveMs(CM_TO_MS(ENDZONE_STEP_CM));
      lastMs = millis();
      continue;
    }

    // 4) Otherwise keep driving; sweep periodically once we are near the pickup area
    driveStraight();
    drivenMs += dt;

    if (drivenMs >= searchZoneMs && (drivenMs - lastSweepAt) >= sweepEveryMs)
    {
      rotateMotor(0, 0);
      lastSweepAt = drivenMs;
      if (sweepForCube() && approachCube()) { gotCube = true; break; }
      lastMs = millis();
    }
  }

  rotateMotor(0, 0);
  if (!gotCube) return;                              // timed out: hand over anyway

  // Cube is gripped. Turn right toward SEP and drive until an IR sensor sees it.
  pivotMs(+1, TURN_90_MS);
  unsigned long t0 = millis();
  driveStraight();
  while (millis() - t0 < PAST_SEP_TIMEOUT_MS)
  {
    if (digitalRead(IR_SENSOR_LEFT)   == LINE_COLOR ||
        digitalRead(IR_SENSOR_MIDDLE) == LINE_COLOR ||
        digitalRead(IR_SENSOR_RIGHT)  == LINE_COLOR) break;
  }
  rotateMotor(0, 0);
}

// =====================================================================
//  PART 2: LINE FOLLOWING (from the end of SEP to EXIT)
// =====================================================================
void startLineFollow()
{
  lastSide = 0;
  lastSideMs = millis();
  lost = false;
  pivoting = false;
  lostSince = 0;

  // reset previous sensor snapshot
  prevLeftSensor = 0;
  prevMiddleSensor = 0;
  prevRightSensor = 0;
}

// One step of line following. Called repeatedly from loop().
void runLineFollow()
{
  bool right  = (digitalRead(IR_SENSOR_RIGHT)  == LINE_COLOR);
  bool middle = (digitalRead(IR_SENSOR_MIDDLE) == LINE_COLOR);
  bool left   = (digitalRead(IR_SENSOR_LEFT)   == LINE_COLOR);
  unsigned long now = millis();

  // Detect transition from WBW (middle only) to WBB (middle+right) => turn right
  // or to BBW (left+middle) => turn left. We use the snapshot from the previous call.
  if (prevLeftSensor == 0 && prevMiddleSensor == 1 && prevRightSensor == 0)
  {
    if (!left && middle && right)
    {
      // WBW -> WBB: initiate a right turn step and return to let the loop re-evaluate sensors
      turnStep(+1);
      prevLeftSensor = left ? 1 : 0;
      prevMiddleSensor = middle ? 1 : 0;
      prevRightSensor = right ? 1 : 0;
      return;
    }
    else if (left && middle && !right)
    {
      // WBW -> BBW: initiate a left turn step and return to let the loop re-evaluate sensors
      turnStep(-1);
      prevLeftSensor = left ? 1 : 0;
      prevMiddleSensor = middle ? 1 : 0;
      prevRightSensor = right ? 1 : 0;
      return;
    }
  }

  if (left || middle || right)
  {
    lost = false;

    // Remember which side the line is on (used when the line vanishes at a corner)
    if (right && !left)      { lastSide = +1; lastSideMs = now; }
    else if (left && !right) { lastSide = -1; lastSideMs = now; }

    if (left && right)
    {
      // Black on both sides: SEP block, diamond vertices, crossings -> straight
      rotateMotor(MOTOR_SPEED, MOTOR_SPEED);
    }
    else if (middle)
    {
      // Middle on the line: go straight, ignore side branches
      rotateMotor(MOTOR_SPEED, MOTOR_SPEED);
    }
    else if (right)
    {
      rotateMotor(SOFT_TURN_SPEED, MOTOR_SPEED);   // steer right
    }
    else
    {
      rotateMotor(MOTOR_SPEED, SOFT_TURN_SPEED);   // steer left
    }
  }
  else
  {
    // No sensor sees the line
    if (!lost)
    {
      lost = true;
      lostSince = now;
      // Side sensor saw the line just before it vanished -> corner, pivot toward it.
      // Otherwise we were centred -> gap or end of line, go straight.
      pivoting = (lastSide != 0) && (now - lastSideMs < SIDE_MEMORY_MS);
    }

    unsigned long lostFor = now - lostSince;

    if (pivoting)
    {
      if (lostFor > PIVOT_MAX_MS)
      {
        finishCourse(false);                       // could not find the line again: keep the cube
      }
      else if (lastSide > 0)
      {
        rotateMotor(-MOTOR_SPEED, MOTOR_SPEED);    // pivot right
      }
      else
      {
        rotateMotor(MOTOR_SPEED, -MOTOR_SPEED);    // pivot left
      }
    }
    else
    {
      if (lostFor > LOST_STOP_MS)
      {
        finishCourse(true);                        // line ended: we are in the EXIT box, release the cube
      }
      else
      {
        rotateMotor(MOTOR_SPEED, MOTOR_SPEED);     // cross the gap
      }
    }
  }

  // update previous sensor snapshot for the next step
  prevLeftSensor = left ? 1 : 0;
  prevMiddleSensor = middle ? 1 : 0;
  prevRightSensor = right ? 1 : 0;
}

// Stop the robot. If the course finished normally, open the claw to release the cube.
void finishCourse(bool openClaw)
{
  rotateMotor(0, 0);
  if (openClaw)
  {
    myServo.write(SERVO_OPEN_ANGLE);
    delay(GRIP_SETTLE_MS);                         // give the servo time to open
  }
  phase = PHASE_DONE;
}

// =====================================================================
//  HELPERS
// =====================================================================
// Distance in cm as a float, or 0 if no echo came back within the 30 ms timeout (about 5 m)
float readCM(int trigPin, int echoPin)
{
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 30000UL);
  return duration * 0.0343 / 2.0;
}

void rotateMotor(int rightMotorSpeed, int leftMotorSpeed)
{
  // Make the right motor a bit faster to cancel the drift to the right
  rightMotorSpeed = constrain((long)rightMotorSpeed * (MOTOR_SPEED + RIGHT_MOTOR_TRIM) / MOTOR_SPEED, -255, 255);

  if (rightMotorSpeed < 0)
  {
    digitalWrite(rightMotorPin1, LOW);
    digitalWrite(rightMotorPin2, HIGH);
  }
  else if (rightMotorSpeed > 0)
  {
    digitalWrite(rightMotorPin1, HIGH);
    digitalWrite(rightMotorPin2, LOW);
  }
  else
  {
    digitalWrite(rightMotorPin1, LOW);
    digitalWrite(rightMotorPin2, LOW);
  }

  if (leftMotorSpeed < 0)
  {
    digitalWrite(leftMotorPin1, LOW);
    digitalWrite(leftMotorPin2, HIGH);
  }
  else if (leftMotorSpeed > 0)
  {
    digitalWrite(leftMotorPin1, HIGH);
    digitalWrite(leftMotorPin2, LOW);
  }
  else
  {
    digitalWrite(leftMotorPin1, LOW);
    digitalWrite(leftMotorPin2, LOW);
  }

  analogWrite(enableRightMotor, abs(rightMotorSpeed));
  analogWrite(enableLeftMotor, abs(leftMotorSpeed));
}
