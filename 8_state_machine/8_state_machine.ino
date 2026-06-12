#include <Arduino.h>
#include <RPLidar.h>
#include <math.h>
#include "LocalMecanumDriver.h"

/*
  Figure-8 (8字形) foam track state machine.

  Track layout: a rectangular loop with a crossbar through the middle that
  forms a figure-8. The car starts on the crossbar, travelling toward the
  first T-junction.

  Route (one full lap = 8 turns):
    Half A (4 right turns): crossbar → T-junction right → corner right →
                             corner right → T-junction right → back on crossbar
    Half B (4 left turns):  crossbar → T-junction left → corner left →
                             corner left → T-junction left → back on crossbar
  Then flip direction and repeat indefinitely.

  T-junctions look identical to normal corners from the LiDAR's perspective
  (front wall present, one side open), so the same corner-detection logic works.

  Lidar angle convention:
    lidar 180 deg -> car front/head, about 8 cm behind the front bumper
    lidar   0 deg -> car rear/tail, about 20 cm in front of the rear bumper
    lidar  90 deg -> car left side
    lidar 270 deg -> car right side

  Mecanum command convention:
    forward      = ( +, +, +, + )
    right strafe = ( +, -, -, + )
    right rotate = ( +, -, +, - )
*/

// -------- Hardware --------
RPLidar lidar;
LocalMecanumDriver mecanum(9, 8, 12, 13, 11, 10, 46, 21);

// -------- Lidar angle convention --------
static const int FRONT_DEG = 180;
static const int REAR_DEG = 0;
static const int LEFT_DEG = 90;
static const int RIGHT_DEG = 270;

static const int LEFT_FRONT_DEG = 135;
static const int RIGHT_FRONT_DEG = 225;
static const int LEFT_REAR_DEG = 45;
static const int RIGHT_REAR_DEG = 315;

// -------- Track geometry --------
static const float TRACK_WIDTH_M = 0.45f;
static const float CAR_WIDTH_M = 0.23f;
static const float LIDAR_TO_FRONT_BUMPER_M = 0.08f;

static const float SIDE_TARGET_M = TRACK_WIDTH_M * 0.5f;
static const float BODY_HALF_WIDTH_M = CAR_WIDTH_M * 0.5f;
static const float NOMINAL_SIDE_CLEARANCE_M = (TRACK_WIDTH_M - CAR_WIDTH_M) * 0.5f;

// -------- Lidar filtering --------
static const float LIDAR_MIN_VALID_M = 0.11f;
static const float LIDAR_MAX_VALID_M = 4.00f;
static const float INVALID_RANGE_M = 9.99f;

static const int MIN_SCAN_POINTS = 120;
static const int MIN_SECTOR_POINTS = 3;
static const int ROBUST_SAMPLES = 5;

// -------- Distance thresholds --------
static const float FRONT_SLOW_M = 0.84f;
static const float FRONT_TURN_TRIGGER_M = 0.44f;
static const float FRONT_HARD_BRAKE_M = 0.25f;
static const float FRONT_CLEAR_AFTER_TURN_M = 0.56f;

static const float SIDE_TOO_CLOSE_M = BODY_HALF_WIDTH_M + 0.045f;
static const float SIDE_EMERGENCY_M = BODY_HALF_WIDTH_M + 0.025f;
static const float SIDE_VALID_MAX_M = 0.64f;
static const float SIDE_OPEN_M = 0.48f;
static const float DIAGONAL_OPEN_M = 0.52f;
static const float TRACK_PRESENT_MAX_M = 0.95f;

static const float CENTER_OK_M = 0.060f;
static const float YAW_OK_M = 0.075f;
static const float ALIGN_YAW_OK_M = 0.100f;
static const float TURN_EXIT_YAW_OK_M = 0.150f;

// -------- Motion tuning, logical PWM range is -100..100 --------
static const int STRAIGHT_FAST_PWM = 100;
static const int STRAIGHT_SLOW_PWM = 36;
static const int ALIGN_FORWARD_PWM = 24;
static const int ALIGN_ROTATE_PWM = 36;
static const int PRE_TURN_ROTATE_PWM = 12;
static const int CORNER_APPROACH_FORWARD_PWM = 12;
static const int TURN_ROTATE_FAST_PWM = 54;
static const int TURN_ROTATE_SLOW_PWM = 42;
static const int TURN_ROTATE_FINE_PWM = 30;
static const int TURN_ROTATE_BRAKE_PWM = 22;
static const int TURN_ARC_FORWARD_PWM = 16;
static const int TURN_EXIT_FORWARD_PWM = 18;
static const int CORNER_BACKOFF_PWM = 18;

static const float STRAFE_KP = 145.0f;
static const float YAW_KP = 110.0f;
static const int STRAFE_LIMIT_PWM = 28;
static const int TURN_STRAFE_LIMIT_PWM = 18;
static const int YAW_LIMIT_STRAIGHT_PWM = 17;
static const int YAW_LIMIT_ALIGN_PWM = 26;

// -------- Time/debounce tuning --------
static const unsigned long DEBUG_PERIOD_MS = 700;
static const unsigned long ACQUIRE_MIN_MS = 450;
static const unsigned long ACQUIRE_MAX_MS = 2200;
static const unsigned long ALIGN_MIN_MS = 80;
static const unsigned long ALIGN_MAX_MS = 1200;
static const unsigned long MIN_STRAIGHT_BEFORE_CORNER_MS = 250;
static const unsigned long CORNER_DECIDE_MAX_MS = 750;
static const unsigned long TURN_OPEN_LOOP_MIN_MS = 640;
static const unsigned long TURN_SENSOR_MIN_MS = 760;
static const unsigned long TURN_FAST_MS = 460;
static const unsigned long TURN_SOFT_MAX_MS = 1180;
static const unsigned long TURN_MAX_MS = 1850;
static const unsigned long TURN_FAILSAFE_BACKOFF_MS = 180;
static const unsigned long LIDAR_RECOVER_PERIOD_MS = 1500;

static const int CORNER_CONFIRM_SCANS = 2;
static const int TURN_DECISION_CONFIRM_SCANS = 2;
static const int READY_CONFIRM_SCANS = 2;
static const int LOST_TRACK_CONFIRM_SCANS = 8;

// Number of same-direction turns per half-lap before flipping direction.
static const int TURNS_PER_HALF_LAP = 4;

enum TrackState {
  STATE_ACQUIRE_STRAIGHT = 1,
  STATE_STRAIGHT = 2,
  STATE_CORNER_DECIDE = 3,
  STATE_TURN_RIGHT = 4,
  STATE_TURN_LEFT = 5,
  STATE_ALIGN_AFTER_TURN = 6,
  STATE_STOPPED = 7
};

enum TurnDirection {
  TURN_UNKNOWN = 0,
  TURN_RIGHT = 1,
  TURN_LEFT = -1
};

// First half-lap direction. The figure-8 route always starts with right turns
// out of the crossbar, so we pre-seed the locked direction here instead of
// relying on CORNER_DECIDE at the first T-junction (which can stall when both
// sides look partially open in the crossbar geometry).
static const TurnDirection INITIAL_TURN_DIRECTION = TURN_RIGHT;

struct RangeStat {
  int count;
  float near;
  float mean;
};

struct ScanFeatures {
  bool ok;
  RangeStat front;
  RangeStat frontWide;
  RangeStat rear;
  RangeStat left;
  RangeStat right;
  RangeStat leftWide;
  RangeStat rightWide;
  RangeStat leftFront;
  RangeStat rightFront;
  RangeStat leftRear;
  RangeStat rightRear;
};

static float scanDistance[360];
static int scanPointCount = 0;
static unsigned long scanIndex = 0;

static TrackState state = STATE_ACQUIRE_STRAIGHT;
static TurnDirection lockedTurnDirection = TURN_UNKNOWN;
static TurnDirection currentTurnDirection = TURN_UNKNOWN;

// completedTurns: turns completed within the current half-lap (resets every TURNS_PER_HALF_LAP)
static int completedTurns = 0;
// halfLaps: total half-laps completed (each half-lap = TURNS_PER_HALF_LAP turns)
static int halfLaps = 0;
static int cornerConfirm = 0;
static int readyConfirm = 0;
static int decisionConfirm = 0;
static int lostTrackConfirm = 0;

static int lastForwardCommand = 0;
static int lastStrafeCommand = 0;
static int lastRotateCommand = 0;

static unsigned long stateStartMs = 0;
static unsigned long lastMotionUpdateMs = 0;
static unsigned long lastDebugMs = 0;
static unsigned long lastRecoverMs = 0;
static unsigned long lastTurnBackoffMs = 0;

// Explicit prototypes keep Arduino's .ino preprocessor from guessing wrong.
static int clampInt(int value, int lo, int hi);
static float clampFloat(float value, float lo, float hi);
static bool validDistance(float distance);
static bool usable(const RangeStat &stat);
static int angularDistance(int angle, int center);
static bool inSector(int angle, int center, int halfWidth);
static void clearScanBuffer();
static RangeStat makeEmptyStat();
static void insertSmallest(float value, float smallest[], int size);
static RangeStat sectorStat(int centerDeg, int halfWidthDeg);
static ScanFeatures buildFeatures();
static bool statCloserThan(const RangeStat &stat, float distance);
static bool statFartherThan(const RangeStat &stat, float distance);
static bool frontIsClear(const ScanFeatures &features);
static bool frontIsCloseForCorner(const ScanFeatures &features);
static bool frontIsDangerouslyClose(const ScanFeatures &features);
static bool sideWallsUsable(const ScanFeatures &features);
static bool trackLooksPresent(const ScanFeatures &features);
static float sideCenterError(const ScanFeatures &features);
static float yawError(const ScanFeatures &features);
static int sideCorrectionPwm(const ScanFeatures &features);
static int yawCorrectionPwm(const ScanFeatures &features, int limitPwm);
static bool yawDataUsable(const ScanFeatures &features);
static bool poseLooksAligned(const ScanFeatures &features);
static float opennessScore(const RangeStat &stat, float openDistance);
static float leftCornerOpenScore(const ScanFeatures &features);
static float rightCornerOpenScore(const ScanFeatures &features);
static TurnDirection detectedCornerDirection(const ScanFeatures &features);
static TurnDirection fallbackCornerDirection(const ScanFeatures &features);
static bool turnExitReady(const ScanFeatures &features, unsigned long elapsed);
static int insideClearanceStrafe(const ScanFeatures &features, TurnDirection turnDirection);
static void driveRaw(int lf, int rf, int lr, int rr);
static void driveVector(int forward, int strafeRight, int rotateRight);
static const char *stateName(TrackState value);
static const char *turnName(TurnDirection value);
static void enterState(TrackState nextState);
static void onHalfLapComplete(const ScanFeatures &features);
static void printDebug(const ScanFeatures &features);
static void updateAcquireStraight(const ScanFeatures &features);
static void updateStraight(const ScanFeatures &features);
static void updateCornerDecide(const ScanFeatures &features);
static void updateQuarterTurn(const ScanFeatures &features, TurnDirection turnDirection);
static void updateAlignAfterTurn(const ScanFeatures &features);
static void updateTrackState(const ScanFeatures &features);
static void storeLidarPoint(float distanceM, float angleDeg);
static bool requestLidarScan();
static void recoverLidarIfNeeded();

static int clampInt(int value, int lo, int hi) {
  if (value < lo) return lo;
  if (value > hi) return hi;
  return value;
}

static float clampFloat(float value, float lo, float hi) {
  if (value < lo) return lo;
  if (value > hi) return hi;
  return value;
}

static bool validDistance(float distance) {
  return distance >= LIDAR_MIN_VALID_M && distance <= LIDAR_MAX_VALID_M;
}

static bool usable(const RangeStat &stat) {
  return stat.count >= MIN_SECTOR_POINTS && stat.near < INVALID_RANGE_M;
}

static int angularDistance(int angle, int center) {
  int diff = abs(angle - center);
  return diff > 180 ? 360 - diff : diff;
}

static bool inSector(int angle, int center, int halfWidth) {
  return angularDistance(angle, center) <= halfWidth;
}

static void clearScanBuffer() {
  for (int i = 0; i < 360; i++) {
    scanDistance[i] = 0.0f;
  }
  scanPointCount = 0;
}

static RangeStat makeEmptyStat() {
  RangeStat stat;
  stat.count = 0;
  stat.near = INVALID_RANGE_M;
  stat.mean = INVALID_RANGE_M;
  return stat;
}

static void insertSmallest(float value, float smallest[], int size) {
  for (int i = 0; i < size; i++) {
    if (value < smallest[i]) {
      for (int j = size - 1; j > i; j--) {
        smallest[j] = smallest[j - 1];
      }
      smallest[i] = value;
      return;
    }
  }
}

static RangeStat sectorStat(int centerDeg, int halfWidthDeg) {
  float smallest[ROBUST_SAMPLES];
  float sum = 0.0f;
  RangeStat stat = makeEmptyStat();

  for (int i = 0; i < ROBUST_SAMPLES; i++) {
    smallest[i] = INVALID_RANGE_M;
  }

  for (int angle = 0; angle < 360; angle++) {
    float distance = scanDistance[angle];
    if (inSector(angle, centerDeg, halfWidthDeg) && validDistance(distance)) {
      stat.count++;
      sum += distance;
      insertSmallest(distance, smallest, ROBUST_SAMPLES);
    }
  }

  if (stat.count > 0) {
    int used = stat.count < ROBUST_SAMPLES ? stat.count : ROBUST_SAMPLES;
    float nearSum = 0.0f;
    for (int i = 0; i < used; i++) {
      nearSum += smallest[i];
    }
    stat.near = nearSum / used;
    stat.mean = sum / stat.count;
  }

  return stat;
}

static ScanFeatures buildFeatures() {
  ScanFeatures features;
  features.ok = scanPointCount >= MIN_SCAN_POINTS;
  features.front = sectorStat(FRONT_DEG, 12);
  features.frontWide = sectorStat(FRONT_DEG, 31);
  features.rear = sectorStat(REAR_DEG, 16);
  features.left = sectorStat(LEFT_DEG, 15);
  features.right = sectorStat(RIGHT_DEG, 15);
  features.leftWide = sectorStat(LEFT_DEG, 32);
  features.rightWide = sectorStat(RIGHT_DEG, 32);
  features.leftFront = sectorStat(LEFT_FRONT_DEG, 18);
  features.rightFront = sectorStat(RIGHT_FRONT_DEG, 18);
  features.leftRear = sectorStat(LEFT_REAR_DEG, 18);
  features.rightRear = sectorStat(RIGHT_REAR_DEG, 18);
  return features;
}

static bool statCloserThan(const RangeStat &stat, float distance) {
  return usable(stat) && stat.near < distance;
}

static bool statFartherThan(const RangeStat &stat, float distance) {
  return !usable(stat) || stat.near > distance;
}

static bool frontIsClear(const ScanFeatures &features) {
  return statFartherThan(features.front, FRONT_CLEAR_AFTER_TURN_M) &&
         statFartherThan(features.frontWide, FRONT_CLEAR_AFTER_TURN_M - 0.06f);
}

static bool frontIsCloseForCorner(const ScanFeatures &features) {
  return statCloserThan(features.frontWide, FRONT_TURN_TRIGGER_M);
}

static bool frontIsDangerouslyClose(const ScanFeatures &features) {
  return statCloserThan(features.front, FRONT_HARD_BRAKE_M);
}

static bool sideWallsUsable(const ScanFeatures &features) {
  bool leftOk = usable(features.left) && features.left.near < SIDE_VALID_MAX_M;
  bool rightOk = usable(features.right) && features.right.near < SIDE_VALID_MAX_M;
  return leftOk || rightOk;
}

static bool trackLooksPresent(const ScanFeatures &features) {
  return (usable(features.left) && features.left.near < TRACK_PRESENT_MAX_M) ||
         (usable(features.right) && features.right.near < TRACK_PRESENT_MAX_M) ||
         (usable(features.frontWide) && features.frontWide.near < TRACK_PRESENT_MAX_M) ||
         (usable(features.rear) && features.rear.near < TRACK_PRESENT_MAX_M);
}

static float sideCenterError(const ScanFeatures &features) {
  bool leftOk = usable(features.left) && features.left.near < SIDE_VALID_MAX_M;
  bool rightOk = usable(features.right) && features.right.near < SIDE_VALID_MAX_M;

  if (leftOk && rightOk) {
    return features.right.near - features.left.near;
  }

  if (rightOk) {
    return features.right.near - SIDE_TARGET_M;
  }

  if (leftOk) {
    return SIDE_TARGET_M - features.left.near;
  }

  return 0.0f;
}

static float yawError(const ScanFeatures &features) {
  float error = 0.0f;
  int weight = 0;

  if (usable(features.rightFront) && usable(features.rightRear)) {
    error += features.rightFront.near - features.rightRear.near;
    weight++;
  }

  if (usable(features.leftFront) && usable(features.leftRear)) {
    error += features.leftRear.near - features.leftFront.near;
    weight++;
  }

  if (weight == 0) {
    return 0.0f;
  }

  return error / weight;
}

static int sideCorrectionPwm(const ScanFeatures &features) {
  float correction = STRAFE_KP * sideCenterError(features);

  if (statCloserThan(features.right, SIDE_TOO_CLOSE_M)) {
    correction -= 18.0f;
  }
  if (statCloserThan(features.left, SIDE_TOO_CLOSE_M)) {
    correction += 18.0f;
  }
  if (statCloserThan(features.right, SIDE_EMERGENCY_M)) {
    correction -= 14.0f;
  }
  if (statCloserThan(features.left, SIDE_EMERGENCY_M)) {
    correction += 14.0f;
  }

  return (int)clampFloat(correction, -STRAFE_LIMIT_PWM, STRAFE_LIMIT_PWM);
}

static int yawCorrectionPwm(const ScanFeatures &features, int limitPwm) {
  float correction = YAW_KP * yawError(features);
  return (int)clampFloat(correction, -limitPwm, limitPwm);
}

static bool yawDataUsable(const ScanFeatures &features) {
  return (usable(features.rightFront) && usable(features.rightRear)) ||
         (usable(features.leftFront) && usable(features.leftRear));
}

static bool poseLooksAligned(const ScanFeatures &features) {
  return sideWallsUsable(features) &&
         frontIsClear(features) &&
         yawDataUsable(features) &&
         fabs(sideCenterError(features)) < CENTER_OK_M &&
         fabs(yawError(features)) < YAW_OK_M;
}

static float opennessScore(const RangeStat &stat, float openDistance) {
  if (!usable(stat)) {
    return 0.25f;
  }
  if (stat.near > openDistance + 0.22f) {
    return 1.00f;
  }
  if (stat.near > openDistance + 0.10f) {
    return 0.75f;
  }
  if (stat.near > openDistance) {
    return 0.45f;
  }
  if (stat.near < SIDE_TOO_CLOSE_M) {
    return -0.65f;
  }
  if (stat.near < SIDE_TARGET_M + 0.08f) {
    return -0.25f;
  }
  return 0.0f;
}

static float leftCornerOpenScore(const ScanFeatures &features) {
  return
    1.15f * opennessScore(features.leftFront, DIAGONAL_OPEN_M) +
    0.60f * opennessScore(features.leftWide, SIDE_OPEN_M) +
    0.25f * opennessScore(features.left, SIDE_OPEN_M);
}

static float rightCornerOpenScore(const ScanFeatures &features) {
  return
    1.15f * opennessScore(features.rightFront, DIAGONAL_OPEN_M) +
    0.60f * opennessScore(features.rightWide, SIDE_OPEN_M) +
    0.25f * opennessScore(features.right, SIDE_OPEN_M);
}

static TurnDirection detectedCornerDirection(const ScanFeatures &features) {
  float leftScore = leftCornerOpenScore(features);
  float rightScore = rightCornerOpenScore(features);

  const float margin = 0.45f;
  if (rightScore - leftScore > margin) {
    return TURN_RIGHT;
  }
  if (leftScore - rightScore > margin) {
    return TURN_LEFT;
  }
  return TURN_UNKNOWN;
}

static TurnDirection fallbackCornerDirection(const ScanFeatures &features) {
  float leftScore = leftCornerOpenScore(features);
  float rightScore = rightCornerOpenScore(features);

  if (rightScore > leftScore + 0.05f) {
    return TURN_RIGHT;
  }
  if (leftScore > rightScore + 0.05f) {
    return TURN_LEFT;
  }

  return TURN_UNKNOWN;
}

static bool turnExitReady(const ScanFeatures &features, unsigned long elapsed) {
  if (elapsed < TURN_SENSOR_MIN_MS || !frontIsClear(features) || !sideWallsUsable(features)) {
    return false;
  }

  if (!yawDataUsable(features)) {
    return elapsed >= TURN_SOFT_MAX_MS;
  }

  if (fabs(yawError(features)) < TURN_EXIT_YAW_OK_M) {
    return true;
  }

  return elapsed >= TURN_SOFT_MAX_MS && fabs(yawError(features)) < TURN_EXIT_YAW_OK_M * 1.6f;
}

static int insideClearanceStrafe(const ScanFeatures &features, TurnDirection turnDirection) {
  int strafe = sideCorrectionPwm(features) / 2;

  if (turnDirection == TURN_RIGHT) {
    if (statCloserThan(features.right, SIDE_TOO_CLOSE_M) ||
        statCloserThan(features.rightFront, SIDE_TARGET_M + 0.02f)) {
      strafe -= 16;
    }
  } else if (turnDirection == TURN_LEFT) {
    if (statCloserThan(features.left, SIDE_TOO_CLOSE_M) ||
        statCloserThan(features.leftFront, SIDE_TARGET_M + 0.02f)) {
      strafe += 16;
    }
  }

  return clampInt(strafe, -TURN_STRAFE_LIMIT_PWM, TURN_STRAFE_LIMIT_PWM);
}

static void driveRaw(int lf, int rf, int lr, int rr) {
  mecanum.driveAllMotor(
    clampInt(lf, -100, 100),
    clampInt(rf, -100, 100),
    clampInt(lr, -100, 100),
    clampInt(rr, -100, 100)
  );
}

static void driveVector(int forward, int strafeRight, int rotateRight) {
  lastForwardCommand = forward;
  lastStrafeCommand = strafeRight;
  lastRotateCommand = rotateRight;

  int lf = forward + strafeRight + rotateRight;
  int rf = forward - strafeRight - rotateRight;
  int lr = forward - strafeRight + rotateRight;
  int rr = forward + strafeRight - rotateRight;

  int maxAbs = max(max(abs(lf), abs(rf)), max(abs(lr), abs(rr)));
  if (maxAbs > 100) {
    lf = (lf * 100) / maxAbs;
    rf = (rf * 100) / maxAbs;
    lr = (lr * 100) / maxAbs;
    rr = (rr * 100) / maxAbs;
  }

  driveRaw(lf, rf, lr, rr);
}

static const char *stateName(TrackState value) {
  switch (value) {
    case STATE_ACQUIRE_STRAIGHT: return "ACQUIRE_STRAIGHT";
    case STATE_STRAIGHT:         return "STRAIGHT";
    case STATE_CORNER_DECIDE:    return "CORNER_DECIDE";
    case STATE_TURN_RIGHT:       return "TURN_RIGHT";
    case STATE_TURN_LEFT:        return "TURN_LEFT";
    case STATE_ALIGN_AFTER_TURN: return "ALIGN_AFTER_TURN";
    case STATE_STOPPED:          return "STOPPED";
    default:                     return "UNKNOWN";
  }
}

static const char *turnName(TurnDirection value) {
  switch (value) {
    case TURN_RIGHT: return "RIGHT";
    case TURN_LEFT:  return "LEFT";
    default:         return "UNKNOWN";
  }
}

static void enterState(TrackState nextState) {
  state = nextState;
  stateStartMs = millis();
  lastMotionUpdateMs = stateStartMs;
  cornerConfirm = 0;
  readyConfirm = 0;
  decisionConfirm = 0;
  lastTurnBackoffMs = 0;

  Serial.printf("STATE -> %s | turn=%s | half=%d | turns_in_half=%d\r\n",
                stateName(state),
                turnName(lockedTurnDirection),
                halfLaps,
                completedTurns);

  if (state == STATE_STOPPED) {
    driveRaw(0, 0, 0, 0);
  }
}

// Called when the 4th turn of a half-lap finishes. Flips direction for the next half.
static void onHalfLapComplete(const ScanFeatures &features) {
  halfLaps++;
  completedTurns = 0;

  // Flip the locked direction for the next half-lap.
  lockedTurnDirection = (lockedTurnDirection == TURN_RIGHT) ? TURN_LEFT : TURN_RIGHT;

  Serial.printf("HALF_LAP_DONE total_halves=%d next_turn=%s\r\n",
                halfLaps,
                turnName(lockedTurnDirection));

  bool skipAlign = frontIsClear(features) &&
                   sideWallsUsable(features) &&
                   yawDataUsable(features) &&
                   fabs(yawError(features)) < ALIGN_YAW_OK_M;
  enterState(skipAlign ? STATE_STRAIGHT : STATE_ALIGN_AFTER_TURN);
}

static void printDebug(const ScanFeatures &features) {
  unsigned long now = millis();
  if (now - lastDebugMs < DEBUG_PERIOD_MS) {
    return;
  }
  lastDebugMs = now;

  unsigned long elapsed = now - stateStartMs;

  Serial.printf(
    "#%lu %s ms=%lu turn=%s half=%d n=%d ready=%d F=%.2f FW=%.2f L=%.2f R=%.2f LF=%.2f RF=%.2f CE=%.2f YE=%.2f cmd=%d,%d,%d\r\n",
    scanIndex,
    stateName(state),
    elapsed,
    turnName(lockedTurnDirection),
    halfLaps,
    completedTurns,
    readyConfirm,
    features.front.near,
    features.frontWide.near,
    features.left.near,
    features.right.near,
    features.leftFront.near,
    features.rightFront.near,
    sideCenterError(features),
    yawError(features),
    lastForwardCommand,
    lastStrafeCommand,
    lastRotateCommand
  );
}

static void updateAcquireStraight(const ScanFeatures &features) {
  unsigned long elapsed = millis() - stateStartMs;

  if (elapsed >= ACQUIRE_MIN_MS && poseLooksAligned(features)) {
    readyConfirm++;
  } else if (readyConfirm > 0) {
    readyConfirm--;
  }

  if (readyConfirm >= READY_CONFIRM_SCANS ||
      (elapsed >= ACQUIRE_MAX_MS && frontIsClear(features) && sideWallsUsable(features))) {
    enterState(STATE_STRAIGHT);
    return;
  }

  if (!sideWallsUsable(features)) {
    driveRaw(0, 0, 0, 0);
    return;
  }

  int strafe = sideCorrectionPwm(features);
  int rotate = yawCorrectionPwm(features, YAW_LIMIT_ALIGN_PWM);
  int forward = frontIsClear(features) ? ALIGN_FORWARD_PWM : -8;

  if (frontIsCloseForCorner(features)) {
    TurnDirection direction = detectedCornerDirection(features);
    if (direction != TURN_UNKNOWN) {
      rotate = (direction == TURN_RIGHT) ? ALIGN_ROTATE_PWM : -ALIGN_ROTATE_PWM;
    }
    forward = frontIsDangerouslyClose(features) ? -CORNER_BACKOFF_PWM : 0;
  }

  driveVector(forward, strafe, rotate);
}

static void updateStraight(const ScanFeatures &features) {
  unsigned long elapsed = millis() - stateStartMs;

  if (elapsed > MIN_STRAIGHT_BEFORE_CORNER_MS && frontIsCloseForCorner(features)) {
    cornerConfirm++;
  } else if (cornerConfirm > 0) {
    cornerConfirm--;
  }

  if (cornerConfirm >= CORNER_CONFIRM_SCANS) {
    if (lockedTurnDirection == TURN_RIGHT) {
      enterState(STATE_TURN_RIGHT);
      return;
    }
    if (lockedTurnDirection == TURN_LEFT) {
      enterState(STATE_TURN_LEFT);
      return;
    }
    enterState(STATE_CORNER_DECIDE);
    return;
  }

  int forward = STRAIGHT_FAST_PWM;
  if (statCloserThan(features.frontWide, FRONT_SLOW_M)) {
    float ratio = (features.frontWide.near - FRONT_TURN_TRIGGER_M) / (FRONT_SLOW_M - FRONT_TURN_TRIGGER_M);
    ratio = clampFloat(ratio, 0.0f, 1.0f);
    forward = STRAIGHT_SLOW_PWM + (int)((STRAIGHT_FAST_PWM - STRAIGHT_SLOW_PWM) * ratio);
  }

  if (frontIsDangerouslyClose(features)) {
    forward = -CORNER_BACKOFF_PWM;
  }

  int strafe = sideCorrectionPwm(features);
  int rotate = yawCorrectionPwm(features, YAW_LIMIT_STRAIGHT_PWM);

  if (lockedTurnDirection != TURN_UNKNOWN && statCloserThan(features.frontWide, FRONT_SLOW_M)) {
    int preTurn = (lockedTurnDirection == TURN_RIGHT) ? PRE_TURN_ROTATE_PWM : -PRE_TURN_ROTATE_PWM;
    rotate = clampInt(rotate + preTurn, -YAW_LIMIT_ALIGN_PWM, YAW_LIMIT_ALIGN_PWM);
  }

  if (fabs(yawError(features)) > 0.14f) {
    forward = min(forward, STRAIGHT_SLOW_PWM);
  }

  driveVector(forward, strafe, rotate);
}

static void updateCornerDecide(const ScanFeatures &features) {
  unsigned long elapsed = millis() - stateStartMs;
  TurnDirection detected = detectedCornerDirection(features);

  if (lockedTurnDirection == TURN_UNKNOWN) {
    if (detected != TURN_UNKNOWN) {
      if (currentTurnDirection == detected) {
        decisionConfirm++;
      } else {
        currentTurnDirection = detected;
        decisionConfirm = 1;
      }

      if (decisionConfirm >= TURN_DECISION_CONFIRM_SCANS) {
        lockedTurnDirection = detected;
      }
    }
  } else {
    currentTurnDirection = lockedTurnDirection;
    decisionConfirm = TURN_DECISION_CONFIRM_SCANS;
  }

  if (currentTurnDirection != TURN_UNKNOWN &&
      decisionConfirm >= TURN_DECISION_CONFIRM_SCANS) {
    enterState(currentTurnDirection == TURN_RIGHT ? STATE_TURN_RIGHT : STATE_TURN_LEFT);
    return;
  }

  if (elapsed > CORNER_DECIDE_MAX_MS) {
    TurnDirection fallback = detected != TURN_UNKNOWN ? detected : fallbackCornerDirection(features);
    if (fallback != TURN_UNKNOWN) {
      lockedTurnDirection = fallback;
      currentTurnDirection = fallback;
      enterState(currentTurnDirection == TURN_RIGHT ? STATE_TURN_RIGHT : STATE_TURN_LEFT);
      return;
    }
  }

  if (elapsed > CORNER_DECIDE_MAX_MS + 450 && currentTurnDirection != TURN_UNKNOWN) {
    lockedTurnDirection = currentTurnDirection;
    enterState(currentTurnDirection == TURN_RIGHT ? STATE_TURN_RIGHT : STATE_TURN_LEFT);
    return;
  }

  int strafe = sideCorrectionPwm(features);
  int forward = frontIsDangerouslyClose(features) ? -CORNER_BACKOFF_PWM : CORNER_APPROACH_FORWARD_PWM;
  int rotate = 0;
  if (detected != TURN_UNKNOWN) {
    rotate = (detected == TURN_RIGHT) ? PRE_TURN_ROTATE_PWM : -PRE_TURN_ROTATE_PWM;
  }
  driveVector(forward, strafe, rotate);
}

static void updateQuarterTurn(const ScanFeatures &features, TurnDirection turnDirection) {
  unsigned long elapsed = millis() - stateStartMs;

  if (turnExitReady(features, elapsed)) {
    readyConfirm++;
  } else if (readyConfirm > 0) {
    readyConfirm--;
  }

  if (readyConfirm >= READY_CONFIRM_SCANS) {
    completedTurns++;

    if (completedTurns >= TURNS_PER_HALF_LAP) {
      // Half-lap done: flip direction and continue.
      onHalfLapComplete(features);
    } else {
      bool skipAlign = frontIsClear(features) &&
                       sideWallsUsable(features) &&
                       yawDataUsable(features) &&
                       fabs(yawError(features)) < ALIGN_YAW_OK_M;
      enterState(skipAlign ? STATE_STRAIGHT : STATE_ALIGN_AFTER_TURN);
    }
    return;
  }

  int rotateMagnitude = TURN_ROTATE_FAST_PWM;
  if (elapsed > TURN_FAST_MS) {
    rotateMagnitude = TURN_ROTATE_SLOW_PWM;
  }
  if (frontIsClear(features) && elapsed > TURN_OPEN_LOOP_MIN_MS) {
    rotateMagnitude = TURN_ROTATE_FINE_PWM;
  }
  if (elapsed >= TURN_OPEN_LOOP_MIN_MS &&
      yawDataUsable(features) &&
      fabs(yawError(features)) < TURN_EXIT_YAW_OK_M * 2.0f) {
    rotateMagnitude = TURN_ROTATE_BRAKE_PWM;
  }
  // Brake by time when yaw data is unavailable (e.g. at a T-junction where
  // one side is open and diagonal sensors lose the wall).
  if (!yawDataUsable(features) && elapsed >= TURN_SENSOR_MIN_MS) {
    rotateMagnitude = TURN_ROTATE_BRAKE_PWM;
  }
  if (elapsed > TURN_MAX_MS) {
    rotateMagnitude = TURN_ROTATE_SLOW_PWM;
  }

  int rotate = (turnDirection == TURN_RIGHT) ? rotateMagnitude : -rotateMagnitude;
  int strafe = insideClearanceStrafe(features, turnDirection);
  int forward = 0;

  if (frontIsDangerouslyClose(features)) {
    forward = -CORNER_BACKOFF_PWM;
  } else if (elapsed > TURN_MAX_MS &&
             usable(features.frontWide) &&
             features.frontWide.near < FRONT_TURN_TRIGGER_M + 0.05f) {
    unsigned long now = millis();
    if (lastTurnBackoffMs == 0 || now - lastTurnBackoffMs > 700) {
      lastTurnBackoffMs = now;
    }
    if (now - lastTurnBackoffMs < TURN_FAILSAFE_BACKOFF_MS) {
      forward = -CORNER_BACKOFF_PWM;
    }
  } else if (frontIsClear(features) && elapsed > TURN_SENSOR_MIN_MS) {
    forward = TURN_EXIT_FORWARD_PWM;
  } else if (!usable(features.front) ||
             features.front.near > FRONT_HARD_BRAKE_M + 0.08f) {
    forward = TURN_ARC_FORWARD_PWM;
  }

  driveVector(forward, strafe, rotate);
}

static void updateAlignAfterTurn(const ScanFeatures &features) {
  unsigned long elapsed = millis() - stateStartMs;
  bool ready = sideWallsUsable(features) &&
               frontIsClear(features) &&
               yawDataUsable(features) &&
               fabs(sideCenterError(features)) < CENTER_OK_M &&
               fabs(yawError(features)) < ALIGN_YAW_OK_M;

  if (elapsed >= ALIGN_MIN_MS && ready) {
    readyConfirm++;
  } else if (readyConfirm > 0) {
    readyConfirm--;
  }

  if (readyConfirm >= READY_CONFIRM_SCANS ||
      (elapsed >= ALIGN_MAX_MS && frontIsClear(features) && sideWallsUsable(features))) {
    enterState(STATE_STRAIGHT);
    return;
  }

  int strafe = sideCorrectionPwm(features);
  int rotate = yawCorrectionPwm(features, YAW_LIMIT_ALIGN_PWM);
  int forward = frontIsClear(features) ? ALIGN_FORWARD_PWM : -8;

  if (!frontIsClear(features) && lockedTurnDirection != TURN_UNKNOWN) {
    rotate = (lockedTurnDirection == TURN_RIGHT) ? ALIGN_ROTATE_PWM : -ALIGN_ROTATE_PWM;
    if (frontIsDangerouslyClose(features)) {
      forward = -CORNER_BACKOFF_PWM;
    }
  }

  driveVector(forward, strafe, rotate);
}

static void updateTrackState(const ScanFeatures &features) {
  if (!features.ok) {
    driveRaw(0, 0, 0, 0);
    return;
  }

  if (trackLooksPresent(features)) {
    lostTrackConfirm = 0;
  } else {
    lostTrackConfirm++;
    driveRaw(0, 0, 0, 0);
    if (lostTrackConfirm >= LOST_TRACK_CONFIRM_SCANS) {
      enterState(STATE_STOPPED);
    }
    return;
  }

  printDebug(features);

  switch (state) {
    case STATE_ACQUIRE_STRAIGHT:
      updateAcquireStraight(features);
      break;
    case STATE_STRAIGHT:
      updateStraight(features);
      break;
    case STATE_CORNER_DECIDE:
      updateCornerDecide(features);
      break;
    case STATE_TURN_RIGHT:
      updateQuarterTurn(features, TURN_RIGHT);
      break;
    case STATE_TURN_LEFT:
      updateQuarterTurn(features, TURN_LEFT);
      break;
    case STATE_ALIGN_AFTER_TURN:
      updateAlignAfterTurn(features);
      break;
    case STATE_STOPPED:
    default:
      driveRaw(0, 0, 0, 0);
      break;
  }
}

static void storeLidarPoint(float distanceM, float angleDeg) {
  int angle = (int)(angleDeg + 0.5f);
  if (angle >= 360) {
    angle -= 360;
  }
  if (angle < 0 || angle >= 360) {
    return;
  }

  if (!validDistance(distanceM)) {
    return;
  }

  if (scanDistance[angle] == 0.0f || distanceM < scanDistance[angle]) {
    scanDistance[angle] = distanceM;
  }
  scanPointCount++;
}

static bool requestLidarScan() {
  rplidar_response_device_info_t info;
  if (IS_OK(lidar.getDeviceInfo(info, 100))) {
    lidar.startScan();
    delay(1000);
    return true;
  }
  return false;
}

static void recoverLidarIfNeeded() {
  unsigned long now = millis();
  if (now - lastRecoverMs < LIDAR_RECOVER_PERIOD_MS) {
    return;
  }
  lastRecoverMs = now;

  driveRaw(0, 0, 0, 0);
  requestLidarScan();
}

void setup() {
  Serial.begin(115200);
  lidar.begin(Serial2);
  mecanum.begin();

  clearScanBuffer();
  stateStartMs = millis();
  lastMotionUpdateMs = stateStartMs;
  lastDebugMs = millis();
  lastRecoverMs = millis() - LIDAR_RECOVER_PERIOD_MS;

  Serial.println("Figure-8 track state machine ready");
  Serial.printf("track_width=%.2f car_width=%.2f side_target=%.2f front_offset=%.2f\r\n",
                TRACK_WIDTH_M,
                CAR_WIDTH_M,
                SIDE_TARGET_M,
                LIDAR_TO_FRONT_BUMPER_M);
  lockedTurnDirection = INITIAL_TURN_DIRECTION;
  requestLidarScan();
  enterState(STATE_ACQUIRE_STRAIGHT);
}

void loop() {
  if (IS_OK(lidar.waitPoint())) {
    float distance = lidar.getCurrentPoint().distance / 1000.0f;
    float angle = lidar.getCurrentPoint().angle;
    bool startBit = lidar.getCurrentPoint().startBit;

    if (startBit) {
      ScanFeatures features = buildFeatures();
      scanIndex++;
      updateTrackState(features);
      clearScanBuffer();
    }

    storeLidarPoint(distance, angle);
  } else {
    recoverLidarIfNeeded();
  }
}
