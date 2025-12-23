

//================================================================================
// ADVANCED LINE FOLLOWING ROBOT WITH JUNCTION DETECTION
// Features: PID Line Following, Left-Hand Rule, Smart Junction Detection
//================================================================================

//--------------------------------------------------------------------------------
// 1. PIN DEFINITIONS AND CONSTANTS
//--------------------------------------------------------------------------------

// Motor Driver Pins (L298N)
const int ENA = 10;  // Enable A (Left Motor Speed) - PWM
const int IN1 = 48;  // Input 1 (Left Motor Direction)
const int IN2 = 46;  // Input 2 (Left Motor Direction)
const int ENB = 9;   // Enable B (Right Motor Speed) - PWM
const int IN3 = 44;  // Input 3 (Right Motor Direction)
const int IN4 = 42;  // Input 4 (Right Motor Direction)

// Line Hunting IR Pins
const int HUNT_LEFT = 25;   // Left corner hunting IR
const int HUNT_MIDDLE = 27; // Middle hunting IR (after the 6 array)
const int HUNT_RIGHT = 29;  // Right corner hunting IR

// IR Sensor Pins (6-sensor array)
const int S0 = A0; // Rightmost sensor
const int S1 = A1;
const int S2 = A2; // Middle-right (used for junction detection)
const int S3 = A3; // Middle-left (used for junction detection)
const int S4 = A4;
const int S5 = A5; // Leftmost sensor

// PID Tuning Constants
const float Kp = 70;   // Proportional gain (50)
const float Ki = 0.05;  // Integral gain (0.5)
const float Kd = 0;  // Derivative gain (30)

// Robot Behavior Constants
const int BASE_SPEED = 100;           // Default speed
const int MAX_SPEED = 180;            // Maximum speed
const int MAX_ACCEL = 10;             // Max acceleration per loop
const int SENSOR_THRESHOLD = 400;     // Threshold for black/white detection
const int TURN_SPEED = 140;           // Speed during turns
const int FORWARD_CHECK_TIME = 200;   // Time to move forward for junction classification (ms)
const int Y_FORWARD_TIME = 200;       // Time to move forward at Y junction before turning (ms)
const int TURN_90_DURATION = 600;     // Time for 90-degree turn (ms)
const int TURN_180_DURATION = 1200;   // Time for 180-degree turn (ms)
const int JUNCTION_DELAY = 100;       // Delay after detecting junction before action
const int INITIAL_BLACK_TIME = 500;   // Time to move forward on initial black square (ms)

//--------------------------------------------------------------------------------
// 2. GLOBAL VARIABLES AND TYPE DEFINITIONS
//--------------------------------------------------------------------------------

struct SensorData {
  bool s0, s1, s2, s3, s4, s5;        // Main sensor array
  bool huntLeft, huntMiddle, huntRight; // Hunting sensors
  int activeCount;                     // Number of active main sensors
};

// Junction Types
enum JunctionType {
  NO_JUNCTION,
  Y_JUNCTION,
  T_BOTH,           // T junction with paths on both sides
  T_LEFT,           // T junction with path on left only
  T_RIGHT,          // T junction with path on right only
  LEFT_90,          // L-type left turn
  RIGHT_90,         // L-type right turn
  FOUR_WAY,         // 4-way junction
  DEAD_END
};

// Robot State
enum RobotState { 
  INITIAL_BLACK_SQUARE, // New state for initial black square
  FOLLOWING_LINE,
  CLASSIFYING_JUNCTION,
  Y_FORWARD_MOVE,   // New state for Y junction forward movement
  TURNING_LEFT, 
  TURNING_RIGHT,
  MOVING_STRAIGHT,
  TURNING_AROUND,
  JUNCTION_APPROACH_SPEED
};

// PID variables
float error = 0;
float lastError = 0;
float integral = 0;

// Robot state
RobotState currentState = INITIAL_BLACK_SQUARE; // Start with initial black square check
JunctionType lastJunctionType = NO_JUNCTION;
unsigned long stateStartTime = 0;
bool fromDeadEnd = false; // Track if returning from dead end
bool initialBlackHandled = false; // Track if initial black square was handled

// Junction history for left-hand rule
struct JunctionMemory {
  unsigned long timestamp;
  JunctionType type;
  int action; // 0=left, 1=straight, 2=right
};
JunctionMemory lastVisitedJunction = {0, NO_JUNCTION, -1};

//--------------------------------------------------------------------------------
// 3. FUNCTION PROTOTYPES
//--------------------------------------------------------------------------------
SensorData readSensors();
JunctionType detectAndClassifyJunction(SensorData sensors);
void handleJunctionLeftHandRule(JunctionType junctionType);
float calculateError(SensorData sensors);
float calculatePID(float currentError);
void controlMotors(float correction);
void turnLeft();
void turnRight();
void turnAround();
void moveForward(int speed);
void stopMotors();
void setMotor(int in1, int in2, int pwm, int speed);

//--------------------------------------------------------------------------------
// 4. SETUP FUNCTION
//--------------------------------------------------------------------------------
void setup() {
  Serial.begin(9600);
  Serial.println("Advanced Line Following Robot Initializing...");

  // Motor control pins
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Main sensor array
  pinMode(S0, INPUT);
  pinMode(S1, INPUT);
  pinMode(S2, INPUT);
  pinMode(S3, INPUT);
  pinMode(S4, INPUT);
  pinMode(S5, INPUT);

  // Hunting sensors
  pinMode(HUNT_LEFT, INPUT);
  pinMode(HUNT_MIDDLE, INPUT);
  pinMode(HUNT_RIGHT, INPUT);

  Serial.println("System Ready!");
  delay(2000);
}

//--------------------------------------------------------------------------------
// 5. MAIN LOOP
//--------------------------------------------------------------------------------
void loop() {
  SensorData sensors = readSensors();

  switch (currentState) {
    case INITIAL_BLACK_SQUARE: {
      // Check if all sensors detect black (initial starting square)
      if (sensors.activeCount == 6 && sensors.huntMiddle && !initialBlackHandled) {
        // All sensors detect black - move forward for a bit
        if (stateStartTime == 0) {
          stateStartTime = millis();
          Serial.println("Initial black square detected - Moving forward");
        }
        
        moveForward(BASE_SPEED);
        
        if (millis() - stateStartTime > INITIAL_BLACK_TIME) {
          // Initial black square handling complete
          initialBlackHandled = true;
          currentState = FOLLOWING_LINE;
          stateStartTime = 0;
          Serial.println("Initial black square passed - Starting line following");
        }
      } else {
        // Not all sensors black or already handled - start line following
        currentState = FOLLOWING_LINE;
        initialBlackHandled = true;
        Serial.println("No initial black square - Starting line following");
      }
      break;
    }

    case FOLLOWING_LINE: {
      // Check for junction using S2, S3 and corner hunting IRs
      JunctionType junction = detectAndClassifyJunction(sensors);
      
      if (junction != NO_JUNCTION) {
        // Slow down before processing junction
        moveForward(JUNCTION_APPROACH_SPEED);
        delay(50); // Brief pause for stability
        
        currentState = CLASSIFYING_JUNCTION;
        lastJunctionType = junction;
        stateStartTime = millis();
        Serial.print("Junction detected: ");
        Serial.println(junction);
        delay(JUNCTION_DELAY);
        break;
      }

      // Normal line following with middle hunting IR and S0, S1, S4, S5
      // Middle hunting IR should always detect black for smooth following
      if (!sensors.huntMiddle) {
        Serial.println("Warning: Middle hunting IR lost line!");
      }
      
      error = calculateError(sensors);
      float correction = calculatePID(error);
      controlMotors(correction);
      break;
    }

    case CLASSIFYING_JUNCTION: {
      // Move forward slowly to classify junction type
      moveForward(JUNCTION_APPROACH_SPEED);
      
      if (millis() - stateStartTime > FORWARD_CHECK_TIME) {
        // Re-read sensors after moving forward
        sensors = readSensors();
        
        // Refine junction classification (only for non-Y junctions)
        if (lastJunctionType == LEFT_90) {
          // Verify left junction
          if (sensors.s2 && sensors.s3) {
            lastJunctionType = T_LEFT;
            Serial.println("Classified as T_LEFT");
          } else {
            Serial.println("Confirmed as LEFT_90");
          }
        } else if (lastJunctionType == RIGHT_90) {
          // Verify right junction
          if (sensors.s2 && sensors.s3) {
            lastJunctionType = T_RIGHT;
            Serial.println("Classified as T_RIGHT");
          } else {
            Serial.println("Confirmed as RIGHT_90");
          }
        }
        
        // Check if it's T_BOTH or FOUR_WAY after moving forward
        if (sensors.s2 && sensors.s3 && sensors.huntMiddle) {
          if (!sensors.huntLeft && !sensors.huntRight) {
            lastJunctionType = T_BOTH;
            Serial.println("Classified as T_BOTH");
          } else if (sensors.huntLeft && sensors.huntRight) {
            lastJunctionType = FOUR_WAY;
            Serial.println("Classified as FOUR_WAY");
          }
        }
        
        // Apply left-hand rule
        handleJunctionLeftHandRule(lastJunctionType);
      }
      break;
    }

    case Y_FORWARD_MOVE: {
      // Move forward for 200ms at Y junction before turning left
      moveForward(BASE_SPEED);
      
      if (millis() - stateStartTime > Y_FORWARD_TIME) {
        // Forward movement complete, now turn left
        currentState = TURNING_LEFT;
        stateStartTime = millis();
        Serial.println("Y junction forward move complete - Now turning left");
      }
      break;
    }

    case TURNING_LEFT: {
      turnLeft();
      // Check if turn complete (middle hunting IR detects line)
      if (sensors.huntMiddle && sensors.s2 && sensors.s3) {
        currentState = FOLLOWING_LINE;
        integral = 0;
        lastError = 0;
        Serial.println("Left turn complete");
      }
      break;
    }

    case TURNING_RIGHT: {
      turnRight();
      // Check if turn complete
      if (sensors.huntMiddle && sensors.s2 && sensors.s3) {
        currentState = FOLLOWING_LINE;
        integral = 0;
        lastError = 0;
        Serial.println("Right turn complete");
      }
      break;
    }

    case MOVING_STRAIGHT: {
      moveForward(BASE_SPEED);
      if (millis() - stateStartTime > FORWARD_CHECK_TIME) {
        currentState = FOLLOWING_LINE;
        Serial.println("Straight movement complete");
      }
      break;
    }

    case TURNING_AROUND: {
      turnAround();
      if (millis() - stateStartTime > TURN_180_DURATION) {
        currentState = FOLLOWING_LINE;
        fromDeadEnd = true;
        integral = 0;
        lastError = 0;
        Serial.println("180-degree turn complete");
      }
      break;
    }
  }

  delay(10); // Small delay for stability
}

//--------------------------------------------------------------------------------
// 6. SENSOR READING
//--------------------------------------------------------------------------------
SensorData readSensors() {
  SensorData sensors;
  
  // Read main sensor array (black = true)
  sensors.s0 = analogRead(S0) < SENSOR_THRESHOLD;
  sensors.s1 = analogRead(S1) < SENSOR_THRESHOLD;
  sensors.s2 = analogRead(S2) < SENSOR_THRESHOLD;
  sensors.s3 = analogRead(S3) < SENSOR_THRESHOLD;
  sensors.s4 = analogRead(S4) < SENSOR_THRESHOLD;
  sensors.s5 = analogRead(S5) < SENSOR_THRESHOLD;
  
  // Read hunting sensors
  sensors.huntLeft = digitalRead(HUNT_LEFT) == LOW;   // Assuming LOW = black
  sensors.huntMiddle = digitalRead(HUNT_MIDDLE) == LOW;
  sensors.huntRight = digitalRead(HUNT_RIGHT) == LOW;
  
  sensors.activeCount = sensors.s0 + sensors.s1 + sensors.s2 + 
                        sensors.s3 + sensors.s4 + sensors.s5;
  
  return sensors;
}

//--------------------------------------------------------------------------------
// 7. JUNCTION DETECTION AND CLASSIFICATION
//--------------------------------------------------------------------------------
JunctionType detectAndClassifyJunction(SensorData sensors) {
  // Junction detection uses S2, S3 (middle sensors) and corner hunting IRs
  
  // Check for Y junction first
  // Y junction: One of S2 or S3 sees white, middle hunting IR sees black
  if ((sensors.s2 || sensors.s3) && sensors.huntMiddle && sensors.s0 && sensors.s5) {
    Serial.println("Initial detection: Y_JUNCTION");
    return Y_JUNCTION;
  }
  
  // Check for left-side junction (LEFT_90 or T_LEFT)
  if (sensors.huntLeft && sensors.s2 && sensors.s3) {
    Serial.println("Initial detection: LEFT_90");
    return LEFT_90;
  }
  
  // Check for right-side junction (RIGHT_90 or T_RIGHT)
  if (sensors.huntRight && sensors.s2 && sensors.s3) {
    Serial.println("Initial detection: RIGHT_90");
    return RIGHT_90;
  }
  
  // Check for all sensors black (potential T_BOTH or FOUR_WAY)
  if (sensors.s2 && sensors.s3 && sensors.huntMiddle && 
      (sensors.huntLeft || sensors.huntRight)) {
    Serial.println("Initial detection: Y_JUNCTION (needs classification)");
    return Y_JUNCTION;
  }
  
  // Check for dead end (all sensors white)
  if (!sensors.huntMiddle && !sensors.s2 && !sensors.s3 && 
      sensors.activeCount == 0) {
    Serial.println("Initial detection: DEAD_END");
    return DEAD_END;
  }
  
  return NO_JUNCTION;
}

//--------------------------------------------------------------------------------
// 8. LEFT-HAND RULE JUNCTION HANDLING
//--------------------------------------------------------------------------------
void handleJunctionLeftHandRule(JunctionType junctionType) {
  Serial.print("Applying Left-Hand Rule for: ");
  Serial.println(junctionType);
  
  // Special case: If returning from dead end, don't go straight again
  if (fromDeadEnd && junctionType == FOUR_WAY) {
    // Turn left instead of going straight
    currentState = TURNING_LEFT;
    stateStartTime = millis();
    Serial.println("Action: TURN LEFT (avoiding dead end path)");
    fromDeadEnd = false;
    return;
  }
  
  switch (junctionType) {
    case FOUR_WAY:
      // Special case: Go straight at 4-way junction
      currentState = MOVING_STRAIGHT;
      stateStartTime = millis();
      Serial.println("Action: MOVE STRAIGHT (4-way junction)");
      break;
      
    case T_BOTH:
    case LEFT_90:
      // Left-hand rule: Turn left if left path exists
      currentState = TURNING_LEFT;
      stateStartTime = millis();
      Serial.println("Action: TURN LEFT");
      break;
      
    case T_LEFT:
      // T junction with left path only (no right) - turn left
      currentState = TURNING_LEFT;
      stateStartTime = millis();
      Serial.println("Action: TURN LEFT (T_LEFT - left path available)");
      break;
      
    case T_RIGHT:
      // T junction with right path only (no left) - go straight
      currentState = MOVING_STRAIGHT;
      stateStartTime = millis();
      Serial.println("Action: MOVE STRAIGHT (T_RIGHT - no left path)");
      break;
      
    case RIGHT_90:
      // 90-degree right turn only - turn right
      currentState = TURNING_RIGHT;
      stateStartTime = millis();
      Serial.println("Action: TURN RIGHT (RIGHT_90 - only option)");
      break;
      
    case Y_JUNCTION:
      // Y junction - default to left
      currentState = Y_FORWARD_MOVE;
      stateStartTime = millis();
      Serial.println("Action: TURN LEFT (Y junction)");
      break;
      
    case DEAD_END:
      // Turn around
      currentState = TURNING_AROUND;
      stateStartTime = millis();
      fromDeadEnd = true;
      Serial.println("Action: TURN AROUND (dead end)");
      break;
      
    default:
      currentState = FOLLOWING_LINE;
      Serial.println("Action: CONTINUE");
      break;
  }
  
  // Reset PID
  integral = 0;
  lastError = 0;
}

//--------------------------------------------------------------------------------
// 9. ERROR CALCULATION (using 5 sensors: S0, S1, huntMiddle, S4, S5)
//--------------------------------------------------------------------------------
float calculateError(SensorData sensors) {
  // Use S0, S1, S4, S5 and middle hunting IR for error calculation
  // Middle hunting IR is centered, other sensors spread around it
  
  float weightedSum = 0;
  int activeSensors = 0;
  
  // Weights for error calculation (centered on middle hunting IR)
  if (sensors.s0) { weightedSum += -3.0; activeSensors++; }
  if (sensors.s1) { weightedSum += -2.0; activeSensors++; }
  if (sensors.huntMiddle) { weightedSum += 0.0; activeSensors++; }
  if (sensors.s4) { weightedSum += 2.0; activeSensors++; }
  if (sensors.s5) { weightedSum += 3.0; activeSensors++; }
  
  if (activeSensors == 0) {
    // Line lost - maintain last error
    return lastError;
  }
  
  return weightedSum / activeSensors;
}

//--------------------------------------------------------------------------------
// 10. PID CONTROLLER
//--------------------------------------------------------------------------------
float calculatePID(float currentError) {
  float P = Kp * currentError;
  
  integral += currentError;
  integral = constrain(integral, -50, 50);
  float I = Ki * integral;
  
  float derivative = currentError - lastError;
  float D = Kd * derivative;
  
  lastError = currentError;
  
  return P + I + D;
}

//--------------------------------------------------------------------------------
// 11. MOTOR CONTROL
//--------------------------------------------------------------------------------
void controlMotors(float correction) {
  int leftSpeed = BASE_SPEED - correction;
  int rightSpeed = BASE_SPEED + correction;

  leftSpeed = constrain(leftSpeed, -MAX_SPEED, MAX_SPEED);
  rightSpeed = constrain(rightSpeed, -MAX_SPEED, MAX_SPEED);

  // Smooth acceleration
  static int lastLeftSpeed = 0;
  static int lastRightSpeed = 0;
  leftSpeed = constrain(leftSpeed, lastLeftSpeed - MAX_ACCEL, lastLeftSpeed + MAX_ACCEL);
  rightSpeed = constrain(rightSpeed, lastRightSpeed - MAX_ACCEL, lastRightSpeed + MAX_ACCEL);
  lastLeftSpeed = leftSpeed;
  lastRightSpeed = rightSpeed;

  Serial.print("Error: "); Serial.print(error, 2);
  Serial.print(" | L: "); Serial.print(leftSpeed);
  Serial.print(" | R: "); Serial.println(rightSpeed);

  setMotor(IN1, IN2, ENA, leftSpeed);
  setMotor(IN3, IN4, ENB, rightSpeed);
}

void turnLeft() {
  // Smooth left turn: left motor slow, right motor at turn speed
  setMotor(IN1, IN2, ENA, TURN_SPEED * 0.2); // Left motor at 30% speed
  setMotor(IN3, IN4, ENB, TURN_SPEED);        // Right motor at full turn speed
}

void turnRight() {
  // Smooth right turn: right motor slow, left motor at turn speed
  setMotor(IN1, IN2, ENA, TURN_SPEED);        // Left motor at full turn speed
  setMotor(IN3, IN4, ENB, TURN_SPEED * 0.2); // Right motor at 30% speed
}

void turnAround() {
  // Pivot turn (wheels in opposite directions)
  setMotor(IN1, IN2, ENA, -TURN_SPEED);
  setMotor(IN3, IN4, ENB, TURN_SPEED);
}

void moveForward(int speed) {
  setMotor(IN1, IN2, ENA, speed);
  setMotor(IN3, IN4, ENB, speed);
}

void stopMotors() {
  setMotor(IN1, IN2, ENA, 0);
  setMotor(IN3, IN4, ENB, 0);
}

void setMotor(int in1, int in2, int pwm, int speed) {
  if (speed > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else if (speed < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
  analogWrite(pwm, abs(speed));
}
