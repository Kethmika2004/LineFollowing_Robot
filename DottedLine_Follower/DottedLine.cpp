
// Dotted Line Following Robot with Junction Detection
// For Arduino Mega with 6-channel IR sensor array

// Motor control pins
#define LEFT_MOTOR_PIN1 48
#define LEFT_MOTOR_PIN2 46
#define RIGHT_MOTOR_PIN1 44
#define RIGHT_MOTOR_PIN2 42

// IR sensor pins (assuming analog pins)
#define IR_PIN1 A0  // Leftmost
#define IR_PIN2 A1
#define IR_PIN3 A2
#define IR_PIN4 A3
#define IR_PIN5 A4
#define IR_PIN6 A5  // Rightmost

// Motor speed control
#define BASE_SPEED 120
#define TURN_SPEED 80
#define MAX_SPEED 180

// Threshold values for IR sensors (adjust based on your sensors)
#define BLACK_THRESHOLD 500  // Values above this mean black line detected
#define WHITE_THRESHOLD 100  // Values below this mean white surface

// State variables
enum RobotState {
  FOLLOWING_LINE,
  TURNING_LEFT,
  TURNING_RIGHT,
  TURNING_180,
  AT_JUNCTION,
  RETURNING
};

RobotState currentState = FOLLOWING_LINE;
bool lastTurnWasLeft = false;
bool hasReachedDeadEnd = false;
unsigned long lastLineSeenTime = 0;
unsigned long lineLostThreshold = 500; // ms without line before considering it a dead end

void setup() {
  // Initialize motor control pins
  pinMode(LEFT_MOTOR_PIN1, OUTPUT);
  pinMode(LEFT_MOTOR_PIN2, OUTPUT);
  pinMode(RIGHT_MOTOR_PIN1, OUTPUT);
  pinMode(RIGHT_MOTOR_PIN2, OUTPUT);
  
  // Initialize serial communication for debugging
  Serial.begin(9600);
  
  // Give robot time to stabilize
  delay(2000);
  
  Serial.println("Robot initialized. Starting line following...");
}

void loop() {
  // Read all IR sensors
  bool ir[6];
  ir[0] = digitalRead(IR_PIN1) == HIGH; // Adjust based on your sensor output
  ir[1] = digitalRead(IR_PIN2) == HIGH;
  ir[2] = digitalRead(IR_PIN3) == HIGH;
  ir[3] = digitalRead(IR_PIN4) == HIGH;
  ir[4] = digitalRead(IR_PIN5) == HIGH;
  ir[5] = digitalRead(IR_PIN6) == HIGH;
  
  // For debugging
  Serial.print("Sensors: ");
  for (int i = 0; i < 6; i++) {
    Serial.print(ir[i] ? "1" : "0");
  }
  Serial.print(" State: ");
  Serial.println(currentState);
  
  // State machine
  switch (currentState) {
    case FOLLOWING_LINE:
      handleLineFollowing(ir);
      break;
      
    case TURNING_LEFT:
      executeLeftTurn(ir);
      break;
      
    case TURNING_RIGHT:
      executeRightTurn(ir);
      break;
      
    case TURNING_180:
      execute180Turn(ir);
      break;
      
    case AT_JUNCTION:
      handleJunction(ir);
      break;
      
    case RETURNING:
      handleReturning(ir);
      break;
  }
  
  delay(10); // Small delay for stability
}

void handleLineFollowing(bool ir[6]) {
  // Check if we've lost the line for too long (potential dead end)
  bool lineDetected = ir[0] || ir[1] || ir[2] || ir[3] || ir[4] || ir[5];
  
  if (lineDetected) {
    lastLineSeenTime = millis();
  } else if (millis() - lastLineSeenTime > lineLostThreshold) {
    // We've lost the line for a while, likely at a dead end
    currentState = TURNING_180;
    hasReachedDeadEnd = true;
    Serial.println("Dead end detected. Turning 180 degrees.");
    return;
  }
  
  // Check for junction (multiple sensors detecting line)
  int activeSensors = 0;
  for (int i = 0; i < 6; i++) {
    if (ir[i]) activeSensors++;
  }
  
  if (activeSensors >= 4) {
    currentState = AT_JUNCTION;
    Serial.println("Junction detected.");
    return;
  }
  
  // Normal line following logic
  if (ir[2] || ir[3]) {
    // Center sensors detect line, go straight
    moveForward(BASE_SPEED);
  } 
  else if (ir[1]) {
    // Slightly to the left, adjust right
    adjustRight(BASE_SPEED);
  } 
  else if (ir[4]) {
    // Slightly to the right, adjust left
    adjustLeft(BASE_SPEED);
  } 
  else if (ir[0]) {
    // Far left, turn more right
    adjustRight(BASE_SPEED + 30);
  } 
  else if (ir[5]) {
    // Far right, turn more left
    adjustLeft(BASE_SPEED + 30);
  } 
  else {
    // No line detected, continue in the same direction briefly
    // (helps with dotted lines)
    moveForward(BASE_SPEED - 30);
  }
}

void handleJunction(bool ir[6]) {
  // Determine junction type (left or right)
  bool leftJunction = ir[0] && ir[1];
  bool rightJunction = ir[4] && ir[5];
  
  if (hasReachedDeadEnd) {
    // We're returning, turn in the same direction as before
    if (lastTurnWasLeft) {
      currentState = TURNING_LEFT;
      Serial.println("Returning: Turning left at junction.");
    } else {
      currentState = TURNING_RIGHT;
      Serial.println("Returning: Turning right at junction.");
    }
  } else {
    // First time at this junction
    if (leftJunction && !rightJunction) {
      currentState = TURNING_LEFT;
      lastTurnWasLeft = true;
      Serial.println("Turning left at junction.");
    } else if (rightJunction && !leftJunction) {
      currentState = TURNING_RIGHT;
      lastTurnWasLeft = false;
      Serial.println("Turning right at junction.");
    } else {
      // T-junction or cross junction - default to right
      currentState = TURNING_RIGHT;
      lastTurnWasLeft = false;
      Serial.println("T-junction detected. Turning right by default.");
    }
  }
}

void executeLeftTurn(bool ir[6]) {
  // Turn left until we detect the line again
  turnLeft(TURN_SPEED);
  
  // Check if we've completed the turn
  if (ir[2] && ir[3]) {
    currentState = FOLLOWING_LINE;
    Serial.println("Left turn completed.");
  }
}

void executeRightTurn(bool ir[6]) {
  // Turn right until we detect the line again
  turnRight(TURN_SPEED);
  
  // Check if we've completed the turn
  if (ir[2] && ir[3]) {
    currentState = FOLLOWING_LINE;
    Serial.println("Right turn completed.");
  }
}

void execute180Turn(bool ir[6]) {
  // Turn 180 degrees
  static unsigned long turnStartTime = 0;
  static bool isTurning = false;
  
  if (!isTurning) {
    turnStartTime = millis();
    isTurning = true;
    turnRight(TURN_SPEED);
  }
  
  // Estimate time needed for 180-degree turn (adjust based on your robot)
  if (millis() - turnStartTime > 1500) { // Adjust this value as needed
    isTurning = false;
    currentState = RETURNING;
    Serial.println("180-degree turn completed. Now returning.");
  }
}

void handleReturning(bool ir[6]) {
  // Similar to line following but with different behavior at junctions
  bool lineDetected = ir[0] || ir[1] || ir[2] || ir[3] || ir[4] || ir[5];
  
  if (lineDetected) {
    lastLineSeenTime = millis();
  } else if (millis() - lastLineSeenTime > lineLostThreshold) {
    // Lost the line, try to recover
    moveForward(BASE_SPEED - 30);
    return;
  }
  
  // Check for junction (multiple sensors detecting line)
  int activeSensors = 0;
  for (int i = 0; i < 6; i++) {
    if (ir[i]) activeSensors++;
  }
  
  if (activeSensors >= 4) {
    currentState = AT_JUNCTION;
    return;
  }
  
  // Normal line following logic (same as before)
  if (ir[2] && ir[3]) {
    moveForward(BASE_SPEED);
  } 
  else if (ir[1]) {
    adjustRight(BASE_SPEED);
  } 
  else if (ir[4]) {
    adjustLeft(BASE_SPEED);
  } 
  else if (ir[0]) {
    adjustRight(BASE_SPEED + 30);
  } 
  else if (ir[5]) {
    adjustLeft(BASE_SPEED + 30);
  } 
  else {
    moveForward(BASE_SPEED - 30);
  }
}

// Motor control functions
void moveForward(int speed) {
  analogWrite(LEFT_MOTOR_PIN1, speed);
  analogWrite(LEFT_MOTOR_PIN2, 0);
  analogWrite(RIGHT_MOTOR_PIN1, speed);
  analogWrite(RIGHT_MOTOR_PIN2, 0);
}

void turnLeft(int speed) {
  analogWrite(LEFT_MOTOR_PIN1, 0);
  analogWrite(LEFT_MOTOR_PIN2, speed);
  analogWrite(RIGHT_MOTOR_PIN1, speed);
  analogWrite(RIGHT_MOTOR_PIN2, 0);
}

void turnRight(int speed) {
  analogWrite(LEFT_MOTOR_PIN1, speed);
  analogWrite(LEFT_MOTOR_PIN2, 0);
  analogWrite(RIGHT_MOTOR_PIN1, 0);
  analogWrite(RIGHT_MOTOR_PIN2, speed);
}

void adjustLeft(int speed) {
  analogWrite(LEFT_MOTOR_PIN1, speed - 40);
  analogWrite(LEFT_MOTOR_PIN2, 0);
  analogWrite(RIGHT_MOTOR_PIN1, speed);
  analogWrite(RIGHT_MOTOR_PIN2, 0);
}

void adjustRight(int speed) {
  analogWrite(LEFT_MOTOR_PIN1, speed);
  analogWrite(LEFT_MOTOR_PIN2, 0);
  analogWrite(RIGHT_MOTOR_PIN1, speed - 40);
  analogWrite(RIGHT_MOTOR_PIN2, 0);
}

void stopMotors() {
  analogWrite(LEFT_MOTOR_PIN1, 0);
  analogWrite(LEFT_MOTOR_PIN2, 0);
  analogWrite(RIGHT_MOTOR_PIN1, 0);
  analogWrite(RIGHT_MOTOR_PIN2, 0);
}
