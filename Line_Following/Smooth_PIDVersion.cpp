
//--------------------------------------------------------------------------------
// 1. PIN DEFINITIONS AND CONSTANTS
//--------------------------------------------------------------------------------

// Motor Driver Pins (L298N)
const int ENA = 10;  // Enable A (Left Motor Speed) - PWM
const int IN1 = 48;  // Input 1 (Left Motor Direction)
const int IN2 = 46;  // Input 2 (Left Motor Direction)
const int ENB = 9;  // Enable B (Right Motor Speed) - PWM
const int IN3 = 44;  // Input 3 (Right Motor Direction)
const int IN4 = 42;  // Input 4 (Right Motor Direction)

// IR Sensor Pins (6-sensor array)
const int S0 = A0; // Rightmost sensor
const int S1 = A1;
const int S2 = A2;
const int S3 = A3;
const int S4 = A4;
const int S5 = A5; // Leftmost sensor

// PID Tuning Constants
const float Kp = 50;   // Proportional gain
const float Ki = 0.01; // Integral gain
const float Kd = 0.00;    // Derivative gain

// Robot Behavior Constants
const int BASE_SPEED = 100;    // Default speed
const int MAX_SPEED = 180;     // Maximum speed
const int MAX_ACCEL = 10;      // Max acceleration per loop
const int SENSOR_THRESHOLD = 400; // Threshold for black/white detection

// Smooth Turn Constants
const int TURN_FAST_SPEED = 140; // Speed of the outer wheel during a smooth turn
const int TURN_SLOW_SPEED = 60;  // Speed of the inner wheel during a smooth turn

// --- KEY CONSTANT ---
const float TURN_ERROR_THRESHOLD = 2.0; // If abs(error) > this, it's a corner

const int TURN_AROUND_DURATION = 800; // ms to turn 180 degrees at a dead end
const int LOST_LINE_TIMEOUT = 250; // Time in ms before assuming dead end

// Initial black square handling
const int INITIAL_BLACK_DURATION = 2000; // Time to move forward on initial black (ms)

//--------------------------------------------------------------------------------
// 2. GLOBAL VARIABLES AND TYPE DEFINITIONS
//--------------------------------------------------------------------------------

struct SensorData {
  bool s0, s1, s2, s3, s4, s5;
  int activeCount;
};

// PID variables
float error = 0;
float lastError = 0;
float integral = 0;
float derivative = 0;

// Robot state
enum RobotState { 
  INITIAL_BLACK,        // State for initial black square
  FOLLOWING_LINE, 
  TURNING_LEFT, 
  TURNING_RIGHT, 
  TURNING_AROUND
};
RobotState currentState = INITIAL_BLACK; // Start in initial black state
unsigned long lostLineTime = 0;
unsigned long turnStartTime = 0;
unsigned long initialBlackStartTime = 0;

// Function prototypes
SensorData readSensors();
bool detectJunction(SensorData sensors);
void handleJunction(SensorData sensors);
float calculateError(SensorData sensors);
float calculatePID(float currentError);
void controlMotors(float correction);
void steerTurn(int direction);
void pivotTurn(int direction);
void setMotor(int in1, int in2, int pwm, int speed);

//--------------------------------------------------------------------------------
// 3. SETUP FUNCTION - Runs once at the beginning
//--------------------------------------------------------------------------------
void setup() {
  Serial.begin(9600);
  Serial.println("L298N Line Following Robot Initializing (In-Place Turn Version)...");

  // Set motor control pins to outputs
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Set sensor pins to inputs
  pinMode(S0, INPUT);
  pinMode(S1, INPUT);
  pinMode(S2, INPUT);
  pinMode(S3, INPUT);
  pinMode(S4, INPUT);
  pinMode(S5, INPUT);

  Serial.println("Ready to go!");
  delay(1000);
}

//--------------------------------------------------------------------------------
// 4. MAIN LOOP - Runs repeatedly
//--------------------------------------------------------------------------------
void loop() {
  SensorData sensors = readSensors();
  bool junctionDetected = detectJunction(sensors);
  bool lineLost = (sensors.activeCount == 0);

  // --- Calculate error ONCE at the start of the loop ---
  error = calculateError(sensors);

  // State machine for robot behavior
  switch (currentState) {
    case INITIAL_BLACK:
      // Check if all sensors detect black (initial condition)
      if (sensors.activeCount == 6) {
        // Move forward for the specified duration
        if (initialBlackStartTime == 0) {
          initialBlackStartTime = millis();
        }
        
        // Continue moving forward
        setMotor(IN1, IN2, ENA, BASE_SPEED);
        setMotor(IN3, IN4, ENB, BASE_SPEED);
        
        // Check if initial movement duration has passed
        if (millis() - initialBlackStartTime > INITIAL_BLACK_DURATION) {
          currentState = FOLLOWING_LINE;
          initialBlackStartTime = 0;
          Serial.println("Initial black square handling complete - Starting line following");
        }
      } else {
        // If not all sensors are black, switch to normal following
        currentState = FOLLOWING_LINE;
        Serial.println("Not all sensors black - Starting line following");
      }
      break;

    case FOLLOWING_LINE:
      // if (junctionDetected) {
      //   handleJunction(sensors);
      // }
      // --- ROBUST TURN DETECTION LOGIC ---
      // If the absolute error is very large, it's a sharp corner. Switch to turning state.
      // if (abs(error) > TURN_ERROR_THRESHOLD) {
      //   if (error > 0) { // Positive error means line is to the left
      //     currentState = TURNING_LEFT;
      //     Serial.println("High positive error - Triggering Sharp Left Turn");
      //   } else { // Negative error means line is to the right
      //     currentState = TURNING_RIGHT;
      //     Serial.println("High negative error - Triggering Sharp Right Turn");
      //   }
      //   // Reset PID to prevent old values from interfering with the turn
      //   integral = 0; 
      //   lastError = 0;
      // }
      // else if (lineLost) {
      //   if (lostLineTime == 0) {
      //     lostLineTime = millis();
      //   } else if (millis() - lostLineTime > LOST_LINE_TIMEOUT) {
      //     currentState = TURNING_AROUND;
      //     turnStartTime = millis();
      //     Serial.println("Line lost - Assuming dead end. Turning around.");
      //     lostLineTime = 0;
      //   }
      // } 
      // Default: Normal line following with PID
      // else {
        lostLineTime = 0;
        float correction = calculatePID(error);
        controlMotors(correction);
      // }
      break;

    case TURNING_LEFT:
      steerTurn(-1); 
      if (sensors.s2 || sensors.s3) {
        currentState = FOLLOWING_LINE;
        Serial.println("Left Turn complete - Resuming following");
        integral = 0;
      }
      break;

    case TURNING_RIGHT:
      steerTurn(1); 
      if (sensors.s2 || sensors.s3) {
        currentState = FOLLOWING_LINE;
        Serial.println("Right Turn complete - Resuming following");
        integral = 0;
      }
      break;

    case TURNING_AROUND:
      pivotTurn(1); 
      if (millis() - turnStartTime > TURN_AROUND_DURATION) {
        currentState = FOLLOWING_LINE;
        Serial.println("180-degree turn complete - Searching for line");
      }
      break;
  }
}

//--------------------------------------------------------------------------------
// 5. HELPER FUNCTIONS
//--------------------------------------------------------------------------------

SensorData readSensors() {
  SensorData sensors;
  sensors.s0 = analogRead(S0) < SENSOR_THRESHOLD;
  sensors.s1 = analogRead(S1) < SENSOR_THRESHOLD;
  sensors.s2 = analogRead(S2) < SENSOR_THRESHOLD;
  sensors.s3 = analogRead(S3) < SENSOR_THRESHOLD;
  sensors.s4 = analogRead(S4) < SENSOR_THRESHOLD;
  sensors.s5 = analogRead(S5) < SENSOR_THRESHOLD;
  
  sensors.activeCount = sensors.s0 + sensors.s1 + sensors.s2 + sensors.s3 + sensors.s4 + sensors.s5;
  
  return sensors;
}

bool detectJunction(SensorData sensors) {
  return (sensors.activeCount >= 3);
}

void handleJunction(SensorData sensors) {
  Serial.println("Junction detected. Applying Left-Hand Rule...");
  
  if (sensors.s4 && sensors.s5) {
    currentState = TURNING_LEFT;
    Serial.println("Decision: Turn Left");
  }
  // else if (sensors.s2 && sensors.s3) {
  //   currentState = FOLLOWING_LINE;
  //   Serial.println("Decision: Go Straight");
  // }
  else if (sensors.s0 && sensors.s1) {
    currentState = TURNING_RIGHT;
    Serial.println("Decision: Turn Right");
  }
  // else {
  //   currentState = TURNING_AROUND;
  //   turnStartTime = millis();
  //   Serial.println("Decision: Dead End - Turn Around");
  // }
  
  integral = 0;
  lastError = 0;
}

float calculateError(SensorData sensors) {
  if (sensors.activeCount == 0) {
    return lastError; // Hold last error if line is lost temporarily
  }

  float weightedSum = 0;
  if (sensors.s0) weightedSum += -2.5;
  if (sensors.s1) weightedSum += -1.5;
  if (sensors.s2) weightedSum += -0.5;
  if (sensors.s3) weightedSum += 0.5;
  if (sensors.s4) weightedSum += 1.5;
  if (sensors.s5) weightedSum += 2.5;

  return weightedSum / sensors.activeCount;
}

float calculatePID(float currentError) {
  float P = Kp * currentError;
  integral += currentError;
  integral = constrain(integral, -30, 30);
  float I = Ki * integral;
  derivative = currentError - lastError;
  float D = Kd * derivative;
  lastError = currentError;
  
  return P + I + D;
}

void controlMotors(float correction) {
  int leftSpeed = BASE_SPEED - correction;
  int rightSpeed = BASE_SPEED + correction;

  leftSpeed = constrain(leftSpeed, -MAX_SPEED, MAX_SPEED);
  rightSpeed = constrain(rightSpeed, -MAX_SPEED, MAX_SPEED);

  static int lastLeftSpeed = 0;
  static int lastRightSpeed = 0;
  // leftSpeed = constrain(leftSpeed, lastLeftSpeed - MAX_ACCEL, lastLeftSpeed + MAX_ACCEL);
  // rightSpeed = constrain(rightSpeed, lastRightSpeed - MAX_ACCEL, lastRightSpeed + MAX_ACCEL);
  lastLeftSpeed = leftSpeed;
  lastRightSpeed = rightSpeed;

  Serial.print("left : "); Serial.print(leftSpeed);Serial.print("     ");
  Serial.print("right :"); Serial.println(rightSpeed);Serial.print("     ");

  setMotor(IN1, IN2, ENA, leftSpeed);
  setMotor(IN3, IN4, ENB, rightSpeed);
}

void steerTurn(int direction) {
  int leftSpeed, rightSpeed;

  if (direction < 0) { // Turn Left - Stop left wheel, spin right wheel forward
    leftSpeed = 0;
    rightSpeed = TURN_FAST_SPEED;
  } else { // Turn Right - Stop right wheel, spin left wheel forward
    leftSpeed = TURN_FAST_SPEED;
    rightSpeed = 0;
  }
  
  setMotor(IN1, IN2, ENA, leftSpeed);
  setMotor(IN3, IN4, ENB, rightSpeed);
}

void pivotTurn(int direction) {
  int leftSpeed = TURN_FAST_SPEED * direction;
  int rightSpeed = -TURN_FAST_SPEED * direction;
  
  setMotor(IN1, IN2, ENA, leftSpeed);
  setMotor(IN3, IN4, ENB, rightSpeed);
}

void setMotor(int in1, int in2, int pwm, int speed) {
  if (speed > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else if (speed < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else { // Brake
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
  analogWrite(pwm, abs(speed));
}
