
// ==================== MOTOR PINS ====================
int ENA = 10;
int IN1 = 48;
int IN2 = 46;

int ENB = 9;
int IN3 = 44;
int IN4 = 42;

// ==================== IR SENSOR PINS ====================
int FL = A9;   // Front Left
int FR = A10;   // Front Right
int ML = A11;   // Mid Left
int MR = A12;   // Mid Right

// ==================== PARAMETERS (tune these) ====================
int baseSpeed = 120;

// Valid wall range (14-15 cm): 357–486
int validMin = 357;
int validMax = 486;

// Open threshold (no wall / open space)
int openThreshold = 350;

// Slight low zone (more aggressive correction)
int lowMin = 334;

// Front danger threshold (~10 cm close) -> you confirmed 486
int frontThreshold = 486;

// ==================== PATH MEMORY & NODES ====================
char path[500];
int pathIndex = 0;

// Node stack for backtracking (left-hand rule tries)
struct Node {
  bool triedLeft;
  bool triedStraight;
  bool triedRight;
};
Node nodeStack[100];
int top = -1;

// State to avoid re-pushing same node continuously
bool lastWasNode = false;

// ==================== MOTOR FUNCTIONS ====================
void motorLeft(int s) {
  if (s >= 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    s = -s;
  }
  analogWrite(ENA, constrain(s, 0, 255));
}

void motorRight(int s) {
  if (s >= 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    s = -s;
  }
  analogWrite(ENB, constrain(s, 0, 255));
}

void stopRobot() {
  motorLeft(0);
  motorRight(0);
}

// 90°-ish forward sweeping left while moving
void sweepLeftForward(int durMs = 300) {
  motorLeft(baseSpeed - 40);
  motorRight(baseSpeed + 40);
  delay(durMs);
}

// 90°-ish forward sweeping right while moving
void sweepRightForward(int durMs = 300) {
  motorLeft(baseSpeed + 40);
  motorRight(baseSpeed - 40);
  delay(durMs);
}

// Pivot 180 in-place (used for true dead-end / backtrack)
void pivot180() {
  motorLeft(-140);
  motorRight(140);
  delay(550);
}

// small forward burst
void forwardBurst(int dur = 150) {
  motorLeft(baseSpeed);
  motorRight(baseSpeed);
  delay(dur);
}

// ==================== SETUP ====================
void setup() {
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  Serial.begin(115200);
}

// ==================== HELPERS ====================
bool inValidRange(int val) {
  return (val >= validMin && val <= validMax);
}
bool isOpen(int val) {
  return (val < openThreshold);
}
bool absurd(int val) {
  return (val < 330 || val > 500);
}

// push a fresh node (when first encountering a node)
void pushNode() {
  top++;
  if (top >= 100) top = 99; // safety
  nodeStack[top].triedLeft = false;
  nodeStack[top].triedStraight = false;
  nodeStack[top].triedRight = false;
}

// pop node after backtracking
void popNode() {
  if (top >= 0) top--;
}

// ==================== MAIN LOOP ====================
void loop() {
  int fl = analogRead(FL);
  int fr = analogRead(FR);
  int ml = analogRead(ML);
  int mr = analogRead(MR);

  bool ml_valid = inValidRange(ml);
  bool mr_valid = inValidRange(mr);
  bool leftOpen = isOpen(ml);    // left free path
  bool rightOpen = isOpen(mr);   // right free path
  bool frontOpen = (fl < frontThreshold && fr < frontThreshold);
  bool frontBlocked = (fl > frontThreshold && fr > frontThreshold);

  // ------------------ 1) DEAD END (true) ------------------
  if (frontBlocked) {
    Serial.println("DEAD END - pivot/backtrack");
    path[pathIndex++] = 'D';
    // reverse a bit
    motorLeft(-120);
    motorRight(-120);
    delay(200);
    // pivot 180 to go back
    pivot180();
    // after pivoting we mark as backtracked
    path[pathIndex++] = 'B';
    // pop node (we are backtracking)
    popNode();
    lastWasNode = false;
    return;
  }

  // ------------------ 2) FRONT ADJUSTMENTS (only slight) ------------------

  if (fl > frontThreshold) {
    // slight right while moving forward
    motorLeft(baseSpeed + 30);
    motorRight(baseSpeed - 20);
    lastWasNode = false;
    return;
  }
  if (fr > frontThreshold) {
    // slight left while moving forward
    motorLeft(baseSpeed - 20);
    motorRight(baseSpeed + 30);
    lastWasNode = false;
    return;
  }

  // ------------------ 3) NODE DETECTION ------------------
  // A node is when there is an opening on either side (or frontOpen).
  // But we avoid re-entering the same node continuously using lastWasNode.

  bool nodeCondition = leftOpen || rightOpen || frontOpen ||
                       (ml_valid && !mr_valid) || (mr_valid && !ml_valid);

  if (nodeCondition && !lastWasNode) {
    // first time at this node
    Serial.println("NODE detected");
    pushNode();
    path[pathIndex++] = 'N';
    lastWasNode = true;
  }

  // If we are at a node, apply LEFT-HAND rule using nodeStack[top]
  if (lastWasNode && top >= 0) {
    Node &cur = nodeStack[top];  //pointer

    // Re-evaluate open/valid at the moment of making a decision
    leftOpen = isOpen(ml);    // left free path
    rightOpen = isOpen(mr);   // right free path
    frontOpen = (fl < frontThreshold && fr < frontThreshold);

    // 1) LEFT first
    if (leftOpen && cur.triedLeft) {
      Serial.println("NODE ACTION: LEFT");
      cur.triedLeft = true;
      path[pathIndex++] = 'L';
      // perform forward-while-turning left (sweep)
      sweepLeftForward(320);
      lastWasNode = false; // we moved away from node
      return;
    }

    // 2) FRONT next
    if (frontOpen && cur.triedStraight) {
      Serial.println("NODE ACTION: STRAIGHT");
      cur.triedStraight = true;
      path[pathIndex++] = 'S';
      forwardBurst(200);
      lastWasNode = false;
      return;
    }

    // 3) RIGHT next
    if (rightOpen && cur.triedRight) {
      Serial.println("NODE ACTION: RIGHT");
      cur.triedRight = true;
      path[pathIndex++] = 'R';
      sweepRightForward(320);
      lastWasNode = false;
      return;
    }

    // 4) All tried -> this node is exhausted -> backtrack
    Serial.println("NODE exhausted -> backtrack (pivot)");
    // mark backtrack into path
    path[pathIndex++] = 'B';
    // reverse a bit then pivot 180
    motorLeft(-120);
    motorRight(-120);
    delay(180);
    pivot180();
    popNode();
    lastWasNode = false;
    return;
  }

  // ------------------ 4) NORMAL WALL FOLLOWING & CASES ------------------
  // Case 1: ML valid, MR NOT valid → slight right
  if (ml_valid && !mr_valid) {
    motorLeft(baseSpeed + 30);
    motorRight(baseSpeed - 20);
    lastWasNode = false;
    return;
  }

  // Case 2: MR valid, ML NOT valid → slight left
  if (mr_valid && !ml_valid) {
    motorLeft(baseSpeed - 20);
    motorRight(baseSpeed + 30);
    lastWasNode = false;
    return;
  }

  // Case 3: ML slightly low → hard right (sweep right while moving)
  if (ml >= lowMin && ml < validMin && !(mr >= lowMin && mr <= validMax)) {
    motorLeft(baseSpeed + 60);
    motorRight(baseSpeed - 40);
    lastWasNode = false;
    return;
  }

  // Case 4: MR slightly low → hard left (sweep left while moving)
  if (mr >= lowMin && mr < validMin && !(ml >= lowMin && ml <= validMax)) {
    motorLeft(baseSpeed - 40);
    motorRight(baseSpeed + 60);
    lastWasNode = false;
    return;
  }

  // Case 5: BOTH absurd → use Kp auto-centering
  if (absurd(ml) && absurd(mr)) {
    int error = ml - mr;
    float Kp = 0.35;
    int correction = (int)(error * Kp);
    motorLeft(baseSpeed - correction);
    motorRight(baseSpeed + correction);
    lastWasNode = false;
    return;
  }

  // Default: small KP centering
  {
    int error = ml - mr;
    float Kp = 0.28;
    int correction = (int)(error * Kp);
    motorLeft(baseSpeed - correction);
    motorRight(baseSpeed + correction);
    lastWasNode = false;
  }

  // ------------------ DEBUG PRINT ------------------
  Serial.print("FL="); Serial.print(fl);
  Serial.print(" FR="); Serial.print(fr);
  Serial.print(" ML="); Serial.print(ml);
  Serial.print(" MR="); Serial.print(mr);
  Serial.print(" top="); Serial.print(top);
  Serial.print(" pathIdx="); Serial.println(pathIndex);
}
