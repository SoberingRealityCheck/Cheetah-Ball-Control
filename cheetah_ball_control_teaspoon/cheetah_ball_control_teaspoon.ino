#define NONE 0
#define GRAPH 1
#define PRINT_STATE 2
#define PINS 3
#define SIGNAL 4
const int DEBUG_MODE = SIGNAL;

// these should match the pins the Lateral and throttle are connected to from the RC Reciever.
const int LateralPin = 10;
const int ThrottlePin = 11;

// these should match the output pins to the L293 H-Bridge Motor Driver.
const int ENA_L_Pin = 9;
const int IN1_Pin = 8;
const int IN2_Pin = 7;
const int ENA_R_Pin = 5;
const int IN3_Pin = 4;
const int IN4_Pin = 3;

// these variables will change depending on the width of the pulses sent by the receiver
unsigned long ThrottlePulse;
unsigned long LateralPulse;

// these variables will change from 0-255 depending on the strength of the pulse
int ThrottleMappedPulse;
int LateralMappedPulse;

// determine the total strength of the PWM signals
int ThrottleMotivation = 0;
int LateralMotivation = 0;
int Speed = 0;

// these variables will change depending on the forwards-backwards / left-right direction of the command
int ThrottleInterpretedState;
int LateralInterpretedState;

// robot current state (see #define for value meaning)
int RobotState;

// state variable definitions
#define STATIONARY 0
#define LEFT 1
#define RIGHT 2
#define FORWARDS 3
#define BACKWARDS 4

const int MIN_PULSE = 900UL;
const int MAX_PULSE = 2100UL;

// this is just gonna hang out at 0 to make graphing easier
const int static_variable = 0;

// these need to be figured out experimentally based on seeing what the range on these pulse widths is
const int Throttle_Duration_Max = 2090;
const int Throttle_Duration_Min = 1119;
const int Lateral_Duration_Max = 1879;
const int Lateral_Duration_Min = 879;
const int pulse_period = 20;

void setup() {
  // put your setup code here, to run once:
  pinMode(ThrottlePin, INPUT);
  pinMode(LateralPin, INPUT);

  pinMode(ENA_L_Pin, OUTPUT);
  pinMode(IN1_Pin, OUTPUT);
  pinMode(IN2_Pin, OUTPUT);
  pinMode(ENA_R_Pin, OUTPUT);
  pinMode(IN3_Pin, OUTPUT);
  pinMode(IN4_Pin, OUTPUT);

  Serial.begin(9600);
}


byte GetPWM(byte pin)
{
  unsigned long highTime = pulseIn(pin, HIGH, 21000UL);  // 21 millisecond timeout
  unsigned long lowTime = pulseIn(pin, LOW, 21000UL);  // 21 millisecond timeout

  // pulseIn() returns zero on timeout
  if (highTime == 0 || lowTime == 0)
    return digitalRead(pin) ? 100 : 0;  // HIGH == 100%,  LOW = 0%

  return (100 * highTime) / (highTime + lowTime);  // highTime as percentage of total cycle time
}

void loop() {

  // put your main code here, to run repeatedly:
  // LateralState and ThrottleState are the pulse widths measured from the reciever signal pins
  unsigned long newLateralPulse = pulseIn(LateralPin, HIGH);
  unsigned long newThrottlePulse = pulseIn(ThrottlePin, HIGH);

  if (newLateralPulse < Lateral_Duration_Max && newLateralPulse > Lateral_Duration_Min)
    LateralPulse = newLateralPulse;
  if (newThrottlePulse < Throttle_Duration_Max && newThrottlePulse > Throttle_Duration_Min)
    ThrottlePulse = newThrottlePulse;

  // map those pulse widths to a scale from 0-255 to determine the LED brightness
  LateralMappedPulse = map(LateralPulse, Lateral_Duration_Min, Lateral_Duration_Max, 0, 255);
  ThrottleMappedPulse = map(ThrottlePulse, Throttle_Duration_Min, Throttle_Duration_Max, 0, 255);

  // turn it into more of an absolute magnitude from 1-127 in either direction  
  // weird shit happens if LateralMappedPulse = 127 but thats ok because it triggers (Motivation < 20) in the next step and gets labeled stationary
  LateralMotivation = ((abs(LateralMappedPulse - 127)) * 2) - 1;
  ThrottleMotivation = (abs(ThrottleMappedPulse - 127) * 2) - 1;

  // determine direction of lateral and throttle based on whether the mapped pulse is big, and if so whether its greater than or less than 127
  // there is a 'dead zone' here of 20 motivation in either direction that lets the robot stay stationary when the joystick is basically near the center. 
  // we may want to adjust this later depending on how responsive this feels.-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  if (LateralMotivation < 20) {
    LateralInterpretedState = STATIONARY;
  } else if (LateralMappedPulse > 127) {
    LateralInterpretedState = RIGHT;
  } else if (LateralMappedPulse < 127) {
    LateralInterpretedState = LEFT;
  }

  if (ThrottleMotivation < 20) {
    ThrottleInterpretedState = STATIONARY;
  } else if (ThrottleMappedPulse > 127) {
    ThrottleInterpretedState = BACKWARDS;
  } else if (ThrottleMappedPulse < 127) {
    ThrottleInterpretedState = FORWARDS;
  }
  

  // Determine Robot State -- decide whether 'turning' or 'forwards/backwards' wins

  if (ThrottleMotivation > LateralMotivation) {
    RobotState = ThrottleInterpretedState;
    Speed = ThrottleMotivation;
  } else if (LateralMotivation > ThrottleMotivation) { 
    RobotState = LateralInterpretedState;
    Speed = LateralMotivation;
  } else {
    RobotState = STATIONARY; 
    Speed = 0;
  }

  // Determine Motor State based on Robot State
  if (RobotState == LEFT) {
    // do motor forwards stuff with the hbridge
    analogWrite(ENA_L_Pin, Speed);
    analogWrite(ENA_R_Pin, Speed);
    digitalWrite(IN1_Pin, HIGH);
    digitalWrite(IN2_Pin, LOW);
    digitalWrite(IN3_Pin, HIGH);
    digitalWrite(IN4_Pin, LOW);
  } else if (RobotState == RIGHT) {
    // do motor backwards stuff with the hbridge
    analogWrite(ENA_L_Pin, Speed);
    analogWrite(ENA_R_Pin, Speed);
    digitalWrite(IN1_Pin, LOW);
    digitalWrite(IN2_Pin, HIGH);
    digitalWrite(IN3_Pin, LOW);
    digitalWrite(IN4_Pin, HIGH);
  } else if (RobotState == BACKWARDS) {
    // do motor left stuff with the hbridge
    analogWrite(ENA_L_Pin, Speed);
    analogWrite(ENA_R_Pin, Speed);
    digitalWrite(IN1_Pin, HIGH);
    digitalWrite(IN2_Pin, LOW);
    digitalWrite(IN3_Pin, LOW);
    digitalWrite(IN4_Pin, HIGH);
  } else if (RobotState == FORWARDS) {
    // do motor right stuff with the hbridge
    analogWrite(ENA_L_Pin, Speed);
    analogWrite(ENA_R_Pin, Speed);
    digitalWrite(IN1_Pin, LOW);
    digitalWrite(IN2_Pin, HIGH);
    digitalWrite(IN3_Pin, HIGH);
    digitalWrite(IN4_Pin, LOW);
  } else {
    // do motor stationary stuff
    digitalWrite(IN1_Pin, LOW);
    digitalWrite(IN2_Pin, LOW);
    digitalWrite(IN3_Pin, LOW);
    digitalWrite(IN4_Pin, LOW);
  };

  // serial.print step for debugging and graphing
  if (DEBUG_MODE == GRAPH) {
    Serial.print("LateralMappedPulse:");
    Serial.print(LateralMappedPulse);
    Serial.print(",");
    Serial.print("ThrottleMappedPulse:");
    Serial.print(ThrottleMappedPulse);
    Serial.print(",");
    Serial.print("lateral_motivation:");
    Serial.print(LateralMotivation);
    Serial.print(",");
    Serial.print("throttle_motivation:");
    Serial.print(ThrottleMotivation);
    Serial.print(",");
    Serial.print("robot_state:");
    Serial.print(RobotState);
    Serial.print(",");
    Serial.println("");
  } else if (DEBUG_MODE == PRINT_STATE) {
    Serial.print("Robot State: ");
    if (RobotState == FORWARDS) {Serial.print("FORWARDS");
    } else if (RobotState == BACKWARDS) {Serial.print("BACKWARDS");
    } else if (RobotState == LEFT) {Serial.print("LEFT");
    } else if (RobotState == RIGHT) {Serial.print("RIGHT");
    } else if (RobotState == STATIONARY) {Serial.print("STATIONARY");
    }
    Serial.println("");
  }  else if (DEBUG_MODE == PINS) {
    Serial.print("PINS: ");
    Serial.print("ENA:");
    Serial.print(Speed);
    Serial.print("IN1:");
    Serial.print(digitalRead(IN1_Pin));
    Serial.print("IN2:");
    Serial.print(digitalRead(IN2_Pin));
    Serial.print("IN3:");
    Serial.print(digitalRead(IN3_Pin));
    Serial.print("IN4:");
    Serial.print(digitalRead(IN4_Pin));
    Serial.println("");
  } else if (DEBUG_MODE == SIGNAL) {
    Serial.print("SIGNAL STATE: ");
    Serial.print("LateralPulse:");
    Serial.print(LateralPulse);
    Serial.print("newLateralPulse:");
    Serial.print(newLateralPulse);
    Serial.print("LateralMappedPulse:");
    Serial.print(LateralMappedPulse);
    Serial.print("LateralMotivation:");
    Serial.print(LateralMotivation);
    Serial.println("");
  }
  delay(100);
}
