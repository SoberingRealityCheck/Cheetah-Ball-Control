// these should match the pins the Lateral and throttle are connected to.
// Green cables & LED are for the throttle(CH1), Blue for the Lateral (CH2).
// 5 and 6 are selected as input pins as they are capable of PWM input unlike some other digital pins on the Nano
const int ThrottlePin = 5;
const int LateralPin = 6;

const int MotorPowerPin = 2;

// Row 1 is the front row, Row 2 is the back row

const int MotorL1PinF = 3;
const int MotorL1PinR = 4;
const int MotorR1PinF = 7;
const int MotorR1PinR = 8;
const int MotorL2PinF = 9;
const int MotorL2PinR = 10;
const int MotorR2PinF = 11;
const int MotorR2PinR = 12;

// these variables will change depending on the width of the pulses sent by the receiver
unsigned long ThrottleState;
unsigned long LateralState;

// these variables will change from 0-255 depending on the output brightness of the LEDs
int ThrottleInterpretedState;
int LateralInterpretedState;

// robot current state (see #define for value meaning)
int RobotState;

// motor powered / unpowered state 
int MotorPowerState = 0;

// determine the backwards / forwards state of the motors
int MotorL1State = 0;
int MotorR1State = 0;
int MotorL2State = 0;
int MotorR2State = 0;

const int MIN_PULSE = 900UL;
const int MAX_PULSE = 2100UL;

// this is just gonna hang out at 0 to make graphing easier
const int static_variable = 0;

// these need to be figured out experimentally based on seeing what the range on these pulse widths is
const int Throttle_Duration_Max = 2009;
const int Throttle_Duration_Min = 1009;
const int Lateral_Duration_Max = 1879;
const int Lateral_Duration_Min = 879;
const int pulse_period = 20;

void setup() {
  // put your setup code here, to run once:
  pinMode(ThrottlePin, INPUT);
  pinMode(LateralPin, INPUT);
  
  pinMode(MotorPowerPin, OUTPUT);

  pinMode(MotorL1PinF, OUTPUT);
  pinMode(MotorL1PinR, OUTPUT);
  pinMode(MotorR1PinF, OUTPUT);
  pinMode(MotorR1PinR, OUTPUT);
  pinMode(MotorL2PinF, OUTPUT);
  pinMode(MotorL2PinR, OUTPUT);
  pinMode(MotorR2PinF, OUTPUT);
  pinMode(MotorR2PinR, OUTPUT);

  Serial.begin(9600);
}

#define ROBOT_STATE_STATIONARY 0
#define ROBOT_STATE_TURNING_LEFT 1 
#define ROBOT_STATE_TURNING_RIGHT 2
#define ROBOT_STATE_MOVING_FORWARDS 3
#define ROBOT_STATE_MOVING_BACKWARDS 4

#define MOTOR_STATE_BACKWARDS -1
#define MOTOR_STATE_FORWARDS 1
#define MOTOR_STATE_STATIONARY 0

#define MOTOR_POWER_OFF 0
#define MOTOR_POWER_ON 1

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
  unsigned long newLateralState = pulseIn(LateralPin, HIGH);
  unsigned long newThrottleState = pulseIn(ThrottlePin, HIGH);

  if (newLateralState < 2100 && newLateralState > 600)
    LateralState = newLateralState;
  if (newThrottleState < 2100 && newThrottleState > 600)
    ThrottleState = newThrottleState;
  

  // map those pulse widths to a scale from 0-255 to determine the LED brightness
  LateralInterpretedState = map(LateralState, Lateral_Duration_Min, Lateral_Duration_Max, 0, 255);
  ThrottleInterpretedState = map(ThrottleState, Throttle_Duration_Min, Throttle_Duration_Max, 0, 255);

  // Determine Robot State
  if (LateralInterpretedState < 55) {
    // Turn Left
    RobotState = ROBOT_STATE_TURNING_LEFT;
  } else if (LateralInterpretedState > 200) {
    // Turn Right
    RobotState = ROBOT_STATE_TURNING_RIGHT;
  } else {
    if (ThrottleInterpretedState < 55) {
      // Move Forwards
      RobotState = ROBOT_STATE_MOVING_FORWARDS;
    } else if (ThrottleInterpretedState > 200) {
      // Move Backwards
      RobotState = ROBOT_STATE_MOVING_BACKWARDS;
    } else {
      // Stand Still
      RobotState = ROBOT_STATE_STATIONARY;
    }
  }

  // Determine Motor State
  if (RobotState == ROBOT_STATE_STATIONARY) {
    MotorPowerState = MOTOR_POWER_OFF;
    MotorL1State = MOTOR_STATE_STATIONARY;
    MotorR1State = MOTOR_STATE_STATIONARY;
    MotorL2State = MOTOR_STATE_STATIONARY;
    MotorR2State = MOTOR_STATE_STATIONARY;
    // MotorPowerState = MOTOR_POWER_OFF;
    // MotorL1State = MOTOR_STATE_FORWARDS;
    // MotorR1State = MOTOR_STATE_FORWARDS;
  } else if (RobotState == ROBOT_STATE_TURNING_RIGHT) {
    MotorPowerState = MOTOR_POWER_ON;
    MotorL1State = MOTOR_STATE_BACKWARDS;
    MotorR1State = MOTOR_STATE_FORWARDS;
    MotorL2State = MOTOR_STATE_BACKWARDS;
    MotorR2State = MOTOR_STATE_FORWARDS;
  } else if (RobotState == ROBOT_STATE_TURNING_LEFT) {
    MotorPowerState = MOTOR_POWER_ON;
    MotorL1State = MOTOR_STATE_FORWARDS;
    MotorR1State = MOTOR_STATE_BACKWARDS;
    MotorL2State = MOTOR_STATE_FORWARDS;
    MotorR2State = MOTOR_STATE_BACKWARDS;
  } else if (RobotState == ROBOT_STATE_MOVING_FORWARDS) {
    MotorPowerState = MOTOR_POWER_ON;
    MotorL1State = MOTOR_STATE_FORWARDS;
    MotorR1State = MOTOR_STATE_FORWARDS;
    MotorL2State = MOTOR_STATE_FORWARDS;
    MotorR2State = MOTOR_STATE_FORWARDS;
  } else if (RobotState == ROBOT_STATE_MOVING_BACKWARDS) {
    MotorPowerState = MOTOR_POWER_ON;
    MotorL1State = MOTOR_STATE_BACKWARDS;
    MotorR1State = MOTOR_STATE_BACKWARDS;
    MotorL2State = MOTOR_STATE_BACKWARDS;
    MotorR2State = MOTOR_STATE_BACKWARDS;
  } 

  // translate motor power state into relay command
  if (MotorPowerState == MOTOR_POWER_OFF) {
    digitalWrite(MotorPowerPin, LOW);
  } else if (MotorPowerState == MOTOR_POWER_ON) {
    digitalWrite(MotorPowerPin, HIGH);
  }

  // translate motor state values into actual relay commands
  // these are individual commands instead of one generalized function because 
  // the wiring is difficult to adjust once installed and it is more convenient to 
  // simply flip the code that runs that motor to operate inversely than it is to
  // actually go dig around and fix things inside the electronics box

  // its a lazy solution but it is a solution that will work for the next few days of testing :)
  if (MotorL1State == MOTOR_STATE_STATIONARY) {
    digitalWrite(MotorL1PinF, LOW);
    digitalWrite(MotorL1PinR, LOW);
  }
  if (MotorR1State == MOTOR_STATE_STATIONARY) {
    digitalWrite(MotorR1PinF, LOW);
    digitalWrite(MotorR1PinR, LOW);
  }
  if (MotorL2State == MOTOR_STATE_STATIONARY) {
    digitalWrite(MotorL2PinF, LOW);
    digitalWrite(MotorL2PinR, LOW);
  }
  if (MotorR2State == MOTOR_STATE_STATIONARY) {
    digitalWrite(MotorR2PinF, LOW);
    digitalWrite(MotorR2PinR, LOW);
  }


  if (MotorL1State == MOTOR_STATE_FORWARDS) {
    digitalWrite(MotorL1PinF, LOW);
    digitalWrite(MotorL1PinR, HIGH);
  }
  if (MotorR1State == MOTOR_STATE_FORWARDS) {
    digitalWrite(MotorR1PinF, LOW);
    digitalWrite(MotorR1PinR, HIGH);
  }
  if (MotorL2State == MOTOR_STATE_FORWARDS) {
    digitalWrite(MotorL2PinF, LOW);
    digitalWrite(MotorL2PinR, HIGH);
  }
  if (MotorR2State == MOTOR_STATE_FORWARDS) {
    digitalWrite(MotorR2PinF, LOW);
    digitalWrite(MotorR2PinR, HIGH);
  }
  
  
  if (MotorL1State == MOTOR_STATE_BACKWARDS) {
    digitalWrite(MotorL1PinF, HIGH);
    digitalWrite(MotorL1PinR, LOW);
  }
  if (MotorR1State == MOTOR_STATE_BACKWARDS) {
    digitalWrite(MotorR1PinF, HIGH);
    digitalWrite(MotorR1PinR, LOW);
  }
  if (MotorL2State == MOTOR_STATE_BACKWARDS) {
    digitalWrite(MotorL2PinF, HIGH);
    digitalWrite(MotorL2PinR, LOW);
  }
  if (MotorR2State == MOTOR_STATE_BACKWARDS) {
    digitalWrite(MotorR2PinF, HIGH);
    digitalWrite(MotorR2PinR, LOW);
  }

  // big serial.print step for debugging and graphing
  Serial.print("MotorPowerState:");
  Serial.print(MotorPowerState);
  Serial.print(",");
  Serial.print("MotorR1State:");
  Serial.print(MotorR1State);
  // Serial.print("ThrottlePin:");
  // Serial.print(analogRead(ThrottlePin));
  // Serial.print(",");
  // Serial.print("LateralPin:");
  // Serial.print(analogRead(LateralPin));
  // Serial.print("Robot State:");
  // Serial.print(RobotState);
  // Serial.print(",");
  // Serial.print("Left Motor State:");
  // Serial.print(MotorL1State);
  // Serial.print("Right Motor State:");
  // Serial.print(MotorR1State);
  // Serial.print("Left Motor Forward Pin:");
  // Serial.print(digitalRead(MotorL1PinF));
  // Serial.print("Left Motor Backward Pin:");
  // Serial.print(digitalRead(MotorL1PinR));
  // Serial.print("Right Motor Forward Pin:");
  // Serial.print(digitalRead(MotorR1PinF));
  // Serial.print("Right Motor Backward Pin:");
  // Serial.print(digitalRead(MotorR1PinR));

  // Serial.print("ThrottleInterpretedState:");
  // Serial.print(ThrottleInterpretedState);
  // Serial.print(",");
  // Serial.print("LateralInterpretedState:");
  // Serial.print(LateralInterpretedState);
  // Serial.print(",");
  // Serial.print("Static");
  // Serial.print(static_variable);

  // Serial.print("ThrottleInterpretedState:");
  // Serial.print(GetPWM(ThrottlePin));
  // Serial.print(",");
  // Serial.print("LateralInterpretedState:");
  // Serial.print(GetPWM(LateralPin));
  
  Serial.println("");
  
  //delay a bit
  // delay(11);
}
