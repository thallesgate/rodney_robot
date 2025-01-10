// Include necessary libraries
#include <PID_v1.h>

// Pin definitions
#define ENCODER1_A_PIN PA8
#define ENCODER1_B_PIN PA9
#define ENCODER2_A_PIN PB6
#define ENCODER2_B_PIN PB7
#define MOTOR1_PWM_PIN PA1
#define MOTOR1_IN1_PIN PA2
#define MOTOR1_IN2_PIN PA3
#define MOTOR2_PWM_PIN PA6
#define MOTOR2_IN3_PIN PA5
#define MOTOR2_IN4_PIN PA4

// Wheel and encoder constants
#define WHEEL_DIAMETER 10.0 // in centimeters
#define PULSES_PER_REV 300
#define WHEELBASE 20.8 // in centimeters
#define PI 3.14159265359

// PID tuning parameters
#define Kp 12.0
#define Ki 2.0
#define Kd 1.0

// Encoder variables
volatile long encoder1Count = 0;
volatile long encoder2Count = 0;
volatile long lastEncoder1Count = 0;
volatile long lastEncoder2Count = 0;

// Timing variables
unsigned long lastTime = 0;
unsigned long lastROSMessageTime = 0;
const unsigned long ROSMessageInterval = 33.3; // in milliseconds
int loopCounter = 0;

// Speed control variables
double setpoint1 = 0, input1 = 0, output1 = 0;
double setpoint2 = 0, input2 = 0, output2 = 0;
double filteredInput1 = 0, filteredInput2 = 0;
const double filterAlpha = 0.1; // Smoothing factor

// Linear and angular velocity setpoints
double linearVelocitySetpoint = 0.0; // meters per second
double angularVelocitySetpoint = 0.0; // radians per second

// PID objects
PID pid1(&filteredInput1, &output1, &setpoint1, Kp, Ki, Kd, DIRECT);
PID pid2(&filteredInput2, &output2, &setpoint2, Kp, Ki, Kd, DIRECT);

// Interrupt service routines for encoders
void encoder1ISR() {
    static bool lastA = LOW;
    static bool lastB = LOW;
    bool currentA = digitalRead(ENCODER1_A_PIN);
    bool currentB = digitalRead(ENCODER1_B_PIN);

    if (currentA != lastA) {
        if (currentA == currentB) {
            encoder1Count++;
        } else {
            encoder1Count--;
        }
    }
    if (currentB != lastB) {
        if (currentA != currentB) {
            encoder1Count++;
        } else {
            encoder1Count--;
        }
    }
    lastA = currentA;
    lastB = currentB;
}

void encoder2ISR() {
    static bool lastA = LOW;
    static bool lastB = LOW;
    bool currentA = digitalRead(ENCODER2_A_PIN);
    bool currentB = digitalRead(ENCODER2_B_PIN);

    if (currentA != lastA) {
        if (currentA == currentB) {
            encoder2Count++;
        } else {
            encoder2Count--;
        }
    }
    if (currentB != lastB) {
        if (currentA != currentB) {
            encoder2Count++;
        } else {
            encoder2Count--;
        }
    }
    lastA = currentA;
    lastB = currentB;
}

// Motor control function
void setMotor(int pwmPin, int in1Pin, int in2Pin, double speed) {
    if (speed > 0) {
        digitalWrite(in1Pin, HIGH);
        digitalWrite(in2Pin, LOW);
        analogWrite(pwmPin, (int)speed);
    } else {
        digitalWrite(in1Pin, LOW);
        digitalWrite(in2Pin, HIGH);
        analogWrite(pwmPin, (int)(-speed));
    }
}

void setup() {
    // Set up encoder pins
    pinMode(ENCODER1_A_PIN, INPUT_PULLUP);
    pinMode(ENCODER1_B_PIN, INPUT_PULLUP);
    pinMode(ENCODER2_A_PIN, INPUT_PULLUP);
    pinMode(ENCODER2_B_PIN, INPUT_PULLUP);

    // Attach interrupts
    attachInterrupt(digitalPinToInterrupt(ENCODER1_A_PIN), encoder1ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCODER1_B_PIN), encoder1ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCODER2_A_PIN), encoder2ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ENCODER2_B_PIN), encoder2ISR, CHANGE);

    // Set up motor driver pins
    pinMode(MOTOR1_PWM_PIN, OUTPUT);
    pinMode(MOTOR1_IN1_PIN, OUTPUT);
    pinMode(MOTOR1_IN2_PIN, OUTPUT);
    pinMode(MOTOR2_PWM_PIN, OUTPUT);
    pinMode(MOTOR2_IN3_PIN, OUTPUT);
    pinMode(MOTOR2_IN4_PIN, OUTPUT);

    // Initialize PID controllers
    pid1.SetMode(AUTOMATIC);
    pid2.SetMode(AUTOMATIC);
    pid1.SetOutputLimits(-255, 255);
    pid2.SetOutputLimits(-255, 255);

    // Set initial speed setpoints
    setpoint1 = 0;
    setpoint2 = 0;

    Serial.begin(115200);
}

void loop() {
    // Timing
    unsigned long currentTime = millis();
    double elapsedTime = (currentTime - lastTime) / 1000.0; // in seconds
    if (elapsedTime <= 0) return; // Prevent division by zero

    // Calculate speeds in m/s
    double distancePerPulse = (PI * WHEEL_DIAMETER) / (PULSES_PER_REV); // in meters
    long currentEncoder1Count = encoder1Count;
    long currentEncoder2Count = encoder2Count;
    input1 = ((currentEncoder1Count - lastEncoder1Count) * distancePerPulse) / elapsedTime;
    input2 = ((currentEncoder2Count - lastEncoder2Count) * distancePerPulse) / elapsedTime;

    // Apply low-pass filter
    filteredInput1 = filterAlpha * input1 + (1 - filterAlpha) * filteredInput1;
    filteredInput2 = filterAlpha * input2 + (1 - filterAlpha) * filteredInput2;

    lastEncoder1Count = currentEncoder1Count;
    lastEncoder2Count = currentEncoder2Count;
    lastTime = currentTime;

    // Compute target wheel speeds from linear and angular velocity setpoints
    double targetLeftWheelSpeed = linearVelocitySetpoint - (angularVelocitySetpoint * (WHEELBASE / 2.0));
    double targetRightWheelSpeed = linearVelocitySetpoint + (angularVelocitySetpoint * (WHEELBASE / 2.0));

    // Set PID setpoints
    setpoint1 = targetLeftWheelSpeed;
    setpoint2 = targetRightWheelSpeed;

    // Compute PID
    pid1.Compute();
    pid2.Compute();

    // Set motor speeds
    setMotor(MOTOR1_PWM_PIN, MOTOR1_IN1_PIN, MOTOR1_IN2_PIN, output1);
    setMotor(MOTOR2_PWM_PIN, MOTOR2_IN3_PIN, MOTOR2_IN4_PIN, output2);

    // Send ROS-compatible messages periodically
    if (currentTime - lastROSMessageTime >= ROSMessageInterval) {
        lastROSMessageTime = currentTime;

        Serial.print("twist:");
        Serial.print(" linear: ");
        Serial.print(linearVelocitySetpoint / 100.0);
        Serial.print(" angular: ");
        Serial.println(angularVelocitySetpoint);

        Serial.print("odometry:");
        Serial.print(" left_wheel_speed: ");
        Serial.print(filteredInput1 / 100.0);
        Serial.print(" right_wheel_speed: ");
        Serial.println(filteredInput2  / 100.0);
    }

    // Handle incoming serial data
    if (Serial.available()) {
        String input = Serial.readStringUntil('\n'); // Read one line of input
        if (input.startsWith("twist:")) {
            int linearIndex = input.indexOf("linear:");
            int angularIndex = input.indexOf("angular:");

            if (linearIndex != -1 && angularIndex != -1) {
                // Parse linear velocity
                linearVelocitySetpoint = (input.substring(linearIndex + 7, angularIndex).toDouble()) * 100.0;

                // Parse angular velocity
                angularVelocitySetpoint = input.substring(angularIndex + 8).toDouble();
            }
        }
    }

    delay(25); // Adjust loop rate
}