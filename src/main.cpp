// // code  for PID of one motor with Cytron. PID for only position 
#include <Arduino.h>
#include <util/atomic.h> // For the ATOMIC_BLOCK macro

// Motor control pins
const int pinPwm = 9;   // PWM connected to pin 5
const int pinDir = 8;   // DIR connected to pin 4

// Encoder pins
#define ENCA 3  // Encoder channel A connected to pin 3 (YELLOW)
#define ENCB 2  // Encoder channel B connected to pin 2 (WHITE)

/*******************************************************************************
 * PRIVATE GLOBAL VARIABLES                                                     *
 *******************************************************************************/
float Kp = 1.5;   // Proportional gain
float Ki = 5;   // Integral gain
float Kd = 0.1;   // Derivative gain

// float Kp = 1;   // Proportional gain
// float Ki = 0.00;   // Integral gain
// float Kd = 1.5;   // Derivative gain

// PID control variables
float integral = 0;
float previousError = 0;
unsigned long previousTime = 0;

// Position variable, marked volatile because it's used in an ISR
volatile long posi = 0;

// Desired 
float targetAngle = 0.0;
long targetPosition = 0.0;

// Error threshold (deadband)
float errorThreshold = 10;  // Threshold in encoder counts
int PPR = 13;

/*******************************************************************************
 * FUNCTIONS                                                                    *
 *******************************************************************************/

void readEncoder();

void setup() {                
  // Initialize the PWM and DIR pins as digital outputs.
  pinMode(pinPwm, OUTPUT);
  pinMode(pinDir, OUTPUT);

  // Initialize encoder pins
  pinMode(ENCA, INPUT_PULLUP);
  pinMode(ENCB, INPUT_PULLUP);

  // Attach interrupt for ENCA, triggered on a rising edge
  attachInterrupt(digitalPinToInterrupt(ENCA), readEncoder, RISING);

  Serial.begin(115200);  // Initialize serial communication

  // Serial.println("Enter target angle in degrees:");
}

void loop() {
  // Check if there is serial input available
  //if (Serial.available() > 0) {
    targetAngle = 720;  // Parse the input as a float
    targetPosition = (targetAngle * PPR * 174) / 360; // Convert angle to encoder counts
    
    // Reset PID control variables
    integral = 0;
    previousError = 0;
    previousTime = millis();
  //}

  // Read and print the encoder position immediately
  long pos = 0;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    pos = posi;  // Safely read the encoder position
  }

  // Calculate error in position
  long error = targetPosition - pos;

  // Debug: Print error and position for analysis
  // Serial.print(">Target Position: ");
  // Serial.println(targetPosition);
  // Serial.print(">Current Position: ");
  // Serial.println(pos);
  // Serial.print(">Error: ");
  // Serial.println(error);

  // If the error is within the threshold, stop the motor and keep it stopped
  if (abs(error) < errorThreshold) {
    analogWrite(pinPwm, 0);  // Stop the motor
    Serial.println("Motor stopped within threshold.");
    return;  // Exit the loop to avoid unnecessary calculations
  }
  
  // Calculate Delta time
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - previousTime) / 1000.0;  // Convert to seconds

  // PID calculations
  float proportional = Kp * error;           // Proportional Term
  integral += error * deltaTime * Ki;        // Integral Term
  float derivative = 0;                      // Derivative Term

  if (deltaTime > 0) {
    derivative = Kd * (error - previousError) / deltaTime;
  }

  float PIDoutput1 = proportional + integral + derivative;  // PID Output
  float PIDoutput = constrain(PIDoutput1, -30, 30);  // Allow full range for bidirectional control

  // Determine the direction of the voltage
  if (PIDoutput > 0) {
    digitalWrite(pinDir, HIGH);  // CW direction
  } else {
    digitalWrite(pinDir, LOW);   // CCW direction
    PIDoutput = -PIDoutput;      // Use absolute value for PWM
  }

  // Apply the control output to the motor
  analogWrite(pinPwm, PIDoutput);

  // Update the previous values for the next loop iteration
  previousError = error;
  previousTime = currentTime;
  
  // Convert encoder counts back to degrees
  float currentAngle = (pos * 360) / (PPR * 174);  
  float errorAngle = (error * 360) / (PPR * 174); 
  // Send data to Teleplot
  Serial.print("> currentAngle:");
  Serial.println(currentAngle);
  Serial.print("> errorAngle:");
  Serial.println(errorAngle);
  Serial.print("> PIDoutput1:");
  Serial.println(PIDoutput);

  delay(10);  // Small delay to control the loop rate
}

void readEncoder() {
  int b = digitalRead(ENCB);  // Read the state of ENCA

  // Determine the direction of rotation
  if (b > 0) {
    posi++;  // Clockwise: Increment position
  } else {
    posi--;  // Counterclockwise: Decrement position
  }
}
