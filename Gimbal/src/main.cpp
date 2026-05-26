#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>
#include <MPU6050.h>


// put declarations here:
int myFunction(int, int);
Servo pitchServo;
Servo rollServo;
MPU6050 mpu;

const int PITCH_NEUTRAL = 90; // I am assuming the neutral position is 90.
const int ROLL_NEUTRAL = 90;

float pitch_offset = 0.0;
float roll_offset = 0.0;

const int CALIBRATION_SAMPLES = 500; // for more accuracy increase the number of trials.

float gyroPitchRate, gyroRollRate, accelPitch, accelRoll;

float gyroPitch, gyroRoll;

unsigned long lastTime = 0;
float dt = 0;

float pitch = 0;
float roll  = 0;

const float ALPHA = 0.98;

// target pitch and roll angles

const float setpointPitch = 0;
const float setpointRoll = 0;

float inputPitch = 0;
float inputRoll = 0;

float outputPitch = 0;
float outputRoll = 0;

// ── PID gains (tune these) ────────────────────────────────
float Kp = 2.0;
float Ki = 0.05;
float Kd = 0.8;

// !!IMPORTANT!!
// the above valeus are still subject to be tuned.

// ── PID internal state ────────────────────────────────────
float pitchIntegral  = 0.0,  rollIntegral  = 0.0;
float pitchPrevError = 0.0,  rollPrevError = 0.0;

// ── Output limits: max correction in degrees ──────────────
const float PID_OUT_MIN = -30.0;
const float PID_OUT_MAX =  30.0;

const unsigned long LOOP_INTERVAL_MS = 10;  // 100 Hz update rate

// This should set up everything the Arduino needs before the gimbal starts running.
void initializeHardware() {
	Serial.begin(115200); // 115200 is a baud value. (number of signals per second).

	pitchServo.attach(9); // pin still needs to be decided
	rollServo.attach(10); // pins still need to be decided.
	
	pitchServo.write(PITCH_NEUTRAL); // make sure both servos are at the center.
	rollServo.write(ROLL_NEUTRAL);

	Serial.println("Ready!");

	Wire.begin();
	mpu.initialize();

	if (mpu.testConnection()) {
		Serial.println("GY-87 / MPU6050 connected!");
	}
	else {
		Serial.println("Connection failed - check wiring!");
	}
}

// This should measure gyro drift while the gimbal is sitting still.
void calibrateGyro() {
	Serial.println("Calibrating the gyro!");

	float pitchSum = 0; float rollSum = 0;

	for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
		int16_t ax, ay, az, gx, gy, gz;
	
		// basically get readings from the servo.
		mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

		// Accumulate raw readings
		// Convert to degrees / sec
		pitchSum += gx / 131.0; 
		rollSum += gy / 131.0;
		delay(5);
	}

	pitch_offset = pitchSum / CALIBRATION_SAMPLES;
	roll_offset = rollSum / CALIBRATION_SAMPLES;

	Serial.print("Pitch offset: "); Serial.println(pitch_offset);
  	Serial.print("Roll offset:  "); Serial.println(roll_offset);
  	Serial.println("Calibration complete!");
}

void readSensorData() {
	int16_t ax, ay, az; // accelerometer values
	int16_t gx, gy, gz; // gyro values
	
	mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

	float accelX = ax / 16384.0;
	float accelY = ay / 16384.0;
	float accelZ = az / 16384.0;
	
	accelPitch = atan2(accelY, accelZ) * 180 / PI;
  	accelRoll  = atan2(accelX, accelZ) * 180 / PI;
	
	gyroPitchRate = (gx / 131.0) - pitch_offset;
	gyroRollRate = (gy / 131.0) - roll_offset;

  	// Debug output
  	Serial.print("Gyro Pitch Rate and Accelerometer Pitch: "); Serial.print(gyroPitchRate); Serial.print(" / "); Serial.print(accelPitch);
  	Serial.print(" | Gyro Roll Rate and Accelerometer Roll: ");  Serial.print(gyroRollRate); Serial.print(" / "); Serial.print(accelRoll);

}

void calculateAngles() {
	// unsigned long currentTime = millis();
	// dt = (currentTime - lastTime) / 1000.0;
	// lastTime = currentTime;

	gyroPitch = pitch + gyroPitchRate * dt;
	gyroRoll  = roll + gyroRollRate * dt;

	pitch = ALPHA * gyroPitch + (1 - ALPHA) * accelPitch;
	roll = ALPHA * gyroRoll + (1 - ALPHA) * accelRoll;

	Serial.print("Pitch: "); Serial.print(pitch);
  	Serial.print(" | Roll: ");  Serial.println(roll);
}

void initializePIDControllers() {
	pitchIntegral  = 0.0;
	rollIntegral   = 0.0;
	pitchPrevError = 0.0;
  	rollPrevError  = 0.0;
  	outputPitch    = 0.0;
  	outputRoll     = 0.0;
  	Serial.println("PID controllers initialized.");
}

float computePID(float setpoint, float input, float &integral, float &prevError) {
	// Guard against a zero or huge dt on the first call
  	if (dt <= 0.0 || dt > 0.5) return 0.0;

	float error = setpoint - input;

	// Proportional gain
	float pTerm = Kp * error;

	// Integral gain
	integral += error * dt;
	
	// To clamp the Integral so that it can never go over PID_OUT_MAX
	float integralLimit = PID_OUT_MAX / (Ki > 0 ? Ki : 1.0);
	integral = constrain(integral, -integralLimit, integralLimit);
	float iTerm = Ki * integral;
	
	// Derivative Gain
	float dTerm = Kd * (error - prevError) / dt;

	prevError = error;

	return constrain(pTerm + iTerm + dTerm, PID_OUT_MIN, PID_OUT_MAX);
}

void applyServos() {
	// Feed the current measure angles into the PID inputs
	inputPitch = pitch;
	inputRoll  = roll;

	// Compute  orrections for each axis
	outputPitch = computePID(setpointPitch, inputPitch, pitchIntegral, pitchPrevError);
	outputRoll = computePID(setpointRoll, inputRoll, rollIntegral, rollPrevError);

	// Add correction to neutral center position
	int pitchCmd = PITCH_NEUTRAL + (int)outputPitch;
	int rollCmd = ROLL_NEUTRAL + (int)outputRoll;

	// Final Safety clamp to make sure that the Servo is never out of range
	pitchCmd = constrain(pitchCmd, 0, 180);
	rollCmd  = constrain(rollCmd, 0, 180);

	pitchServo.write(pitchCmd);
  	rollServo.write(rollCmd);

  	// Debug
  	Serial.print("Pitch cmd: "); Serial.print(pitchCmd);
  	Serial.print(" | Roll cmd: ");  Serial.println(rollCmd);
}

void maintainLoopTiming() {
  // Block until the target interval has passed
  while (millis() - lastTime < LOOP_INTERVAL_MS) {
    // deliberate busy-wait — keeps loop timing tight
  }

  unsigned long currentTime = millis();
  dt = (currentTime - lastTime) / 1000.0;  // seconds
  lastTime = currentTime;
}

void printDebugData() {
  // Only print every 200 ms — fast enough to tune, slow enough not to choke Serial
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint < 200) return;
  lastPrint = millis();

  Serial.print("Pitch: ");      Serial.print(pitch);
  Serial.print(" | Roll: ");    Serial.print(roll);
  Serial.print(" | P-out: ");   Serial.print(outputPitch);
  Serial.print(" | R-out: ");   Serial.print(outputRoll);
  Serial.print(" | P-off: ");   Serial.print(pitch_offset);
  Serial.print(" | R-off: ");   Serial.println(roll_offset);
}

void setup() {
  	// put your setup code here, to run once:
  	initializeHardware();
  	calibrateGyro();
	initializePIDControllers();
	lastTime = millis();   // ← add this — so the first dt is ~0 ms, not 50+ seconds
}

void loop() {
  	// put your main code here, to run repeatedly:
	Serial.println("Running...");
	readSensorData();
	calculateAngles();
 	applyServos();
	printDebugData();
}
