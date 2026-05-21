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
  	Serial.print("Gyro Pitch Rate and Accelerometer Pitch: "); Serial.print(gyroPitch, accelPitch);
  	Serial.print(" | Gyro Roll Rate and Accelerometer Roll: ");  Serial.println(gyroRoll, accelRoll);
}

void calcuateAngles() {
	unsigned long currentTime = millis();
	dt = (currentTime - lastTime) / 1000.0;
	lastTime = currentTime;

	gyroPitch = pitch + gyroPitchRate * dt;
	gyroRoll  = roll + gyroRollRate * dt;

	pitch = ALPHA * gyroPitch + (1 - ALPHA) * accelPitch;
	roll = ALPHA * gyroRoll + (1 - ALPHA) * accelRoll;

	Serial.print("Pitch: "); Serial.print(pitch);
  	Serial.print(" | Roll: ");  Serial.println(roll);
}

void setup() {
  	// put your setup code here, to run once:
  	initializeHardware();
  	calibrateGyro();
}

void loop() {
  	// put your main code here, to run repeatedly:
	Serial.println("Running...");
	readSensorData();
}
