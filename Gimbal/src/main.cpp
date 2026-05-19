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

initializeHardware() {
	Serial.begin(115200); // 115200 is a baud value. (number of signals per second).

	pitchServo.attach(); // pin still needs to be decided
	rollServo.attach(); // pins still need to be decided.
	
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

void setup() {
  // put your setup code here, to run once:
  initializeHardware();
  int result = myFunction(2, 3);
}

void loop() {
  // put your main code here, to run repeatedly:
	Serial.println("Running...");
  	// Serial.println(analogRead()); // pin still needs to be decided and I am not sure if we ant to read this as analog or as a normalized value.
	int16_t ax, ay, az;  // Accelerometer
  	int16_t gx, gy, gz;  // Gyroscope

  	mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  	Serial.print("Gyro X: "); Serial.print(gx);
  	Serial.print(" | Y: ");   Serial.print(gy);
  	Serial.print(" | Z: ");   Serial.println(gz);

	// Convert obtained values to degrees per sec.
	
	float gyroX = gx / 131.0;
	float gyroY = gy / 131.0;
	float gyroZ = gz / 131.0;

  	delay(1000);
}

// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}
