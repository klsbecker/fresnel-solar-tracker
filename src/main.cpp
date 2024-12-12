/**
 * @file 	fresnel.ino
 * @brief 	Linear Fresnel Solar Concentrator Controller
 * @author 	Klaus Becker (BeckerKlaus@edu.unisinos.br)
 * 
 * @details This code is responsible for controlling the Linear Fresnel Solar Concentrator.
 * 			The controller can be operated in two modes: automatic and manual.
 * 			In automatic mode, the controller will move the mirrors to track the sun.
 *			To track the sun, the controller will use the MQTT protocol via WiFi to communicate
 * 			with the raspberry pi, which will provide the sun's position.
 * 			In manual mode, the controller will move the mirrors according to the user's command.
 * 			The controller will also check for possible faults, such as stepper motor fault, 
 * 			left limit switch pressed, and right limit switch pressed.
 */

#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <Preferences.h>
#include <MPU6050_light.h>
#include <RunningAverage.h>
#include <errno.h>

// #define __DEBUG__

#ifdef __DEBUG__
	#define DEBUG_PRINT(x) Serial.print(x)
	#define DEBUG_PRINTLN(x) Serial.println(x)
#else
	#define DEBUG_PRINT(x)
	#define DEBUG_PRINTLN(x)
#endif

MPU6050 mpu(Wire);
WiFiClient client;
PubSubClient MQTT(client);
Preferences preferences;
RunningAverage tiltAngleX(100);
RunningAverage tiltAngleY(100);
RunningAverage tiltAngleZ(100);

/**
 * @brief Declare the WiFi parameters
 */
const char *ssid = "iPhone de Klaus"; 	// WiFi network name
const char *password = "batebola"; 		// WiFi network password
 
/**
 * @brief Declare the MQTT parameters
 */
const char *mqttBroker = "fresnel-pi"; 	// MQTT broker name
const int mqttPort = 1883;				// MQTT broker port
const char *mqttSubsTopic = "controller";
const char *mqttPubsTopic = "teste";

/**
 * @brief Declare the INPUT pins
 */
const char PIN_ManualModeSwitch  = 12;	// Manual mode switch
const char PIN_ManualCWButton  = 32;	// Manual CW button
const char PIN_ManualCCWButton = 33;	// Manual CCW button
const char PIN_LeftLimitSwitch   = 25;	// Left limit switch
const char PIN_RightLimitSwitch  = 26;	// Right limit switch
const char PIN_StepperMotorFault = 17;	// Stepper motor fault (NOT USED NOW)
const char PIN_CalibrationMode	 = 23; 	// Calibration mode button (NOT USED NOW)

/**
 * @brief Declare the OUTPUT pins
 */
const char PIN_WiFiStatusLed      = 4;	// WiFi status LED (NOT USED NOW)
const char PIN_SystemFaultLed     = 2;	// System fault LED
const char PIN_StepperMotorStep   = 27;	// Stepper motor step
const char PIN_StepperMotorDir    = 14;	// Stepper motor direction

bool isMPU6050Configured = false;

/**
 * @brief Setup the GPIO pins
 */
void setupGPIO(void)
{
	// Setup the INPUTS pins
	pinMode(PIN_ManualModeSwitch, 	INPUT_PULLUP);
	pinMode(PIN_ManualCWButton, 	INPUT_PULLUP);
	pinMode(PIN_ManualCCWButton, 	INPUT_PULLUP);
	pinMode(PIN_LeftLimitSwitch, 	INPUT_PULLUP);
	pinMode(PIN_RightLimitSwitch, 	INPUT_PULLUP);
	pinMode(PIN_StepperMotorFault, 	INPUT_PULLUP);
	pinMode(PIN_CalibrationMode, 	INPUT_PULLUP);

	// Setup the OUTPUTS pins
	pinMode(PIN_WiFiStatusLed, 		OUTPUT);
	pinMode(PIN_SystemFaultLed,		OUTPUT);
	pinMode(PIN_StepperMotorStep, 	OUTPUT);
	pinMode(PIN_StepperMotorDir, 	OUTPUT);
	// pinMode(PIN_StepperMotorEnable,	OUTPUT);
}

/**
 * @brief Setup the WiFi connection
 */
void setupWiFi(void)
{
	//WiFi.setMinSecurity(WIFI_AUTH_WPA_PSK); // Set the minimum security for the WiFi network
	//WiFi.setHostname("fresnel-esp");  		// Set the hostname for the WiFi network
	WiFi.begin(ssid, password); 			// Connect to the network
}

/**
 * @brief Check if the manual mode switch is enabled
 */
bool isManualMode(void)
{
	return digitalRead(PIN_ManualModeSwitch);
}

/**
 * @brief Check if the calibration mode button is pressed
*/
bool isCalibrationMode(void)
{
	return !digitalRead(PIN_CalibrationMode);
}

void setupMPU6050(void)
{	
	float accelOffsetX, accelOffsetY, accelOffsetZ;
	float gyroOffsetX, gyroOffsetY, gyroOffsetZ;
	
	tiltAngleX.clear();
	tiltAngleY.clear();
	tiltAngleZ.clear();

	Wire.begin();
	
	unsigned long startTime = millis();
	while (mpu.begin() != ERR_OK) {
		if (millis() - startTime > 10000) {
			DEBUG_PRINTLN("MPU6050 device not found");
			return;
		}
		delay(500);
	}
	
	mpu.setFilterGyroCoef(0.5);

	preferences.begin("MPU-Calibration");

	if (isCalibrationMode()) {
		DEBUG_PRINTLN("Calibration Button Pressed");

		mpu.calcOffsets();
		
		accelOffsetX = mpu.getAccXoffset();
		accelOffsetY = mpu.getAccYoffset();
		accelOffsetZ = mpu.getAccZoffset();

		gyroOffsetX = mpu.getGyroXoffset();
		gyroOffsetY = mpu.getGyroYoffset();
		gyroOffsetZ = mpu.getGyroZoffset();

		preferences.putFloat("accelOffsetX", accelOffsetX);
		preferences.putFloat("accelOffsetY", accelOffsetY);
		preferences.putFloat("accelOffsetZ", accelOffsetZ);

		preferences.putFloat("gyroOffsetX", gyroOffsetX);
		preferences.putFloat("gyroOffsetY", gyroOffsetY);
		preferences.putFloat("gyroOffsetZ", gyroOffsetZ);

		DEBUG_PRINTLN("Calibration Done Successfully");
		DEBUG_PRINT("Accel Offset X: ");DEBUG_PRINTLN(accelOffsetX);
		DEBUG_PRINT("Accel Offset Y: ");DEBUG_PRINTLN(accelOffsetY);
		DEBUG_PRINT("Accel Offset Z: ");DEBUG_PRINTLN(accelOffsetZ);
		DEBUG_PRINT("Gyro Offset X: ");DEBUG_PRINTLN(gyroOffsetX);
		DEBUG_PRINT("Gyro Offset Y: ");DEBUG_PRINTLN(gyroOffsetY);
		DEBUG_PRINT("Gyro Offset Z: ");DEBUG_PRINTLN(gyroOffsetZ);
	} else {
		DEBUG_PRINTLN("Calibration Button Not Pressed");
		/* Check if all nedded keys exists */
		if (preferences.isKey("accelOffsetX") && preferences.isKey("accelOffsetY") &&
			preferences.isKey("accelOffsetZ") && preferences.isKey("gyroOffsetX") &&
			preferences.isKey("gyroOffsetY") && preferences.isKey("gyroOffsetZ")) {
			/* If all keys exists gets them */
			accelOffsetX = preferences.getFloat("accelOffsetX");
			accelOffsetY = preferences.getFloat("accelOffsetY");
			accelOffsetZ = preferences.getFloat("accelOffsetZ");

			gyroOffsetX  = preferences.getFloat("gyroOffsetX");
			gyroOffsetY  = preferences.getFloat("gyroOffsetY");
			gyroOffsetZ  = preferences.getFloat("gyroOffsetZ");

			mpu.setAccOffsets(accelOffsetX, accelOffsetY, accelOffsetZ);
			mpu.setGyroOffsets(gyroOffsetX, gyroOffsetY, gyroOffsetZ);

			DEBUG_PRINTLN("Offsets restored successfully");
			DEBUG_PRINT("Accel Offset X: ");DEBUG_PRINTLN(accelOffsetX);
			DEBUG_PRINT("Accel Offset Y: ");DEBUG_PRINTLN(accelOffsetY);
			DEBUG_PRINT("Accel Offset Z: ");DEBUG_PRINTLN(accelOffsetZ);
			DEBUG_PRINT("Gyro Offset X: ");DEBUG_PRINTLN(gyroOffsetX);
			DEBUG_PRINT("Gyro Offset Y: ");DEBUG_PRINTLN(gyroOffsetY);
			DEBUG_PRINT("Gyro Offset Z: ");DEBUG_PRINTLN(gyroOffsetZ);
		} else {
			DEBUG_PRINTLN("Offsets restored failed");
		}
	}

	isMPU6050Configured = true;

	preferences.end();
}

/**
 * @brief Read the MPU6050 sensor
 */
void readMPU6050()
{	
	if (!isMPU6050Configured) {
		DEBUG_PRINTLN("MPU6050 not configured");
		return;
	}

	static unsigned long timer = millis();
	
  	if(millis() - timer > 100) {
		mpu.update();

		tiltAngleX.add(mpu.getAngleX());
		tiltAngleY.add(mpu.getAngleY());
		tiltAngleZ.add(mpu.getAngleZ());

		timer = millis();
	}	
}

/**
 * @brief Print the angles
*/
void printAngles(void) {
	static unsigned long timer = millis();
	
  	if(millis() - timer > 1000) {
		DEBUG_PRINT("X: ");DEBUG_PRINTLN(tiltAngleX.getAverage());
		DEBUG_PRINT("Y: ");DEBUG_PRINTLN(tiltAngleY.getAverage());
		DEBUG_PRINT("Z: ");DEBUG_PRINTLN(tiltAngleZ.getAverage());
		timer = millis();
	}	
}

/**
 * @brief Check the WiFi status
 */
void checkWiFiStatus(void)
{
	static unsigned long currTime, prevTime;
	
	currTime = millis();

	if (currTime - prevTime < 200) {
		return;
	}

	if (WiFi.status() == WL_CONNECTED) {
		digitalWrite(PIN_WiFiStatusLed, HIGH);
	} else {
		digitalWrite(PIN_WiFiStatusLed, !digitalRead(PIN_WiFiStatusLed));
	}

	prevTime = currTime;
}

/**
 * @brief Make a pulse in the pin with period time in microseconds
 * @param pin The pin to pulse
 * @param period The period time
*/
void pulsePin(int pin, int period = 20000)
{
	digitalWrite(pin, HIGH);
	delayMicroseconds(period/2);
	digitalWrite(pin, LOW);
	delayMicroseconds(period/2);
}

/**
 * @brief Turn the stepper motor to CW
 */
void turnStepperMotorCW(void)
{
	pulsePin(PIN_StepperMotorStep);
}

/**
 * @brief Turn the stepper motor to CCW
 */
void turnStepperMotorCCW(void)
{
	pulsePin(PIN_StepperMotorDir);
}

/**
 * @brief Check if the left limit switch is pressed
 */
bool isLeftLimitSwitchPressed(void)
{
	return !digitalRead(PIN_LeftLimitSwitch);
}

/**
 * @brief Check if the left limit switch is pressed
 */
bool isRightLimitSwitchPressed(void)
{
	return !digitalRead(PIN_RightLimitSwitch);
}

/**
 * @brief Check if the stepper motor is in fault
 */
bool isStepperMotorFault(void)
{
	return !digitalRead(PIN_StepperMotorFault);
}

/**
 * @brief Check if the manual left button is pressed
 */
bool isCWButtonPressed(void)
{
	return !digitalRead(PIN_ManualCWButton);
}

/**
 * @brief Check if the manual right button is pressed
 */
bool isCCWButtonPressed(void)
{
	return !digitalRead(PIN_ManualCCWButton);
}

/**
 * @brief Blink the LED fault
 */
void blinkLedFault(void)
{
	static unsigned long currTime, prevTime;
	
	currTime = millis();

	if (currTime - prevTime < 200) {
		return;
	}

	digitalWrite(PIN_SystemFaultLed, !digitalRead(PIN_SystemFaultLed));

	prevTime = currTime;
}

/**
 * @brief Check for any faults
 * @return true if any fault is detected, false otherwise
 */
bool isAnyFaultDetected(void)
{
	if (isStepperMotorFault() ||
		isLeftLimitSwitchPressed() ||
		isRightLimitSwitchPressed()) {
		blinkLedFault();
		return true;
	} else {
		digitalWrite(PIN_SystemFaultLed, LOW);
		return false;
	}
}

void setupMQTT(void)
{
	MQTT.setServer(mqttBroker, mqttPort);
	MQTT.subscribe(mqttSubsTopic);
}

/**
 * @brief Setup the board
 */
void setup(void)
{
	Serial.begin(115200);

	setupGPIO();

	setupWiFi();

	setupMQTT();

	setupMPU6050();
}

/**
 * @brief Main loop
 */
void loop(void)
{
	// readMPU6050();

	// printAngles();

	if (isManualMode()) { /* Manual Mode */
		DEBUG_PRINTLN("Manual Mode");
		if (isCWButtonPressed() && isCCWButtonPressed()) {
			DEBUG_PRINTLN("Both buttons pressed");
			return;
		}

		if (isCWButtonPressed()){
			if (isLeftLimitSwitchPressed()) {
				DEBUG_PRINTLN("Left Limit Switch Pressed");
				blinkLedFault();
			} else {
				DEBUG_PRINTLN("Turning CW ->");
				turnStepperMotorCW();
			}
		}
		
		if (isCCWButtonPressed()) {
			if (isRightLimitSwitchPressed()) {
				DEBUG_PRINTLN("Right Limit Switch Pressed");
				blinkLedFault();
			} else {
				DEBUG_PRINTLN("Turning CCW <-");
				turnStepperMotorCCW();
			}
		}
	} else { /* Automatic Mode */
		DEBUG_PRINTLN("Automatic Mode");
	}
}

