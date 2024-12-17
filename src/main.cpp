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
#include <errno.h>
#include <MPU6050_Kalman.h>
#include <freertos/FreeRTOS.h>

#define __DEBUG__

#ifdef __DEBUG__
	#define DEBUG_PRINT(x) Serial.print(x)
	#define DEBUG_PRINTLN(x) Serial.println(x)
#else
	#define DEBUG_PRINT(x)
	#define DEBUG_PRINTLN(x)
#endif

WiFiClient client;
PubSubClient MQTT(client);
MPU6050_Kalman mpu;

/**
 * @brief Declare the WiFi parameters
 */
const char *ssid = "iPhone de Klaus"; 	// WiFi network name
const char *password = "batebola"; 		// WiFi network password
 
/**
 * @brief Declare the MQTT parameters
 */
const char *mqttBroker = "mqtt.eclipse.org";	// MQTT broker address
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

/**
 * @brief Declare the system parameters
 */
const int STEPPER_MOTOR_STEP_DELAY = 2000; // Stepper motor step delay in microseconds
const float ABSOLUTE_ANGLE_TOLERANCE = 0.8; // Absolute angle tolerance in degrees
const int STEPPER_MOTOR_STEPS_AUTO = 100; // Stepper motor steps in automatic mode
const int MAXIMUM_ANGLE_AUTO = 45; // Maximum angle in automatic mode
const int MINIMUM_ANGLE_AUTO = -45; // Minimum angle in automatic mode

/**
 * @brief Declare the global variables
 */
float desiredAngle = 0.0;	// Desired angle to move the mirrors
float currentAngle = 0.0;	// Current angle of the mirrors

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
	WiFi.setMinSecurity(WIFI_AUTH_WPA_PSK); // Set the minimum security for the WiFi network
	WiFi.setHostname("fresnel-esp");  		// Set the hostname for the WiFi network
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
	mpu.begin();
	mpu.runRollCalibration();
}

/**
 * @brief Read the MPU6050 sensor
 */
void readMPU6050()
{	
	mpu.readSensors();
	currentAngle = mpu.getRollAngle();	
}

/**
 * @brief Print the angles
*/
void printAngles(void) {
	static unsigned long timer = millis();
	
  	if(millis() - timer > 1000) {
		DEBUG_PRINT("Roll Angle: ");DEBUG_PRINTLN(currentAngle);
		timer = millis();
	}
}

/**
 * @brief Check the WiFi status and reconnect if necessary
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
void pulsePin(int pin, int period = STEPPER_MOTOR_STEP_DELAY)
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

/**
 * @brief Callback function when MQTT message is received
 * @param topic The MQTT topic
 * @param message The message content
 */
void mqttCallback(char* topic, byte* message, unsigned int length) {
    char msg[length + 1];
    for (int i = 0; i < length; i++) {
        msg[i] = (char)message[i];
    }
    msg[length] = '\0';

    desiredAngle = atof(msg);

	if (desiredAngle > MAXIMUM_ANGLE_AUTO) {
		desiredAngle = MAXIMUM_ANGLE_AUTO;
	} else if (desiredAngle < MINIMUM_ANGLE_AUTO) {
		desiredAngle = MINIMUM_ANGLE_AUTO;
	}

    DEBUG_PRINT("Received Sun Position: ");
    DEBUG_PRINTLN(desiredAngle);
}

void setupMQTT(void)
{
	MQTT.setServer(mqttBroker, mqttPort);
	MQTT.subscribe(mqttSubsTopic);
	MQTT.setCallback(mqttCallback);
}

/**
 * @brief Check the MQTT status and reconnect if necessary
 */
void checkMQTT(void)
{
	MQTT.loop();

    if (!MQTT.connected()) {
        DEBUG_PRINTLN("MQTT not connected, reconnecting...");
        if (MQTT.connect("FresnelController")) {
            MQTT.subscribe(mqttSubsTopic);
            DEBUG_PRINTLN("MQTT connected");
        } else {
            DEBUG_PRINTLN("MQTT connection failed, retrying...");
        }
    }
}

/**
 * @brief Move the mirrors to the desired position
 */
void moveMirrorsToPosition(void)
{
	float minAcceptableAngle = desiredAngle - ABSOLUTE_ANGLE_TOLERANCE;
	float maxAcceptableAngle = desiredAngle + ABSOLUTE_ANGLE_TOLERANCE;

	if (currentAngle >= minAcceptableAngle && currentAngle <= maxAcceptableAngle) {
		DEBUG_PRINTLN("Mirrors are already in the desired position");
		return;
	}

	if (currentAngle > MAXIMUM_ANGLE_AUTO - ABSOLUTE_ANGLE_TOLERANCE || 
		currentAngle < MINIMUM_ANGLE_AUTO + ABSOLUTE_ANGLE_TOLERANCE) {
		DEBUG_PRINTLN("Mirrors are in the limit position");
		return;
	}

	DEBUG_PRINT("Desired Angle: ");DEBUG_PRINT(desiredAngle);DEBUG_PRINT(" | Current Angle: ");DEBUG_PRINTLN(currentAngle);

	if (desiredAngle > currentAngle) {
		DEBUG_PRINTLN("Moving motor CW");
    	for (int i = 0; i < STEPPER_MOTOR_STEPS_AUTO; i++) {         
			turnStepperMotorCW();
        }
		return;
    } 
	
	if (desiredAngle < currentAngle) {
        DEBUG_PRINTLN("Moving motor CCW");
        for (int i = 0; i < STEPPER_MOTOR_STEPS_AUTO; i++) {
			turnStepperMotorCCW();
		}
		return;
    }
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
	checkWiFiStatus();

	checkMQTT();

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
		readMPU6050();
		moveMirrorsToPosition();
	}
}

