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
#include <A1332.h>
#include <SigmaClipping.h>

#define __DEBUG__

#ifdef __DEBUG__
	#define DEBUG_PRINT(x) Serial.print(x)
	#define DEBUG_PRINTLN(x) Serial.println(x)
#else
	#define DEBUG_PRINT(x)
	#define DEBUG_PRINTLN(x)
#endif

WiFiClient WifiClient;
PubSubClient MQTTClient(WifiClient);

/**
 * @brief Declare the WiFi parameters
 */
const char *ssid = "fresnel-pi"; 			// WiFi network name
const char *password = "12345678"; 			// WiFi network password
const char *hostname = "fresnel-esp"; 		// WiFi network hostname
const IPAddress ipaddr(4, 3, 2, 2); 		// WiFi network IP address
const IPAddress gateway(4, 3, 2, 1); 		// WiFi network gateway
const IPAddress subnet(255, 255, 255, 0); 	// WiFi network subnet

/**
 * @brief Declare the MQTT parameters
 */
const int mqttPort = 1883;					// MQTT broker port
const char *mqttBroker = "4.3.2.1";			// MQTT broker address
const char *mqttSubsTopic = "desired-angle";// MQTT subscription topic
const char *mqttPubsTopic = "current-angle";// MQTT publication topic

/**
 * @brief Declare the INPUT pins
 */
const char PIN_ManualModeSwitch  = 12;	// Manual mode switch
const char PIN_ManualCWButton    = 32;	// Manual CW button
const char PIN_ManualCCWButton   = 33;	// Manual CCW button
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
const int MAN_STEP_DELAY 	  = 20000; 	// Stepper motor step delay in microseconds
const int AUTO_MAX_ANGLE      = 15; 	// Maximum angle in automatic mode
const int AUTO_MIN_ANGLE      = -15; 	// Minimum angle in automatic mode
const int AUTO_MIN_STEP_DELAY = 20000; 	// Minimum step delay in microseconds
const int AUTO_MAX_STEP_DELAY = 100000; // Maximum step delay in microseconds
const int NUM_OF_SAMPLES 	  = 100;	// Number of angle samples
const int SAMPLE_PERIOD		  = 10; 	// Period between readings in milliseconds
const float OFFSET_ANGLE	  = 0;		// Angle calibration offset

/**
 * @brief Declare the global variables
 */
bool manualMode;			// Manual mode flag
bool leftLimitSwitch;		// Left limit switch flag
bool rightLimitSwitch;		// Right limit switch flag
bool turnCW;				// Turn Clockwise flag
bool turnCCW;				// Turn Counter Clockwise flag
float desiredAngle = 0.0;	// Desired angle of the mirrors
float currentAngle = 0.0;	// Current angle of the mirrors

/**
 * @brief Interrupt Service Routine (ISR) for the left limit switch
 */
void IRAM_ATTR leftLimitSwitchISR(void)
{
	leftLimitSwitch = digitalRead(PIN_LeftLimitSwitch) == LOW;
}

/**
 * @brief Interrupt Service Routine (ISR) for the right limit switch
 */
void IRAM_ATTR rightLimitSwitchISR(void)
{
	rightLimitSwitch = digitalRead(PIN_RightLimitSwitch) == LOW;
}

/**
 * @brief Interrupt Service Routine (ISR) for the manual mode switch
 */
void IRAM_ATTR manualModeSwitchISR(void)
{
	manualMode = digitalRead(PIN_ManualModeSwitch) == LOW;
}

/**
 * @brief Interrupt Service Routine (ISR) for the manual CW button
 */
void IRAM_ATTR manualCWButtonISR(void)
{
	turnCW = digitalRead(PIN_ManualCWButton) == LOW;
}

/**
 * @brief Interrupt Service Routine (ISR) for the manual CCW button
 */
void IRAM_ATTR manualCCWButtonISR(void)
{
	turnCCW = digitalRead(PIN_ManualCCWButton) == LOW;
}

/**
 * @brief Setup the GPIO pins
 */
void setupGPIO(void)
{
	// Setup the INPUTS pins
	pinMode(PIN_ManualModeSwitch, 	INPUT_PULLUP);
	manualMode = !digitalRead(PIN_ManualModeSwitch);
	pinMode(PIN_ManualCWButton, 	INPUT_PULLUP);
	turnCW = !digitalRead(PIN_ManualCWButton);
	pinMode(PIN_ManualCCWButton, 	INPUT_PULLUP);
	turnCCW = !digitalRead(PIN_ManualCCWButton);
	pinMode(PIN_LeftLimitSwitch, 	INPUT_PULLUP);
	leftLimitSwitch = !digitalRead(PIN_LeftLimitSwitch);
	pinMode(PIN_RightLimitSwitch, 	INPUT_PULLUP);
	rightLimitSwitch = !digitalRead(PIN_RightLimitSwitch);
	pinMode(PIN_StepperMotorFault, 	INPUT_PULLUP);
	pinMode(PIN_CalibrationMode, 	INPUT_PULLUP);

	// Setup the OUTPUTS pins
	pinMode(PIN_WiFiStatusLed, 		OUTPUT);
	pinMode(PIN_SystemFaultLed,		OUTPUT);
	pinMode(PIN_StepperMotorStep, 	OUTPUT);
	pinMode(PIN_StepperMotorDir, 	OUTPUT);

	// Attach the interrupts
	attachInterrupt(PIN_LeftLimitSwitch,  leftLimitSwitchISR,  CHANGE);
	attachInterrupt(PIN_RightLimitSwitch, rightLimitSwitchISR, CHANGE);
	attachInterrupt(PIN_ManualModeSwitch, manualModeSwitchISR, CHANGE);
	attachInterrupt(PIN_ManualCWButton,   manualCWButtonISR,   CHANGE);
	attachInterrupt(PIN_ManualCCWButton,  manualCCWButtonISR,  CHANGE);
}

/**
 * @brief Task to handle the WiFi connection
 * 
 * @param param 
 */
void taskWiFi(void* param) {
    if (!WiFi.config(ipaddr, gateway, subnet)) {
        DEBUG_PRINTLN("Failed to configure Static IP.");
    }
    WiFi.begin(ssid, password);
    while (true) {
        if (WiFi.status() != WL_CONNECTED) {
            DEBUG_PRINTLN("Reconnecting to Wi-Fi...");
            WiFi.begin(ssid, password);
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

/**
 * @brief Task to handle the MQTT connection
 * 
 * @param param
 */
void taskMQTT(void* param) {
    MQTTClient.setServer(mqttBroker, mqttPort);
    MQTTClient.setCallback([](char* topic, byte* payload, unsigned int length) {
        char message[length + 1];
        memcpy(message, payload, length);
        message[length] = '\0';
        desiredAngle = atof(message);
        desiredAngle = constrain(desiredAngle, AUTO_MIN_ANGLE, AUTO_MAX_ANGLE);
        DEBUG_PRINT("Desired angle updated: ");
        DEBUG_PRINTLN(desiredAngle);
    });

    while (true) {
        if (!MQTTClient.connected()) {
            DEBUG_PRINTLN("Connecting to MQTT broker...");
            if (MQTTClient.connect(hostname)) {
                MQTTClient.subscribe(mqttSubsTopic);
                DEBUG_PRINTLN("MQTT connected!");
            } else {
                DEBUG_PRINTLN("Failed to connect to MQTTClient.");
                vTaskDelay(2000 / portTICK_PERIOD_MS);
            }
        }
        MQTTClient.loop();
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

/**
 * @brief Task to handle the A1332 sensor
 * 
 * @param param
 */
void taskAngle(void* param) {
    static float angles[NUM_OF_SAMPLES];
    int sampleIndex = 0;

    while (true) {
		// Get the samples
        for (sampleIndex = 0; sampleIndex < NUM_OF_SAMPLES; sampleIndex++) {
            angles[sampleIndex] = A1332_GetAngle();
            vTaskDelay(SAMPLE_PERIOD / portTICK_PERIOD_MS);
        }


        // Apply Sigma Clipping
        float filteredAngle = SigmaClipping::filter(angles, sampleIndex);

		currentAngle = filteredAngle + OFFSET_ANGLE;
		DEBUG_PRINT("Current Angle: "); DEBUG_PRINTLN(currentAngle);
    }
}

/**
 * @brief Make a pulse in the pin with period time in microseconds
 * @param pin The pin to pulse
 * @param period The period time
*/
void pulsePin(int pin, int period = MAN_STEP_DELAY)
{
	DEBUG_PRINT("PERIOD: ");DEBUG_PRINTLN(period);
	digitalWrite(pin, HIGH);
	delayMicroseconds(period/2);
	digitalWrite(pin, LOW);
	delayMicroseconds(period/2);
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
 * @brief Turn the stepper motor to Clockwise (CW)
 * 
 * @param stepDelay The step delay in microseconds
 */
void turnStepperMotorCW(int stepDelay = MAN_STEP_DELAY)
{
	if(leftLimitSwitch) {
		DEBUG_PRINTLN("Left Limit Switch Pressed");
		blinkLedFault();
		return;
	}
	DEBUG_PRINTLN("Turning Motor CW");
	pulsePin(PIN_StepperMotorStep, stepDelay);
}

/**
 * @brief Turn the stepper motor to Counter Clockwise (CCW)
 * 
 * @param stepDelay The step delay in microseconds
 */
void turnStepperMotorCCW(int stepDelay = MAN_STEP_DELAY)
{
	if(rightLimitSwitch) {
		DEBUG_PRINTLN("Right Limit Switch Pressed");
		blinkLedFault();
		return;
	}
	DEBUG_PRINTLN("Turning Motor CCW");
	pulsePin(PIN_StepperMotorDir, stepDelay);
}

/**
 * @brief Calculate the step delay based on the error
 * 
 * @param error The error value
 * @return The step delay
 */
int calculateStepDelay(float error) {
	if (error < (AUTO_MAX_ANGLE - AUTO_MIN_ANGLE) * 25 / 100) {
		return map(error * 100, 0, (AUTO_MAX_ANGLE - AUTO_MIN_ANGLE) * 25, AUTO_MAX_STEP_DELAY, AUTO_MIN_STEP_DELAY);
	} else {
		return AUTO_MIN_STEP_DELAY;
	}
}

void moveToPositionIncremental() {
	DEBUG_PRINT("Desired Angle: ");DEBUG_PRINT(desiredAngle);DEBUG_PRINT(" | Current Angle: ");DEBUG_PRINTLN(currentAngle);

    if (abs(currentAngle - desiredAngle)) {
        float error = abs(desiredAngle - currentAngle);
        int stepDelay = calculateStepDelay(error);

        if (desiredAngle > currentAngle) {
    		turnStepperMotorCW(stepDelay);
		} else {
			turnStepperMotorCCW(stepDelay);
    	}
    } else {
        DEBUG_PRINTLN("Target position reached.");
    }
}


/**
 * @brief Setup the board
 */
void setup(void)
{
	Wire.begin();
	Serial.begin(115200);

	setupGPIO();

    xTaskCreate(taskWiFi, "WiFi Task", 2048, NULL, 1, NULL);
    xTaskCreate(taskMQTT, "MQTT Task", 2048, NULL, 1, NULL);
    xTaskCreate(taskAngle, "MPU Task", 2048, NULL, 1, NULL);
}

/**
 * @brief Main loop
 */
void loop(void)
{
	if (manualMode) { 
		/* Manual Mode */
		if (turnCW && turnCCW) {
			DEBUG_PRINTLN("Both buttons pressed");
		} else {
			if (turnCW) {
				turnStepperMotorCW();
			} else if (turnCCW) {
				turnStepperMotorCCW();
			}
		}
	} else { 
		/* Automatic Mode */
		moveToPositionIncremental();
	}
}


