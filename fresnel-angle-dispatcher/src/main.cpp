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
#include "freertos/semphr.h"

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
const char *hostname = "fresnel-angle"; 		// WiFi network hostname
const IPAddress ipaddr(4, 3, 2, 3); 		// WiFi network IP address
const IPAddress gateway(4, 3, 2, 1); 		// WiFi network gateway
const IPAddress subnet(255, 255, 255, 0); 	// WiFi network subnet

/**
 * @brief Declare the MQTT parameters
 */
const int mqttPort = 1883;								// MQTT broker port
const char *mqttBroker = "4.3.2.1";						// MQTT broker address
const char *mqttPubsTopicCurrentAngle = "current-angle";// MQTT publication topic

/**
 * @brief Declare the OUTPUT pins
 */
const char PIN_SystemFaultLed     = 2;	// System fault LED

/**
 * @brief Declare the system parameters
 */
const int NUM_OF_SAMPLES 	  = 100;	// Number of angle samples
const int SAMPLE_PERIOD		  = 10; 	// Period between readings in milliseconds
const float OFFSET_ANGLE	  = -246.53;// Angle calibration offset

/**
 * @brief Setup the GPIO pins
 */
void setupGPIO(void)
{
	// Setup the OUTPUTS pins
	pinMode(PIN_SystemFaultLed,		OUTPUT);
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

    while (true) {
        if (WiFi.status() == WL_CONNECTED && !MQTTClient.connected()) {
            DEBUG_PRINTLN("Connecting to MQTT broker...");
            if (MQTTClient.connect(hostname)) {
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
	float currentAngle = 0.0;

    while (true) {
		// Get the samples
        for (sampleIndex = 0; sampleIndex < NUM_OF_SAMPLES; sampleIndex++) {
			do {
				angles[sampleIndex] = A1332_GetAngle();
				vTaskDelay(SAMPLE_PERIOD / portTICK_PERIOD_MS);
			} while (angles[sampleIndex] == A1332_ANGVAL_ERR);
        }

        // Apply Sigma Clipping
        float filteredAngle = SigmaClipping::filter(angles, sampleIndex);

		currentAngle = filteredAngle + OFFSET_ANGLE;

		if (MQTTClient.connected()) {
			char message[10];
			dtostrf(currentAngle, 1, 2, message);
			MQTTClient.publish(mqttPubsTopicCurrentAngle, message);
			DEBUG_PRINT("Current angle published: "); DEBUG_PRINTLN(currentAngle);
		} else {
			DEBUG_PRINTLN("MQTT not connected, not publishing current angle: "); DEBUG_PRINTLN(currentAngle);
		}

		// Wait for a while before taking the next sample
		vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

/**
 * @brief Blink the LED fault
 */
void blinkLedFault(void)
{
	static unsigned long currTime, prevTime;
	
	currTime = millis();

	if (currTime - prevTime < 500) {
		return;
	}

	digitalWrite(PIN_SystemFaultLed, !digitalRead(PIN_SystemFaultLed));

	prevTime = currTime;
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
    xTaskCreate(taskAngle, "Angle Task", 2048, NULL, 1, NULL);
}

/**
 * @brief Main loop
 */
void loop(void)
{
	// Check if the WiFi or MQTT connection is lost
	if (WiFi.status() != WL_CONNECTED || !MQTTClient.connected()) {
		blinkLedFault();
	} else {
		digitalWrite(PIN_SystemFaultLed, LOW);
	}

	vTaskDelay(500 / portTICK_PERIOD_MS);
}


