/*
 * Copyright (C) 2017 Robert Bosch. All Rights Reserved. 
 *
 * Disclaimer
 *
 * Common:
 * Bosch Sensortec products are developed for the consumer goods industry. They may only be used
 * within the parameters of the respective valid product data sheet.  Bosch Sensortec products are
 * provided with the express understanding that there is no warranty of fitness for a particular purpose.
 * They are not fit for use in life-sustaining, safety or security sensitive systems or any system or device
 * that may lead to bodily harm or property damage if the system or device malfunctions. In addition,
 * Bosch Sensortec products are not fit for use in products which interact with motor vehicle systems.
 * The resale and/or use of products are at the purchasers own risk and his own responsibility. The
 * examination of fitness for the intended use is the sole responsibility of the Purchaser.
 *
 * The purchaser shall indemnify Bosch Sensortec from all third party claims, including any claims for
 * incidental, or consequential damages, arising from any product use not covered by the parameters of
 * the respective valid product data sheet or not approved by Bosch Sensortec and reimburse Bosch
 * Sensortec for all costs in connection with such claims.
 *
 * The purchaser must monitor the market for the purchased products, particularly with regard to
 * product safety and inform Bosch Sensortec without delay of all security relevant incidents.
 *
 * Engineering Samples are marked with an asterisk (*) or (e). Samples may vary from the valid
 * technical specifications of the product series. They are therefore not intended or fit for resale to third
 * parties or for use in end products. Their sole purpose is internal client testing. The testing of an
 * engineering sample may in no way replace the testing of a product series. Bosch Sensortec
 * assumes no liability for the use of engineering samples. By accepting the engineering samples, the
 * Purchaser agrees to indemnify Bosch Sensortec from all claims arising from the use of engineering
 * samples. 
 *
 * Special:
 * This software module (hereinafter called "Software") and any information on application-sheets
 * (hereinafter called "Information") is provided free of charge for the sole purpose to support your
 * application work. The Software and Information is subject to the following terms and conditions:
 *
 * The Software is specifically designed for the exclusive use for Bosch Sensortec products by
 * personnel who have special experience and training. Do not use this Software if you do not have the
 * proper experience or training.
 *
 * This Software package is provided `` as is `` and without any expressed or implied warranties,
 * including without limitation, the implied warranties of merchantability and fitness for a particular
 * purpose.
 *
 * Bosch Sensortec and their representatives and agents deny any liability for the functional impairment
 * of this Software in terms of fitness, performance and safety. Bosch Sensortec and their
 * representatives and agents shall not be liable for any direct or indirect damages or injury, except as
 * otherwise stipulated in mandatory applicable law.
 *
 * The Information provided is believed to be accurate and reliable. Bosch Sensortec assumes no
 * responsibility for the consequences of use of such Information nor for any infringement of patents or
 * other rights of third parties which may result from its use. No license is granted by implication or
 * otherwise under any patent or patent rights of Bosch. Specifications mentioned in the Information are
 * subject to change without notice.
 *
 * It is not allowed to deliver the source code of the Software to any third party without permission of
 * Bosch Sensortec.
 *
 */

/*!
 * @file bsec_iot_example.ino
 *
 * @brief
 * Example for using of BSEC library in a fixed configuration with the BME680 sensor.
 * This works by running an endless loop in the bsec_iot_loop() function.
 */

/*!
 * @addtogroup bsec_examples BSEC Examples
 * @brief BSEC usage examples
 * @{*/

/**********************************************************************************************************************/
/* header files */
/**********************************************************************************************************************/

#include <Arduino.h>
#include "bsec_integration.h"
#include <WiFi.h>
#include <Wire.h>
#include <ArduinoJson.h>
#include "menu.h"
#include "EEPROM_utils.h"
#include "user_functions.h"
#include "mic.h"
#include "averages.h"
#include <PubSubClient.h>
#include "bsec_integration.h"
#include "ClosedCube_OPT3001.h"


#define OPT3001_ADDRESS 0x44  //opt3001 ambient sensor address


/**********************************************************************************************************************/
/* defines */
/**********************************************************************************************************************/
#define EEPROM_SIZE         1024
#define SAVE_STATE_SAMPLES 30000

/**********************************************************************************************************************/
/* global variables */
/**********************************************************************************************************************/
/* connection parameters */
const char* ssid = "BTIA_MGD4CLJ";  //TP-Link_8744
const char* password = "E2mUwdihPmDr"; //90108822
//BTIA_3rd_Floor
//RobertBosch0100

/*mqtt global variables*/

const int mqttPort =  1883; // mqtt port HiveMQ
const char* mqtt_server = "broker.hivemq.com"; 

/*mqtt definitions*/

/*all the constant values may be changed */

WiFiClient GezaClient;  //GezaClient
PubSubClient client(GezaClient);

/* interrupt flag */
volatile int interruptCounter;
/* interrupt (seconds) counter */
uint64_t totalInterruptCounter;
/* hardware timer */ 
hw_timer_t * timer = NULL;
/* portMUX_TYPE variable used synchronize the main loop and the ISR */
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

/* Establish connection timer counter */
uint8_t connectionCounter;
// Removed because load state does not work properly
// /* Save state counter */
// uint32_t saveCounter = 0;
/* state save flag */
uint8_t save = 0;

/* LED */
const int ledPinRED = 19;
const int ledPinGREEN = 18;

/* Number of failed requests */
uint8_t failed_requests = 0;


/* Message to be published by MQTT client */
// char pub_str[100],IAQ[100],LOUDNESS[100];

/**********************************************************************************************************************/
/* functions */
/**********************************************************************************************************************/
ClosedCube_OPT3001 opt3001; //object using the opt3001's functions
/*!
 * @brief       ISR function.
 *              Sets flag so that interrupt is signalled in the main loop.
 *
 * @return      None
 */
void IRAM_ATTR onTimer() {
  portENTER_CRITICAL_ISR(&timerMux);
  interruptCounter++;
  portEXIT_CRITICAL_ISR(&timerMux);
}

/*!
 * @brief       Write explanation
 *
 * @return      None
 */
 void callback(char* topic, byte* payload, unsigned int length)
{
  Serial.print("Message arrived : ");
  Serial.print(topic);
  Serial.print(" : ");
  for (int i = 0; i < length; i++)
  {
    Serial.println((char)payload[i]);
  }
  if ((char)payload[0] == 'o' && (char)payload[1] == 'n')
  {
    digitalWrite(2, LOW);
  }
  else if ((char)payload[0] == 'o' && (char)payload[1] == 'f' && (char)payload[2] == 'f' ) 
  {
    digitalWrite(2, HIGH);
  }
}
void configureSensor() {
	OPT3001_Config newConfig;   /*configuring the sensor*/
	
	newConfig.RangeNumber = B1100;	
	newConfig.ConvertionTime = B0;
	newConfig.Latch = B1;
	newConfig.ModeOfConversionOperation = B11;

	OPT3001_ErrorCode errorConfig = opt3001.writeConfig(newConfig);
	if (errorConfig != NO_ERROR)
		Serial.printf("OPT3001 configuration", errorConfig);
	else {
		OPT3001_Config sensorConfig = opt3001.readConfig();
		Serial.println("OPT3001 Current Config:");
		Serial.println("------------------------------");
		
		Serial.print("Conversion ready (R):");
		Serial.println(sensorConfig.ConversionReady,HEX);

		Serial.print("Conversion time (R/W):");
		Serial.println(sensorConfig.ConvertionTime, HEX);

		Serial.print("Fault count field (R/W):");
		Serial.println(sensorConfig.FaultCount, HEX);

		Serial.print("Flag high field (R-only):");
		Serial.println(sensorConfig.FlagHigh, HEX);

		Serial.print("Flag low field (R-only):");
		Serial.println(sensorConfig.FlagLow, HEX);

		Serial.print("Latch field (R/W):");
		Serial.println(sensorConfig.Latch, HEX);

		Serial.print("Mask exponent field (R/W):");
		Serial.println(sensorConfig.MaskExponent, HEX);

		Serial.print("Mode of conversion operation (R/W):");
		Serial.println(sensorConfig.ModeOfConversionOperation, HEX);

		Serial.print("Polarity field (R/W):");
		Serial.println(sensorConfig.Polarity, HEX);

		Serial.print("Overflow flag (R-only):");
		Serial.println(sensorConfig.OverflowFlag, HEX);

		Serial.print("Range number (R/W):");
		Serial.println(sensorConfig.RangeNumber, HEX);

		Serial.println("------------------------------");
	}
}

void printError(String text, OPT3001_ErrorCode error) {  /*error code of opt3001*/
	Serial.print(text);
	Serial.print(": [ERROR] Code #");
	Serial.println(error);
}
float lux;
void printResult(String text, OPT3001 result) { /*displaying the result in 'lux'*/
	if (result.error == NO_ERROR) {
        float lightness;
		Serial.print(text);
		Serial.print(": ");
		Serial.print(result.lux);
        lux=result.lux;
		Serial.println(" lux");
        lightness=map(result.lux,0,8386.5,0,100);  //mapping the lux value in order to show in percentage
        Serial.printf("\nlightness in percentage: %.2f\n ",lightness);
	}
	else {
		printError(text,result.error);
	}
}
float getLuxResult() { /*displaying the result in 'lux'*/
        return lux;
}

void setup()
{
    opt3001.begin(OPT3001_ADDRESS);
    //  char ClId[]="";  used for concatenating the publishing data
    // setup pin 19 (RED LED) as a digital output pin
    pinMode (ledPinRED, OUTPUT);
    // setup pin 18 (GREEN LED) as a digital output pin
    pinMode (ledPinGREEN, OUTPUT);

    digitalWrite (ledPinRED, LOW);	// turn on the RED LED
    digitalWrite (ledPinGREEN, LOW);	// turn on the GREEN LED

    /* BSEC return value */
    return_values_init ret;

    /* Initialize EEPROM */
    EEPROM.begin(EEPROM_SIZE);
    /* Init I2C and serial communication */
    Wire.begin();
    Serial.begin(115200);
    /* Set timeout to 5 seconds when entergin data */
    Serial.setTimeout(5000);

    /* Check EEPROM flag to see if defaults already written */
    /* This will happen only once, after programming */
    if (EEPROM_get_flag()!='x')
    {
        /* Write defaults to memory */
        Serial.print("Writing defaults to memory");
        delay(300);
        Serial.print(".");
        delay(300);
        Serial.print(".");
        delay(300);
        Serial.println(".");
        EEPROM_write_defaults();
        // Here we should set default state, and write it to memory
    }

    /* Call to the function which initializes the BSEC library 
     * Switch on low-power mode and provide no temperature offset */
    ret = bsec_iot_init(BSEC_SAMPLE_RATE_LP, 5.0f, bus_write, bus_read, sleep_BME, state_load, config_load);
    if (ret.bme680_status)
    {
        /* Could not intialize BME680 */
        Serial.println("Error while initializing BME680");
        return;
    }
    else if (ret.bsec_status)
    {
        /* Could not intialize BSEC library */
        Serial.println("Error while initializing BSEC library");
        return;
    }
    /*Displaying the chip id of bme680*/
    displayChipID();
    
    /* Display BSEC version */
    bsec_version_t  version;
    bsec_get_version(&version);
    Serial.printf("BSEC version: %d.%d.%d.%d\r\n",version.major, version.minor, version.major_bugfix, version.minor_bugfix);

    /* Initialize I2S */
    init_mic(); 

    /* Enter the main menu for changing connection parameters, if desired ($$$ pressed) */
    if(menu_combination())
    {
        enter_menu();
    }
    /* Read data from EEPROM */
    // EEPROM_read_data(SSID_ADDRESS,&ssid);
    // EEPROM_read_data(PASS_ADDRESS,&password);

    /* Set prescaler to 80 on timer so that it counts microseconds (clock is 80MHz). */
    timer = timerBegin(0, 80, true);
    /* Bind timer to a handling function. */
    timerAttachInterrupt(timer, &onTimer, true);
    /* Specify the counter value in which the timer interrupt will be generated. */
    timerAlarmWrite(timer, 1000000, true);
    /* Enable the timer with a call to the timerAlarmEnable function. */
    timerAlarmEnable(timer);
    /* Initialize counter for computing averages */
    reset_counter();
    reset_stored_values();

    /* Set required MQTT data */
    client.setServer(mqtt_server, 1883);
    client.setCallback(callback);

    digitalWrite (ledPinRED, HIGH);	// turn off the RED LED
    digitalWrite (ledPinGREEN, HIGH);	// turn off the GREEN LED
}

#define SAMPLES 128 // make it a power of two for best DMA performance

void loop()
{   
    /* MAC address of the actal device */
    String mac = String(WiFi.macAddress());
    /* An interrupt occured (1 second passed) */
    
    OPT3001 result = opt3001.readResult();

    if (interruptCounter > 0) 
    {
        /* Clear the interrupt */
        portENTER_CRITICAL(&timerMux);
        interruptCounter--;
        portEXIT_CRITICAL(&timerMux);

        /* Increase the total interrupt counter, performed every second */
        totalInterruptCounter++;
        /* Increase connection counter */
        connectionCounter++;
        
        // toggle GREEN LED each second
        if ((totalInterruptCounter%2)==0)
        { 
            digitalWrite (ledPinGREEN, LOW);	// turn on the GREEN LED
        }
        else
        {
            digitalWrite (ledPinGREEN, HIGH);	// turn off the GREEN LED
        }

        /* Reset connection timer every 20 seconds */
        if ((totalInterruptCounter%20)==0)
        {
            Serial.println("Ending loop (20s).");
            connectionCounter=0;
        }
        /* Connect to wiFi every second after 3 readings of the BME (10 seconds) */
        if(connectionCounter==1)
        {
            Serial.println("Setting connection parameters.");
            IPAddress local_IP(0, 0, 0, 0);
            IPAddress gateway(192, 168, 1, 254);
            IPAddress subnet(255, 255, 255, 0);
            IPAddress primaryDNS(8, 8, 8, 8); //optional
            IPAddress secondaryDNS(8, 8, 4, 4); //optional
            WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS);
            WiFi.begin(ssid,password);
            Serial.print("Connecting       ");
            Serial.println(connectionCounter);
        }
        /* If connected to network connect to MQTT server */
        else if(connectionCounter==6)
        {
            Serial.println("Checking connection status.");
            if (WiFi.status() == WL_CONNECTED)
            {     
                Serial.println("Connected to WiFi.");
                digitalWrite (ledPinRED, HIGH);	// turn off the RED LED
                Serial.println("Turned LED off.");

                /* Try connecting to MQTT server */
                Serial.println("Attempting MQTT connection");
                if(client.connect("HiveMQ"))
                {
                  Serial.println("Connected to MQTT server.");
                  client.publish("ESP3","Connected!");
                }
                else
                {
                  Serial.print("Failed, ");
                  switch(client.state())
                  {
                      case 1: Serial.print("Connection refused, unacceptable protocol version");break;
                      case 2: Serial.print("Connection refused, identifier rejected");break;
                      case 3: Serial.print("Connection refused, server unavailable");break;
                      case 4: Serial.print("Connection refused, bad user name or password");break;
                      case 5: Serial.print("Connection refused, not authorized");break;
                  }
                  Serial.print("\nrc= "+client.state());
                }
               /* -------------------------------------------------------- */
            }
            else
            {
                digitalWrite (ledPinRED, LOW);	// turn on the RED LED
                Serial.println("No connection");
            }
        }
        
        /* If connected to MQTT server send the data */
        else if(connectionCounter==12)
        {
            Serial.print("Formatting JSON     ");
            Serial.println(connectionCounter);
            char JSONmessageBuffer[150];
            StaticJsonBuffer<200> JSONbuffer;   /*Declaring static JSON buffer*/
            JsonObject& JSONencoder = JSONbuffer.createObject(); /*creating JSON Object*/
            /*JSON encoders for id,mac and values read by the sensors*/
            JSONencoder["h"] = get_rh();
            JSONencoder["i"] = compute_air_q_average();   
            JSONencoder["a"] = get_iaq_RAM_acc_val();
            JSONencoder["m"] = mac;
            JSONencoder["p"] = get_p();
            // JSONencoder["v"] = get_peak_to_peak_val();
            JSONencoder["s"] = compute_loudness_average()/10;// if using dBa then divide by 10
            JSONencoder["t"] = get_temp();
            JSONencoder["chip id"] = getChipID();
            JSONencoder["lux"]= getLuxResult();
           
            /* Initialize counter for computing averages */
            reset_counter();
            reset_stored_values();
        
            JSONencoder.prettyPrintTo(JSONmessageBuffer, sizeof(JSONmessageBuffer));
            Serial.println(JSONmessageBuffer);
        
            Serial.println("Checking MQTT server connection status.");
            if (client.connected())
            {   
               /*data publishing using JSON*/
               /*JsonArray& firstdev = JSONencoder.createNestedArray("firstdev");
                
                 firstdev.add(get_iaq_RAM_val());
                 client.loop();
                 Serial.println("-------------");
             */
              /* dtostrf(totalInterruptCounter,2,2,pub_str);
                 strcat(pub_str," is the value of interrupt counter");
                 client.publish("gezaTesting",pub_str);*/
             if (client.publish("ESP3", JSONmessageBuffer) == true) {   
                Serial.println("Success sending message ");
                }
                else{
                 Serial.println("Error sending message");
                }
                
            }
            else
            {
                digitalWrite (ledPinRED, LOW);	// turn on the RED LED
                Serial.println("No connection to MQTT server");
            }
        }
        //displaying sensor readings
        if ((totalInterruptCounter%3)==0)
        {
            bsec_iot_op(sleep_BME, totalInterruptCounter, output_ready, state_save, 0, get_timestamp_us);
           
            Serial.print("\nSampling microphone ");
            Serial.println(connectionCounter);
            
            getReading();
            // Add curretn values to sum for average and increase number of values counter
            add_to_stored_values(get_iaq_RAM_val(), get_max_val());
            increase_counter();
            
            Serial.print("Sampling sensor     ");
            Serial.println(connectionCounter);
            Serial.print("Current IAQ value   ");
            Serial.println(get_iaq_RAM_val());

        displayChipID(); //chip id for bme680
    
        Serial.print("OPT3001 Chip Id     ");
        Serial.println(opt3001.readDeviceID());
        Serial.print("OPT3001 Manufacturer ID: ");
	    Serial.println(opt3001.readManufacturerID());

        configureSensor();
        
        Serial.println("\nHigh limit: "); //high lux limit for ambient sensor
        opt3001.readHighLimit();
        Serial.println("\nLow limit: ");  
        opt3001.readLowLimit();
        
        printResult("\nResult: ",result); 
            /*data publishing using char concatenation*/
            /*
            dtostrf(get_iaq_RAM_val(),2,2,IAQ);
            strcat(ClId," sensor 1: ");
            strcat(ClId," , aq:");
            strcat(ClId,IAQ);
            strcat(IAQ," is the value of inside air quality");
            client.publish("gezaTesting",IAQ);
            Serial.print("Loudness value      ");
            Serial.println(get_max_val()/10);
            dtostrf(get_max_val()/10,2,2,LOUDNESS);
            strcat(ClId," l:");
            strcat(ClId,LOUDNESS);
            strcat(LOUDNESS," is the value of Loudness");
            client.publish("gezaTesting",ClId);
            Serial.print("Save flag value     ");
            Serial.println(save);*/
        }
        /* Disconnect from WiFi */
        else if(connectionCounter==15)
        {
            Serial.println("Disonnecting from WiFi.");    
            client.publish("ESP3","Disconnected from Wifi!");
            /* Disconnect from Wi-Fi*/
            WiFi.disconnect();
            Serial.print("Disconnected        ");
            Serial.println(connectionCounter);
        }
    }
}

/*! @}*/

