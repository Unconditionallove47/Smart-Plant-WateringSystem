/******************************************************/
//       THIS IS A GENERATED FILE - DO NOT EDIT       //
/******************************************************/

#include "Particle.h"
#line 1 "c:/Users/kalif/Documents/IoT/l14_soilmoisture-Unconditionallove47/L14_04_PlantWater/src/L14_04_PlantWater.ino"
/*
 * Project L14_04_PlantWater
 * Description:Self Watering Plant 
 * Author:Kalif Purce
 * Date:7/12/21
 */

//Pump pin setup
void setup();
void loop();
void printValues();
void testanimate(const uint8_t *bitmap, uint8_t w, uint8_t h);
void OledText(void);
void displaySettings();
void displaySetup();
void MQTT_connect();
void autoWater();
void airQualitySensor();
void dustSensor();
#line 9 "c:/Users/kalif/Documents/IoT/l14_soilmoisture-Unconditionallove47/L14_04_PlantWater/src/L14_04_PlantWater.ino"
const int PUMPPIN=11;
//Pin to read moisture
const int MOISTUREPIN=A3;
//To read moisture value
int moistureValue;
//used for adafruit.io manual water
int manualButton;
//oled and adafruit live time
String DateTime , TimeOnly ;

//timestamps for current and last time
unsigned long timeStamp;    //Timestamp for current time
unsigned long lastStamp;


#include "Air_Quality_Sensor.h"
 AirQualitySensor sensor(A2);

//dust sensor pin and longs for its setup
int dustPin = A1;
unsigned long duration;
unsigned long starttime;
unsigned long sampletime_ms = 30000;//sampe 30s ;
unsigned long lowpulseoccupancy = 0;

//ratio and concentration for dust sensor
float ratio = 0;
float concentration = 0;


#include <Adafruit_MQTT.h>
#include "Adafruit_MQTT/Adafruit_MQTT.h" 
#include "Adafruit_MQTT/Adafruit_MQTT_SPARK.h" 


#include "credentials.h"




//bme library's
  #include <Wire.h>
  #include <SPI.h>
  #include <Adafruit_Sensor.h>
  #include <Adafruit_BME280.h>
  Adafruit_BME280 bme; // I2C
  #define SEALEVELPRESSURE_HPA (1013.25)

//rotation setting for oled, and its defines,library's, etc
  int rot = 0;
#include "Adafruit_GFX.h"
#include "Adafruit_SSD1306.h"
 #define OLED_RESET     4 // Reset pin # (or -1 if sharing Arduino reset pin)
 #define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
 Adafruit_SSD1306 display(OLED_RESET);
 #define NUMFLAKES     10 // Number of snowflakes in the animation example
 #define LOGO_HEIGHT   1
 #define LOGO_WIDTH    1
 #define XPOS   0 // Indexes into the 'icons' array in function below
 #define YPOS   1
 #define DELTAY 2


//adafruit.io settings for publish and sub
TCPClient TheClient; 
Adafruit_MQTT_SPARK mqtt(&TheClient,AIO_SERVER,AIO_SERVERPORT,AIO_USERNAME,AIO_KEY); 
Adafruit_MQTT_Publish TemperatureObject = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/Temperature");
Adafruit_MQTT_Publish HumidityObject = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/Humidity");
Adafruit_MQTT_Publish MoistureObject = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/Soil-Moisture");
Adafruit_MQTT_Publish PressureObject = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/Pressure");
Adafruit_MQTT_Publish AirQualityObject = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/AirQuality");
Adafruit_MQTT_Publish DustObject = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/DustSensor");
Adafruit_MQTT_Subscribe ManualWaterObject = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/ManualWater");
unsigned long last, lastTime;
float tempValue,humidityValue,soilValue,pressureValue,dustValue,airqualityValue;

// setup() runs once, when the device is first turned on.


void setup() {



  //turns bme on,dont forget
bme.begin(0x76);


Serial.begin(9600);

//input setting for dust sensor pin
pinMode(dustPin,INPUT);
    starttime = millis();//get the current time;



//air quality sensor serial monitor test settings
 Serial.println("Waiting sensor to init...");
  delay(10000);
  
  if (sensor.init()) {
    Serial.println("Sensor ready.");
  }
  else {
    Serial.println("Sensor ERROR!");
  }





//moisture pin set to input
pinMode(MOISTUREPIN, INPUT);



//pin for pump set to output
pinMode(PUMPPIN, OUTPUT);


Time . zone ( -7) ; // MST = -7, MDT = -6
Particle . syncTime () ; // Sync time with Particle Cloud

//sets evrything for oled to work
displaySetup();

//sets the style of oled text
OledText();

//subscribe setup for manual water
mqtt.subscribe(&ManualWaterObject);

}


void loop() {


MQTT_connect();
  timeStamp = millis();     // TIMESTAMP TO MILLISECONDS
  DateTime = Time . timeStr () ; // Current Date and Time from Particle Time class
  TimeOnly = DateTime . substring (11 ,19) ;

//connects pump to manual button on dashboard
  analogWrite(PUMPPIN,manualButton);

//BME prints for temp and such
  printValues();

//prints actual text to oled
  displaySettings();

//tests dryness in soil to autowater
  autoWater();

//tests the current air quality
  airQualitySensor();

//checks dust concentration in air
  dustSensor();






//MQTT ping to make sure it still works
  if ((millis()-last)>120000) {
      Serial.printf("Pinging MQTT \n");
    if(! mqtt.ping()) {
      Serial.printf("Disconnecting \n");
      mqtt.disconnect();
    }
  last = millis();
  }


  // publish to cloud every 30 seconds
  tempValue =(bme.readTemperature()) ;
  dustValue=(concentration);
  airqualityValue=(sensor.getValue());
  humidityValue =(bme.readHumidity()) ;
  moistureValue =(analogRead(MOISTUREPIN));
  pressureValue =bme.readAltitude(SEALEVELPRESSURE_HPA);
  if((millis()-lastTime > 15000)) {
    if(mqtt.Update()) {
      TemperatureObject.publish(tempValue);
      AirQualityObject.publish(airqualityValue);
      DustObject.publish(dustValue);
      HumidityObject.publish(humidityValue);
      MoistureObject.publish(moistureValue);
      PressureObject.publish(pressureValue);
      Serial.printf("Publishing %f \n",tempValue); 
    } 
    lastTime = millis();
  }



  // this is our 'wait for incoming subscription packets' busy subloop
  Adafruit_MQTT_Subscribe *subscription;
  while ((subscription = mqtt.readSubscription(1000))) {
    if (subscription == &ManualWaterObject) {
      manualButton = atoi((char *)ManualWaterObject.lastread);
      digitalWrite(PUMPPIN,manualButton);
      Serial.printf("Received %i from Adafruit.io feed soil-moisture \n",manualButton);
    }
  }
}



//function for BME writeouts to serial monitor
void printValues() {
  Serial.print("Temperature = ");
  Serial.print(bme.readTemperature());
  Serial.println(" *C");
  Serial.print("Pressure = ");
  Serial.print(bme.readPressure() / 100.0F);
  Serial.println(" hPa");
  Serial.print("Approx. Altitude = ");
  Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
  Serial.println(" m");
  Serial.print("Humidity = ");
  Serial.print(bme.readHumidity());
  Serial.println(" %");
  Serial.println();
  }



//function to text oled
  void testanimate(const uint8_t *bitmap, uint8_t w, uint8_t h) {
  int8_t f, icons[NUMFLAKES][3];
  // Initialize 'snowflake' positions
    for (f = 0; f < NUMFLAKES; f++) {
    icons[f][XPOS]   = random(1 - LOGO_WIDTH, display.width());
    icons[f][YPOS]   = -LOGO_HEIGHT;
    icons[f][DELTAY] = random(1, 6);
    Serial.print(F("x: "));
    Serial.print(icons[f][XPOS], DEC);
    Serial.print(F(" y: "));
    Serial.print(icons[f][YPOS], DEC);
    Serial.print(F(" dy: "));
    Serial.println(icons[f][DELTAY], DEC);
      }
  }


//function for text style setup for oled 
  void OledText(void) {
  display.clearDisplay();
  display.setTextSize(1);             // Normal 1:1 pixel scale
  display.setTextColor(WHITE);        // Draw white text
  display.setTextSize(1.5);
  display.setCursor(0, 0);            // Start at top-left corner
  display.display();
  }


//function to qrite text to oled
 void displaySettings(){
  display.clearDisplay();
  display.setCursor(0,0);
  display.setTextColor(WHITE,BLACK); // Draw 'inverse' text
  display.setTextSize(1);
  display.setRotation(rot);
  display.printf("Its Se%cor KalifPurce\n",0xA4);
  display. printf (" Date and time is %s\n", DateTime . c_str () );
  display.printf("Time is %s\n" , TimeOnly.c_str());
  display.display();
  }



//function that makes sure oled is functioning with test
 void displaySetup(){
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.display();
  display.drawPixel(100, 100, WHITE);
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.display();
  // Invert and restore display, pausing in-between
  display.invertDisplay(true);
  // delay(1000);
  display.invertDisplay(false);
  // delay(1000);
  }



//connects MQTT automatically using function
void MQTT_connect() {
  int8_t ret;
  // Stop if already connected.
  if (mqtt.connected()) {
    return;
   }
  Serial.print("Connecting to MQTT... ");
     while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
      Serial.printf("%s\n",(char *)mqtt.connectErrorString(ret));
      Serial.printf("Retrying MQTT connection in 5 seconds..\n");
      mqtt.disconnect();
      delay(5000);  // wait 5 seconds
     }
  Serial.printf("MQTT Connected!\n");
} 




//function to auto water plant
void autoWater() {
  if(moistureValue > 2700) {
    digitalWrite(PUMPPIN,HIGH);
    delay(2000);
    digitalWrite(PUMPPIN,LOW);
    delay(1000);
  }
  else {
    digitalWrite(PUMPPIN,LOW);
    delay(1000); 
  }
}


//function for the air quality sensor
void airQualitySensor(){
int quality = sensor.slope();

  Serial.print("Sensor value: ");
  Serial.println(sensor.getValue());
  
  if (quality == AirQualitySensor::FORCE_SIGNAL) {
    Serial.println("High pollution! Force signal active.");
  }
  else if (quality == AirQualitySensor::HIGH_POLLUTION) {
    Serial.println("High pollution!");
  }
  else if (quality == AirQualitySensor::LOW_POLLUTION) {
    Serial.println("Low pollution!");
  }
  else if (quality == AirQualitySensor::FRESH_AIR) {
    Serial.println("Fresh air.");
  }
  
  delay(1000);


}


//function for dust sensor
void dustSensor(){
duration = pulseIn(dustPin, LOW);
    lowpulseoccupancy = lowpulseoccupancy+duration;
 
    if ((millis()-starttime) > sampletime_ms)//if the sampel time == 30s
    {
        ratio = lowpulseoccupancy/(sampletime_ms*10.0);  // Integer percentage 0=>100
        concentration = 1.1*pow(ratio,3)-3.8*pow(ratio,2)+520*ratio+0.62; // using spec sheet curve
        Serial.print(lowpulseoccupancy);
        Serial.print(",");
        Serial.print(ratio);
        Serial.print(",");
        Serial.println(concentration);
        lowpulseoccupancy = 0;
        starttime = millis();
    }

}