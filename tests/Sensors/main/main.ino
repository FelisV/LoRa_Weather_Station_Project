// Include the sensors.h file which contains the class definition, etc.
#include <sensors.h>

// Instance of the Sensors class
Sensors sensors;

void setup() {
  Serial.begin(115200);
  sensors.activeGrove();
  sensors.initBMP280();
  sensors.configureBMP280(Adafruit_BMP280::MODE_NORMAL, 
    Adafruit_BMP280::SAMPLING_X2, 
    Adafruit_BMP280::SAMPLING_X16, 
    Adafruit_BMP280::FILTER_X16, 
    Adafruit_BMP280::STANDBY_MS_1000);
  sensors.initDHT();  
  sensors.initGPS();
  delay(2000); // 2s to allow the sensors to initialize correctly
}

void loop() {
  // BMP280:
  //sensors.readBMP280(Adafruit_BMP280::MODE_NORMAL);
  //sensors.debug_BMP280_data();
  // DHT20:
  //sensors.readDHT();
  //sensors.debug_DHT_data();
  // GPS:
  sensors.readGPS();
  //sensors.debug_GPS_data();
  // Display sensor data:
  sensors.debug_Sensors_data();
  Serial.println();
  Serial.println();
  delay(1000);  // 1s delay
}