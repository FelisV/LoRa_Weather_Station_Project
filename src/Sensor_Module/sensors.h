#ifndef SENSORS_H
#define SENSORS_H

// extra Libraries
#include <Adafruit_BMP280.h>
#include <DHT.h>
#include <TinyGPS++.h>
#include "LoRaWan.h"

// Definition of the DHT sensor type
#define DHTTYPE DHT20

// Define USE_GPS to activate GPS sensor
#define USE_GPS 1
#ifdef USE_GPS
#endif

// Definition of "debug" for different board architectures
#if defined(ARDUINO_ARCH_AVR)
    #define debug  Serial
#elif defined(ARDUINO_ARCH_SAMD) ||  defined(ARDUINO_ARCH_SAM)
    #define debug  SerialUSB
#else
    #define debug  Serial
#endif

// Sensors class definition
class Sensors {
public:
    // Constructor for the Sensors class with the DHT sensor type
    Sensors() : dht(DHTTYPE) {}

    // Grove sensors activation
    void activeGrove();

    // Grove sensors desactivation
    void desactiveGrove();

    /*************************************************************************************************************************************************************************************************************************************************** */
    // BMP280
    // Initialize the BMP280
    void initBMP280();

    // Configure the BMP280
    void configureBMP280(Adafruit_BMP280::sensor_mode mode, 
                         Adafruit_BMP280::sensor_sampling tempSampling, 
                         Adafruit_BMP280::sensor_sampling pressSampling, 
                         Adafruit_BMP280::sensor_filter filter, 
                         Adafruit_BMP280::standby_duration timeStandby);

    // Read BMP280 data and store in variables (bmp280Temp, bmp280Pressure, bmp280Altitude)
    void readBMP280(Adafruit_BMP280::sensor_mode mode);

    // Get the values from variables (bmp280Temp, bmp280Pressure, bmp280Altitude)
    float get_BMP280_temp();
    float get_BMP280_pressure();
    float get_BMP280_altitude();

    // Debug BMP280 values (bmp280Temp, bmp280Pressure, bmp280Altitude)
    void debug_BMP280_data();

    /*************************************************************************************************************************************************************************************************************************************************** */
    // DHT
    // Initialize the DHT20
    void initDHT();

    // Read DHT20 data and store in variables (dhtTemp, dhtHumidity)
    void readDHT();

    // Get the values from variables (dhtTemp, dhtHumidity)
    float get_DHT_Temp();
    float get_DHT_Humidity();

    // Debug DHT values (dhtTemp, dhtHumidity)
    void debug_DHT_data();

    /*************************************************************************************************************************************************************************************************************************************************** */
    // GPS
    // Initialize the GPS sensor
    void initGPS();

    // Read GPS data and store in variables (gpsLatitude, gpsLongitude, gpsAltitude, gpsYear, gpsMonth, gpsDay, gpsHour, gpsMinute, gpsSecond)
    bool readGPS();

    // Get the values from variables (gpsLatitude, gpsLongitude, gpsAltitude, gpsYear, gpsMonth, gpsDay, gpsHour, gpsMinute, gpsSecond)
    float get_GPS_Latitude();
    float get_GPS_Longitude();
    float get_GPS_Altitude();

    int get_GPS_Year();
    int get_GPS_Month();
    int get_GPS_Day();

    int get_GPS_Hour();
    int get_GPS_Minute();
    int get_GPS_Second();

    // Debug GPS values (gpsLatitude, gpsLongitude, gpsAltitude, gpsYear, gpsMonth, gpsDay, gpsHour, gpsMinute, gpsSecond)
    void debug_GPS_data();

    /*************************************************************************************************************************************************************************************************************************************************** */
    // Debug all sensor values 
    void debug_Sensors_data();

private:
    // BMP280 variables
    float bmp280Temp; // Temperature in degrees Celsius
    float bmp280Pressure; // Pressure in Pa
    float bmp280Altitude; // Altitude in meters
    float seaLevelPressure = 1013.25; // Default sea level pressure in hPa

    // DHT variables
    float dhtTemp; // Temperature in degrees Celsius
    float dhtHumidity; // Humidity in %

    // GPS variables
    float gpsLatitude; 
    float gpsLongitude;
    float gpsAltitude;
    int gpsYear;
    int gpsMonth;
    int gpsDay;
    int gpsHour;
    int gpsMinute;
    int gpsSecond;

    // Sensor objects
    Adafruit_BMP280 bmp;
    DHT dht;
    TinyGPSPlus gps;
};

#endif