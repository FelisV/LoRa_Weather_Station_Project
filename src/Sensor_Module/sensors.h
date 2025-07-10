/**
 * @file sensors.h
 * @brief Header file for the Sensors class, which manages various sensors including BMP280, DHT20, and GPS
 * @author
 * @date
 */

#ifndef SENSORS_H
#define SENSORS_H

// extra Libraries
#include <Arduino.h>
#include <Adafruit_BMP280.h>
#include <DHT.h>
#include <TinyGPS++.h>
#include "LoRaWan.h"
#include "I2CScanner.h"

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

/**
 * @class Sensors
 * @brief Sensors class definition
 */
class Sensors {
public:
    /**
     * @brief Constructor for the Sensors class
     * @param dhtType The type of DHT sensor (default is DHT20)
     */
    Sensors() : dht(DHTTYPE) {}

    /**
     * @brief Activate Grove sensors
     */
    void activeGrove();

    /**
     * @brief Deactivate Grove sensors
     */
    void desactiveGrove();

    /*************************************************************************************************************************************************************************************************************************************************** */
    // BMP280

    /**
     * @brief Initialize the BMP280 sensor\n
     * If the sensor is found, it sets the bmp280_initialized flags to true, false otherwise\n
     * @return True if initialization is successful, false otherwise        
     */    
    bool initBMP280();

    /**
     * @brief Configure the BMP280 sensor
     * @note Is necessary set the parameters, otherwise the sensor will not work properly 
     * @param mode The sensor mode
     * @param tempSampling The temperature sampling setting
     * @param pressSampling The pressure sampling setting
     * @param filter The filter setting
     * @param timeStandby The standby duration
     */
    void configureBMP280(Adafruit_BMP280::sensor_mode mode, 
                         Adafruit_BMP280::sensor_sampling tempSampling, 
                         Adafruit_BMP280::sensor_sampling pressSampling, 
                         Adafruit_BMP280::sensor_filter filter, 
                         Adafruit_BMP280::standby_duration timeStandby);

    /**
     * @brief Read BMP280 data and store in variables (bmp280Temp, bmp280Pressure, bmp280Altitude)
     * @note If the sensor is not initialized or if the control measurement register value is 0, it returns false 
     * @param mode The sensor mode
     */
    bool readBMP280(Adafruit_BMP280::sensor_mode mode);

    /**
     * @brief Get the temperature from the BMP280 sensor stored in bmp280Temp variable
     * @return The temperature in degrees Celsius
     */
    float get_BMP280_temp();

    /**
     * @brief Get the pressure from the BMP280 sensor stored in bmp280Pressure variable
     * @return The pressure in Pascals
     */    
    float get_BMP280_pressure();

    /**
     * @brief Get the altitude from the BMP280 sensor stored in bmp280Altitude variable
     * @return The altitude in meters
     */    
    float get_BMP280_altitude();

    /**
     * @brief Get the status of the bmp280_initialized flag
     * @return True if bmp280_initialized flag is true, false if it is false
     */
    bool get_BMP280_initialized();

    /**
     * @brief Get the status of the control measurement register value (BMP280_REGISTER_CONTROL = 0xF4)\n
     * After reading the control measurement register, applies a bitwise AND operation with 0x03 to get the control measurement value\n
     * This value indicates the current measurement mode of the BMP280 sensor:\n
     * - 00: Sleep mode\n
     * - 01 and 10: Forced mode\n
     * - 11: Normal mode
     * @return The value of the control measurement register
     */
    uint8_t get_Ctrl_Meas_Value();

    /**
     * @brief Debug BMP280 values (bmp280Temp, bmp280Pressure, bmp280Altitude)\n
     * This function prints the BMP280 sensor data to the serial monitor\n
     * It includes temperature(°C), pressure(hPa), and altitude(m) values
     */
    void debug_BMP280_data();

    /*************************************************************************************************************************************************************************************************************************************************** */
    // DHT

    /**
     * @brief Initialize the DHT20 sensor\n
     * If the sensor is found, it sets dht_initialized flag to true, false otherwise\n
     * @return True if initialization is successful, false otherwise
     */
    bool initDHT();

    /**
     * @brief Read DHT20 data and store in variables (dhtTemp, dhtHumidity)\n
     * @note When the sensor is not calibrated, it returns false
     * It uses the readTempAndHumidity function to get the values
     * @return True if the values are valid, false otherwise
     */
    bool readDHT();

    /**
     * @brief Get the temperature from the DHT20 sensor stored in dhtTemp variable
     * @return The temperature in degrees Celsius
     */
    float get_DHT_Temp();

    /**
     * @brief Get the humidity from the DHT20 sensor stored in dhtHumidity variable
     * @return The humidity in percentage
     */    
    float get_DHT_Humidity();
    
    /**
     * @brief Get the status of the dht_initialized flag
     * @return True if the dht_initialized flag is true, false if it is false
     */
    bool get_DHT_initialized();

    /**
     * @brief Debug DHT values (dhtTemp, dhtHumidity)\n
     * This function prints the DHT20 sensor data to the serial monitor\n
     * It includes temperature(°C) and humidity(%) values
     */
    void debug_DHT_data();

    /*************************************************************************************************************************************************************************************************************************************************** */
    // GPS
    /**
     * @brief Initialize GPS and LoRaWAN communication\n
     * It initializes the Serial2 for GPS communication and sets the LoRaWAN device reset\n
     * It waits for GPS data for up to 5 seconds\n
     * If GPS data is received, it sets the gps_initialized flag to true, otherwise false\n
     * @return True if initialization is successful, false otherwise
     */
    bool initGPS();

    /**
     * @brief Read GPS data and store in variables (gpsLatitude, gpsLongitude, gpsAltitude, gpsYear, gpsMonth, gpsDay, gpsHour, gpsMinute, gpsSecond)\n
     * This function reads the GPS data from the GPS module and stores it in the respective variables\n
     * It checks if the location and time data are valid before storing them
     * @return True if the GPS data is valid, false otherwise
     */    
    bool readGPS();

    /**
     * @brief Get the latitude from the GPS sensor stored in gpsLatitude variable
     * @return The latitude in degrees
     */
    float get_GPS_Latitude();

    /**
     * @brief Get the longitude from the GPS sensor stored in gpsLongitude variable
     * @return The longitude in degrees
     */
    float get_GPS_Longitude();

    /**
     * @brief Get the altitude from the GPS sensor stored in gpsAltitude variable
     * @return The altitude in meters
     */
    float get_GPS_Altitude();

    /**
     * @brief Get the year from the GPS sensor stored in gpsYear variable
     * @return The year
     */
    uint16_t get_GPS_Year();

    /**
     * @brief Get the month from the GPS sensor stored in gpsMonth variable
     * @return The month
     */
    uint8_t get_GPS_Month();

    /**
     * @brief Get the day from the GPS sensor stored in gpsDay variable
     * @return The day
     */
    uint8_t get_GPS_Day();

    /**
     * @brief Get the hour from the GPS sensor stored in gpsHour variable
     * @return The hour
     */
    uint8_t get_GPS_Hour();

    /**
     * @brief Get the minute from the GPS sensor stored in gpsMinute variable
     * @return The minute
     */
    uint8_t get_GPS_Minute();

    /**
     * @brief Get the second from the GPS sensor stored in gpsSecond variable
     * @return The second
     */
    uint8_t get_GPS_Second();

    /**
     * @brief Get the status of the GPS sensor
     * @return True if the GPS sensor is initialized, false otherwise
     */
    bool get_GPS_initialized();

    /**
     * @brief Get the status of the GPS location
     * @return True if the GPS location is valid, false otherwise
     */
    bool get_GPS_location_isValid();

    /**
     * @brief Get the status of the GPS time validity
     * @return True if the GPS time is valid, false otherwise
     */
    bool get_GPS_time_isValid();

    /**
     * @brief Debug GPS values (gpsLatitude, gpsLongitude, gpsAltitude, gpsYear, gpsMonth, gpsDay, gpsHour, gpsMinute, gpsSecond)\n
     * This function prints the GPS sensor data to the serial monitor\n
     * It includes latitude, longitude, altitude, and date/time values in the format dd/mm/yyyy hh:mm:ss
     */
    void debug_GPS_data();

    /*************************************************************************************************************************************************************************************************************************************************** */
    /**
     * @brief Debug all sensor values (BMP280, DHT20, GPS)\n
     * This function prints the data from all sensors to the serial monitor
     */ 
    void debug_Sensors_data();
    
    /*************************************************************************************************************************************************************************************************************************************************** */
    // Battery Present, Status and Getters

    /**
     * @brief Get the pin number for battery status
     * @return The pin number for battery status
     */
    static int get_Pin_Battery_Status();

    /**
     * @brief Check if the battery is present\n
     * This function reads the analog voltage from the battery using "analogRead"\n
     * if the voltage is above 3.0V, it sets the battery_Present flag to true, false otherwise\n
     * @return True if the battery is present, false otherwise
     */
    bool check_Battery_Present();

    /**
     * @brief Update the battery voltage and charging status\n
     * This function reads the analog voltage from the battery using "analogRead"\n
     * Calculates the actual battery voltage (based on a voltage divider)\n
     * Estimates the battery percentage, and updates the battery_Voltage and battery_Status variables:\n
     * - battery_voltage (0-100)\n
     * - battery_Status (true if charging, false if not charging)
     * @return True if the battery status is updated successfully, false otherwise
     */
    bool update_Battery_Status();

    /**
     * @brief Get the battery voltage from the battery_Voltage variable (0-100)
     * @return The battery voltage as an unsigned char (0-100)
     */
    unsigned char get_Battery_Voltage();

    /**
     * @brief Get the battery charging status from the battery_Status variable
     * @return True if the battery is charging, false if it is not charging
     */
    bool get_Battery_Status();

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
    uint16_t gpsYear;
    uint8_t gpsMonth;
    uint8_t gpsDay;
    uint8_t gpsHour;
    uint8_t gpsMinute;
    uint8_t gpsSecond;

    // Sensor objects
    Adafruit_BMP280 bmp;
    DHT dht;
    TinyGPSPlus gps;

    // sensors flags
    bool bmp280_initialized;
    bool dht_initialized;
    bool gps_initialized;

    // Battery variables
    static const int pin_battery_status  = A5;
    static const int pin_battery_voltage = A4;
    unsigned char battery_Voltage; 
    bool battery_Status;
};

#endif