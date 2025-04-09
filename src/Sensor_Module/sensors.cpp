#include "sensors.h"

// Activate Grove sensors
void Sensors::activeGrove() {
    digitalWrite(38, HIGH);
}

// Desactivate Grove sensors
void Sensors::desactiveGrove() {
    digitalWrite(38, LOW);
}

/*************************************************************************************************************************************************************************************************************************************************** */
// BMP280
// Initialize the BMP280
void Sensors::initBMP280() 
{
    if (!bmp.begin()) 
    {
        Serial.println("Could not find a valid BMP280 sensor, check wiring.");
        while (1) delay(10);
    }
    Serial.println("BMP280 sensor found.");
    Serial.println("BMP280 sensor initialized.");
}
// Configure the BMP280 
void Sensors::configureBMP280(Adafruit_BMP280::sensor_mode mode, 
    Adafruit_BMP280::sensor_sampling tempSampling, 
    Adafruit_BMP280::sensor_sampling pressSampling, 
    Adafruit_BMP280::sensor_filter filter, 
    Adafruit_BMP280::standby_duration timeStandby)
{
    bmp.setSampling(mode, tempSampling, pressSampling, filter, timeStandby);
    Serial.println("BMP280 sensor configured.");
}

// Read BMP280 data and store in variables (bmp280Temp, bmp280Pressure, bmp280Altitude)
void Sensors::readBMP280(Adafruit_BMP280::sensor_mode mode)
{
    switch (mode)
    {
        case Adafruit_BMP280::MODE_SLEEP:
            Serial.println("Sleep mode.");
            break;
        case Adafruit_BMP280::MODE_FORCED:
            if(!bmp.takeForcedMeasurement()){
                Serial.println("Error in reading BMP280 sensor.");
                break;
            };
            if(!(bmp280Temp = bmp.readTemperature()) || !(bmp280Pressure = bmp.readPressure()) || !(bmp280Altitude = bmp.readAltitude(seaLevelPressure))){
                Serial.println("Error in reading BMP280 sensor.");
                break;
            };

            break;
        case Adafruit_BMP280::MODE_NORMAL:
        if(!(bmp280Temp = bmp.readTemperature()) || !(bmp280Pressure = bmp.readPressure()) || !(bmp280Altitude = bmp.readAltitude(seaLevelPressure))){
            Serial.println("Error in reading BMP280 sensor.");
            break;
        };
            break;
        default:
        Serial.println("Error in reading BMP280 sensor. Please select a valid mode.");
        break;
    }
}

// Get the values from variables (bmp280Temp, bmp280Pressure, bmp280Altitude)
// Return temperatue value in degrees Celsius
float Sensors::get_BMP280_temp(){
    return bmp280Temp;
}
// Return pressure value in Pa
float Sensors::get_BMP280_pressure(){
    return bmp280Pressure;
}
// Return altitude value in meters
float Sensors::get_BMP280_altitude(){
    return bmp280Altitude;
}

// Debug BMP280 values (bmp280Temp, bmp280Pressure, bmp280Altitude)
void Sensors::debug_BMP280_data(){
    Serial.println(F("BMP280 sensor data:"));
    Serial.print(F("Temperature: "));
    Serial.print(get_BMP280_temp());
    Serial.print(F("°C"));
    Serial.print(F(" Pressure: "));
    Serial.print(get_BMP280_pressure()/100);
    Serial.print(F("hPa"));
    Serial.print(F(" Rel. Altitude: "));
    Serial.print(get_BMP280_altitude());
    Serial.print(F("m"));
}

/*************************************************************************************************************************************************************************************************************************************************** */
// DHT
// Initialize DHT20 and I2C communication
void Sensors::initDHT() {
    debug.begin(115200);
    debug.println("DHT20 test!");
    Wire.begin();
    dht.begin();
}

// Array to store temperature and humidity values
float temp_hum_val[2] = {0}; 
// Read DHT20 data and store in variables (dhtTemp, dhtHumidity)
void Sensors::readDHT() {
    dht.readTempAndHumidity(temp_hum_val); // Read temperature and humidity values
    dhtHumidity = temp_hum_val[0]; // Store humidity value
    dhtTemp = temp_hum_val[1]; // Store temperature value
}

// Get the values from variables (dhtTemp, dhtHumidity)
// Return temperature value in degrees Celsius
float Sensors::get_DHT_Temp() {
    return dhtTemp;
}

// Return humidity value in % 
float Sensors::get_DHT_Humidity() {
    return dhtHumidity;
}

// Debug DHT values (dhtTemp, dhtHumidity)
void Sensors::debug_DHT_data() {
  Serial.println(F("DHT20 sensor data:"));
  Serial.print(F("Temperature: "));
  Serial.print(get_DHT_Temp());
  Serial.print(F("°C"));
  Serial.print(F(" Humidity: "));
  Serial.print(get_DHT_Humidity());
  Serial.print(F("%"));
}
/*************************************************************************************************************************************************************************************************************************************************** */
// GPS
// Initialize GPS and LoRaWAN communication
void Sensors::initGPS() {
    #ifdef USE_GPS
    Serial2.begin(9600); // Initialize GPS communication on Serial2
#endif
  lora.init();
  lora.setDeviceReset();
}

// Read GPS data and store in variables (gpsLatitude, gpsLongitude, gpsAltitude, gpsYear, gpsMonth, gpsDay, gpsHour, gpsMinute, gpsSecond)
bool Sensors::readGPS() {
    while (Serial2.available() > 0) {
        char c = Serial2.read();
        gps.encode(c);
    }
    bool gps_location_isValid = gps.location.isValid();
    if (gps_location_isValid) {
        gpsLatitude = gps.location.lat();
        gpsLongitude = gps.location.lng();
        gpsAltitude = gps.altitude.meters();
    }
    if (gps.time.isValid()) {
        gpsYear = gps.date.year();
        gpsMonth = gps.date.month();
        gpsDay = gps.date.day();
        gpsHour = gps.time.hour();
        gpsMinute = gps.time.minute();
        gpsSecond = gps.time.second();
    }
    return gps_location_isValid;
}

// Get the values from variables (gpsLatitude, gpsLongitude, gpsAltitude, gpsYear, gpsMonth, gpsDay, gpsHour, gpsMinute, gpsSecond)
// Return latitude value in degrees
float Sensors::get_GPS_Latitude() {
    return gpsLatitude;
}
// Return longitude value in degrees
float Sensors::get_GPS_Longitude() {
    return gpsLongitude;
}
// Return altitude value in meters
float Sensors::get_GPS_Altitude() {
    return gpsAltitude;
}
// Return value in years
int Sensors::get_GPS_Year() {
    return gpsYear;
}
// Return value in months
int Sensors::get_GPS_Month() {
    return gpsMonth;
}
// Return value in days
int Sensors::get_GPS_Day() {
    return gpsDay;
}
// Return value in hours
int Sensors::get_GPS_Hour() {
    return gpsHour;
}
// Return value in minutes
int Sensors::get_GPS_Minute() {
    return gpsMinute;
}
// Return value in seconds
int Sensors::get_GPS_Second() {
    return gpsSecond;
}

// Debug GPS values (gpsLatitude, gpsLongitude, gpsAltitude, gpsYear, gpsMonth, gpsDay, gpsHour, gpsMinute, gpsSecond)
void Sensors::debug_GPS_data(){
    SerialUSB.print(F("Location: "));
    // Check if stored values are within valid GPS limits
    if (get_GPS_Latitude() >= -90.0 && get_GPS_Latitude() <= 90.0 &&
        get_GPS_Longitude() >= -180.0 && get_GPS_Longitude() <= 180.0) {
        
        SerialUSB.print(get_GPS_Latitude(), 6);
        SerialUSB.print(F(","));
        SerialUSB.print(get_GPS_Longitude(), 6);
        
        SerialUSB.print(F(" Altitude: "));
        if (get_GPS_Altitude() >= -500.0 && get_GPS_Altitude() <= 100000.0) {
            SerialUSB.print(get_GPS_Altitude());
            SerialUSB.print(F("m"));
        } else {
            SerialUSB.print(F("INVALID"));
        }
    }  
    else {
        SerialUSB.print(F("INVALID"));
    }
    // GPS Date and Time in the format dd/mm/yyyy hh:mm:ss.
    SerialUSB.print(F("  Date/Time: "));
    if (get_GPS_Year() >= 2025 && get_GPS_Year() <= 2100 &&
        get_GPS_Month() >= 1 && get_GPS_Month() <= 12 &&
        get_GPS_Day() >= 1 && get_GPS_Day() <= 31 &&
        get_GPS_Hour() >= 0 && get_GPS_Hour() <= 23 &&
        get_GPS_Minute() >= 0 && get_GPS_Minute() <= 59 &&
        get_GPS_Second() >= 0 && get_GPS_Second() <= 59) {
        SerialUSB.print(get_GPS_Day());
        SerialUSB.print(F("/"));
        SerialUSB.print(get_GPS_Month());
        SerialUSB.print(F("/"));
        SerialUSB.print(get_GPS_Year());
        SerialUSB.print(F(" "));
        SerialUSB.print(get_GPS_Hour());
        SerialUSB.print(F(":"));
        SerialUSB.print(get_GPS_Minute());
        SerialUSB.print(F(":"));
        SerialUSB.print(get_GPS_Second());
    }
    else {
        SerialUSB.print(F("INVALID"));
    }
};

/*************************************************************************************************************************************************************************************************************************************************** */
// Debug all sensor values
void Sensors::debug_Sensors_data() {
    debug_BMP280_data();
    Serial.println();
    debug_DHT_data();
    Serial.println();
    debug_GPS_data();
}