#include "sensors.h"

void Sensors::activeGrove() {
    digitalWrite(38, HIGH);
}

void Sensors::desactiveGrove() {
    digitalWrite(38, LOW);
}

/*************************************************************************************************************************************************************************************************************************************************** */
// BMP280

bool Sensors::initBMP280() 
{
    if (!bmp.begin()) 
    {
        bmp280_initialized = false;
        return false;
    } else {
    bmp280_initialized = true;
    return true;
    }
}

void Sensors::configureBMP280(Adafruit_BMP280::sensor_mode mode, 
    Adafruit_BMP280::sensor_sampling tempSampling, 
    Adafruit_BMP280::sensor_sampling pressSampling, 
    Adafruit_BMP280::sensor_filter filter, 
    Adafruit_BMP280::standby_duration timeStandby)
{
    bmp.setSampling(mode, tempSampling, pressSampling, filter, timeStandby);
    Serial.println("BMP280 sensor configured.");
}

bool Sensors::readBMP280(Adafruit_BMP280::sensor_mode mode)
{
    if(get_BMP280_initialized() == false) {
        return false;
    }

    if(get_Ctrl_Meas_Value() == 0) {
        return false;
    }

    bool success = true;

    switch (mode)
    {
        case Adafruit_BMP280::MODE_SLEEP:
            Serial.println("Sleep mode.");
            success = false;
            break;

        case Adafruit_BMP280::MODE_FORCED:
            if (!bmp.takeForcedMeasurement()) {
                Serial.println("Error in reading BMP280 sensor.");
                success = false;
                break;
            }

        case Adafruit_BMP280::MODE_NORMAL:

            bmp280Temp = bmp.readTemperature();
            bmp280Pressure = bmp.readPressure();
            bmp280Altitude = bmp.readAltitude(seaLevelPressure);

            if (isnan(bmp280Temp) || isnan(bmp280Pressure) || isnan(bmp280Altitude)) {
                Serial.println("Error in reading BMP280 sensor.");
                success = false;
                break;
            }

            if (bmp280Temp < -40 || bmp280Temp > 85) {
                Serial.println("Temperature out of range.");
                success = false;
                break;
            }

            if (bmp280Pressure < 30000 || bmp280Pressure > 110000) {
                Serial.println("Pressure out of range.");
                success = false;
                break;
            }

            break;

        default:
            Serial.println("Error in reading BMP280 sensor. Please select a valid mode.");
            success = false;
            break;
    }

    return success;
}

float Sensors::get_BMP280_temp(){
    return bmp280Temp;
}

float Sensors::get_BMP280_pressure(){
    return bmp280Pressure;
}

float Sensors::get_BMP280_altitude(){
    return bmp280Altitude;
}

bool Sensors::get_BMP280_initialized() {
    return bmp280_initialized;
}

uint8_t Sensors::get_Ctrl_Meas_Value() {
    return bmp.read8(BMP280_REGISTER_CONTROL) & 0x03;
}

// Debug BMP280 values (bmp280Temp, bmp280Pressure, bmp280Altitude)
void Sensors::debug_BMP280_data(){
    Serial.println(F("BMP280 sensor data:"));
    Serial.print(F("Temperature: "));
    Serial.print(get_BMP280_temp());
    Serial.print(F("°C"));
    Serial.print(F(" Pressure: "));
    Serial.print(get_BMP280_pressure()/100); // Convert Pa to hPa
    Serial.print(F("hPa"));
    Serial.print(F(" Rel. Altitude: "));
    Serial.print(get_BMP280_altitude());
    Serial.print(F("m"));
}

/*************************************************************************************************************************************************************************************************************************************************** */
// DHT

bool Sensors::initDHT() {
    Wire.begin();
    if(dht.begin()){

        dht_initialized = true;
        return true;
    } else {
        dht_initialized = false;
        return false;
    }
}

// Array to store temperature and humidity values
float temp_hum_val[2] = {0}; 

bool Sensors::readDHT() {
    if(get_DHT_initialized() == false) {
        return false;
    }
    if(dht.readTempAndHumidity(temp_hum_val) == 0) { // Read temperature and humidity values
        dhtHumidity = temp_hum_val[0]; // Store humidity value
        dhtTemp = temp_hum_val[1]; // Store temperature value
        if (dhtHumidity >= 0 && dhtHumidity <= 100 && dhtTemp >= -40 && dhtTemp <= 80) {
            return true;
        }
    }
    Serial.println("Error in reading DHT20 sensor.");
    return false;
}

float Sensors::get_DHT_Temp() {
    return dhtTemp;
}

float Sensors::get_DHT_Humidity() {
    return dhtHumidity;
}

bool Sensors::get_DHT_initialized() {
    return dht_initialized;
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

bool Sensors::initGPS() {
    #ifdef USE_GPS
    Serial2.begin(9600); // Initialize GPS communication on Serial2
#endif
  lora.init();
  lora.setDeviceReset();
#ifdef USE_GPS
    unsigned long start = millis();
    char c;
    // If GPS data is received within 5 seconds, it will be considered initialized
    while (millis() - start < 5000) {
        while (Serial2.available() > 0) {
            c = Serial2.read();
            gps.encode(c);
        }
        if (gps.charsProcessed() >= 10) {
            gps_initialized = true;
            return true;
        }
    }
    SerialUSB.println(F("No GPS detected: check wiring."));
    SerialUSB.println(gps.charsProcessed());
    gps_initialized = false;
    return false;
#else
    gps_initialized = false;
    return false;
#endif
}

bool Sensors::get_GPS_initialized() {
    return gps_initialized;
}

bool Sensors::readGPS() {
    if(get_GPS_initialized() == false) {
        return false;
    }
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

float Sensors::get_GPS_Latitude() {
    return gpsLatitude;
}

float Sensors::get_GPS_Longitude() {
    return gpsLongitude;
}

float Sensors::get_GPS_Altitude() {
    return gpsAltitude;
}

uint16_t Sensors::get_GPS_Year() {
    return gpsYear;
}

uint8_t Sensors::get_GPS_Month() {
    return gpsMonth;
}

uint8_t Sensors::get_GPS_Day() {
    return gpsDay;
}

uint8_t Sensors::get_GPS_Hour() {
    return gpsHour;
}

uint8_t Sensors::get_GPS_Minute() {
    return gpsMinute;
}

uint8_t Sensors::get_GPS_Second() {
    return gpsSecond;
}

bool Sensors::get_GPS_location_isValid() {
    return gps.location.isValid();
}
bool Sensors::get_GPS_time_isValid() {
    return gps.time.isValid();
}

void Sensors::debug_GPS_data(){
    SerialUSB.print(F("Location: "));
    // Check if stored values are within valid GPS limits
    if (get_GPS_Latitude() >= -90.0 && get_GPS_Latitude() <= 90.0 &&
        get_GPS_Longitude() >= -180.0 && get_GPS_Longitude() <= 180.0) {
        
        SerialUSB.print(get_GPS_Latitude(), 6);
        SerialUSB.print(F(","));
        SerialUSB.print(get_GPS_Longitude(), 6);
        
        SerialUSB.print(F(" Altitude: "));
        if (get_GPS_Altitude() >= -50.0 && get_GPS_Altitude() <= 18000.0) {
            SerialUSB.print(get_GPS_Altitude());
            SerialUSB.print(F("m"));
        } else {
            SerialUSB.print(F("INVALID"));
        }
    }  
    else {
        SerialUSB.print(F("INVALID"));
    }
    // GPS Date and Time in the format dd/mm/yyyy hh:mm:ss
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

void Sensors::debug_Sensors_data() {
    debug_BMP280_data();
    Serial.println();
    debug_DHT_data();
    Serial.println();
    debug_GPS_data();
}

/*************************************************************************************************************************************************************************************************************************************************** */

int Sensors::get_Pin_Battery_Status() {
    return Sensors::pin_battery_status;
}

bool Sensors::check_Battery_Present() {
    int a = analogRead(pin_battery_voltage);
    float v = a / 1023.0 * 3.3 * 11;
    if(v > 3.0){
        return true;
    }
    return false;
}

bool Sensors::update_Battery_Status() {
    int a = analogRead(pin_battery_voltage);
    float v = a / 1023.0*3.3*11.0;
    float percent = (v - 3.0) / (4.2 - 3.0) * 100;
    battery_Voltage = static_cast<unsigned char>(round(percent));
    //battery_Voltage = v;
    battery_Status = digitalRead(pin_battery_status);
    if(battery_Voltage > 1) {
        return true;
    }
        return false;
}

unsigned char Sensors::get_Battery_Voltage() {
    return battery_Voltage;
}

bool Sensors::get_Battery_Status() {
    return battery_Status;
}