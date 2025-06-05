#include <sensors.h>
// Instance of the Sensors class
Sensors sensors;
unsigned char error_flags = 0;
void printError(unsigned char val) {
  for (int i = 7; i >= 0; i--) {
    Serial.print((val >> i) & 1);
  }
  Serial.println();
}
void setup() {
  Serial.begin(115200);
  sensors.activeGrove();
  if(sensors.initBMP280()){
    sensors.configureBMP280(Adafruit_BMP280::MODE_NORMAL, 
      Adafruit_BMP280::SAMPLING_X2, 
      Adafruit_BMP280::SAMPLING_X16, 
      Adafruit_BMP280::FILTER_X16, 
      Adafruit_BMP280::STANDBY_MS_1000);
  } else {
    error_flags |= (1 << 0);
  }

  if(!sensors.initDHT()){
    error_flags |= (1 << 2);
  }
  if(!sensors.initGPS()){
    error_flags |= (1 << 4);
  };
  //if(!sensors.check_Battery_Present()){
  //  error_flags |= (1 << 6);
  //}
  delay(2000); // 2s to allow the sensors to initialize correctly
}

void loop() {
  error_flags &= ~( (1 << 1) | (1 << 3) | (1 << 5) | (1 << 7));
  //Serial.print("Error Flags  before reading: ");
  //printError(error_flags);
  //BMP280
  if(sensors.readBMP280(Adafruit_BMP280::MODE_NORMAL)){
    sensors.debug_BMP280_data();
  } else {
      error_flags |= (1 << 1);
  }
  Serial.println();
  //DHT20
  if(sensors.readDHT()){
    sensors.debug_DHT_data();
  } else {
    error_flags |= (1 << 3);
  } 
  Serial.println();  
  // GPS:
  if(sensors.readGPS()){
    sensors.debug_GPS_data();
  } else {
    error_flags |= (1 << 5);
  }
  // BAT:
  //if(sensors.get_Battery_Present()){
  //  if(sensors.update_Battery_Status()){
  //    Serial.print("Battery Voltage: ");
  //    Serial.print(sensors.get_Battery_Voltage());
  //    Serial.print("%, Status: ");
  //    Serial.println(sensors.get_Battery_Status() ? "Charging" : "Not Charging");
  //  } else {
  //    error_flags |= (1 << 7);
  //  }
  //}
  Serial.println();
  Serial.print("Error Flags after reading: ");
  printError(error_flags);
  //Serial.print("Error Flags after reading 2: ");
  //Serial.print(error_flags);
  Serial.println();
  delay(1000);  // 1s delay
}