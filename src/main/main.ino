#include <sensors.h>
#include <LoRa_Interface.h>

Sensors sensors;
LoRa_IF lora_interface;
// ----------------------------------- LoRaWAN Configuration ----------------------------
#define APP_EUI "8CF957200007DDBF"
#define DEV_EUI "70B3D57ED006F7D8"
#define APP_KEY "C4C3B34C4DF922628196685A7ABE2125"

unsigned long last_gps_sent_time;

typedef struct 
{
  unsigned short T;
  unsigned short AP;
  unsigned short RH;
  unsigned short LAT;
  unsigned short LON;
} Payload;

void setup() {
  Serial.begin(9600);

  // ----------------------------------- Init Sensors -----------------------------------
  sensors.activeGrove();
  sensors.initBMP280();
  sensors.configureBMP280(Adafruit_BMP280::MODE_NORMAL, 
    Adafruit_BMP280::SAMPLING_X2, 
    Adafruit_BMP280::SAMPLING_X16, 
    Adafruit_BMP280::FILTER_X16, 
    Adafruit_BMP280::STANDBY_MS_1000);
  sensors.initDHT();  
  sensors.initGPS();

  // ------------------------------------------------------------------------------------

  // ----------------------------------- Init LoRa --------------------------------------

    // Create the configuration class
    LoRaConfig conf;

    // Setting keys
    conf.NwkSKey = APP_KEY; conf.AppSKey = APP_KEY, conf.AppKey = APP_KEY;

    // Set mode LWOTAA or LWABP
    conf.mode = LWOTAA;

    // Set dataRate and Physical type
    conf.dataRate = DR0;
    conf.physicalType = EU868;

    // Set channel frequencies
    conf.channelFreq[0] = 868.1;
    conf.channelFreq[1] = 868.3;
    conf.channelFreq[2] = 868.5;

    // Set Reveive Window First
    conf.receive1Channel = 0;
    conf.receive1ChannelFreq = 868.1;

    // Set Reveive Window Second
    conf.receive2ChannelFreq = 869.5;
    conf.receive2ChannelDataRate = DR3;

    // Set the duty cycle limitation
    conf.dutyCycle = false;

    // Set the output power
    conf.power = 14;

    lora_interface.InitLoRa(&conf);

    // ----------------------------------------------------------------------------------

  delay(2000); // 2s to allow the sensors to initialize correctly

}

void loop() {
  // BMP280:
  sensors.readBMP280(Adafruit_BMP280::MODE_NORMAL);
  // DHT20:
  sensors.readDHT();

  Payload payload;

  payload.T = ((sensors.get_BMP280_temp() + sensors.get_DHT_Temp()) / 2) * 1000;

  payload.AP = sensors.get_BMP280_pressure()/10;

  payload.RH = sensors.get_DHT_Humidity() * 100;

  if(millis() - last_gps_sent_time > 20000) 
  {
    last_gps_sent_time = millis();
    payload.LAT = 42.24215 * 100;
    payload.LON = 39.048 * 100;
    lora_interface.SendMessage((unsigned char *)&payload, sizeof(Payload), 10);
  } else 
  {
    lora_interface.SendMessage((unsigned char *)&payload, sizeof(Payload)-4, 10);
  }

  delay(1000);
}