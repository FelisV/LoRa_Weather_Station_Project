#include <LoRaWan.h>

#include <sensors.h>
#include <LoRa_Interface.h>
#include <EnergySaving.h>
#include <RTCInt.h>

// ----------------------------------- LoRaWAN Configuration ----------------------------
#define APP_EUI "8CF957200007DDBF"
#define DEV_EUI "70B3D57ED006F7D8"
#define APP_KEY "C4C3B34C4DF922628196685A7ABE2125"
#define WAIT_TIME 1 // in minutes
#define BATTERYSENDCOOLDOWN 8
#define GPSSENDCOOLDOWN 6

Sensors sensors;
LoRa_IF lora_interface;
EnergySaving nrgSave;
RTCInt rtc;

unsigned long last_gps_sent_time;

typedef struct
{
  unsigned short T;
  unsigned short AP;
  unsigned short RH;
  unsigned short LAT;
  unsigned short LON;
} Payload;

typedef struct
{
  char initData; // BMP280 - DHT20 - GPS - LoRa
} InitPayload;

unsigned char error_flags = 0b10000000;
unsigned char prev_error_flags = 0b10000000;

uint8_t batteryNotSentTimes = 0;
uint8_t gpsNotSentTimes = 0;

bool send_message_setup = false;

void printError(unsigned char val)
{
  for (int i = 7; i >= 0; i--)
  {
    Serial.print((val >> i) & 1);
  }
  Serial.println();
}

void updateGPS()
{
  if (sensors.readGPS())
  {
    error_flags &= ~(1 << 5);
  }
  else
  {
    error_flags |= (1 << 5);
  }
  Serial.println("GPS updating ");
}

void sendGPSData()
{
  float LAT = sensors.get_GPS_Latitude();
  float LON = sensors.get_GPS_Longitude();
  Serial.print(LAT);
  Serial.print("%\n");
  Serial.print(LON);
  Serial.print("%\n");

  float lat_min = -90.0;
  float lat_range = 180.0;
  float lon_min = -180.0;
  float lon_range = 360.0;

  float lat_step = lat_range / (1 << 24); // ≈ 0.0000107°
  float lon_step = lon_range / (1 << 24); // ≈ 0.0000215°

  uint32_t encoded_lat = encodeValue(LAT, lat_min, lat_step);
  uint32_t encoded_lon = encodeValue(LON, lon_min, lon_step);

  uint8_t buffer[6];

  // Latitude (3 bytes)
  buffer[0] = (encoded_lat >> 16) & 0xFF;
  buffer[1] = (encoded_lat >> 8) & 0xFF;
  buffer[2] = encoded_lat & 0xFF;

  // Longitude (3 bytes)
  buffer[3] = (encoded_lon >> 16) & 0xFF;
  buffer[4] = (encoded_lon >> 8) & 0xFF;
  buffer[5] = encoded_lon & 0xFF;
  send_message((unsigned char *)buffer, 6, 1);
  Serial.println("GPS data sent: ");
}

void updateBattery()
{
  if (sensors.update_Battery_Status())
  {
    error_flags &= ~(1 << 6);
  }
  else
  {
    error_flags |= (1 << 6);
  }
  Serial.println("Battery updating ");
}

void sendBattData()
{
  uint8_t battLevel = sensors.get_Battery_Voltage();
  if (battLevel > 127)
  {
    battLevel = 127;
  }
  battLevel = battLevel & 0x7F;
  // Serial.print(battLevel);
  // Serial.print("%\n");
  send_message((unsigned char *)&battLevel, 1, 1);
  Serial.println("Battery data sent: ");
}

void updateSensors()
{
  // BMP280:
  if (sensors.readBMP280(Adafruit_BMP280::MODE_NORMAL))
  {
    error_flags &= ~(1 << 1);
  }
  else
  {
    error_flags |= (1 << 1);
  }
  // DHT20:
  if (sensors.readDHT())
  {
    error_flags &= ~(1 << 3);
  }
  else
  {
    error_flags |= (1 << 3);
  }
  Serial.println("Sensors updating ");
}

void sendSensorData()
{
  float tempSum = 0;
  float tempValues = 0;
  if (!((error_flags >> 1) & 0x01))
  {
    tempSum += sensors.get_BMP280_temp();
    tempValues++;
  }
  if (!((error_flags >> 3) & 0x01))
  {
    tempSum += sensors.get_DHT_Temp();
    tempValues++;
  }
  float tempMean = tempSum / tempValues;

  Serial.println(tempMean);
  Serial.println(sensors.get_BMP280_pressure());
  Serial.println(sensors.get_DHT_Humidity());

  uint32_t tempEncoded = encodeValue(tempMean, -40, 0.5);                              // 8 bits
  uint32_t pressureEncoded = encodeValue(sensors.get_BMP280_pressure() / 100, 300, 1); // 10 bits
  uint32_t rhEncoded = encodeValue(sensors.get_DHT_Humidity(), 0, 3);                  // 6 bits

  Serial.println(tempEncoded);
  Serial.println(pressureEncoded);
  Serial.println(rhEncoded);
  Serial.println("Encoded values hex: ");
  Serial.println(tempEncoded, HEX);
  Serial.println(pressureEncoded, HEX);
  Serial.println(rhEncoded, HEX);
  // Pack: [RH:6][Pressure:10][Temp:8] = total 24 bits
  uint32_t packed = 0;
  packed |= ((uint32_t)rhEncoded & 0x3F) << 18;
  packed |= ((uint32_t)pressureEncoded & 0x3FF) << 8;
  packed |= ((uint32_t)tempEncoded & 0xFF);
  send_message((unsigned char *)&packed, 3, 1);
  Serial.print("Packed data: ");
  Serial.println(packed, HEX);
  Serial.println("Sensor data sent: ");
}

uint32_t encodeValue(float value, float min, float step)
{
  return round((value - min) / step);
}

void send_message(unsigned char *data, unsigned char length, unsigned char timeout)
{
  if (!lora_interface.SendMessage(data, length, timeout))
  {
    lora_interface.OTAAJoin();
  }
  /*Serial.print("Sending message: ");
  for (unsigned char i = 0; i < length; i++) {
    Serial.print(data[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
  */
}

void blink()
{
  for (int i = 0; i < 20; i++)
  {
    digitalWrite(13, HIGH);
    delay(20);
    digitalWrite(13, LOW);
    delay(20);
  }
}

void setup()
{
  Serial.begin(9600);
  Serial.print("Start\n");
  pinMode(13, OUTPUT);

  // ----------------------------------- Init Sensors -----------------------------------
  sensors.activeGrove();
  // BMP 280
  if (sensors.initBMP280())
  {
    sensors.configureBMP280(Adafruit_BMP280::MODE_NORMAL,
                            Adafruit_BMP280::SAMPLING_X1,
                            Adafruit_BMP280::SAMPLING_X1,
                            Adafruit_BMP280::FILTER_X16,
                            Adafruit_BMP280::STANDBY_MS_1000);
  }
  else
  {
    error_flags |= (1 << 0);
  }
  // DHT20
  if (!sensors.initDHT())
  {
    error_flags |= (1 << 2);
  }

  // GPS
  if (!sensors.initGPS())
  {
    error_flags |= (1 << 4);
  };
  //  ----------------------------------- Init Batt --------------------------------------

  pinMode(A5, INPUT);

  // ----------------------------------------------------------------------------------
  // ----------------------------------- Init LoRa --------------------------------------

  // Create the configuration class
  LoRaConfig conf;

  // Setting keys
  conf.NwkSKey = APP_KEY;
  conf.AppSKey = APP_KEY, conf.AppKey = APP_KEY;

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
  // -----------------------------------------------------------------------------------
  delay(30000); // 30s to allow the sensors to initialize correctly
  // ----------------------------------- Init RTC --------------------------------------
  rtc.begin(TIME_H24);
  rtc.setTime(12, 48, 0, 0);
  rtc.setDate(22, 10, 15);
  // rtc.enableAlarm(SEC, ALARM_INTERRUPT, rest_alarm_int);
  rtc.enableAlarm(MMSS, ALARM_INTERRUPT, rest_alarm_int);
  nrgSave.begin(WAKE_RTC_ALARM); // standby setup for external interrupts
  rtc.setAlarm();
  Serial.println("START");
}

void loop()
{

  if (!send_message_setup)
  {
    send_message((unsigned char *)&error_flags, 1, 1);
    send_message_setup = true;
  }
  else
  {
    if (gpsNotSentTimes >= GPSSENDCOOLDOWN)
    {
      gpsNotSentTimes = 0;
      // GPS:
      updateGPS();

      if (error_flags != prev_error_flags)
      {
        send_message((unsigned char *)&error_flags, 1, 1);
        prev_error_flags = error_flags;
      }
      else
      {
        if (error_flags & (1 << 5))
        {
          updateSensors();
          Serial.println("after gps error");
          if (error_flags != prev_error_flags)
          {
            send_message((unsigned char *)&error_flags, 1, 1);
          }
          else
          {
            if ((error_flags & (1 << 1)) && (error_flags & (1 << 3)))
            {
              send_message((unsigned char *)&error_flags, 1, 1);
            }
            else
            {
              sendSensorData();
            }
          }
        }
        else
        {
          sendGPSData();
        }
      }
    }
    else if (batteryNotSentTimes >= BATTERYSENDCOOLDOWN)
    {
      batteryNotSentTimes = 0;
      // BAT:
      updateBattery();

      if (error_flags != prev_error_flags)
      {
        send_message((unsigned char *)&error_flags, 1, 1);
        prev_error_flags = error_flags;
      }
      else
      {
        if (error_flags & (1 << 6))
        {
          updateSensors();
          Serial.println("after batt error");
          if (error_flags != prev_error_flags)
          {
            send_message((unsigned char *)&error_flags, 1, 1);
          }
          else
          {
            if ((error_flags & (1 << 1)) && (error_flags & (1 << 3)))
            {
              send_message((unsigned char *)&error_flags, 1, 1);
            }
            else
            {
              sendSensorData();
            }
          }
        }
        else
        {
          sendBattData();
        }
      }
    }
    else
    {
      // BMP280 - DHT20:
      updateSensors();

      if (error_flags != prev_error_flags)
      {
        send_message((unsigned char *)&error_flags, 1, 1);
        prev_error_flags = error_flags;
      }
      else
      {
        if ((error_flags & (1 << 1)) && (error_flags & (1 << 3)))
        {
          send_message((unsigned char *)&error_flags, 1, 1);
        }
        else
        {
          sendSensorData();
        }
      }
    }
    gpsNotSentTimes++;
    batteryNotSentTimes++;
  }
  blink();
  // delay(1000);
  nrgSave.standby(); // now mcu go to standby
}

void rest_alarm_int(void) // interrupt routine
{
  /*
  unsigned int nextSec = (rtc.getSecond() + WAIT_TIME - 1) % 60;
  rtc.local_time.second = nextSec;
  rtc.setAlarm(); // Load new alarm
  */
  unsigned int nextMin = (rtc.getMinute() + WAIT_TIME) % 60;
  rtc.local_time.minute = nextMin;
  rtc.setAlarm(); // Load new alarm
}