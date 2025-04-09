#include "LoRa_Interface.h"

unsigned char test_data[10] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 0xA};

LoRa_IF lora_interface;

#define APP_EUI "8CF957200007DDBF"
#define DEV_EUI "70B3D57ED006F7D8"
#define APP_KEY "C4C3B34C4DF922628196685A7ABE2125"

void setup(void)
{
    // ----------------------------------- Setting the config parameters -----------------------------------

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
    conf.dutyCycle = true;

    // Set the output power
    conf.power = 14;

    // ----------------------------------------------------------------------------------------------------

    lora_interface.InitLoRa(&conf);
}


void loop(void)
{   
    lora_interface.SendMessage("Hello World!", 10);
    lora_interface.SendMessage(test_data, sizeof(char) * 10, 10);
}
