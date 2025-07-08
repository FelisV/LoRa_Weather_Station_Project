/** \file LoRa_Interface.cpp
 *  \brief Source for the LoRa_IF class
 * 
 */

#include "LoRaWan.h"
#include "LoRa_Interface.h"

bool LoRa_IF::OTAAJoin() 
{
    return lora.setOTAAJoin(JOIN, DEFAULT_TIMEOUT);
}

void LoRa_IF::InitLoRa(const LoRaConfig *config) 
{   
    lora.init();
    
    memset(buffer, 0, 256);
    lora.getVersion(buffer, 256, 1);
    SerialUSB.print(buffer); 
    
    memset(buffer, 0, 256);
    lora.getId(buffer, 256, 1);
    SerialUSB.print(buffer);
    
    lora.setKey(config->NwkSKey, config->AppSKey, config->AppKey);
    
    // Set Device mode and Rate
    lora.setDeciveMode(config->mode);
    lora.setDataRate(config->dataRate, config->physicalType);
    
    // Set Channels
    lora.setChannel(0, config->channelFreq[0]);
    lora.setChannel(1, config->channelFreq[1]);
    lora.setChannel(2, config->channelFreq[2]);
    
    // Set Receive Windows 
    lora.setReceiceWindowFirst(config->receive1Channel, config->receive1ChannelFreq);
    lora.setReceiceWindowSecond(config->receive2ChannelFreq, config->receive2ChannelDataRate);
    
    // Disabling duty cycle for testing
    lora.setDutyCycle(config->dutyCycle);
    lora.setJoinDutyCycle(config->dutyCycle);
    
    // Set Output Power Level
    lora.setPower(config->power);
    
    // Block the code until joined
    if(config->mode == LWOTAA) 
    {
      while(!OTAAJoin());
    }
}

bool LoRa_IF::SendMessage(unsigned char* data, unsigned char length, unsigned char timeout) 
{
    return lora.transferPacket(data, length, timeout);
}

bool LoRa_IF::SendMessage(char * data, unsigned char timeout) 
{
    return lora.transferPacket(data, timeout);
}

short LoRa_IF::ReceiveMessage(bool result) 
{
    if(result)
    {
        short length;
        short rssi;
        
        memset(buffer, 0, 256);
        length = lora.receivePacket(buffer, 256, &rssi);
        
        if(length)
        {
            SerialUSB.print("Length is: ");
            SerialUSB.println(length);
            SerialUSB.print("RSSI is: ");
            SerialUSB.println(rssi);
            SerialUSB.print("Data is: ");
            for(unsigned char i = 0; i < length; i ++)
            {
                SerialUSB.print("0x");
                SerialUSB.print(buffer[i], HEX);
                SerialUSB.print(" ");
            }
            SerialUSB.println();
        }

        return length;
    }
}
