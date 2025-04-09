/** \file LoRa_Interface.h
 *  \brief LoRa_IF header
 * 
 */

#include <LoRaWan.h>


/**
 * \struct LoRaConfig
 * \brief LoRa Config structure to be used in the \ref InitLoRa(const LoRaConfig *config) function
 * It is used to simplify the data transmission.
 */
struct LoRaConfig {
    char* NwkSKey; /** < The network session key */
    char* AppSKey; /** < The application session key */
    char* AppKey; /** < The application key */
    _device_mode_t mode; /** < The mode of device */
    _data_rate_t dataRate; /** < The date rate of encoding */
    _physical_type_t physicalType; /** < The type of ISM */
    float channelFreq[3]; /** < The frequency values of channels 0, 1 and 3*/
    unsigned char receive1Channel; /** < Set receice window 1 channel mapping channel number, range from 0 to 71*/
    float receive1ChannelFreq; /** < The frequency value of channel */
    float receive2ChannelFreq; /** < Set receice window 2 channel frequency*/
    _data_rate_t receive2ChannelDataRate; /** < The date rate value of window 2 channel*/
    bool dutyCycle; /** < duty cycle limitation */
    short power; /** < The output power value*/
};

/**
 * \class LoRa_IF
 * \brief LoRa Interface Class
 * It is used to simplify the data transmission.
 */
class LoRa_IF 
{
public:
    char buffer[256];

    /**
     * \brief Initialization for the LoRa module
     * \param config Configuration struct
     */
    void InitLoRa(const LoRaConfig *config);

    /**
     * \brief Send Message to the LoRa network
     * \param data Data block to be transmitted 
     * \param length Size in bytes of the data block
     * \param timeout The over time of transfer
     * \return True : transfer done \n False : transfer failed
     */
    bool SendMessage(unsigned char* data, unsigned char length, unsigned char timeout);

    bool SendMessage(char * data, unsigned char timeout);

    short ReceiveMessage(bool result);
};