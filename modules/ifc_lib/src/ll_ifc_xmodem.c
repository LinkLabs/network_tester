#include "ll_ifc_xmodem.h"

#define TIMEOUT (3) ///< Seconds
#define MAX_ATTEMPTS (16) ///< Max amount of attempts for a send

#define SOH (0x01)  ///< Start of Header
#define EOT (0x04)  ///< End of Transmission
#define ACK (0x06)  ///< Acknowledge
#define NAK (0x15)  ///< Not Acknowledge
#define ETB (0x17)  ///< End of Transmission Block
#define CAN (0x18)  ///< Cancel
#define CCC (0x43)  ///< ASCII “C”

// returns the response of the client
// only if the responce is...
//    * ACK
//    * NAK
//    * CAN
//    * CCC
//    * \n
static int32_t ll_xmodem_get_client_response(uint8_t* response)
{
    struct time time_start;
    gettime(&time_start);

    do
    {
        int32_t ret = transport_read(response, 1);
        if (ret < -1)
        {
            return ret;
        }

        if (ll_difftime_from_now(&time_start) >= TIMEOUT)
        {
            return -2;
        }
    }
    while((ACK != *response) && (NAK != *response) && (CAN != *response) && (CCC != *response) && ('\n' != *response));

    return 0;
}

// Send a XModem packet, returns client response
static int32_t ll_xmodem_send_packet(uint8_t* payload, uint16_t payload_len, uint8_t packet_number)
{
    if (payload_len > 128 || payload == NULL)
    {
        return LL_IFC_ERROR_INCORRECT_PARAMETER;
    }

    // Build XModem Packet
    uint8_t packet[133] = {0x1A}, response = 0;
    uint16_t crc = 0;

    // Initalize Packet with <SUB>
    memset(packet, 0x1A, 133 * sizeof(uint8_t));
    // Start of Header
    packet[0] = SOH;
    // Packet Number
    packet[1] = packet_number & 0xff;
    packet[2] = 0xff - packet_number;
    // Payload
    memcpy(packet + 3 * sizeof(uint8_t), payload, payload_len * sizeof(uint8_t));
    // CRC
    crc = crc16((char *)packet + 3 * sizeof(uint8_t), 128);
    packet[132] = crc & 0xff;
    packet[131] = crc >> 8;

    // Send XModem Packet
    int32_t ret = transport_write(packet, 133);
    if (ret < 0)
    {
        return ret;
    }

    // returns the client response
    ret = ll_xmodem_get_client_response(&response);
    if (ret < 0)
    {
        return ret;
    }

    return response;
}

// send a single byte through xmodem
static int32_t ll_xmodem_send_byte(uint8_t byte)
{
    uint8_t response = 0;
    struct time time_start;
    gettime(&time_start);

    do
    {
        int32_t ret = transport_write(&byte, 1);
        if (ret < 0)
        {
            return ret;
        }

        ret = ll_xmodem_get_client_response(&response);
        if (ret < 0)
        {
            return ret;
        }

        if (ll_difftime_from_now(&time_start) >= TIMEOUT)
        {
            return -2;
        }
    }
    while (ACK != response);

    return response;
}

// returns 0 when the requested response is recieved, negative otherwise
static int32_t ll_xmodem_wait_for(uint8_t response)
{
    struct time time_start;
    uint8_t res = 0;
    gettime(&time_start);

    do
    {
        int32_t ret = ll_xmodem_get_client_response(&res);
        if (ret < 0)
        {
            return ret;
        }

        if (ll_difftime_from_now(&time_start) >= TIMEOUT)
        {
            return -2;
        }

    }
    while(response != res);

    return 0;
}

// get the string that the module bootloader spits out
//
// data - location of the data
// len - length of the allocated string
static int32_t ll_xmodem_parse_module_output(char* data, uint8_t len)
{
    struct time time_start;
    gettime(&time_start);

    uint8_t buf;
    uint8_t i = 0;

    do
    {
        int32_t ret = transport_read(&buf, 1);
        if (ret >= 0 && buf != 0x0 && buf != 0xff && buf != '\n' && buf != '\r')
        {
            data[i++] = (char) buf;
        }

        if (ll_difftime_from_now(&time_start) >= TIMEOUT)
        {
            return -2;
        }
    }
    while (i < len);

    return 0;
}

int32_t ll_xmodem_prepare_module(bool is_host_ifc_active)
{
    // This condition should fail when the module is already in the bootloader
    if(is_host_ifc_active)
    {
        // Put Module in Bootloader Mode
        ll_bootloader_mode();
        sleep_ms(2000);
    }

    // Send upload request to the module
    uint8_t buf = 'u';
    int32_t ret = transport_write(&buf, 1);
    if (ret < 0)
    {
        return ret;
    }

    // Wait for the bootloader to stop talking
    ret = ll_xmodem_wait_for('\n');
    if (ret < 0)
    {
        return ret;
    }

    // Ready for XModem Upload
    return 0;
}

int32_t ll_xmodem_send(ll_xmodem_callbacks_t *cb, uint8_t* payload, size_t payload_len)
{
    if (payload == NULL || payload_len < 1)
    {
        return LL_IFC_ERROR_INCORRECT_PARAMETER;
    }

    if (cb->progress == NULL)
    {
        return -5;
    }

    // Wait for the reciever to send us the start signal.
    int32_t ret = ll_xmodem_wait_for('C');
    if (ret < 0)
    {
        return ret;
    }

    // Start XModem Transfer
    uint32_t offset        = 0; // the amount of bytes sent.
    uint8_t  bytes_to_send = 0; // the amount of bytes to send.
    uint8_t  packet_number = 1; // packet number (starts at 1).
    uint8_t  atmpt_num     = 0; // number of failed packets.

    // Send Data Payload
    do
    {
        bytes_to_send = payload_len - offset > 128 ? 128 : payload_len - offset;
        ret = ll_xmodem_send_packet(payload + offset * sizeof(uint8_t), bytes_to_send, packet_number);
        switch (ret)
        {
            case ACK:
                offset += bytes_to_send;
                packet_number++;
                atmpt_num = 0;
                cb->progress(offset, payload_len);
                break;
            case NAK:
                atmpt_num++;
                if (atmpt_num > MAX_ATTEMPTS)
                {
                    return -4;
                }
                break;
            case CAN:
                return -3;
            case CCC: // ignore start byte
                break;
            default:
                if (ret < 0)
                {
                    return ret; // something broke...
                }
        }
    }
    while (offset < payload_len);

    // Finish Transfer
    ret = ll_xmodem_send_byte(EOT);
    if (ret < 0)
    {
        return ret;
    }

    // Don't sent ETB.

    // Wait 10 seconds while updating the client.
    for (uint8_t i = 0; i < 10; i++)
    {
        sleep_ms(1000);
        cb->progress(offset, payload_len);
    }

    char fw_downloaded[20] = { '\0' };
    ll_xmodem_parse_module_output(fw_downloaded, 19);

    char fw_verify[15] = { '\0' };
    ll_xmodem_parse_module_output(fw_verify, 14);

    char fw_activate[18] = { '\0' };
    ll_xmodem_parse_module_output(fw_activate, 17);

    if (strcmp(fw_downloaded, "Firmware downloaded") != 0)
    {
        return -6;
    }

    if (strcmp(fw_verify, "Verifying...OK") != 0)
    {
        return -7;
    }

    if (strcmp(fw_activate, "Activating...DONE") != 0)
    {
        return -8;
    }

    // Reboot the Module
    uint8_t buf = 'r';
    ret = transport_write(&buf, 1);
    if (ret < 0)
    {
        return ret;
    }

    return 0;
}
