/**
 * Receive license file and write it to USER DATA area.
 *
 * @copyright Thinnect Inc. 2020
 * @license <PROPRIETARY>
 */

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <inttypes.h>

#include "radio.h"
#include "mist_comm.h"
#include "mist_comm_am.h"
#include "license_rcvr.h"
#include "platform_eui.h"

#include "em_msc.h"
#include "em_se.h"

#include "SignatureArea.h"
#include "DeviceSignature.h"

#include "loglevels.h"
#define __MODUUL__ "licrcvr"
#define __LOG_LEVEL__ (LOG_LEVEL_licRcvr & BASE_LOG_LEVEL)
#include "log.h"
#include "sys_panic.h"

#define AM_LICENSE_FLAG_RESPOND  (1 << 0)
#define AM_LICENSE_FLAG_SENT (1 << 2)
#define AM_LICENSE_FLAG_RCVD (1 << 3)
#define AM_LICENSE_FLAGS (7)

#define FLAGS_TIMEOUT_MS 60000


static void radio_send_done (comms_layer_t * comms, comms_msg_t * msg, comms_error_t result, void * user)
{
    lic_rcvr_t * lic = (lic_rcvr_t*)user;
    logger(result == COMMS_SUCCESS ? LOG_DEBUG1: LOG_WARN1, "snt %d", (int)result);
    osThreadFlagsSet(lic->thread, AM_LICENSE_FLAG_SENT);
}


static void licrcvr_loop (void * arg)
{
    lic_rcvr_t * lic = (lic_rcvr_t*)arg;

    for(;;)
    {
        am_addr_t dest = 0;

        comms_init_message(lic->comms, &(lic->msg));
        am_license_eui_packet_t * packet = comms_get_payload(lic->comms, &(lic->msg), sizeof(am_license_eui_packet_t));

        if (NULL == packet)
        {
            sys_panic("pckt");
        }

        while (osOK != osMutexAcquire(lic->mutex, osWaitForever));

        platform_eui(packet->eui);
        if(0 != lic->respond)
        {
            dest = lic->respond;
            packet->header = EUI64_RESPONSE;
            lic->respond = 0;
        }
        else if(0 != lic->exists)
        {
            dest = lic->exists;
            packet->header = LICFILE_EXISTS;
            lic->exists = 0;
        }
        else if(0 != lic->lic_done)
        {
            int8_t ret;
            uint8_t current_sign[USERDATA_SIZE] = {[0 ... (USERDATA_SIZE-1)] = 0xFF};
            uint16_t signlen = sigGetLength();

            sigAreaRead(0, &current_sign, signlen);
            memcpy(&current_sign[signlen], lic->data, sizeof(lic->data));

            ret = SE_eraseUserData();
            if (ret != SE_RESPONSE_OK)
            {
                err1("!eraseUserData: %i", ret);
            }

            uint8_t rem = (signlen+sizeof(lic->data)) % sizeof(uint32_t);

            ret = SE_writeUserData(0, &current_sign, signlen+sizeof(lic->data)+(sizeof(uint32_t)-rem));
            if (ret != SE_RESPONSE_OK)
            {
                err1("!writeUserData: %i", ret);
            }

            dest = lic->lic_done;
            packet->header = LICFILE_DONE;
            lic->lic_done = 0;
        }

        osMutexRelease(lic->mutex);

        if (0 != dest)
        {
            comms_am_set_destination(lic->comms, &(lic->msg), dest);

            comms_set_packet_type(lic->comms, &(lic->msg), AMID_LICENSE_RCVR);
            comms_set_payload_length(lic->comms, &(lic->msg), sizeof(am_license_eui_packet_t));

            comms_error_t result = comms_send(lic->comms, &(lic->msg), radio_send_done, lic);
            logger(result == COMMS_SUCCESS ? LOG_DEBUG1: LOG_WARN1, "snd %d", (int)result);
            if (COMMS_SUCCESS == result)
            {
                osThreadFlagsWait(AM_LICENSE_FLAG_SENT, osFlagsWaitAny, osWaitForever);
            }

            // Message has been sent, now take a break for a bit
            osDelay(100);
        }
        else
        {
            uint32_t flags;
            flags = osThreadFlagsWait(AM_LICENSE_FLAGS, osFlagsWaitAny, FLAGS_TIMEOUT_MS);
            if (flags == osErrorTimeout)
            {
                debug1("exit thread");
                lic->thread = NULL;
                osThreadExit();
            }
        }
    }
}


static void receive_message (comms_layer_t * comms, const comms_msg_t * msg, void * user)
{
    lic_rcvr_t * lic = (lic_rcvr_t*)user;
    uint8_t buf[128];

    uint8_t length = comms_get_payload_length(comms, msg);
    uint16_t liclen = sigGetLicenseFile(buf);

    if (length >= sizeof(am_license_packet_t))
    {
        am_license_packet_t * packet = (am_license_packet_t*)comms_get_payload(comms, msg, sizeof(uint8_t));
        am_addr_t source = comms_am_get_source(comms, msg);

        switch(packet->header)
        {
            case EUI64_REQUEST:
            {
                const osThreadAttr_t lic_thread_attr = { .name = "license", .stack_size = 3072 };
                if ((lic->thread == NULL) || (osThreadGetState(lic->thread) == osThreadTerminated))
                {
                    lic->thread = osThreadNew(licrcvr_loop, lic, &lic_thread_attr);
                }

                if (liclen == 0)
                {
                    lic->respond = source;
                }
                else
                {
                    lic->exists = source;
                }

                osThreadFlagsSet(lic->thread, AM_LICENSE_FLAG_RESPOND);
                break;
            }

            case LICFILE_RESPONSE:
            {
                uint8_t eui[8];
                platform_eui(eui);

                am_license_file_t * license = (am_license_file_t*)comms_get_payload(comms, msg, sizeof(am_license_file_t));
                if (!memcmp(eui, &license->data[EUI_START_POS], sizeof(eui)))
                {
                    memcpy(lic->data, license->data, length-1);
                    lic->lic_done = source;
                    osThreadFlagsSet(lic->thread, AM_LICENSE_FLAG_RCVD);
                }
                break;
            }

            default:
                warnb1("?", packet, length);
                break;
        }
    }
    else warn1("size %u", (unsigned int)comms_get_payload_length(comms, msg));
}


void license_rcvr_init (comms_layer_t * comms, lic_rcvr_t * lic)
{
    debug1("licrcvr init");
    lic->comms = comms;

    comms_register_recv(comms, &(lic->rcvr), receive_message, lic, AMID_LICENSE_RCVR);

    const osMutexAttr_t app_mutex_attr = { .attr_bits = osMutexPrioInherit };
    lic->mutex = osMutexNew(&app_mutex_attr);
}
