/*
 * Copyright (C) 2014 BlueKitchen GmbH
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holders nor the names of
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 * 4. Any redistribution, use, or modification is done solely for
 *    personal benefit and not for any commercial purpose or for
 *    monetary gain.
 *
 * THIS SOFTWARE IS PROVIDED BY BLUEKITCHEN GMBH AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL BLUEKITCHEN
 * GMBH OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * Please inquire about commercial licensing options at
 * contact@bluekitchen-gmbh.com
 *
 */

#define BTSTACK_FILE__ "gatt_dice.c"

// *****************************************************************************
/* EXAMPLE_START(gatt_dice): GATT Server - Heartbeat Counter over GATT
 *
 * @text All newer operating systems provide GATT Client functionality.
 * The LE Counter examples demonstrates how to specify a minimal GATT Database
 * with a custom GATT Service and a custom Characteristic that sends periodic
 * notifications.
 */
// *****************************************************************************

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ble/gatt-service/battery_service_server.h"
#include "btstack.h"
#include "esp_random.h"
#include "gatt_dice.h"

#define HEARTBEAT_PERIOD_MS 2500

/* @section Main Application Setup
 *
 * @text Listing MainConfiguration shows main application code.
 * It initializes L2CAP, the Security Manager and configures the ATT Server with the
 * pre-compiled ATT Database generated from $le_dice.gatt$. Additionally, it enables
 * the Battery Service Server with the current battery level. Finally, it configures the
 * advertisements and the heartbeat handler and boots the Bluetooth stack. In this
 * example, the Advertisement contains the Flags attribute and the device name. The flag
 * 0x06 indicates: LE General Discoverable Mode and BR/EDR not supported.
 */

/* LISTING_START(MainConfiguration): Init L2CAP SM ATT Server and start heartbeat timer
 */
static int le_notification_enabled;
static btstack_timer_source_t heartbeat;
static btstack_packet_callback_registration_t hci_event_callback_registration;
static btstack_packet_callback_registration_t sm_event_callback_registration;
static hci_con_handle_t con_handle;
static uint8_t battery = 100;

#define ENABLE_GATT_OVER_CLASSIC

#ifdef ENABLE_GATT_OVER_CLASSIC
static uint8_t gatt_service_buffer[70];
#endif

static void hci_packet_handler(
    uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size
);
static void sm_packet_handler(
    uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size
);
static uint16_t att_read_callback(
    hci_con_handle_t con_handle, uint16_t att_handle, uint16_t offset, uint8_t *buffer,
    uint16_t buffer_size
);
static int att_write_callback(
    hci_con_handle_t con_handle, uint16_t att_handle, uint16_t transaction_mode,
    uint16_t offset, uint8_t *buffer, uint16_t buffer_size
);
static void heartbeat_handler(struct btstack_timer_source *ts);
static void beat(void);

// Flags general discoverable, BR/EDR supported (== not supported flag not set) when
// ENABLE_GATT_OVER_CLASSIC is enabled
#ifdef ENABLE_GATT_OVER_CLASSIC
#define APP_AD_FLAGS 0x02
#else
#define APP_AD_FLAGS 0x06
#endif

const uint8_t adv_data[] = {
    // Flags general discoverable
    0x02,
    BLUETOOTH_DATA_TYPE_FLAGS,
    APP_AD_FLAGS,
    // Name
    0x09,
    BLUETOOTH_DATA_TYPE_COMPLETE_LOCAL_NAME,
    'E',
    'S',
    'P',
    ' ',
    'd',
    'i',
    'c',
    'e',
    // Complete List of 128-bit Service Class UUIDs - Dice Service
    0x11,
    BLUETOOTH_DATA_TYPE_INCOMPLETE_LIST_OF_128_BIT_SERVICE_CLASS_UUIDS,
    0xE3,
    0xAB,
    0xB8,
    0x27,
    0x91,
    0x38,
    0xAB,
    0xA2,
    0xCA,
    0x47,
    0x9B,
    0xC4,
    0x2E,
    0x08,
    0x6A,
    0xE5,
};
const uint8_t adv_data_len = sizeof(adv_data);

static void le_dice_setup(void) {

#ifdef ENABLE_CROSS_TRANSPORT_KEY_DERIVATION
    printf("Cross transport key derivavtion is enabled\n");
#endif

    l2cap_init();

    // setup SM: Display only
    sm_init();

#ifdef ENABLE_LE_SECURE_CONNECTIONS
    printf("Require LE secure connections\n");
    sm_set_secure_connections_only_mode(true);
    sm_set_io_capabilities(IO_CAPABILITY_NO_INPUT_NO_OUTPUT);
    sm_set_authentication_requirements(
        SM_AUTHREQ_SECURE_CONNECTION | SM_AUTHREQ_MITM_PROTECTION
    );
#endif

#ifdef ENABLE_GATT_OVER_CLASSIC
    // init SDP, create record for GATT and register with SDP
    sdp_init();
    memset(gatt_service_buffer, 0, sizeof(gatt_service_buffer));
    gatt_create_sdp_record(
        gatt_service_buffer, sdp_create_service_record_handle(),
        ATT_SERVICE_GATT_SERVICE_START_HANDLE, ATT_SERVICE_GATT_SERVICE_END_HANDLE
    );
    btstack_assert(de_get_len(gatt_service_buffer) <= sizeof(gatt_service_buffer));
    sdp_register_service(gatt_service_buffer);

    // configure Classic GAP
    gap_set_local_name("ESP dice");
    gap_ssp_set_io_capability(SSP_IO_CAPABILITY_DISPLAY_YES_NO);
    gap_discoverable_control(1);
#endif

    // setup ATT server
    att_server_init(profile_data, att_read_callback, att_write_callback);

    // setup battery service
    battery_service_server_init(battery);

    // setup advertisements
    uint16_t adv_int_min = 0x0030;
    uint16_t adv_int_max = 0x0030;
    uint8_t adv_type = 0;
    bd_addr_t null_addr;
    memset(null_addr, 0, 6);
    gap_advertisements_set_params(
        adv_int_min, adv_int_max, adv_type, 0, null_addr, 0x07, 0x00
    );
    gap_advertisements_set_data(adv_data_len, (uint8_t *)adv_data);
    gap_advertisements_enable(1);

    printf("Advertising %d bytes\n", adv_data_len);

    // register for HCI events
    hci_event_callback_registration.callback = &hci_packet_handler;
    hci_add_event_handler(&hci_event_callback_registration);

    // register for SM events
    sm_event_callback_registration.callback = &sm_packet_handler;
    sm_add_event_handler(&sm_event_callback_registration);

    // register for ATT event
    att_server_register_packet_handler(hci_packet_handler);

    // set one-shot timer
    heartbeat.process = &heartbeat_handler;
    btstack_run_loop_set_timer(&heartbeat, HEARTBEAT_PERIOD_MS);
    btstack_run_loop_add_timer(&heartbeat);

    // beat once
    beat();
}
/* LISTING_END */

/*
 * @section Heartbeat Handler
 *
 * @text The heartbeat handler updates the value of the single Characteristic provided
 * in this example, and request a ATT_EVENT_CAN_SEND_NOW to send a notification if
 * enabled see Listing heartbeat.
 */

/* LISTING_START(heartbeat): Hearbeat Handler */
static int dice = 0;
static char dice_string[2];
static int dice_string_len;

static void beat(void) {
    dice = (esp_random() % 6) + 1;
    dice_string_len = snprintf(dice_string, sizeof(dice_string), "%u", dice);
    printf("Current dice roll: %s\n", dice_string);
}

static void heartbeat_handler(struct btstack_timer_source *ts) {
    if (le_notification_enabled) {
        beat();
        att_server_request_can_send_now_event(con_handle);
    }

    // simulate battery drain
    battery--;
    if (battery < 50) {
        battery = 100;
    }
    battery_service_server_set_battery_value(battery);

    btstack_run_loop_set_timer(ts, HEARTBEAT_PERIOD_MS);
    btstack_run_loop_add_timer(ts);
}
/* LISTING_END */

/*
 * @section Packet Handler
 *
 * @text The packet handler is used to:
 *        - stop the dice after a disconnect
 *        - send a notification when the requested ATT_EVENT_CAN_SEND_NOW is received
 */

/* LISTING_START(packetHandler): Packet Handler */
static void hci_packet_handler(
    uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size
) {
    UNUSED(channel);
    UNUSED(size);

    if (packet_type != HCI_EVENT_PACKET)
        return;

    switch (hci_event_packet_get_type(packet)) {
    case HCI_EVENT_DISCONNECTION_COMPLETE:
        le_notification_enabled = 0;
        break;
    case ATT_EVENT_CAN_SEND_NOW:
        att_server_notify(
            con_handle, ATT_CHARACTERISTIC_FF3F_01_VALUE_HANDLE, (uint8_t *)dice_string,
            dice_string_len
        );
        break;
    default:
        break;
    }
}

/* LISTING_END */

/*
 * @section Security Manager Packet Handler
 *
 * @text The packet handler is used to handle Security Manager events
 */

/* LISTING_START(packetHandler): Security Manager Packet Handler */
static void sm_packet_handler(
    uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size
) {
    UNUSED(channel);
    UNUSED(size);

    if (packet_type != HCI_EVENT_PACKET)
        return;

    hci_con_handle_t con_handle;
    bd_addr_t addr;
    bd_addr_type_t addr_type;
    uint8_t status;

    switch (hci_event_packet_get_type(packet)) {
    case HCI_EVENT_META_GAP:
        switch (hci_event_gap_meta_get_subevent_code(packet)) {
        case GAP_SUBEVENT_LE_CONNECTION_COMPLETE:
            printf("Connection complete\n");
            con_handle =
                gap_subevent_le_connection_complete_get_connection_handle(packet);
            UNUSED(con_handle);

            // for testing, choose one of the following actions

            // manually start pairing
            // sm_request_pairing(con_handle);

            // gatt client request to authenticated characteristic in sm_pairing_central
            // (short cut, uses hard-coded value handle)
            // gatt_client_read_value_of_characteristic_using_value_handle(&packet_handler,
            // con_handle, 0x0009);

            // general gatt client request to trigger mandatory authentication
            // gatt_client_discover_primary_services(&packet_handler, con_handle);
            break;
        default:
            break;
        }
        break;
    case SM_EVENT_JUST_WORKS_REQUEST:
        printf("Just Works requested\n");
        sm_just_works_confirm(sm_event_just_works_request_get_handle(packet));
        break;
    case SM_EVENT_NUMERIC_COMPARISON_REQUEST:
        printf(
            "Confirming numeric comparison: %lu\n",
            sm_event_numeric_comparison_request_get_passkey(packet)
        );
        sm_numeric_comparison_confirm(sm_event_passkey_display_number_get_handle(packet)
        );
        break;
    case SM_EVENT_PASSKEY_DISPLAY_NUMBER:
        printf(
            "Display Passkey: %lu\n",
            sm_event_passkey_display_number_get_passkey(packet)
        );
        break;
    case SM_EVENT_IDENTITY_CREATED:
        sm_event_identity_created_get_identity_address(packet, addr);
        printf(
            "Identity created: type %u address %s\n",
            sm_event_identity_created_get_identity_addr_type(packet),
            bd_addr_to_str(addr)
        );
        break;
    case SM_EVENT_IDENTITY_RESOLVING_STARTED:
        sm_event_identity_resolving_started_get_address(packet, addr);
        printf(
            "Identity resolving started type: %u address %s\n",
            sm_event_identity_resolving_started_get_addr_type(packet),
            bd_addr_to_str(addr)
        );
        break;
    case SM_EVENT_IDENTITY_RESOLVING_SUCCEEDED:
        sm_event_identity_resolving_succeeded_get_identity_address(packet, addr);
        printf(
            "Identity resolved: type %u address %s\n",
            sm_event_identity_resolving_succeeded_get_identity_addr_type(packet),
            bd_addr_to_str(addr)
        );
        break;
    case SM_EVENT_IDENTITY_RESOLVING_FAILED:
        sm_event_identity_created_get_address(packet, addr);
        printf("Identity resolving failed\n");
        break;
    case SM_EVENT_PAIRING_STARTED:
        printf("Pairing started\n");
        break;
    case SM_EVENT_PAIRING_COMPLETE:
        switch (sm_event_pairing_complete_get_status(packet)) {
        case ERROR_CODE_SUCCESS:
            printf("Pairing complete, success\n");
            break;
        case ERROR_CODE_CONNECTION_TIMEOUT:
            printf("Pairing failed, timeout\n");
            break;
        case ERROR_CODE_REMOTE_USER_TERMINATED_CONNECTION:
            printf("Pairing failed, disconnected\n");
            break;
        case ERROR_CODE_AUTHENTICATION_FAILURE:
            printf(
                "Pairing failed, authentication failure with reason = %u\n",
                sm_event_pairing_complete_get_reason(packet)
            );
            break;
        default:
            break;
        }
        break;
    case SM_EVENT_REENCRYPTION_STARTED:
        sm_event_reencryption_complete_get_address(packet, addr);
        printf(
            "Bonding information exists for addr type %u, identity addr %s -> "
            "re-encryption started\n",
            sm_event_reencryption_started_get_addr_type(packet), bd_addr_to_str(addr)
        );
        break;
    case SM_EVENT_REENCRYPTION_COMPLETE:
        switch (sm_event_reencryption_complete_get_status(packet)) {
        case ERROR_CODE_SUCCESS:
            printf("Re-encryption complete, success\n");
            break;
        case ERROR_CODE_CONNECTION_TIMEOUT:
            printf("Re-encryption failed, timeout\n");
            break;
        case ERROR_CODE_REMOTE_USER_TERMINATED_CONNECTION:
            printf("Re-encryption failed, disconnected\n");
            break;
        case ERROR_CODE_PIN_OR_KEY_MISSING:
            printf("Re-encryption failed, bonding information missing\n\n");
            printf("Assuming remote lost bonding information\n");
            printf("Deleting local bonding information to allow for new pairing...\n");
            sm_event_reencryption_complete_get_address(packet, addr);
            addr_type = sm_event_reencryption_started_get_addr_type(packet);
            gap_delete_bonding(addr_type, addr);
            break;
        default:
            break;
        }
        break;
    case GATT_EVENT_QUERY_COMPLETE:
        status = gatt_event_query_complete_get_att_status(packet);
        switch (status) {
        case ATT_ERROR_INSUFFICIENT_ENCRYPTION:
            printf("GATT Query failed, Insufficient Encryption\n");
            break;
        case ATT_ERROR_INSUFFICIENT_AUTHENTICATION:
            printf("GATT Query failed, Insufficient Authentication\n");
            break;
        case ATT_ERROR_BONDING_INFORMATION_MISSING:
            printf("GATT Query failed, Bonding Information Missing\n");
            break;
        case ATT_ERROR_SUCCESS:
            printf("GATT Query successful\n");
            break;
        default:
            printf(
                "GATT Query failed, status 0x%02x\n",
                gatt_event_query_complete_get_att_status(packet)
            );
            break;
        }
        break;
    default:
        break;
    }
}

/*
 * @section ATT Read
 *
 * @text The ATT Server handles all reads to constant data. For dynamic data like the
 * custom characteristic, the registered att_read_callback is called. To handle long
 * characteristics and long reads, the att_read_callback is first called with buffer ==
 * NULL, to request the total value length. Then it will be called again requesting a
 * chunk of the value. See Listing attRead.
 */

/* LISTING_START(attRead): ATT Read */

// ATT Client Read Callback for Dynamic Data
// - if buffer == NULL, don't copy data, just return size of value
// - if buffer != NULL, copy data and return number bytes copied
// @param offset defines start of attribute value
static uint16_t att_read_callback(
    hci_con_handle_t connection_handle, uint16_t att_handle, uint16_t offset,
    uint8_t *buffer, uint16_t buffer_size
) {
    UNUSED(connection_handle);

    if (att_handle == ATT_CHARACTERISTIC_FF3F_01_VALUE_HANDLE) {
        return att_read_callback_handle_blob(
            (const uint8_t *)dice_string, dice_string_len, offset, buffer, buffer_size
        );
    }
    return 0;
}
/* LISTING_END */

/*
 * @section ATT Write
 *
 * @text The only valid ATT writes in this example are to the Client Characteristic
 * Configuration, which configures notification and indication and to the the
 * Characteristic Value. If the ATT handle matches the client configuration handle, the
 * new configuration value is stored and used in the heartbeat handler to decide if a
 * new value should be sent. If the ATT handle matches the characteristic value handle,
 * we print the write as hexdump See Listing attWrite.
 */

/* LISTING_START(attWrite): ATT Write */
static int att_write_callback(
    hci_con_handle_t connection_handle, uint16_t att_handle, uint16_t transaction_mode,
    uint16_t offset, uint8_t *buffer, uint16_t buffer_size
) {
    switch (att_handle) {
    case ATT_CHARACTERISTIC_FF3F_01_CLIENT_CONFIGURATION_HANDLE:
        le_notification_enabled =
            little_endian_read_16(buffer, 0) ==
            GATT_CLIENT_CHARACTERISTICS_CONFIGURATION_NOTIFICATION;
        con_handle = connection_handle;
        break;
    case ATT_CHARACTERISTIC_FF3F_01_VALUE_HANDLE:
        printf(
            "Write: transaction mode %u, offset %u, data (%u bytes): ",
            transaction_mode, offset, buffer_size
        );
        printf_hexdump(buffer, buffer_size);
        break;
    default:
        break;
    }
    return 0;
}
/* LISTING_END */

int btstack_main(void);
int btstack_main(void) {
    le_dice_setup();

    // turn on!
    hci_power_control(HCI_POWER_ON);

    return 0;
}
/* EXAMPLE_END */
