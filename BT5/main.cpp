/*
 * copyright (c) 2012 - 2018, nordic semiconductor asa
 * all rights reserved.
 *
 * redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. redistributions in binary form, except as embedded into a nordic
 *    semiconductor asa integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/**@example examples/heart_rate_collector
 *
 * @brief Heart Rate Collector Sample Application main file.
 *
 * This file contains the source code for a sample application that acts as a BLE Central device.
 * This application scans for a Heart Rate Sensor device and reads it's heart rate data.
 * https://www.bluetooth.com/specifications/gatt/viewer?attributeXmlFile=org.bluetooth.service.heart_rate.xml
 *
 * Structure of this file
 * - Includes
 * - Definitions
 * - Global variables
 * - Global functions
 * - Event functions
 * - Event dispatcher
 * - Main
 */

//try github
//try github with the same account

/** Includes */
#include "ble.h"
#include "sd_rpc.h"

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include <chrono>
#include <ctime>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

using std::ostream;
using std::string;
using std::vector;

static vector<uint16_t> vecHIDhandle;
static uint16_t m_voice_handle;
/** Definitions */
#define DEFAULT_BAUD_RATE                                                                          \
    1000000 /**< The baud rate to be used for serial communication with nRF5 device. */

#ifdef _WIN32
#define DEFAULT_UART_PORT_NAME "COM4"
#endif
#ifdef __APPLE__
#define DEFAULT_UART_PORT_NAME "/dev/tty.usbmodem00000"
#endif
#ifdef __linux__
#define DEFAULT_UART_PORT_NAME "/dev/ttyACM0"
#endif

enum {
    UNIT_0_625_MS = 625,  /**< Number of microseconds in 0.625 milliseconds. */
    UNIT_1_25_MS  = 1250, /**< Number of microseconds in 1.25 milliseconds. */
    UNIT_10_MS    = 10000 /**< Number of microseconds in 10 milliseconds. */
};

#define MSEC_TO_UNITS(TIME, RESOLUTION) (((TIME)*1000) / (RESOLUTION))

#define SCAN_INTERVAL 0x00A0 /**< Determines scan interval in units of 0.625 milliseconds. */
#define SCAN_WINDOW 0x0050   /**< Determines scan window in units of 0.625 milliseconds. */
#define SCAN_TIMEOUT                                                                               \
    0x0 /**< Scan timeout between 0x01 and 0xFFFF in seconds, 0x0 disables timeout. */

#define MIN_CONNECTION_INTERVAL                                                                    \
    MSEC_TO_UNITS(7.5, UNIT_1_25_MS) /**< Determines minimum connection interval in milliseconds.  \
                                      */
#define MAX_CONNECTION_INTERVAL                                                                    \
    MSEC_TO_UNITS(7.5, UNIT_1_25_MS) /**< Determines maximum connection interval in milliseconds.  \
                                      */
#define SLAVE_LATENCY 0              /**< Slave Latency in number of connection events. */
#define CONNECTION_SUPERVISION_TIMEOUT                                                             \
    MSEC_TO_UNITS(4000,                                                                            \
                  UNIT_10_MS) /**< Determines supervision time-out in units of 10 milliseconds. */

//#define TARGET_DEV_NAME                                                                            \
//    "RemoteB029" /**< Connect to a peripheral using a given advertising name here. */
// const string sub_target_dev_name = "D4B8";

#define TARGET_DEV_NAME                                                                            \
    "VinoX_BT5_2" /**< Connect to a peripheral using a given advertising name here. */
const string sub_target_dev_name = "1846"; // 18-46-44-81-44-54

#define MAX_PEER_COUNT 1 /**< Maximum number of peer's application intends to manage. */

#define BLE_UUID_HOGP_SERVICE 0x1812
#define BLE_UUID_HID_UUID 0x2A4D

#define BLE_UUID_HEART_RATE_SERVICE 0x180D /**< Heart Rate service UUID. */
#define BLE_UUID_HEART_RATE_MEASUREMENT_CHAR                                                       \
    0x2A37 /**< Heart Rate Measurement characteristic UUID. */
#define BLE_UUID_CCCD 0x2902
#define BLE_CCCD_NOTIFY 0x01

#define BLE_UUID_BATTERY_SERVICE 0x180F
#define BLE_UUID_BATTERY_LEVEL 0x2A19

#define BLE_UUID_DEVICE_INFO_SERVICE 0x180A // 0x2A50
#define BLE_UUID_DEVICE_INFO_UUID 0x2A50

#define HOGP_UUID_SERVICE 0x1812

#define STRING_BUFFER_SIZE 50

typedef struct
{
    uint8_t *p_data;   /**< Pointer to data. */
    uint16_t data_len; /**< Length of data. */
} data_t;

/** Global variables */
static uint8_t m_connected_devices      = 0;
static uint16_t m_connection_handle     = 0;
static uint16_t m_service_start_handle  = 0;
static uint16_t m_service_end_handle    = 0;
static uint16_t m_hrm_char_handle       = 0;
static uint16_t m_hrm_cccd_handle       = 0;
static bool m_connection_is_in_progress = false;
static adapter_t *m_adapter             = NULL;

static uint32_t m_desc_start_handle = 0;

#if NRF_SD_BLE_API >= 5
static uint32_t m_config_id = 1;
#endif

#if NRF_SD_BLE_API >= 6
static uint8_t mp_data[100] = {0};
static ble_data_t m_adv_report_buffer;
#endif

static const ble_gap_scan_params_t m_scan_param = {
#if NRF_SD_BLE_API >= 6
    0, // Set if accept extended advertising packetets.
    0, // Set if report inomplete reports.
#endif
    0, // Set if active scanning.
#if NRF_SD_BLE_API < 6
    0, // Set if selective scanning.
#endif
#if NRF_SD_BLE_API >= 6
    BLE_GAP_SCAN_FP_ACCEPT_ALL,
    BLE_GAP_PHY_1MBPS,
#endif
#if NRF_SD_BLE_API == 2
    NULL, // Set white-list.
#endif
#if NRF_SD_BLE_API == 3 || NRF_SD_BLE_API == 5
    0, // Set adv_dir_report.
#endif
    (uint16_t)SCAN_INTERVAL,
    (uint16_t)SCAN_WINDOW,
    (uint16_t)SCAN_TIMEOUT
#if NRF_SD_BLE_API >= 6
    ,
    {0} // Set chennel mask.
#endif
};

static const ble_gap_conn_params_t m_connection_param = {
    (uint16_t)MIN_CONNECTION_INTERVAL, (uint16_t)MAX_CONNECTION_INTERVAL, (uint16_t)SLAVE_LATENCY,
    (uint16_t)CONNECTION_SUPERVISION_TIMEOUT};

static string GetString(uint8_t *str)
{
    int n = -1;
    char buffer[STRING_BUFFER_SIZE];
    n                    = sprintf(buffer, "0x%s\n", str);
    string deviceAddress = buffer;
    return deviceAddress;
}

static bool CompareDeviceName(string targetDeviceName, string scanedDeviceName)
{
    printf("scaned device name %s\n", scanedDeviceName.c_str());
    printf(" target device name %s\n ", targetDeviceName.c_str());
    if (scanedDeviceName.find(targetDeviceName) != string::npos)
    {
        return true;
    }
    else
    {
        return false;
    }
}
/** Global functions */

/**@brief Function for handling error message events from sd_rpc.
 *
 * @param[in] adapter The transport adapter.
 * @param[in] code Error code that the error message is associated with.
 * @param[in] message The error message that the callback is associated with.
 */
static void status_handler(adapter_t *adapter, sd_rpc_app_status_t code, const char *message)
{
    printf("Status: %d, message: %s\n", (uint32_t)code, message);
    fflush(stdout);
}

/**@brief Function for handling the log message events from sd_rpc.
 *
 * @param[in] adapter The transport adapter.
 * @param[in] severity Level of severity that the log message is associated with.
 * @param[in] message The log message that the callback is associated with.
 */
static void log_handler(adapter_t *adapter, sd_rpc_log_severity_t severity, const char *message)
{
    switch (severity)
    {
        case SD_RPC_LOG_ERROR:
            printf("Error: %s\n", message);
            fflush(stdout);
            break;

        case SD_RPC_LOG_WARNING:
            printf("Warning: %s\n", message);
            fflush(stdout);
            break;

        case SD_RPC_LOG_INFO:
            printf("Info: %s\n", message);
            fflush(stdout);
            break;

        default:
            printf("Log: %s\n", message);
            fflush(stdout);
            break;
    }
}

/**@brief Function for initializing serial communication with the target nRF5 Bluetooth slave.
 *
 * @param[in] serial_port The serial port the target nRF5 device is connected to.
 *
 * @return The new transport adapter.
 */
static adapter_t *adapter_init(char *serial_port, uint32_t baud_rate)
{
    physical_layer_t *phy;
    data_link_layer_t *data_link_layer;
    transport_layer_t *transport_layer;

    phy = sd_rpc_physical_layer_create_uart(serial_port, baud_rate, SD_RPC_FLOW_CONTROL_NONE,
                                            SD_RPC_PARITY_NONE);
    data_link_layer = sd_rpc_data_link_layer_create_bt_three_wire(phy, 250);
    transport_layer = sd_rpc_transport_layer_create(data_link_layer, 1500);
    return sd_rpc_adapter_create(transport_layer);
}

/**@brief Function for converting a BLE address to a string.
 *
 * @param[in] address       Bluetooth Low Energy address.
 * @param[out] string_buffer The serial port the target nRF5 device is connected to.
 */
static void ble_address_to_string_convert(ble_gap_addr_t address, uint8_t *string_buffer)
{
    const int address_length = 6;
    char temp_str[3];

    for (int i = address_length - 1; i >= 0; --i)
    {
        sprintf(temp_str, "%02X", address.addr[i]);
        strcat((char *)string_buffer, temp_str);
    }
}

/**
 * @brief Parses advertisement data, providing length and location of the field in case
 *        matching data is found.
 *
 * @param[in]  Type of data to be looked for in advertisement data.
 * @param[in]  Advertisement report length and pointer to report.
 * @param[out] If data type requested is found in the data report, type data length and
 *             pointer to data will be populated here.
 *
 * @retval NRF_SUCCESS if the data type is found in the report.
 * @retval NRF_ERROR_NOT_FOUND if the data type could not be found.
 */
static uint32_t adv_report_parse(uint8_t type, data_t *p_advdata, data_t *p_typedata)
{
    uint32_t index = 0;
    uint8_t *p_data;

    p_data = p_advdata->p_data;

    while (index < p_advdata->data_len)
    {
        uint8_t field_length = p_data[index];
        uint8_t field_type   = p_data[index + 1];

        if (field_type == type)
        {
            p_typedata->p_data   = &p_data[index + 2];
            p_typedata->data_len = field_length - 1;
            return NRF_SUCCESS;
        }
        index += field_length + 1;
    }
    return NRF_ERROR_NOT_FOUND;
}

/**@brief Function for searching a given name in the advertisement packets.
 *
 * @details Use this function to parse received advertising data and to find a given
 * name in them either as 'complete_local_name' or as 'short_local_name'.
 *
 * @param[in]   p_adv_report   advertising data to parse.
 * @param[in]   name_to_find   name to search.
 * @return   true if the given name was found, false otherwise.
 */
static bool find_adv_name(const ble_gap_evt_adv_report_t *p_adv_report, const char *name_to_find)
{
    uint32_t err_code;
    data_t adv_data;
    data_t dev_name;

    // Initialize advertisement report for parsing
#if NRF_SD_BLE_API >= 6
    adv_data.p_data   = (uint8_t *)p_adv_report->data.p_data;
    adv_data.data_len = p_adv_report->data.len;
#else
    adv_data.p_data                                         = (uint8_t *)p_adv_report->data;
    adv_data.data_len                                       = p_adv_report->dlen;
#endif

    // search for advertising names
    err_code = adv_report_parse(BLE_GAP_AD_TYPE_COMPLETE_LOCAL_NAME, &adv_data, &dev_name);
    if (err_code == NRF_SUCCESS)
    {
        string targetDeviceName = name_to_find;
        string scanedDeviceName = GetString(dev_name.p_data);
        // if (memcmp(name_to_find, dev_name.p_data, dev_name.data_len )== 0)
        if (CompareDeviceName(targetDeviceName, scanedDeviceName))
        {
            printf("target device name: 0x%s\n", dev_name.p_data);
            return true;
        }
        else
        {
            printf("target device name: 0x%s\n", dev_name.p_data);
        }
    }
    else
    {
        // Look for the short local name if it was not found as complete
        err_code = adv_report_parse(BLE_GAP_AD_TYPE_SHORT_LOCAL_NAME, &adv_data, &dev_name);
        if (err_code != NRF_SUCCESS)
        {
            printf("NOT success, error code : %d\n", err_code);
            return false;
        }
        string targetDeviceName = name_to_find;
        string scanedDeviceName = GetString(dev_name.p_data);
        // if (memcmp(name_to_find, dev_name.p_data, dev_name.data_len )== 0)
        if (CompareDeviceName(targetDeviceName, scanedDeviceName))
        {
            printf("target device name: 0x%s\n", dev_name.p_data);
            return true;
        }
        else
        {
            printf("target device name: 0x%s\n", dev_name.p_data);
        }
    }
    return false;
}

/**@brief Function for initializing the BLE stack.
 *
 * @return NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t ble_stack_init()
{
    uint32_t err_code;
    uint32_t *app_ram_base = NULL;

#if NRF_SD_BLE_API <= 3
    ble_enable_params_t ble_enable_params;
    memset(&ble_enable_params, 0, sizeof(ble_enable_params));
#endif

#if NRF_SD_BLE_API == 3
    ble_enable_params.gatt_enable_params.att_mtu = GATT_MTU_SIZE_DEFAULT;
#elif NRF_SD_BLE_API < 3
    ble_enable_params.gatts_enable_params.attr_tab_size     = BLE_GATTS_ATTR_TAB_SIZE_DEFAULT;
    ble_enable_params.gatts_enable_params.service_changed   = false;
    ble_enable_params.common_enable_params.p_conn_bw_counts = NULL;
    ble_enable_params.common_enable_params.vs_uuid_count    = 1;
#endif

#if NRF_SD_BLE_API <= 3
    ble_enable_params.gap_enable_params.periph_conn_count  = 1;
    ble_enable_params.gap_enable_params.central_conn_count = 1;
    ble_enable_params.gap_enable_params.central_sec_count  = 1;

    err_code = sd_ble_enable(m_adapter, &ble_enable_params, app_ram_base);
#else
    err_code                                                = sd_ble_enable(m_adapter, app_ram_base);
#endif

    switch (err_code)
    {
        case NRF_SUCCESS:
            break;
        case NRF_ERROR_INVALID_STATE:
            printf("BLE stack already enabled\n");
            fflush(stdout);
            break;
        default:
            printf("Failed to enable BLE stack. Error code: %d\n", err_code);
            fflush(stdout);
            break;
    }

    return err_code;
}

#if NRF_SD_BLE_API < 5
/**@brief Set BLE option for the BLE role and connection bandwidth.
 *
 * @return NRF_SUCCESS on option set successfully, otherwise an error code.
 */
static uint32_t ble_options_set()
{
#if NRF_SD_BLE_API <= 3
    ble_opt_t opt;
    ble_common_opt_t common_opt;

    common_opt.conn_bw.role               = BLE_GAP_ROLE_CENTRAL;
    common_opt.conn_bw.conn_bw.conn_bw_rx = BLE_CONN_BW_HIGH;
    common_opt.conn_bw.conn_bw.conn_bw_tx = BLE_CONN_BW_HIGH;
    opt.common_opt                        = common_opt;

    return sd_ble_opt_set(m_adapter, BLE_COMMON_OPT_CONN_BW, &opt);
#else
    return NRF_ERROR_NOT_SUPPORTED;
#endif
}
#endif

#if NRF_SD_BLE_API >= 5
/**@brief Function for setting configuration for the BLE stack.
 *
 * @return NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t ble_cfg_set(uint8_t conn_cfg_tag)
{
    const uint32_t ram_start = 0; // Value is not used by ble-driver
    uint32_t error_code;
    ble_cfg_t ble_cfg;

    // Configure the connection roles.
    memset(&ble_cfg, 0, sizeof(ble_cfg));

#if NRF_SD_BLE_API >= 6
    ble_cfg.gap_cfg.role_count_cfg.adv_set_count = BLE_GAP_ADV_SET_COUNT_DEFAULT;
#endif
    ble_cfg.gap_cfg.role_count_cfg.periph_role_count  = 0;
    ble_cfg.gap_cfg.role_count_cfg.central_role_count = 1;
    ble_cfg.gap_cfg.role_count_cfg.central_sec_count  = 1;
    ble_cfg.common_cfg.vs_uuid_cfg.vs_uuid_count      = BLE_UUID_VS_COUNT_DEFAULT;

    error_code = sd_ble_cfg_set(m_adapter, BLE_GAP_CFG_ROLE_COUNT, &ble_cfg, ram_start);
    if (error_code != NRF_SUCCESS)
    {
        printf("sd_ble_cfg_set() failed when attempting to set BLE_GAP_CFG_ROLE_COUNT. Error code: "
               "0x%02X\n",
               error_code);
        fflush(stdout);
        return error_code;
    }

    memset(&ble_cfg, 0x00, sizeof(ble_cfg));
    ble_cfg.conn_cfg.conn_cfg_tag                 = conn_cfg_tag;
    ble_cfg.conn_cfg.params.gatt_conn_cfg.att_mtu = 150;

    error_code = sd_ble_cfg_set(m_adapter, BLE_CONN_CFG_GATT, &ble_cfg, ram_start);
    if (error_code != NRF_SUCCESS)
    {
        printf("sd_ble_cfg_set() failed when attempting to set BLE_CONN_CFG_GATT. Error code: "
               "0x%02X\n",
               error_code);
        fflush(stdout);
        return error_code;
    }

    return NRF_SUCCESS;
}
#endif

/**@brief Start scanning (GAP Discovery procedure, Observer Procedure).
 * *
 * @return NRF_SUCCESS on successfully initiating scanning procedure, otherwise an error code.
 */
static uint32_t scan_start()
{
#if NRF_SD_BLE_API >= 6
    m_adv_report_buffer.p_data = mp_data;
    m_adv_report_buffer.len    = sizeof(mp_data);
#endif

    uint32_t error_code = sd_ble_gap_scan_start(m_adapter, &m_scan_param
#if NRF_SD_BLE_API >= 6
                                                ,
                                                &m_adv_report_buffer
#endif
    );

    if (error_code != NRF_SUCCESS)
    {
        printf("Scan start failed with error code: %d\n", error_code);
        fflush(stdout);
    }
    else
    {
        printf("Scan started\n");
        fflush(stdout);
    }

    return error_code;
}

/**@brief Function called upon connecting to BLE peripheral.
 *
 * @details Initiates primary service discovery.
 *
 * @return NRF_SUCCESS on success, otherwise an error code.
 */

static uint32_t service_discovery_start()
{
    uint32_t err_code;
    uint16_t start_handle = 0x01;
    ble_uuid_t srvc_uuid;
    printf("Discovering primary services\n");
    fflush(stdout);

    srvc_uuid.type = BLE_UUID_TYPE_BLE;
    srvc_uuid.uuid = 0x1812;

    err_code = sd_ble_gattc_primary_services_discover(m_adapter, m_connection_handle, start_handle,
                                                      &srvc_uuid);
    if (err_code != NRF_SUCCESS)
    {
        printf("Failed to initiate or continue a GATT Primary Service Discovery procedure\n");
        fflush(stdout);
    }
    return err_code;
}

#define BLE_UUID_ATV_BASE_UUID                                                                     \
    {                                                                                              \
        {                                                                                          \
            0x64, 0xb6, 0x17, 0xf6, 0x01, 0xaf, 0x7d, 0xbc, 0x05, 0x4f, 0x21, 0x5a, 0x01, 0x00,    \
                0x5e, 0xab                                                                         \
        }                                                                                          \
    }
#define BLE_UUID_ATV_BASE_UUID_R                                                                   \
    {                                                                                              \
        {                                                                                          \
            0xab, 0x5e, 0x00, 0x01, 0x5a, 0x21, 0x4f, 0x05, 0xbc, 0x7d, 0xaf, 0x01, 0xf6, 0x17,    \
                0xb6, 0x64                                                                         \
        }                                                                                          \
    }
#define BLE_UUID_ATV_SERVICE_UUID 0x0001
static uint32_t service_discovery_voice()
{
    uint32_t err_code;
    uint16_t start_handle = 0x0001; // must
    ble_uuid_t srvc_uuid;
    // uint16_t srvc_handle = 0x54;
    printf("Discovering VOICE services\n");
    fflush(stdout);

    ble_uuid128_t voice_uuid = BLE_UUID_ATV_BASE_UUID; // how!! how to add it???
    srvc_uuid.type           = BLE_UUID_TYPE_VENDOR_BEGIN;
    sd_ble_uuid_vs_add(m_adapter, &voice_uuid, &srvc_uuid.type);
    // sd_ble_gatts_service_add(m_adapter, srvc_uuid.type, &srvc_uuid, &srvc_handle);

    srvc_uuid.uuid = BLE_UUID_ATV_SERVICE_UUID;
    err_code = sd_ble_gattc_primary_services_discover(m_adapter, m_connection_handle, start_handle,
                                                      &srvc_uuid);
    //#define 	BLE_GATT_STATUS_ATTERR_ATTRIBUTE_NOT_FOUND   0x010A
    if (err_code != NRF_SUCCESS)
    {
        printf("Failed to initiate or continue VOICE Service Discovery procedure\n");
        fflush(stdout);
    }
    return err_code;
}

static void findMoreService()
{
    int32_t error_code    = 0;
    uint16_t start_handle = m_service_end_handle + 1;
    error_code =
        sd_ble_gattc_primary_services_discover(m_adapter, m_connected_devices, start_handle, NULL);
    if (error_code != NRF_SUCCESS)
    {
        printf("fail to discover more services at start handle %04X, error code %04X!\n",
               start_handle, error_code);
    }
    else
    {
        printf("find more service at start handle %04X!\n", start_handle);
    }
}

static void findMoreService(uint16_t moreHandle)
{
    int32_t error_code    = 0;
    uint16_t start_handle = moreHandle;
    error_code =
        sd_ble_gattc_primary_services_discover(m_adapter, m_connected_devices, start_handle, NULL);
    if (error_code != NRF_SUCCESS)
    {
        printf("fail to discover more services at start handle %04X!\n", start_handle);
    }
    else
    {
        printf("find more service at start handle %04X!\n", start_handle);
    }
}

// static uint32_t service_discovery_voice()
//{
//    uint32_t err_code;
//    uint16_t start_handle = 0x01;
//    ble_uuid_t srvc_uuid;
//
//    printf("Discovering primary services\n");
//    fflush(stdout);
//
//    srvc_uuid.type = BLE_UUID_TYPE_VENDOR_BEGIN;
//    //srvc_uuid.uuid
//    // BLE_UUID_HEART_RATE_SERVICE;
//
//    // Initiate procedure to find the primary BLE_UUID_HEART_RATE_SERVICE.
//    err_code = sd_ble_gattc_primary_services_discover(m_adapter, m_connection_handle,
//    start_handle,
//                                                      &srvc_uuid);
//    if (err_code != NRF_SUCCESS)
//    {
//        printf("Failed to initiate or continue a GATT Primary Service Discovery procedure\n");
//        fflush(stdout);
//    }
//
//    return err_code;
//}

/**@brief Function called upon discovering a BLE peripheral's primary service(s).
 *
 * @details Initiates service's (m_service) characteristic discovery.
 *
 * @return NRF_SUCCESS on success, otherwise an error code.
 */
static uint16_t moreHandle_Start = 1;
static void EnableHOGPNotice(uint16_t hid_char_handle)
{
    ble_gattc_write_params_t write_params;
    memset(&write_params, 0, sizeof(write_params));
    write_params.write_op  = BLE_GATT_OP_WRITE_REQ;
    write_params.flags     = BLE_GATT_EXEC_WRITE_FLAG_PREPARED_WRITE;
    write_params.handle    = hid_char_handle + 2;
    write_params.offset    = 0;
    write_params.len       = 2;
    uint8_t write_value[2] = {0x01, 0x00};
    write_params.p_value   = write_value;

    // if (write_params.handle == 0x30)
    //       return;

    uint32_t write_gatt_status = sd_ble_gattc_write(m_adapter, m_connection_handle, &write_params);
    if (write_gatt_status != NRF_SUCCESS)
    {
        printf("wrrite gatt handle is 0x%X\n", write_params.handle);
        printf("wrrite gatt failed. Error code 0x%X\n", write_gatt_status);
        fflush(stdout);
    }
    else
    {
        printf("wrrite gatt handle is 0x%X\n", write_params.handle);
        printf("wrrite gatt successfully.  0x%X\n", write_gatt_status);
        fflush(stdout);
    }
}

static void EnableVoiceNotice(uint16_t voice_char_handle)
{
    ble_gattc_write_params_t write_params;
    memset(&write_params, 0, sizeof(write_params));
    write_params.write_op  = BLE_GATT_OP_WRITE_REQ;
    write_params.flags     = BLE_GATT_EXEC_WRITE_FLAG_PREPARED_WRITE;
    write_params.handle    = voice_char_handle;
    write_params.offset    = 0;
    write_params.len       = 2;
    uint8_t write_value[2] = {0x01, 0x00};
    write_params.p_value   = write_value;

    // if (write_params.handle == 0x30)
    //       return;

    uint32_t write_gatt_status = sd_ble_gattc_write(m_adapter, m_connection_handle, &write_params);
    if (write_gatt_status != NRF_SUCCESS)
    {
        printf("wrrite voice handle is 0x%X\n", write_params.handle);
        printf("wrrite voice failed. Error code 0x%X\n", write_gatt_status);
        fflush(stdout);
    }
    else
    {
        printf("wrrite voice handle is 0x%X\n", write_params.handle);
        printf("wrrite voice successfully.  0x%X\n", write_gatt_status);
        fflush(stdout);
    }
}
static void OpenMic()
{
    ble_gattc_write_params_t write_params;
    memset(&write_params, 0, sizeof(write_params));
    write_params.write_op  = BLE_GATT_OP_WRITE_REQ;
    write_params.flags     = BLE_GATT_EXEC_WRITE_FLAG_PREPARED_WRITE;
    write_params.handle    = m_voice_handle;
    write_params.offset    = 0;
    write_params.len       = 3;
    uint8_t write_value[3] = {0x0c, 0x00, 0x02};
    write_params.p_value   = write_value;

    // if (write_params.handle == 0x30)
    //       return;

    uint32_t write_gatt_status = sd_ble_gattc_write(m_adapter, m_connection_handle, &write_params);
    if (write_gatt_status != NRF_SUCCESS)
    {
        printf("wrrite voice handle is 0x%X\n", write_params.handle);
        printf("wrrite voice failed. Error code 0x%X\n", write_gatt_status);
        fflush(stdout);
    }
    else
    {
        printf("wrrite voice handle is 0x%X\n", write_params.handle);
        printf("wrrite voice successfully.  0x%X\n", write_gatt_status);
        fflush(stdout);
    }
}
static void CloseMic()
{
    ble_gattc_write_params_t write_params;
    memset(&write_params, 0, sizeof(write_params));
    write_params.write_op  = BLE_GATT_OP_WRITE_REQ;
    write_params.flags     = BLE_GATT_EXEC_WRITE_FLAG_PREPARED_WRITE;
    write_params.handle    = m_voice_handle;
    write_params.offset    = 0;
    write_params.len       = 1;
    uint8_t write_value[1] = {0x0d};
    write_params.p_value   = write_value;

    // if (write_params.handle == 0x30)
    //       return;

    uint32_t write_gatt_status = sd_ble_gattc_write(m_adapter, m_connection_handle, &write_params);
    if (write_gatt_status != NRF_SUCCESS)
    {
        printf("wrrite voice handle is 0x%X\n", write_params.handle);
        printf("wrrite voice failed. Error code 0x%X\n", write_gatt_status);
        fflush(stdout);
    }
    else
    {
        printf("wrrite voice handle is 0x%X\n", write_params.handle);
        printf("wrrite voice successfully.  0x%X\n", write_gatt_status);
        fflush(stdout);
    }
}
static uint32_t char_discovery_start()
{
    ble_gattc_handle_range_t handle_range;

    printf("Discovering characteristics from %2X to %2X\n", m_service_start_handle,
           m_service_end_handle);
    fflush(stdout);

    handle_range.start_handle = m_service_start_handle;
    handle_range.end_handle   = m_service_end_handle;

    return sd_ble_gattc_characteristics_discover(m_adapter, m_connection_handle, &handle_range);
}
static uint32_t descr_discovery_start()
{
    ble_gattc_handle_range_t handle_range;

    printf("Discovering characteristic's descriptors\n");
    fflush(stdout);

    if (m_desc_start_handle == 0)
    {
        printf("No heart rate measurement characteristic handle found\n");
        fflush(stdout);
        return NRF_ERROR_INVALID_STATE;
    }

    handle_range.start_handle = m_desc_start_handle;
    handle_range.end_handle   = m_service_end_handle;

    return sd_ble_gattc_descriptors_discover(m_adapter, m_connection_handle, &handle_range);
}
static uint32_t char_discovery_start(uint16_t moreDiscvy)
{
    ble_gattc_handle_range_t handle_range;

    printf("Discovering more characteristics with end from %2X to %2X\n", moreDiscvy,
           m_service_end_handle);
    fflush(stdout);

    handle_range.start_handle = moreDiscvy;
    handle_range.end_handle   = m_service_end_handle;

    if ((handle_range.end_handle - handle_range.start_handle < 5) && moreDiscvy == 0x39 &&
        (vecHIDhandle.size() > 0))
    {
        printf("stop discovering characteristics from %2X to %2X\n", moreDiscvy,
               m_service_end_handle);
        for (size_t i = 0; i < vecHIDhandle.size(); i++)
        {
            EnableHOGPNotice(vecHIDhandle[i]);
            _sleep(1000);
        }
        vecHIDhandle.clear();
        // findMoreService();
        descr_discovery_start();
        return 1;
    }
    // find more service

    return sd_ble_gattc_characteristics_discover(m_adapter, m_connection_handle, &handle_range);
}

static void EnableAllHIDService()
{
    for (size_t i = 0; i < vecHIDhandle.size(); i++)
    {
        EnableHOGPNotice(vecHIDhandle[i]);
        _sleep(100);
    }
}
/**@brief Function called upon discovering service's characteristics.
 *
 * @details Initiates heart rate monitor (m_hrm_char_handle) characteristic's descriptor discovery.
 *
 * @return NRF_SUCCESS on success, otherwise an error code.
 */

/**@brief Function that write's the HRM characteristic's CCCD.
 * *
 * @return NRF_SUCCESS on success, otherwise an error code.
 */
static uint32_t hrm_cccd_set(uint8_t value)
{
    ble_gattc_write_params_t write_params;
    uint8_t cccd_value[2] = {value, 0};

    printf("Setting HRM CCCD\n");
    fflush(stdout);

    if (m_hrm_cccd_handle == 0)
    {
        printf("Error. No CCCD handle has been found\n");
        fflush(stdout);
        return NRF_ERROR_INVALID_STATE;
    }

    write_params.handle   = m_hrm_cccd_handle;
    write_params.len      = 2;
    write_params.p_value  = cccd_value;
    write_params.write_op = BLE_GATT_OP_WRITE_REQ;
    write_params.offset   = 0;

    return sd_ble_gattc_write(m_adapter, m_connection_handle, &write_params);
}

/** Event functions */

/**@brief Function called on BLE_GAP_EVT_CONNECTED event.
 *
 * @details Update connection state and proceed to discovering the peer's GATT services.
 *
 * @param[in] p_ble_gap_evt GAP event.
 */
static void update_phy(const ble_gap_evt_t *const p_ble_gap_evt)
{
    printf("update phyics\n");

    fflush(stdout);
    /*err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
    APP_ERROR_CHECK(err_code);*/
    uint16_t m_conn_handle = p_ble_gap_evt->conn_handle;
    ble_gap_phys_t py_params;
    py_params.tx_phys = BLE_GAP_PHY_2MBPS;
    py_params.rx_phys = BLE_GAP_PHY_2MBPS;
    uint32_t nResult  = -1;
    nResult           = sd_ble_gap_phy_update(m_adapter, m_conn_handle, &py_params);
    printf("update phys status is %d\n", nResult);
}
static void on_connected(const ble_gap_evt_t *const p_ble_gap_evt)
{
    printf("Connection established\n");
    fflush(stdout);

    m_connected_devices++;
    m_connection_handle         = p_ble_gap_evt->conn_handle;
    m_connection_is_in_progress = false;

    // service_discovery_start();
    update_phy(p_ble_gap_evt);
}

static void on_pair(const ble_gap_evt_t *const p_ble_gap_evt)
{
    printf("\non_pairing\n");
    fflush(stdout);
    /*err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
    APP_ERROR_CHECK(err_code);*/
    uint16_t m_conn_handle = p_ble_gap_evt->conn_handle;
    // 1连接一建立就发送安全请求，从而促使手机发送配对请求过?

    ble_gap_sec_params_t params;
    params.bond            = 0; // 1 to bond, 0 to pair?
    params.mitm            = 0;
    params.lesc            = 0;
    params.keypress        = 0;
    params.io_caps         = BLE_GAP_IO_CAPS_NONE; // BLE_GAP_IO_CAPS_KEYBOARD_DISPLAY;
    params.oob             = 0;
    params.min_key_size    = 7;
    params.max_key_size    = 16;
    params.kdist_own.enc   = 1;
    params.kdist_own.id    = 1;
    params.kdist_own.link  = 0;
    params.kdist_own.sign  = 0;
    params.kdist_peer.enc  = 1;
    params.kdist_peer.id   = 1;
    params.kdist_peer.link = 0;
    params.kdist_peer.sign = 0;

    uint32_t nResult = -1;
    nResult          = sd_ble_gap_authenticate(m_adapter, m_conn_handle, &params);
    printf("pair status is %d\n", nResult);
}

/**@brief Function called on BLE_GAP_EVT_ADV_REPORT event.
 *
 * @details Create a connection if received advertising packet corresponds to desired BLE device.
 *
 * @param[in] p_ble_gap_evt Advertising Report Event.
 */
static void on_adv_report(const ble_gap_evt_t *const p_ble_gap_evt)
{
    uint32_t err_code;
    uint8_t str[STRING_BUFFER_SIZE] = {0};

    // Log the Bluetooth device address of advertisement packet received.
    ble_address_to_string_convert(p_ble_gap_evt->params.adv_report.peer_addr, str);
    int n = -1;
    char buffer[STRING_BUFFER_SIZE];
    n                    = sprintf(buffer, "0x%s\n", str);
    string deviceAddress = GetString(str);
    // 1804ED207FBB   bt 412
    if (deviceAddress.find(sub_target_dev_name) != string::npos)
    {
        printf("target device address: 0x%s\n", str);
    }
    else
    {
        return;
    }
    // printf("Received advertisement report with device address: 0x%s\n", str);
    fflush(stdout);

    if (find_adv_name(&p_ble_gap_evt->params.adv_report, TARGET_DEV_NAME))
    {
        printf(" connect start ");

        if (m_connected_devices >= MAX_PEER_COUNT || m_connection_is_in_progress)
        {
            return;
        }
        else
        {
            printf(" connecting ");
        }
        err_code = sd_ble_gap_connect(m_adapter, &(p_ble_gap_evt->params.adv_report.peer_addr),
                                      &m_scan_param, &m_connection_param
#if NRF_SD_BLE_API >= 5
                                      ,
                                      m_config_id
#endif
        );
        if (err_code != NRF_SUCCESS)
        {
            printf("Connection Request Failed, reason %d\n", err_code);
            fflush(stdout);
            return;
        }
        else
        {
            printf("Connection Request Successfully, result %d\n", err_code);
        }

        m_connection_is_in_progress = true;
    }
    else
    {
        printf(" compare name failed! ");
    }
#if NRF_SD_BLE_API >= 6
    else
    {
        err_code = sd_ble_gap_scan_start(m_adapter, NULL, &m_adv_report_buffer);

        if (err_code != NRF_SUCCESS)
        {
            printf("Scan start failed with error code: %d\n", err_code);
            fflush(stdout);
        }
        else
        {
            printf("Scan started\n");
            fflush(stdout);
        }
    }
#endif
}

/**@brief Function called on BLE_GAP_EVT_TIMEOUT event.
 *
 * @param[in] ble_gap_evt_t Timeout Event.
 */
static void on_timeout(const ble_gap_evt_t *const p_ble_gap_evt)
{
    if (p_ble_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_CONN)
    {
        m_connection_is_in_progress = false;
    }
    else if (p_ble_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_SCAN)
    {
        scan_start();
    }
}

static void on_phy_update(const ble_gap_evt_t *const p_ble_gap_evt)
{
    if (p_ble_gap_evt->params.phy_update.status == BLE_GATT_STATUS_SUCCESS)
    {
        uint8_t tx_phy = p_ble_gap_evt->params.phy_update.tx_phy;
        uint8_t rx_phy = p_ble_gap_evt->params.phy_update.rx_phy;
        printf("receive phy updated tx is %d, rx is %d\n", tx_phy, rx_phy);
    }
    else
    {
        printf("receive phy update status is %d\n", p_ble_gap_evt->params.phy_update.status);
    }
}
// Macl add
const char hexmap[] = {'0', '1', '2', '3', '4', '5', '6', '7',
                       '8', '9', 'a', 'b', 'c', 'd', 'e', 'f'};

static std::string hexStr(const uint8_t *data, int len)
{
    std::string s(len * 2, ' ');
    for (int i = 0; i < len; ++i)
    {
        s[2 * i]     = hexmap[(data[i] & 0xF0) >> 4];
        s[2 * i + 1] = hexmap[data[i] & 0x0F];
    }
    return s;
}
static std::string hexStr(uint8_t *data, int len)
{
    std::string s(len * 2, ' ');
    for (int i = 0; i < len; ++i)
    {
        s[2 * i]     = hexmap[(data[i] & 0xF0) >> 4];
        s[2 * i + 1] = hexmap[data[i] & 0x0F];
    }
    return s;
}
/**@brief Function called on BLE_GATTC_EVT_PRIM_SRVC_DISC_RSP event.
 *
 * @details Update service state and proceed to discovering the service's GATT characteristics.
 *
 * @param[in] p_ble_gattc_evt Primary Service Discovery Response Event.
 */
// static void on_service_discovery_response(const ble_gattc_evt_t *const p_ble_gattc_evt)
//{
//    int count;
//    int service_index;
//    const ble_gattc_service_t *service;
//
//    printf("Received service discovery response\n");
//    fflush(stdout);
//
//    if (p_ble_gattc_evt->gatt_status != NRF_SUCCESS)
//    {
//        printf("Service discovery failed. Error code 0x%X\n", p_ble_gattc_evt->gatt_status);
//        fflush(stdout);
//        return;
//    }
//
//    count = p_ble_gattc_evt->params.prim_srvc_disc_rsp.count;
//
//    if (count == 0)
//    {
//        printf("Service not found\n");
//        fflush(stdout);
//        return;
//    }
//
//    if (count > 1)
//    {
//        printf("Warning, discovered multiple primary services. Ignoring all but the first\n");
//    }
//
//    // service_index = 0; /* We expect to discover only the Heart Rate service as requested. */
//    // service       = &(p_ble_gattc_evt->params.prim_srvc_disc_rsp.services[service_index]);
//
//    for (int iServiceIndex = 0; iServiceIndex < count; iServiceIndex++)
//    {
//        service = &(p_ble_gattc_evt->params.prim_srvc_disc_rsp.services[iServiceIndex]);
//        printf("traverse service discovered with UUID: 0x%04X\n", service->uuid.uuid);
//        /*printf("traverse service discovered with UUID: 0x%02X 0x%02X\n",
//               p_ble_gattc_evt->params.attr_info_disc_rsp.info.attr_info128->uuid.uuid128[0],
//               p_ble_gattc_evt->params.attr_info_disc_rsp.info.attr_info128->uuid.uuid128[1]);*/
//        // p_ble_gattc_evt->params.attr_info_disc_rsp.info.attr_info128->uuid.uuid128[0];
//
//        m_service_end_handle = service->handle_range.end_handle;
//
//        //     if (service->uuid.uuid == 0x0001)
//        //     {
//        //         ble_uuid128_t srvc_uuid128;
//        //         // srvc_uuid128.uuid128 =
//        //         p_ble_gattc_evt->params.attr_info_disc_rsp_info.attr_info128; for (int iUUID128
//        =
//        //         0; iUUID128 < 16; iUUID128++)
//        //         {
//        //             srvc_uuid128.uuid128[iUUID128] =
//        //             p_ble_gattc_evt->params.attr_info_disc_rsp.info
//        //                                                  .attr_info128->uuid.uuid128[iUUID128];
//        //         }
//
//        // uint8_t *rsp_data     = (uint8_t *)p_ble_gattc_evt->params.read_rsp.data;
//        //         uint16_t rsp_data_len = p_ble_gattc_evt->params.read_rsp.len;
//        // uint8_t uuidData[16] = {0};
//        //         if (rsp_data_len == 16)
//        //         {
//        //             // Mask 16-bit UUID part to zeros.
//        //             // rsp_data[12] = 0x00;
//        //             // rsp_data[13] = 0x00;
//
//        //             // Copy gathered 128bit UUID as future base.
//        //             memcpy(uuidData, rsp_data, 16);
//        //             // err_code = sd_ble_uuid_vs_add((const ble_uuid128_t *)&m_uuid128base,
//        //             // &m_uuid128base_type);
//        //             printf("Read UUID is %s\n", hexStr(uuidData, 16).c_str());
//        //}
//        //         else{
//        //             printf("rsp of data len is %d\n", rsp_data_len);
//        //	}
//
//        //         printf("Discovered vender service. UUID: 0x%32X, "
//        //                "start handle: 0x%04X, end handle: 0x%04X\n",
//        //                srvc_uuid128.uuid128, service->handle_range.start_handle,
//        //                m_service_end_handle);
//
//        //         if (service->uuid.type == BLE_UUID_TYPE_UNKNOWN)
//        //         {
//        //             uint32_t err_code = sd_ble_gattc_read(m_adapter, m_connection_handle,
//        //                                                   service->handle_range.start_handle,
//        0);
//        //             if (err_code != NRF_SUCCESS)
//        //             {
//        //                 printf("sd_ble_gattc_read failed. Error code 0x%X\n", err_code);
//        //             }
//        //         }
//        //         if (service->uuid.type == BLE_UUID_TYPE_VENDOR_BEGIN)
//        //         {
//        //             uint32_t err_code = sd_ble_gattc_read(m_adapter, m_connection_handle,
//        //                                                   service->handle_range.start_handle,
//        0);
//        //             if (err_code != NRF_SUCCESS)
//        //             {
//        //                 printf("sd_ble_gattc_read failed. Error code 0x%X\n", err_code);
//        //             }
//        //         }
//        //     }
//
//        if (service->uuid.uuid == BLE_UUID_BATTERY_SERVICE)
//        {
//            m_service_start_handle = service->handle_range.start_handle;
//            // m_service_end_handle = service->handle_range.end_handle;
//
//            printf("Discovered BATTERY service. UUID: 0x%04X, "
//                   "start handle: 0x%04X, end handle: 0x%04X\n",
//                   service->uuid.uuid, m_service_start_handle, m_service_end_handle);
//            fflush(stdout);
//
//            char_discovery_start();
//
//            ble_uuid_t srvc_uuid;
//            srvc_uuid.type = BLE_UUID_TYPE_BLE;
//            srvc_uuid.uuid = 0x2a19;
//
//            ble_gattc_handle_range_t handle_range;
//
//            handle_range.start_handle = m_service_start_handle;
//            handle_range.end_handle   = m_service_end_handle;
//
//            sd_ble_gattc_char_value_by_uuid_read(m_adapter, m_connection_handle, &srvc_uuid,
//                                                 &handle_range);
//        }
//        else if (service->uuid.uuid == BLE_UUID_DEVICE_INFO_SERVICE)
//        {
//            m_service_start_handle = service->handle_range.start_handle;
//            // m_service_end_handle = service->handle_range.end_handle;
//
//            printf("Discovered DEVICE_INFO service. UUID: 0x%04X, "
//                   "start handle: 0x%04X, end handle: 0x%04X\n",
//                   service->uuid.uuid, m_service_start_handle, m_service_end_handle);
//            fflush(stdout);
//
//            char_discovery_start();
//
//            ble_uuid_t srvc_uuid;
//            srvc_uuid.type = BLE_UUID_TYPE_BLE;
//            srvc_uuid.uuid = 0x2a50;
//
//            ble_gattc_handle_range_t handle_range;
//
//            handle_range.start_handle = m_service_start_handle;
//            handle_range.end_handle   = m_service_end_handle;
//
//            sd_ble_gattc_char_value_by_uuid_read(m_adapter, m_connection_handle, &srvc_uuid,
//                                                 &handle_range);
//        }
//        else if (service->uuid.uuid == HOGP_UUID_SERVICE)
//        {
//            m_service_start_handle = service->handle_range.start_handle;
//            // m_service_end_handle = service->handle_range.end_handle;
//
//            printf("Discovered HOGP service. UUID: 0x%04X, "
//                   "start handle: 0x%04X, end handle: 0x%04X\n",
//                   service->uuid.uuid, m_service_start_handle, m_service_end_handle);
//            fflush(stdout);
//
//            char_discovery_start();
//
//            /* ble_uuid_t srvc_uuid;
//            srvc_uuid.type = BLE_UUID_TYPE_BLE;
//            srvc_uuid.uuid = 0x2a50;
//
//            ble_gattc_handle_range_t handle_range;
//
//            handle_range.start_handle = m_service_start_handle;
//            handle_range.end_handle   = m_service_end_handle;
//
//            sd_ble_gattc_char_value_by_uuid_read(m_adapter, m_connection_handle, &srvc_uuid,
//                                                 &handle_range); */
//        }
//        else
//        {
//            m_service_start_handle = service->handle_range.start_handle;
//            m_service_end_handle   = service->handle_range.end_handle;
//
//            printf("Discovered other service. UUID: 0x%04X, "
//                   "start handle: 0x%04X, end handle: 0x%04X\n",
//                   service->uuid.uuid, m_service_start_handle, m_service_end_handle);
//            fflush(stdout);
//
//            ble_uuid_t srvc_uuid;
//            srvc_uuid.type = BLE_UUID_TYPE_BLE;
//            srvc_uuid.uuid = service->uuid.uuid;
//
//            ble_gattc_handle_range_t handle_range;
//
//            handle_range.start_handle = m_service_start_handle;
//            handle_range.end_handle   = m_service_end_handle;
//
//            sd_ble_gattc_char_value_by_uuid_read(m_adapter, m_connection_handle, &srvc_uuid,
//                                                 &handle_range);
//        }
//    }
//
//    /* if (service->uuid.uuid != BLE_UUID_HOGP_SERVICE)
//     {
//         printf("Unknown service discovered with UUID: 0x%04X\n", service->uuid.uuid);
//         fflush(stdout);
//         return;
//     }
//
//     m_service_start_handle = service->handle_range.start_handle;
//     m_service_end_handle   = service->handle_range.end_handle;
//
//     printf("Discovered heart rate service. UUID: 0x%04X, "
//            "start handle: 0x%04X, end handle: 0x%04X\n",
//            service->uuid.uuid, m_service_start_handle, m_service_end_handle);
//     fflush(stdout);
//
//     char_discovery_start();*/
//}

static void on_service_discovery_response(const ble_gattc_evt_t *const p_ble_gattc_evt)
{
    int count;
    int service_index;
    const ble_gattc_service_t *service;

    printf("Received service discovery response\n");
    fflush(stdout);

    if (p_ble_gattc_evt->gatt_status != NRF_SUCCESS)
    {
        printf("Service discovery failed. Error code 0x%X\n", p_ble_gattc_evt->gatt_status);
        fflush(stdout);
        return;
    }

    count = p_ble_gattc_evt->params.prim_srvc_disc_rsp.count;

    if (count == 0)
    {
        printf("Service not found\n");
        fflush(stdout);
        return;
    }

    if (count > 1)
    {
        printf("Warning, discovered multiple primary services. Ignoring all but the first\n");
    }

    // service_index = 0; /* We expect to discover only the Heart Rate service as requested. */
    // service       = &(p_ble_gattc_evt->params.prim_srvc_disc_rsp.services[service_index]);

    for (int iServiceIndex = 0; iServiceIndex < count; iServiceIndex++)
    {
        service = &(p_ble_gattc_evt->params.prim_srvc_disc_rsp.services[iServiceIndex]);

        /*printf("traverse service discovered with UUID: 0x%02X 0x%02X\n",
               p_ble_gattc_evt->params.attr_info_disc_rsp.info.attr_info128->uuid.uuid128[0],
               p_ble_gattc_evt->params.attr_info_disc_rsp.info.attr_info128->uuid.uuid128[1]);*/
        // p_ble_gattc_evt->params.attr_info_disc_rsp.info.attr_info128->uuid.uuid128[0];

        m_service_end_handle = service->handle_range.end_handle;
        printf("traverse service discovered with UUID: 0x%04X;  handle 0x%04X\n",
               service->uuid.uuid, m_service_end_handle);

        if (service->uuid.uuid == 0x0001 || service->uuid.type == BLE_UUID_TYPE_UNKNOWN)
        {
            ble_uuid128_t srvc_uuid128;
            // srvc_uuid128.uuid128 = p_ble_gattc_evt->params.attr_info_disc_rsp_info.attr_info128;
            for (int iUUID128 = 0; iUUID128 < 16; iUUID128++)
            {
                srvc_uuid128.uuid128[iUUID128] = p_ble_gattc_evt->params.attr_info_disc_rsp.info
                                                     .attr_info128->uuid.uuid128[iUUID128];
            }

            uint8_t *rsp_data     = (uint8_t *)p_ble_gattc_evt->params.read_rsp.data;
            uint16_t rsp_data_len = p_ble_gattc_evt->params.read_rsp.len;
            uint8_t uuidData[16]  = {0};
            if (rsp_data_len == 16)
            {
                // Mask 16-bit UUID part to zeros.
                // rsp_data[12] = 0x00;
                // rsp_data[13] = 0x00;

                // Copy gathered 128bit UUID as future base.
                memcpy(uuidData, rsp_data, 16);
                // err_code = sd_ble_uuid_vs_add((const ble_uuid128_t *)&m_uuid128base,
                // &m_uuid128base_type);
                printf("Read UUID is %s\n", hexStr(uuidData, 16).c_str());
            }
            else
            {
                printf("rsp of data len is %d\n", rsp_data_len);
            }

            m_service_start_handle = service->handle_range.start_handle;
            m_service_end_handle   = service->handle_range.end_handle;
            char_discovery_start();
            printf("Discovered vender service. UUID: 0x%32X, "
                   "start handle: 0x%04X, end handle: 0x%04X\n",
                   srvc_uuid128.uuid128, service->handle_range.start_handle, m_service_end_handle);
            _sleep(1000);
            if (service->uuid.type == BLE_UUID_TYPE_UNKNOWN)
            {
                uint32_t err_code;
                err_code = sd_ble_gattc_read(m_adapter, m_connection_handle,
                                             service->handle_range.start_handle, 0);

                if (err_code != NRF_SUCCESS)
                {
                    printf("sd_ble_gattc_read BLE_UUID_TYPE_UNKNOWN failed. Error code 0x%X\n",
                           err_code);
                }
                else
                {
                    printf("read BLE_UUID_TYPE_UNKNOWN");
                }
            }
            if (service->uuid.type == BLE_UUID_TYPE_VENDOR_BEGIN)
            {
                uint32_t err_code;
                err_code = sd_ble_gattc_read(m_adapter, m_connection_handle,
                                             service->handle_range.start_handle,
                                             0); // can get UUID from here
                                                 // can get UUID from here
                                                 // can get UUID from here
                if (err_code != NRF_SUCCESS)
                {
                    printf("sd_ble_gattc_read BLE_UUID_TYPE_VENDOR_BEGIN failed. Error code 0x%X\n",
                           err_code);
                }
                else
                {
                    printf("read BLE_UUID_TYPE_VENDOR_BEGIN");
                }
                for (int iRead = m_service_start_handle; iRead <= m_service_end_handle; iRead++)
                {
                    //_sleep(1000);
                    m_service_start_handle = iRead;
                    char_discovery_start();
                    /*err_code = sd_ble_gattc_read(m_adapter, m_connection_handle, iRead, 0);
                    if (err_code != NRF_SUCCESS)
                    {
                        printf("sd_ble_gattc_read in loop BLE_UUID_TYPE_VENDOR_BEGIN failed. Error
                    code " "0x%X\n", err_code); }else{ printf("read in loop
                    BLE_UUID_TYPE_VENDOR_BEGIN");
                                                }*/
                }
            }
        }

        if (service->uuid.uuid == BLE_UUID_BATTERY_SERVICE)
        {
            m_service_start_handle = service->handle_range.start_handle;
            // m_service_end_handle = service->handle_range.end_handle;

            printf("Discovered BATTERY service. UUID: 0x%04X, "
                   "start handle: 0x%04X, end handle: 0x%04X\n",
                   service->uuid.uuid, m_service_start_handle, m_service_end_handle);
            fflush(stdout);

            char_discovery_start();

            ble_uuid_t srvc_uuid;
            srvc_uuid.type = BLE_UUID_TYPE_BLE;
            srvc_uuid.uuid = 0x2a19;

            ble_gattc_handle_range_t handle_range;

            handle_range.start_handle = m_service_start_handle;
            handle_range.end_handle   = m_service_end_handle;

            sd_ble_gattc_char_value_by_uuid_read(m_adapter, m_connection_handle, &srvc_uuid,
                                                 &handle_range);
        }
        else if (service->uuid.uuid == BLE_UUID_DEVICE_INFO_SERVICE)
        {
            m_service_start_handle = service->handle_range.start_handle;
            // m_service_end_handle = service->handle_range.end_handle;

            printf("Discovered DEVICE_INFO service. UUID: 0x%04X, "
                   "start handle: 0x%04X, end handle: 0x%04X\n",
                   service->uuid.uuid, m_service_start_handle, m_service_end_handle);
            fflush(stdout);

            char_discovery_start();

            ble_uuid_t srvc_uuid;
            srvc_uuid.type = BLE_UUID_TYPE_BLE;
            srvc_uuid.uuid = 0x2a50;

            ble_gattc_handle_range_t handle_range;

            handle_range.start_handle = m_service_start_handle;
            handle_range.end_handle   = m_service_end_handle;

            sd_ble_gattc_char_value_by_uuid_read(m_adapter, m_connection_handle, &srvc_uuid,
                                                 &handle_range);
        }
        else if (service->uuid.uuid == HOGP_UUID_SERVICE)
        {
            m_service_start_handle = service->handle_range.start_handle;
            // m_service_end_handle = service->handle_range.end_handle;

            printf("Discovered HOGP service. UUID: 0x%04X, "
                   "start handle: 0x%04X, end handle: 0x%04X\n",
                   service->uuid.uuid, m_service_start_handle, m_service_end_handle);
            fflush(stdout);

            char_discovery_start();

            ble_uuid_t srvc_uuid;
            srvc_uuid.type = BLE_UUID_TYPE_BLE;
            // srvc_uuid.uuid = 0x2a50;
            srvc_uuid.uuid = 0x1812;

            ble_gattc_handle_range_t handle_range;

            handle_range.start_handle = m_service_start_handle;
            handle_range.end_handle   = m_service_end_handle;

            // sd_ble_gattc_read(m_adapter, m_connection_handle, m_service_start_handle, 0);

            sd_ble_gattc_char_value_by_uuid_read(m_adapter, m_connection_handle, &srvc_uuid,
                                                 &handle_range);
        }
        else
        {
            m_service_start_handle = service->handle_range.start_handle;
            m_service_end_handle   = service->handle_range.end_handle;

            printf("Discovered other service. UUID: 0x%04X, "
                   "start handle: 0x%04X, end handle: 0x%04X\n",
                   service->uuid.uuid, m_service_start_handle, m_service_end_handle);
            fflush(stdout);

            ble_uuid_t srvc_uuid;
            srvc_uuid.type = BLE_UUID_TYPE_BLE;
            srvc_uuid.uuid = service->uuid.uuid;

            ble_gattc_handle_range_t handle_range;

            handle_range.start_handle = m_service_start_handle;
            handle_range.end_handle   = m_service_end_handle;

            /*for (int iRead = m_service_start_handle; iRead <= m_service_end_handle; iRead++)
            {
                _sleep(1000);
                sd_ble_gattc_read(m_adapter, m_connection_handle, iRead, 0);
            }*/

            /*sd_ble_gattc_char_value_by_uuid_read(m_adapter, m_connection_handle, &srvc_uuid,
                                                 &handle_range);*/
        }
    }

    /* if (service->uuid.uuid != BLE_UUID_HOGP_SERVICE)
     {Discovered vender service. UUID:
         printf("Unknown service discovered with UUID: 0x%04X\n", service->uuid.uuid);
         fflush(stdout);
         return;
     }

     m_service_start_handle = service->handle_range.start_handle;
     m_service_end_handle   = service->handle_range.end_handle;

     printf("Discovered heart rate service. UUID: 0x%04X, "
            "start handle: 0x%04X, end handle: 0x%04X\n",
            service->uuid.uuid, m_service_start_handle, m_service_end_handle);
     fflush(stdout);

     char_discovery_start();*/

    // findMoreService(m_service_end_handle + 1);
}

/**@brief Function called on BLE_GATTC_EVT_CHAR_DISC_RSP event.
 *
 * @details Update characteristic state and proceed to discovering the characteristicss
 * descriptors.
 *
 * @param[in] p_ble_gattc_evt Characteristic Discovery Response Event.
 */
static uint32_t char_discovery_more_start(uint16_t m_service_more_handle)
{
    ble_gattc_handle_range_t handle_range;

    printf("Discovering more characteristics, start handle: 0x%04X, end handle: 0x%04X\n",
           m_service_more_handle, m_service_end_handle);
    fflush(stdout);

    handle_range.start_handle = m_service_more_handle;
    handle_range.end_handle   = m_service_end_handle;

    uint32_t error_code;
    error_code =
        sd_ble_gattc_characteristics_discover(m_adapter, m_connection_handle, &handle_range);

    if (error_code != NRF_SUCCESS)
    {
        printf("fail to discover characteristics\n");
        fflush(stdout);
    }
    return error_code;
}
static void on_characteristic_discovery_response(const ble_gattc_evt_t *const p_ble_gattc_evt)
{
    int count = p_ble_gattc_evt->params.char_disc_rsp.count;

    if (p_ble_gattc_evt->gatt_status != NRF_SUCCESS)
    {
        printf("Characteristic discovery failed. Error code 0x%X\n", p_ble_gattc_evt->gatt_status);
        fflush(stdout);
        return;
    }

    printf("Received characteristic discovery response, characteristics count: %d\n", count);
    fflush(stdout);

    for (int i = 0; i < count; i++)
    {
        printf("Characteristic handle: 0x%04X, UUID: 0x%04X\n",
               p_ble_gattc_evt->params.char_disc_rsp.chars[i].handle_decl,
               p_ble_gattc_evt->params.char_disc_rsp.chars[i].uuid.uuid);
        fflush(stdout);

        moreHandle_Start    = p_ble_gattc_evt->params.char_disc_rsp.chars[i].handle_decl;
        m_desc_start_handle = p_ble_gattc_evt->params.char_disc_rsp.chars[i].handle_decl;

        if (p_ble_gattc_evt->params.char_disc_rsp.chars[i].uuid.uuid == 0x0003)
        {
            uint32_t err_code =
                sd_ble_gattc_read(m_adapter, m_connection_handle, m_desc_start_handle, 0);
            if (err_code != NRF_SUCCESS)
            {
                printf("sd_ble_gattc_read in DISC_RSP failed. Error code 0x%X\n", err_code);
            }
        }

        if (p_ble_gattc_evt->params.char_disc_rsp.chars[i].uuid.uuid ==
            BLE_UUID_HEART_RATE_MEASUREMENT_CHAR)
        {
            m_hrm_char_handle = p_ble_gattc_evt->params.char_disc_rsp.chars[i].handle_decl;
        }

        if (p_ble_gattc_evt->params.char_disc_rsp.chars[i].uuid.uuid == 0x0002)
        {
            m_voice_handle = p_ble_gattc_evt->params.char_disc_rsp.chars[i]
                                 .handle_value; // write handle, + 1 notification handle
            printf("voice handle is 0x%X\n", m_voice_handle);
        }

        if (p_ble_gattc_evt->params.char_disc_rsp.chars[i].uuid.uuid == BLE_UUID_HID_UUID)
        {
            // EnableHOGPNotice(moreHandle_Start + 1);
            bool foundRepeatHandle = false;
            for (size_t i = 0; i < vecHIDhandle.size(); i++)
            {
                if (moreHandle_Start == vecHIDhandle[i])
                    foundRepeatHandle = true;
            }
            if (!foundRepeatHandle)
            {
                vecHIDhandle.push_back(moreHandle_Start);
                // EnableHOGPNotice(moreHandle_Start);//will disturb next cycle
            }
            printf("FOUND HID handle: 0x%04X, UUID: 0x%04X\n",
                   p_ble_gattc_evt->params.char_disc_rsp.chars[i].handle_decl,
                   p_ble_gattc_evt->params.char_disc_rsp.chars[i].uuid.uuid);
        }
        //     if (moreHandle_Start != 0x000F)
        //    {
        //         if (moreHandle_Start != 0x0039)
        //        {
        //            if ((m_service_end_handle - moreHandle_Start > 4))
        //                // char_discovery_more_start();

        //                char_discovery_more_start(moreHandle_Start);
        //        //}
        ////        else
        ////        {
        ////            if ((m_service_end_handle - moreHandle_Start > 2))
        ////                // char_discovery_more_start();

        ////                char_discovery_more_start(moreHandle_Start);
        //}
        //    }
    }
    if (moreHandle_Start != 0x000F)
    {
        if (moreHandle_Start < 0x0039)
        {
            if ((m_service_end_handle - moreHandle_Start > 4))
                // char_discovery_more_start();

                char_discovery_more_start(moreHandle_Start);
            //}
            //        else
            //        {
            //            if ((m_service_end_handle - moreHandle_Start > 2))
            //                // char_discovery_more_start();

            //                char_discovery_more_start(moreHandle_Start);
        }
        else if (moreHandle_Start == 0x0039)
        {
            EnableAllHIDService();
            service_discovery_voice();
        }
        /* else if (moreHandle_Start == 0x55)
         {
             char_discovery_more_start(moreHandle_Start);
                         }*/
    }

    // descr_discovery_start();
}

/**@brief Function called on BLE_GATTC_EVT_DESC_DISC_RSP event.
 *
 * @details Update CCCD descriptor state and proceed to prompting user to toggle notifications.
 *
 * @param[in] p_ble_gattc_evt Descriptor Discovery Response Event.
 */
static void on_descriptor_discovery_response(const ble_gattc_evt_t *const p_ble_gattc_evt)
{
    int count = p_ble_gattc_evt->params.desc_disc_rsp.count;

    if (p_ble_gattc_evt->gatt_status != NRF_SUCCESS)
    {
        printf("Descriptor discovery failed. Error code 0x%X\n", p_ble_gattc_evt->gatt_status);
        fflush(stdout);
        return;
    }

    printf("Received descriptor discovery response, descriptor count: %d\n", count);
    fflush(stdout);

    for (int i = 0; i < count; i++)
    {
        printf("Descriptor handle: 0x%04X, UUID: 0x%04X\n",
               p_ble_gattc_evt->params.desc_disc_rsp.descs[i].handle,
               p_ble_gattc_evt->params.desc_disc_rsp.descs[i].uuid.uuid);
        fflush(stdout);

        if (p_ble_gattc_evt->params.desc_disc_rsp.descs[i].uuid.uuid == BLE_UUID_CCCD)
        {
            m_hrm_cccd_handle = p_ble_gattc_evt->params.desc_disc_rsp.descs[i].handle;
            printf("Press enter to toggle notifications on the HRM characteristic\n");
            fflush(stdout);
        }
    }
}

static uint8_t batteryLvlValue[1] = {0};
static uint8_t readvalue[20]      = {0};
static void on_read_response(const ble_gattc_evt_t *const p_ble_gattc_evt)
{
    printf("Received read response.\n");
    fflush(stdout);

    if (p_ble_gattc_evt->gatt_status != NRF_SUCCESS)
    {
        printf("Error. read operation failed. Error code 0x%X\n", p_ble_gattc_evt->gatt_status);
        fflush(stdout);
    }
    else
    {
        // batteryLvlValue[0] = p_ble_gattc_evt->params.char_vals_read_rsp.values[0];
        // batteryLvlValue[1] = p_ble_gattc_evt->params.char_vals_read_rsp.values[1];

        // printf("Received battery lvl measurement: %d\n", batteryLvlValue[0]);
        // printf("Received battery lvl measurement: %d\n", batteryLvlValue[1]);

        // Response should contain full 128-bit UUID.
        uint8_t *rsp_data     = (uint8_t *)p_ble_gattc_evt->params.read_rsp.data;
        uint16_t rsp_data_len = p_ble_gattc_evt->params.read_rsp.len;
        uint8_t uuidData[16]  = {0};
        if (rsp_data_len == 16)
        {
            // Mask 16-bit UUID part to zeros.
            // rsp_data[12] = 0x00;
            // rsp_data[13] = 0x00;

            // Copy gathered 128bit UUID as future base.
            memcpy(uuidData, rsp_data, 16);
            // err_code = sd_ble_uuid_vs_add((const ble_uuid128_t *)&m_uuid128base,
            // &m_uuid128base_type);
            printf("Read UUID of readResp is %s\n", hexStr(uuidData, 16).c_str());
            printf("Read Handle of readResp is %02X\n", p_ble_gattc_evt->params.read_rsp.handle);
            if (hexStr(uuidData, 16) == "64b617f601af7dbc054f215a01005eab")
            {
                printf("write voice notification of %d.\n",
                       p_ble_gattc_evt->params.read_rsp.handle + 5);
                EnableVoiceNotice(p_ble_gattc_evt->params.read_rsp.handle + 5);
            }
            /*if (m_service_end_handle < 0xFFFF)
                findMoreService(p_ble_gattc_evt->params.read_rsp.handle);*/

            /*p_ble_gattc_evt->params.read_rsp.handle
            printf("FOUND HID handle: 0x%04X, UUID: 0x%04X\n",
       p_ble_gattc_evt->params.char_disc_rsp.chars[i].handle_decl,
       p_ble_gattc_evt->params.char_disc_rsp.chars[i].uuid.uuid);*/
        }
        else
        {
            printf("Read UUID of readResp is %s\n", hexStr(uuidData, 2).c_str());
            /*if (m_service_end_handle < 0xFFFF)
                findMoreService(p_ble_gattc_evt->params.read_rsp.handle);*/
        }
        // p_ble_gattc_evt->params

        /*if (p_ble_gattc_evt->params.read_rsp.handle == m_hrm_char_handle + 1)
        {
            for (uint16_t x = p_ble_gattc_evt->params.read_rsp.offset;
                 x < p_ble_gattc_evt->params.read_rsp.offset + 1; x++)
            {
                readvalue[x] = p_ble_gattc_evt->params.read_rsp
                                   .data[x - p_ble_gattc_evt->params.read_rsp.offset];

                printf("Received battery lvl measurement: %d\n", readvalue[x]);
                printf("offset is : %d\n", x);
            }
        }

        if (p_ble_gattc_evt->params.read_rsp.handle == m_device_info_handle + 1)
        {
            for (uint16_t x = p_ble_gattc_evt->params.read_rsp.offset;
                 x < p_ble_gattc_evt->params.read_rsp.offset + 1; x++)
            {
                readvalue[x] = p_ble_gattc_evt->params.read_rsp
                                   .data[x - p_ble_gattc_evt->params.read_rsp.offset];

                printf("Received device info value : %d\n", readvalue[x]);
                printf("offset is : %d\n", x);
            }
        }*/
    }
}
static void on_read_response_by_uuid(const ble_gattc_evt_t *const p_ble_gattc_evt)
{
    printf("Received read response.\n");
    fflush(stdout);

    if (p_ble_gattc_evt->gatt_status != NRF_SUCCESS)
    {
        printf("Error. read operation failed. Error code 0x%X\n", p_ble_gattc_evt->gatt_status);
        fflush(stdout);
    }
    else
    {
        // Response should contain full 128-bit UUID.
        uint8_t *rsp_data     = (uint8_t *)p_ble_gattc_evt->params.read_rsp.data;
        uint16_t rsp_data_len = p_ble_gattc_evt->params.read_rsp.len;
        uint8_t uuidData[16]  = {0};
        if (rsp_data_len == 16)
        {
            // Copy gathered 128bit UUID as future base.
            memcpy(uuidData, rsp_data, 16);
            printf("Read UUID of readRespByUUID is %s\n", hexStr(uuidData, 16).c_str());
        }
        else
        {
            printf("Read UUID of readRespByUUID is %s\n", hexStr(uuidData, 2).c_str());
        }
    }
}

/**@brief Function called on BLE_GATTC_EVT_WRITE_RSP event.
 *
 * @param[in] p_ble_gattc_evt Write Response Event.
 */
static void on_write_response(const ble_gattc_evt_t *const p_ble_gattc_evt)
{
    printf("Received write response.\n");
    fflush(stdout);

    if (p_ble_gattc_evt->gatt_status != NRF_SUCCESS)
    {
        printf("Error. Write operation failed. Error code 0x%X\n", p_ble_gattc_evt->gatt_status);
        fflush(stdout);
    }
}

/**@brief Function called on BLE_GATTC_EVT_HVX event.
 *
 * @details Logs the received heart rate measurement.
 *
 * @param[in] p_ble_gattc_evt Handle Value Notification/Indication Event.
 */
std::string GetBinFileName_CurrentTime()
{
    time_t now = time(0);
    struct tm tstruct;
    char buf[80];
    tstruct = *localtime(&now);
    strftime(buf, sizeof(buf), "%m%d_%H%M%S.bin", &tstruct);
    return buf;
}
std::string GetWaveFileName_CurrentTime()
{
    time_t now = time(0);
    struct tm tstruct;
    char buf[80];
    tstruct = *localtime(&now);
    strftime(buf, sizeof(buf), "%m%d_%H%M%S.wav", &tstruct);
    return buf;
}
static void on_hvx(const ble_gattc_evt_t *const p_ble_gattc_evt)
{
    static int nCount = 0;
    // printf("Received notification from hanld : %2x\n", p_ble_gattc_evt->params.hvx.handle);
    if (p_ble_gattc_evt->params.hvx.handle >= m_hrm_char_handle ||
        p_ble_gattc_evt->params.hvx.handle <= m_hrm_cccd_handle ||
        p_ble_gattc_evt->params.hvx.handle == m_voice_handle ||
        p_ble_gattc_evt->params.hvx.handle == m_voice_handle + 2) // Heart rate measurement.
    {
        // We know the heart rate reading is encoded as 2 bytes [flag, value].
        int nLen = p_ble_gattc_evt->params.hvx.len;
        // printf("Received: %d\n", p_ble_gattc_evt->params.hvx.len);
        switch (nLen)
        {
            case 1:
            {
                printf("Received : %2x\n", p_ble_gattc_evt->params.hvx.data[0]);
            }
            break;
            case 2:
            {
                printf("Received: %2x %2x\n", p_ble_gattc_evt->params.hvx.data[0],
                       p_ble_gattc_evt->params.hvx.data[1]);
            }
            break;
            case 3:
            {
                printf("Received: %2x %2x %2x\n", p_ble_gattc_evt->params.hvx.data[0],
                       p_ble_gattc_evt->params.hvx.data[1], p_ble_gattc_evt->params.hvx.data[2]);
            }
            break;
            case 4:
            {
                printf("Received: %2x %2x %2x %2x\n", p_ble_gattc_evt->params.hvx.data[0],
                       p_ble_gattc_evt->params.hvx.data[1], p_ble_gattc_evt->params.hvx.data[2],
                       p_ble_gattc_evt->params.hvx.data[3]);
                if ((p_ble_gattc_evt->params.hvx.data[0] == 0x21) &&
                    (p_ble_gattc_evt->params.hvx.data[1] == 0x02))
                {
                    std::ofstream adpcmOutFile("C:\\1\\adpcm.bin", std::ios::trunc);
                    adpcmOutFile.close();
                    OpenMic();
                }
            }
            break;
            case 5:
            {
                printf("Received: %2x %2x %2x %2x %2x\n", p_ble_gattc_evt->params.hvx.data[0],
                       p_ble_gattc_evt->params.hvx.data[1], p_ble_gattc_evt->params.hvx.data[2],
                       p_ble_gattc_evt->params.hvx.data[3], p_ble_gattc_evt->params.hvx.data[4]);
            }
            break;
            case 6:
            {
                printf("Received: %2x %2x %2x %2x %2x %2x\n", p_ble_gattc_evt->params.hvx.data[0],
                       p_ble_gattc_evt->params.hvx.data[1], p_ble_gattc_evt->params.hvx.data[2],
                       p_ble_gattc_evt->params.hvx.data[3], p_ble_gattc_evt->params.hvx.data[4],
                       p_ble_gattc_evt->params.hvx.data[5]);
            }
            break;
            case 7:
            {
                printf("Received: %2x %2x %2x %2x %2x %2x %2x\n",
                       p_ble_gattc_evt->params.hvx.data[0], p_ble_gattc_evt->params.hvx.data[1],
                       p_ble_gattc_evt->params.hvx.data[2], p_ble_gattc_evt->params.hvx.data[3],
                       p_ble_gattc_evt->params.hvx.data[4], p_ble_gattc_evt->params.hvx.data[5],
                       p_ble_gattc_evt->params.hvx.data[6]);
            }
            break;
            case 8:
            {
                printf("Received: %2x %2x %2x %2x %2x %2x %2x %2x\n",
                       p_ble_gattc_evt->params.hvx.data[0], p_ble_gattc_evt->params.hvx.data[1],
                       p_ble_gattc_evt->params.hvx.data[2], p_ble_gattc_evt->params.hvx.data[3],
                       p_ble_gattc_evt->params.hvx.data[4], p_ble_gattc_evt->params.hvx.data[5],
                       p_ble_gattc_evt->params.hvx.data[6], p_ble_gattc_evt->params.hvx.data[7]);
            }
            break;
            case 9:
            {
                printf("Received: %2x %2x %2x %2x %2x %2x %2x %2x %2x\n",
                       p_ble_gattc_evt->params.hvx.data[0], p_ble_gattc_evt->params.hvx.data[1],
                       p_ble_gattc_evt->params.hvx.data[2], p_ble_gattc_evt->params.hvx.data[3],
                       p_ble_gattc_evt->params.hvx.data[4], p_ble_gattc_evt->params.hvx.data[5],
                       p_ble_gattc_evt->params.hvx.data[6], p_ble_gattc_evt->params.hvx.data[7],
                       p_ble_gattc_evt->params.hvx.data[8]);
            }
            break;
            case 10:
            {
                printf("Received: %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x\n",
                       p_ble_gattc_evt->params.hvx.data[0], p_ble_gattc_evt->params.hvx.data[1],
                       p_ble_gattc_evt->params.hvx.data[2], p_ble_gattc_evt->params.hvx.data[3],
                       p_ble_gattc_evt->params.hvx.data[4], p_ble_gattc_evt->params.hvx.data[5],
                       p_ble_gattc_evt->params.hvx.data[6], p_ble_gattc_evt->params.hvx.data[7],
                       p_ble_gattc_evt->params.hvx.data[8], p_ble_gattc_evt->params.hvx.data[9]);
            }
            break;
            case 134:
            {
                printf("Received: %2x %2x %2x %2x %2x %2x %2x %2x %2x %2x\n",
                       p_ble_gattc_evt->params.hvx.data[0], p_ble_gattc_evt->params.hvx.data[1],
                       p_ble_gattc_evt->params.hvx.data[2], p_ble_gattc_evt->params.hvx.data[3],
                       p_ble_gattc_evt->params.hvx.data[4], p_ble_gattc_evt->params.hvx.data[5],
                       p_ble_gattc_evt->params.hvx.data[6], p_ble_gattc_evt->params.hvx.data[7],
                       p_ble_gattc_evt->params.hvx.data[8], p_ble_gattc_evt->params.hvx.data[9]);
                char dataADPCM[134];
                for (int iDataADPCM = 0; iDataADPCM < 134; iDataADPCM++)
                {
                    dataADPCM[iDataADPCM] = p_ble_gattc_evt->params.hvx.data[iDataADPCM];
                }
                string adpcmFileName = "C:\\1\\adpcm.bin";
                static std::ofstream adpcmOutFile(adpcmFileName.c_str(),
                                                  std::ios::out | std::ios::app | std::ios::binary |
                                                      std::ios::ate);
                try
                {
                    // adpcmOutFile.open("C:\\1\\adpcm.txt", std::ios::app | std::ios::binary |
                    // std::ios::ate);
                    if (adpcmOutFile.is_open())
                    {
                        adpcmOutFile.write(dataADPCM, 134);
                        // adpcmOutFile << dataADPCM;
                    }
                    else
                    {
                        adpcmOutFile.open("C:\\1\\adpcm.bin",
                                          std::ios::app | std::ios::binary | std::ios::ate);
                        adpcmOutFile.write(dataADPCM, 134);
                    }
                }
                catch (std::exception ex)
                {
                    printf("exception in write file %s.\n", ex.what());
                }

                nCount++;
                if (nCount == 200)
                {
                    CloseMic();
                    adpcmOutFile.close();
                    nCount = 0;

                    // decode
                    string cmd = "C:\\1\\ADPCM_PCM.exe c:\\1\\adpcm.bin C:\\1\\";
                    cmd += GetWaveFileName_CurrentTime();
                    cmd += " 16000";
                    std::system(cmd.c_str());
                }
                // volice
            }
            break;
            default:
            {
                printf("Received length is %d\n", p_ble_gattc_evt->params.hvx.len);
            }
        }
    }
    else // Unknown data.
    {
        printf("Un-parsed data received on handle: %04X\n", p_ble_gattc_evt->params.hvx.handle);
    }

    fflush(stdout);
}

/**@brief Function called on BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST event.
 *
 * @details Update GAP connection parameters.
 *
 * @param[in] p_ble_gap_evt Connection Parameter Update Event.
 */
static void on_conn_params_update_request(const ble_gap_evt_t *const p_ble_gap_evt)
{
    uint32_t err_code = sd_ble_gap_conn_param_update(
        m_adapter, m_connection_handle,
        &(p_ble_gap_evt->params.conn_param_update_request.conn_params));
    if (err_code != NRF_SUCCESS)
    {
        printf("Conn params update failed, err_code %d\n", err_code);
        fflush(stdout);
    }
    else
    {
        printf("Conn params update successfully!\n");
        fflush(stdout);
	}
}

#if NRF_SD_BLE_API >= 3
/**@brief Function called on BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST event.
 *
 * @details Replies to an ATT_MTU exchange request by sending an Exchange MTU Response to the
 * client.
 *
 * @param[in] p_ble_gatts_evt Exchange MTU Request Event.
 */
static void on_exchange_mtu_request(const ble_gatts_evt_t *const p_ble_gatts_evt)
{
    uint32_t err_code = sd_ble_gatts_exchange_mtu_reply(m_adapter, m_connection_handle,
#if NRF_SD_BLE_API < 5
                                                        GATT_MTU_SIZE_DEFAULT);
#else
                                                        150); // modify by Macl ,
                                                              // BLE_GATT_ATT_MTU_DEFAULT
#endif

    if (err_code != NRF_SUCCESS)
    {
        printf("MTU exchange request reply failed, err_code %d\n", err_code);
        fflush(stdout);
    }
    else
    {
        printf("MTU exchange successfully!\n");
        fflush(stdout);
    }
}

/**@brief Function called on BLE_GATTC_EVT_EXCHANGE_MTU_RSP event.
 *
 * @details Logs the new BLE server RX MTU size.
 *
 * @param[in] p_ble_gattc_evt Exchange MTU Response Event.
 */
static void on_exchange_mtu_response(const ble_gattc_evt_t *const p_ble_gattc_evt)
{
    uint16_t server_rx_mtu = p_ble_gattc_evt->params.exchange_mtu_rsp.server_rx_mtu;

    printf("MTU response received. New ATT_MTU is %d\n", server_rx_mtu);
    fflush(stdout);
}
#endif

/** Event dispatcher */

/**@brief Function for handling the Application's BLE Stack events.
 *
 * @param[in] adapter The transport adapter.
 * @param[in] p_ble_evt Bluetooth stack event.
 */
static void ble_evt_dispatch(adapter_t *adapter, ble_evt_t *p_ble_evt)
{
    if (p_ble_evt == NULL)
    {
        printf("Received an empty BLE event\n");
        fflush(stdout);
        return;
    }

    switch (p_ble_evt->header.evt_id)
    {
            // Macl add
        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            ble_gap_sec_params_t sec_params;
            sd_ble_gap_sec_params_reply(adapter, p_ble_evt->evt.common_evt.conn_handle,
                                        BLE_GAP_SEC_STATUS_SUCCESS, NULL, NULL);
            printf("Central Accepts Peripheral parameters : success, central_params:NULL, NULL\n");
            break;
        // Macl add
        case BLE_GAP_EVT_KEY_PRESSED:
            // on_keys_response(&(p_ble_evt->evt.gattc_evt));
            break;
            // Macl add
        case BLE_GAP_EVT_CONN_PARAM_UPDATE:
            /*sd_ble_gap_conn_param_update(
                adapter, p_ble_evt->evt.common_evt.conn_handle,
                &p_ble_evt->evt.gap_evt.params.conn_param_update.conn_params);*/
            
            //sd_ble_gap_conn_param_update(adapter, p_ble_evt->evt.common_evt.conn_handle, NULL);
            printf("connection parameter is Min = %d Max = %d, latency = %d, timeout = %d\n",
                   p_ble_evt->evt.gap_evt.params.conn_param_update.conn_params.min_conn_interval,
                   p_ble_evt->evt.gap_evt.params.conn_param_update.conn_params.max_conn_interval,
                   p_ble_evt->evt.gap_evt.params.conn_param_update.conn_params.slave_latency,
                   p_ble_evt->evt.gap_evt.params.conn_param_update.conn_params.conn_sup_timeout);
            // service_discovery_start();//it will run if remote update params
            break;
            // Macl add;
        case BLE_GAP_EVT_CONN_SEC_UPDATE:
            printf("CONN_SEC_UPDATE\n");
            service_discovery_start(); // it will run if remote update params
                                       // ble_enable_params

            // service_discovery_voice();
            break;
            // Macl add
        case BLE_GAP_EVT_AUTH_STATUS:
            printf("auth_status value is %d\n",
                   p_ble_evt->evt.gap_evt.params.auth_status.auth_status);

            service_discovery_start();
            break;
        case BLE_GAP_EVT_CONNECTED:
            on_connected(&(p_ble_evt->evt.gap_evt));
            /*on_pair(&(p_ble_evt->evt.gap_evt));*/
            update_phy(&(p_ble_evt->evt.gap_evt));
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            printf("Disconnected, reason: 0x%02X\n",
                   p_ble_evt->evt.gap_evt.params.disconnected.reason);
            fflush(stdout);
            m_connected_devices--;
            m_connection_handle = 0;
            break;

        case BLE_GAP_EVT_ADV_REPORT:
            on_adv_report(&(p_ble_evt->evt.gap_evt));
            break;

        case BLE_GAP_EVT_TIMEOUT:
            on_timeout(&(p_ble_evt->evt.gap_evt));
            break;

        case BLE_GAP_EVT_PHY_UPDATE:
            on_phy_update(&(p_ble_evt->evt.gap_evt));
            on_pair(&(p_ble_evt->evt.gap_evt));

        case BLE_GATTC_EVT_PRIM_SRVC_DISC_RSP:
            on_service_discovery_response(&(p_ble_evt->evt.gattc_evt));
            break;

        case BLE_GATTC_EVT_CHAR_DISC_RSP:
            on_characteristic_discovery_response(&(p_ble_evt->evt.gattc_evt));
            // update_phy(&(p_ble_evt->evt.gap_evt));
            // char_discovery_start(moreHandle_Start);
            break;

        case BLE_GATTC_EVT_DESC_DISC_RSP:
            on_descriptor_discovery_response(&(p_ble_evt->evt.gattc_evt));
            // findMoreService();
            // service_discovery_voice();
            break;

        case BLE_GATTC_EVT_CHAR_VAL_BY_UUID_READ_RSP:
            // on_read_response_by_uuid(&(p_ble_evt->evt.gattc_evt));
            break;

        case BLE_GATTC_EVT_READ_RSP:
            on_read_response(&(p_ble_evt->evt.gattc_evt));
            break;

        case BLE_GATTC_EVT_WRITE_RSP:
            on_write_response(&(p_ble_evt->evt.gattc_evt));
            break;

        case BLE_GATTC_EVT_HVX:
            on_hvx(&(p_ble_evt->evt.gattc_evt));
            break;

        case BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST:
            on_conn_params_update_request(&(p_ble_evt->evt.gap_evt));
            break;

#if NRF_SD_BLE_API >= 3
        case BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST:
            on_exchange_mtu_request(&(p_ble_evt->evt.gatts_evt));
            break;

        case BLE_GATTC_EVT_EXCHANGE_MTU_RSP:
            on_exchange_mtu_response(&(p_ble_evt->evt.gattc_evt));
            break;
#endif

        default:
            printf("Received an un-handled event with ID: %d\n", p_ble_evt->header.evt_id);
            fflush(stdout);
            break;
    }
}

/** Main */

/**@brief Function for application main entry.
 *
 * @param[in] argc Number of arguments (program expects 0 or 1 arguments).
 * @param[in] argv The serial port of the target nRF5 device (Optional).
 */
int main(int argc, char *argv[])
{
    uint32_t error_code;
    char *serial_port  = DEFAULT_UART_PORT_NAME;
    uint32_t baud_rate = DEFAULT_BAUD_RATE;
    uint8_t cccd_value = 0;

    if (argc > 1)
    {
        serial_port = argv[1];
    }

    printf("Serial port used: %s\n", serial_port);
    printf("Baud rate used: %d\n", baud_rate);
    fflush(stdout);

    m_adapter = adapter_init(serial_port, baud_rate);
    // sd_rpc_conn_reset(m_adapter, sd_rpc_reset_t::SOFT_RESET);
    sd_rpc_log_handler_severity_filter_set(m_adapter, SD_RPC_LOG_INFO);
    error_code = sd_rpc_open(m_adapter, status_handler, ble_evt_dispatch, log_handler);

    if (error_code != NRF_SUCCESS)
    {
        printf("Failed to open nRF BLE Driver. Error code: 0x%02X\n", error_code);
        fflush(stdout);
        return error_code;
    }

#if NRF_SD_BLE_API >= 5
    ble_cfg_set(m_config_id);
#endif

    error_code = ble_stack_init();

    if (error_code != NRF_SUCCESS)
    {
        return error_code;
    }

#if NRF_SD_BLE_API < 5
    error_code = ble_options_set();

    if (error_code != NRF_SUCCESS)
    {
        return error_code;
    }
#endif

    error_code = scan_start();

    if (error_code != NRF_SUCCESS)
    {
        return error_code;
    }

    // Endlessly loop.
    for (;;)
    {
        char c = (char)getchar();
        if (c == 'q' || c == 'Q')
        {
            error_code = sd_rpc_close(m_adapter);

            if (error_code != NRF_SUCCESS)
            {
                printf("Failed to close nRF BLE Driver. Error code: 0x%02X\n", error_code);
                fflush(stdout);
                return error_code;
            }

            printf("Closed\n");
            fflush(stdout);

            return NRF_SUCCESS;
        }

        // Toggle notifications on the HRM characteristic every time user input is received.
        cccd_value ^= BLE_CCCD_NOTIFY;
        hrm_cccd_set(cccd_value);
    }
}
