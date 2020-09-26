/**
 * @file conn_manager.c
 * @author Raul Camacho
 * @date July 2020
 */

#pragma GCC optimize ("O0")

// Standard libraries
#include <stdint.h>
#include <stdio.h>
#include <string.h>

// Nordic common library
#include "nordic_common.h"

// SoftDevice header s132
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_soc.h"

// nrf drivers
#include "nrf.h"
#include "nrf_gpio.h"

// nrf app headers
#include "app_error.h"
#include "app_uart.h"
#include "app_util.h"

// ble headers
#include "ble.h"
#include "ble_gap.h"
#include "ble_hci.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "ble_db_discovery.h"
#include "ble_conn_state.h"
#include "ble_nus_c.h"

// nrf ble headers
#include "nrf_ble_gatt.h"
#include "nrf_ble_scan.h"

// ipac headers
#include "conn_manager.h"
#include "hardware.h"
#include "json_c.h"

/* ------------------  local typedefs -----------------*/

typedef void (*action_fcs)(ble_nus_c_t * p_ble_nus_c, ble_nus_c_evt_t const * p_ble_nus_evt);
typedef enum{
    EMERGENCY = 1,
    SERVICE,
    LOW_BATT,
    DISCONNECTION
} caller_evt_t;

/* ----------------  local definitions ----------------*/

/** 
 * Tag that refers to the BLE stack configuration that is set with @ref sd_ble_cfg_set. 
 * The default tag is @ref APP_BLE_CONN_CFG_TAG. 
 */
#define APP_BLE_CONN_CFG_TAG    1
/** 
 * BLE observer priority of the application. There is no need to modify this value. 
 */
#define APP_BLE_OBSERVER_PRIO   3
/** 
 * ID string to search. 
 */
#define CMD_ID_STR              "id"
/** 
 * No conection value for beds registered. 
 */
#define NO_CONNECTION            0xFF
/** 
 * Number of names registered to look on scan process. 
 */
#define NAME_REGISTER_LEN        2
/** 
 * Number of max beds to be registered to look on scan process. 
 */
#define BED_QTY                  7
/** 
 * Percentage of battery level to be notified as low. 
 */
#define LOW_BATT_NOTIF_LEVEL     25

/* -----------------  local variables -----------------*/

/** GATT module instance. */
NRF_BLE_GATT_DEF(m_gatt);
/** NUS client instances. */
BLE_NUS_C_ARRAY_DEF(m_nus_c, NRF_SDH_BLE_CENTRAL_LINK_COUNT);
/** ACS client instances. */
BLE_ACS_C_ARRAY_DEF(m_acs_c, NRF_SDH_BLE_CENTRAL_LINK_COUNT);
/** BAS client instances. */
BLE_BAS_C_ARRAY_DEF(m_bas_c, NRF_SDH_BLE_CENTRAL_LINK_COUNT);
/** Database discovery module instances. */
BLE_DB_DISCOVERY_ARRAY_DEF(m_db_disc, NRF_SDH_BLE_CENTRAL_LINK_COUNT);
/** Scanning Module instance. */
NRF_BLE_SCAN_DEF(m_scan);
/**< BLE GATT Queue instance. */
NRF_BLE_GQ_DEF(m_ble_gatt_queue,
               NRF_SDH_BLE_CENTRAL_LINK_COUNT,
               NRF_BLE_GQ_QUEUE_SIZE);
/** 
 * Name of the device to try to connect to. 
 * This name is searched for in the scanning report data. 
 */
static char m_target_periph_name[2][21] = {
    "34401540363542274495",
    "39066043182642957650"
};
/** NUS fifo length */
static u_int16_t m_ble_nus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - OPCODE_LENGTH - HANDLE_LENGTH;
/** Registered bed callers */
static u_int16_t conn_beds[BED_QTY] = { 
                    NO_CONNECTION,
                    NO_CONNECTION,
                    NO_CONNECTION,
                    NO_CONNECTION,
                    NO_CONNECTION, 
                    NO_CONNECTION, 
                    NO_CONNECTION
                    };

/* ------------ local functions prototypes ------------*/

static void ble_stack_init(void);
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context);
static void gatt_init(void);
static void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt);
static void db_discovery_init(void);
static void db_disc_handler(ble_db_discovery_evt_t * p_evt);
static void nus_c_init(void);
static void ble_nus_c_evt_handler(ble_nus_c_t * p_ble_nus_c, ble_nus_c_evt_t const * p_ble_nus_evt);
static void services_error_handler(u_int32_t nrf_error);
static void scan_init(void);
static void scan_evt_handler(scan_evt_t const * p_scan_evt);
static u_int8_t conn_handle_to_bed(u_int16_t conn_handle);
static void notify_evt(u_int8_t bed, caller_evt_t evt);
static void acs_c_init(void);
static void ble_acs_c_evt_handler(ble_acs_c_t * p_ble_acs_c, ble_acs_c_evt_t const * p_ble_acs_evt);
static void bas_c_init(void);
static void ble_bas_c_evt_handler(ble_bas_c_t * p_ble_bas_c, ble_bas_c_evt_t * p_ble_bas_evt);

/* ------------- local functions pointers -------------*/

/* ----------------- public functions -----------------*/

/**
 * @brief Function for initializing BLE connection.
 */
void conn_init(void)
{
    ble_stack_init();
    gatt_init();
    db_discovery_init();
    nus_c_init();
    acs_c_init();
    bas_c_init();
    ble_conn_state_init();
    scan_init();
}

/**
 * @brief Function for start scanning BLE devices.
 */
void conn_start_scan(void)
{
    ret_code_t ret;

    ret = nrf_ble_scan_start(&m_scan);
    APP_ERROR_CHECK(ret);
    // Turn on the LED to signal scanning.
    nrf_gpio_pin_clear(CENTRAL_SCANNING_LED);
}

/**
 * @brief Function for getting NUS fifo length.
 */
u_int16_t * conn_get_nus_c_max_len(void)
{
    return ((u_int16_t *)&m_ble_nus_max_data_len);
}

/**
 * @brief Function for sending a string to a perif device.
 */
void conn_send_string(u_int8_t * str, u_int16_t length, u_int8_t nus_instance)
{
    u_int32_t ret_val = nus_instance;

    if (NO_CONNECTION == conn_beds[nus_instance])
    {
        return;
    }

    do
    {
        ret_val = ble_nus_c_string_send(&m_nus_c[(conn_beds[nus_instance])], str, length);
        if ((ret_val != NRF_ERROR_INVALID_STATE) && (ret_val != NRF_ERROR_RESOURCES))
        {
            APP_ERROR_CHECK(ret_val);
        }
    } while (ret_val != NRF_SUCCESS);
}

void conn_mic_enable(u_int8_t nus_instance)
{
    u_int32_t err_code = ble_acs_c_mic_notif_enable(&m_acs_c[(conn_beds[nus_instance])]);
    APP_ERROR_CHECK(err_code);
}

void conn_mic_disable(u_int8_t nus_instance)
{
    u_int32_t err_code = ble_acs_c_mic_notif_disable(&m_acs_c[(conn_beds[nus_instance])]);
    APP_ERROR_CHECK(err_code);
}

/* -----------------  local functions -----------------*/

/**
 * @brief Function for initializing the BLE stack.
 * @details Initializes the SoftDevice and the BLE event interrupts.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    u_int32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}

/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    static u_int8_t bed_no = 0xFF;
    static bool conn_enabled = false;
    ret_code_t err_code;
    u_int8_t  adv_data [100];

    // For readability.
    ble_gap_evt_t const * p_gap_evt = &p_ble_evt->evt.gap_evt;

    if ( BLE_GAP_EVT_ADV_SET_TERMINATED > (p_ble_evt->header.evt_id))
    {
        strcpy((char *)&adv_data, (char *)p_gap_evt->params.adv_report.data.p_data); 

        for (size_t i = 0; i < NAME_REGISTER_LEN; i++)
        {
            if (NULL != strstr((const char *)adv_data, m_target_periph_name[i]))
            {
                bed_no = i;
                conn_enabled = true;
            }
        }
    }

    switch (p_ble_evt->header.evt_id)
    {
    // Upon connection, check which peripheral is connected, initiate DB
    // discovery, update LEDs status, and resume scanning, if necessary.
    case BLE_GAP_EVT_CONNECTED:
    {
        APP_ERROR_CHECK_BOOL(p_gap_evt->conn_handle < NRF_SDH_BLE_CENTRAL_LINK_COUNT);

        err_code = ble_nus_c_handles_assign(&m_nus_c[p_gap_evt->conn_handle],
                                            p_gap_evt->conn_handle,
                                            NULL);
        APP_ERROR_CHECK(err_code);

        err_code = ble_acs_c_handles_assign(&m_acs_c[p_gap_evt->conn_handle],
                                            p_gap_evt->conn_handle,
                                            NULL);
        APP_ERROR_CHECK(err_code);

        err_code = ble_bas_c_handles_assign(&m_bas_c[p_gap_evt->conn_handle],
                                            p_gap_evt->conn_handle,
                                            NULL);
        APP_ERROR_CHECK(err_code);

        if (conn_enabled)
        {
            conn_beds[bed_no] = p_gap_evt->conn_handle;
            conn_enabled = false;
        }

        err_code = ble_db_discovery_start(&m_db_disc[p_gap_evt->conn_handle],
                                          p_gap_evt->conn_handle);
        APP_ERROR_CHECK(err_code);

        // Update LEDs status and check whether it is needed to look for more
        // peripherals to connect to.
        nrf_gpio_pin_clear(CENTRAL_CONNECTED_LED);
        if (ble_conn_state_central_conn_count() == NRF_SDH_BLE_CENTRAL_LINK_COUNT)
        {
            nrf_gpio_pin_set(CENTRAL_SCANNING_LED);
        }
        else
        {
            // Resume scanning.
            nrf_gpio_pin_clear(CENTRAL_SCANNING_LED);
            conn_start_scan();
        }
    }
    break; // BLE_GAP_EVT_CONNECTED

    // Upon disconnection, reset the connection handle of the peer that disconnected, update
    // the LEDs status and start scanning again.
    case BLE_GAP_EVT_DISCONNECTED:
    {
        notify_evt(conn_handle_to_bed(p_gap_evt->conn_handle), DISCONNECTION);

        if (ble_conn_state_central_conn_count() == 0)
        {
            // Turn off the LED that indicates the connection.
            nrf_gpio_pin_set(CENTRAL_CONNECTED_LED);
        }

        // Start scanning.
        conn_start_scan();

        // Turn on the LED for indicating scanning.
        nrf_gpio_pin_clear(CENTRAL_SCANNING_LED);
    }
    break;

    case BLE_GAP_EVT_TIMEOUT:
    {
        // Timeout for scanning is not specified, so only the connection requests can time out.
        if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_CONN)
        {
        }
    }
    break;

    case BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST:
    {
        // Accept parameters requested by peer.
        err_code = sd_ble_gap_conn_param_update(p_gap_evt->conn_handle,
                                                &p_gap_evt->params.conn_param_update_request.conn_params);
        APP_ERROR_CHECK(err_code);
    }
    break;

    case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
    {
        ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
        err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
        APP_ERROR_CHECK(err_code);
    }
    break;

    case BLE_GATTC_EVT_TIMEOUT:
    {
        // Disconnect on GATT client timeout event.
        err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                         BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
        APP_ERROR_CHECK(err_code);
    }
    break;

    case BLE_GATTS_EVT_TIMEOUT:
    {
        // Disconnect on GATT server timeout event.
        err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                         BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
        APP_ERROR_CHECK(err_code);
    }
    break;

    default:
        // No implementation needed.
        break;
    }
}

/**
 * @brief Function for initializing the GATT module.
 */
static void gatt_init(void)
{
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_gatt_att_mtu_central_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
    APP_ERROR_CHECK(err_code);
}

/**
 * @brief Function for handling events from the GATT library. 
 */
static void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt)
{
    if (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED)
    {
        m_ble_nus_max_data_len = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
    }
}

/** 
 * @brief Database discovery initialization.
 */
static void db_discovery_init(void)
{
    ble_db_discovery_init_t db_init;

    memset(&db_init, 0, sizeof(ble_db_discovery_init_t));

    db_init.evt_handler  = db_disc_handler;
    db_init.p_gatt_queue = &m_ble_gatt_queue;

    ret_code_t err_code = ble_db_discovery_init(&db_init);
    APP_ERROR_CHECK(err_code);
}

/**
 * @brief Function for handling database discovery events.
 *
 * @details This function is a callback function to handle events from the database discovery module.
 *          Depending on the UUIDs that are discovered, this function forwards the events
 *          to their respective services.
 *
 * @param[in] p_event  Pointer to the database discovery event.
 */
static void db_disc_handler(ble_db_discovery_evt_t * p_evt)
{
    ble_nus_c_on_db_disc_evt(&m_nus_c[p_evt->conn_handle], p_evt);
    ble_acs_c_on_db_disc_evt(&m_acs_c[p_evt->conn_handle], p_evt);
    ble_bas_on_db_disc_evt(&m_bas_c[p_evt->conn_handle], p_evt);
}

/** 
 * @brief Nordic UART service init.
 */
static void nus_c_init(void)
{
    ret_code_t       err_code;
    ble_nus_c_init_t init;

    init.evt_handler   = ble_nus_c_evt_handler;
    init.error_handler = services_error_handler;
    init.p_gatt_queue  = &m_ble_gatt_queue;

    for (u_int32_t i = 0; i < NRF_SDH_BLE_CENTRAL_LINK_COUNT; i++)
    {
        err_code = ble_nus_c_init(&m_nus_c[i], &init);
        APP_ERROR_CHECK(err_code);
    }
}

/**@brief Callback handling Nordic UART Service (NUS) client events.
 *
 * @details This function is called to notify the application of NUS client events.
 *
 * @param[in]   p_ble_nus_c   NUS client handle. This identifies the NUS client.
 * @param[in]   p_ble_nus_evt Pointer to the NUS client event.
 */
static void ble_nus_c_evt_handler(ble_nus_c_t * p_ble_nus_c, ble_nus_c_evt_t const * p_ble_nus_evt)
{
    ret_code_t err_code;
    u_int8_t * received = NULL;
    
    switch (p_ble_nus_evt->evt_type)
    {
        case BLE_NUS_C_EVT_DISCOVERY_COMPLETE:
            err_code = ble_nus_c_handles_assign(p_ble_nus_c, p_ble_nus_evt->conn_handle, &p_ble_nus_evt->handles);
            APP_ERROR_CHECK(err_code);

            err_code = ble_nus_c_tx_notif_enable(p_ble_nus_c);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_NUS_C_EVT_NUS_TX_EVT:
            received = (u_int8_t *)p_ble_nus_evt->p_data;
            received = json_c_parser((const u_int8_t *)received, (const u_int8_t * const)CMD_ID_STR);
            caller_evt_t evt = (caller_evt_t )(*received - '0');
            notify_evt(conn_handle_to_bed(p_ble_nus_c->conn_handle), evt);
            break;

        case BLE_NUS_C_EVT_DISCONNECTED:
            conn_start_scan();
            break;
    }
}

/**
 * @brief Function for handling the Services Client errors.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void services_error_handler(u_int32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

/**
 * @brief Function for initializing the scanning and setting the filters.
 */
static void scan_init(void)
{
    ret_code_t          err_code;
    nrf_ble_scan_init_t init_scan;

    memset(&init_scan, 0, sizeof(init_scan));

    init_scan.connect_if_match = true;
    init_scan.conn_cfg_tag     = APP_BLE_CONN_CFG_TAG;

    err_code = nrf_ble_scan_init(&m_scan, &init_scan, scan_evt_handler);
    APP_ERROR_CHECK(err_code);

    for (size_t i = 0; i < NAME_REGISTER_LEN; i++){
        err_code = nrf_ble_scan_filter_set(&m_scan, SCAN_NAME_FILTER, m_target_periph_name[i]);
        APP_ERROR_CHECK(err_code);
    }

    err_code = nrf_ble_scan_filters_enable(&m_scan, NRF_BLE_SCAN_NAME_FILTER, false);
    APP_ERROR_CHECK(err_code);
}

/**
 * @brief Function for handling Scanning Module events.
 */
static void scan_evt_handler(scan_evt_t const * p_scan_evt)
{
    ret_code_t err_code;

    switch(p_scan_evt->scan_evt_id)
    {
        case NRF_BLE_SCAN_EVT_CONNECTING_ERROR:
        {
            err_code = p_scan_evt->params.connecting_err.err_code;
            APP_ERROR_CHECK(err_code);
        } break;

        default:
            break;
    }
}

/**
 * @brief Function for getting the number of bed based on the conn_handle.
 */
static u_int8_t conn_handle_to_bed(u_int16_t conn_handle)
{
    for (size_t i = 0; i < BED_QTY; i++)
    {
        if (conn_handle == conn_beds[i])
        {
            return (i + 1);
        }
    }
    // Should not get to this point
    return NO_CONNECTION;
}

/**
 * @brief Function for notifying a caller evt.
 */
static void notify_evt(u_int8_t bed, caller_evt_t evt)
{
    if (BED_QTY < bed)
    {
        return;
    }
    u_int8_t notify_str[20];
    sprintf((char *)notify_str, "{\"bed\": %d, \"id\": %d}", bed, evt);
    uart_send_string(notify_str, strlen((char *)notify_str));
}

/** 
 * @brief Audio Custom service init.
 */
static void acs_c_init(void)
{
    ret_code_t       err_code;
    ble_acs_c_init_t init;

    init.evt_handler   = ble_acs_c_evt_handler;
    init.error_handler = services_error_handler;
    init.p_gatt_queue  = &m_ble_gatt_queue;

    for (u_int32_t i = 0; i < NRF_SDH_BLE_CENTRAL_LINK_COUNT; i++)
    {
        err_code = ble_acs_c_init(&m_acs_c[i], &init);
        APP_ERROR_CHECK(err_code);
    }
}

/**@brief Callback handling Audio Custom Service (ACS) client events.
 *
 * @details This function is called to notify the application of ACS client events.
 *
 * @param[in]   p_ble_acs_c   ACS client handle. This identifies the ACS client.
 * @param[in]   p_ble_acs_evt Pointer to the ACS client event.
 */
static void ble_acs_c_evt_handler(ble_acs_c_t * p_ble_acs_c, ble_acs_c_evt_t const * p_ble_acs_evt)
{
    ret_code_t err_code;
    
    switch (p_ble_acs_evt->evt_type)
    {
        case BLE_ACS_C_EVT_DISCOVERY_COMPLETE:
            err_code = ble_acs_c_handles_assign(p_ble_acs_c, p_ble_acs_evt->conn_handle, &p_ble_acs_evt->handles);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_ACS_C_EVT_MIC_EVT:
            uart_send_audio_frames(p_ble_acs_evt->p_data, p_ble_acs_evt->data_len);
            break;

        case BLE_ACS_C_EVT_DISCONNECTED:
            conn_start_scan();
            break;
    }
}

/** 
 * @brief Battery service init.
 */
static void bas_c_init(void)
{
    ret_code_t       err_code;
    ble_bas_c_init_t init;

    init.evt_handler   = ble_bas_c_evt_handler;
    init.error_handler = services_error_handler;
    init.p_gatt_queue  = &m_ble_gatt_queue;

    for (u_int32_t i = 0; i < NRF_SDH_BLE_CENTRAL_LINK_COUNT; i++)
    {
        err_code = ble_bas_c_init(&m_bas_c[i], &init);
        APP_ERROR_CHECK(err_code);
    }
}

/**@brief Callback handling Battery Service (BAS) client events.
 *
 * @details This function is called to notify the application of BAS client events.
 *
 * @param[in]   p_bas_bas_c   BAS client handle. This identifies the BAS client.
 * @param[in]   p_ble_bas_evt Pointer to the BAS client event.
 */
static void ble_bas_c_evt_handler(ble_bas_c_t * p_ble_bas_c, ble_bas_c_evt_t * p_ble_bas_evt)
{
    ret_code_t err_code;
    
    switch (p_ble_bas_evt->evt_type)
    {
        case BLE_BAS_C_EVT_DISCOVERY_COMPLETE:
            err_code = ble_bas_c_handles_assign(p_ble_bas_c, p_ble_bas_evt->conn_handle, &p_ble_bas_evt->params.bas_db);
            APP_ERROR_CHECK(err_code);

            err_code = ble_bas_c_bl_notif_enable(p_ble_bas_c);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_BAS_C_EVT_BATT_NOTIFICATION:
            if (LOW_BATT_NOTIF_LEVEL > p_ble_bas_evt->params.battery_level)
            {
                notify_evt(conn_handle_to_bed(p_ble_bas_c->conn_handle), LOW_BATT);
            }
            break;

        case BLE_BAS_C_EVT_BATT_READ_RESP:
            break;
    }
}