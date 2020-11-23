#ifndef BLE_ACS_C_H__
#define BLE_ACS_C_H__

#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_gatt.h"
#include "ble_db_discovery.h"
#include "ble_srv_common.h"
#include "nrf_ble_gq.h"
#include "nrf_sdh_ble.h"

#ifndef BLE_ACS_C_BLE_OBSERVER_PRIO
#define BLE_ACS_C_BLE_OBSERVER_PRIO 2
#endif

/**@brief   Macro for defining a ble_acs_c instance.
 *
 * @param   _name   Name of the instance.
 * @hideinitializer
 */
#define BLE_ACS_C_DEF(_name)                                                                        \
static ble_acs_c_t _name;                                                                           \
NRF_SDH_BLE_OBSERVER(_name ## _obs,                                                                 \
                     BLE_ACS_C_BLE_OBSERVER_PRIO,                                                   \
                     ble_acs_c_on_ble_evt, &_name)

/** @brief Macro for defining multiple ble_acs_c instances.
 *
 * @param   _name   Name of the array of instances.
 * @param   _cnt    Number of instances to define.
 * @hideinitializer
 */
#define BLE_ACS_C_ARRAY_DEF(_name, _cnt)                 \
static ble_acs_c_t _name[_cnt];                          \
NRF_SDH_BLE_OBSERVERS(_name ## _obs,                     \
                      BLE_ACS_C_BLE_OBSERVER_PRIO,       \
                      ble_acs_c_on_ble_evt, &_name, _cnt)


// 50E9xxxx-693A-4E74-B11F-C91C353C4263
#define ACS_BASE_UUID  {{0x63, 0x42, 0x3C, 0x35, 0x1C, 0xC9, 0x1F, 0xB1, \
                        0x74, 0x4E, 0x3A, 0x69, 0x00, 0x00, 0xE9, 0x50}} /**< Used vendor specific UUID. */

#define BLE_UUID_ACS_SERVICE        0x0500      /**< The UUID of the Audio Custom Service. */
#define BLE_UUID_ACS_CONFIG_CHAR    0x0501      /**< The UUID of the config Characteristic. */
#define BLE_UUID_ACS_MIC_CHAR       0x0502      /**< The UUID of the microphone Characteristic. */

#define BLE_ACS_MAX_DATA_LEN        (NRF_SDH_BLE_GATT_MAX_MTU_SIZE - 3)     /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Audio Custom service module. */
#define BLE_ACS_MIC_FRAME_SIZE      CONFIG_AUDIO_FRAME_SIZE_BYTES

/**@brief ACS Client event type. */
typedef enum
{
    BLE_ACS_C_EVT_DISCOVERY_COMPLETE,   /**< Event indicating that the ACS service and its characteristics were found. */
    BLE_ACS_C_EVT_MIC_EVT,          /**< Event indicating that the central received something from a peer. */
    BLE_ACS_C_EVT_DISCONNECTED          /**< Event indicating that the ACS server disconnected. */
} ble_acs_c_evt_type_t;

/**@brief Handles on the connected peer device needed to interact with it. */
typedef struct
{
    uint16_t acs_mic_handle;        /**< Handle of the ACS MIC characteristic, as provided by a discovery. */
    uint16_t acs_mic_cccd_handle;   /**< Handle of the CCCD of the ACS MIC characteristic, as provided by a discovery. */
    uint16_t acs_config_handle;     /**< Handle of the ACS CONFIG characteristic, as provided by a discovery. */
} ble_acs_c_handles_t;

/**@brief Structure containing the ACS event data received from the peer. */
typedef struct
{
    ble_acs_c_evt_type_t evt_type;
    uint16_t             conn_handle;
    uint16_t             max_data_len;
    uint8_t            * p_data;
    uint16_t             data_len;
    ble_acs_c_handles_t  handles;     /**< Handles on which the Audio Custom service characteristics were discovered on the peer device. This is filled if the evt_type is @ref BLE_ACS_C_EVT_DISCOVERY_COMPLETE.*/
} ble_acs_c_evt_t;

// Forward declaration of the ble_acs_t type.
typedef struct ble_acs_c_s ble_acs_c_t;

/**@brief   Event handler type.
 *
 * @details This is the type of the event handler that is to be provided by the application
 *          of this module to receive events.
 */
typedef void (* ble_acs_c_evt_handler_t)(ble_acs_c_t * p_ble_acs_c, ble_acs_c_evt_t const * p_evt);

/**@brief ACS Client structure. */
struct ble_acs_c_s
{
    uint8_t                   uuid_type;      /**< UUID type. */
    uint16_t                  conn_handle;    /**< Handle of the current connection. Set with @ref ble_acs_c_handles_assign when connected. */
    ble_acs_c_handles_t       handles;        /**< Handles on the connected peer device needed to interact with it. */
    ble_acs_c_evt_handler_t   evt_handler;    /**< Application event handler to be called when there is an event related to the ACS. */
    ble_srv_error_handler_t   error_handler;  /**< Function to be called in case of an error. */
    nrf_ble_gq_t            * p_gatt_queue;   /**< Pointer to BLE GATT Queue instance. */
};

/**@brief ACS Client initialization structure. */
typedef struct
{
    ble_acs_c_evt_handler_t   evt_handler;    /**< Application event handler to be called when there is an event related to the ACS. */
    ble_srv_error_handler_t   error_handler;  /**< Function to be called in case of an error. */
    nrf_ble_gq_t            * p_gatt_queue;   /**< Pointer to BLE GATT Queue instance. */
} ble_acs_c_init_t;


/**@brief     Function for initializing the Audio Custom Service client module.
 *
 * @details   This function registers with the Database Discovery module
 *            for the ACS. The Database Discovery module looks for the presence
 *            of a ACS instance at the peer when a discovery is started.
 *            
 * @param[in] p_ble_acs_c      Pointer to the ACS client structure.
 * @param[in] p_ble_acs_c_init Pointer to the ACS initialization structure that contains the
 *                             initialization information.
 *
 * @retval    NRF_SUCCESS If the module was initialized successfully.
 * @retval    err_code    Otherwise, this function propagates the error code
 *                        returned by the Database Discovery module API
 *                        @ref ble_db_discovery_evt_register.
 */
uint32_t ble_acs_c_init(ble_acs_c_t * p_ble_acs_c, ble_acs_c_init_t * p_ble_acs_c_init);


/**@brief Function for handling events from the Database Discovery module.
 *
 * @details This function handles an event from the Database Discovery module, and determines
 *          whether it relates to the discovery of ACS at the peer. If it does, the function
 *          calls the application's event handler to indicate that ACS was
 *          discovered at the peer. The function also populates the event with service-related
 *          information before providing it to the application.
 *
 * @param[in] p_ble_acs_c Pointer to the ACS client structure.
 * @param[in] p_evt       Pointer to the event received from the Database Discovery module.
 */
 void ble_acs_c_on_db_disc_evt(ble_acs_c_t * p_ble_acs_c, ble_db_discovery_evt_t * p_evt);


 /**@brief     Function for handling BLE events from the SoftDevice.
 *
 * @details   This function handles the BLE events received from the SoftDevice. If a BLE
 *            event is relevant to the ACS module, the function uses the event's data to update
 *            internal variables and, if necessary, send events to the application.
 *
 * @param[in] p_ble_evt     Pointer to the BLE event.
 * @param[in] p_context     Pointer to the ACS client structure.
 */
void ble_acs_c_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context);


/**@brief   Function for requesting the peer to start sending notification of MIC characteristic.
 *
 * @details This function enables notifications of the ACS MIC characteristic at the peer
 *          by writing to the CCCD of the ACS MIC characteristic.
 *
 * @param   p_ble_acs_c Pointer to the ACS client structure.
 *
 * @retval  NRF_SUCCESS If the operation was successful. 
 * @retval  err_code 	Otherwise, this API propagates the error code returned by function @ref nrf_ble_gq_item_add.
 */
uint32_t ble_acs_c_mic_notif_enable(ble_acs_c_t * p_ble_acs_c);


/**@brief   Function for requesting the peer to stop sending notification of MIC characteristic.
 *
 * @details This function disables notifications of the ACS MIC characteristic at the peer
 *          by writing to the CCCD of the ACS MIC characteristic.
 *
 * @param   p_ble_acs_c Pointer to the ACS client structure.
 *
 * @retval  NRF_SUCCESS If the operation was successful. 
 * @retval  err_code 	Otherwise, this API propagates the error code returned by function @ref nrf_ble_gq_item_add.
 */
uint32_t ble_acs_c_mic_notif_disable(ble_acs_c_t * p_ble_acs_c);


/**@brief Function for assigning handles to this instance of acs_c.
 *
 * @details Call this function when a link has been established with a peer to
 *          associate the link to this instance of the module. This makes it
 *          possible to handle several links and associate each link to a particular
 *          instance of this module. The connection handle and attribute handles are
 *          provided from the discovery event @ref BLE_ACS_C_EVT_DISCOVERY_COMPLETE.
 *
 * @param[in] p_ble_acs_c    Pointer to the ACS client structure instance to associate with these
 *                           handles.
 * @param[in] conn_handle    Connection handle to associated with the given ACS Instance.
 * @param[in] p_peer_handles Attribute handles on the ACS server that you want this ACS client to
 *                           interact with.
 *
 * @retval    NRF_SUCCESS    If the operation was successful.
 * @retval    NRF_ERROR_NULL If a p_acs was a NULL pointer.
 * @retval    err_code       Otherwise, this API propagates the error code returned 
 *                           by function @ref nrf_ble_gq_item_add.
 */
uint32_t ble_acs_c_handles_assign(ble_acs_c_t *               p_ble_acs_c,
                                  uint16_t                    conn_handle,
                                  ble_acs_c_handles_t const * p_peer_handles);

#endif