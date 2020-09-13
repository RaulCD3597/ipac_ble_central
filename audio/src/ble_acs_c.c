#include "sdk_common.h"
#include <stdlib.h>

#include "ble.h"
#include "ble_acs_c.h"
#include "ble_gattc.h"
#include "ble_srv_common.h"
#include "app_error.h"


/**@brief Function for intercepting the errors of GATTC and the BLE GATT Queue.
 *
 * @param[in] nrf_error   Error code.
 * @param[in] p_ctx       Parameter from the event handler.
 * @param[in] conn_handle Connection handle.
 */
static void gatt_error_handler(uint32_t   nrf_error,
                               void     * p_ctx,
                               uint16_t   conn_handle)
{
    ble_acs_c_t * p_ble_acs_c = (ble_acs_c_t *)p_ctx;

    if (p_ble_acs_c->error_handler != NULL)
    {
        p_ble_acs_c->error_handler(nrf_error);
    }
}


void ble_acs_c_on_db_disc_evt(ble_acs_c_t * p_ble_acs_c, ble_db_discovery_evt_t * p_evt)
{
    ble_acs_c_evt_t acs_c_evt;
    memset(&acs_c_evt,0,sizeof(ble_acs_c_evt_t));

    ble_gatt_db_char_t * p_chars = p_evt->params.discovered_db.charateristics;

    // Check if the ACS was discovered.
    if (    (p_evt->evt_type == BLE_DB_DISCOVERY_COMPLETE)
        &&  (p_evt->params.discovered_db.srv_uuid.uuid == BLE_UUID_ACS_SERVICE)
        &&  (p_evt->params.discovered_db.srv_uuid.type == p_ble_acs_c->uuid_type))
    {
        for (uint32_t i = 0; i < p_evt->params.discovered_db.char_count; i++)
        {
            switch (p_chars[i].characteristic.uuid.uuid)
            {
                case BLE_UUID_ACS_CONFIG_CHAR:
                    acs_c_evt.handles.acs_config_handle = p_chars[i].characteristic.handle_value;
                    break;

                case BLE_UUID_ACS_MIC_CHAR:
                    acs_c_evt.handles.acs_mic_handle        = p_chars[i].characteristic.handle_value;
                    acs_c_evt.handles.acs_mic_cccd_handle   = p_chars[i].cccd_handle;
                    break;

                default:
                    break;
            }
        }
        if (p_ble_acs_c->evt_handler != NULL)
        {
            acs_c_evt.conn_handle = p_evt->conn_handle;
            acs_c_evt.evt_type    = BLE_ACS_C_EVT_DISCOVERY_COMPLETE;
            p_ble_acs_c->evt_handler(p_ble_acs_c, &acs_c_evt);
        }
    }
}


/**@brief     Function for handling Handle Value Notification received from the SoftDevice.
 *
 * @details   This function uses the Handle Value Notification received from the SoftDevice
 *            and checks if it is a notification of the ACS MIC characteristic from the peer.
 *            If it is, this function decodes the data and sends it to the application.
 *            
 * @param[in] p_ble_acs_c Pointer to the ACS Client structure.
 * @param[in] p_ble_evt   Pointer to the BLE event received.
 */
static void on_hvx(ble_acs_c_t * p_ble_acs_c, ble_evt_t const * p_ble_evt)
{
    // HVX can only occur from client sending.
    if (   (p_ble_acs_c->handles.acs_mic_handle != BLE_GATT_HANDLE_INVALID)
        && (p_ble_evt->evt.gattc_evt.params.hvx.handle == p_ble_acs_c->handles.acs_mic_handle)
        && (p_ble_acs_c->evt_handler != NULL))
    {
        ble_acs_c_evt_t ble_acs_c_evt;

        ble_acs_c_evt.evt_type = BLE_ACS_C_EVT_ACS_MIC_EVT;
        ble_acs_c_evt.p_data   = (uint8_t *)p_ble_evt->evt.gattc_evt.params.hvx.data;
        ble_acs_c_evt.data_len = p_ble_evt->evt.gattc_evt.params.hvx.len;

        p_ble_acs_c->evt_handler(p_ble_acs_c, &ble_acs_c_evt);
    }
}


uint32_t ble_acs_c_init(ble_acs_c_t * p_ble_acs_c, ble_acs_c_init_t * p_ble_acs_c_init)
{
    uint32_t      err_code;
    ble_uuid_t    uart_uuid;
    ble_uuid128_t acs_base_uuid = ACS_BASE_UUID;

    VERIFY_PARAM_NOT_NULL(p_ble_acs_c);
    VERIFY_PARAM_NOT_NULL(p_ble_acs_c_init);
    VERIFY_PARAM_NOT_NULL(p_ble_acs_c_init->p_gatt_queue);

    err_code = sd_ble_uuid_vs_add(&acs_base_uuid, &p_ble_acs_c->uuid_type);
    VERIFY_SUCCESS(err_code);

    uart_uuid.type = p_ble_acs_c->uuid_type;
    uart_uuid.uuid = BLE_UUID_ACS_SERVICE;

    p_ble_acs_c->conn_handle                = BLE_CONN_HANDLE_INVALID;
    p_ble_acs_c->evt_handler                = p_ble_acs_c_init->evt_handler;
    p_ble_acs_c->error_handler              = p_ble_acs_c_init->error_handler;
    p_ble_acs_c->handles.acs_mic_handle     = BLE_GATT_HANDLE_INVALID;
    p_ble_acs_c->handles.acs_config_handle  = BLE_GATT_HANDLE_INVALID;
    p_ble_acs_c->p_gatt_queue               = p_ble_acs_c_init->p_gatt_queue;

    return ble_db_discovery_evt_register(&uart_uuid);
}


void ble_acs_c_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context)
{
    ble_acs_c_t * p_ble_acs_c = (ble_acs_c_t *)p_context;

    if ((p_ble_acs_c == NULL) || (p_ble_evt == NULL))
    {
        return;
    }

    if ( (p_ble_acs_c->conn_handle == BLE_CONN_HANDLE_INVALID)
       ||(p_ble_acs_c->conn_handle != p_ble_evt->evt.gap_evt.conn_handle)
       )
    {
        return;
    }

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GATTC_EVT_HVX:
            on_hvx(p_ble_acs_c, p_ble_evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            if (p_ble_evt->evt.gap_evt.conn_handle == p_ble_acs_c->conn_handle
                    && p_ble_acs_c->evt_handler != NULL)
            {
                ble_acs_c_evt_t acs_c_evt;

                acs_c_evt.evt_type = BLE_ACS_C_EVT_DISCONNECTED;

                p_ble_acs_c->conn_handle = BLE_CONN_HANDLE_INVALID;
                p_ble_acs_c->evt_handler(p_ble_acs_c, &acs_c_evt);
            }
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for creating a message for writing to the CCCD. */
static uint32_t cccd_configure(ble_acs_c_t * p_ble_acs_c, bool notification_enable)
{
    nrf_ble_gq_req_t cccd_req;
    uint8_t          cccd[BLE_CCCD_VALUE_LEN];
    uint16_t         cccd_val = notification_enable ? BLE_GATT_HVX_NOTIFICATION : 0;

    memset(&cccd_req, 0, sizeof(nrf_ble_gq_req_t));

    cccd[0] = LSB_16(cccd_val);
    cccd[1] = MSB_16(cccd_val);

    cccd_req.type                        = NRF_BLE_GQ_REQ_GATTC_WRITE;
    cccd_req.error_handler.cb            = gatt_error_handler;
    cccd_req.error_handler.p_ctx         = p_ble_acs_c;
    cccd_req.params.gattc_write.handle   = p_ble_acs_c->handles.acs_mic_cccd_handle;
    cccd_req.params.gattc_write.len      = BLE_CCCD_VALUE_LEN;
    cccd_req.params.gattc_write.offset   = 0;
    cccd_req.params.gattc_write.p_value  = cccd;
    cccd_req.params.gattc_write.write_op = BLE_GATT_OP_WRITE_REQ;
    cccd_req.params.gattc_write.flags    = BLE_GATT_EXEC_WRITE_FLAG_PREPARED_WRITE;

    return nrf_ble_gq_item_add(p_ble_acs_c->p_gatt_queue, &cccd_req, p_ble_acs_c->conn_handle);
}


uint32_t ble_acs_c_mic_notif_enable(ble_acs_c_t * p_ble_acs_c)
{
    VERIFY_PARAM_NOT_NULL(p_ble_acs_c);

    if ( (p_ble_acs_c->conn_handle == BLE_CONN_HANDLE_INVALID)
       ||(p_ble_acs_c->handles.acs_mic_cccd_handle == BLE_GATT_HANDLE_INVALID)
       )
    {
        return NRF_ERROR_INVALID_STATE;
    }
    return cccd_configure(p_ble_acs_c, true);
}


uint32_t ble_acs_c_mic_notif_disable(ble_acs_c_t * p_ble_acs_c)
{
    VERIFY_PARAM_NOT_NULL(p_ble_acs_c);

    if ( (p_ble_acs_c->conn_handle == BLE_CONN_HANDLE_INVALID)
       ||(p_ble_acs_c->handles.acs_mic_cccd_handle == BLE_GATT_HANDLE_INVALID)
       )
    {
        return NRF_ERROR_INVALID_STATE;
    }
    return cccd_configure(p_ble_acs_c, true);
}


uint32_t ble_acs_c_handles_assign(ble_acs_c_t               * p_ble_acs,
                                  uint16_t                    conn_handle,
                                  ble_acs_c_handles_t const * p_peer_handles)
{
    VERIFY_PARAM_NOT_NULL(p_ble_acs);

    p_ble_acs->conn_handle = conn_handle;
    if (p_peer_handles != NULL)
    {
        p_ble_acs->handles.acs_mic_cccd_handle = p_peer_handles->acs_mic_cccd_handle;
        p_ble_acs->handles.acs_mic_handle      = p_peer_handles->acs_mic_handle;
        p_ble_acs->handles.acs_config_handle   = p_peer_handles->acs_config_handle;
    }
    return nrf_ble_gq_conn_handle_register(p_ble_acs->p_gatt_queue, conn_handle);
}