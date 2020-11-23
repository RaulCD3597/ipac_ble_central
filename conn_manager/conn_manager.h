/**
 * @file conn_manager.h
 * @author Raul Camacho
 * @date July 2020
 * @brief Provides functions to manage BLE connection for IPAC_BLE central device
 */

#ifndef _CONN_MANAGER_H_
#define _CONN_MANAGER_H_

#include "ble_acs_c.h"
#include "ble_bas_c.h"
#include "sdk_macros.h"

/* ----------------- public functions -----------------*/

void conn_init(void);

void conn_start_scan(void);

uint16_t * conn_get_nus_c_max_len(void);

void conn_send_string(u_int8_t * str, uint16_t length, u_int8_t nus_instance);

void conn_mic_enable(u_int8_t nus_instance);

void conn_mic_disable(u_int8_t nus_instance);

void scan_init(void);

void conn_bed_register(u_int8_t bed_no, u_int8_t * device_id);

#endif /* _CONN_MANAGER_H_ */