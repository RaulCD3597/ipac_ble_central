/**
 * @file conn_manager.h
 * @author Raul Camacho
 * @date July 2020
 * @brief Provides functions to manage BLE connection for IPAC_BLE central device
 */

#ifndef _CONN_MANAGER_H_
#define _CONN_MANAGER_H_

/* ----------------- public functions -----------------*/

void conn_init(void);

void conn_start_scan(void);

uint16_t * conn_get_nus_c_max_len(void);

void conn_send_string(uint8_t * str, uint16_t length, uint8_t nus_instance);

#endif /* _CONN_MANAGER_H_ */