/**
 * @file conn_manager.h
 * @author Raul Camacho
 * @date July 2020
 * @brief Provides functions to manage BLE connection for IPAC_BLE central device
 */

#ifndef _CONN_MANAGER_H_
#define _CONN_MANAGER_H_

/* ----------------- public functions -----------------*/

void Conn_Init(void);
void Conn_StartScan(void);

#endif /* _CONN_MANAGER_H_ */