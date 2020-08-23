/**
 * @file hardware.h
 * @author Raul Camacho
 * @date July 2020
 * @brief Provides hardware control functions
 */

#ifndef _HARDWARE_H_
#define _HARDWARE_H_

/* ---------------- public definitions ----------------*/

/* ------------------- public enums -------------------*/

enum
{
    CENTRAL_SCANNING_LED = 17,
    CENTRAL_CONNECTED_LED,
};

/* ----------------- public functions -----------------*/

void hardware_init(void);

void uart_send_string(uint8_t * p_data, uint16_t data_len);

#endif /* _HARDWARE_H_ */