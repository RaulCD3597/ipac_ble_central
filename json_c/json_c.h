/**
 * @file json_c.h
 * @author Raul Camacho
 * @date July 2020
 * @brief Provides functions to manage json frames.
 */

#ifndef _JSON_C_H_
#define _JSON_C_H_

#include <stdint.h>

/* ---------------- public definitions ----------------*/

/* ------------------- public enums -------------------*/

/* ----------------- public functions -----------------*/

uint8_t * json_c_parser(const uint8_t * str, const uint8_t * const param);

#endif /* _JSON_C_H_ */