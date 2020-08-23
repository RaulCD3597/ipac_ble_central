/**
 * @file json_c.c
 * @author Raul Camacho
 * @date July 2020
 * @brief Provides functions to manage json frames.
 */

//json header
#include "json_c.h"
#include <string.h>
#include <stdio.h>

/* ----------------  local definitions ----------------*/

/* -----------------  local variables -----------------*/

/* ------------ local functions prototypes ------------*/

/* ----------------- public functions -----------------*/

uint8_t * json_c_parser(const uint8_t * str, const uint8_t * const param)
{
    uint8_t * received = (uint8_t *)str;
    char search[20];
    sprintf(search, "\"%s\":", param);

    if (NULL != (received = (uint8_t *)strstr((const char *)received, search)))
    {
        received += strlen(search);
        while (' ' == *received)
        {
            received++;
        }
        return received;
    }

    return NULL;
}

/* -----------------  local functions -----------------*/