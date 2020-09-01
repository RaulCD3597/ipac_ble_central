/** 
 * @file main.c
 * @author Raul Camacho
 * @date July 2020
 */

// Standard libraries
#include <stdint.h>
#include <stdio.h>
#include <string.h>

// Nordic common library
#include "nordic_common.h"

// nrf drivers
#include "nrf_pwr_mgmt.h"

// nrf app headers
#include "app_error.h"

// IPAC headers
#include "hardware.h"
#include "conn_manager.h"

/**
 * @brief Function for handling asserts in the SoftDevice.
 *
 * @details This function is called in case of an assert in the SoftDevice.
 *
 * @warning This handler is only an example and is not meant for the final product. You need to analyze
 *          how your product is supposed to react in case of an assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num     Line number of the failing assert call.
 * @param[in] p_file_name  File name of the failing assert call.
 */
void assert_nrf_callback(u_int16_t line_num, const u_int8_t *p_file_name)
{
    app_error_handler(0xDEADBEEF, line_num, p_file_name);
}

/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling the idle state (main loop).
 *
 * @details This function handles any pending log operations, then sleeps until the next event occurs.
 */
static void idle_state_handle(void)
{
    nrf_pwr_mgmt_run();
}

int main(void)
{
    // Initialize.
    hardware_init();
    power_management_init();
    conn_init();

    // Start execution.
    conn_start_scan();

    for (;;)
    {
        idle_state_handle();
    }
}
