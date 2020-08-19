/**
 * @file hardware.c
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
#include "nrf.h"
#include "nrf_gpio.h"

// nrf app headers
#include "app_timer.h"
#include "app_error.h"
#include "app_uart.h"
#include "app_util.h"

// ipac headers
#include "hardware.h"

/* ----------------  local definitions ----------------*/

#define RX_PIN_NUMBER             8
#define TX_PIN_NUMBER             6
#define RTS_PIN_NUMBER            5
#define CTS_PIN_NUMBER            7
#define UART_TX_BUF_SIZE 256 /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE 256 /**< UART RX buffer size. */

/* -----------------  local variables -----------------*/

/* ------------ local functions prototypes ------------*/

static void uart_init(void);
static void uart_event_handle(app_uart_evt_t * p_event);
static void timer_init(void);
static void leds_init(void);

/* ----------------- public functions -----------------*/

void hardware_init(void)
{
    uart_init();
    timer_init();
    leds_init();
}

/* -----------------  local functions -----------------*/

/**
 * @brief Function for initializing the UART. 
 */
static void uart_init(void)
{
    ret_code_t err_code;

    app_uart_comm_params_t const comm_params =
        {
            .rx_pin_no = RX_PIN_NUMBER,
            .tx_pin_no = TX_PIN_NUMBER,
            .rts_pin_no = RTS_PIN_NUMBER,
            .cts_pin_no = CTS_PIN_NUMBER,
            .flow_control = APP_UART_FLOW_CONTROL_DISABLED,
            .use_parity = false,
            .baud_rate = UART_BAUDRATE_BAUDRATE_Baud115200};

    APP_UART_FIFO_INIT(&comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uart_event_handle,
                       APP_IRQ_PRIORITY_LOWEST,
                       err_code);

    APP_ERROR_CHECK(err_code);
}

/**
 * @brief   Function for handling app_uart events.
 *
 * @details This function receives a single character from the app_uart module and appends it to
 *          a string. The string is sent over BLE when the last character received is a
 *          'new line' '\n' (hex 0x0A) or if the string reaches the maximum data length.
 */
static void uart_event_handle(app_uart_evt_t *p_event)
{
    switch (p_event->evt_type)
    {
    case APP_UART_DATA_READY:
        break;

    case APP_UART_COMMUNICATION_ERROR:
        APP_ERROR_HANDLER(p_event->data.error_communication);
        break;

    case APP_UART_FIFO_ERROR:
        APP_ERROR_HANDLER(p_event->data.error_code);
        break;

    default:
        break;
    }
}

/**
 * @brief Function for initializing the timer.
 */
static void timer_init(void)
{
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
}

/**
 * @brief Function for the LEDs initialization.
 *
 * @details Initializes all LEDs used by the application.
 */
static void leds_init(void)
{
    uint8_t led = CENTRAL_SCANNING_LED;
    for (; led <= CENTRAL_CONNECTED_LED; led++)
    {
        nrf_gpio_cfg_output(led);
        nrf_gpio_pin_set(led);
    }
}