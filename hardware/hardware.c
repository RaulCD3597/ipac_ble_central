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
#include "conn_manager.h"

/* ----------------  local definitions ----------------*/

#define RX_PIN_NUMBER               8
#define TX_PIN_NUMBER               6
#define RTS_PIN_NUMBER              5
#define CTS_PIN_NUMBER              7
#define UART_TX_BUF_SIZE            256 /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE            256 /**< UART RX buffer size. */
#define BED_STRING_CMD              "\"bed\":"
#define CALL_STRING_CMD             "\"start_call\":"

/* -----------------  local variables -----------------*/

/* ------------ local functions prototypes ------------*/

static void uart_init(void);
static void uart_event_handle(app_uart_evt_t * p_event);
static void uart_payload_parser(const uint8_t * payload);
static void timer_init(void);
static void leds_init(void);

/* ----------------- public functions -----------------*/

void hardware_init(void)
{
    uart_init();
    timer_init();
    leds_init();
}

/**
 * @brief Function for handling characters received by the Nordic UART Service (NUS).
 *
 * @details This function takes a list of characters of length data_len and prints the characters out on UART.
 */
void uart_send_string(uint8_t * p_data, uint16_t data_len)
{
    ret_code_t ret_val;

    for (uint32_t i = 0; i < data_len; i++)
    {
        do
        {
            ret_val = app_uart_put(p_data[i]);
            if ((ret_val != NRF_SUCCESS) && (ret_val != NRF_ERROR_BUSY))
            {
                APP_ERROR_CHECK(ret_val);
            }
        } while (ret_val == NRF_ERROR_BUSY);
    }
    while (app_uart_put('\n') == NRF_ERROR_BUSY)
        ;
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
    static uint8_t data_array[UART_RX_BUF_SIZE];
    static uint16_t index = 0;

    switch (p_event->evt_type)
    {
    case APP_UART_DATA_READY:
        UNUSED_VARIABLE(app_uart_get(&data_array[index]));
        index++;

        if ((data_array[index - 1] == '\n') ||
            (data_array[index - 1] == '\r') ||
            (index >= UART_RX_BUF_SIZE ))
        {
            uart_payload_parser((const uint8_t *)data_array);

            index = 0;
        }
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
 * @brief   Function for parsing uart frames.
 *
 * @details This function searches for a known command and triggers the action required
 *      by the commad received.
 */
static void uart_payload_parser(const uint8_t * payload)
{
    uint8_t *received = (uint8_t *)payload;

    if (NULL != (received = (uint8_t *)strstr((const char *)received, BED_STRING_CMD)))
    {
        received += strlen(BED_STRING_CMD);
        while (' ' == *received)
        {
            received++;
        }
        uint8_t nus_instance = (*received) - '1';

        received = (uint8_t *)payload;
        if (NULL != (received = (uint8_t *)strstr((const char *)received, CALL_STRING_CMD)))
        {
            received += strlen(CALL_STRING_CMD);
            while (' ' == *received)
            {
                received++;
            }
            if (!memcmp(received, "true", 4))
            {
                uint8_t cmd[] = "{\"on_call\": true}";
                conn_send_string(cmd, strlen((const char *)cmd), nus_instance);
            }
            else if (!memcmp(received, "false", 5))
            {
                uint8_t cmd[] = "{\"on_call\": false}";
                conn_send_string(cmd, strlen((const char *)cmd), nus_instance);
            }
        }
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