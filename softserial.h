#pragma once

#include <driver/gpio.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SOFTSERIAL_MAX_RX_BUF 64

typedef struct {
    uint8_t data[SOFTSERIAL_MAX_RX_BUF];
    uint8_t tail;
    uint8_t head;
    uint8_t overrun;
} softserial_buffer_t;

typedef struct {
    uint32_t baudrate;
    gpio_num_t rx_pin;
    gpio_num_t tx_pin;
    // Optional RS485 tx enable pin (high -> tx enabled)
    gpio_num_t rs485_tx_enable_pin;
    // Flag: Enable RS485
    uint8_t use_rs485;
    volatile softserial_buffer_t buffer;
    uint16_t bit_time;
} softserial;

/**
 * Initialize a softserial unit.
 *
 * @param cfg A softserial struct, specifying the parameters.
 *   The caller must own the struct and keep it accessible.
 * @return ESP_OK or an ESP error code
 */
extern esp_err_t softserial_init(softserial* cfg);

/**
 * Query availability of received data.
 *
 * @param s The unit to query.
 * @return true, if data is available, false otherwise.
 */
extern uint8_t softserial_available(softserial* s);

/**
 * Send a single byte.
 *
 * @param s The unit to use for sending.
 * @param data The data to send.
 * @return ESP_OK or an ESP error code
 */
extern esp_err_t softserial_putchar(softserial* s, uint8_t data);

/**
 * Send a string of bytes.
 *
 * @param s The unit to use for sending.
 * @param str The 0-terminated data to send.
 * @return ESP_OK or an ESP error code
 */
extern esp_err_t softserial_puts(softserial* s, const uint8_t* str);

/**
 * Receive a single byte.
 * @param s The unit to use for sending.
 * @return The data byte or 0 if the buffer was empty.
 */
extern uint8_t softserial_getc(softserial* s);

/**
 * Receive bytes.
 *
 * @param s The unit to use for reading.
 * @param buffer The buffer to fill with the received data.
 * @param maxlen The maximum number of bytes to read.
 * @return Number of bytes read (-1 on overrun)
 */
extern size_t softserial_read(softserial* s, uint8_t* buffer, size_t maxlen);

/**
 * Receive bytes until linefeed found or maxlen is reached.
 *
 * @param s The unit to use for reading.
 * @param buffer The buffer to fill with the received data.
 * @param maxlen The maximum number of bytes to read.
 * @return Number of bytes read (-1 on overrun)
 */
extern size_t softserial_readline(softserial* s, uint8_t* buffer, size_t maxlen);

/**
 * Check and reset overrun flag.
 */
extern uint8_t softserial_overrun(softserial* s);

#ifdef __cplusplus
}
#endif
