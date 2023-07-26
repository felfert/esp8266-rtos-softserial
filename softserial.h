#pragma once

#include <FreeRTOS.h>
#include <freertos/event_groups.h>
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

typedef enum {
    SOFTSERIAL_USE_RX    = 1, // Enable input
    SOFTSERIAL_USE_TX    = 2, // Enable output
    SOFTSERIAL_USE_RS485 = 4, // Enable RS485 support (external MAX485 chip required)
} softserial_features_t;

typedef struct {
    /**
     * The desired features of this unit.
     */
    uint8_t features;
    /**
     * The desired baud rate of this unit.
     */
    uint32_t baudrate;
    /**
     * The GPIO pin to be used as RX data (input)
     * Possible range GPIO_NUM_0 .. GPIO_NUM_16
     */
    gpio_num_t rx_pin;
    /**
     * The GPIO pin to be used as TX data (output)
     * Possible range GPIO_NUM_0 .. GPIO_NUM_16
     */
    gpio_num_t tx_pin;
    /**
     * The RS485 TX enable pin (high -> tx enabled)
     * Possible range GPIO_NUM_0 .. GPIO_NUM_16
     */
    gpio_num_t rs485_pin;
    /**
     * Optional RTOS event group for event-based receiving.
     * If set to non-null, this should point to an event group,
     * that was created in your application.
     */
    EventGroupHandle_t event_group;
    /**
     * Optional RTOS event bit to set, if data has been received.
     * If event_group is set AND rx_event is > 0, then
     * xEventGroupSetBitsFromISR() is used in the ISR to
     * notify any task that data has been received.
     */
    EventBits_t rx_event;
    /**
     * Internal use, do not modify directly.
     */
    volatile softserial_buffer_t buffer;
    /**
     * Internal use, do not modify directly.
     */
    uint16_t bit_time;
} softserial;

/**
 * Initialize a softserial unit.
 *
 * @param cfg A softserial struct, specifying the parameters.
 *   The caller must own the struct and keep it accessible.
 * @return ESP_OK or an ESP error code.
 *
 * Please note, that this library uses gpio_install_isr_service()
 * which is incompatible with using gpio_isr_register(). So if your
 * application needs to use other GPIO related ISRs, then you
 * MUST NOT use gpio_isr_register(). See
 * https://docs.espressif.com/projects/esp8266-rtos-sdk/en/latest/api-reference/peripherals/gpio.html#_CPPv424gpio_install_isr_service
 * for more information.
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
extern ssize_t softserial_read(softserial* s, uint8_t* buffer, size_t maxlen);

/**
 * Receive bytes until linefeed found or maxlen is reached.
 *
 * @param s The unit to use for reading.
 * @param buffer The buffer to fill with the received data.
 * @param maxlen The maximum number of bytes to read.
 * @return Number of bytes read (-1 on overrun)
 */
extern ssize_t softserial_readline(softserial* s, uint8_t* buffer, size_t maxlen);

/**
 * Check and reset overrun flag.
 * @param s The unit to use for reading.
 * @return 1 if a buffer overrun has happened, 0 otherwise.
 *
 * Note, that this gets invoked by softserial_read() and softserial_readline()
 */
extern uint8_t softserial_overrun(softserial* s);

/**
 * LOG tag for EXP_LOGx functions.
 */
extern const char *TAG_SOFTSERIAL;

#ifdef __cplusplus
}
#endif
