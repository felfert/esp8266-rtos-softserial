#include "softserial.h"

#include "rom/ets_sys.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"

/* Log TAG */
static const char* TAG      = "softserial";

static int64_t TMAX = 0x7fffffffffffffff;

// Bitmask of GPIO pins in use (1 = GPIO in use);
static uint32_t used_pins = 0;

static uint8_t numinstances = 0;

/**
 * Check, if specified GPIO pins are not overlapping or already in use.
 */
static esp_err_t check_pins(uint32_t out_pbm, uint32_t in_pbm)
{
    if (in_pbm & out_pbm) {
        ESP_LOGE(TAG, "TX pin(s) and RX pin must not be the same");
        return ESP_ERR_INVALID_ARG;
    }
    if (used_pins & out_pbm) {
        ESP_LOGE(TAG, "TX pin(s) already in use");
        return ESP_ERR_INVALID_ARG;
    }
    if (used_pins & in_pbm) {
        ESP_LOGE(TAG, "RX pin already in use");
        return ESP_ERR_INVALID_ARG;
    }
    used_pins |= (in_pbm | out_pbm);
    return ESP_OK;
}

/**
 * The actual ISR.
 * @param s Pointer to the corresponding instance.
 */
static void softserial_isr(void* arg)
{
    softserial* s = (softserial *)arg;
    uint8_t level;

    // Disable interrupts for RX pin
    gpio_set_intr_type(s->rx_pin, GPIO_INTR_DISABLE);

    // Check level
    level = gpio_get_level(s->rx_pin);
    if (!level) {
        // Pin is low therefore we have a start bit
        // Wait till start bit is half over so we can sample the next one in the center
        os_delay_us(s->bit_time / 2);

        // Now sample bits
        unsigned i;
        uint8_t data = 0;
        uint64_t start_time = TMAX & esp_timer_get_time();
        for (i = 0; i < 8; i++) {
            while ((TMAX & esp_timer_get_time()) < (start_time + (s->bit_time * (i + 1)))) {
                if ((TMAX & esp_timer_get_time()) < start_time) {
                    // Timer overflow: escape from while loop
                    break;
                }
            }
            data >>= 1;
            // Read bit
            if (gpio_get_level(s->rx_pin)) {
                data |= 0x80;
            }
        }
        uint8_t next = (s->buffer.tail + 1) % SOFTSERIAL_MAX_RX_BUF;
        if (next != s->buffer.head) {
            // Save new data in buffer: tail points to where byte goes
            s->buffer.data[s->buffer.tail] = data;
            s->buffer.tail = next;
        }
        else {
            // buffer is full, set the overrun flag
            s->buffer.overrun = 1;
        }
        // Wait for stop bit
        os_delay_us(s->bit_time);
    }
    // Reactivate interrupts for RX pin
    gpio_set_intr_type(s->rx_pin, GPIO_INTR_ANYEDGE);
}

esp_err_t softserial_init(softserial* s)
{

    esp_err_t ret;
    gpio_config_t tx_gpio_conf = {
        .pin_bit_mask = 1ULL << s->tx_pin,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config_t rx_gpio_conf = {
        .pin_bit_mask = 1ULL << s->rx_pin,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_ANYEDGE,
    };
    if (s->use_rs485) {
        tx_gpio_conf.pin_bit_mask |= (1ULL << s->rs485_tx_enable_pin);
    }
    ret = check_pins(tx_gpio_conf.pin_bit_mask, rx_gpio_conf.pin_bit_mask);
    if (ESP_OK != ret) {
        return ret;
    }

    // Set bit time
    if (s->baudrate <= 0) {
        ESP_LOGE(TAG, "Invalid baud rate (%d)", s->baudrate);
        return ESP_ERR_INVALID_ARG;
    }
    else {
        s->bit_time = (1000000 / s->baudrate);
        if ( ((100000000 / s->baudrate) - (100 * s->bit_time)) > 50 ) {
            s->bit_time++;
        }
        ESP_LOGD(TAG, "bit_time is %d", s->bit_time);
    }

    if (0 == numinstances) {
        ret = gpio_install_isr_service(0);
        if (ret) {
            return ret;
        }
    }
    numinstances++;

    // Init TX pin and RS485 TX enable pin
    ESP_LOGD(TAG, "TX init");
    ret = gpio_config(&tx_gpio_conf);
    if (ESP_OK != ret) {
        ESP_LOGE(TAG, "Invalid TX setup");
        return ret;
    }
    ESP_LOGD(TAG, "TX init done");

    // Init RX pin
    ret = gpio_config(&rx_gpio_conf);
    if (ESP_OK != ret) {
        ESP_LOGE(TAG, "Invalid RX setup");
        return ret;
    }
    ESP_LOGD(TAG, "RX init done");

    // Register isr_handler
    ret = gpio_isr_handler_add(s->rx_pin, softserial_isr, (void *)s);
    if (ESP_OK != ret) {
        ESP_LOGE(TAG, "Failed to add ISR handler");
        return ret;
    }
    ESP_LOGD(TAG, "init done");
    return ESP_OK;
}



uint8_t softserial_getc(softserial* s)
{
    // Empty buffer?
    if (s->buffer.head == s->buffer.tail) {
        return 0;
    }

    // Fetch next byte from head
    uint8_t d = s->buffer.data[s->buffer.head];
    s->buffer.head = (s->buffer.head + 1) % SOFTSERIAL_MAX_RX_BUF;
    return d;
}

// Is data available?
uint8_t softserial_available(softserial* s)
{
    return (s->buffer.tail + SOFTSERIAL_MAX_RX_BUF - s->buffer.head) % SOFTSERIAL_MAX_RX_BUF;
}

static inline uint8_t chbit(uint8_t data, uint8_t bit)
{
    if ((data & bit) != 0) {
        return 1;
    }
    else {
        return 0;
    }
}

esp_err_t softserial_putchar(softserial* s, uint8_t data)
{
    esp_err_t ret;
    unsigned i;
    int64_t start_time = TMAX & esp_timer_get_time();

    if (s->use_rs485) {
        // TX enable
        ret = gpio_set_level(s->rs485_tx_enable_pin, 1);
        if (ESP_OK != ret) {
            return ret;
        }
    }

    // Start Bit
    ret = gpio_set_level(s->tx_pin, 0);
    if (ESP_OK != ret) {
        return ret;
    }
    for (i = 0; i < 8; i ++ ) {
        while ((TMAX & esp_timer_get_time()) < (start_time + (s->bit_time*(i+1)))) {
            // Timer overflow: escape from while loop
            if ((TMAX & esp_timer_get_time()) < start_time) {
                break;
            }
        }
        ret = gpio_set_level(s->tx_pin, chbit(data, 1 << i));
        if (ESP_OK != ret) {
            return ret;
        }
    }

    // Stop bit
    while ((TMAX & esp_timer_get_time()) < (start_time + (s->bit_time*9))) {
        // Timer overflow: escape from while loop
        if ((TMAX & esp_timer_get_time()) < start_time) {
            break;
        }
    }
    ret = gpio_set_level(s->tx_pin, 1);
    if (ESP_OK != ret) {
        return ret;
    }

    // Delay after byte, for new sync
    os_delay_us(s->bit_time * 6);

    if (s->use_rs485) {
        // TX disable
        ret = gpio_set_level(s->rs485_tx_enable_pin, 0);
        if (ESP_OK != ret) {
            return ret;
        }
    }
    return ESP_OK;
}

esp_err_t softserial_puts(softserial* s, const uint8_t* str)
{
    while (*str) {
        int ret = softserial_putchar(s, *(str++));
        if (ESP_OK != ret) {
            return ret;
        }
    }
    return ESP_OK;
}

uint8_t softserial_overrun(softserial *s)
{
    uint8_t ret = s->buffer.overrun;
    if (ret) {
        s->buffer.overrun = 0;
    }
    return ret;
}

static inline size_t read_internal(softserial* s, uint8_t* buffer, size_t maxlen, uint8_t checklf)
{
    uint8_t ch;
    size_t len = 0;
    if (softserial_overrun(s)) {
        return -1;
    }

    while (softserial_available(s)) {
        ch = softserial_getc(s);

        if (checklf && (ch == '\n')) {
            break;
        }
        else if (len < maxlen - 1) {
            *buffer++ = ch;
            len++;
        }
        else {
            break;
        }
    }
    // Terminate string
    *buffer++ = '\0';

    return len;
}

size_t softserial_readline(softserial* s, uint8_t* buffer, size_t maxlen)
{
    return read_internal(s, buffer, maxlen, 1);
}

size_t softserial_read(softserial* s, uint8_t* buffer, size_t maxlen)
{
    return read_internal(s, buffer, maxlen, 0);
}
