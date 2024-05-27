#pragma once

/**
 * @file pixart.h
 *
 * @brief Common header file for all optical motion sensor by PIXART
 */

#ifdef __cplusplus
extern "C" {
#endif

enum pixart_input_mode { MOVE = 0,
                         SCROLL,
                         SNIPE };

struct device {
    void *config;
    void *data;
};

struct spi_buf {
    uint8_t *buf;
    uint32_t len;
};

struct spi_buf_set {
    const struct spi_buf *buffers;
    uint32_t count;
};

struct k_work {
    void (*work_handler)(struct k_work *work);
};

struct k_work_delayable {
    void (*work_handler)(struct k_work_delayable *work);
};

struct gpio_callback {
    void (*func)(const struct device *gpiob, struct gpio_callback *cb, uint32_t pins);
};

/* device data structure */
struct pixart_data {
    const struct device *dev;

    enum pixart_input_mode curr_mode;
    uint32_t curr_cpi;
    int32_t scroll_delta_x;
    int32_t scroll_delta_y;

#ifdef CONFIG_PMW3610_POLLING_RATE_125_SW
    int64_t last_poll_time;
    int16_t last_x;
    int16_t last_y;
#endif

    // motion interrupt isr
    struct gpio_callback irq_gpio_cb;
    // the work structure holding the trigger job
    struct k_work trigger_work;

    // the work structure for delayable init steps
    struct k_work_delayable init_work;
    int async_init_step;

    //
    bool ready;           // whether init is finished successfully
    bool last_read_burst; // todo: needed?
    int err;              // error code during async init

    // for pmw3610 smart algorithm
    bool sw_smart_flag;
};

struct spi_dt_spec {
};

struct gpio_dt_spec {
    uint8_t port;
    uint8_t pin;
};

// device config data structure
struct pixart_config {
    struct gpio_dt_spec irq_gpio;
    struct spi_dt_spec bus;
    struct gpio_dt_spec cs_gpio;
    size_t scroll_layers_len;
    int32_t *scroll_layers;
    size_t snipe_layers_len;
    int32_t *snipe_layers;
};

#ifdef __cplusplus
}
#endif

/**
 * @}
 */