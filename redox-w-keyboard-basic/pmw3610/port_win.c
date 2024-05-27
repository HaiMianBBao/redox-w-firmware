/* Copyright 2023 Cheng Liren
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <Windows.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "port_config.h"
#include "pixart.h"

struct k_work_delayable *gwork = NULL;

void (*work_handler)(struct k_work_delayable *work) = NULL;

static uint32_t current_count = 0;
static uint32_t target_count = 0;

DWORD WINAPI MyThreadFunction(LPVOID lpParam)
{
    while (1) {
        current_count++;
        if (current_count == target_count) {
            work_handler(gwork);
        }

        Sleep(1);
    }

    return 0;
}

void k_work_schedule(struct k_work_delayable *work, uint32_t ms)
{
    work_handler = work->work_handler;
    gwork = work;

    static bool init = false;
    if (!init) {
        HANDLE myHandle;
        DWORD myThreadID;

        // 创建线程
        myHandle = CreateThread(0, 0, MyThreadFunction, 0, 0, &myThreadID);
        if (myHandle == NULL) {
            printf("Failed to create thread\n");
            return;
        }
    }

    target_count = (current_count + ms) % (UINT32_MAX);
}

uint32_t k_uptime_get(void)
{
    return current_count;
}

static struct gpio_callback *g_cb[4] = { NULL };

static void gpio_irq_handler(uint32_t pin, uint32_t action)
{
    uint8_t port = (pin >> 5);
    if (g_cb[port]) {
        g_cb[port]->func(NULL, g_cb[port], pin);
    }
}

int8_t gpio_pin_configure_dt(const struct gpio_dt_spec *gpio, uint8_t pin_func)
{
    LOG_RAW("gpio_pin_configure_dt --> port %d, pin %d ,pin_func %d \r\n", gpio->port, gpio->pin, pin_func);

    if (pin_func == GPIO_INPUT) {
    } else if (pin_func == GPIO_OUTPUT_INACTIVE) {
    }

    return EOK;
}

int8_t gpio_pin_set_dt(const struct gpio_dt_spec *gpio, bool enable)
{
    LOG_RAW("gpio_pin_set_dt --> port %d, pin %d ,enable %d \r\n", gpio->port, gpio->pin, enable);
    return EOK;
}

int8_t gpio_pin_interrupt_configure_dt(const struct gpio_dt_spec *gpio, uint8_t enable)
{
    LOG_RAW("gpio_pin_interrupt_configure_dt --> port %d, pin %d ,enable %d \r\n", gpio->port, gpio->pin, enable);
    if (enable) {
        // nrf_drv_gpiote_in_event_enable(((gpio->port << 5) | gpio->pin), true);
    } else {
        // nrf_drv_gpiote_in_event_enable(((gpio->port << 5) | gpio->pin), false);
    }

    return EOK;
}

int8_t spi_write_dt(const struct spi_dt_spec *bus, const struct spi_buf_set *buffer)
{
    LOG_RAW("spi_write_dt --> buffer_count %d buffer_len %d", buffer->count, buffer->buffers->len);

    for (uint32_t i = 0; i < buffer->buffers->len; i++) {
        LOG_RAW("%02x ", buffer->buffers[i]);
    }

    LOG_RAW("\r\n");
    return EOK;
}

int8_t spi_read_dt(const struct spi_dt_spec *bus, const struct spi_buf_set *buffer)
{
    LOG_RAW("spi_read_dt --> buffer_count %d buffer_len %d\r\n", buffer->count, buffer->buffers->len);
    return EOK;
}

int8_t device_is_ready(uint8_t port)
{
    LOG_RAW("device_is_ready --> \r\n");
    return 1;
}

void k_busy_wait(uint32_t tick)
{
    LOG_RAW("k_busy_wait --> us %d \r\n", tick);
    /* delay us */
}

void gpio_init_callback(struct gpio_callback *cb, void (*func)(const struct device *gpiob, struct gpio_callback *cb, uint32_t pins), uint32_t bit)
{
    LOG_RAW("gpio_init_callback --> \r\n");
    static bool has_init = false;
    if (!has_init) {
        has_init = true;
        // nrf_drv_gpiote_init();
    }

    cb->func = func;
}

int8_t gpio_add_callback(const struct gpio_dt_spec *gpio, struct gpio_callback *cb)
{
    LOG_RAW("gpio_add_callback --> port %d pin %d \r\n", gpio->port, gpio->pin);

    // static nrf_drv_gpiote_in_config_t gpiote_in_config = NRFX_GPIOTE_CONFIG_IN_SENSE_HITOLO(false); /* Falling edge trigger */
    // gpiote_in_config.pull = NRF_GPIO_PIN_PULLUP;
    // gpiote_in_config.sense = GPIOTE_CONFIG_POLARITY_HiToLo;
    // nrf_drv_gpiote_in_init(((gpio->port << 5) | gpio->pin), &gpiote_in_config, gpio_irq_handler);

    if (gpio->port > 4) {
        LOG_ERR("gpio_add_callback");
        return EIO;
    }

    g_cb[gpio->port] = cb;

    return EOK;
}

void k_work_init(struct k_work *work, void (*cb)(struct k_work *work))
{
    work->work_handler = cb;
    LOG_RAW("k_work_init -->\r\n");
}

void k_work_init_delayable(struct k_work_delayable *delay_work, void (*cb)(struct k_work_delayable *work))
{
    delay_work->work_handler = cb;
    LOG_RAW("k_work_init_delayable -->\r\n");
}

void input_report_rel(const struct device *dev, uint8_t input_id, int16_t data, bool enable, uint32_t time_out)
{
    LOG_RAW("input_report_rel --> input_id %d data %d enable %d time_out %d\r\n", input_id, data, enable, time_out);
}

void k_work_submit(struct k_work *work)
{
    LOG_RAW("k_work_submit -->\r\n");
    if (work == NULL) {
        LOG_ERR("work == NULL");
        return;
    }

    if (work->work_handler == NULL) {
        LOG_ERR("work->work_handler == NULL");
        return;
    }

    work->work_handler((struct k_work *)work);
}

static inline void delay_ms(uint32_t ms)
{
    LOG_INF("delay_ms --> %d ms", ms);
}
