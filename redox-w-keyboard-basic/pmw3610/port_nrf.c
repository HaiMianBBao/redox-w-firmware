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
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "port_config.h"
#include "pixart.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "nrf_drv_gpiote.h"
#include "iopincfg.h"
#include "spi.h"

#define SPI_SCK_PIN     17
#define SPI_MOSI_PIN    20
#define SPI_MISO_PIN    20
#define SPI_CS_PIN      24
#define PMW3610_IRQ_PIN 22

#define NRF_GPIO_GET_PORT(pin)       (pin >> 5)
#define NRF_GPIO_GET_PIN_NUMBER(pin) (pin & 0b11111)
#define NRF_GPIO_GET_PIN(port, pin)  (port << 5 | pin)

#define MAX_PORT 4
uint32_t en_cb = 0x0;
static struct gpio_callback *g_cb[MAX_PORT] = { NULL };

static SPIDev_t vDevData;

static void gpio_irq_handler(uint32_t pin, nrf_gpiote_polarity_t action)
{
    uint8_t port = NRF_GPIO_GET_PORT(pin);
    uint8_t local_pin = NRF_GPIO_GET_PIN_NUMBER(pin);
    if (g_cb[port]) {
        if (en_cb & (1 << local_pin)) {
            g_cb[port]->func(NULL, g_cb[port], local_pin);
        }
    }
}

int8_t gpio_pin_configure_dt(const struct gpio_dt_spec *gpio, uint8_t pin_func)
{
    LOG_RAW("gpio_pin_configure_dt --> port %d, pin %d ,pin_func %d \r\n", gpio->port, gpio->pin, pin_func);

    if (pin_func == GPIO_INPUT) {
        nrf_gpio_cfg_input(NRF_GPIO_GET_PIN(gpio->port, gpio->pin), NRF_GPIO_PIN_NOPULL);
    } else if (pin_func == GPIO_OUTPUT_INACTIVE) {
        nrf_gpio_cfg_output(NRF_GPIO_GET_PIN(gpio->port, gpio->pin));
    }

    return EOK;
}

int8_t gpio_pin_set_dt(const struct gpio_dt_spec *gpio, bool enable)
{
    LOG_RAW("gpio_pin_set_dt --> port %d, pin %d ,enable %d \r\n", gpio->port, gpio->pin, enable);

    if (enable) {
        nrf_gpio_pin_set(NRF_GPIO_GET_PIN(gpio->port, gpio->pin));
    } else {
        nrf_gpio_pin_clear(NRF_GPIO_GET_PIN(gpio->port, gpio->pin));
    }

    return EOK;
}

int8_t gpio_pin_interrupt_configure_dt(const struct gpio_dt_spec *gpio, uint8_t enable)
{
    LOG_RAW("gpio_pin_interrupt_configure_dt --> port %d, pin %d ,enable %d \r\n", gpio->port, gpio->pin, enable);
    if (enable) {
        nrf_drv_gpiote_in_event_enable(NRF_GPIO_GET_PIN(gpio->port, gpio->pin), true);
    } else {
        nrf_drv_gpiote_in_event_enable(NRF_GPIO_GET_PIN(gpio->port, gpio->pin), false);
    }

    return EOK;
}

int spi_write_dt(const struct spi_dt_spec *bus, const struct spi_buf_set *buffer)
{
    if (buffer == NULL || buffer->count <= 0) {
        return -EIO;
    }

    LOG_RAW("spi_write_dt --> buffer_count %d buffer_len %d", buffer->count, buffer->buffers->len);

    for (uint32_t i = 0; i < buffer->buffers->len; i++) {
        LOG_RAW("%02x ", buffer->buffers[i]);
    }

    LOG_RAW("\r\n");

    /* Real send */
    DevIntrf_t *const pDev = &vDevData.DevIntrf;
    uint32_t DevAddr = 0;
    uint8_t *pBuff = buffer->buffers[0].buf;
    int BuffLen = buffer->buffers[0].len;

    int count = 0;
    int nrtry = pDev->MaxRetry;

    do {
        if (DeviceIntrfStartTx(pDev, DevAddr)) {
            pDev->bNoStop = false;
            count = pDev->TxData(pDev, pBuff, BuffLen);
            if (count < 0) {
                break;
            }
            DeviceIntrfStopTx(pDev);
        }
    } while (count == 0 && nrtry-- > 0);

    return count;
}

int spi_read_dt(const struct spi_dt_spec *bus, const struct spi_buf_set *buffer)
{
    if (buffer == NULL || buffer->count <= 0) {
        return -EIO;
    }

    LOG_RAW("spi_read_dt --> buffer_count %d buffer_len %d\r\n", buffer->count, buffer->buffers->len);

    DevIntrf_t *const pDev = &vDevData.DevIntrf;
    uint32_t DevAddr = 0;
    uint8_t *pBuff = buffer->buffers[0].buf;
    int BuffLen = buffer->buffers[0].len;

    int count = 0;
    int nrtry = pDev->MaxRetry;

    do {
        if (DeviceIntrfStartRx(pDev, DevAddr)) {
            pDev->bNoStop = false;
            count = pDev->RxData(pDev, pBuff, BuffLen);

            if (count < 0) {
                break;
            }
            DeviceIntrfStopRx(pDev);
        }
    } while (count == 0 && nrtry-- > 0);

    return count;
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
    nrf_delay_us(tick);
}

void gpio_init_callback(struct gpio_callback *cb, void (*func)(const struct device *gpiob, struct gpio_callback *cb, uint32_t pins), uint32_t bit)
{
    LOG_RAW("gpio_init_callback --> \r\n");
    static bool has_init = false;
    if (!has_init) {
        has_init = true;
        __ASSERT_NO_MSG(nrf_drv_gpiote_init());
    }

    en_cb |= bit; /* connect pin callback */
    cb->func = func;
}

int8_t gpio_add_callback(const struct gpio_dt_spec *gpio, struct gpio_callback *cb)
{
    LOG_RAW("gpio_add_callback --> port %d pin %d \r\n", gpio->port, gpio->pin);

#define NRFX_GPIOTE_CONFIG_IN_SENSE_HITOLO(hi_accu) \
    {                                               \
        .sense = NRF_GPIOTE_POLARITY_HITOLO,        \
        .pull = NRF_GPIO_PIN_NOPULL,                \
        .is_watcher = false,                        \
        .hi_accuracy = hi_accu,                     \
    }

    static nrf_drv_gpiote_in_config_t gpiote_in_config = NRFX_GPIOTE_CONFIG_IN_SENSE_HITOLO(false); /* Falling edge trigger */
    gpiote_in_config.pull = NRF_GPIO_PIN_PULLUP;
    gpiote_in_config.sense = GPIOTE_CONFIG_POLARITY_HiToLo;
    __ASSERT_NO_MSG(nrf_drv_gpiote_in_init(NRF_GPIO_GET_PIN(gpio->port, gpio->pin), &gpiote_in_config, gpio_irq_handler));

    if (gpio->port > MAX_PORT) {
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

uint32_t current_count = 0;
uint32_t target_count = 0;

struct k_work_delayable *gwork = NULL;
void (*work_handler)(struct k_work_delayable *work) = NULL;

void k_work_schedule(struct k_work_delayable *work, uint32_t ms)
{
    work_handler = work->work_handler;
    gwork = work;
    target_count = (current_count + ms) % (UINT32_MAX);
}

uint32_t k_uptime_get(void)
{
    return current_count;
}

/* Init */
static struct device pmw3610_device;
static struct pixart_data pmw3610_pixart_data;
static struct pixart_config pmw3610_pixart_config;

//********** SPI Master **********
static const IOPinCfg_t s_SpiMasterPins[] = {
    { 0, SPI_SCK_PIN, 1,
      IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL }, // SCK
    { 0, SPI_MISO_PIN, 1,
      IOPINDIR_INPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL }, // MISO
    { 0, SPI_MOSI_PIN, 1,
      IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL }, // MOSI
    { 0, SPI_CS_PIN, 1,
      IOPINDIR_OUTPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL }, // CS
};

static const SPICfg_t s_SpiMasterCfg = {
    0, /* SPI 0 */
    SPIPHY_3WIRE,
    SPIMODE_MASTER,
    s_SpiMasterPins,
    sizeof(s_SpiMasterPins) / sizeof(IOPinCfg_t),
    1000000, // Speed in Hz
    8,       // Data Size
    5,       // Max retries
    SPIDATABIT_MSB,
    SPIDATAPHASE_SECOND_CLK, // Data phase
    SPICLKPOL_LOW,           // clock polarity
    SPICSEL_AUTO,
    false, // DMA
    false,
    6, //APP_IRQ_PRIORITY_LOW,      // Interrupt priority
    0xff,
    NULL
};

void pmw3610_init_public(void)
{
    memset(&pmw3610_device, 0, sizeof(pmw3610_device));
    memset(&pmw3610_pixart_data, 0, sizeof(pmw3610_pixart_data));
    memset(&pmw3610_pixart_config, 0, sizeof(pmw3610_pixart_config));

    /* pmw3610_pixart_config */
    pmw3610_pixart_config.cs_gpio.port = 0;
    pmw3610_pixart_config.cs_gpio.pin = SPI_CS_PIN;
    pmw3610_pixart_config.irq_gpio.port = 0;
    pmw3610_pixart_config.irq_gpio.pin = PMW3610_IRQ_PIN;

    pmw3610_device.data = &pmw3610_pixart_data;
    pmw3610_device.config = &pmw3610_pixart_config;

    /* Init spi */
    memset((void *)&vDevData, 0, (int)sizeof(vDevData));

    extern bool SPIInit(SPIDev_t *const pDev, const SPICfg_t *pCfgData);

    if (SPIInit(&vDevData, &s_SpiMasterCfg)) {
        LOG_INF("SPI Bus init success");
    }

    extern int pmw3610_init(const struct device *dev);
    pmw3610_init(&pmw3610_device);
}
