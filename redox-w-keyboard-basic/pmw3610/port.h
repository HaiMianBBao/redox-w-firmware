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
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "pixart.h"

#define abs(x) ((x) < 0 ? -(x) : (x))

int8_t gpio_pin_configure_dt(const struct gpio_dt_spec *gpio, uint8_t pin_func);
int8_t gpio_pin_set_dt(const struct gpio_dt_spec *gpio, bool enable);
int8_t gpio_pin_interrupt_configure_dt(const struct gpio_dt_spec *gpio, uint8_t enable);
int spi_write_dt(const struct spi_dt_spec *bus, const struct spi_buf_set *buffer);
int spi_read_dt(const struct spi_dt_spec *bus, const struct spi_buf_set *buffer);
int8_t device_is_ready(uint8_t port);
void k_busy_wait(uint32_t tick);
void gpio_init_callback(struct gpio_callback *cb, void (*func)(const struct device *gpiob, struct gpio_callback *cb, uint32_t pins), uint32_t bit);
int8_t gpio_add_callback(const struct gpio_dt_spec *gpio, struct gpio_callback *cb);
void k_work_init(struct k_work *work, void (*cb)(struct k_work *work));
void k_work_init_delayable(struct k_work_delayable *delay_work, void (*cb)(struct k_work_delayable *work));
void input_report_rel(const struct device *dev, uint8_t input_id, int16_t data, bool enable, uint32_t time_out);
void k_work_submit(struct k_work *work);
void k_work_schedule(struct k_work_delayable *work, uint32_t ms);
uint32_t k_uptime_get(void);
