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

// 12-bit two's complement value to int16_t
// adapted from https://stackoverflow.com/questions/70802306/convert-a-12-bit-signed-number-in-c
#define TOINT16(val, bits) (((struct { int16_t value : bits; }){ val }).value)
#define ARRAY_SIZE(arr)    (sizeof(arr) / sizeof((arr)[0]))

#define CONTAINER_OF(ptr, type, member) \
    ((type *)((char *)(ptr)-offsetof(type, member)))

#define BIT(x) (1 << (x))

#define unlikely(x) (x)

#define K_MSEC(x) (x)

#define LOG_ERR(format, ...)                                                     \
    do {                                                                         \
        printf("ERROR: %s:%d: " format "\n", __FILE__, __LINE__, ##__VA_ARGS__); \
    } while (0)

#define LOG_WRN(format, ...)                            \
    do {                                                \
        printf("WARNING: " format "\n", ##__VA_ARGS__); \
    } while (0)

#define LOG_INF(format, ...)                         \
    do {                                             \
        printf("INFO: " format "\n", ##__VA_ARGS__); \
    } while (0)

#define LOG_RAW(format, ...)                         \
    do {                                             \
        printf(format, ##__VA_ARGS__);               \
    } while (0)

#ifdef NDEBUG
#define __ASSERT_NO_MSG(expr) ((void)0)
#else
#define __ASSERT_NO_MSG(expr)                                                              \
    do {                                                                                   \
        if (!(expr)) {                                                                     \
            printf("Assertion failed: %s, file %s, line %d\n", #expr, __FILE__, __LINE__); \
            while (1) {                                                                    \
            }                                                                              \
        }                                                                                  \
    } while (0)
#endif

#define IS_ENABLED(option) ((option == 1))

/* Error code */
#ifndef EOK
#define EOK     0
#endif

#ifndef EIO
#define EIO     127
#endif

#ifndef EINVAL
#define EINVAL  126
#endif

#ifndef ENOTSUP
#define ENOTSUP 125
#endif

#ifndef EBUSY
#define EBUSY   124
#endif

#ifndef ENODEV
#define ENODEV  123
#endif

//TODO: 暂不清楚这些参数
#define CONFIG_PMW3610_REST1_SAMPLE_TIME_MS    40
#define CONFIG_PMW3610_REST2_SAMPLE_TIME_MS    0
#define CONFIG_PMW3610_RUN_DOWNSHIFT_TIME_MS   128
#define CONFIG_PMW3610_REST1_DOWNSHIFT_TIME_MS 9600
#define CONFIG_PMW3610_REST2_DOWNSHIFT_TIME_MS 0
#define CONFIG_PMW3610_REST3_SAMPLE_TIME_MS    0

#define CONFIG_PMW3610_CPI               2400
#define CONFIG_PMW3610_CPI_DIVIDOR       4
#define CONFIG_PMW3610_SNIPE_CPI         800
#define CONFIG_PMW3610_SNIPE_CPI_DIVIDOR 4
#define CONFIG_PMW3610_SCROLL_TICK       32
#define CONFIG_PMW3610_POLLING_RATE_250
// #define CONFIG_PMW3610_POLLING_RATE_125_SW
#define CONFIG_PMW3610_SMART_ALGORITHM

#define CONFIG_PMW3610_ORIENTATION_0   false
#define CONFIG_PMW3610_ORIENTATION_90  true
#define CONFIG_PMW3610_ORIENTATION_180 false
#define CONFIG_PMW3610_ORIENTATION_270 false
#define CONFIG_PMW3610_INVERT_X        false
#define CONFIG_PMW3610_INVERT_Y        false

#define GPIO_INT_LEVEL_ACTIVE 1
#define GPIO_INT_DISABLE      0

#define K_FOREVER        0xffff
#define INPUT_REL_X      0
#define INPUT_REL_Y      1
#define INPUT_REL_WHEEL  2
#define INPUT_REL_HWHEEL 3

#define GPIO_INPUT 0
#define GPIO_OUTPUT_INACTIVE 1
