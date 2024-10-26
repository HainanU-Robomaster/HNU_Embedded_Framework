/******************************************************************************
 * Copyright 2020-2023 The Firmament Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#ifndef MAG_H__
#define MAG_H__

#include <rtthread.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifdef BSP_USING_IST8310
#include "ist8310.h"
#endif /* BSP_USING_IST8310 */

#define MAG_RANGE_2GA  2
#define MAG_RANGE_4GA  4
#define MAG_RANGE_8GA  6
#define MAG_RANGE_12GA 12
#define MAG_RANGE_16GA 16

struct mag_configure {
    rt_uint32_t sample_rate_hz; /* sample rate in Hz */
    rt_uint16_t dlpf_freq_hz;   /* internal low-pass filter cur-off freq in Hz */
    rt_uint32_t mag_range_ga;   /* mag measure range in gauss */
};

struct mag_ops{
    rt_err_t (*mag_init)(const char* i2c_bus_name);
    rt_err_t (*mag_read)(float data[3]);
};

#ifdef __cplusplus
}
#endif

#endif
