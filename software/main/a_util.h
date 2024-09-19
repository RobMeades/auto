/*
 * Copyright 2024 Rob Meades
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
 */

#ifndef _A_UTIL_H_
#define _A_UTIL_H_

/** @file
 * @brief Utility functions.
 */

#ifdef __cplusplus
extern "C" {
#endif

/* ----------------------------------------------------------------
 * COMPILE-TIME MACROS
 * -------------------------------------------------------------- */

/* ----------------------------------------------------------------
 * TYPES
 * -------------------------------------------------------------- */

/* ----------------------------------------------------------------
 * FUNCTIONS
 * -------------------------------------------------------------- */

/** Return the time since this MCU booted.
 *
 * @return  the time since the MCU booted in milliseconds.
 */
int64_t aUtilTimeSinceBootMs();

/** Delay for the given number of milliseconds.  This is an OS
 * delay rather than a busy wait, so other OS tasks may continue.
 * Note that it is also, therefore, subject to the resolution of
 * the OS task, which on ESP-IDF is usually 10 ms.
 *
 * @param timeMs  the time to delay for in milliseconds.
 */
void aUtilDelayMs(size_t timeMs);

#ifdef __cplusplus
}
#endif

/** @}*/

#endif // _A_UTIL_H_

// End of file
