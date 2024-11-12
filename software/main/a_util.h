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

/** Compute the number of elements in an array.
 */
#define A_UTIL_ARRAY_COUNT(array) (sizeof(array) / sizeof(array[0]))

/** Helper to make sure that lock/unlock pairs are always balanced.
 */
#define A_MUTEX_LOCK(x)      { xSemaphoreTake(x, (TickType_t) portMAX_DELAY)

/** Helper to make sure that lock/unlock pairs are always balanced.
 */
#define A_MUTEX_UNLOCK(x)    } xSemaphoreGive(x)

/* ----------------------------------------------------------------
 * TYPES
 * -------------------------------------------------------------- */

/** Structure to hold a linked-list entry.
 */
typedef struct aUtilLinkedList_t {
    void *pEntry;
    struct aUtilLinkedList_t *pNext;
} aUtilLinkedList_t;

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

/** Set an MCU pin to be an output at the given level; the level
 * on the output pin can be read-back afterwards.
 *
 * @param pin   the MCU pin to set.
 * @param level the level, 0 for low, else high.
 * @return      ESP_OK on success, else error code.
 */
esp_err_t aUtilPinOutputSet(gpio_num_t pin, int32_t level);

/** Add an entry to the start of a linked list.  This function is
 * NOT thread-safe.
 *
 * @param[in] ppList  a pointer to the root of the linked list,
 *                    cannot be NULL.
 * @param[in] pEntry  a pointer to the entry to add to the linked
 *                    list.
 * @return            true if addition of the entry was successful.
 */
bool aUtilLinkedListAdd(aUtilLinkedList_t **ppList, void *pEntry);

/** Remove an entry from a linked list.  This function is NOT thread-safe.
 *
 * @param[in] ppList  a pointer to the root of the linked list,
 *                    cannot be NULL.
 * @param[in] pEntry  the entry to remove; the memory pointed-to is not
 *                    touched in any way, it is up to the caller to free
 *                    it if required.
 */
void aUtilLinkedListRemove(aUtilLinkedList_t **ppList, void *pEntry);

/** Get the first entry in a linked list.  This function is NOT
 * thread-safe and calls to pAUtilLinkedListGetNext() after calling
 * this function should not be interleaved with calls to
 * aUtilLinkedListAdd() or aUtilLinkedListRemove().
 *
 * @param[in] ppList  a pointer to the root of the linked list,
 *                    cannot be NULL.
 * @param[in] ppSaved workspace required by this function, may be
 *                    NULL if you do not plan to call pAUtilLinkedListGetNext().
 * @return            the first entry in the list; NULL if there are
 *                    no entries.
 */
void *pAUtilLinkedListGetFirst(aUtilLinkedList_t **ppList,
                               aUtilLinkedList_t **ppSaved);

/** Get the next entry in a linked list; may be called after
 * pAUtilLinkedListGetFirst() to get subsequent entries.
 * This function is NOT thread-safe and calls to
 * this function should not be interleaved with calls to
 * aUtilLinkedListAdd() or aUtilLinkedListRemove().
 *
 * @param[in] ppSaved the ppSaved that was passed to
 *                    pAUtilLinkedListGetFirstworkspace().
 * @return            the next entry in the list; NULL if there are
 *                    no more entries.
 */
void *pAUtilLinkedListGetNext(aUtilLinkedList_t **ppSaved);

#ifdef __cplusplus
}
#endif

/** @}*/

#endif // _A_UTIL_H_

// End of file
