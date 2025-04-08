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

/** @file
 * @brief Utility functions.
 */

#include <stdlib.h> // For malloc()/free()

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <esp_timer.h>

#include <driver/gpio.h>

#include <a_util.h>

/* ----------------------------------------------------------------
 * COMPILE-TIME MACROS: MISC
 * -------------------------------------------------------------- */

// Prefix for all logging prints from this file.
#define A_LOG_TAG "A_UTIL: "

/* ----------------------------------------------------------------
 * STATIC FUNCTIONS
 * -------------------------------------------------------------- */

/* ----------------------------------------------------------------
 * PUBLIC FUNCTIONS
 * -------------------------------------------------------------- */

// Wot it says.
int64_t aUtilTimeSinceBootMs()
{
    return esp_timer_get_time() / 1000;
}

// Task block for the given number of milliseconds.
void aUtilDelayMs(size_t timeMs)
{
    vTaskDelay(timeMs / portTICK_PERIOD_MS);
}

// Add an entry to the front of a linked list.
bool aUtilLinkedListAdd(aUtilLinkedList_t **ppList, void *pEntry)
{
    aUtilLinkedList_t *pListEntry = NULL;
    aUtilLinkedList_t *pListTmp;
    bool success = false;

    if (ppList != NULL) {
        pListEntry = malloc(sizeof(aUtilLinkedList_t));
        if (pListEntry != NULL) {
            pListEntry->pEntry = pEntry;
            pListTmp = *ppList;
            *ppList = pListEntry;
            pListEntry->pNext = pListTmp;
            success = true;
        }
    }

    return success;
}

// Remove an entry from a linked list.
void aUtilLinkedListRemove(aUtilLinkedList_t **ppList, void *pEntry)
{
    aUtilLinkedList_t *pThis;
    aUtilLinkedList_t *pPrev;

    if (ppList != NULL) {
        pThis = *ppList;
        pPrev = pThis;
        while (pThis != NULL) {
            if (pThis->pEntry == pEntry) {
                if (pThis == *ppList) {
                    *ppList = pThis->pNext;
                } else {
                    pPrev->pNext = pThis->pNext;
                }
                free(pThis);
                pThis = NULL;
            } else {
                pPrev = pThis;
                pThis = pThis->pNext;
            }
        }
    }
}

// Get the first entry in a linked list.
void *pAUtilLinkedListGetFirst(aUtilLinkedList_t **ppList,
                               aUtilLinkedList_t **ppSaved)
{
    void *pEntry = NULL;

    if (ppSaved != NULL) {
        *ppSaved = NULL;
    }
    if ((ppList != NULL) && (*ppList != NULL)) {
        pEntry = (*ppList)->pEntry;
        if (ppSaved != NULL) {
            *ppSaved = (*ppList)->pNext;
        }
    }

    return pEntry;
}

// Get the next entry in a linked list.
void *pAUtilLinkedListGetNext(aUtilLinkedList_t **ppSaved)
{
    void *pEntry = NULL;

    if ((ppSaved != NULL) && (*ppSaved != NULL)) {
        pEntry = (*ppSaved)->pEntry;
        *ppSaved = (*ppSaved)->pNext;
    }

    return pEntry;
}

// End of file
