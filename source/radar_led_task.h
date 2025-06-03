/******************************************************************************
 * File name: radar_led_task.h
 *
 * Description: This file contains the function prototypes and constants used
 *   in radar_led_task.c.
 *
 * ===========================================================================
 * Copyright (C) 2021 Infineon Technologies AG. All rights reserved.
 * ===========================================================================
 *
 * ===========================================================================
 * Infineon Technologies AG (INFINEON) is supplying this file for use
 * exclusively with Infineon's sensor products. This file can be freely
 * distributed within development tools and software supporting such
 * products.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 * OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 * INFINEON SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES, FOR ANY REASON
 * WHATSOEVER.
 * ===========================================================================
 */

#pragma once

/* Header file includes */
#include "cyabs_rtos.h"
#include "cycfg.h"

/* Header file for local task */
#include "radar_task.h"

/* Header file for library */
#include "mtb_radar_sensing.h"

/*******************************************************************************
 * Macros
 ******************************************************************************/
#define RADAR_LED_TASK_NAME       "RADAR LED TASK"
#define RADAR_LED_TASK_STACK_SIZE (512)
#define RADAR_LED_TASK_PRIORITY   (2)

extern TaskHandle_t radar_led_task_handle;
/*******************************************************************************
 * Functions
 ******************************************************************************/
void radar_led_task(void *pvParameters);
void radar_led_set_pattern(mtb_radar_sensing_event_t event);

/* [] END OF FILE */
