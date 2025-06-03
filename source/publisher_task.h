/******************************************************************************
 * File Name:   publisher_task.h
 *
 * Description: This file is the public interface of publisher_task.c
 *
 * Related Document: See README.md
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

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

/*******************************************************************************
 * Macros
 ******************************************************************************/
/* Task parameters for Button Task. */
#define PUBLISHER_TASK_PRIORITY   (2)
#define PUBLISHER_TASK_STACK_SIZE (1024 * 2)

#define MQTT_PUB_QUEUE_LENGTH (10u)
#define MQTT_PUB_MSG_MAX_SIZE (64u)
/*******************************************************************************
 * Typedefines
 ******************************************************************************/
/* Commands for the Publisher Task. */
typedef enum
{
    PUBLISHER_INIT,
    PUBLISHER_DEINIT,
    PUBLISH_MQTT_MSG
} publisher_cmd_t;

/* Struct to be passed via the publisher task queue */
typedef struct{
    publisher_cmd_t cmd;
    char data[MQTT_PUB_MSG_MAX_SIZE];
} publisher_data_t;

/*******************************************************************************
 * Extern Variables
 ******************************************************************************/
extern TaskHandle_t publisher_task_handle;
extern QueueHandle_t publisher_task_q;

/*******************************************************************************
 * Function Prototypes
 ******************************************************************************/
void publisher_task(void *pvParameters);

/* [] END OF FILE */
