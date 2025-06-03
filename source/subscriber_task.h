/******************************************************************************
 * File Name:   subscriber_task.h
 *
 * Description: This file is the public interface of subscriber_task.c
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
#include "task.h"
#include "semphr.h"
#include "queue.h"
#include "cy_mqtt_api.h"

/*******************************************************************************
* Macros
*******************************************************************************/
/* Task parameters for Subscriber Task. */
#define SUBSCRIBER_TASK_PRIORITY           (2)
#define SUBSCRIBER_TASK_STACK_SIZE         (1024 * 2)

#define MQTT_SUB_QUEUE_LENGTH              (1u)
#define MQTT_SUB_MSG_MAX_SIZE              (512u)

/*******************************************************************************
* Global Variables
********************************************************************************/
/* Commands for the Subscriber Task. */
typedef enum
{
    SUBSCRIBE_TO_TOPIC,
    UNSUBSCRIBE_FROM_TOPIC,
} subscriber_cmd_t;

/* Struct to be passed via the subscriber task queue */
typedef struct{
    subscriber_cmd_t cmd;
} subscriber_data_t;

/*******************************************************************************
* Extern Variables
*******************************************************************************/
extern TaskHandle_t subscriber_task_handle;
extern SemaphoreHandle_t sem_sub_payload;
extern char sub_msg_payload[512];
extern QueueHandle_t subscriber_task_q;

/*******************************************************************************
* Function Prototypes
*******************************************************************************/
void subscriber_task(void *pvParameters);
void mqtt_subscription_callback(cy_mqtt_publish_info_t *received_msg_info);

/* [] END OF FILE */
