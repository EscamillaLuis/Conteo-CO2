/******************************************************************************
 * File Name:   mqtt_task.h
 *
 * Description: This file is the public interface of mqtt_task.c
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
#include "cy_mqtt_api.h"


/*******************************************************************************
* Macros
*******************************************************************************/
/* Task parameters for MQTT Client Task. */
#define MQTT_CLIENT_TASK_PRIORITY       (2)
#define MQTT_CLIENT_TASK_STACK_SIZE     (1024 * 2)

/*******************************************************************************
* Global Variables
*******************************************************************************/
/* Commands for the MQTT Client Task. */
typedef enum
{
    HANDLE_MQTT_SUBSCRIBE_FAILURE,
    HANDLE_MQTT_PUBLISH_FAILURE,
    HANDLE_DISCONNECTION
} mqtt_task_cmd_t;

/*******************************************************************************
 * Extern variables
 ******************************************************************************/
extern cy_mqtt_t mqtt_connection;
extern QueueHandle_t mqtt_task_q;

/*******************************************************************************
* Function Prototypes
*******************************************************************************/
void mqtt_client_task(void *pvParameters);

/* [] END OF FILE */
