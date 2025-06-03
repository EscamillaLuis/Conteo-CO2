/******************************************************************************
 * File Name:   subscriber_task.c
 *
 * Description: This file contains the task that subscribes to the topic
 *              'MQTT_SUB_TOPIC', and configure the sensor module based on the
 *              notifications received from the MQTT subscriber callback.
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

#include "FreeRTOS.h"
#include "cybsp.h"
#include "cyhal.h"
#include "string.h"

/* Task header files */
#include "mqtt_task.h"
#include "pasco2_config_task.h"
#include "subscriber_task.h"

/* Configuration file for MQTT client */
#include "mqtt_client_config.h"

/* Middleware libraries */
#include "cy_mqtt_api.h"
#include "cy_retarget_io.h"

/******************************************************************************
* Macros
******************************************************************************/
/* Maximum number of retries for MQTT subscribe operation */
#define MAX_SUBSCRIBE_RETRIES                   (3u)

/* Time interval in milliseconds between MQTT subscribe retries. */
#define MQTT_SUBSCRIBE_RETRY_INTERVAL_MS        (1000)

/* The number of MQTT topics to be subscribed to. */
#define SUBSCRIPTION_COUNT                      (1)


/******************************************************************************
 * Global Variables
 *******************************************************************************/
/* Task handle for this task. */
TaskHandle_t subscriber_task_handle;

/* Handle of the queue holding the commands for the subscriber task */
QueueHandle_t subscriber_task_q;

/* Configure the subscription information structure. */
cy_mqtt_subscribe_info_t subscribe_info =
{
    .qos = (cy_mqtt_qos_t) MQTT_MESSAGES_QOS,
    .topic = MQTT_PUB_TOPIC,
    .topic_len = (sizeof(MQTT_PUB_TOPIC) - 1)
};

/******************************************************************************
* Function Prototypes
*******************************************************************************/
static void subscribe_to_topic(void);
static void unsubscribe_from_topic(void);

/* Semaphore used to protect message payload */
SemaphoreHandle_t sem_sub_payload = NULL;
/* Subscribed message paylaod. Maximum with size of 512 byte. */
char sub_msg_payload[512] = {0};

/******************************************************************************
 * Function Name: subscriber_task
 ******************************************************************************
 * Summary:
 *  Task that sets up the user LED GPIO, subscribes to the specified MQTT topic,
 *  and controls the user LED based on the received commands over the message
 *  queue. The task can also unsubscribe from the topic based on the commands
 *  via the message queue.
 *
 * Parameters:
 *  void *pvParameters : Task parameter defined during task creation (unused)
 *
 * Return:
 *  void
 *
 ******************************************************************************/
void subscriber_task(void *pvParameters)
{
    subscriber_data_t subscriber_q_data;

    /* To avoid compiler warnings */
    (void) pvParameters;

    /* Initialize semaphore to protect payload */
    sem_sub_payload = xSemaphoreCreateMutex();
    if (sem_sub_payload == NULL)
    {
        printf(" 'sem_sub_payload' semaphore creation failed... Task suspend\n\n");
        vTaskSuspend(NULL);
    }

    /* Subscribe to the specified MQTT topic. */
    subscribe_to_topic();

    /* Create a message queue to communicate with other tasks and callbacks. */
    subscriber_task_q = xQueueCreate(MQTT_SUB_QUEUE_LENGTH, sizeof(subscriber_data_t));

    while (true)
    {
        /* Wait for commands from other tasks and callbacks. */
        if (pdTRUE == xQueueReceive(subscriber_task_q, &subscriber_q_data, portMAX_DELAY))
        {
            switch(subscriber_q_data.cmd)
            {
                case SUBSCRIBE_TO_TOPIC:
                {
                    subscribe_to_topic();
                    break;
                }

                case UNSUBSCRIBE_FROM_TOPIC:
                {
                    unsubscribe_from_topic();
                    break;
                }
            }
        }
    }
}

/******************************************************************************
 * Function Name: subscribe_to_topic
 ******************************************************************************
 * Summary:
 *  Function that subscribes to the MQTT topic specified by the macro
 *  'MQTT_SUB_TOPIC'. This operation is retried a maximum of
 *  'MAX_SUBSCRIBE_RETRIES' times with interval of
 *  'MQTT_SUBSCRIBE_RETRY_INTERVAL_MS' milliseconds.
 *
 * Parameters:
 *  void
 *
 * Return:
 *  void
 *
 ******************************************************************************/
static void subscribe_to_topic(void)
{
    /* Status variable */
    cy_rslt_t result = CY_RSLT_SUCCESS;

    /* Command to the MQTT client task */
    mqtt_task_cmd_t mqtt_task_cmd;

    /* Subscribe with the configured parameters. */
    for (uint32_t retry_count = 0; retry_count < MAX_SUBSCRIBE_RETRIES; retry_count++)
    {
        result = cy_mqtt_subscribe(mqtt_connection, &subscribe_info, SUBSCRIPTION_COUNT);
        if (result == CY_RSLT_SUCCESS)
        {
            printf("MQTT client subscribed to the topic '%.*s' successfully.\n\n",
                    subscribe_info.topic_len, subscribe_info.topic);
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(MQTT_SUBSCRIBE_RETRY_INTERVAL_MS));
    }

    if (result != CY_RSLT_SUCCESS)
    {
        printf("MQTT Subscribe failed with error 0x%0X after %d retries...\n\n",
               (int)result, MAX_SUBSCRIBE_RETRIES);

        /* Notify the MQTT client task about the subscription failure */
        mqtt_task_cmd = HANDLE_MQTT_SUBSCRIBE_FAILURE;
        xQueueSend(mqtt_task_q, &mqtt_task_cmd, portMAX_DELAY);
    }
}

/******************************************************************************
 * Function Name: mqtt_subscription_callback
 ******************************************************************************
 * Summary:
 *  Callback to handle incoming MQTT messages. This callback prints the
 *  contents of the incoming message and informs the subscriber task, via a
 *  message queue, to turn on / turn off the device based on the received
 *  message.
 *
 * Parameters:
 *  cy_mqtt_publish_info_t *received_msg_info : Information structure of the
 *                                              received MQTT message
 *
 * Return:
 *  void
 *
 ******************************************************************************/
void mqtt_subscription_callback(cy_mqtt_publish_info_t *received_msg_info)
{
    /* Received MQTT message */
    const char *received_msg = received_msg_info->payload;
    int received_msg_len = received_msg_info->payload_len;

    printf("  Subsciber: Incoming MQTT message received:\n"
           "    Publish topic name: %.*s\n"
           "    Publish QoS: %d\n"
           "    Publish payload: %.*s\n\n",
           received_msg_info->topic_len, received_msg_info->topic,
           (int) received_msg_info->qos,
           received_msg_len, (const char *)received_msg);

    if (received_msg_len >= sizeof(sub_msg_payload))
    {
        printf("Subscribed topic: '%.*s', received message too long. Buffer overflow.\n",
               received_msg_info->topic_len, received_msg_info->topic);
        return;
    }

    if (xSemaphoreTake(sem_sub_payload, portMAX_DELAY) == pdTRUE)
    {
        memset(sub_msg_payload, '\0', sizeof(sub_msg_payload));
        memcpy(sub_msg_payload, received_msg, received_msg_len);
        xSemaphoreGive(sem_sub_payload);

        /* Notify the pasco2 configuration task. */
        xTaskNotifyGive(pasco2_config_task_handle);
    }
}

/******************************************************************************
 * Function Name: unsubscribe_from_topic
 ******************************************************************************
 * Summary:
 *  Function that unsubscribes from the topic specified by the macro
 *  'MQTT_SUB_TOPIC'.
 *
 * Parameters:
 *  void
 *
 * Return:
 *  void
 *
 ******************************************************************************/
static void unsubscribe_from_topic(void)
{
    cy_rslt_t result = cy_mqtt_unsubscribe(mqtt_connection,
                                           (cy_mqtt_unsubscribe_info_t *) &subscribe_info,
                                           SUBSCRIPTION_COUNT);

    if (result != CY_RSLT_SUCCESS)
    {
        printf("MQTT Unsubscribe operation failed with error 0x%0X!\n", (int)result);
    }
}

/* [] END OF FILE */
