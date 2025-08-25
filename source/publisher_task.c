/******************************************************************************
 * File Name:   radar_config_task.c
 *
 * Description: This file contains the task that handles parsing the new
 *              configuration coming from remote server and setting it to the
 *              xensiv-radar-sensing library.
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

/* Header file from system */
#include "stdbool.h"
#include "stdio.h"
#include "stdlib.h"

/* Header file from library */
#include "cy_json_parser.h"

/* Header file for local tasks */
#include "publisher_task.h"
#include "radar_config_task.h"
#include "radar_task.h"
#include "subscriber_task.h"

/*******************************************************************************
 * Global Variables
 ******************************************************************************/
TaskHandle_t radar_config_task_handle = NULL;

/*******************************************************************************
 * Local Variables
 ******************************************************************************/
static char json_value[32] = {0};

/*******************************************************************************
 * Function Name: json_parser_cb
 *******************************************************************************
 * Summary:
 *   Callback function that parses incoming json string.
 *
 * Parameters:
 *      json_object: incoming json object
 *      arg: callback data. Here it should be the context of
 *           mtb_radar_sensing_context_t.
 *
 * Return:
 *   none
 ******************************************************************************/
static cy_rslt_t json_parser_cb(cy_JSON_object_t *json_object, void *arg)
{
    mtb_radar_sensing_context_t *context = (mtb_radar_sensing_context_t *)arg;

    bool bad_entry = false;
    bool not_success = false;
    /* Reset and get new value for each new json object entry */
    memset(json_value, '\0', 32);
    memcpy(json_value, json_object->value, json_object->value_length);

    publisher_data_t publisher_q_data = { .cmd = PUBLISH_MQTT_MSG, .data = {0} };

#ifdef RADAR_ENTRANCE_COUNTER_MODE
    /* Supported keys and values for entrance counter */
    if (memcmp(json_object->object_string, "radar_counter_installation", json_object->object_string_length) == 0)
    {
        if (mtb_radar_sensing_set_parameter(context, "radar_counter_installation", json_value) !=
            MTB_RADAR_SENSING_SUCCESS)
        {
            not_success = true;
        }
    }
    else if (memcmp(json_object->object_string, "radar_counter_orientation", json_object->object_string_length) == 0)
    {
        if (mtb_radar_sensing_set_parameter(context, "radar_counter_orientation", json_value) !=
            MTB_RADAR_SENSING_SUCCESS)
        {
            not_success = true;
        }
    }
    else if (memcmp(json_object->object_string, "radar_counter_ceiling_height", json_object->object_string_length) == 0)
    {
        if (mtb_radar_sensing_set_parameter(context, "radar_counter_ceiling_height", json_value) !=
            MTB_RADAR_SENSING_SUCCESS)
        {
            not_success = true;
        }
    }
    else if (memcmp(json_object->object_string, "radar_counter_entrance_width", json_object->object_string_length) == 0)
    {
        if (mtb_radar_sensing_set_parameter(context, "radar_counter_entrance_width", json_value) !=
            MTB_RADAR_SENSING_SUCCESS)
        {
            not_success = true;
        }
    }
    else if (memcmp(json_object->object_string, "radar_counter_sensitivity", json_object->object_string_length) == 0)
    {
        if (mtb_radar_sensing_set_parameter(context, "radar_counter_sensitivity", json_value) !=
            MTB_RADAR_SENSING_SUCCESS)
        {
            not_success = true;
        }
    }
    else if (memcmp(json_object->object_string,
                    "radar_counter_traffic_light_zone",
                    json_object->object_string_length) == 0)
    {
        if (mtb_radar_sensing_set_parameter(context, "radar_counter_traffic_light_zone", json_value) !=
            MTB_RADAR_SENSING_SUCCESS)
        {
            not_success = true;
        }
    }
    else if (memcmp(json_object->object_string, "radar_counter_reverse", json_object->object_string_length) == 0)
    {
        if (mtb_radar_sensing_set_parameter(context, "radar_counter_reverse", json_value) != MTB_RADAR_SENSING_SUCCESS)
        {
            not_success = true;
        }
    }
    else if (memcmp(json_object->object_string, "radar_counter_min_person_height", json_object->object_string_length) ==
             0)
    {
        if (mtb_radar_sensing_set_parameter(context, "radar_counter_min_person_height", json_value) !=
            MTB_RADAR_SENSING_SUCCESS)
        {
            not_success = true;
        }
    }
    else if (memcmp(json_object->object_string, "radar_counter_in_number", json_object->object_string_length) == 0)
    {
        entrance_count_in = atoi(json_value);
    }
    else if (memcmp(json_object->object_string, "radar_counter_out_number", json_object->object_string_length) == 0)
    {
        entrance_count_out = atoi(json_value);
    }
#else
    /* Supported keys and values for presence detection */
    if (memcmp(json_object->object_string, "radar_presence_range_max", json_object->object_string_length) == 0)
    {
        if (mtb_radar_sensing_set_parameter(context, "radar_presence_range_max", json_value) !=
            MTB_RADAR_SENSING_SUCCESS)
        {
            not_success = true;
        }
    }
    else if (memcmp(json_object->object_string, "radar_presence_sensitivity", json_object->object_string_length) == 0)
    {
        if (mtb_radar_sensing_set_parameter(context, "radar_presence_sensitivity", json_value) !=
            MTB_RADAR_SENSING_SUCCESS)
        {
            not_success = true;
        }
    }
#endif
    else
    {
        /* Invalid input json key */
        bad_entry = true;
    }

    // Keep in mind: local_pub_msg has a size of MQTT_PUB_DATA_MAX_SIZE.
    if (bad_entry)
    {
    	snprintf(publisher_q_data.data,
    	         sizeof(publisher_q_data.data),
                 "\"%.*s\": invalid entry key.",
                 json_object->object_string_length,
                 json_object->object_string);
    }
    else if (not_success)
    {
    	snprintf(publisher_q_data.data,
    	         sizeof(publisher_q_data.data),
                 "%.*s: configuration failed.",
                 json_object->object_string_length,
                 json_object->object_string);
    }
    else
    {
    	snprintf(publisher_q_data.data,
    	         sizeof(publisher_q_data.data),
                 "Config => %.*s: %.*s",
                 json_object->object_string_length,
                 json_object->object_string,
                 json_object->value_length,
                 json_object->value);
    }

    /* Send message back to publish queue. If queue is full, message will be dropped. */
    xQueueSendToBack(publisher_task_q, &publisher_q_data, 0);

    return bad_entry ? CY_RSLT_JSON_GENERIC_ERROR : CY_RSLT_SUCCESS;
}

/*******************************************************************************
 * Function Name: radar_config_task
 *******************************************************************************
 * Summary:
 *      Parse incoming json string, and set new configuration to
 *      xensiv-radar-sensing library.
 *
 * Parameters:
 *   pvParameters: thread
 *
 * Return:
 *   none
 ******************************************************************************/
void radar_config_task(void *pvParameters)
{
    cy_rslt_t result;

    /* To avoid compiler warnings */
    (void)pvParameters;

    /* Register JSON parser to parse input configuration JSON string */
    cy_JSON_parser_register_callback(json_parser_cb, (void *)&radar_sensing_context);

    while (true)
    {
        /* Block till a notification is received from the subscriber task. */
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        /* Get mutex to block any other json parse jobs */
        if (xSemaphoreTake(sem_sub_payload, portMAX_DELAY) == pdTRUE)
        {
            /* Get mutex to block mtb_radar_sensing_process in radar task */
            if (xSemaphoreTake(sem_radar_sensing_context, portMAX_DELAY) == pdTRUE)
            {
                result = cy_JSON_parser(sub_msg_payload, strlen(sub_msg_payload));
                if (result != CY_RSLT_SUCCESS)
                {
                    printf("radar_config_task: json parser error!\n");
                }
                xSemaphoreGive(sem_radar_sensing_context);
            }
            xSemaphoreGive(sem_sub_payload);
        }
    }
}

/* [] END OF FILE */
