/******************************************************************************
 * File Name:   pasco2_config_task.c
 *
 * Description: This file contains the task that handles parsing the new
 *              configuration coming from remote server and setting it to the
 *              sensor-xensiv-pasco2 library.
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
#include "pasco2_config_task.h"
#include "pasco2_task.h"
#include "publisher_task.h"
#include "subscriber_task.h"

/*******************************************************************************
 * Global Variables
 ******************************************************************************/
TaskHandle_t pasco2_config_task_handle = NULL;

/*******************************************************************************
 * Function Name: json_parser_cb
 *******************************************************************************
 * Summary:
 *   Callback function that parses incoming json string.
 *
 * Parameters:
 *   json_object: incoming json object
 *   arg: callback data. Here it should be the context of xensiv_pasco2_t.
 *
 * Return:
 *   none
 ******************************************************************************/
static cy_rslt_t json_parser_cb(cy_JSON_object_t *json_object, void *arg)
{
    #define JSON_VALUE_LENGTH 32
    xensiv_pasco2_t *context = (xensiv_pasco2_t *)arg;

    bool bad_entry = false;
    char json_value[JSON_VALUE_LENGTH];

    if (json_object->value_length >= JSON_VALUE_LENGTH)
    {
        return CY_RSLT_JSON_GENERIC_ERROR;
    }
    memcpy(json_value, json_object->value, json_object->value_length);
    json_value[json_object->value_length] = '\0';

    publisher_data_t publisher_q_data;
    publisher_q_data.cmd = PUBLISH_MQTT_MSG;

    /* Supported keys and values for pasco2 configuration */
    if (memcmp(json_object->object_string, "pasco2_measurement_period", json_object->object_string_length) == 0)
    {
        const uint16_t measurement_period = atoi(json_value);
        if ((measurement_period < XENSIV_PASCO2_MEAS_RATE_MIN) || (measurement_period > XENSIV_PASCO2_MEAS_RATE_MAX))
        {
            printf("CO2 sensor measurement period configuration error, Valid range is [10-4095]\n\n");
        }
        else
        {
            xensiv_pasco2_measurement_config_t meas_config = {
                .b.op_mode = XENSIV_PASCO2_OP_MODE_IDLE,
                .b.boc_cfg = XENSIV_PASCO2_BOC_CFG_AUTOMATIC
            };
            int32_t status = xensiv_pasco2_set_measurement_config(context, meas_config);

            status |= xensiv_pasco2_set_measurement_rate(context, measurement_period);

            meas_config = (xensiv_pasco2_measurement_config_t){
                .b.op_mode = XENSIV_PASCO2_OP_MODE_CONTINUOUS,
                .b.boc_cfg = XENSIV_PASCO2_BOC_CFG_AUTOMATIC
            };
            status |= xensiv_pasco2_set_measurement_config(context, meas_config);

            if (status == CY_RSLT_SUCCESS)
            {
                pasco2_process_delay_s = measurement_period;
                snprintf(publisher_q_data.data,
                        sizeof(publisher_q_data.data),
                        "Config => %.*s: %.*s",
                        json_object->object_string_length,
                        json_object->object_string,
                        json_object->value_length,
                        json_object->value);
            }
            else
            {
                snprintf(publisher_q_data.data, sizeof(publisher_q_data.data),
                         "pasco2_measurement_period set configuration failed.");
            }
        }
    }
    else
    {
        /* Invalid input json key */
        bad_entry = true;
        snprintf(publisher_q_data.data,
                 sizeof(publisher_q_data.data),
                 "{\"%.*s\": \"invalid json key\"}",
                 json_object->object_string_length,
                 json_object->object_string);
    }

    /* Send message back to publish queue. */
    xQueueSendToBack(publisher_task_q, &publisher_q_data, 0);

    return bad_entry ? CY_RSLT_JSON_GENERIC_ERROR : CY_RSLT_SUCCESS;
}

/*******************************************************************************
 * Function Name: pasco2_config_task
 *******************************************************************************
 * Summary:
 *      Parse incoming json string, and set new configuration to
 *      sensor-xensiv-pasco2 library.
 *
 * Parameters:
 *   pvParameters: thread
 *
 * Return:
 *   none
 ******************************************************************************/
void pasco2_config_task(void *pvParameters)
{
    cy_rslt_t result;

    /* To avoid compiler warnings */
    (void)pvParameters;

    /* Register JSON parser to parse input configuration JSON string */
    cy_JSON_parser_register_callback(json_parser_cb, (void *)&xensiv_pasco2);

    while (true)
    {
        /* Block till a notification is received from the subscriber task. */
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        /* Get mutex to block any other json parse jobs */
        if (xSemaphoreTake(sem_sub_payload, portMAX_DELAY) == pdTRUE)
        {
            /* Get mutex to block mtb_radar_sensing_process in radar task */
            if (xSemaphoreTake(sem_pasco2_context, portMAX_DELAY) == pdTRUE)
            {
                printf("parse config ... \n");
                result = cy_JSON_parser(sub_msg_payload, strlen(sub_msg_payload));
                if (result != CY_RSLT_SUCCESS)
                {
                    printf("pasco2_config_task: json parser error!\n");
                }
                xSemaphoreGive(sem_pasco2_context);
            }
            xSemaphoreGive(sem_sub_payload);
        }
    }
}

/* [] END OF FILE */
