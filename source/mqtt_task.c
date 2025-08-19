#include "cybsp.h"
#include "cyhal.h"

/* FreeRTOS header files */
#include "FreeRTOS.h"
#include "task.h"

/* Task header files */
#include "mqtt_task.h"
#include "pasco2_task.h"
#include "publisher_task.h"
#include "subscriber_task.h"
#include "radar_task.h"
#include "Obtener_fecha.h"


/* Configuration file for Wi-Fi and MQTT client */
#include "wifi_config.h"
#include "mqtt_client_config.h"

/* Middleware libraries */
#include "cy_retarget_io.h"
#include "cy_wcm.h"
#include "cy_lwip.h"

#include "cy_mqtt_api.h"
#include "clock.h"

/* LwIP header files */
#include "lwip/netif.h"
#include "lwip/ip6_addr.h"
#include "lwip/ip4_addr.h"
#include "lwip/apps/sntp.h"
#include "lwip/ip_addr.h"

/*******************************************************************************
 * Macros
 ******************************************************************************/
#define WIFI_MAX_RETRY            		 (MAX_WIFI_CONN_RETRIES)
#define WIFI_RETRY_INTERVAL_MS     		 (WIFI_CONN_RETRY_INTERVAL_MS)
#define MQTT_TASK_QUEUE_LENGTH           (3u)
#define TASK_CREATION_DELAY_MS           (2000u)
#define WCM_INITIALIZED                  (1lu << 0)
#define WIFI_CONNECTED                   (1lu << 1)
#define LIBS_INITIALIZED                 (1lu << 2)
#define BUFFER_INITIALIZED               (1lu << 3)
#define MQTT_INSTANCE_CREATED            (1lu << 4)
#define MQTT_CONNECTION_SUCCESS          (1lu << 5)
#define MQTT_MSG_RECEIVED                (1lu << 6)
#define RADAR_ENTRANCE_COUNTER_MODE


#define CHECK_RESULT(result, init_mask, error_message...)      \
                     do                                        \
                     {                                         \
                         if ((int)result == CY_RSLT_SUCCESS)   \
                         {                                     \
                             status_flag |= init_mask;         \
                         }                                     \
                         else                                  \
                         {                                     \
                             printf(error_message);            \
                             return result;                    \
                         }                                     \
                     } while(0)

/*******************************************************************************
 * Global Variables
 ******************************************************************************/
cy_mqtt_t mqtt_connection = NULL;
QueueHandle_t mqtt_task_q = NULL;
uint32_t status_flag = 0;
uint8_t *mqtt_network_buffer = NULL;

/*******************************************************************************
* Prototipos de Funciones Estáticas
*******************************************************************************/
static cy_rslt_t wifi_connect(void);
static cy_rslt_t mqtt_init(void);
static cy_rslt_t mqtt_connect(void);
static void mqtt_event_callback(cy_mqtt_t mqtt_handle, cy_mqtt_event_t event, void *user_data);
static void cleanup(void);

#if GENERATE_UNIQUE_CLIENT_ID
static cy_rslt_t mqtt_get_unique_client_identifier(char *mqtt_client_identifier);
#endif /* GENERATE_UNIQUE_CLIENT_ID */

/*******************************************************************************
 * Function Name: mqtt_client_task
 ******************************************************************************/
void mqtt_client_task(void *pvParameters)
{
    mqtt_task_cmd_t mqtt_status;
    subscriber_data_t subscriber_q_data;
    publisher_data_t publisher_q_data;

    cy_wcm_config_t config = {.interface = CY_WCM_INTERFACE_TYPE_STA};

    (void) pvParameters;

    mqtt_task_q = xQueueCreate(MQTT_TASK_QUEUE_LENGTH, sizeof(mqtt_task_cmd_t));

    if (CY_RSLT_SUCCESS != cy_wcm_init(&config))
    {
        printf("\nWi-Fi Connection Manager initialization failed!\n");
        goto exit_cleanup;
    }

    status_flag |= WCM_INITIALIZED;
    printf("\nWi-Fi Connection Manager initialized.\n");

    if (CY_RSLT_SUCCESS != wifi_connect())
    {
        goto exit_cleanup;
    }
    // Inicializa el cliente NTP y crea la tarea que reinicia contadores a diario
    init_sntp_client();

    if (pdPASS != xTaskCreate(tarea_verificar_fecha, "Tarea Fecha", 1024, NULL, 1, NULL))
    {
        printf("No se pudo crear la tarea de verificación de fecha.\n");
        goto exit_cleanup;
    }


    if ((CY_RSLT_SUCCESS != mqtt_init()) || (CY_RSLT_SUCCESS != mqtt_connect()))
    {
        goto exit_cleanup;
    }

    if (pdPASS != xTaskCreate(subscriber_task, "Subscriber task", SUBSCRIBER_TASK_STACK_SIZE,
                              NULL, SUBSCRIBER_TASK_PRIORITY, &subscriber_task_handle))
    {
        printf("Failed to create the Subscriber task!\n");
        goto exit_cleanup;
    }

    vTaskDelay(pdMS_TO_TICKS(TASK_CREATION_DELAY_MS));

    if (pdPASS != xTaskCreate(publisher_task, "Publisher task", PUBLISHER_TASK_STACK_SIZE,
                              NULL, PUBLISHER_TASK_PRIORITY, &publisher_task_handle))
    {
        printf("Failed to create Publisher task!\n");
        goto exit_cleanup;
    }

    if (pdPASS != xTaskCreate(pasco2_task, PASCO2_TASK_NAME, PASCO2_TASK_STACK_SIZE,
                              NULL, PASCO2_TASK_PRIORITY, &pasco2_task_handle))
    {
        printf("Failed to create '%s' task!\n", PASCO2_TASK_NAME);
        goto exit_cleanup;
    }

    if (pdPASS != xTaskCreate(radar_task, RADAR_TASK_NAME, RADAR_TASK_STACK_SIZE,
                              NULL, RADAR_TASK_PRIORITY, &radar_task_handle))
    {
        printf("Failed to create '%s' task!\n", RADAR_TASK_NAME);
        goto exit_cleanup;
    }

    while (true)
    {
        if (pdTRUE == xQueueReceive(mqtt_task_q, &mqtt_status, portMAX_DELAY))
        {
            switch(mqtt_status)
            {
                case HANDLE_MQTT_PUBLISH_FAILURE:
                    break;
                case HANDLE_MQTT_SUBSCRIBE_FAILURE:
                    break;
                case HANDLE_DISCONNECTION:
                    publisher_q_data.cmd = PUBLISHER_DEINIT;
                    xQueueSend(publisher_task_q, &publisher_q_data, portMAX_DELAY);

                    cy_mqtt_disconnect(mqtt_connection);

                    if (cy_wcm_is_connected_to_ap() == 0)
                    {
                        status_flag &= ~(WIFI_CONNECTED);
                        printf("Initiating Wi-Fi Reconnection...\n");
                        if (CY_RSLT_SUCCESS != wifi_connect())
                        {
                            goto exit_cleanup;
                        }
                    }

                    printf("Initiating MQTT Reconnection...\n");
                    if (CY_RSLT_SUCCESS != mqtt_connect())
                    {
                        goto exit_cleanup;
                    }

                    subscriber_q_data.cmd = SUBSCRIBE_TO_TOPIC;
                    xQueueSend(subscriber_task_q, &subscriber_q_data, portMAX_DELAY);

                    publisher_q_data.cmd = PUBLISHER_INIT;
                    xQueueSend(publisher_task_q, &publisher_q_data, portMAX_DELAY);
                    break;

                default:
                    break;
            }
        }
    }

    exit_cleanup:
    printf("\nTerminating Publisher, Subscriber, and Sensor tasks...\n");
    if (subscriber_task_handle != NULL)
    {
        vTaskDelete(subscriber_task_handle);
    }
    if (publisher_task_handle != NULL)
    {
        vTaskDelete(publisher_task_handle);
    }
    if (pasco2_task_handle != NULL)
    {
        pasco2_task_cleanup();
        vTaskDelete(pasco2_task_handle);
    }
    if (radar_task_handle != NULL)
    {
        radar_task_cleanup();
        vTaskDelete(radar_task_handle);
    }
    cleanup();
    printf("\nCleanup Done\nTerminating the MQTT task...\n\n");
    vTaskDelete(NULL);
}

/******************************************************************************
 * Wi-Fi and MQTT connection helper functions (no changes)
 ******************************************************************************/
static cy_rslt_t wifi_connect(void)
{
    cy_rslt_t result = CY_RSLT_SUCCESS;
    cy_wcm_connect_params_t connect_param;
    cy_wcm_ip_address_t ip_addr;

    if (cy_wcm_is_connected_to_ap() == 0)
    {
        memset(&connect_param, 0, sizeof(connect_param));
        memcpy(connect_param.ap_credentials.SSID, WIFI_SSID, sizeof(WIFI_SSID));
        memcpy(connect_param.ap_credentials.password, WIFI_PASSWORD, sizeof(WIFI_PASSWORD));
        connect_param.ap_credentials.security = WIFI_SECURITY;

        printf("Conectando a Wi-Fi AP '%s'...\n", connect_param.ap_credentials.SSID);

        for (uint32_t retry = 0;; retry++)
        {
            result = cy_wcm_connect_ap(&connect_param, &ip_addr);
            if (result == CY_RSLT_SUCCESS)
            {
                printf("Wi-Fi conectado.\n");
                status_flag |= WIFI_CONNECTED;

                if (ip_addr.version == CY_WCM_IP_VER_V4)
                {
                    printf("IPv4: %s\n", ip4addr_ntoa((const ip4_addr_t *)&ip_addr.ip.v4));
                }
                else
                {
                    printf("IPv6: %s\n", ip6addr_ntoa((const ip6_addr_t *)&ip_addr.ip.v6));
                }
                return result;
            }
            printf("Falla Wi-Fi (0x%08X). Reintento en %d ms, intento #%u.\n",
                   (unsigned int)result,
                   WIFI_RETRY_INTERVAL_MS,
                   (unsigned int)(retry + 1));
            vTaskDelay(pdMS_TO_TICKS(WIFI_RETRY_INTERVAL_MS));
        }
    }
    return result;
}

/*******************************************************************************
* Función: mqtt_init
*******************************************************************************/
static cy_rslt_t mqtt_init(void)
{
    cy_rslt_t result = cy_mqtt_init();
    if (result == CY_RSLT_SUCCESS)
    {
        status_flag |= LIBS_INITIALIZED;
    }
    else
    {
        printf("Error: cy_mqtt_init.\n");
        return result;
    }

    mqtt_network_buffer = (uint8_t *)pvPortMalloc(MQTT_NETWORK_BUFFER_SIZE);
    if (mqtt_network_buffer == NULL)
    {
        printf("Fallo al reservar buffer MQTT.\n");
        return ~CY_RSLT_SUCCESS;
    }
    status_flag |= BUFFER_INITIALIZED;

    /* Crear instancia MQTT */
    result = cy_mqtt_create(
                 mqtt_network_buffer,
                 MQTT_NETWORK_BUFFER_SIZE,
                 security_info,
                 &broker_info,
                 mqtt_event_callback,
                 NULL,
                 &mqtt_connection
             );
    if (result == CY_RSLT_SUCCESS)
    {
        status_flag |= MQTT_INSTANCE_CREATED;
        printf("Instancia MQTT creada.\n");
    }
    else
    {
        printf("Error al crear instancia MQTT (0x%08X).\n", (unsigned int)result);
    }

    return result;
}

/*******************************************************************************
* Función: mqtt_connect
*******************************************************************************/
static cy_rslt_t mqtt_connect(void)
{
    cy_rslt_t result = CY_RSLT_SUCCESS;
    char mqtt_client_identifier[MQTT_CLIENT_IDENTIFIER_MAX_LEN + 1] = MQTT_CLIENT_IDENTIFIER;

#if GENERATE_UNIQUE_CLIENT_ID
    result = mqtt_get_unique_client_identifier(mqtt_client_identifier);
    if (result != CY_RSLT_SUCCESS)
    {
        printf("Error generando client ID único.\n");
        return result;
    }
#endif

    if (strlen(MQTT_USERNAME) > 0)
    {
        connection_info.username = MQTT_USERNAME;
        connection_info.password = MQTT_PASSWORD;
        connection_info.username_len = (uint16_t)strlen(MQTT_USERNAME);
        connection_info.password_len = (uint16_t)strlen(MQTT_PASSWORD);
    }

    connection_info.client_id = mqtt_client_identifier;
    connection_info.client_id_len = (uint16_t)strlen(mqtt_client_identifier);

    printf("Conectando MQTT con ID='%.*s' a '%.*s'...\n",
           connection_info.client_id_len,
           connection_info.client_id,
           broker_info.hostname_len,
           broker_info.hostname);

    for (uint32_t retry = 0; retry < MAX_MQTT_CONN_RETRIES; retry++)
    {
        if (cy_wcm_is_connected_to_ap() == 0)
        {
            printf("Wi-Fi desconectado, reintentando...\n");
            status_flag &= ~WIFI_CONNECTED;
            result = wifi_connect();
            if (result != CY_RSLT_SUCCESS)
            {
                return result;
            }
        }

        result = cy_mqtt_connect(mqtt_connection, &connection_info);
        if (result == CY_RSLT_SUCCESS)
        {
            printf("MQTT conectado correctamente.\n");
            status_flag |= MQTT_CONNECTION_SUCCESS;
            return result;
        }
        printf("Falla al conectar MQTT (0x%08X). Reintento en %u ms, quedan %u.\n",
               (unsigned int)result,
               MQTT_CONN_RETRY_INTERVAL_MS,
               (unsigned int)(MAX_MQTT_CONN_RETRIES - retry - 1));
        vTaskDelay(pdMS_TO_TICKS(MQTT_CONN_RETRY_INTERVAL_MS));
    }

    printf("No se pudo conectar a MQTT tras %u reintentos.\n", (unsigned)MAX_MQTT_CONN_RETRIES);
    return result;
}
/*******************************************************************************
* Función: mqtt_event_callback
*******************************************************************************/
static void mqtt_event_callback(cy_mqtt_t mqtt_handle,
                                cy_mqtt_event_t event,
                                void *user_data)
{
    (void) mqtt_handle;
    (void) user_data;

    mqtt_task_cmd_t mqtt_task_cmd;

    switch (event.type)
    {
        case CY_MQTT_EVENT_TYPE_DISCONNECT:
        {
            status_flag &= ~MQTT_CONNECTION_SUCCESS;
            printf("MQTT desconectado inesperadamente.\n");

            mqtt_task_cmd = HANDLE_DISCONNECTION;
            xQueueSend(mqtt_task_q, &mqtt_task_cmd, portMAX_DELAY);
            break;
        }
        case CY_MQTT_EVENT_TYPE_SUBSCRIPTION_MESSAGE_RECEIVE:
        {
            /* Delegamos el manejo al subscriber_task */
            cy_mqtt_publish_info_t *rx_msg = &(event.data.pub_msg.received_message);
            mqtt_subscription_callback(rx_msg);
            break;
        }
        default:
            printf("Evento MQTT desconocido (type=%d)\n", (int)event.type);
            break;
    }
}

#if GENERATE_UNIQUE_CLIENT_ID
/*******************************************************************************
* Función: mqtt_get_unique_client_identifier
*******************************************************************************/
static cy_rslt_t mqtt_get_unique_client_identifier(char *mqtt_client_identifier)
{
    if (0 > snprintf(mqtt_client_identifier,
                     (MQTT_CLIENT_IDENTIFIER_MAX_LEN + 1),
                     MQTT_CLIENT_IDENTIFIER "%lu",
                     (unsigned long)xTaskGetTickCount()))
    {
        return ~CY_RSLT_SUCCESS;
    }
    return CY_RSLT_SUCCESS;
}
#endif

/*******************************************************************************
* Función: cleanup
*******************************************************************************/
static void cleanup(void)
{
    if (status_flag & MQTT_CONNECTION_SUCCESS)
    {
        cy_mqtt_disconnect(mqtt_connection);
    }
    if (status_flag & MQTT_INSTANCE_CREATED)
    {
        cy_mqtt_delete(mqtt_connection);
        mqtt_connection = NULL;
    }
    if (status_flag & BUFFER_INITIALIZED)
    {
        vPortFree(mqtt_network_buffer);
    }
    if (status_flag & LIBS_INITIALIZED)
    {
        cy_mqtt_deinit();
    }
    if (status_flag & WIFI_CONNECTED)
    {
        cy_wcm_disconnect_ap();
        printf("Wi-Fi desconectado.\n");
    }
    if (status_flag & WCM_INITIALIZED)
    {
        cy_wcm_deinit();
        printf("WCM desinicializado.\n");
    }
}
