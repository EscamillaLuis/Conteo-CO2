#include <inttypes.h>

/* Header file includes */
#include "cy_retarget_io.h"
#include "cycfg.h"
#include "cyhal.h"

/* Header file for local task */
#include "publisher_task.h"
#include "radar_config_task.h"
#include "radar_led_task.h"
#include "radar_task.h"

/*******************************************************************************
 * Macros
 ******************************************************************************/
/* Pin number designated for LED RED */
#define LED_RGB_RED (CYBSP_GPIOA0)
/* Pin number designated for LED GREEN */
#define LED_RGB_GREEN (CYBSP_GPIOA1)
/* Pin number designated for LED BLUE */
#define LED_RGB_BLUE (CYBSP_GPIOA2)
/* LED off */
#define LED_STATE_OFF (0U)
/* LED on */
#define LED_STATE_ON (1U)
/* RADAR sensor SPI frequency */
#define SPI_FREQUENCY (25000000UL)

#define RADAR_ENTRANCE_COUNTER_MODE

/*******************************************************************************
 * Global Variables
 ******************************************************************************/
TaskHandle_t radar_task_handle = NULL;

/* Semaphore to protect radar sensing context */
SemaphoreHandle_t sem_radar_sensing_context = NULL;
/* Radar sensing context */
mtb_radar_sensing_context_t radar_sensing_context = {0};

/* Entrance counter In & Out number. Can be reset from remote server. */
int32_t entrance_count_in = 0;
int32_t entrance_count_out = 0;

/*******************************************************************************
 * Local Variables
 ******************************************************************************/
static char local_pub_msg[MQTT_PUB_MSG_MAX_SIZE] = {0};
static int32_t occupy_status = 0;

/*******************************************************************************
 * Function Name: radar_sensing_callback
 ******************************************************************************/
static void radar_sensing_callback(mtb_radar_sensing_context_t *context,
                                   mtb_radar_sensing_event_t event,
                                   mtb_radar_sensing_event_info_t *event_info,
                                   void *data)
{
    static int32_t last_entrance_count_in = 0;
    static int32_t last_entrance_count_out = 0;

    (void)context;
    (void)data;

    radar_led_set_pattern(event);

    switch (event)
    {
#ifdef RADAR_ENTRANCE_COUNTER_MODE
        case MTB_RADAR_SENSING_EVENT_COUNTER_IN:
            ++entrance_count_in;
            break;
        case MTB_RADAR_SENSING_EVENT_COUNTER_OUT:
            ++entrance_count_out;
            break;
        case MTB_RADAR_SENSING_EVENT_COUNTER_OCCUPIED:
            occupy_status = 1;
            break;
        case MTB_RADAR_SENSING_EVENT_COUNTER_FREE:
            occupy_status = 0;
            break;
#endif
        default:
            printf("Unknown event. Error!\n");
            snprintf(local_pub_msg, sizeof(local_pub_msg), "{\"Radar Module\": \"Unknown event. Error!\"}");
            return;
    }

#ifdef RADAR_ENTRANCE_COUNTER_MODE
    if (entrance_count_in != last_entrance_count_in ||
    	entrance_count_out != last_entrance_count_out)
    {
        snprintf(local_pub_msg,
                 sizeof(local_pub_msg),
                 "{\"IN_Count\": %ld, \"OUT_Count\": %ld}",
                 entrance_count_in, entrance_count_out);

        publisher_data_t radar_data = {.cmd = PUBLISH_MQTT_MSG};
        strncpy(radar_data.data, local_pub_msg, sizeof(radar_data.data) - 1);
        if (xQueueSendToBack(publisher_task_q, &radar_data, 0) != pdPASS)
        {
            printf("Error: No se pudo enviar el mensaje de conteo a la cola del publicador\n");
        }

        last_entrance_count_in = entrance_count_in;
        last_entrance_count_out = entrance_count_out;
    }
#endif
}

/*******************************************************************************
 * Function Name: ifx_currenttime
 ******************************************************************************/
static uint64_t ifx_currenttime()
{
    return (uint64_t)xTaskGetTickCount() * portTICK_PERIOD_MS;
}

/*******************************************************************************
 * Function Name: radar_task
 ******************************************************************************/
void radar_task(void *pvParameters)
{
    (void)pvParameters;

    cyhal_spi_t mSPI;

    mtb_radar_sensing_hw_cfg_t hw_cfg = {.spi_cs = CYBSP_SPI_CS,
                                         .reset = CYBSP_GPIO11,
                                         .ldo_en = CYBSP_GPIO5,
                                         .irq = CYBSP_GPIO10,
                                         .spi = &mSPI};


    cyhal_gpio_init(hw_cfg.reset, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, true); // @suppress("Field cannot be resolved")
    cyhal_gpio_init(hw_cfg.ldo_en, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, true); // @suppress("Field cannot be resolved")
    cyhal_gpio_init(hw_cfg.irq, CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_PULLDOWN, false); // @suppress("Field cannot be resolved")
    cyhal_gpio_init(hw_cfg.spi_cs, CYHAL_GPIO_DIR_OUTPUT, CYHAL_GPIO_DRIVE_STRONG, true); // @suppress("Field cannot be resolved")

    if (cyhal_spi_init(hw_cfg.spi, // @suppress("Field cannot be resolved")
                       CYBSP_SPI_MOSI,
                       CYBSP_SPI_MISO,
                       CYBSP_SPI_CLK,
                       NC,
                       NULL,
                       8,
                       CYHAL_SPI_MODE_00_MSB,
                       false) != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    if (cyhal_spi_set_frequency(hw_cfg.spi, SPI_FREQUENCY) != CY_RSLT_SUCCESS) // @suppress("Field cannot be resolved")
    {
        CY_ASSERT(0);
    }

#ifdef RADAR_ENTRANCE_COUNTER_MODE
    if (mtb_radar_sensing_init(&radar_sensing_context, &hw_cfg, MTB_RADAR_SENSING_MASK_COUNTER_EVENTS) != MTB_RADAR_SENSING_SUCCESS)
    {
        printf("**** Error en mtb_radar_sensing_init: ¿Radar no conectado? ****\n");
        vTaskSuspend(NULL);
    }

    if (mtb_radar_sensing_register_callback(&radar_sensing_context, radar_sensing_callback, NULL) != MTB_RADAR_SENSING_SUCCESS)
    {
        CY_ASSERT(0);
    }

    mtb_radar_sensing_set_parameter(&radar_sensing_context, "radar_counter_installation", "ceiling"); // "ceiling", "side"
    mtb_radar_sensing_set_parameter(&radar_sensing_context, "radar_counter_orientation", "landscape"); //"landscape", "portrait"
    mtb_radar_sensing_set_parameter(&radar_sensing_context, "radar_counter_ceiling_height", "2.3");
    mtb_radar_sensing_set_parameter(&radar_sensing_context, "radar_counter_entrance_width", "1.0");
    mtb_radar_sensing_set_parameter(&radar_sensing_context, "radar_counter_sensitivity", "0.4");
    mtb_radar_sensing_set_parameter(&radar_sensing_context, "radar_counter_traffic_light_zone", "0.0");
    mtb_radar_sensing_set_parameter(&radar_sensing_context, "radar_counter_reverse", "false");
    mtb_radar_sensing_set_parameter(&radar_sensing_context, "radar_counter_min_person_height", "1.2");
#endif

    if (mtb_radar_sensing_enable(&radar_sensing_context) != MTB_RADAR_SENSING_SUCCESS)
    {
        CY_ASSERT(0);
    }

    sem_radar_sensing_context = xSemaphoreCreateMutex();
    if (sem_radar_sensing_context == NULL)
    {
        printf("Error: No se pudo crear el semáforo 'sem_radar_sensing_context'\n");
        vTaskSuspend(NULL);
    }

    xTaskCreate(radar_config_task, RADAR_CONFIG_TASK_NAME, RADAR_CONFIG_TASK_STACK_SIZE, NULL, RADAR_CONFIG_TASK_PRIORITY, &radar_config_task_handle);
    xTaskCreate(radar_led_task, RADAR_LED_TASK_NAME, RADAR_LED_TASK_STACK_SIZE, NULL, RADAR_LED_TASK_PRIORITY, &radar_led_task_handle);


    for (;;)
    {
        if (xSemaphoreTake(sem_radar_sensing_context, portMAX_DELAY) == pdTRUE)
        {
            if (mtb_radar_sensing_process(&radar_sensing_context, ifx_currenttime()) != MTB_RADAR_SENSING_SUCCESS)
            {
                printf("Error en ifx_radar_sensing_process\n");
                CY_ASSERT(0);
            }
            xSemaphoreGive(sem_radar_sensing_context);


            vTaskDelay(MTB_RADAR_SENSING_PROCESS_DELAY);
        }
    }
}

void reset_contadores(void)
{
    entrance_count_in = 0;
    entrance_count_out = 0;
    occupy_status = 0;

    printf("Contadores reiniciados automaticamente.\n");

    snprintf(local_pub_msg,
             sizeof(local_pub_msg),
             "{\"IN_Count\": %ld, \"OUT_Count\": %ld}",
             entrance_count_in, entrance_count_out);

    if (publisher_task_q != NULL)
        {
            publisher_data_t radar_data = {.cmd = PUBLISH_MQTT_MSG};
            strncpy(radar_data.data, local_pub_msg, sizeof(radar_data.data) - 1);
            if (xQueueSendToBack(publisher_task_q, &radar_data, 0) != pdPASS)
            {
                printf("Error: No se pudo enviar el mensaje de reinicio a la cola del publicador\n");
            }
        }
    else
    {
    	printf("Advertencia: cola del publicador no inicializada; se omite publicación de reinicio\n");
    }
}


/*******************************************************************************
 * Function Name: radar_task_cleanup
 ******************************************************************************/
void radar_task_cleanup(void)
{
    if (radar_config_task_handle != NULL)
    {
        vTaskDelete(radar_config_task_handle);
    }
    if (radar_led_task_handle != NULL)
    {
        vTaskDelete(radar_led_task_handle);
    }
}

/* [] END OF FILE */
