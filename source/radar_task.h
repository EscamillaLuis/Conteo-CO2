#pragma once

/* Header file includes */
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

/* Header file for library */
#include "mtb_radar_sensing.h"

/*******************************************************************************
 * Macros
 ******************************************************************************/
#define RADAR_TASK_NAME       "RADAR PRESENCE TASK"
#define RADAR_TASK_STACK_SIZE (1024 * 4)
#define RADAR_TASK_PRIORITY   (3)

/**
 * Compile time switch to determine which function mode the radar module is
 * working on. By default undefine the following macro, radar module works in
 * 'PresenceDetection' mode. Define the following macro, it works in
 * 'EntranceCounter' mode.
 */
#undef RADAR_ENTRANCE_COUNTER_MODE

/*******************************************************************************
 * Global Variables
 ******************************************************************************/
extern TaskHandle_t radar_task_handle;

extern SemaphoreHandle_t sem_radar_sensing_context;
extern mtb_radar_sensing_context_t radar_sensing_context;

extern int32_t entrance_count_in;
extern int32_t entrance_count_out;

/*******************************************************************************
 * Functions
 ******************************************************************************/
void radar_task(void *pvParameters);
void radar_task_cleanup(void);

/* [] END OF FILE */
