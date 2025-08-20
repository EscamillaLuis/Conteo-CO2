#include "Obtener_fecha.h"
#include "lwip/apps/sntp.h"
#include <time.h>
#include <stdio.h>
#include "FreeRTOS.h"
#include "task.h"
#include <stdlib.h>

extern void reset_contadores(void);

// Variables para manejo de tiempo NTP
static time_t tiempo_base_ntp = 0;
static uint32_t tick_base_ntp = 0;

#define HORA_RESET     23
#define MINUTO_RESET   59

static int ultimo_reset_dia = -1;

void init_sntp_client(void)
{
    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_setservername(0, "pool.ntp.org");
    sntp_init();
    printf("Cliente SNTP inicializado\n");
}

void set_time_from_sntp(uint32_t sec)
{
    setenv("TZ", "CST6CDT,M4.1.0,M10.5.0", 1);  // Zona horaria CDMX con horario de verano
    tzset();

    tiempo_base_ntp = (time_t)sec;
    tick_base_ntp = xTaskGetTickCount();

    struct tm* hora_local = localtime(&tiempo_base_ntp);
    printf("Tiempo sincronizado (CDMX):  %s", asctime(hora_local));
}

/**
 * @brief Tarea que revisa si se alcanzó la hora de reinicio especificada.
 */
void tarea_verificar_fecha(void *arg)
{
    while (1)
    {
        if (tiempo_base_ntp == 0)
        {
            printf("Esperando sincronización NTP...\n");
            vTaskDelay(pdMS_TO_TICKS(5000));
            continue;
        }

        uint32_t ticks_transcurridos = xTaskGetTickCount() - tick_base_ntp;
        time_t tiempo_actual = tiempo_base_ntp + (ticks_transcurridos / 1000);
        struct tm* ahora_tm = localtime(&tiempo_actual);

        if (ahora_tm != NULL)
        {
            printf("Hora actual (CDMX): %02d:%02d:%02d | Día: %d\n",
                   ahora_tm->tm_hour, ahora_tm->tm_min, ahora_tm->tm_sec,
                   ahora_tm->tm_mday);

            // Solo reinicia si: el día cambió Y ya es 1:00 AM o más
            if (ahora_tm->tm_mday != ultimo_reset_dia &&
                ahora_tm->tm_hour >= 1)
            {
                ultimo_reset_dia = ahora_tm->tm_mday;

                printf("Nuevo día y hora >= 01:00. Reiniciando contadores...\n");
                reset_contadores();
            }
        }
        else
        {
            printf("No se pudo obtener hora local\n");
        }

        vTaskDelay(pdMS_TO_TICKS(60000));  // Verifica cada minuto
    }
}

