#ifndef OBTENER_FECHA_H
#define OBTENER_FECHA_H

#include <stdint.h>

// Inicializa SNTP (cliente NTP)
void init_sntp_client(void);

// Función que establece la hora (la llama LWIP internamente)
void set_time_from_sntp(uint32_t sec);

// Tarea que revisa si ha cambiado el día (para reiniciar contadores)
void tarea_verificar_fecha(void *arg);

#endif // OBTENER_FECHA_H
