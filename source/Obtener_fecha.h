#ifndef OBTENER_FECHA_H
#define OBTENER_FECHA_H

#include <stdint.h>

void init_sntp_client(void);

void set_time_from_sntp(uint32_t sec);

void tarea_verificar_fecha(void *arg);

#endif // OBTENER_FECHA_H
