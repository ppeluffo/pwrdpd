/* 
 * File:   valves.h
 * Author: pablo
 *
 * Created on 24 de mayo de 2023, 03:45 PM
 */

#ifndef VALVES_H
#define	VALVES_H

#ifdef	__cplusplus
extern "C" {
#endif

#include "FreeRTOS.h"
#include "task.h"
#include "pines.h"
#include "stdint.h"
#include "stdlib.h"
#include "string.h"
#include "drv8814.h"
#include "xprintf.h"


typedef enum { VALVE_CLOSE=0, VALVE_OPEN } t_valve_state;
    
typedef struct {
    uint16_t pulseWidth;
    uint8_t state;
} t_valves;
   
#define NRO_VALVES 4


void valve_init(void);
void valve_open(uint8_t vid);
void valve_close(uint8_t vid);

void valve_set_pulseWidth(uint8_t id, uint16_t pulseWidth);
uint16_t valve_get_pulseWidth(uint8_t id);
uint8_t valve_get_state(uint8_t id);

void valve_set_local_config(t_valves *from_sconf );
void valve_get_local_config(t_valves *to_sconf );
void valve_default_config(void);

#ifdef	__cplusplus
}
#endif

#endif	/* VALVES_H */

