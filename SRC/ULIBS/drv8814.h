/* 
 * File:   drv8814.h
 * Author: pablo
 *
 * Created on 22 de mayo de 2023, 10:39 AM
 */

#ifndef DRV8814_H
#define	DRV8814_H

#ifdef	__cplusplus
extern "C" {
#endif

#include "pines.h"
#include "stdint.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stdint.h"
#include "stdlib.h"


void DRV8814_init(uint8_t id);    
void DRV8814_awake(uint8_t id);
void DRV8814_sleep(uint8_t id);
void DRV8814_pulse_Amas_Amenos(uint8_t id, uint16_t dtime );
void DRV8814_pulse_Amenos_Amas(uint8_t id, uint16_t dtime );
void DRV8814_pulse_Bmas_Bmenos(uint8_t id, uint16_t dtime );
void DRV8814_pulse_Bmenos_Bmas(uint8_t id, uint16_t dtime );
//
bool DRV8814_test( char *s_id, char *s_pinName, char *s_action);

#ifdef	__cplusplus
}
#endif

#endif	/* DRV8814_H */

