/* 
 * File:   steppers.h
 * Author: pablo
 *
 * Created on 22 de mayo de 2023, 11:59 AM
 */

#ifndef STEPPERS_H
#define	STEPPERS_H

#ifdef	__cplusplus
extern "C" {
#endif

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "pines.h"
#include "stdint.h"
#include "stdlib.h"
#include "string.h"
#include "drv8814.h"
#include "xprintf.h"
    
typedef enum { STEPPER_REV = 0, STEPPER_FWD = 1 } t_stepper_dir;
typedef enum { RUNxTIME = 0, RUNxPULSES = 1 } t_stepper_runMode;

typedef struct {
    uint8_t dir;            // Direccion de movimiento
    uint16_t pulseWidth;    // Ancho del pulso
    uint16_t period;        // Duracion del pulso
    uint32_t pulses2run;    // Pulsos a aplicar al motor. 
    uint32_t ms2run;        // Milisegundos a tener corriendo el motor
    uint8_t runMode;        // Modo a operar el motor: 0: tiempo, 1: pulsos
    bool run;               // Estado de funcionamiento del motor
    uint32_t pulseCounter;  // Pulsos que lleva aplicado el motor.
    uint32_t msCounter;     // milisegundos que lleva corriendo el motor.
    
    int8_t phase;
    
} t_stepper;

#define NRO_STEPPERS 3
//
void stepper_stop(uint8_t id);
void stepper_run(uint8_t id);
//
// SETTERS
void stepper_set_dir(uint8_t id, uint8_t dir);
void stepper_set_pulseWidth(uint8_t id, uint16_t pulseWidth);
void stepper_set_period(uint8_t id, uint16_t period);
void stepper_set_pulses2run(uint8_t id, uint32_t pulses2run);
void stepper_set_ms2run(uint8_t id, uint32_t ms2run);
void stepper_set_runMode(uint8_t id, uint8_t runMode);
void stepper_set_state(uint8_t id, bool f_run);

//GETTERS
uint8_t stepper_get_dir(uint8_t id);
uint16_t stepper_get_pulseWidth(uint8_t id);
uint16_t stepper_get_period(uint8_t id);
uint32_t stepper_get_pulses2run(uint8_t id);
uint32_t stepper_get_ms2run(uint8_t id);
uint8_t stepper_get_runMode(uint8_t id);
uint8_t stepper_get_state(uint8_t id);
uint32_t stepper_get_pulseCounter(uint8_t id);
uint32_t stepper_get_msCounter(uint8_t id);

void STEPPER_INIT_OUTOUTOFRTOS(void);
uint8_t pv_stepper_init_phase(void);
void pv_stepper_next_phase( uint8_t id);
void pv_stepper_set_phase( uint8_t id );

void steppers_get_local_config(t_stepper *to_sconf );
void steppers_set_local_config(t_stepper *from_sconf );
void steppers_default_config(void);

#ifdef	__cplusplus
}
#endif

#endif	/* STEPPERS_H */

