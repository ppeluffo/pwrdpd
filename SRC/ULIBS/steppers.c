
#include "steppers.h"
#include "led.h"

// https://www.monolithicpower.com/bipolar-stepper-motors-part-i-control-modes

/*
 STEPPERS:
 * Por ahora, el periodo es fijo en 50ms ya que usamos un solo timer para todos
 * los steppers.
 * 
 */
TimerHandle_t stepper_xTimer;
StaticTimer_t stepper_xTimerBuffer;
void stepper_TimerCallback( TimerHandle_t xTimer );


t_stepper STEPPERLIST[NRO_STEPPERS];
//
//------------------------------------------------------------------------------ 
void steppers_default_config(void)
{
uint8_t i;
    
    for (i=0; i<NRO_STEPPERS;i++) {
        STEPPERLIST[i].dir = STEPPER_FWD;
        STEPPERLIST[i].pulseWidth = 45;
        STEPPERLIST[i].ms2run = 0;
        STEPPERLIST[i].period = 50;
        STEPPERLIST[i].pulses2run = 0;
        STEPPERLIST[i].phase = pv_stepper_init_phase();
        STEPPERLIST[i].pulseCounter = 0;
        STEPPERLIST[i].msCounter = 0;
        STEPPERLIST[i].run = false;
    }

}
//------------------------------------------------------------------------------ 
void steppers_set_local_config(t_stepper *from_sconf )
{
    /*
     * Copia la configuracion local en el *dst
     */
    
uint8_t size = NRO_STEPPERS * sizeof(t_stepper);

    memcpy( (char *)&STEPPERLIST[0], from_sconf, size);
} 
//------------------------------------------------------------------------------ 
void steppers_get_local_config(t_stepper *to_sconf )
{
    /*
     * Copia la configuracion local en el *dst
     */
uint8_t size = NRO_STEPPERS * sizeof(t_stepper);
    
    memcpy( to_sconf, (char *)&STEPPERLIST[0], size);
}   
//------------------------------------------------------------------------------    
void stepper_stop(uint8_t id)
{
    STEPPERLIST[id].run = false;
    DRV8814_sleep(id);

}
//------------------------------------------------------------------------------
void stepper_run(uint8_t id)
{
    // Activo el driver
    DRV8814_awake(id);
	vTaskDelay( ( TickType_t)( 1000 / portTICK_PERIOD_MS ) );

    STEPPERLIST[id].msCounter = 0;
    STEPPERLIST[id].pulseCounter = 0;
    STEPPERLIST[id].run = true;

}
//------------------------------------------------------------------------------
void stepper_set_state(uint8_t id, bool f_run)
{
    /*
     * Prende o apaga el motor.
     */
    STEPPERLIST[id].run = f_run;
    
}
//------------------------------------------------------------------------------
void stepper_set_runMode(uint8_t id, uint8_t runMode)
{
    /*
     *  Modo a operar el motor: 0: tiempo, 1: pulsos
     */
    STEPPERLIST[id].runMode = runMode;
    
}
//------------------------------------------------------------------------------
void stepper_set_ms2run(uint8_t id, uint32_t ms2run)
{
    /*
     * Setea el la cantidad de tiempo que tener el motor moviendose
     */
    
    STEPPERLIST[id].ms2run = ms2run;
    //xprintf_P(PSTR("DEBUG ms2run: %d=%lu\r\n"), id, STEPPERLIST[id].ms2run );
    
}
//------------------------------------------------------------------------------
void stepper_set_pulses2run(uint8_t id, uint32_t pulses2run)
{
    /*
     * Setea el la cantidad de pulsos a aplicar
     */
    
    STEPPERLIST[id].pulses2run = pulses2run;
    
}
//------------------------------------------------------------------------------
void stepper_set_period(uint8_t id, uint16_t period)
{
    /*
     * Setea el periodo del tren de pulsos que se aplica al motor.
     * Es para todos los motores.
     */
    
uint8_t i;
    
    for (i=0; i<NRO_STEPPERS; i++) {
        STEPPERLIST[i].period = period;
    }
}
//------------------------------------------------------------------------------
void stepper_set_pulseWidth(uint8_t id, uint16_t pulseWidth)
{
    /*
     * Setea el ancho del pulso que se aplica en c/fase al motor
     */
    
    STEPPERLIST[id].pulseWidth = pulseWidth;
    
    //xprintf_P(PSTR("DEBUG: %d=%d\r\n"), id, STEPPERLIST[id].pulseWidth );
}
//------------------------------------------------------------------------------
void stepper_set_dir(uint8_t id, uint8_t dir)
{
    /*
     * Setea la direccion de movimiento del motor id
     */
    STEPPERLIST[id].dir = dir;
    
}
//------------------------------------------------------------------------------
uint8_t stepper_get_dir(uint8_t id)
{
    return ( STEPPERLIST[id].dir );
}
//------------------------------------------------------------------------------
uint16_t stepper_get_pulseWidth(uint8_t id)
{
    
    return( STEPPERLIST[id].pulseWidth );
}
//------------------------------------------------------------------------------
uint16_t stepper_get_period(uint8_t id)
{
    // EL periodo es por ahora fijo en 50ms.
    return(50);
    //return( STEPPERLIST[id].period );
}
//------------------------------------------------------------------------------
uint32_t stepper_get_pulses2run(uint8_t id)
{
    
    return( STEPPERLIST[id].pulses2run );
}
//------------------------------------------------------------------------------
uint32_t stepper_get_ms2run(uint8_t id)
{
    
    return( STEPPERLIST[id].ms2run );
}
//------------------------------------------------------------------------------
uint8_t stepper_get_runMode(uint8_t id)
{
    
    return( STEPPERLIST[id].runMode );
}
//------------------------------------------------------------------------------
uint8_t stepper_get_state(uint8_t id)
{
    
    return( STEPPERLIST[id].run );
}
//------------------------------------------------------------------------------
uint32_t stepper_get_pulseCounter(uint8_t id)
{
    
    return( STEPPERLIST[id].pulseCounter );
}
//------------------------------------------------------------------------------
uint32_t stepper_get_msCounter(uint8_t id)
{
    
    return( STEPPERLIST[id].msCounter );
}
//------------------------------------------------------------------------------
//
void STEPPER_INIT_OUTOUTOFRTOS(void)
{
	// Configuro el timer que va a generar los pulsos del stepper
	// Se debe correr antes que empieze el RTOS

uint8_t i;
    
    for (i=0; i<NRO_STEPPERS;i++) {
        STEPPERLIST[i].phase = pv_stepper_init_phase();
        STEPPERLIST[i].pulseCounter = 0;
        STEPPERLIST[i].msCounter = 0;
        STEPPERLIST[i].run = false;
    }
        
	stepper_xTimer = xTimerCreateStatic ("STEPPER",
			pdMS_TO_TICKS( 50 ),
			pdTRUE,
			( void * ) 0,
			stepper_TimerCallback,
			&stepper_xTimerBuffer
			);
    //
    // El timer arranca siempre
    xTimerStart( stepper_xTimer, 10 );
}
//------------------------------------------------------------------------------
void stepper_TimerCallback( TimerHandle_t xTimer )
{
	/*
     *  Genera un pulso.
	 *  Si esta en modo pulso, cuenta hasta llegar al final.
     *  Si esta en modo tiempo, cada pulso son 50ms que se van acumulando.
     */

uint8_t i;
    
    for (i=0; i<NRO_STEPPERS; i++) {
        if ( STEPPERLIST[i].run ) {
            pv_stepper_set_phase(i);
            
            STEPPERLIST[i].msCounter += 50;
            STEPPERLIST[i].pulseCounter++;
            
            // Si el modo esta en tiempo.
            if ( STEPPERLIST[i].runMode == 0) {
                if ( STEPPERLIST[i].msCounter >= STEPPERLIST[i].ms2run ) {
                    STEPPERLIST[i].run = false;
                    continue;
                }
            } 

            // Si el modo esta en pulsos.
            if ( STEPPERLIST[i].runMode == 1) {
                if ( STEPPERLIST[i].pulseCounter >= STEPPERLIST[i].pulses2run ) {
                    STEPPERLIST[i].run = false;
                    continue;
                }
            }

            // Sigo contando.
            pv_stepper_next_phase(i);
        }
    }

}
//------------------------------------------------------------------------------
uint8_t pv_stepper_init_phase(void)
{
	return(2);
}
//------------------------------------------------------------------------------
void pv_stepper_next_phase( uint8_t id )
{
    
	if ( STEPPERLIST[id].dir == STEPPER_FWD ) {
		STEPPERLIST[id].phase++;
		if ( STEPPERLIST[id].phase == 4) {
			STEPPERLIST[id].phase = 0;
		}

	}

	if ( STEPPERLIST[id].dir == STEPPER_REV ) {
		STEPPERLIST[id].phase--;
		if ( STEPPERLIST[id].phase == -1 ) {
			STEPPERLIST[id].phase = 3;
		}
	}  
}
//------------------------------------------------------------------------------
void pv_stepper_set_phase( uint8_t id)
{
	// Aplica el pulso al motor y genera la siguiente secuencia

    switch ( STEPPERLIST[id].phase) {
        case 0:
            // A+A-
            DRV8814_pulse_Amas_Amenos(id, STEPPERLIST[id].pulseWidth);
            break;
        case 1:
            // B+B-
            DRV8814_pulse_Bmas_Bmenos(id, STEPPERLIST[id].pulseWidth);
            break;
        case 2:
            // B- 180 degree
            // A-A+
            DRV8814_pulse_Amenos_Amas(id, STEPPERLIST[id].pulseWidth);
            break;
        case 3:
            // A-, 90 degree
            // B-B+
            DRV8814_pulse_Bmenos_Bmas(id, STEPPERLIST[id].pulseWidth);
            break;
    }
}
//------------------------------------------------------------------------------