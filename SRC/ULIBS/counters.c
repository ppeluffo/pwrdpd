/*
 * https://stackoverflow.com/questions/13730786/c-pass-int-array-pointer-as-parameter-into-a-function
 *  
 */
#include "counters.h"


counter_t CNTCB[MAX_COUNTERS];

TimerHandle_t counter_xTimer;
StaticTimer_t counter_xTimerBuffer;
void counter_TimerCallback( TimerHandle_t xTimer );


void COUNTER_INIT_OUTOUTOFRTOS(void)
{
	// Configuro el timer que va a generar los pulsos del stepper
	// Se debe correr antes que empieze el RTOS
    /* C/tick, si está en 0, sumo 1*/
    
uint8_t i;
    
	counter_xTimer = xTimerCreateStatic ("CNT",
			pdMS_TO_TICKS( 10 ),
			pdTRUE,
			( void * ) 0,
			counter_TimerCallback,
			&counter_xTimerBuffer
			);
    //
    for (i=0; i<MAX_COUNTERS;i++) {
        CNTCB[i].nivel = 0;
        CNTCB[i].nivel_anterior = 0;
        CNTCB[i].count = 0;
    }
    
    // El timer arranca siempre
    xTimerStart( counter_xTimer, 10 );
}
//------------------------------------------------------------------------------
void counter_init(void)
{
    // Los pines son entradas
    //CNT0_CONFIG();
   // CNT1_CONFIG();
   // CNT2_CONFIG();
    
    PORTC.DIR &= ~PIN1_bm;
    PORTC.DIR &= ~PIN2_bm;
    PORTC.DIR &= ~PIN3_bm;
    // Habilito a interrumpir, pullup, flanco de bajada.
    //PORTC.PIN1CTRL = PORT_ISC1_bm | PORT_ISC0_bm;
    //PORTC.PIN2CTRL = PORT_ISC1_bm | PORT_ISC0_bm;
    //PORTC.PIN3CTRL = PORT_ISC1_bm | PORT_ISC0_bm;
    
}
//------------------------------------------------------------------------------
void counter_TimerCallback( TimerHandle_t xTimer )
{
uint8_t i;

    //return;

    for(i=0; i<MAX_COUNTERS;i++) {

        CNTCB[i].nivel = counter_read_pin(i);
        if ( ( CNTCB[i].nivel_anterior == 1) && ( CNTCB[i].nivel == 0) ) {
            CNTCB[i].count++;
        }
        CNTCB[i].nivel_anterior = CNTCB[i].nivel;
    }
}
// -----------------------------------------------------------------------------
void counter_clear(uint8_t i)
{
    CNTCB[i].count = 0;
}
// -----------------------------------------------------------------------------
uint16_t counter_read(uint8_t i)
{
    return(CNTCB[i].count);
}
// -----------------------------------------------------------------------------
uint8_t counter_read_pin(uint8_t i)
{
    switch(i) {
        case 0:
            return ( ( CNT0_PORT.IN & CNT0_PIN_bm ) >> CNT0_PIN) ;
            break;
        case 1:
            return ( ( CNT1_PORT.IN & CNT1_PIN_bm ) >> CNT1_PIN) ;
            break;     
        case 2:
            return ( ( CNT2_PORT.IN & CNT2_PIN_bm ) >> CNT2_PIN) ;
            break;
    }
    return(0);
}
// -----------------------------------------------------------------------------
uint8_t COUNTER_cmdread_pin( char *s_id)
{
    return ( counter_read_pin( atoi(s_id)));
}
// -----------------------------------------------------------------------------
/*
ISR(PORTC_PORT_vect)
{

    // Borro las flags.
    PORTC.INTFLAGS = 0xFF;
    VPORTC.INTFLAGS = 0xff;
    
    CNTCB[0].count++;
    CNTCB[1].count++;
    CNTCB[2].count++;
            
}
*/