/*
 * File:   tkCtl.c
 * Author: pablo
 *
 * Created on 25 de octubre de 2021, 12:50 PM
 */


#include "pwrdpd.h"

#define TKCTL_DELAY_S	5

void sys_watchdog_check(void);
void sys_daily_reset(void);

//------------------------------------------------------------------------------
void tkCtl(void * pvParameters)
{

	// Esta es la primer tarea que arranca.

( void ) pvParameters;

	vTaskDelay( ( TickType_t)( 500 / portTICK_PERIOD_MS ) );
    xprintf_P(PSTR("Starting tkCtl..\r\n"));
    
    // Leo la configuracion 
    if ( ! load_config() )  {
       xprintf_P(PSTR("Loading config default..\r\n"));
       config_default();
    }
       
    WDG_INIT();
    // Por ultimo habilito a todas las otras tareas a arrancar
    
    valve_init();
   
    starting_flag = true;
    
	for( ;; )
	{
        vTaskDelay( ( TickType_t)( 1000 / portTICK_PERIOD_MS ) );
		//vTaskDelay( ( TickType_t)( 1000 * TKCTL_DELAY_S / portTICK_PERIOD_MS ) );
        led_flash();
        sys_watchdog_check();
        sys_daily_reset();
               
	}
}
//------------------------------------------------------------------------------
void sys_watchdog_check(void)
{
    // El watchdog se inicializa en 2F.
    // Cada tarea debe poner su bit en 0. Si alguna no puede, se resetea
    // Esta funcion se corre cada 5s (TKCTL_DELAY_S)
    
static uint8_t wdg_count = 0;

    //xprintf_P(PSTR("wdg reset\r\n"));
    //wdt_reset();
    //return;
        
    // EL wdg lo leo cada 300secs ( 5 x 60 )
    if ( wdg_count++ < 10 ) {
        wdt_reset();
    }
    
    // Analizo los watchdows individuales
    wdg_count = 0;
    //xprintf_P(PSTR("tkCtl: check wdg [0x%02X]\r\n"), sys_watchdog );

    if ( sys_watchdog != 0 ) {  
        xprintf_P(PSTR("tkCtl: reset by wdg [0x%02X]\r\n"), sys_watchdog );
        vTaskDelay( ( TickType_t)( 1000 / portTICK_PERIOD_MS ) );
        reset();
    } else {
        wdt_reset();
        WDG_INIT();
    }
}
//------------------------------------------------------------------------------
void sys_daily_reset(void)
{
	// Todos los dias debo resetearme para restaturar automaticamente posibles
	// problemas.
	// Se invoca 1 vez por minuto ( 60s ).

static uint32_t ticks_to_reset = 86400 / TKCTL_DELAY_S ; // ticks en 1 dia.

	//xprintf_P( PSTR("DEBUG dailyReset..\r\n\0"));
	while ( --ticks_to_reset > 0 ) {
		return;
	}

	xprintf_P( PSTR("Daily Reset !!\r\n\0") );
    vTaskDelay( ( TickType_t)( 2000 / portTICK_PERIOD_MS ) );
    reset();
    
}
//------------------------------------------------------------------------------
