#include "pwrdpd.h"
#include "frtos_cmd.h"
#include "modbus.h"

//------------------------------------------------------------------------------
void tkModbus(void * pvParameters)
{

	// Esta es la primer tarea que arranca.

( void ) pvParameters;

    while ( ! starting_flag )
        vTaskDelay( ( TickType_t)( 100 / portTICK_PERIOD_MS ) );


uint8_t c = 0;
uint16_t ptrA;
    
    xprintf_P(PSTR("Starting tkModbus..\r\n" ));
       
    modbus_init_rx_buffer();
    
	// loop
	for( ;; )
	{
        kick_wdt(MBUS_WDG_bp);
         
		c = '\0';	// Lo borro para que luego del un CR no resetee siempre el timer.
        // Mientras hallan datos para leer, los leo.
        while ( xfgetc( fdRS485A, (char *)&c ) == 1 ) {
            modbus_put_rx_buffer(c);
        }
        
        // Esperamos 50ms y vemos si llegaron mas datos.
        ptrA = modbus_get_rxPtr();
        vTaskDelay( ( TickType_t)( 50 / portTICK_PERIOD_MS ) );
        // Despues de 50ms no llego nada: proceso
        if ( ( ptrA > 0 ) && ( ptrA == modbus_get_rxPtr() ) ) {
            modbus_process_rx_frame();
            modbus_flush_rx_frame();
        }
               
	}    
}
//------------------------------------------------------------------------------
