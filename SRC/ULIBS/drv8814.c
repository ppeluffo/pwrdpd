
#include "drv8814.h"
#include "string.h"
#include <avr/pgmspace.h>

#include "xprintf.h"

bool DRV8814_test( char *s_id, char *s_pinName, char *s_action)
{
    
uint8_t id = atoi(s_id);

    // BPH
    if (! strcmp_P( strupr(s_pinName), PSTR("BPH") ) ) {
        if (! strcmp_P( strupr(s_action), PSTR("SET"))) {
            switch(id) {
                case 0:
                    SET_DRV8814_0_BPH;
                    return(true);
                case 1:
                    SET_DRV8814_1_BPH;
                    return(true);
                case 2:
                    SET_DRV8814_2_BPH;
                    return(true);                    
            }
            return(false);
        }
        
        if (!strcmp_P( strupr(s_action), PSTR("CLEAR"))) {
            switch(id) {
                case 0:
                    CLEAR_DRV8814_0_BPH;
                    return(true);
                case 1:
                    CLEAR_DRV8814_1_BPH;
                    return(true);
                case 2:
                    CLEAR_DRV8814_2_BPH;
                    return(true);
            }
            return(false);
        }
        return (false);
    }

    // APH
    if (! strcmp_P( strupr(s_pinName), PSTR("APH") ) ) {
        if (! strcmp_P( strupr(s_action), PSTR("SET"))) {
            switch(id) {
                case 0:
                    SET_DRV8814_0_APH;
                    return(true);
                case 1:
                    SET_DRV8814_1_APH;
                    return(true);
                case 2:
                    SET_DRV8814_2_APH;
                    return(true);
            }
            return(false);
        }
        
        if (!strcmp_P( strupr(s_action), PSTR("CLEAR"))) {
            switch(id) {
                case 0:
                    CLEAR_DRV8814_0_APH;
                    return(true);
                case 1:
                    CLEAR_DRV8814_1_APH;
                    return(true);
                case 2:
                    CLEAR_DRV8814_2_APH;
                    return(true);
            }
            return(false);
        }
        return (false);
    }

    // BEN
    if (! strcmp_P( strupr(s_pinName), PSTR("BEN") ) ) {
        if (! strcmp_P( strupr(s_action), PSTR("SET"))) {
            switch(id) {
                case 0:
                    SET_DRV8814_0_BEN;
                    return(true);
                case 1:
                    SET_DRV8814_1_BEN;
                    return(true);
                case 2:
                    SET_DRV8814_2_BEN;
                    return(true);
            }
            return(false);
        }
        
        if (!strcmp_P( strupr(s_action), PSTR("CLEAR"))) {
            switch(id) {
                case 0:
                    CLEAR_DRV8814_0_BEN;
                    return(true);
                case 1:
                    CLEAR_DRV8814_1_BEN;
                    return(true);
                case 2:
                    CLEAR_DRV8814_2_BEN;
                    return(true);
            }
            return(false);
        }
        return (false);
    }

    // AEN
    if (! strcmp_P( strupr(s_pinName), PSTR("AEN") ) ) {
        if (! strcmp_P( strupr(s_action), PSTR("SET"))) {
            switch(id) {
                case 0:
                    SET_DRV8814_0_AEN;
                    return(true);
                case 1:
                    SET_DRV8814_1_AEN;
                    return(true);
                case 2:
                    SET_DRV8814_2_AEN;
                    return(true);
            }
            return(false);
        }
        
        if (!strcmp_P( strupr(s_action), PSTR("CLEAR"))) {
            switch(id) {
                case 0:
                    CLEAR_DRV8814_0_AEN;
                    return(true);
                case 1:
                    CLEAR_DRV8814_1_AEN;
                    return(true);
                case 2:
                    CLEAR_DRV8814_2_AEN;
                    return(true);
            }
            return(false);
        }
        return (false);
    }

    // RESET
    if (! strcmp_P( strupr(s_pinName), PSTR("RESET") ) ) {
        if (! strcmp_P( strupr(s_action), PSTR("SET"))) {
            switch(id) {
                case 0:
                    SET_DRV8814_0_RESET;
                    return(true);
                case 1:
                    SET_DRV8814_1_RESET;
                    return(true);
                case 2:
                    xprintf_P(PSTR("DEBUG1\r\n"));
                    SET_DRV8814_2_RESET;
                    return(true);
            }
            return(false);
        }
        
        if (!strcmp_P( strupr(s_action), PSTR("CLEAR"))) {
            switch(id) {
                case 0:
                    CLEAR_DRV8814_0_RESET;
                    return(true);
                case 1:
                    CLEAR_DRV8814_1_RESET;
                    return(true);
                case 2:
                    xprintf_P(PSTR("DEBUG2\r\n"));
                    CLEAR_DRV8814_2_RESET;
                    return(true);
            }
            return(false);
        }
        return (false);
    }
    
    // SLEEP
    if (! strcmp_P( strupr(s_pinName), PSTR("SLEEP") ) ) {
        if (! strcmp_P( strupr(s_action), PSTR("SET"))) {
            switch(id) {
                case 0:
                    SET_DRV8814_0_SLEEP;
                    return(true);
                case 1:
                    SET_DRV8814_1_SLEEP;
                    return(true);
                case 2:
                    SET_DRV8814_2_SLEEP;
                    return(true);
            }
            return(false);
        }
        
        if (!strcmp_P( strupr(s_action), PSTR("CLEAR"))) {
            switch(id) {
                case 0:
                    CLEAR_DRV8814_0_SLEEP;
                    return(true);
                case 1:
                    CLEAR_DRV8814_1_SLEEP;
                    return(true);
                case 2:
                    CLEAR_DRV8814_2_SLEEP;
                    return(true);
            }
            return(false);
        }
        return (false);
    }

    return (false);
}
// -----------------------------------------------------------------------------
void DRV8814_init(uint8_t id)
{
    // Configura los pines del DRV para mover los steppers.
    switch(id) {
        case 0:
            CONFIG_DRV8814_0_RESET;
            CONFIG_DRV8814_0_SLEEP;
            CONFIG_DRV8814_0_AEN;
            CONFIG_DRV8814_0_BEN;
            CONFIG_DRV8814_0_APH;
            CONFIG_DRV8814_0_BPH;
            break;
        case 1:
            CONFIG_DRV8814_1_RESET;
            CONFIG_DRV8814_1_SLEEP;
            CONFIG_DRV8814_1_AEN;
            CONFIG_DRV8814_1_BEN;
            CONFIG_DRV8814_1_APH;
            CONFIG_DRV8814_1_BPH;
            break; 
        case 2:
            CONFIG_DRV8814_2_RESET;
            CONFIG_DRV8814_2_SLEEP;
            CONFIG_DRV8814_2_AEN;
            CONFIG_DRV8814_2_BEN;
            CONFIG_DRV8814_2_APH;
            CONFIG_DRV8814_2_BPH;
            break;
        case 3:
            CONFIG_DRV8814_3_RESET;
            CONFIG_DRV8814_3_SLEEP;
            CONFIG_DRV8814_3_AEN;
            CONFIG_DRV8814_3_BEN;
            CONFIG_DRV8814_3_APH;
            CONFIG_DRV8814_3_BPH;
            break;
        case 4:
            CONFIG_DRV8814_4_RESET;
            CONFIG_DRV8814_4_SLEEP;
            CONFIG_DRV8814_4_AEN;
            CONFIG_DRV8814_4_BEN;
            CONFIG_DRV8814_4_APH;
            CONFIG_DRV8814_4_BPH;
            break;
    }
    //
}
// -----------------------------------------------------------------------------
void DRV8814_awake(uint8_t id)
{
	// Saco al driver 8814 de reposo.
    switch(id) {
        case 0:
            SET_DRV8814_0_RESET;
            SET_DRV8814_0_SLEEP;
            break;
        case 1:
            SET_DRV8814_1_RESET;
            SET_DRV8814_1_SLEEP;
            break;
        case 2:
            SET_DRV8814_2_RESET;
            SET_DRV8814_2_SLEEP;
            break;
        case 3:
            SET_DRV8814_3_RESET;
            SET_DRV8814_3_SLEEP;
            break;
        case 4:
            SET_DRV8814_4_RESET;
            SET_DRV8814_4_SLEEP;
            break;
    }
	
}
//------------------------------------------------------------------------------
void DRV8814_sleep(uint8_t id)
{
	// Pongo en reposo
    switch(id) {
        case 0:
            CLEAR_DRV8814_0_RESET;
            CLEAR_DRV8814_0_SLEEP;
            break;
        case 1:
            CLEAR_DRV8814_1_RESET;
            CLEAR_DRV8814_1_SLEEP;
            break;
        case 2:
            CLEAR_DRV8814_2_RESET;
            CLEAR_DRV8814_2_SLEEP;
            break;            
        case 3:
            CLEAR_DRV8814_3_RESET;
            CLEAR_DRV8814_3_SLEEP;
            break;
        case 4:
            CLEAR_DRV8814_4_RESET;
            CLEAR_DRV8814_4_SLEEP;
            break;   
    }

}
//------------------------------------------------------------------------------
void DRV8814_pulse_Amas_Amenos(uint8_t id, uint16_t dtime )
{
	// Aout1 = H, Aout2 = L

    switch(id) {
        case 0:
            SET_DRV8814_0_APH;	// Direccion del pulso forward
            SET_DRV8814_0_AEN;	// Habilito el pulso  
            break;
        case 1:
            SET_DRV8814_1_APH;	
            SET_DRV8814_1_AEN;
            break;
        case 2:
            SET_DRV8814_2_APH;	
            SET_DRV8814_2_AEN;
            break; 
        case 3:
            SET_DRV8814_3_APH;	
            SET_DRV8814_3_AEN;
            break;
        case 4:
            SET_DRV8814_4_APH;	
            SET_DRV8814_4_AEN;
            break;             
    }
	
    if (dtime > 0) {
        vTaskDelay( ( TickType_t)( dtime / portTICK_PERIOD_MS ) );
    }
    
    switch(id) {
        case 0:
            CLEAR_DRV8814_0_AEN;	// Deshabilito el pulso
            break;
        case 1:
            CLEAR_DRV8814_1_AEN;
            break;
        case 2:
            CLEAR_DRV8814_2_AEN;
            break;
        case 3:
            CLEAR_DRV8814_3_AEN;
            break;
        case 4:
            CLEAR_DRV8814_4_AEN;
            break;
    }

}
//------------------------------------------------------------------------------
void DRV8814_pulse_Amenos_Amas(uint8_t id, uint16_t dtime )
{
	// Aout1 = L, Aout2 = H
    switch (id) {
        case 0:
            CLEAR_DRV8814_0_APH;	// Direccion del pulso reverse
            SET_DRV8814_0_AEN;      // Habilito el pulso
            break;
        case 1:
            CLEAR_DRV8814_1_APH;	
            SET_DRV8814_1_AEN;      
            break;
        case 2:
            CLEAR_DRV8814_2_APH;	
            SET_DRV8814_2_AEN;      
            break;
        case 3:
            CLEAR_DRV8814_3_APH;	
            SET_DRV8814_3_AEN;      
            break;
        case 4:
            CLEAR_DRV8814_4_APH;	
            SET_DRV8814_4_AEN;      
            break;
    }
            
    if (dtime > 0) {
        vTaskDelay( ( TickType_t)( dtime / portTICK_PERIOD_MS ) );
    }
    
    switch (id) {
        case 0:
            CLEAR_DRV8814_0_AEN;	// Deshabilito el pulso
            break;
        case 1:
            CLEAR_DRV8814_1_AEN;	
            break;
        case 2:
            CLEAR_DRV8814_2_AEN;	
            break;
        case 3:
            CLEAR_DRV8814_3_AEN;	
            break;
        case 4:
            CLEAR_DRV8814_4_AEN;	
            break;
    }
}
//------------------------------------------------------------------------------
void DRV8814_pulse_Bmas_Bmenos(uint8_t id, uint16_t dtime )
{
	// Bout1 = H, Bout2 = L
    switch (id) {
        case 0:
            SET_DRV8814_0_BPH;	// Direccion del pulso forward
            SET_DRV8814_0_BEN;	// Habilito el pulso
            break;
        case 1:
            SET_DRV8814_1_BPH;	
            SET_DRV8814_1_BEN;	
            break;
        case 2:
            SET_DRV8814_2_BPH;	
            SET_DRV8814_2_BEN;	
            break;
        case 3:
            SET_DRV8814_3_BPH;	
            SET_DRV8814_3_BEN;	
            break;
        case 4:
            SET_DRV8814_4_BPH;	
            SET_DRV8814_4_BEN;	
            break;
    }
            
    if (dtime > 0) {
        vTaskDelay( ( TickType_t)( dtime / portTICK_PERIOD_MS ) );
    }
    
    switch (id) {
        case 0:
            CLEAR_DRV8814_0_BEN;	// Deshabilito el pulso
            break;
        case 1:
            CLEAR_DRV8814_1_BEN;
            break;
        case 2:
            CLEAR_DRV8814_2_BEN;
            break;
        case 3:
            CLEAR_DRV8814_3_BEN;
            break;
        case 4:
            CLEAR_DRV8814_4_BEN;
            break;
    }    
}
//------------------------------------------------------------------------------
void DRV8814_pulse_Bmenos_Bmas(uint8_t id, uint16_t dtime )
{
	// Bout1 = L, Bout2 = H 
    
    switch (id) {
        case 0:
            CLEAR_DRV8814_0_BPH;	// Direccion del pulso forward
            SET_DRV8814_0_BEN;	// Habilito el pulso
            break;
        case 1:
            CLEAR_DRV8814_1_BPH;	
            SET_DRV8814_1_BEN;	
            break;
        case 2:
            CLEAR_DRV8814_2_BPH;	
            SET_DRV8814_2_BEN;	
            break;
        case 3:
            CLEAR_DRV8814_3_BPH;	
            SET_DRV8814_3_BEN;	
            break;
        case 4:
            CLEAR_DRV8814_4_BPH;	
            SET_DRV8814_4_BEN;	
            break;
    }
            
    if (dtime > 0) {
        vTaskDelay( ( TickType_t)( dtime / portTICK_PERIOD_MS ) );
    }
    
    switch (id) {
        case 0:
            CLEAR_DRV8814_0_BEN;	// Deshabilito el pulso
            break;
        case 1:
            CLEAR_DRV8814_1_BEN;
            break;
        case 2:
            CLEAR_DRV8814_2_BEN;
            break;
        case 3:
            CLEAR_DRV8814_3_BEN;
            break;
        case 4:
            CLEAR_DRV8814_4_BEN;
            break;
    }

}
//------------------------------------------------------------------------------
