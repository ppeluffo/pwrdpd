
#include "valves.h"


t_valves VALVELIST[NRO_VALVES];


//------------------------------------------------------------------------------ 
void valve_default_config(void)
{
uint8_t i;

    for (i=0; i < NRO_VALVES; i++ ) {
        VALVELIST[i].pulseWidth = 30;
        VALVELIST[i].state = VALVE_CLOSE;
    }
}
//------------------------------------------------------------------------------ 
void valve_set_local_config(t_valves *from_sconf )
{
    /*
     * Copia la configuracion local en el *dst
     */
    
uint8_t size = NRO_VALVES * sizeof(t_valves);


    memcpy( (char *)&VALVELIST[0], from_sconf, size);
} 
//------------------------------------------------------------------------------ 
void valve_get_local_config(t_valves *to_sconf )
{
    /*
     * Copia la configuracion local en el *dst
     */
uint8_t size = NRO_VALVES * sizeof(t_valves);

    
    memcpy( to_sconf, (char *)&VALVELIST[0], size );
}   
//------------------------------------------------------------------------------  
void valve_init(void)
{
    
    uint8_t i;
    
    for (i=0; i < NRO_VALVES; i++) {
        VALVELIST[i].pulseWidth = 30;
        VALVELIST[i].state = VALVE_CLOSE;
        valve_close(i);
    }
}
//------------------------------------------------------------------------------
void valve_set_pulseWidth(uint8_t id, uint16_t pulseWidth)
{
    VALVELIST[id].pulseWidth = pulseWidth;
}
//------------------------------------------------------------------------------
uint16_t valve_get_pulseWidth(uint8_t id)
{
    return( VALVELIST[id].pulseWidth );
}
//------------------------------------------------------------------------------
uint8_t valve_get_state(uint8_t id)
{
    return( VALVELIST[id].state );    
}
//------------------------------------------------------------------------------
void valve_open(uint8_t vid)
{
    
    switch(vid) {
        case 0:
            DRV8814_awake(3);
            DRV8814_pulse_Amas_Amenos(3, VALVELIST[0].pulseWidth);
            DRV8814_sleep(3);
            break;
        case 1:
            DRV8814_awake(3);
            DRV8814_pulse_Bmas_Bmenos(3, VALVELIST[1].pulseWidth);
            DRV8814_sleep(3);        
            break;
        case 2:
            DRV8814_awake(4);
            DRV8814_pulse_Amas_Amenos(4, VALVELIST[2].pulseWidth);
            DRV8814_sleep(4);   
            break;
        case 3:
            DRV8814_awake(4);
            DRV8814_pulse_Bmas_Bmenos(4, VALVELIST[3].pulseWidth);
            DRV8814_sleep(4);            
            break;     
    }
    
    VALVELIST[vid].state = VALVE_OPEN;
}
//------------------------------------------------------------------------------
void valve_close(uint8_t vid)
{
    
    switch(vid) {
        case 0:
            DRV8814_awake(3);
            DRV8814_pulse_Amenos_Amas(3, VALVELIST[0].pulseWidth);
            DRV8814_sleep(3);
            break;
        case 1:
            DRV8814_awake(3);
            DRV8814_pulse_Bmenos_Bmas(3, VALVELIST[1].pulseWidth);;
            DRV8814_sleep(3);        
            break;
        case 2:
            DRV8814_awake(4);
            DRV8814_pulse_Amenos_Amas(4, VALVELIST[2].pulseWidth);
            DRV8814_sleep(4);   
            break;
        case 3:
            DRV8814_awake(4);
            DRV8814_pulse_Bmenos_Bmas(4, VALVELIST[3].pulseWidth);
            DRV8814_sleep(4);            
            break;     
    }
    
    VALVELIST[vid].state = VALVE_CLOSE;
}
//------------------------------------------------------------------------------
