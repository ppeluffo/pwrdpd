
#include "pwrdpd.h"
#include "frtos_cmd.h"

static void cmdClsFunction(void);
static void cmdHelpFunction(void);
static void cmdResetFunction(void);
static void cmdStatusFunction(void);
static void cmdWriteFunction(void);
static void cmdReadFunction(void);
static void cmdTestFunction(void);

static bool pv_CMD_WRITE_steppers(void);
static bool pv_CMD_WRITE_valves(void);

static void pv_snprintfP_OK(void );
static void pv_snprintfP_ERR(void );

//------------------------------------------------------------------------------
void tkCmd(void * pvParameters)
{

	// Esta es la primer tarea que arranca.

( void ) pvParameters;

    while ( ! starting_flag )
        vTaskDelay( ( TickType_t)( 100 / portTICK_PERIOD_MS ) );

	//vTaskDelay( ( TickType_t)( 500 / portTICK_PERIOD_MS ) );

uint8_t c = 0;

    FRTOS_CMD_init();

    FRTOS_CMD_register( "cls", cmdClsFunction );
	FRTOS_CMD_register( "help", cmdHelpFunction );
    FRTOS_CMD_register( "reset", cmdResetFunction );
    FRTOS_CMD_register( "status", cmdStatusFunction );
    FRTOS_CMD_register( "write", cmdWriteFunction );
    FRTOS_CMD_register( "read", cmdReadFunction );
    FRTOS_CMD_register( "test", cmdTestFunction );
    
    xprintf_P(PSTR("Starting tkCmd..\r\n" ));
    xprintf_P(PSTR("Spymovil %s %s %s %s \r\n") , HW_MODELO, FRTOS_VERSION, FW_REV, FW_DATE);
      
	// loop
	for( ;; )
	{
        kick_wdt(CMD_WDG_bp);
         
		c = '\0';	// Lo borro para que luego del un CR no resetee siempre el timer.
        while ( xgetc( (char *)&c ) == 1 ) {
            FRTOS_CMD_process(c);
        }
        
        // Espero 10ms si no hay caracteres en el buffer
        vTaskDelay( ( TickType_t)( 10 / portTICK_PERIOD_MS ) );
               
	}    
}
//------------------------------------------------------------------------------
static void cmdTestFunction(void)
{

    FRTOS_CMD_makeArgv();

        
    if (!strcmp_P( strupr(argv[1]), PSTR("DRV8814"))  ) {
        DRV8814_test( argv[2],argv[3], argv[4])? pv_snprintfP_OK() : pv_snprintfP_ERR();
        return;
    }   
    //
    xprintf_P(PSTR("test drv8814 id pin{sleep,reset,aph,bph,aen,ben} {set,clear}\r\n"));

    
}
//------------------------------------------------------------------------------
static void cmdHelpFunction(void)
{

    FRTOS_CMD_makeArgv();
        
    if ( !strcmp_P( strupr(argv[1]), PSTR("WRITE"))) {
		xprintf_P( PSTR("-write:\r\n"));
        xprintf_P( PSTR("  config\r\n"));
        xprintf_P( PSTR("  stepper id dir {fw,rev}\r\n"));
        xprintf_P( PSTR("             pulsewidth,period,pulses2run,ms2run {val}\r\n"));
        xprintf_P( PSTR("             runmode {time,pulse}\r\n"));
        xprintf_P( PSTR("             state {run,stop}\r\n"));
        //
        xprintf_P( PSTR("  valve id pulsewidth {val}\r\n"));
        xprintf_P( PSTR("           state {open,close}\r\n"));
        
        xprintf_P( PSTR("  modbus addr {val}\r\n"));
        
    }  else if ( !strcmp_P( strupr(argv[1]), PSTR("READ"))) {
		xprintf_P( PSTR("-read:\r\n"));
        xprintf_P( PSTR("  config\r\n"));
        xprintf_P( PSTR("  counter {0,1,2}:\r\n"));

    	// HELP RESET
	} else if (!strcmp_P( strupr(argv[1]), PSTR("RESET"))) {
		xprintf_P( PSTR("-reset\r\n"));
        xprintf_P( PSTR("  counter {0,1,2}\r\n"));
        
    } else if (!strcmp_P( strupr(argv[1]), PSTR("TEST"))) {
		xprintf_P( PSTR("-test\r\n"));
        xprintf_P(PSTR("   drv8814 id pin{sleep,reset,aph,bph,aen,ben} {set,clear}\r\n"));
        xprintf_P(PSTR("   stepper id run dir npulses dtime\r\n"));
        xprintf_P(PSTR("              stop\r\n"));
        return;
        
    }  else {
        // HELP GENERAL
        xprintf_P( PSTR("Available commands are:\r\n"));
        xprintf_P( PSTR("-cls\r\n"));
        xprintf_P( PSTR("-help\r\n"));
        xprintf_P( PSTR("-status\r\n"));
        xprintf_P( PSTR("-reset\r\n"));
        xprintf_P( PSTR("-write...\r\n"));
        xprintf_P( PSTR("-read...\r\n"));

    }

}
//------------------------------------------------------------------------------
static void cmdReadFunction(void)
{
    
    FRTOS_CMD_makeArgv();  
    
    uint8_t pin;
    
    if (!strcmp_P( strupr(argv[1]), PSTR("CONFIG"))  ) {
        load_config();
        pv_snprintfP_OK();
        return;
    }
    
    if (!strcmp_P( strupr(argv[1]), PSTR("COUNTER"))  ) {
        pin =  COUNTER_cmdread_pin(argv[2]);
        xprintf_P(PSTR("PIN=%d\r\n"),pin);
        return;
    }
    // CMD NOT FOUND
	xprintf("ERROR\r\nCMD NOT DEFINED\r\n\0");
	return;
 
}
//------------------------------------------------------------------------------
static void cmdClsFunction(void)
{
	// ESC [ 2 J
	xprintf("\x1B[2J\0");
}
//------------------------------------------------------------------------------
static void cmdResetFunction(void)
{
    
    FRTOS_CMD_makeArgv();
   
    if (!strcmp_P( strupr(argv[1]), PSTR("COUNTER"))  ) {
        counter_clear(atoi(argv[2]));
        return;
    }
    
    xprintf("Reset..\r\n");
    reset();
}
//------------------------------------------------------------------------------
static void cmdStatusFunction(void)
{

uint8_t i;
    
    // https://stackoverflow.com/questions/12844117/printing-defined-constants

    xprintf_P(PSTR("Spymovil %s %s TYPE=%s, VER=%s %s\r\n"), HW_MODELO, FRTOS_VERSION, FW_TYPE, FW_REV, FW_DATE);
    
    xprintf_P(PSTR("Modbus addr: 0x%02X\r\n"), modbus_get_localaddress() );
    
    xprintf_P(PSTR("Steppers:\r\n"));
    xprintf_P(PSTR(" id | Dir | pWidth | period | pulses2run | ms2Run | runmode | pulseCounter | msCounter | state\r\n"));
    xprintf_P(PSTR("----|-----|--------|--------|------------|--------|---------|--------------|-----------|------\r\n"));
    for (i=0; i<NRO_STEPPERS; i++) {
        xprintf_P( PSTR(" %02d |"),i);
        (stepper_get_dir(i) == STEPPER_REV )? xprintf_P(PSTR(" REV |")): xprintf_P(PSTR(" FW  |"));
        xprintf_P(PSTR("  %03d   |"), stepper_get_pulseWidth(i));
        xprintf_P(PSTR("  %03d   |"), stepper_get_period(i));
        xprintf_P(PSTR("   %05lu    |"), stepper_get_pulses2run(i));
        xprintf_P(PSTR(" %05lu  |"), stepper_get_ms2run(i));
        (stepper_get_runMode(i) == RUNxTIME )? xprintf_P(PSTR("  TIME   |")): xprintf_P(PSTR(" PULSOS  |"));
        xprintf_P(PSTR("     %05lu    |"), stepper_get_pulseCounter(i));
        xprintf_P(PSTR("   %05lu   |"), stepper_get_msCounter(i));
        stepper_get_state(i) ? xprintf_P(PSTR(" RUNNING\r\n")): xprintf_P(PSTR(" STOP\r\n"));
    }
    
    xprintf_P(PSTR("Valves:\r\n"));
    for (i=0; i<NRO_VALVES; i++) {
        xprintf_P(PSTR(" V%02d: "), i);
        ( valve_get_state(i) == VALVE_OPEN )? xprintf_P(PSTR("OPEN  | ")): xprintf_P(PSTR("CLOSE | "));
        xprintf_P(PSTR("pw=%d\r\n"), valve_get_pulseWidth(i));
    }
    xprintf_P( PSTR("Cnt0=%d\r\n"), counter_read(0));
    xprintf_P( PSTR("Cnt1=%d\r\n"), counter_read(1));
    xprintf_P( PSTR("Cnt2=%d\r\n"), counter_read(2));
    
    
}
//------------------------------------------------------------------------------
static void cmdWriteFunction(void)
{

    FRTOS_CMD_makeArgv();

    if (!strcmp_P( strupr(argv[1]), PSTR("MODBUS"))  ) {
        if (!strcmp_P( strupr(argv[2]), PSTR("ADDR"))  ) {
            modbus_config_localaddress(argv[3]);
            pv_snprintfP_OK();
            return;
        } else {
           pv_snprintfP_ERR(); 
        }
    }
    
    if (!strcmp_P( strupr(argv[1]), PSTR("CONFIG"))  ) {
        save_config();
        pv_snprintfP_OK();
        return;
    }
    
    if (!strcmp_P( strupr(argv[1]), PSTR("LED"))  ) {
        if (!strcmp_P( strupr(argv[2]), PSTR("ON"))  ) { 
            PRENDER_LED();
            return;  
        
        } else if (!strcmp_P( strupr(argv[2]), PSTR("OFF"))  ) { 
            APAGAR_LED();
            return; 
        
        } else { 
            pv_snprintfP_ERR();
        }
    }
        
    // write stepper id dir {fw,rev}
    // write stepper id pulsewidth,period,pulses2run,ms2run
    // write stepper id runmode {time,pulse}
    // write stepper id state {run,stop}
    if (!strcmp_P( strupr(argv[1]), PSTR("STEPPER"))  ) {
        
        if ( pv_CMD_WRITE_steppers()) {
            pv_snprintfP_OK();
            return;
        } else {
            pv_snprintfP_ERR();
            return;
        } 
    }
    
    // write valve pulsewidth
    // write valve state {open,close}
    if (!strcmp_P( strupr(argv[1]), PSTR("VALVE"))  ) {
        if ( pv_CMD_WRITE_valves()) {
            pv_snprintfP_OK();
            return;
        } else {
            pv_snprintfP_ERR();
            return;
        } 
    }
    
    // CMD NOT FOUND
	xprintf("ERROR\r\nCMD NOT DEFINED\r\n\0");
	return;
 
}
//------------------------------------------------------------------------------
static void pv_snprintfP_OK(void )
{
	xprintf("ok\r\n\0");
}
//------------------------------------------------------------------------------
static void pv_snprintfP_ERR(void)
{
	xprintf("error\r\n\0");
}
//------------------------------------------------------------------------------
static bool pv_CMD_WRITE_steppers(void)
{
    
    if (atoi(argv[2]) >= NRO_STEPPERS ) {
        return (false);
    }
            
    // write stepper id dir {fw,rev}
    if (!strcmp_P( strupr(argv[3]), PSTR("DIR"))  ) {
        
        if (!strcmp_P( strupr(argv[4]), PSTR("FW"))  ) {
            stepper_set_dir( atoi(argv[2]), STEPPER_FWD);
            return(true);
        }
        
        if (!strcmp_P( strupr(argv[4]), PSTR("REV"))  ) {
            stepper_set_dir( atoi(argv[2]), STEPPER_REV);
            return(true);
        }  
        
        return(false);
    }
    
    // write stepper id pulsewidth,period,pulses2run,ms2run
    if (!strcmp_P( strupr(argv[3]), PSTR("PULSEWIDTH"))  ) {
        stepper_set_pulseWidth(atoi(argv[2]),atoi(argv[4]));
        return (true);
    }
    
    if (!strcmp_P( strupr(argv[3]), PSTR("PERIOD"))  ) {
        stepper_set_period(atoi(argv[2]),atoi(argv[4]));
        return (true);
    }
    
    if (!strcmp_P( strupr(argv[3]), PSTR("PULSES2RUN"))  ) {
        stepper_set_pulses2run(atoi(argv[2]),atol(argv[4]));
        return (true);
    }

    if (!strcmp_P( strupr(argv[3]), PSTR("MS2RUN"))  ) {
        stepper_set_ms2run(atoi(argv[2]), atol(argv[4]));
        return (true); 
    }
    
    // write stepper id runmode {time,pulse}
    if (!strcmp_P( strupr(argv[3]), PSTR("RUNMODE"))  ) {
        
        if (!strcmp_P( strupr(argv[4]), PSTR("TIME"))  ) {
            stepper_set_runMode( atoi(argv[2]), RUNxTIME);
            return(true);
            }
        
        if (!strcmp_P( strupr(argv[4]), PSTR("PULSE"))  ) {
            stepper_set_runMode( atoi(argv[2]), RUNxPULSES);
            return(true);
        }  
        
        return(false);
    }   
    
    // write stepper id state {run,stop}
    if (!strcmp_P( strupr(argv[3]), PSTR("STATE"))  ) {
        
        if (!strcmp_P( strupr(argv[4]), PSTR("RUN"))  ) {
            stepper_run( atoi(argv[2]));
            return(true);
            }
        
        if (!strcmp_P( strupr(argv[4]), PSTR("STOP"))  ) {
            stepper_stop( atoi(argv[2]));
            return(true);
        }  
        
        return(false);
    }      
    
    return(false);
}
//------------------------------------------------------------------------------
static bool pv_CMD_WRITE_valves(void)
{
    
    if (atoi(argv[2]) >= NRO_VALVES ) {
        return (false);
    }
            
    // write valve pulsewidth
    if (!strcmp_P( strupr(argv[3]), PSTR("PULSEWIDTH"))  ) {
        valve_set_pulseWidth(atoi(argv[2]),atoi(argv[4]));
        return (true);
    }
        
    // write valve state {open,close}   
    if (!strcmp_P( strupr(argv[3]), PSTR("STATE"))  ) {
        
        if (!strcmp_P( strupr(argv[4]), PSTR("OPEN"))  ) {
            valve_open( atoi(argv[2]));
            return(true);
            }
        
        if (!strcmp_P( strupr(argv[4]), PSTR("CLOSE"))  ) {
            valve_close( atoi(argv[2]));
            return(true);
        }  
        
        return(false);
    }      
    
    return(false);
}
//------------------------------------------------------------------------------
