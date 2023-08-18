/* Microchip Technology Inc. and its subsidiaries.  You may use this software 
 * and any derivatives exclusively with Microchip products. 
 * 
 * THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS".  NO WARRANTIES, WHETHER 
 * EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED 
 * WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A 
 * PARTICULAR PURPOSE, OR ITS INTERACTION WITH MICROCHIP PRODUCTS, COMBINATION 
 * WITH ANY OTHER PRODUCTS, OR USE IN ANY APPLICATION. 
 *
 * IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
 * INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND 
 * WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS 
 * BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE 
 * FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS 
 * IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF 
 * ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *
 * MICROCHIP PROVIDES THIS SOFTWARE CONDITIONALLY UPON YOUR ACCEPTANCE OF THESE 
 * TERMS. 
 */

/* 
 * File:   
 * Author: 
 * Comments:
 * Revision history: 
 */

// This is a guard condition so that contents of this file are not included
// more than once.  
#ifndef XC_HEADER_TEMPLATE_H
#define	XC_HEADER_TEMPLATE_H


#ifdef	__cplusplus
extern "C" {
#endif /* __cplusplus */

    // TODO If C++ is being used, regular C code needs function names to have C 
    // linkage so the functions can be used by the c code. 

#ifdef	__cplusplus
}
#endif /* __cplusplus */

#ifndef F_CPU
#define F_CPU 24000000
#endif


#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "list.h"
#include "croutine.h"
#include "semphr.h"
#include "timers.h"
#include "limits.h"
#include "portable.h"

#include "protected_io.h"
#include "ccp.h"

#include <avr/wdt.h> 


#include "frtos-io.h"
#include "xprintf.h"
#include "xgetc.h"
#include "i2c.h"
#include "eeprom.h"
#include "rtc79410.h"
#include "nvm.h"
#include "led.h"
#include "pines.h"
#include "drv8814.h"
#include "steppers.h"
#include "valves.h"
#include "counters.h"

#define FW_REV "1.0.1"
#define FW_DATE "@ 20230818"
#define HW_MODELO "DPDPWR FRTOS R001 HW:AVR128DA64"
#define FRTOS_VERSION "FW:FreeRTOS V202111.00"
#define FW_TYPE "DPDV1"

#define SYSMAINCLK 24

#define tkCtl_TASK_PRIORITY	 	( tskIDLE_PRIORITY + 1 )
#define tkCmd_TASK_PRIORITY 	( tskIDLE_PRIORITY + 1 )
#define tkModbus_TASK_PRIORITY 	( tskIDLE_PRIORITY + 1 )

#define tkCtl_STACK_SIZE		384
#define tkCmd_STACK_SIZE		384
#define tkModbus_STACK_SIZE		384

StaticTask_t tkCtl_Buffer_Ptr;
StackType_t tkCtl_Buffer [tkCtl_STACK_SIZE];

StaticTask_t tkCmd_Buffer_Ptr;
StackType_t tkCmd_Buffer [tkCmd_STACK_SIZE];

StaticTask_t tkModbus_Buffer_Ptr;
StackType_t tkModbus_Buffer [tkModbus_STACK_SIZE];

SemaphoreHandle_t sem_SYSVars;
StaticSemaphore_t SYSVARS_xMutexBuffer;
#define MSTOTAKESYSVARSSEMPH ((  TickType_t ) 10 )

TaskHandle_t xHandle_tkCtl, xHandle_tkCmd, xHandle_tkModbus;

void tkCtl(void * pvParameters);
void tkCmd(void * pvParameters);
void tkModbus(void * pvParameters);

void system_init();
void reset(void);

void kick_wdt( uint8_t bit_pos);

void config_default(void);
bool config_debug( char *tipo, char *valor);
bool save_config_in_NVM(void);
bool load_config_from_NVM(void);

void save_config(void);
bool load_config(void);
void config_default(void);

bool starting_flag;


struct {   
    bool debug;
} systemVars;

struct {
    counter_t CNTCB[MAX_COUNTERS];
    t_stepper STEPPERLIST[NRO_STEPPERS];
    t_valves VALVELIST[NRO_VALVES];
    uint8_t modbus_address;
    uint8_t checksum;
} systemConf;

uint8_t sys_watchdog;

#define CMD_WDG_bp    0
#define MBUS_WDG_bp   1

#define WDG_bm 0x03

#define WDG_INIT() ( sys_watchdog = WDG_bm )

#endif	/* XC_HEADER_TEMPLATE_H */

