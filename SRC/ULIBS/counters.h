/* 
 * File:   counters.h
 * Author: pablo
 *
 * Created on 15 de septiembre de 2022, 04:46 PM
 */

#ifndef COUNTERS_H
#define	COUNTERS_H

#ifdef	__cplusplus
extern "C" {
#endif

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
    
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
    
#include "xprintf.h"
#include "ringBuffer.h"
    
#define CNT0_PORT	PORTC
#define CNT0_PIN    1   
#define CNT0_PIN_bm	PIN1_bm
#define CNT0_PIN_bp	PIN1_bp
    
#define CNT1_PORT	PORTC
#define CNT1_PIN    2
#define CNT1_PIN_bm	PIN2_bm
#define CNT1_PIN_bp	PIN2_bp

#define CNT2_PORT	PORTC
#define CNT2_PIN    3
#define CNT2_PIN_bm	PIN3_bm
#define CNT2_PIN_bp	PIN3_bp
    
// Los CNTx son inputs
#define CNT0_CONFIG()    ( CNT0_PORT.DIR &= ~CNT0_PIN_bm )
#define CNT1_CONFIG()    ( CNT1_PORT.DIR &= ~CNT1_PIN_bm )
#define CNT2_CONFIG()    ( CNT2_PORT.DIR &= ~CNT2_PIN_bm )

typedef enum { P_OUTSIDE=0, P_CHECKING, P_INSIDE, P_FILTER } pulse_t;

#define MAX_COUNTERS    3

typedef struct {
    uint8_t nivel_anterior;
    uint8_t nivel;
    uint16_t count;
} counter_t;

void COUNTER_INIT_OUTOUTOFRTOS(void);
void counter_init(void);
void counter_clear(uint8_t i);
uint16_t counter_read(uint8_t i);
uint8_t counter_read_pin(uint8_t i);
uint8_t COUNTER_cmdread_pin( char *s_id);


#ifdef	__cplusplus
}
#endif

#endif	/* COUNTERS_H */

