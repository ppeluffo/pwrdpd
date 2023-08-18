/* 
 * File:   modbus.h
 * Author: pablo
 *
 * Created on 6 de marzo de 2023, 09:35 AM
 */

#ifndef MODBUS_H
#define	MODBUS_H

#ifdef	__cplusplus
extern "C" {
#endif

#include "frtos-io.h"
#include "stdint.h"
#include "xprintf.h"
#include "linearBuffer.h"
    
#define RS485A_RX_BUFFER_SIZE 64
char modbus_rx_buffer[RS485A_RX_BUFFER_SIZE];
lBuffer_s modbusRXbuffer;

#define RS485A_TX_BUFFER_SIZE 64
char modbus_tx_buffer[RS485A_TX_BUFFER_SIZE];

/*
 * En modbus leemos de a 1 canal, no bloques !!
 * Los canales pueden ser holding_registers ( 0x03 ) o input_registers (0x04).
 * Cada registro son 2 bytes por lo que siempre leemos 2 x N.
 * N: nro_recds.
 * En tipo ( 'F','I' ) indicamos como los interpretamos.
 * En divisor_10 indicamos el factor por el que multiplicamos para tener la magnitud.
 *
 */

typedef struct {
    uint8_t address;
    uint8_t fcode;
    uint16_t reg_address;
    uint16_t nro_registers;
    uint16_t reg_value;
    uint16_t rcv_cks;

} modbus_rx_block_t;

modbus_rx_block_t MBRXCBK;

void modbus_init_rx_buffer(void);
void  modbus_put_rx_buffer( char c);
uint16_t modbus_get_rxPtr(void);
void modbus_process_rx_frame(void);
void modbus_flush_rx_frame(void);
char *modbus_get_rx_buffer(void);
void modbus_print_rx_frame(void);
void modbus_process_fcode03(void);
void modbus_process_fcode06(void);

void modbus_config_localaddress( char *s_addr);
uint8_t modbus_get_localaddress( void );
void modbus_default_config( void );

void modbus_set_local_config( uint8_t modbus_address );
void modbus_get_local_config( uint8_t *modbus_address );

#ifdef	__cplusplus
}
#endif

#endif	/* MODBUS_H */

