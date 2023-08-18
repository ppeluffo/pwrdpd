
#include "modbus.h"
#include "math.h"


uint8_t MODBUS_LOCAL_ADDRESS;

uint16_t pv_modbus_CRC16( uint8_t *msg, uint8_t msg_size );

//------------------------------------------------------------------------------
void modbus_init_rx_buffer(void)
{
    // Creamos el RX buffer.
    lBchar_CreateStatic ( &modbusRXbuffer, modbus_rx_buffer, RS485A_RX_BUFFER_SIZE );
}
//------------------------------------------------------------------------------
void  modbus_put_rx_buffer( char c)
{
    // Almacenamos el byte en el rxbuffer.
    
    lBchar_Put( &modbusRXbuffer, c);
}
//------------------------------------------------------------------------------
uint16_t modbus_get_rxPtr(void)
{
    return( lBchar_GetCount(&modbusRXbuffer) );
}
//------------------------------------------------------------------------------
char *modbus_get_rx_buffer(void)
{
    return ( lBchar_get_buffer(&modbusRXbuffer) );
}
//------------------------------------------------------------------------------
void modbus_process_rx_frame(void)
{
        
    modbus_print_rx_frame();
    
    // Determino la direccion del mensaje
    MBRXCBK.address = modbus_rx_buffer[0];
    if ( MBRXCBK.address != 0 ) {
        // Mensaje destinado a otro.
        return;
    }
    
    MBRXCBK.fcode = modbus_rx_buffer[1];
    switch (MBRXCBK.fcode) {
        case 0x03:
            // Read holding register.
            modbus_process_fcode03();
            break;
        case 0x06:
            // Write single register
            modbus_process_fcode06();
            break;
        default:
            // No implementamos otro método
            // Envio mensaje de error
            
            return;
    }  
}
//------------------------------------------------------------------------------
void modbus_flush_rx_frame(void)
{
    lBchar_Flush( &modbusRXbuffer );
}
//------------------------------------------------------------------------------
void modbus_print_rx_frame(void)
{
    
    
uint16_t buffer_length;
uint8_t i;
char *ptr;

    buffer_length = modbus_get_rxPtr();
    ptr = modbus_get_rx_buffer();
    xprintf_P( PSTR("MODBUS: RX (len=%d):"),  buffer_length );
    
	for ( i = 0 ; i < buffer_length; i++ ) {
		xprintf_P( PSTR("[0x%02X]"), *ptr);
        ptr++;
	}
    
	xprintf_P( PSTR("\r\n"));
    
}
//------------------------------------------------------------------------------
void modbus_process_fcode03(void)
{
    // FCODE=0x03, Read holding register.
    
uint16_t crc_calc;


    MBRXCBK.reg_address = modbus_rx_buffer[2] << 8 | modbus_rx_buffer[3];
    MBRXCBK.nro_registers = modbus_rx_buffer[4] << 8 | modbus_rx_buffer[5];
    MBRXCBK.rcv_cks = modbus_rx_buffer[6] << 8 | modbus_rx_buffer[7];

    // Compruebo el checksum
	crc_calc = pv_modbus_CRC16( (uint8_t *)&modbus_rx_buffer[0], 6 );
	if ( crc_calc != MBRXCBK.rcv_cks) {
		xprintf_P( PSTR("MODBUS: RX CRC ERROR: rx[0x%02x], calc[0x%02x]\r\n\0"), MBRXCBK.rcv_cks, crc_calc);
		// De acuerdo al protocolo se ignora el frame recibido con errores CRC
		// Envio mensaje de error.
        return;
	}
    
    // Proceso el request
}
//------------------------------------------------------------------------------
void modbus_process_fcode06(void)
{
    // FCODE=0x06, Write single register.
    
uint16_t crc_calc;

    MBRXCBK.reg_address = modbus_rx_buffer[2] << 8 | modbus_rx_buffer[3];
    MBRXCBK.reg_value = modbus_rx_buffer[4] << 8 | modbus_rx_buffer[5];
    MBRXCBK.rcv_cks = modbus_rx_buffer[6] << 8 | modbus_rx_buffer[7];

    // Compruebo el checksum
	crc_calc = pv_modbus_CRC16( (uint8_t *)&modbus_rx_buffer[0], 6 );
	if ( crc_calc != MBRXCBK.rcv_cks) {
		xprintf_P( PSTR("MODBUS: RX CRC ERROR: rx[0x%02x], calc[0x%02x]\r\n\0"), MBRXCBK.rcv_cks, crc_calc);
		// De acuerdo al protocolo se ignora el frame recibido con errores CRC
		// Envio mensaje de error.
        return;
	}
    
    // Proceso el request
}
//------------------------------------------------------------------------------
uint16_t pv_modbus_CRC16( uint8_t *msg, uint8_t msg_size )
{

uint16_t crc = 0xFFFF;
uint16_t pos;
uint16_t data;
int i;

	for ( pos = 0; pos < msg_size; pos++) {
		data = (uint16_t)*msg++;
		crc ^= data;          // XOR byte into least sig. byte of crc

		for (i = 8; i != 0; i--) {    // Loop over each bit
			if ((crc & 0x0001) != 0) {      // If the LSB is set
				crc >>= 1;                    // Shift right and XOR 0xA001
				crc ^= 0xA001;
			} else {                            // Else LSB is not set
				crc >>= 1;                    // Just shift right
			}
		}
	}
	// Note, this number has low and high bytes swapped, so use it accordingly (or swap bytes)
	return crc;
}
//------------------------------------------------------------------------------
void modbus_config_localaddress( char *s_addr)
{
    MODBUS_LOCAL_ADDRESS = atoi(s_addr);    
}
//------------------------------------------------------------------------------
uint8_t modbus_get_localaddress( void )
{
    return( MODBUS_LOCAL_ADDRESS);
}
//------------------------------------------------------------------------------
void modbus_default_config( void )
{
    MODBUS_LOCAL_ADDRESS=0x02;
}
//------------------------------------------------------------------------------
void modbus_set_local_config( uint8_t modbus_address )
{
    MODBUS_LOCAL_ADDRESS = modbus_address;
} 
//------------------------------------------------------------------------------ 
void modbus_get_local_config( uint8_t *modbus_address )
{
    *modbus_address = MODBUS_LOCAL_ADDRESS;
}   
//------------------------------------------------------------------------------    
