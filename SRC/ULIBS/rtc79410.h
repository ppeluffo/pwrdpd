/*------------------------------------------------------------------------------------
 * l_rtc.h
 * Autor: Pablo Peluffo @ 2015
 * Basado en Proycon AVRLIB de Pascal Stang.
 *
 * Son funciones que impelementan la API de acceso al RTC del sistema con FRTOS.
 *
 *
*/

// --------------------------------------------------------------------------------
// LIBRERIA PARA EL RTC M79410 USADO EN LOS DATALOGGER SERIE SPX.
// --------------------------------------------------------------------------------

#ifndef AVRLIBFRTOS_RTC_SP5KFRTOS_H_
#define AVRLIBFRTOS_RTC_SP5KFRTOS_H_

#include "frtos-io.h"
#include "stdint.h"
#include "i2c.h"
#include "xprintf.h"
#include "stdbool.h"

#define FORMAT_LONG true
#define FORMAT_SHORT false

//--------------------------------------------------------------------------------
// API START

typedef struct
{
	// Tamanio: 7 byte.
	// time of day
	uint8_t sec;
	uint8_t min;
	uint8_t hour;
	// date
	uint8_t day;
	uint8_t month;
	uint8_t year;

} RtcTimeType_t;

int16_t RTC_read( uint16_t rdAddress, char *data, uint8_t length );
int16_t RTC_write( uint16_t wrAddress, char *data, uint8_t length );

int16_t RTCSRAM_test_write( char *addr, char *str );
int16_t RTCSRAM_test_read( char *addr, char *size );

//------------------------------------------------------------------------------------
// API publica
void RTC_init(void);
bool RTC_read_dtime(RtcTimeType_t *rtc);
bool RTC_write_dtime(RtcTimeType_t *rtc);
bool RTC_write_time( char *stime );
void RTC_read_time(  bool format_long );
char *RTC_logprint( bool format_long );
bool RTC_has_drift(RtcTimeType_t *rtc_new, uint16_t max_drift );

// API END
//--------------------------------------------------------------------------------

void RTC_rtc2str(char *str, RtcTimeType_t *rtc, bool format_long );
bool RTC_str2rtc(char *str, RtcTimeType_t *rtc);

// Direccion del bus I2C donde esta el RTC79410
#define RTC79410_DEVADDR		   	0xDE

// Direcciones de registros

#define RTC79410_RTCSEC			0x00
#define RTC79410_RTCMIN			0x01
#define RTC79410_RTCHOUR		0x02
#define RTC79410_RTCWKDAY		0x03
#define RTC79410_RTCDATE		0x04
#define RTC79410_RTCMTH			0x05
#define RTC79410_RTCYEAR		0x06
#define RTC79410_CONTROL		0x07

// Direccion base donde comienza la SRA
#define RTC79410_ALM0SEC		0x0A
#define RTC79410_ALM0MIN		0x0B
#define RTC79410_ALM0HOUR		0x0C
#define RTC79410_ALM0WKDAY		0x0D
#define RTC79410_ALM0DATE		0x0E
#define RTC79410_ALM0MTH		0x0F

#define RTC79410_SRAM_INIT			0x20
#define FAT_ADDRESS					0x20

#endif /* AVRLIBFRTOS_RTC_SP5KFRTOS_H_ */
