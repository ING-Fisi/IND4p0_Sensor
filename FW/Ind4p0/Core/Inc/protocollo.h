/*
 * protocollo.h
 *
 *  Created on: Apr 3, 2023
 *      Author: clott
 */

/******************************************************************************
 *  VERSIONE PER BUMPS&SHAKES INTERFACE
 ******************************************************************************/
#ifndef PROTOCOLLO_H_
#define PROTOCOLLO_H_

#include "stdint.h"

#define ETX_TAG 0xAF
#define STX_TAG 0xBE

#define ETX_TAG_FFT 0xABCD
#define STX_TAG_FFT 0xDEFB

#define AUDIBLE_RANGE_START 0
#define AUDIBLE_RANGE_END   500 /* very optimistic */
#define SAMPLE_FREQ 512
#define SAMPLES                    512            // 256 real party and 256 imaginary parts */
#define FFT_SIZE                SAMPLES / 2        // FFT size is always the same size as we have samples, so 256 in



#pragma pack(1)
typedef enum uart_command
{
    bump_event_cmd,
    shake_event_cmd,
	shake_monitor_cmd,
    reboot_cmd,
    ack_cmd,
    nack_cmd,
    setConfiguration_cmd,
    saveConfiguration_cmd,
    getConfiguration_cmd,
	getRawData_cmd
    //alarm_disabled_cmd

}uart_command;
#pragma pack()



typedef enum sensor_configuration_bytes
{
    row_data = 0,
    conf_limit_value = 0, //1,
    end_of_byte = 2 //10

}sensor_configuration_bytes;


#pragma pack(1)
typedef struct protocollo_uart
{
    uint8_t stx;
    uint8_t cmd;
    uint16_t payload[10];
    uint16_t crc;
    uint8_t etx;

}protocollo_uart;
#pragma pack()


#pragma pack(1)
typedef struct protocollo_uart_fft
{
    uint16_t stx;
    float fft[FFT_SIZE];
    uint16_t etx;

}protocollo_uart_fft;
#pragma pack()


#pragma pack(1)
typedef struct protocollo_uart_sample
{
    uint16_t stx;
    float sample[SAMPLES];
    uint16_t etx;

}protocollo_uart_sample;
#pragma pack()



#endif /* PROTOCOLLO_H_ */
/******************************************************************************/




/******************************************************************************
 *  VERSIONE PER SERIAL PLOTTER
 ******************************************************************************/
//#ifndef PROTOCOLLO_H_
//#define PROTOCOLLO_H_
//
//#include "stdint.h"
//
//#define ETX_TAG 0xAF
//#define STX_TAG 0xBE
//
//#define ETX_TAG_FFT 0xABCD
//#define STX_TAG_FFT 0xDEFB
//
//
//
//#pragma pack(1)
//typedef enum uart_command
//{
//    alarm_event,
//    set_edge_level,
//    reboot,
//    ack_cmd,
//    nack_cmd
//
//}uart_command;
//#pragma pack()
//
//
//
//typedef enum piezo_configuration_bytes
//{
//    row_data = 0,
//    conf_limit_value = 1,
//    end_of_byte = 10
//
//}piezo_configuration_bytes;
//
//
//#pragma pack(1)
//typedef struct protocollo_uart
//{
//    uint8_t stx;
//    uint8_t cmd;
//    uint16_t piezo_conf[end_of_byte];
//    uint16_t crc;
//    uint8_t etx;
//
//}protocollo_uart;
//#pragma pack()
//
//
//#pragma pack(1)
//typedef struct protocollo_uart_fft
//{
//    uint16_t stx;
//    float32_t fft[FFT_SIZE];
//    uint16_t etx;
//
//}protocollo_uart_fft;
//#pragma pack()
//
//#pragma pack(1)
//typedef struct protocollo_uart_sample
//{
//    uint16_t stx;
//    float32_t sample[SAMPLES];
//    uint16_t etx;
//
//}protocollo_uart_sample;
//#pragma pack()
//
//
//
//#endif /* PROTOCOLLO_H_ */
/******************************************************************************/
