/*
 * ________________________________________________________________________________________________________
 * Copyright (c) 2020 GebraBit Inc. All rights reserved.
 *
 * This software, related documentation and any modifications thereto (collectively “Software”) is subject
 * to GebraBit and its licensors' intellectual property rights under U.S. and international copyright
 * and other intellectual property rights laws. 
 *
 * GebraBit and its licensors retain all intellectual property and proprietary rights in and to the Software
 * and any use, reproduction, disclosure or distribution of the Software without an express license agreement
 * from GebraBit is strictly prohibited.
 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT 
 * NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT IN  
 * NO EVENT SHALL GebraBit BE LIABLE FOR ANY DIRECT, SPECIAL, INDIRECT, INCIDENTAL, OR CONSEQUENTIAL DAMAGES, 
 * OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT,
 * NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE
 * OF THE SOFTWARE.
 * @Author       	: Mehrdad Zeinali
 * ________________________________________________________________________________________________________
 */
#ifndef	__HTU2XD_H__
#define	__HTU2XD_H__
#include "main.h"
#include "stdint.h"
#include "stdlib.h"
#include "stdio.h"
#include "string.h"
#include "math.h"
#include "i2c.h"
/************************************************
 *              USER REGISTER MAP               *
 ***********************************************/ 
#define HTU2XD_I2C		              									&hi2c1	
#define HTU2XD_ADDRESS 																0x40
#define HTU2XD_WRITE_ADDRESS 			  									((HTU2XD_ADDRESS<<1)|0)
#define HTU2XD_READ_ADDRESS 													((HTU2XD_ADDRESS<<1)|1)
#define HTU2XD_WRITE_USER_REGISTER_CMD						    0xE6
#define HTU2XD_READ_USER_REGISTER_CMD							    0xE7
#define HTU2XD_RESET_CMD													    0xFE
#define HTU2XD_TRIGGER_TEMPERATURE_MEASUREMENT_CMD		0xE3  ///Hold Master
#define HTU2XD_TRIGGER_HUMIDITY_MEASUREMENT_CMD		    0xE5  ///Hold Master
#define HTU2XD_USER_REGISTER_RESOLUTION_BIT_MASK      0x81
// Processing constants
#define HTU2XD_TEMPERATURE_COEFFICIENT							(float)(-0.15)
#define HTU2XD_CONSTANT_A														(float)(8.1332)
#define HTU2XD_CONSTANT_B														(float)(1762.39)
#define HTU2XD_CONSTANT_C														(float)(235.66)
// Coefficients for temperature computation
#define TEMPERATURE_COEFF_MUL												(175.72)
#define TEMPERATURE_COEFF_ADD												(-46.85)
// Coefficients for relative humidity computation
#define HUMIDITY_COEFF_MUL													(125)
#define HUMIDITY_COEFF_ADD													(-6)
/*----------------------------------------------*
 *           USER REGISTER MAP End              *
 *----------------------------------------------*/ 
 /************************************************
 *         MSB Bit Start Location Begin         *
 ***********************************************/ 
#define START_MSB_BIT_AT_0                    0
#define START_MSB_BIT_AT_1                    1
#define START_MSB_BIT_AT_2                    2
#define START_MSB_BIT_AT_3                    3
#define START_MSB_BIT_AT_4                    4
#define START_MSB_BIT_AT_5                    5
#define START_MSB_BIT_AT_6                    6
#define START_MSB_BIT_AT_7                    7
/*----------------------------------------------*
 *        MSB Bit Start Location End            *
 *----------------------------------------------*/ 
/************************************************
 *          Bit Field Length Begin              *
 ***********************************************/ 
#define BIT_LENGTH_1                          1
#define BIT_LENGTH_2                          2
#define BIT_LENGTH_3                          3
#define BIT_LENGTH_4                          4
#define BIT_LENGTH_5                          5
#define BIT_LENGTH_6                          6
#define BIT_LENGTH_7                          7
#define BIT_LENGTH_8                          8
/*----------------------------------------------*
 *          Bit Field Length End                *
 *----------------------------------------------*/
#define REGISTER_DATA_BUFFER_SIZE             3
#define FIFO_DATA_BUFFER_SIZE                 100
#define COLOR_LUX_RESOLUTION                  0.015f
/**************************************************
 *     Values For Disable And Enable Functions    *
 **************************************************/ 
typedef enum Ability
{  
	Disable = 0     ,                      
	Enable     
}HTU2XD_Ability;    
/*************************************************
 *           Values For Battery Status           *
 **************************************************/
typedef enum Battery_Status
{
 HTU2XD_BATTERY_VDD_OK,
 HTU2XD_BATTERY_VDD_LOW
}HTU2XD_Battery_Status;
/*************************************************
 *                 Values For OTP   			       *
 **************************************************/
typedef enum OTP
{  
	OTP_DISABLE = 1     ,                      
	OTP_ENABLE  = 0    
}HTU2XD_OTP; 
/*************************************************
 *       Values For Measurement Resolution       *
 **************************************************/ 
typedef enum Measurement_Resolution
{
	HTU2XD_HUMIDITY_12BIT_TEMPERATURE_14BIT = 0x00 ,
	HTU2XD_HUMIDITY_8BIT_TEMPERATURE_12BIT  = 0x01 ,
	HTU2XD_HUMIDITY_10BIT_TEMPERATURE_13BIT = 0x80 ,
	HTU2XD_HUMIDITY_11BIT_TEMPERATURE_11BIT = 0x81
}HTU2XD_Measurement_Resolution;
/*************************************************
 *     Values For Humidity Conversion Time       *
 **************************************************/ 
typedef enum Humidity_Conversion_Time
{
	HUMIDITY_12BIT_MEASUREMENT_TIME = 16 ,
	HUMIDITY_11BIT_MEASUREMENT_TIME = 8 ,
	HUMIDITY_10BIT_MEASUREMENT_TIME = 5 ,
	HUMIDITY_8BIT_MEASUREMENT_TIME  = 3
}HTU2XD_Humidity_Conversion_Time;
/*************************************************
 *   Values For Temperature Conversion Time      *
 **************************************************/ 
typedef enum Temperature_Conversion_Time
{
	TEMPERATURE_14BIT_MEASUREMENT_TIME = 50 ,
	TEMPERATURE_13BIT_MEASUREMENT_TIME = 25 ,
	TEMPERATURE_12BIT_MEASUREMENT_TIME = 13 ,
	TEMPERATURE_11BIT_MEASUREMENT_TIME = 7
}HTU2XD_Temperature_Conversion_Time;
/*************************************************
 *           Values For Reset Process             *
 **************************************************/ 
typedef enum 
{  
	FAILED = 0     ,                      
	DONE     
}HTU2XD_Reset_Status;
/*************************************************
 *           Values For CRC Status               *
 **************************************************/ 
typedef enum CRC_Status 
{  
	CRC_ERROR = 0     ,                      
	CRC_OK     
}HTU2XD_CRC_Status;
 /*************************************************
 *  Defining HTU2XD Register & Data As Struct   *
 **************************************************/
typedef	struct HTU2XD
{
	  uint8_t                       	   Register_Cache;
	  HTU2XD_Reset_Status					       RESET;
	  HTU2XD_Battery_Status              BATTERY_VDD; 
	  HTU2XD_OTP										     OTP;
	  HTU2XD_Ability								     ON_CHIP_HEATER;
	  HTU2XD_Measurement_Resolution      MEASUREMENT_RESOLUTION;
	  HTU2XD_Humidity_Conversion_Time    HUMIDITY_MEASUREMENT_TIME;
	  HTU2XD_Temperature_Conversion_Time TEMPERATURE_MEASUREMENT_TIME;  
		uint8_t                            ADC_TEMPERATURE[REGISTER_DATA_BUFFER_SIZE];
		uint16_t                           ADC_TEMPERATURE_DATA;
		uint8_t                            ADC_HUMIDITY[REGISTER_DATA_BUFFER_SIZE];
		uint16_t													 ADC_HUMIDITY_DATA;
		uint8_t 													 HTU2XD_CRC;
		HTU2XD_CRC_Status 								 CRC_CHECK;
    float 														 TEMPERATURE;
		float 														 HUMIDITY;
	  float 														 COMPANSATED_HUMIDITY;
//	  double														 PARTIAL_PRESSURE;
//		double 														 DEW_POINT;
}GebraBit_HTU2XD;
/*
:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/********************************************************
 *  Declare Read&Write HTU2XD Register Values Functions *
 ********************************************************/
extern void GB_HTU2XD_Read_User_Register(uint8_t *data)		;
extern void GB_HTU2XD_Burst_Read(uint8_t regAddr,  uint8_t *data, uint16_t byteQuantity);
extern void GB_HTU2XD_Read_User_Register_Bits ( uint8_t start_bit, uint8_t len, uint8_t* data);	
extern void GB_HTU2XD_Write_User_Register(uint8_t data)	;
extern void GB_HTU2XD_Burst_Write(uint8_t regAddr,  uint8_t *data, uint16_t byteQuantity)								;
extern void GB_HTU2XD_Write_User_Register_Bits( uint8_t start_bit, uint8_t len, uint8_t data);
/********************************************************
 *       Declare MS5611 Configuration Functions         *
 ********************************************************/
extern void GB_HTU2XD_Soft_Reset ( GebraBit_HTU2XD * HTU2XD )  ;
extern void GB_HTU2XD_Check_Battery_Voltage_VDD ( GebraBit_HTU2XD * HTU2XD  ) ;
extern void GB_HTU2XD_On_Chip_Heater ( GebraBit_HTU2XD * HTU2XD , HTU2XD_Ability heater )  ;
extern void GB_HTU2XD_Read_On_Chip_Heater_Status ( GebraBit_HTU2XD * HTU2XD , HTU2XD_Ability heater )    ;
extern void GB_HTU2XD_OTP ( GebraBit_HTU2XD * HTU2XD , HTU2XD_OTP otp )  ;
extern void GB_HTU2XD_Read_OTP ( GebraBit_HTU2XD * HTU2XD , HTU2XD_OTP otp ) ;
extern void GB_HTU2XD_Measurement_Resolution ( GebraBit_HTU2XD * HTU2XD , HTU2XD_Measurement_Resolution res ) ;
extern void GB_HTU2XD_Read_Measurement_Resolution ( GebraBit_HTU2XD * HTU2XD   )   ;
extern void GB_HTU2XD_CRC_Check( GebraBit_HTU2XD * HTU2XD , uint16_t value, uint8_t crc)  ;
extern void GB_HTU2XD_ADC_Temperature_Raw_Data ( GebraBit_HTU2XD * HTU2XD )  ;
extern void GB_HTU2XD_ADC_Humidity_Raw_Data ( GebraBit_HTU2XD * HTU2XD )   ;
extern void GB_HTU2XD_initialize( GebraBit_HTU2XD * HTU2XD )  ;
extern void GB_HTU2XD_Configuration(GebraBit_HTU2XD * HTU2XD)  ;
extern void GB_HTU2XD_Get_Data(GebraBit_HTU2XD * HTU2XD);
#endif
