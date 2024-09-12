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
#include "GebraBit_HTU2XD.h"

extern I2C_HandleTypeDef 		hi2c1;
/*========================================================================================================================================= 
 * @brief     Read  data from User Register
 * @param     data    Pointer to Variable that  data is saved .
 * @return    None
 ========================================================================================================================================*/
void GB_HTU2XD_Read_User_Register(uint8_t *data)																			
{
	HAL_I2C_Mem_Read(HTU2XD_I2C,HTU2XD_READ_ADDRESS,HTU2XD_READ_USER_REGISTER_CMD,I2C_MEMADD_SIZE_8BIT,data,1,200);
}
/*========================================================================================================================================= 
 * @brief     Read multiple data from first spacial register address.
 * @param     regAddr First Register Address of HTU2XD that reading multiple data start from this address
 * @param     data    Pointer to Variable that multiple data is saved .
 * @param     byteQuantity Quantity of data that we want to read .
 * @return    None
 ========================================================================================================================================*/
void GB_HTU2XD_Burst_Read(uint8_t regAddr,  uint8_t *data, uint16_t byteQuantity)																			
{
	HAL_I2C_Mem_Read(HTU2XD_I2C,HTU2XD_READ_ADDRESS,regAddr,1,data,byteQuantity,200);
}
/*========================================================================================================================================= 
 * @brief     Read data from spacial bits of a register.
 * @param     start_bit   Start Bit location .(0 to 7)
 * @param     len         Quantity of Bits want to read(1 to 8) 
 * @param     data        Pointer to Variable that register Bits value is saved .
 * @return    None
 ========================================================================================================================================*/
void GB_HTU2XD_Read_User_Register_Bits ( uint8_t start_bit, uint8_t len, uint8_t* data)
{
	uint8_t tempData = 0;
	GB_HTU2XD_Read_User_Register(&tempData);
	uint8_t mask = ((1 << len) - 1) << (start_bit - len + 1); //formula for making a broom of 1&0 for gathering desired bits
	tempData &= mask; // zero all non-important bits in data
	tempData >>= (start_bit - len + 1); //shift data to zero position
	*data = tempData;
}
/*========================================================================================================================================= 
 * @brief     Write  data To User Register
 * @param     data    Pointer to Variable that  data is saved .
 * @return    None
 ========================================================================================================================================*/
void GB_HTU2XD_Write_User_Register(uint8_t data)																			/*		Read Burst Data From Register			*/
{
	HAL_I2C_Mem_Write(HTU2XD_I2C,HTU2XD_WRITE_ADDRESS,HTU2XD_WRITE_USER_REGISTER_CMD,I2C_MEMADD_SIZE_8BIT,&data,1,200);
}
/*========================================================================================================================================= 
 * @brief     Write multiple data to  spacial register address.
 * @param     regAddr First Register Address of HTU2XD that reading multiple data start from this address
 * @param     data    Pointer to Variable that multiple data is saved .
 * @param     byteQuantity Quantity of data that we want to read .
 * @return    None
 ========================================================================================================================================*/
void GB_HTU2XD_Burst_Write(uint8_t regAddr,  uint8_t *data, uint16_t byteQuantity)																			/*		Read Burst Data From Register			*/
{
	HAL_I2C_Mem_Write(HTU2XD_I2C,HTU2XD_WRITE_ADDRESS,regAddr,1,data,byteQuantity,200);
}
/*=========================================================================================================================================
 * @brief     Write data to spacial bits of a register.
 * @param     start_bit   Start Bit location .(0 to 7)
 * @param     len         Quantity of Bits want to read(1 to 8) 
 * @param     data        Pointer to Variable that register Bits value is saved .
 * @return    None
 ========================================================================================================================================*/
void GB_HTU2XD_Write_User_Register_Bits( uint8_t start_bit, uint8_t len, uint8_t data) 
{
	uint8_t tempData = 0;
	GB_HTU2XD_Read_User_Register(&tempData);	
	uint8_t mask = ((1 << len) - 1) << (start_bit - len + 1);
	data <<= (start_bit - len + 1); // shift data into correct position
	data &= mask; // zero all non-important bits in data
	tempData &= ~(mask); // zero all important bits in existing byte
	tempData |= data; // combine data with existing byte
	GB_HTU2XD_Write_User_Register(tempData);
}
/*=========================================================================================================================================
 * @brief     Reset HTU2XD
 * @param     HTU2XD   HTU2XD Struct
 * @return    Nothing
 ========================================================================================================================================*/
void GB_HTU2XD_Soft_Reset ( GebraBit_HTU2XD * HTU2XD )  
{
	uint8_t TBuff[1];
	TBuff[0]=HTU2XD_RESET_CMD;
	HAL_I2C_Master_Transmit(HTU2XD_I2C,HTU2XD_WRITE_ADDRESS,TBuff,1,100);
	HAL_Delay(20);
	HTU2XD->RESET = DONE ;
}
/*=========================================================================================================================================
 * @brief     Check Battery Voltage VDD
 * @param     HTU2XD   HTU2XD Struct BATTERY_VDD variable
 * @return    Nothing
 ========================================================================================================================================*/
void GB_HTU2XD_Check_Battery_Voltage_VDD ( GebraBit_HTU2XD * HTU2XD  ) 
{
 GB_HTU2XD_Read_User_Register_Bits(START_MSB_BIT_AT_6, BIT_LENGTH_1, &HTU2XD->BATTERY_VDD); 
}
/*=========================================================================================================================================
 * @brief     Enable Or Disable On Chip Heater
 * @param     HTU2XD   HTU2XD Struct ON_CHIP_HEATER variable
 * @param     heater        Value is from HTU2XD_Ability Enume
 * @return    Nothing
 ========================================================================================================================================*/
void GB_HTU2XD_On_Chip_Heater ( GebraBit_HTU2XD * HTU2XD , HTU2XD_Ability heater ) 
{
 GB_HTU2XD_Write_User_Register_Bits(START_MSB_BIT_AT_2, BIT_LENGTH_1, heater);
 HTU2XD->ON_CHIP_HEATER= heater ; 
}
/*=========================================================================================================================================
 * @brief     Check On Chip Heater Status
 * @param     HTU2XD   HTU2XD Struct ON_CHIP_HEATER variable
 * @return    Nothing
 ========================================================================================================================================*/
void GB_HTU2XD_Read_On_Chip_Heater_Status ( GebraBit_HTU2XD * HTU2XD , HTU2XD_Ability heater ) 
{
 GB_HTU2XD_Read_User_Register_Bits(START_MSB_BIT_AT_2, BIT_LENGTH_1, &HTU2XD->ON_CHIP_HEATER);
}
/*=========================================================================================================================================
 * @brief     Set OTP
 * @param     HTU2XD   HTU2XD Struct OTP variable
 * @param     otp        Value is from HTU2XD_OTP Enume
 * @return    Nothing
 ========================================================================================================================================*/
void GB_HTU2XD_OTP ( GebraBit_HTU2XD * HTU2XD , HTU2XD_OTP otp ) 
{
 GB_HTU2XD_Write_User_Register_Bits(START_MSB_BIT_AT_1, BIT_LENGTH_1, otp);
 HTU2XD->OTP= otp ; 
}
/*=========================================================================================================================================
 * @brief     Read OTP Status
 * @param     HTU2XD   HTU2XD Struct OTP variable
 * @return    Nothing
 ========================================================================================================================================*/
void GB_HTU2XD_Read_OTP ( GebraBit_HTU2XD * HTU2XD , HTU2XD_OTP otp ) 
{
 GB_HTU2XD_Read_User_Register_Bits(START_MSB_BIT_AT_1, BIT_LENGTH_1, &HTU2XD->OTP);
}
/*=========================================================================================================================================
 * @brief     Set Measurement Resolution
 * @param     HTU2XD   HTU2XD Struct  HUMIDITY_MEASUREMENT_TIME & TEMPERATURE_MEASUREMENT_TIME variable
 * @param     res        Value is from HTU2XD_Measurement_Resolution Enume
 * @return    Nothing
 ========================================================================================================================================*/
void GB_HTU2XD_Measurement_Resolution ( GebraBit_HTU2XD * HTU2XD , HTU2XD_Measurement_Resolution res ) 
{
	uint8_t register_value ;
	GB_HTU2XD_Read_User_Register(&register_value);
	register_value &= ~HTU2XD_USER_REGISTER_RESOLUTION_BIT_MASK;
	register_value |= res; 
	GB_HTU2XD_Write_User_Register(register_value);
  HTU2XD->MEASUREMENT_RESOLUTION = res ;
	 	switch(HTU2XD->MEASUREMENT_RESOLUTION)
	 {
	  case HTU2XD_HUMIDITY_12BIT_TEMPERATURE_14BIT:
		HTU2XD->HUMIDITY_MEASUREMENT_TIME = HUMIDITY_12BIT_MEASUREMENT_TIME ; 
		HTU2XD->TEMPERATURE_MEASUREMENT_TIME = TEMPERATURE_14BIT_MEASUREMENT_TIME ;
    break;
		case HTU2XD_HUMIDITY_8BIT_TEMPERATURE_12BIT:
		HTU2XD->HUMIDITY_MEASUREMENT_TIME = HUMIDITY_8BIT_MEASUREMENT_TIME ; 
		HTU2XD->TEMPERATURE_MEASUREMENT_TIME = TEMPERATURE_12BIT_MEASUREMENT_TIME ;
    break;	
		case HTU2XD_HUMIDITY_10BIT_TEMPERATURE_13BIT:
		HTU2XD->HUMIDITY_MEASUREMENT_TIME = HUMIDITY_10BIT_MEASUREMENT_TIME ; 
		HTU2XD->TEMPERATURE_MEASUREMENT_TIME = TEMPERATURE_13BIT_MEASUREMENT_TIME ;
    break;	
		case HTU2XD_HUMIDITY_11BIT_TEMPERATURE_11BIT:
		HTU2XD->HUMIDITY_MEASUREMENT_TIME = HUMIDITY_11BIT_MEASUREMENT_TIME ;  
		HTU2XD->TEMPERATURE_MEASUREMENT_TIME = TEMPERATURE_11BIT_MEASUREMENT_TIME ;
    break;	
	 }
}
/*=========================================================================================================================================
 * @brief     Read Measurement Resolution
 * @param     HTU2XD   HTU2XD Struct  HUMIDITY_MEASUREMENT_TIME & TEMPERATURE_MEASUREMENT_TIME variable
 * @return    Nothing
 ========================================================================================================================================*/
void GB_HTU2XD_Read_Measurement_Resolution ( GebraBit_HTU2XD * HTU2XD   ) 
{
	uint8_t register_value ;
	GB_HTU2XD_Read_User_Register(&register_value);
	register_value &= HTU2XD_USER_REGISTER_RESOLUTION_BIT_MASK;
  HTU2XD->MEASUREMENT_RESOLUTION = register_value ;
	switch(HTU2XD->MEASUREMENT_RESOLUTION)
	 {
	  case HTU2XD_HUMIDITY_12BIT_TEMPERATURE_14BIT:
		HTU2XD->HUMIDITY_MEASUREMENT_TIME = HUMIDITY_12BIT_MEASUREMENT_TIME ; 
		HTU2XD->TEMPERATURE_MEASUREMENT_TIME = TEMPERATURE_14BIT_MEASUREMENT_TIME ;
    break;
		case HTU2XD_HUMIDITY_8BIT_TEMPERATURE_12BIT:
		HTU2XD->HUMIDITY_MEASUREMENT_TIME = HUMIDITY_8BIT_MEASUREMENT_TIME ; 
		HTU2XD->TEMPERATURE_MEASUREMENT_TIME = TEMPERATURE_12BIT_MEASUREMENT_TIME ;
    break;	
		case HTU2XD_HUMIDITY_10BIT_TEMPERATURE_13BIT:
		HTU2XD->HUMIDITY_MEASUREMENT_TIME = HUMIDITY_10BIT_MEASUREMENT_TIME ; 
		HTU2XD->TEMPERATURE_MEASUREMENT_TIME = TEMPERATURE_13BIT_MEASUREMENT_TIME ;
    break;	
		case HTU2XD_HUMIDITY_11BIT_TEMPERATURE_11BIT:
		HTU2XD->HUMIDITY_MEASUREMENT_TIME = HUMIDITY_11BIT_MEASUREMENT_TIME ;  
		HTU2XD->TEMPERATURE_MEASUREMENT_TIME = TEMPERATURE_11BIT_MEASUREMENT_TIME ;
    break;	
	 }
}
/*=========================================================================================================================================
 * @brief     Check CRC
 * @param     HTU2XD   HTU2XD Struct  CRC_CHECK variable
 * @param     value        Value that must be compare with crc
 * @param     crc          CRC Value
 * @return    Nothing
 ========================================================================================================================================*/

void GB_HTU2XD_CRC_Check( GebraBit_HTU2XD * HTU2XD , uint16_t value, uint8_t crc)
{
	uint32_t polynom = 0x988000; // x^8 + x^5 + x^4 + 1
	uint32_t msb     = 0x800000;
	uint32_t mask    = 0xFF8000;
	uint32_t result  = (uint32_t)value<<8; // Pad with zeros as specified in spec
	
	while( msb != 0x80 ) {
		
		// Check if msb of current value is 1 and apply XOR mask
		if( result & msb )
			result = ((result ^ polynom) & mask) | ( result & ~mask);
			
		// Shift by one
		msb >>= 1;
		mask >>= 1;
		polynom >>=1;
	}
	if( result == crc )
		HTU2XD->CRC_CHECK = CRC_OK;
	else
		HTU2XD->CRC_CHECK = CRC_ERROR;
}

/*=========================================================================================================================================
 * @brief     Read HTU2XD ADC Temperature Raw Data
 * @param     HTU2XD   HTU2XD Struct ADC_TEMPERATURE_DATA variable
 * @return    Nothing
 ========================================================================================================================================*/
void GB_HTU2XD_ADC_Temperature_Raw_Data ( GebraBit_HTU2XD * HTU2XD ) 
{
	HAL_I2C_Mem_Read(HTU2XD_I2C,HTU2XD_WRITE_ADDRESS,HTU2XD_TRIGGER_TEMPERATURE_MEASUREMENT_CMD,I2C_MEMADD_SIZE_8BIT,HTU2XD->ADC_TEMPERATURE,3,200);
	HTU2XD->ADC_TEMPERATURE_DATA = ((uint16_t)(HTU2XD->ADC_TEMPERATURE[0] << 8) | (HTU2XD->ADC_TEMPERATURE[1]));
	HTU2XD->HTU2XD_CRC = HTU2XD->ADC_TEMPERATURE[2] ;
	GB_HTU2XD_CRC_Check( HTU2XD , HTU2XD->ADC_TEMPERATURE_DATA , HTU2XD->HTU2XD_CRC) ;
	HAL_Delay(400);
}
/*
M403Z 
*/
/*=========================================================================================================================================
 * @brief     Read HTU2XD ADC Humidity Raw Data
 * @param     HTU2XD   HTU2XD Struct ADC_HUMIDITY_DATA variable
 * @return    Nothing
 ========================================================================================================================================*/
void GB_HTU2XD_ADC_Humidity_Raw_Data ( GebraBit_HTU2XD * HTU2XD ) 
{
	HAL_I2C_Mem_Read(HTU2XD_I2C,HTU2XD_WRITE_ADDRESS,HTU2XD_TRIGGER_HUMIDITY_MEASUREMENT_CMD,I2C_MEMADD_SIZE_8BIT,HTU2XD->ADC_HUMIDITY,3,200);
	HTU2XD->ADC_HUMIDITY_DATA = ((uint16_t)(HTU2XD->ADC_HUMIDITY[0] << 8) | (HTU2XD->ADC_HUMIDITY[1]));
	HTU2XD->HTU2XD_CRC = HTU2XD->ADC_HUMIDITY[2] ;
	GB_HTU2XD_CRC_Check( HTU2XD , HTU2XD->ADC_HUMIDITY_DATA , HTU2XD->HTU2XD_CRC) ;
	HAL_Delay(400);
}
/*=========================================================================================================================================
 * @brief     Calculate Temperature
 * @param     HTU2XD   HTU2XD Struct TEMPERATURE variable
 * @return    Nothing
 ========================================================================================================================================*/
void GB_HTU2XD_Temperature ( GebraBit_HTU2XD * HTU2XD  ) 
{
  GB_HTU2XD_ADC_Temperature_Raw_Data(HTU2XD);
	// Perform conversion function
	HTU2XD->TEMPERATURE = (float)HTU2XD->ADC_TEMPERATURE_DATA * TEMPERATURE_COEFF_MUL / (1UL<<16) + TEMPERATURE_COEFF_ADD;
}
/*=========================================================================================================================================
 * @brief     Calculate Humidity
 * @param     HTU2XD   HTU2XD Struct HUMIDITY variable
 * @return    Nothing
 ========================================================================================================================================*/
void GB_HTU2XD_Humidity ( GebraBit_HTU2XD * HTU2XD  )  
{
  GB_HTU2XD_ADC_Humidity_Raw_Data(HTU2XD);
	// Perform conversion function
	HTU2XD->HUMIDITY = (float)HTU2XD->ADC_HUMIDITY_DATA * HUMIDITY_COEFF_MUL / (1UL<<16) + HUMIDITY_COEFF_ADD;
}
/*=========================================================================================================================================
 * @brief     Calculate Compensated Humidity
 * @param     HTU2XD   HTU2XD Struct HUMIDITY variable
 * @return    Nothing
 ========================================================================================================================================*/
void GB_HTU2XD_Compensated_Humidity( GebraBit_HTU2XD * HTU2XD  ) 
{
 HTU2XD->COMPANSATED_HUMIDITY = HTU2XD->HUMIDITY + (25 - HTU2XD->TEMPERATURE) * HTU2XD_TEMPERATURE_COEFFICIENT;
}
/*=========================================================================================================================================
 * @brief     Calculate Dew Point
 * @param     HTU2XD   HTU2XD Struct DEW_POINT variable and PARTIAL_PRESSURE variable
 * @return    Nothing
 ========================================================================================================================================*/
void GB_HTU2XD_Dew_Point( GebraBit_HTU2XD * HTU2XD  ) 
{
	double partial_pressure;
	double dew_point;	
// Missing power of 10
//	HTU2XD->PARTIAL_PRESSURE = pow( 10, HTU2XD_CONSTANT_A - HTU2XD_CONSTANT_B / (HTU2XD->TEMPERATURE + HTU2XD_CONSTANT_C) );
//	HTU2XD->DEW_POINT        = - HTU2XD_CONSTANT_B / (log10( HTU2XD->COMPANSATED_HUMIDITY * partial_pressure / 100) - HTU2XD_CONSTANT_A) - HTU2XD_CONSTANT_C;
}
/*=========================================================================================================================================
 * @brief     initialize HTU2XD
 * @param     HTU2XD     HTU2XD Struct 
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_HTU2XD_initialize( GebraBit_HTU2XD * HTU2XD )
{
  GB_HTU2XD_Soft_Reset   ( HTU2XD ) ;
  GB_HTU2XD_Check_Battery_Voltage_VDD ( HTU2XD ) ;
	GB_HTU2XD_OTP ( HTU2XD , OTP_DISABLE ) ;
}
/*=========================================================================================================================================
 * @brief     Configure HTU2XD
 * @param     HTU2XD  Configure HTU2XD 
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_HTU2XD_Configuration(GebraBit_HTU2XD * HTU2XD)
{
	GB_HTU2XD_On_Chip_Heater( HTU2XD , Disable ) ;
  GB_HTU2XD_Measurement_Resolution ( HTU2XD , HTU2XD_HUMIDITY_10BIT_TEMPERATURE_13BIT ) ; 
}
/*=========================================================================================================================================
 * @brief     Get Data  
 * @param     HTU2XD       GebraBit_HTU2XD Staruct
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_HTU2XD_Get_Data(GebraBit_HTU2XD * HTU2XD)
{
  GB_HTU2XD_Temperature( HTU2XD );
	GB_HTU2XD_Humidity(HTU2XD);
	GB_HTU2XD_Compensated_Humidity( HTU2XD  );
//	GB_HTU2XD_Dew_Point( HTU2XD  );
}
/*----------------------------------------------------------------------------------------------------------------------------------------*
 *                                                                      End                                                               *
 *----------------------------------------------------------------------------------------------------------------------------------------*/
//  GB_HTU2XD_Read_User_Register(&data);
//	GB_HTU2XD_Write_User_Register(0x83);
//	GB_HTU2XD_Read_User_Register_Bits(START_MSB_BIT_AT_7, BIT_LENGTH_4, &HTU2XD_Module.Register_Cache);
//	GB_HTU2XD_Read_User_Register_Bits(START_MSB_BIT_AT_3, BIT_LENGTH_4, &HTU2XD_Module.Register_Cache);
//	GB_HTU2XD_Write_User_Register_Bits(START_MSB_BIT_AT_7, BIT_LENGTH_4, 0);
//	GB_HTU2XD_Write_User_Register_Bits(START_MSB_BIT_AT_3, BIT_LENGTH_4, 3);
//	GB_HTU2XD_Read_User_Register(&data);