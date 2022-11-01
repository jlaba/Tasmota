/**************************************************************************/
/*! 
    @file     TSL45315.h
    @author   Joachim (joachim.langenbach@engsas.de)

    This is a library for the TSL45315
    This library works with the Watterott TSL45315 breakout
    ----> https://learn.watterott.com/retired/sensors/tsl45315/

    Code adapted from https://github.com/adidax/Makerblog_TSL45315
	
    Check out the links above for our tutorials and wiring diagrams 
    These chips use I2C to communicate

*/
/**************************************************************************/

#ifndef _TSL45315_H_
#define _TSL45315_H_

#if ARDUINO >= 100
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif
#include <Wire.h>

#define TSL45315_I2C_ADDR 		(0x29)

#define TSL45315_REG_CONTROL  	(0x00)
#define TSL45315_REG_CONFIG   	(0x01)
#define TSL45315_REG_DATALOW  	(0x04)
#define TSL45315_REG_DATAHIGH 	(0x05)
#define TSL45315_REG_ID       	(0x0A)

// Sensing time for one measurement
#define TSL45315_TIME_M1		(0x00)	// M=1 T=400ms
#define TSL45315_TIME_M2		(0x01)	// M=2 T=200ms
#define TSL45315_TIME_M4		(0x02)	// M=4 T=100ms


/**************************************************************************/
/*! 
    @brief  Class that stores state and functions for interacting with TSL45315 Light Sensor
*/
/**************************************************************************/
class TSL45315
{
	public:
		TSL45315(uint8_t resolution = TSL45315_TIME_M4);
  
		boolean   begin   ( void );
//   void      enable  ( void );
		void      disable ( void );
		uint32_t readLux(void);

	private:
		uint16_t _low, _high, _timerfactor;
		uint8_t _resolution;
};
#endif
