/**************************************************************************/
/*!
	@file     TSL45315.cpp
    @author   Joachim (joachim.langenbach@engsas.de)

    This is a library for the TSL45315
    This library works with the Watterott TSL45315 breakout
    ----> https://learn.watterott.com/retired/sensors/tsl45315/

    Code adapted from https://github.com/adidax/Makerblog_TSL45315

    Check out the links above for our tutorials and wiring diagrams
    These chips use I2C to communicate

    @section LICENSE

    Software License Agreement (BSD License)

    Copyright (c) 2022 Joachim Langenbach
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:
    1. Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.
    3. Neither the name of the copyright holders nor the
    names of its contributors may be used to endorse or promote products
    derived from this software without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
    EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
    DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
    (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
    ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
/**************************************************************************/
#if defined(ESP8266) || defined(ESP32)
  #include <pgmspace.h>
#else
  #include <avr/pgmspace.h>
#endif
#if defined(__AVR__)
  #include <util/delay.h>
#endif
#include <stdlib.h>

#include "TSL45315.h"

TSL45315::_TSL45315(uint8_t resolution)
 {
	 _resolution = resolution;
	 _timerfactor = 0;
	 if (resolution == uint8_t(TSL45315_TIME_M1)) {
		 _timerfactor = 1;
	 }
	 if (resolution == uint8_t(TSL45315_TIME_M2)) {
		 _timerfactor = 2;
	 }
	 if (resolution == uint8_t(TSL45315_TIME_M4)) {
		 _timerfactor = 4;
	 }
 }


 /*========================================================================*/
 /*                           PUBLIC FUNCTIONS                             */
 /*========================================================================*/
 boolean TSL45315::begin(void)
 {
	Wire.begin();
	Wire.beginTransmission(TSL45315_I2C_ADDR);
	Wire.write(0x80|TSL45315_REG_ID);
	Wire.endTransmission();

	Wire.requestFrom(TSL45315_I2C_ADDR, 1);
	while(Wire.available())
	{
		unsigned char c = Wire.read();
		c = c & 0xF0;
		if (c != 0xA0) {
			return false;
		}
	}

	Wire.beginTransmission(TSL45315_I2C_ADDR);
	Wire.write(0x80|TSL45315_REG_CONTROL);
	Wire.write(0x03);
	Wire.endTransmission();

	Wire.beginTransmission(TSL45315_I2C_ADDR);
	Wire.write(0x80|TSL45315_REG_CONFIG);
	Wire.write(_resolution);
	Wire.endTransmission();

	return true;
 }


boolean TSL45315::disable(void)
{
	Wire.beginTransmission(TSL45315_I2C_ADDR);
	Wire.write(0x80|TSL45315_REG_CONTROL);
	Wire.write(0x00);
	Wire.endTransmission();
	return true;
}


uint32_t TSL45315::readLux(void)
{
	uint32_t lux;

	Wire.beginTransmission(TSL45315_I2C_ADDR);
	Wire.write(0x80|TSL45315_REG_DATALOW);
	Wire.endTransmission();
	Wire.requestFrom(TSL45315_I2C_ADDR, 2);
	_low = Wire.read();
	_high = Wire.read();
	while(Wire.available()){
		Wire.read();
	}

	lux  = (_high<<8) | _low;
	lux = lux * _timerfactor;
	return lux;
}
