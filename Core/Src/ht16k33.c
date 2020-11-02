/**************************************************************************/
/*!
    @file     ht16k33.c
    @author   K. Townsend (microBuilder.eu)

    @section DESCRIPTION

    I2C Driver for the HT16K33 16*8 LED Driver

    This driver is based on the HT16K33 Library from Limor Fried
    (Adafruit Industries) at:
    https://github.com/adafruit/Adafruit-LED-Backpack-Library

    @section LICENSE

    Software License Agreement (BSD License)

    Copyright (c) 2012, K. Townsend
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
#include "ht16k33.h"
#include <string.h>
#include <stdio.h>

I2C_HandleTypeDef* _hi2c;

/**************************************************************************/
/* Private Methods                                                        */
/**************************************************************************/

/**************************************************************************/
/*!
    @brief  Writes to a register over I2C
*/
/**************************************************************************/
HAL_StatusTypeDef ht16k33_WriteRegister (uint8_t reg)
{
  return HAL_I2C_Master_Transmit(_hi2c, HT16K33_I2C_ADDRESS, &reg, 1, HAL_MAX_DELAY);
}

/**************************************************************************/
/*!
    @brief  Writes an unsigned 8 bit values over I2C
*/
/**************************************************************************/
HAL_StatusTypeDef ht16k33_Write8 (uint8_t reg, uint8_t value)
{
  uint8_t buf[] = {reg, value};
  return HAL_I2C_Master_Transmit(_hi2c, HT16K33_I2C_ADDRESS, buf, 2, HAL_MAX_DELAY);
}

/**************************************************************************/
/* Public Methods                                                         */
/**************************************************************************/

/**************************************************************************/
/*!
    @brief Initialises the HT16K33 LED driver
*/
/**************************************************************************/
HAL_StatusTypeDef ht16k33_Init(I2C_HandleTypeDef* hi2c)
{
  HAL_StatusTypeDef ret;
  _hi2c = hi2c;

  // Turn the oscillator on
  ret = ht16k33_WriteRegister(HT16K33_REGISTER_SYSTEM_SETUP | 0x01);

  // Turn blink off
  ret |= ht16k33_SetBlinkRate(HT16K33_BLINKRATE_OFF);

  // Set max brightness
  ret |= ht16k33_SetBrightness(15);

  return ret;
}

/**************************************************************************/
/*!
    @brief Sets the display brightness/dimming (0..15)
*/
/**************************************************************************/
HAL_StatusTypeDef ht16k33_SetBrightness(uint8_t brightness)
{
  if (brightness > 15) brightness = 15;
  return ht16k33_WriteRegister(HT16K33_REGISTER_DIMMING | brightness);
}

/**************************************************************************/
/*!
    @brief Sets the display blink rate
*/
/**************************************************************************/
HAL_StatusTypeDef ht16k33_SetBlinkRate(ht16k33BlinkRate_t blinkRate)
{
  if (blinkRate > HT16K33_BLINKRATE_HALFHZ) blinkRate = HT16K33_BLINKRATE_OFF;
  return ht16k33_WriteRegister(HT16K33_REGISTER_DISPLAY_SETUP | 0x01 | (blinkRate << 1));
}

/**************************************************************************/
/*!
    @brief Updates the display memory
*/
/**************************************************************************/
HAL_StatusTypeDef ht16k33_WriteDisplay(uint8_t *data)
{
  static uint8_t buf[17];
  buf[0] = 0x00;                      // Start at address 0
  memcpy(buf+1, data, 16); 
  return HAL_I2C_Master_Transmit(_hi2c, HT16K33_I2C_ADDRESS, buf, 17, HAL_MAX_DELAY);
}

HAL_StatusTypeDef ht16k33_WriteDigits(int value){
    static char data[9];
    static uint8_t buf[16];
    sprintf(data, "%08d", value);
    for (int i=0; i<8; i++) {
        buf[i*2] = _ht16k33_numbertable[data[i] & 0x0F];
        buf[i*2+1] = 0x00;
    }
    return ht16k33_WriteDisplay(buf);
}

HAL_StatusTypeDef ht16k33_WriteHex(char *p){
  static uint8_t buf[16];
  for (int i=0; i<8 && *p != '\0'; i++, p++) {
      uint8_t k;
      if(*p >= '0' && *p <= '9')
          k = (*p - '0');
      else if (*p >= 'A' && *p <= 'F') 
          k = (10 + (*p - 'A'));
      else if (*p >= 'a' && *p <= 'f')
          k = (10 + (*p - 'a'));
      else
          break;
      buf[i*2] = _ht16k33_numbertable[k];
      buf[i*2+1] = 0x00;
  }
  return ht16k33_WriteDisplay(buf);
}
