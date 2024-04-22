#include "stdio.h"
#include <stdint.h>

int main(void) {
typedef volatile       uint8_t  RwReg8;  /**< Read-Write  8-bit register (volatile unsigned int) */
typedef volatile       uint16_t RwReg16; /**< Read-Write 16-bit register (volatile unsigned int) */
typedef volatile       uint32_t RwReg;   /**< Read-Write 32-bit register (volatile unsigned int) */
typedef volatile       uint8_t  RoReg8;  /**< Read only  8-bit register (volatile const unsigned int) */

#define REG_ADC_CTRLA              (*(RwReg8 *)0x42002000UL)
#define REG_ADC_REFCTRL            (*(RwReg8 *)0x42002001UL)
#define REG_ADC_SAMPCTRL           (*(RwReg8 *)0x42002003UL)
#define REG_ADC_CALIB              (*(RwReg16*)0x42002028UL)
#define REG_ADC_CTRLB              (*(RwReg16*)0x42002004UL)
#define REG_ADC_INPUTCTRL          (*(RwReg  *)0x42002010UL)
#define REG_ADC_INTFLAG            (*(RwReg8 *)0x42002018UL) /**< \brief (ADC) Interrupt Flag Status and Clear */
#define REG_ADC_EVCTRL             (*(RwReg8 *)0x42002014UL) /**< \brief (ADC) Event Control */
#define REG_ADC_STATUS             (*(RoReg8 *)0x42002019UL) /**< \brief (ADC) Status */


    REG_ADC_CTRLA = 1;
  do {
  }   while (REG_ADC_STATUS< 0);
    	 
    REG_ADC_CALIB  = (*((uint16_t*)0x806024) << 5) & 0x700 | *((uint16_t*)0x806020) >> 27 | (*((uint16_t*)0x806024) << 5) & 0xff | (*((uint16_t*)0x806024) & 7) << 5;
    REG_ADC_SAMPCTRL = 1;
    REG_ADC_REFCTRL = 3;
    REG_ADC_INPUTCTRL = 0x1270000;
    REG_ADC_CTRLB = 0x101;
    REG_ADC_INTFLAG = 0xF;
    REG_ADC_EVCTRL = 1;
  do {
  }  while (REG_ADC_STATUS< 0);
