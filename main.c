#include "stdio.h"
#include <stdint.h>

typedef volatile       uint8_t  RwReg8;  /**< Read-Write  8-bit register (volatile unsigned int) */
typedef volatile       uint16_t RwReg16; /**< Read-Write 16-bit register (volatile unsigned int) */
typedef volatile       uint32_t RwReg;   /**< Read-Write 32-bit register (volatile unsigned int) */
typedef volatile       uint32_t RoReg;   /**< Read only 32-bit register (volatile const unsigned int) */
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

#define REG_SYSCTRL_PCLKSR         (*(RoReg  *)0x4000080CUL) /**< \brief (SYSCTRL) Power and Clocks Status */
#define REG_SYSCTRL_OSC32K         (*(RwReg  *)0x40000818UL) /**< \brief (SYSCTRL) 32kHz Internal Oscillator (OSC32K) Control */
#define REG_SYSCTRL_DFLLCTRL       (*(RwReg16*)0x40000824UL) /**< \brief (SYSCTRL) DFLL48M Control */
#define REG_SYSCTRL_DFLLVAL        (*(RwReg  *)0x40000828UL) /**< \brief (SYSCTRL) DFLL48M Value */

#define REG_GCLK_STATUS            (*(RoReg8 *)0x40000C01UL) /**< \brief (GCLK) Status */
#define REG_GCLK_CLKCTRL           (*(RwReg16*)0x40000C02UL) /**< \brief (GCLK) Generic Clock Control */
#define REG_GCLK_GENCTRL           (*(RwReg  *)0x40000C04UL) /**< \brief (GCLK) Generic Clock Generator Control */
#define REG_GCLK_GENDIV            (*(RwReg  *)0x40000C08UL) /**< \brief (GCLK) Generic Clock Generator Division */

#define REG_PM_APBCMASK            (*(RwReg  *)0x40000420UL) /**< \brief (PM) APBC Mask */

#define REG_NVMCTRL_CTRLB          (*(RwReg  *)0x41004004UL) /**< \brief (NVMCTRL) Control B */

#define REG_EVSYS_CHANNEL          (*(RwReg  *)0x42000404UL) /**< \brief (EVSYS) Channel */
#define REG_EVSYS_USER             (*(RwReg16*)0x42000408UL) /**< \brief (EVSYS) User Multiplexer */

void Config_NVMCTRL ()
{
	REG_NVMCTRL_CTRLB = 0x82;
}

void config_EventSystem ()
{
	REG_EVSYS_USER = 0x10C; //0001 00001100 connect channel 0, ADC_START as Event user
	REG_EVSYS_CHANNEL = 0x61F0000; // 0000 0110 0001 1111 00000000 00000000, EVGEN: 00011111 = TC1 OVF = TC1 underflow/overflow
}

void config_Sysctrl_PM_and_GCLK ()
{
	REG_SYSCTRL_OSC32K = 0;
	REG_SYSCTRL_DFLLCTRL = REG_SYSCTRL_DFLLCTRL & 0xFF7F;
    
	do {
	} while (((REG_SYSCTRL_PCLKSR) & 0x10) == 0); //DFLLRDY
    
	uint32_t calibdat = *((uint32_t*)0x806024) >> 0x1a;
	if (calibdat == 0x3f) 
		calibdat = 0x1f;
    	
	calibdat = calibdat << 10;
	REG_SYSCTRL_DFLLVAL = calibdat | *((uint32_t*)0x806028) & 0x3ff;
	REG_SYSCTRL_DFLLCTRL = 2;
    
	do {
	} while (((REG_SYSCTRL_PCLKSR) & 0x10) == 0); //DFLLRDY
    
	REG_GCLK_GENCTRL = 0x10700;
    
	do {
	} while (REG_GCLK_STATUS != 0);
    
	REG_GCLK_GENCTRL = 0x30701;
	REG_GCLK_GENDIV = 0x301;
    
	do {
	} while (REG_GCLK_STATUS != 0);
    
	REG_GCLK_CLKCTRL = 0x4007;
	REG_GCLK_CLKCTRL += 0x108;
	REG_GCLK_CLKCTRL += 3;    
	REG_GCLK_CLKCTRL += 1;    
    
	REG_PM_APBCMASK = 0x14A; 
	REG_PM_APBCMASK = 0; 
}

void adc_config() {

	REG_ADC_CTRLA = 1;
	do {
	}   while (REG_ADC_STATUS != 0);
    
	uint32_t tmp =  (*((uint32_t*)0x806024) << 5) & 0x700 | *((uint32_t*)0x806020) >> 27 | (*((uint32_t*)0x806024) << 5) & 0xff | ((*((uint32_t*)0x806024) & 7) << 5);	 
	REG_ADC_CALIB = tmp;    
	REG_ADC_SAMPCTRL = 1;
	REG_ADC_REFCTRL = 3;
	REG_ADC_INPUTCTRL = 0x1270000;
	REG_ADC_CTRLB = 0x101;
	REG_ADC_INTFLAG = 0xF;
	REG_ADC_EVCTRL = 1;
	do {
	}  while (REG_ADC_STATUS != 0);
}

int main()
{
	REG_NVMCTRL_CTRLB = 6;
	config_Sysctrl_PM_and_GCLK ();
	Config_NVMCTRL();
	config_EventSystem();
	adc_config();
	return 0 ;
}
