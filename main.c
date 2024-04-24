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

#define REG_TC1_CTRLA              (*(RwReg16*)0x42001800UL) /**< \brief (TC1) Control A */
#define REG_TC1_EVCTRL             (*(RwReg16*)0x4200180AUL) /**< \brief (TC1) Event Control */
#define REG_TC1_INTFLAG            (*(RwReg8 *)0x4200180EUL) /**< \brief (TC1) Interrupt Flag Status and Clear */
#define REG_TC1_STATUS             (*(RoReg8 *)0x4200180FUL) /**< \brief (TC1) Status */
#define REG_TC1_COUNT16_CC0        (*(RwReg16*)0x42001818UL) /**< \brief (TC1) COUNT16 Compare/Capture 0 */

#define REG_PORT_DIR	            (*(RwReg  *)0x41004400UL)
#define REG_PORT_OUT	            (*(RwReg  *)0x41004410UL) 
#define REG_PINCFG2	            (*(RwReg8 *)0x41004442UL) 
#define REG_PINCFG3	            (*(RwReg8 *)0x41004443UL) 
#define REG_PINCFG4	            (*(RwReg8 *)0x41004444UL) 
#define REG_PINCFG5	            (*(RwReg8 *)0x41004445UL) 
#define REG_PINCFG6	            (*(RwReg8 *)0x41004446UL) 
#define REG_PINCFG7	            (*(RwReg8 *)0x41004447UL) 
#define REG_PINCFG8	            (*(RwReg8 *)0x41004448UL) 
#define REG_PINCFG9	            (*(RwReg8 *)0x41004449UL) 
#define REG_PINCFG10	            (*(RwReg8 *)0x4100444AUL) 
#define REG_PINCFG11	            (*(RwReg8 *)0x4100444BUL) 

#define REG_PINCFG14	            (*(RwReg8 *)0x4100444EUL) 
#define REG_PINCFG15	            (*(RwReg8 *)0x4100444FUL) 
#define REG_PINCFG22	            (*(RwReg8 *)0x41004456UL) 
#define REG_PINCFG23	            (*(RwReg8 *)0x41004457UL) 
#define REG_PINCFG25	            (*(RwReg8 *)0x41004459UL) 
#define REG_PINCFG27	            (*(RwReg8 *)0x4100445BUL) 
#define REG_PINCFG28	            (*(RwReg8 *)0x4100445CUL) 
#define REG_PINCFG30	            (*(RwReg8 *)0x4100445EUL) 
#define REG_PINCFG31	            (*(RwReg8 *)0x4100445FUL) 

#define REG_PORT_PMUX1             (*(RwReg8 *)0x41004431UL) 
#define REG_PORT_PMUX2	            (*(RwReg8 *)0x41004432UL) 
#define REG_PORT_PMUX3	            (*(RwReg8 *)0x41004433UL) 
#define REG_PORT_PMUX5	            (*(RwReg8 *)0x41004435UL) 
#define REG_PORT_PMUX7	            (*(RwReg8 *)0x41004437UL) 
#define REG_PORT_PMUX11            (*(RwReg8 *)0x4100443BUL) 


void config_PORT()
{
	REG_PORT_DIR = 0x3030000;
	REG_PORT_OUT = 0xC00000;
	REG_PINCFG2 = 1;
	REG_PINCFG2 = 1;
	REG_PINCFG4 = 1;	
	REG_PINCFG5 = 1;
	REG_PINCFG6 = 1;
	REG_PINCFG7 = 1;
	REG_PINCFG8 = 4;
	REG_PINCFG9 = 4;
	REG_PINCFG10 = 1;
	REG_PINCFG14 = 1;						
	REG_PINCFG15 = 1;	
	REG_PINCFG22 = 5;
	REG_PINCFG23 = 5;	
	REG_PINCFG25 = 4;
	REG_PINCFG27 = 4;	
	REG_PINCFG28 = 4;
	REG_PINCFG30 = 4;		
	REG_PINCFG31 = 4;	
	REG_PORT_PMUX1 = 0x11;	 
	REG_PORT_PMUX2 = 0x11;
	REG_PORT_PMUX3 = 0x11;
	REG_PORT_PMUX5 = 0x11;
	REG_PORT_PMUX7 = 0x11;
	REG_PORT_PMUX11 = 0x22;					
}

void ConfigureTimerCounter1 () {
	REG_TC1_CTRLA = 1; //SWRST
	do {
	} while (REG_TC1_STATUS >= 0x80); //We defined it as unsigned, checking for bit 7, SYNCBUSY.

	REG_TC1_CTRLA = 0x60; //0110 0000 Match PWM, 16 bit mode
	REG_TC1_COUNT16_CC0 = 0x4C; //The period.
	REG_TC1_INTFLAG = 0x3B; //clear flags
	REG_TC1_EVCTRL = 0x100; // bit 8 = Overflow/Underflow event is enabled and will be generated for every counter overflow/underflow.
	do {
	} while (REG_TC1_STATUS >= 0x80); //We defined it as unsigned, checking for bit 7, SYNCBUSY.
}

void Config_NVMCTRL ()
{
	REG_NVMCTRL_CTRLB = 0x82; //1000 0010, Write commands must be issued through the CMD register. 1 wait states for a read operation
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
	config_PORT();
	config_Sysctrl_PM_and_GCLK ();
	Config_NVMCTRL();
	config_EventSystem();
	adc_config();
	ConfigureTimerCounter1 ();
	return 0 ;
}
