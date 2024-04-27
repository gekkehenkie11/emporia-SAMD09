#include "stdio.h"
#include <stdint.h>
#include <stdbool.h> 

typedef volatile       uint8_t  RwReg8;  /**< Read-Write  8-bit register (volatile unsigned int) */
typedef volatile       uint8_t  RoReg8;  /**< Read only  8-bit register (volatile const unsigned int) */
typedef volatile       uint16_t RwReg16; /**< Read-Write 16-bit register (volatile unsigned int) */
typedef volatile       uint16_t RoReg16; /**< Read only 16-bit register (volatile unsigned int) */
typedef volatile       uint32_t RwReg;   /**< Read-Write 32-bit register (volatile unsigned int) */
typedef volatile       uint32_t RoReg;   /**< Read only 32-bit register (volatile const unsigned int) */


#define REG_ADC_CTRLA              (*(RwReg8 *)0x42002000UL)
#define REG_ADC_REFCTRL            (*(RwReg8 *)0x42002001UL)
#define REG_ADC_SAMPCTRL           (*(RwReg8 *)0x42002003UL)
#define REG_ADC_CTRLB              (*(RwReg16*)0x42002004UL)
#define REG_ADC_INPUTCTRL          (*(RwReg  *)0x42002010UL)
#define REG_ADC_EVCTRL             (*(RwReg8 *)0x42002014UL) /**< \brief (ADC) Event Control */
#define REG_ADC_INTFLAG            (*(RwReg8 *)0x42002018UL) /**< \brief (ADC) Interrupt Flag Status and Clear */
#define REG_ADC_STATUS             (*(RoReg8 *)0x42002019UL) /**< \brief (ADC) Status */
#define REG_ADC_RESULT             (*(RoReg16*)0x4200201AUL) /**< \brief (ADC) Result */
#define REG_ADC_CALIB              (*(RwReg16*)0x42002028UL)

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

#define REG_DMAC_CTRL              (*(RwReg16*)0x41004800UL) /**< \brief (DMAC) Control */
#define REG_DMAC_PRICTRL0          (*(RwReg  *)0x41004814UL) /**< \brief (DMAC) Priority Control 0 */
#define REG_DMAC_BASEADDR          (*(RwReg  *)0x41004834UL) /**< \brief (DMAC) Descriptor Memory Section Base Address */
#define REG_DMAC_WRBADDR           (*(RwReg  *)0x41004838UL) /**< \brief (DMAC) Write-Back Memory Section Base Address */
#define REG_DMAC_CHID              (*(RwReg8 *)0x4100483FUL) /**< \brief (DMAC) Channel ID */
#define REG_DMAC_CHCTRLA           (*(RwReg8 *)0x41004840UL) /**< \brief (DMAC) Channel Control A */
#define REG_DMAC_CHCTRLB           (*(RwReg  *)0x41004844UL) /**< \brief (DMAC) Channel Control B */
#define REG_DMAC_CHINTENSET        (*(RwReg8 *)0x4100484DUL) /**< \brief (DMAC) Channel Interrupt Enable Set */

#define REG_NVIC_SETENA	    (*(RwReg  *)0xE000E100UL) //Interrupt Set-Enable Register
#define REG_NVIC_PRIO1		    (*(RwReg  *)0xE000E404UL) //Interrupt Priority Register 1
#define REG_NVIC_PRIO2		    (*(RwReg  *)0xE000E408UL) //Interrupt Priority Register 1
#define REG_NVIC_PRIO3		    (*(RwReg  *)0xE000E40CUL) //Interrupt Priority Register 3

#define REG_GCLK_CLKCTRL           (*(RwReg16*)0x40000C02UL) /**< \brief (GCLK) Generic Clock Control */

#define REG_SERCOM1_I2CM_CTRLA     (*(RwReg  *)0x42000C00UL) /**< \brief (SERCOM1) I2CM Control A */
#define REG_SERCOM1_I2CM_CTRLB     (*(RwReg  *)0x42000C04UL) /**< \brief (SERCOM1) I2CM Control B */
#define REG_SERCOM1_I2CM_INTENSET  (*(RwReg8 *)0x42000C16UL) /**< \brief (SERCOM1) I2CM Interrupt Enable Set */
#define REG_SERCOM1_I2CM_SYNCBUSY  (*(RoReg  *)0x42000C1CUL) /**< \brief (SERCOM1) I2CM Syncbusy */
#define REG_SERCOM1_I2CS_ADDR      (*(RwReg  *)0x42000C24UL) /**< \brief (SERCOM1) I2CS Address */


/* Exception Table */
__attribute__ ((section(".vectors")))
const DeviceVectors exception_table = {

        /* Configure Initial Stack Pointer, using linker-generated symbols */
        .pvStack                = (void*) (&_estack),

        .pfnReset_Handler       = (void*) Reset_Handler,
        .pfnNonMaskableInt_Handler = (void*) NonMaskableInt_Handler,
        .pfnHardFault_Handler   = (void*) HardFault_Handler,
        .pvReservedM12          = (void*) (0UL), /* Reserved */
        .pvReservedM11          = (void*) (0UL), /* Reserved */
        .pvReservedM10          = (void*) (0UL), /* Reserved */
        .pvReservedM9           = (void*) (0UL), /* Reserved */
        .pvReservedM8           = (void*) (0UL), /* Reserved */
        .pvReservedM7           = (void*) (0UL), /* Reserved */
        .pvReservedM6           = (void*) (0UL), /* Reserved */
        .pfnSVCall_Handler      = (void*) SVCall_Handler,
        .pvReservedM4           = (void*) (0UL), /* Reserved */
        .pvReservedM3           = (void*) (0UL), /* Reserved */
        .pfnPendSV_Handler      = (void*) PendSV_Handler,
        .pfnSysTick_Handler     = (void*) SysTick_Handler,

        /* Configurable interrupts */
        .pfnPM_Handler          = (void*) PM_Handler,             /*  0 Power Manager */
        .pfnSYSCTRL_Handler     = (void*) SYSCTRL_Handler,        /*  1 System Control */
        .pfnWDT_Handler         = (void*) WDT_Handler,            /*  2 Watchdog Timer */
        .pfnRTC_Handler         = (void*) RTC_Handler,            /*  3 Real-Time Counter */
        .pfnEIC_Handler         = (void*) EIC_Handler,            /*  4 External Interrupt Controller */
        .pfnNVMCTRL_Handler     = (void*) NVMCTRL_Handler,        /*  5 Non-Volatile Memory Controller */
        .pfnDMAC_Handler        = (void*) DMAC_Handler,           /*  6 Direct Memory Access Controller */
        .pvReserved7            = (void*) (0UL),                  /*  7 Reserved */
        .pfnEVSYS_Handler       = (void*) EVSYS_Handler,          /*  8 Event System Interface */
        .pfnSERCOM0_Handler     = (void*) SERCOM0_Handler,        /*  9 Serial Communication Interface 0 */
        .pfnSERCOM1_Handler     = (void*) SERCOM1_Handler,        /* 10 Serial Communication Interface 1 */
        .pvReserved11           = (void*) (0UL),                  /* 11 Reserved */
        .pvReserved12           = (void*) (0UL),                  /* 12 Reserved */
        .pfnTC1_Handler         = (void*) TC1_Handler,            /* 13 Basic Timer Counter 0 */
        .pfnTC2_Handler         = (void*) TC2_Handler,            /* 14 Basic Timer Counter 1 */
        .pfnADC_Handler         = (void*) ADC_Handler,            /* 15 Analog Digital Converter */
        .pvReserved16           = (void*) (0UL),                  /* 16 Reserved */
        .pvReserved17           = (void*) (0UL),                  /* 17 Reserved */
        .pfnPTC_Handler         = (void*) PTC_Handler             /* 18 Peripheral Touch Controller */
};


bool alldataready = false;

void sendESPpacket()
{
	//TODO create this routine
}

void  enableDMA ()
{
	*((uint16_t*)0x20000022) = 8;//BTCNT, number of beats per transaction. We're moving 8 ADC results each time.
	*((uint32_t*)0x20000024) = REG_ADC_RESULT;//Source address
	*((uint32_t*)0x20000028) = 0x20000040 ;//Destination address 0x20000030 + 0x10 (transaction length)
	REG_DMAC_CHID = 0;
	REG_DMAC_CHCTRLA = REG_DMAC_CHCTRLA | 2;//Enable the DMA channel;
}

void enableADC()
{
	REG_ADC_CTRLA = REG_ADC_CTRLA | 2;
	do {
	}   while (REG_ADC_STATUS != 0);
}

void enable_TC1()
{
	REG_TC1_CTRLA = REG_TC1_CTRLA | 2;
	do {
	} while (REG_TC1_STATUS >= 0x80); //We defined it as unsigned, checking for bit 7, SYNCBUSY.
}

void COnfigSerCom1 ()
{
	REG_GCLK_CLKCTRL = 0x410F; //01000001 00001111, Clock Enable, GCLK1, GCLK_SERCOM1_CORE
	REG_SERCOM1_I2CM_CTRLB = 0x500; // 0000 0101 00000000, 
	do {
  	} while (REG_SERCOM1_I2CM_SYNCBUSY != 0);
  	
  	REG_SERCOM1_I2CS_ADDR = 0xC8; //the as address
  	REG_SERCOM1_I2CM_CTRLA = 0x100012;//0001 0000 - 0000 0000 - 0001 0010, 50-100ns hold time, enable, 
	do {
  	} while (REG_SERCOM1_I2CM_SYNCBUSY != 0);
  	
  	REG_NVIC_PRIO2 = (REG_NVIC_PRIO2 & 0xFF00FFFF) | 0xC00000;
  	REG_NVIC_SETENA = 0x400; //Enable interrupt: 0100 00000000 = int 10 = our SerCom IRQ.
  	REG_SERCOM1_I2CM_INTENSET = 5;
}

void configureNestedVectoredInterruptController ()
{
	__asm__ __volatile__("dmb sy");
	__asm__ __volatile__("CPSIE I");
	REG_NVIC_PRIO1 = (REG_NVIC_PRIO1 & 0xFF00FFFF) | 0x400000;
	REG_NVIC_SETENA = 0x40; //Enable interrupt: 01000000 = int 6 = our DMA IRQ.
	REG_NVIC_PRIO3 = (REG_NVIC_PRIO3 & 0xFFFF00FF) | 0xC000;
	REG_NVIC_SETENA = 0x2000; //Enable interrupt: 00100000 00000000 = int 13, why ??
}

void configureDirectMemoryAccessController ()
{
	REG_DMAC_BASEADDR = 0x20000020; //Descriptor memory section base address, 0x10 bytes long
	*((uint16_t*)0x20000020) = 0x909; //0000 1001 0000 1001.  stepsize = 0, So next address = beatsize*1
					   //STEPSEL = 0, so DST
					   // DSTINC = 1, so auto increment destination
					   // SRCINC = 0, so no auto increment on source.
					   //Beatsize = 16 bit
					   // blockact = 1 = INT = Channel in normal operation and block interrupt
					   //Event Output Selection = 0, DISABLE, no event generation
					   //valid.
	REG_DMAC_WRBADDR  = 0x20000010; //Write-Back Memory Section Base Address, 0x10 bytes long
	REG_DMAC_PRICTRL0 = 0x81818181; //10000001 10000001 10000001 10000001, Round-robin scheduling scheme for channels with level 3 priority.
	REG_DMAC_CHID = 0;//Channel ID 0
	REG_DMAC_CHCTRLB = 0x801200 ; // 00000000 10000000 00010010 00000000
					//Software Command: no action
					//Trigger action: BEAT. One trigger required for each beat transfer
					//Channel resume operation
					//Trigger source = ADC Result Ready Trigger
					//Channel priority level 0
					//Channel event generation is disabled.
					//Channel event action will not be executed on any incoming event.
					//Event Input Action = NO action
	REG_DMAC_CHINTENSET = 3;//enable Channel Transfer Complete interrupt and Channel Transfer Error interrupt.
	REG_DMAC_CTRL = 0xF02; //00001111 00000010 = Transfer requests for all Priority levels are handled. No CRC. DMA enable.			   
}

void config_PORT()
{
	REG_PORT_DIR = 0x3030000; //00000011 00000011 00000000 00000000, so outputs are pin 16, 17, 24, 25. Rest is input.
	REG_PORT_OUT = 0xC00000; // 00000000 11000000 00000000 00000000, so 22 and 23 driven high, rest driven low.
	REG_PINCFG2 = 1;
	REG_PINCFG3 = 1;
	REG_PINCFG4 = 1;	
	REG_PINCFG5 = 1;
	REG_PINCFG6 = 1;
	REG_PINCFG7 = 1;
	REG_PINCFG8 = 4;
	REG_PINCFG9 = 4;
	REG_PINCFG10 = 1;
	REG_PINCFG11 = 1;
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
	} while ((REG_SYSCTRL_PCLKSR & 0x10) == 0); //DFLLRDY. Wait for oscillator stabilization
    
	uint32_t calibdat = *((uint32_t*)0x806024) >> 0x1a;
	if (calibdat == 0x3f) 
		calibdat = 0x1f;
    	
	calibdat = calibdat << 10;
	REG_SYSCTRL_DFLLVAL = calibdat | *((uint32_t*)0x806028) & 0x3ff;
	REG_SYSCTRL_DFLLCTRL = 2;
    
	do {
	} while ((REG_SYSCTRL_PCLKSR & 0x10) == 0); //DFLLRDY. Wait for oscillator stabilization
    
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
    
        //Load ADC factory calibration values
	uint32_t tmp =  (*((uint32_t*)0x806024) << 5) & 0x700 | *((uint32_t*)0x806020) >> 27 | (*((uint32_t*)0x806024) << 5) & 0xff | ((*((uint32_t*)0x806024) & 7) << 5);	 
	REG_ADC_CALIB = tmp;    
	REG_ADC_SAMPCTRL = 1; //Sampling Time Length
	REG_ADC_REFCTRL = 3; //VREFA as reference
	REG_ADC_INPUTCTRL = 0x1270000; // 00000001 00100111 00000000 00000000
					//  00000001 = GAIN 2X.
					// 0010  =inputoffset: 2.
					// 0111 = Inputscan= 7+1 = 8
	REG_ADC_CTRLB = 0x101;  //12 bits conversion
				//Disable digital result correction
				//The ADC conversion result is right-adjusted in the RESULT register.
				//Single conversion mode
				//Differential mode. In this mode, the voltage difference between the MUXPOS and MUXNEG
				//inputs will be converted by the ADC
	REG_ADC_INTFLAG = 0xF; //clear intflags
	REG_ADC_EVCTRL = 1; //A new conversion will be triggered on any incoming event
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
	configureDirectMemoryAccessController();
	adc_config();
	ConfigureTimerCounter1 ();
	configureNestedVectoredInterruptController();
	enableDMA ();
	enableADC ();
	enable_TC1();
	COnfigSerCom1();
	
	for (;;) //main program loop
	{		
		if (alldataready)
			sendESPpacket();
	}
	
	return 0 ;
}
