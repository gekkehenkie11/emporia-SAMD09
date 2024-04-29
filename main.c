#include "stdio.h"
#include <stdint.h>
#include <stdbool.h> 

typedef volatile       uint8_t  RwReg8;  /**< Read-Write  8-bit register (volatile unsigned int) */
typedef volatile       uint8_t  RoReg8;  /**< Read only  8-bit register (volatile const unsigned int) */
typedef volatile       uint16_t RwReg16; /**< Read-Write 16-bit register (volatile unsigned int) */
typedef volatile       uint16_t RoReg16; /**< Read only 16-bit register (volatile unsigned int) */
typedef volatile       uint32_t RwReg;   /**< Read-Write 32-bit register (volatile unsigned int) */
typedef volatile       uint32_t RoReg;   /**< Read only 32-bit register (volatile const unsigned int) */

#ifdef __cplusplus
  #define   __I     volatile             /*!< Defines 'read only' permissions                 */
#else
  #define   __I     volatile const       /*!< Defines 'read only' permissions                 */
#endif
#define     __O     volatile             /*!< Defines 'write only' permissions                */
#define     __IO    volatile             /*!< Defines 'read / write' permissions              */

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

#define REG_SERCOM1_I2CS_CTRLA     (*(RwReg  *)0x42000C00UL) /**< \brief (SERCOM1) I2CM Control A */
#define REG_SERCOM1_I2CS_CTRLB     (*(RwReg  *)0x42000C04UL) /**< \brief (SERCOM1) I2CM Control B */
#define REG_SERCOM1_I2CS_INTENSET  (*(RwReg8 *)0x42000C16UL) /**< \brief (SERCOM1) I2CM Interrupt Enable Set */
#define REG_SERCOM1_I2CS_INTFLAG   (*(RwReg8 *)0x42000C18UL) /**< \brief (SERCOM1) I2CS Interrupt Flag Status and Clear */
#define REG_SERCOM1_I2CS_STATUS    (*(RwReg16*)0x42000C1AUL) /**< \brief (SERCOM1) I2CS Status */
#define REG_SERCOM1_I2CS_SYNCBUSY  (*(RoReg  *)0x42000C1CUL) /**< \brief (SERCOM1) I2CM Syncbusy */
#define REG_SERCOM1_I2CS_ADDR      (*(RwReg  *)0x42000C24UL) /**< \brief (SERCOM1) I2CS Address */
#define REG_SERCOM1_I2CS_DATA      (*(RwReg8 *)0x42000C28UL) /**< \brief (SERCOM1) I2CS Data */

#define SCB_VTOR_TBLOFF_Pos                 7                                             /*!< SCB VTOR: TBLOFF Position */
#define SCB_VTOR_TBLOFF_Msk                (0x1FFFFFFUL << SCB_VTOR_TBLOFF_Pos)           /*!< SCB VTOR: TBLOFF Mask */


#define DUMMY __attribute__ ((weak, alias ("irq_handler_dummy")))


#define ESPpacketlength       0x11C

bool alldataready = false;
uint8_t ESPbyteIndex = 0; 
uint8_t temp = 0;
uint8_t EspPacket[ESPpacketlength]; //The final packet that we send to the ESP
uint16_t DMAresults[8]; //We copy the 8 ADC results to this buffer using DMA
			//Layout: MainCT1_V, MainCT1_A, MainCT2_V, MainCT2_A, MainCT_3_V, MainCT_3_A, Mux1_A, Mux2_A

struct DMAdescriptorType {           
  uint16_t BTCTRL;
  uint16_t BTCNT;
  uint32_t SRCADDR;
  uint32_t DSTADDR;
  uint32_t DESCADDR;
}  DMAdescriptor, DMAdescriptorwriteback;

extern unsigned int _etext;
extern unsigned int _data;
extern unsigned int _edata;
extern unsigned int _bss;
extern unsigned int _ebss;
extern int main(void);

void irq_handler_reset(void);
void irq_handler_dmac(void);
void irq_handler_sercom1(void);
DUMMY void irq_handler_nmi(void);
DUMMY void irq_handler_hard_fault(void);
DUMMY void irq_handler_sv_call(void);
DUMMY void irq_handler_pend_sv(void);
DUMMY void irq_handler_sys_tick(void);
DUMMY void irq_handler_pm(void);
DUMMY void irq_handler_sysctrl(void);
DUMMY void irq_handler_wdt(void);
DUMMY void irq_handler_rtc(void);
DUMMY void irq_handler_eic(void);
DUMMY void irq_handler_nvmctrl(void);
DUMMY void irq_handler_evsys(void);
DUMMY void irq_handler_sercom0(void);
DUMMY void irq_handler_tc1(void);
DUMMY void irq_handler_tc2(void);
DUMMY void irq_handler_adc(void);
DUMMY void irq_handler_ptc(void);

extern void _stack_top(void);

//-----------------------------------------------------------------------------
__attribute__ ((used, section(".vectors")))
void (* const vectors[])(void) =
{
  &_stack_top,                   // 0 - Initial Stack Pointer Value

  // Cortex-M0+ handlers
  irq_handler_reset,             // 1 - Reset
  irq_handler_nmi,               // 2 - NMI
  irq_handler_hard_fault,        // 3 - Hard Fault
  0,                             // 4 - Reserved
  0,                             // 5 - Reserved
  0,                             // 6 - Reserved
  0,                             // 7 - Reserved
  0,                             // 8 - Reserved
  0,                             // 9 - Reserved
  0,                             // 10 - Reserved
  irq_handler_sv_call,           // 11 - SVCall
  0,                             // 12 - Reserved
  0,                             // 13 - Reserved
  irq_handler_pend_sv,           // 14 - PendSV
  irq_handler_sys_tick,          // 15 - SysTick

  // Peripheral handlers
  irq_handler_pm,                // 0 - Power Manager
  irq_handler_sysctrl,           // 1 - System Controller
  irq_handler_wdt,               // 2 - Watchdog Timer
  irq_handler_rtc,               // 3 - Real Time Counter
  irq_handler_eic,               // 4 - External Interrupt Controller
  irq_handler_nvmctrl,           // 5 - Non-Volatile Memory Controller
  irq_handler_dmac,              // 6 - Direct Memory Access Controller, we use this interrupt!
  0,		                 // 7 - Reserved (usb)
  irq_handler_evsys,             // 8 - Event System
  irq_handler_sercom0,           // 9 - Serial Communication Interface 0
  irq_handler_sercom1,           // 10 - Serial Communication Interface 1, we use this interrupt!
  0,		                  // 11 - Reserved (Serial Communication Interface 2)
  0,                             // 12 - Reserved
  irq_handler_tc1,               // 13 - Timer/Counter 1
  irq_handler_tc2,               // 14 - Timer/Counter 2
  irq_handler_adc,               // 15 - Analog-to-Digital Converter
  0,                             // 16 - Reserved
  0,		                 // 17 - Digital-to-Analog Converter
  irq_handler_ptc,               // 18 - Peripheral Touch Controller
};

/* Memory mapping of Cortex-M0+ Hardware */
#define SCS_BASE            (0xE000E000UL)                            /*!< System Control Space Base Address */
#define SysTick_BASE        (SCS_BASE +  0x0010UL)                    /*!< SysTick Base Address              */
#define NVIC_BASE           (SCS_BASE +  0x0100UL)                    /*!< NVIC Base Address                 */
#define SCB_BASE            (SCS_BASE +  0x0D00UL)                    /*!< System Control Block Base Address */

#define SCB                 ((SCB_Type       *)     SCB_BASE      )   /*!< SCB configuration struct           */
#define SysTick             ((SysTick_Type   *)     SysTick_BASE  )   /*!< SysTick configuration struct       */
#define NVIC                ((NVIC_Type      *)     NVIC_BASE     )   /*!< NVIC configuration struct          */

typedef struct
{
  __I  uint32_t CPUID;                   /*!< Offset: 0x000 (R/ )  CPUID Base Register                                   */
  __IO uint32_t ICSR;                    /*!< Offset: 0x004 (R/W)  Interrupt Control and State Register                  */
  __IO uint32_t VTOR;                    /*!< Offset: 0x008 (R/W)  Vector Table Offset Register                          */
  __IO uint32_t AIRCR;                   /*!< Offset: 0x00C (R/W)  Application Interrupt and Reset Control Register      */
  __IO uint32_t SCR;                     /*!< Offset: 0x010 (R/W)  System Control Register                               */
  __IO uint32_t CCR;                     /*!< Offset: 0x014 (R/W)  Configuration Control Register                        */
       uint32_t RESERVED1;
  __IO uint32_t SHP[2];                  /*!< Offset: 0x01C (R/W)  System Handlers Priority Registers. [0] is RESERVED   */
  __IO uint32_t SHCSR;                   /*!< Offset: 0x024 (R/W)  System Handler Control and State Register             */
} SCB_Type;


void irq_handler_dummy(void)
{
  while (1);
}

void irq_handler_reset(void)
{
  unsigned int *src, *dst;

  src = &_etext;
  dst = &_data;
  while (dst < &_edata)
    *dst++ = *src++;

  dst = &_bss;
  while (dst < &_ebss)
    *dst++ = 0;

  SCB->VTOR = (uint32_t)vectors;

  main();

  while (1);
}


void irq_handler_sercom1(void) //We use IRQ sources "DRDY" and "Stop"
{
	if ((REG_SERCOM1_I2CS_INTFLAG & 4) == 1) //Bit 2 – DRDY: Data Ready
	{
		if ((REG_SERCOM1_I2CS_STATUS & 8) == 1) //DIR == 1 = Master read operation is in progress.
		{
			if (ESPbyteIndex <  ESPpacketlength)
				REG_SERCOM1_I2CS_DATA = *(uint8_t*)(EspPacket + ESPbyteIndex); //write data
			else
				REG_SERCOM1_I2CS_DATA = 0xFF;
				
			if (ESPbyteIndex == 00)
				temp =  *(uint8_t*)(EspPacket); //first byte of data packet.
					
			if (ESPbyteIndex <=  ESPpacketlength)	
				ESPbyteIndex++;
		}
		else
			ESPbyteIndex = REG_SERCOM1_I2CS_DATA; //read data
	}
	
	if ((REG_SERCOM1_I2CS_INTFLAG & 1) == 1) //Bit 0 – PREC: Stop Received. This flag is set when a stop condition is detected for a transaction being processed
	{
		REG_SERCOM1_I2CS_INTFLAG = REG_SERCOM1_I2CS_INTFLAG | 1; //Writing a one to this bit will clear the Stop Received interrupt flag.
		if (ESPbyteIndex > ESPpacketlength)
		{
			if (temp != 0)
				*(uint8_t*)(EspPacket) = 0;
		}
		ESPbyteIndex = 0;
	}
}

void irq_handler_dmac(void)
{
	//TODO implement function. We process our ADC results here!
}

void sendESPpacket()
{
	//TODO create this routine
}

void  enableDMA ()
{
	DMAdescriptor.BTCNT = 8;//BTCNT, number of beats per transaction. We're moving 8 ADC results each time.
	DMAdescriptor.SRCADDR = REG_ADC_RESULT;//Source address
	DMAdescriptor.DSTADDR = &DMAresults + 1 ;//Destination address + (transaction length), see manual
	DMAdescriptor.DESCADDR = 0;	
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
	REG_SERCOM1_I2CS_CTRLB = 0x500; // 0000 0101 00000000, Send ACK. Automatic acknowledge is enabled.Group command is disabled.Smart mode is enabled.
	do {
  	} while (REG_SERCOM1_I2CS_SYNCBUSY != 0);
  	
  	REG_SERCOM1_I2CS_ADDR = 0xC8; //the as address
  	REG_SERCOM1_I2CS_CTRLA = 0x100012;//0001 0000 - 0000 0000 - 0001 0010, slave config(!), 50-100ns hold time, enable, 
  					   //Standard-mode (Sm) up to 100 kHz and Fast-mode (Fm) up to 400 kHz
	do {
  	} while (REG_SERCOM1_I2CS_SYNCBUSY != 0);
  	
  	REG_NVIC_PRIO2 = (REG_NVIC_PRIO2 & 0xFF00FFFF) | 0xC00000;
  	REG_NVIC_SETENA = 0x400; //Enable interrupt: 0100 00000000 = int 10 = our SerCom IRQ.
  	REG_SERCOM1_I2CS_INTENSET = 5;//enables the Data Ready interrupt and the Stop Received interrupt
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
	REG_DMAC_BASEADDR = &DMAdescriptor; 
	DMAdescriptor.BTCTRL = 0x909; //0000 1001 0000 1001.  stepsize = 0, So next address = beatsize*1
					   //STEPSEL = 0, so DST
					   // DSTINC = 1, so auto increment destination
					   // SRCINC = 0, so no auto increment on source.
					   //Beatsize = 16 bit
					   // blockact = 1 = INT = Channel in normal operation and block interrupt
					   //Event Output Selection = 0, DISABLE, no event generation
					   //valid.
	REG_DMAC_WRBADDR  = &DMAdescriptorwriteback; //Write-Back Memory Section Base Address, 0x10 bytes long
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

int main(void)
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
	return 0;
}
