# 1 "include/samd10.h"
# 1 "<built-in>"
# 1 "<command-line>"
# 1 "/usr/include/stdc-predef.h" 1 3 4
# 1 "<command-line>" 2
# 1 "include/samd10.h"
# 58 "include/samd10.h"
# 1 "include/samd10c13a.h" 1
# 66 "include/samd10c13a.h"
# 1 "/usr/lib/gcc/x86_64-linux-gnu/5/include/stdint.h" 1 3 4
# 9 "/usr/lib/gcc/x86_64-linux-gnu/5/include/stdint.h" 3 4
# 1 "/usr/include/stdint.h" 1 3 4
# 25 "/usr/include/stdint.h" 3 4
# 1 "/usr/include/features.h" 1 3 4
# 367 "/usr/include/features.h" 3 4
# 1 "/usr/include/x86_64-linux-gnu/sys/cdefs.h" 1 3 4
# 410 "/usr/include/x86_64-linux-gnu/sys/cdefs.h" 3 4
# 1 "/usr/include/x86_64-linux-gnu/bits/wordsize.h" 1 3 4
# 411 "/usr/include/x86_64-linux-gnu/sys/cdefs.h" 2 3 4
# 368 "/usr/include/features.h" 2 3 4
# 391 "/usr/include/features.h" 3 4
# 1 "/usr/include/x86_64-linux-gnu/gnu/stubs.h" 1 3 4
# 10 "/usr/include/x86_64-linux-gnu/gnu/stubs.h" 3 4
# 1 "/usr/include/x86_64-linux-gnu/gnu/stubs-64.h" 1 3 4
# 11 "/usr/include/x86_64-linux-gnu/gnu/stubs.h" 2 3 4
# 392 "/usr/include/features.h" 2 3 4
# 26 "/usr/include/stdint.h" 2 3 4
# 1 "/usr/include/x86_64-linux-gnu/bits/wchar.h" 1 3 4
# 27 "/usr/include/stdint.h" 2 3 4
# 1 "/usr/include/x86_64-linux-gnu/bits/wordsize.h" 1 3 4
# 28 "/usr/include/stdint.h" 2 3 4
# 36 "/usr/include/stdint.h" 3 4

# 36 "/usr/include/stdint.h" 3 4
typedef signed char int8_t;
typedef short int int16_t;
typedef int int32_t;

typedef long int int64_t;







typedef unsigned char uint8_t;
typedef unsigned short int uint16_t;

typedef unsigned int uint32_t;



typedef unsigned long int uint64_t;
# 65 "/usr/include/stdint.h" 3 4
typedef signed char int_least8_t;
typedef short int int_least16_t;
typedef int int_least32_t;

typedef long int int_least64_t;






typedef unsigned char uint_least8_t;
typedef unsigned short int uint_least16_t;
typedef unsigned int uint_least32_t;

typedef unsigned long int uint_least64_t;
# 90 "/usr/include/stdint.h" 3 4
typedef signed char int_fast8_t;

typedef long int int_fast16_t;
typedef long int int_fast32_t;
typedef long int int_fast64_t;
# 103 "/usr/include/stdint.h" 3 4
typedef unsigned char uint_fast8_t;

typedef unsigned long int uint_fast16_t;
typedef unsigned long int uint_fast32_t;
typedef unsigned long int uint_fast64_t;
# 119 "/usr/include/stdint.h" 3 4
typedef long int intptr_t;


typedef unsigned long int uintptr_t;
# 134 "/usr/include/stdint.h" 3 4
typedef long int intmax_t;
typedef unsigned long int uintmax_t;
# 10 "/usr/lib/gcc/x86_64-linux-gnu/5/include/stdint.h" 2 3 4
# 67 "include/samd10c13a.h" 2


# 68 "include/samd10c13a.h"
typedef volatile const uint32_t RoReg;
typedef volatile const uint16_t RoReg16;
typedef volatile const uint8_t RoReg8;





typedef volatile uint32_t WoReg;
typedef volatile uint16_t WoReg16;
typedef volatile uint32_t WoReg8;
typedef volatile uint32_t RwReg;
typedef volatile uint16_t RwReg16;
typedef volatile uint8_t RwReg8;
# 96 "include/samd10c13a.h"
typedef enum IRQn
{

  NonMaskableInt_IRQn = -14,
  HardFault_IRQn = -13,
  SVCall_IRQn = -5,
  PendSV_IRQn = -2,
  SysTick_IRQn = -1,

  PM_IRQn = 0,
  SYSCTRL_IRQn = 1,
  WDT_IRQn = 2,
  RTC_IRQn = 3,
  EIC_IRQn = 4,
  NVMCTRL_IRQn = 5,
  DMAC_IRQn = 6,
  EVSYS_IRQn = 8,
  SERCOM0_IRQn = 9,
  SERCOM1_IRQn = 10,
  TCC0_IRQn = 12,
  TC1_IRQn = 13,
  TC2_IRQn = 14,
  ADC_IRQn = 15,
  AC_IRQn = 16,
  DAC_IRQn = 17,
  PTC_IRQn = 18,

  PERIPH_COUNT_IRQn = 19
} IRQn_Type;

typedef struct _DeviceVectors
{

  void* pvStack;


  void* pfnReset_Handler;
  void* pfnNMI_Handler;
  void* pfnHardFault_Handler;
  void* pfnReservedM12;
  void* pfnReservedM11;
  void* pfnReservedM10;
  void* pfnReservedM9;
  void* pfnReservedM8;
  void* pfnReservedM7;
  void* pfnReservedM6;
  void* pfnSVC_Handler;
  void* pfnReservedM4;
  void* pfnReservedM3;
  void* pfnPendSV_Handler;
  void* pfnSysTick_Handler;


  void* pfnPM_Handler;
  void* pfnSYSCTRL_Handler;
  void* pfnWDT_Handler;
  void* pfnRTC_Handler;
  void* pfnEIC_Handler;
  void* pfnNVMCTRL_Handler;
  void* pfnDMAC_Handler;
  void* pfnReserved7;
  void* pfnEVSYS_Handler;
  void* pfnSERCOM0_Handler;
  void* pfnSERCOM1_Handler;
  void* pfnReserved11;
  void* pfnTCC0_Handler;
  void* pfnTC1_Handler;
  void* pfnTC2_Handler;
  void* pfnADC_Handler;
  void* pfnAC_Handler;
  void* pfnDAC_Handler;
  void* pfnPTC_Handler;
} DeviceVectors;


void Reset_Handler ( void );
void NMI_Handler ( void );
void HardFault_Handler ( void );
void SVC_Handler ( void );
void PendSV_Handler ( void );
void SysTick_Handler ( void );


void PM_Handler ( void );
void SYSCTRL_Handler ( void );
void WDT_Handler ( void );
void RTC_Handler ( void );
void EIC_Handler ( void );
void NVMCTRL_Handler ( void );
void DMAC_Handler ( void );
void EVSYS_Handler ( void );
void SERCOM0_Handler ( void );
void SERCOM1_Handler ( void );
void TCC0_Handler ( void );
void TC1_Handler ( void );
void TC2_Handler ( void );
void ADC_Handler ( void );
void AC_Handler ( void );
void DAC_Handler ( void );
void PTC_Handler ( void );
# 212 "include/samd10c13a.h"
# 1 "include/core_cm0plus.h" 1
# 112 "include/core_cm0plus.h"
# 1 "include/core_cmInstr.h" 1
# 286 "include/core_cmInstr.h"
__attribute__( ( always_inline ) ) static inline void __NOP(void)
{
  __asm volatile ("nop");
}







__attribute__( ( always_inline ) ) static inline void __WFI(void)
{
  __asm volatile ("wfi");
}







__attribute__( ( always_inline ) ) static inline void __WFE(void)
{
  __asm volatile ("wfe");
}






__attribute__( ( always_inline ) ) static inline void __SEV(void)
{
  __asm volatile ("sev");
}
# 330 "include/core_cmInstr.h"
__attribute__( ( always_inline ) ) static inline void __ISB(void)
{
  __asm volatile ("isb");
}







__attribute__( ( always_inline ) ) static inline void __DSB(void)
{
  __asm volatile ("dsb");
}







__attribute__( ( always_inline ) ) static inline void __DMB(void)
{
  __asm volatile ("dmb");
}
# 365 "include/core_cmInstr.h"
__attribute__( ( always_inline ) ) static inline uint32_t __REV(uint32_t value)
{
  uint32_t result;

  __asm volatile ("rev %0, %1" : "=r" (result) : "r" (value) );
  return(result);
}
# 381 "include/core_cmInstr.h"
__attribute__( ( always_inline ) ) static inline uint32_t __REV16(uint32_t value)
{
  uint32_t result;

  __asm volatile ("rev16 %0, %1" : "=r" (result) : "r" (value) );
  return(result);
}
# 397 "include/core_cmInstr.h"
__attribute__( ( always_inline ) ) static inline int32_t __REVSH(int32_t value)
{
  uint32_t result;

  __asm volatile ("revsh %0, %1" : "=r" (result) : "r" (value) );
  return(result);
}
# 414 "include/core_cmInstr.h"
__attribute__( ( always_inline ) ) static inline uint32_t __ROR(uint32_t op1, uint32_t op2)
{

  __asm volatile ("ror %0, %0, %1" : "+r" (op1) : "r" (op2) );
  return(op1);
}
# 113 "include/core_cm0plus.h" 2
# 1 "include/core_cmFunc.h" 1
# 315 "include/core_cmFunc.h"
__attribute__( ( always_inline ) ) static inline void __enable_irq(void)
{
  __asm volatile ("cpsie i");
}







__attribute__( ( always_inline ) ) static inline void __disable_irq(void)
{
  __asm volatile ("cpsid i");
}
# 338 "include/core_cmFunc.h"
__attribute__( ( always_inline ) ) static inline uint32_t __get_CONTROL(void)
{
  uint32_t result;

  __asm volatile ("MRS %0, control" : "=r" (result) );
  return(result);
}
# 353 "include/core_cmFunc.h"
__attribute__( ( always_inline ) ) static inline void __set_CONTROL(uint32_t control)
{
  __asm volatile ("MSR control, %0" : : "r" (control) );
}
# 365 "include/core_cmFunc.h"
__attribute__( ( always_inline ) ) static inline uint32_t __get_IPSR(void)
{
  uint32_t result;

  __asm volatile ("MRS %0, ipsr" : "=r" (result) );
  return(result);
}
# 380 "include/core_cmFunc.h"
__attribute__( ( always_inline ) ) static inline uint32_t __get_APSR(void)
{
  uint32_t result;

  __asm volatile ("MRS %0, apsr" : "=r" (result) );
  return(result);
}
# 395 "include/core_cmFunc.h"
__attribute__( ( always_inline ) ) static inline uint32_t __get_xPSR(void)
{
  uint32_t result;

  __asm volatile ("MRS %0, xpsr" : "=r" (result) );
  return(result);
}
# 410 "include/core_cmFunc.h"
__attribute__( ( always_inline ) ) static inline uint32_t __get_PSP(void)
{
  register uint32_t result;

  __asm volatile ("MRS %0, psp\n" : "=r" (result) );
  return(result);
}
# 425 "include/core_cmFunc.h"
__attribute__( ( always_inline ) ) static inline void __set_PSP(uint32_t topOfProcStack)
{
  __asm volatile ("MSR psp, %0\n" : : "r" (topOfProcStack) );
}
# 437 "include/core_cmFunc.h"
__attribute__( ( always_inline ) ) static inline uint32_t __get_MSP(void)
{
  register uint32_t result;

  __asm volatile ("MRS %0, msp\n" : "=r" (result) );
  return(result);
}
# 452 "include/core_cmFunc.h"
__attribute__( ( always_inline ) ) static inline void __set_MSP(uint32_t topOfMainStack)
{
  __asm volatile ("MSR msp, %0\n" : : "r" (topOfMainStack) );
}
# 464 "include/core_cmFunc.h"
__attribute__( ( always_inline ) ) static inline uint32_t __get_PRIMASK(void)
{
  uint32_t result;

  __asm volatile ("MRS %0, primask" : "=r" (result) );
  return(result);
}
# 479 "include/core_cmFunc.h"
__attribute__( ( always_inline ) ) static inline void __set_PRIMASK(uint32_t priMask)
{
  __asm volatile ("MSR primask, %0" : : "r" (priMask) );
}
# 114 "include/core_cm0plus.h" 2
# 191 "include/core_cm0plus.h"
typedef union
{
  struct
  {

    uint32_t _reserved0:27;





    uint32_t Q:1;
    uint32_t V:1;
    uint32_t C:1;
    uint32_t Z:1;
    uint32_t N:1;
  } b;
  uint32_t w;
} APSR_Type;




typedef union
{
  struct
  {
    uint32_t ISR:9;
    uint32_t _reserved0:23;
  } b;
  uint32_t w;
} IPSR_Type;




typedef union
{
  struct
  {
    uint32_t ISR:9;

    uint32_t _reserved0:15;





    uint32_t T:1;
    uint32_t IT:2;
    uint32_t Q:1;
    uint32_t V:1;
    uint32_t C:1;
    uint32_t Z:1;
    uint32_t N:1;
  } b;
  uint32_t w;
} xPSR_Type;




typedef union
{
  struct
  {
    uint32_t nPRIV:1;
    uint32_t SPSEL:1;
    uint32_t FPCA:1;
    uint32_t _reserved0:29;
  } b;
  uint32_t w;
} CONTROL_Type;
# 276 "include/core_cm0plus.h"
typedef struct
{
  volatile uint32_t ISER[1];
       uint32_t RESERVED0[31];
  volatile uint32_t ICER[1];
       uint32_t RSERVED1[31];
  volatile uint32_t ISPR[1];
       uint32_t RESERVED2[31];
  volatile uint32_t ICPR[1];
       uint32_t RESERVED3[31];
       uint32_t RESERVED4[64];
  volatile uint32_t IP[8];
} NVIC_Type;
# 301 "include/core_cm0plus.h"
typedef struct
{
  volatile const uint32_t CPUID;
  volatile uint32_t ICSR;

  volatile uint32_t VTOR;



  volatile uint32_t AIRCR;
  volatile uint32_t SCR;
  volatile uint32_t CCR;
       uint32_t RESERVED1;
  volatile uint32_t SHP[2];
  volatile uint32_t SHCSR;
} SCB_Type;
# 416 "include/core_cm0plus.h"
typedef struct
{
  volatile uint32_t CTRL;
  volatile uint32_t LOAD;
  volatile uint32_t VAL;
  volatile const uint32_t CALIB;
} SysTick_Type;
# 611 "include/core_cm0plus.h"
static inline void NVIC_EnableIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISER[0] = (1 << ((uint32_t)(IRQn) & 0x1F));
}
# 623 "include/core_cm0plus.h"
static inline void NVIC_DisableIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ICER[0] = (1 << ((uint32_t)(IRQn) & 0x1F));
}
# 639 "include/core_cm0plus.h"
static inline uint32_t NVIC_GetPendingIRQ(IRQn_Type IRQn)
{
  return((uint32_t) ((((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISPR[0] & (1 << ((uint32_t)(IRQn) & 0x1F)))?1:0));
}
# 651 "include/core_cm0plus.h"
static inline void NVIC_SetPendingIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ISPR[0] = (1 << ((uint32_t)(IRQn) & 0x1F));
}
# 663 "include/core_cm0plus.h"
static inline void NVIC_ClearPendingIRQ(IRQn_Type IRQn)
{
  ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->ICPR[0] = (1 << ((uint32_t)(IRQn) & 0x1F));
}
# 678 "include/core_cm0plus.h"
static inline void NVIC_SetPriority(IRQn_Type IRQn, uint32_t priority)
{
  if(IRQn < 0) {
    ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SHP[( ((((uint32_t)(IRQn) & 0x0F)-8) >> 2) )] = (((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SHP[( ((((uint32_t)(IRQn) & 0x0F)-8) >> 2) )] & ~(0xFF << ( (((uint32_t)(IRQn) ) & 0x03) * 8 ))) |
        (((priority << (8 - 2)) & 0xFF) << ( (((uint32_t)(IRQn) ) & 0x03) * 8 )); }
  else {
    ((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IP[( ((uint32_t)(IRQn) >> 2) )] = (((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IP[( ((uint32_t)(IRQn) >> 2) )] & ~(0xFF << ( (((uint32_t)(IRQn) ) & 0x03) * 8 ))) |
        (((priority << (8 - 2)) & 0xFF) << ( (((uint32_t)(IRQn) ) & 0x03) * 8 )); }
}
# 700 "include/core_cm0plus.h"
static inline uint32_t NVIC_GetPriority(IRQn_Type IRQn)
{

  if(IRQn < 0) {
    return((uint32_t)((((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->SHP[( ((((uint32_t)(IRQn) & 0x0F)-8) >> 2) )] >> ( (((uint32_t)(IRQn) ) & 0x03) * 8 ) ) >> (8 - 2))); }
  else {
    return((uint32_t)((((NVIC_Type *) ((0xE000E000UL) + 0x0100UL) )->IP[ ( ((uint32_t)(IRQn) >> 2) )] >> ( (((uint32_t)(IRQn) ) & 0x03) * 8 ) ) >> (8 - 2))); }
}






static inline void NVIC_SystemReset(void)
{
  __DSB();

  ((SCB_Type *) ((0xE000E000UL) + 0x0D00UL) )->AIRCR = ((0x5FA << 16) |
                 (1UL << 2));
  __DSB();
  while(1);
}
# 752 "include/core_cm0plus.h"
static inline uint32_t SysTick_Config(uint32_t ticks)
{
  if (ticks > (0xFFFFFFUL << 0)) return (1);

  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->LOAD = (ticks & (0xFFFFFFUL << 0)) - 1;
  NVIC_SetPriority (SysTick_IRQn, (1<<2) - 1);
  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->VAL = 0;
  ((SysTick_Type *) ((0xE000E000UL) + 0x0010UL) )->CTRL = (1UL << 2) |
                   (1UL << 1) |
                   (1UL << 0);
  return (0);
}
# 213 "include/samd10c13a.h" 2
# 225 "include/samd10c13a.h"
# 1 "include/component/ac.h" 1
# 61 "include/component/ac.h"
typedef union {
  struct {
    uint8_t SWRST:1;
    uint8_t ENABLE:1;
    uint8_t RUNSTDBY:1;
    uint8_t :4;
    uint8_t LPMUX:1;
  } bit;
  uint8_t reg;
} AC_CTRLA_Type;
# 89 "include/component/ac.h"
typedef union {
  struct {
    uint8_t START0:1;
    uint8_t START1:1;
    uint8_t :6;
  } bit;
  struct {
    uint8_t START:2;
    uint8_t :6;
  } vec;
  uint8_t reg;
} AC_CTRLB_Type;
# 117 "include/component/ac.h"
typedef union {
  struct {
    uint16_t COMPEO0:1;
    uint16_t COMPEO1:1;
    uint16_t :2;
    uint16_t WINEO0:1;
    uint16_t :3;
    uint16_t COMPEI0:1;
    uint16_t COMPEI1:1;
    uint16_t :6;
  } bit;
  struct {
    uint16_t COMPEO:2;
    uint16_t :2;
    uint16_t WINEO:1;
    uint16_t :3;
    uint16_t COMPEI:2;
    uint16_t :6;
  } vec;
  uint16_t reg;
} AC_EVCTRL_Type;
# 166 "include/component/ac.h"
typedef union {
  struct {
    uint8_t COMP0:1;
    uint8_t COMP1:1;
    uint8_t :2;
    uint8_t WIN0:1;
    uint8_t :3;
  } bit;
  struct {
    uint8_t COMP:2;
    uint8_t :2;
    uint8_t WIN:1;
    uint8_t :3;
  } vec;
  uint8_t reg;
} AC_INTENCLR_Type;
# 203 "include/component/ac.h"
typedef union {
  struct {
    uint8_t COMP0:1;
    uint8_t COMP1:1;
    uint8_t :2;
    uint8_t WIN0:1;
    uint8_t :3;
  } bit;
  struct {
    uint8_t COMP:2;
    uint8_t :2;
    uint8_t WIN:1;
    uint8_t :3;
  } vec;
  uint8_t reg;
} AC_INTENSET_Type;
# 240 "include/component/ac.h"
typedef union {
  struct {
    volatile const uint8_t COMP0:1;
    volatile const uint8_t COMP1:1;
    volatile const uint8_t :2;
    volatile const uint8_t WIN0:1;
    volatile const uint8_t :3;
  } bit;
  struct {
    volatile const uint8_t COMP:2;
    volatile const uint8_t :2;
    volatile const uint8_t WIN:1;
    volatile const uint8_t :3;
  } vec;
  uint8_t reg;
} AC_INTFLAG_Type;
# 277 "include/component/ac.h"
typedef union {
  struct {
    uint8_t STATE0:1;
    uint8_t STATE1:1;
    uint8_t :2;
    uint8_t WSTATE0:2;
    uint8_t :2;
  } bit;
  struct {
    uint8_t STATE:2;
    uint8_t :6;
  } vec;
  uint8_t reg;
} AC_STATUSA_Type;
# 316 "include/component/ac.h"
typedef union {
  struct {
    uint8_t READY0:1;
    uint8_t READY1:1;
    uint8_t :5;
    uint8_t SYNCBUSY:1;
  } bit;
  struct {
    uint8_t READY:2;
    uint8_t :6;
  } vec;
  uint8_t reg;
} AC_STATUSB_Type;
# 347 "include/component/ac.h"
typedef union {
  struct {
    uint8_t STATE0:1;
    uint8_t STATE1:1;
    uint8_t :2;
    uint8_t WSTATE0:2;
    uint8_t :2;
  } bit;
  struct {
    uint8_t STATE:2;
    uint8_t :6;
  } vec;
  uint8_t reg;
} AC_STATUSC_Type;
# 386 "include/component/ac.h"
typedef union {
  struct {
    uint8_t WEN0:1;
    uint8_t WINTSEL0:2;
    uint8_t :5;
  } bit;
  uint8_t reg;
} AC_WINCTRL_Type;
# 416 "include/component/ac.h"
typedef union {
  struct {
    uint32_t ENABLE:1;
    uint32_t SINGLE:1;
    uint32_t SPEED:2;
    uint32_t :1;
    uint32_t INTSEL:2;
    uint32_t :1;
    uint32_t MUXNEG:3;
    uint32_t :1;
    uint32_t MUXPOS:2;
    uint32_t :1;
    uint32_t SWAP:1;
    uint32_t OUT:2;
    uint32_t :1;
    uint32_t HYST:1;
    uint32_t :4;
    uint32_t FLEN:3;
    uint32_t :5;
  } bit;
  uint32_t reg;
} AC_COMPCTRL_Type;
# 521 "include/component/ac.h"
typedef union {
  struct {
    uint8_t VALUE:6;
    uint8_t :2;
  } bit;
  uint8_t reg;
} AC_SCALER_Type;
# 540 "include/component/ac.h"
typedef struct {
  volatile AC_CTRLA_Type CTRLA;
  volatile AC_CTRLB_Type CTRLB;
  volatile AC_EVCTRL_Type EVCTRL;
  volatile AC_INTENCLR_Type INTENCLR;
  volatile AC_INTENSET_Type INTENSET;
  volatile AC_INTFLAG_Type INTFLAG;
       RoReg8 Reserved1[0x1];
  volatile const AC_STATUSA_Type STATUSA;
  volatile const AC_STATUSB_Type STATUSB;
  volatile const AC_STATUSC_Type STATUSC;
       RoReg8 Reserved2[0x1];
  volatile AC_WINCTRL_Type WINCTRL;
       RoReg8 Reserved3[0x3];
  volatile AC_COMPCTRL_Type COMPCTRL[2];
       RoReg8 Reserved4[0x8];
  volatile AC_SCALER_Type SCALER[2];
} Ac;
# 226 "include/samd10c13a.h" 2
# 1 "include/component/adc.h" 1
# 61 "include/component/adc.h"
typedef union {
  struct {
    uint8_t SWRST:1;
    uint8_t ENABLE:1;
    uint8_t RUNSTDBY:1;
    uint8_t :5;
  } bit;
  uint8_t reg;
} ADC_CTRLA_Type;
# 85 "include/component/adc.h"
typedef union {
  struct {
    uint8_t REFSEL:4;
    uint8_t :3;
    uint8_t REFCOMP:1;
  } bit;
  uint8_t reg;
} ADC_REFCTRL_Type;
# 117 "include/component/adc.h"
typedef union {
  struct {
    uint8_t SAMPLENUM:4;
    uint8_t ADJRES:3;
    uint8_t :1;
  } bit;
  uint8_t reg;
} ADC_AVGCTRL_Type;
# 162 "include/component/adc.h"
typedef union {
  struct {
    uint8_t SAMPLEN:6;
    uint8_t :2;
  } bit;
  uint8_t reg;
} ADC_SAMPCTRL_Type;
# 181 "include/component/adc.h"
typedef union {
  struct {
    uint16_t DIFFMODE:1;
    uint16_t LEFTADJ:1;
    uint16_t FREERUN:1;
    uint16_t CORREN:1;
    uint16_t RESSEL:2;
    uint16_t :2;
    uint16_t PRESCALER:3;
    uint16_t :5;
  } bit;
  uint16_t reg;
} ADC_CTRLB_Type;
# 241 "include/component/adc.h"
typedef union {
  struct {
    uint8_t WINMODE:3;
    uint8_t :5;
  } bit;
  uint8_t reg;
} ADC_WINCTRL_Type;
# 270 "include/component/adc.h"
typedef union {
  struct {
    uint8_t FLUSH:1;
    uint8_t START:1;
    uint8_t :6;
  } bit;
  uint8_t reg;
} ADC_SWTRIG_Type;
# 291 "include/component/adc.h"
typedef union {
  struct {
    uint32_t MUXPOS:5;
    uint32_t :3;
    uint32_t MUXNEG:5;
    uint32_t :3;
    uint32_t INPUTSCAN:4;
    uint32_t INPUTOFFSET:4;
    uint32_t GAIN:4;
    uint32_t :4;
  } bit;
  uint32_t reg;
} ADC_INPUTCTRL_Type;
# 410 "include/component/adc.h"
typedef union {
  struct {
    uint8_t STARTEI:1;
    uint8_t SYNCEI:1;
    uint8_t :2;
    uint8_t RESRDYEO:1;
    uint8_t WINMONEO:1;
    uint8_t :2;
  } bit;
  uint8_t reg;
} ADC_EVCTRL_Type;
# 438 "include/component/adc.h"
typedef union {
  struct {
    uint8_t RESRDY:1;
    uint8_t OVERRUN:1;
    uint8_t WINMON:1;
    uint8_t SYNCRDY:1;
    uint8_t :4;
  } bit;
  uint8_t reg;
} ADC_INTENCLR_Type;
# 465 "include/component/adc.h"
typedef union {
  struct {
    uint8_t RESRDY:1;
    uint8_t OVERRUN:1;
    uint8_t WINMON:1;
    uint8_t SYNCRDY:1;
    uint8_t :4;
  } bit;
  uint8_t reg;
} ADC_INTENSET_Type;
# 492 "include/component/adc.h"
typedef union {
  struct {
    volatile const uint8_t RESRDY:1;
    volatile const uint8_t OVERRUN:1;
    volatile const uint8_t WINMON:1;
    volatile const uint8_t SYNCRDY:1;
    volatile const uint8_t :4;
  } bit;
  uint8_t reg;
} ADC_INTFLAG_Type;
# 519 "include/component/adc.h"
typedef union {
  struct {
    uint8_t :7;
    uint8_t SYNCBUSY:1;
  } bit;
  uint8_t reg;
} ADC_STATUS_Type;
# 537 "include/component/adc.h"
typedef union {
  struct {
    uint16_t RESULT:16;
  } bit;
  uint16_t reg;
} ADC_RESULT_Type;
# 555 "include/component/adc.h"
typedef union {
  struct {
    uint16_t WINLT:16;
  } bit;
  uint16_t reg;
} ADC_WINLT_Type;
# 573 "include/component/adc.h"
typedef union {
  struct {
    uint16_t WINUT:16;
  } bit;
  uint16_t reg;
} ADC_WINUT_Type;
# 591 "include/component/adc.h"
typedef union {
  struct {
    uint16_t GAINCORR:12;
    uint16_t :4;
  } bit;
  uint16_t reg;
} ADC_GAINCORR_Type;
# 610 "include/component/adc.h"
typedef union {
  struct {
    uint16_t OFFSETCORR:12;
    uint16_t :4;
  } bit;
  uint16_t reg;
} ADC_OFFSETCORR_Type;
# 629 "include/component/adc.h"
typedef union {
  struct {
    uint16_t LINEARITY_CAL:8;
    uint16_t BIAS_CAL:3;
    uint16_t :5;
  } bit;
  uint16_t reg;
} ADC_CALIB_Type;
# 652 "include/component/adc.h"
typedef union {
  struct {
    uint8_t DBGRUN:1;
    uint8_t :7;
  } bit;
  uint8_t reg;
} ADC_DBGCTRL_Type;
# 670 "include/component/adc.h"
typedef struct {
  volatile ADC_CTRLA_Type CTRLA;
  volatile ADC_REFCTRL_Type REFCTRL;
  volatile ADC_AVGCTRL_Type AVGCTRL;
  volatile ADC_SAMPCTRL_Type SAMPCTRL;
  volatile ADC_CTRLB_Type CTRLB;
       RoReg8 Reserved1[0x2];
  volatile ADC_WINCTRL_Type WINCTRL;
       RoReg8 Reserved2[0x3];
  volatile ADC_SWTRIG_Type SWTRIG;
       RoReg8 Reserved3[0x3];
  volatile ADC_INPUTCTRL_Type INPUTCTRL;
  volatile ADC_EVCTRL_Type EVCTRL;
       RoReg8 Reserved4[0x1];
  volatile ADC_INTENCLR_Type INTENCLR;
  volatile ADC_INTENSET_Type INTENSET;
  volatile ADC_INTFLAG_Type INTFLAG;
  volatile const ADC_STATUS_Type STATUS;
  volatile const ADC_RESULT_Type RESULT;
  volatile ADC_WINLT_Type WINLT;
       RoReg8 Reserved5[0x2];
  volatile ADC_WINUT_Type WINUT;
       RoReg8 Reserved6[0x2];
  volatile ADC_GAINCORR_Type GAINCORR;
  volatile ADC_OFFSETCORR_Type OFFSETCORR;
  volatile ADC_CALIB_Type CALIB;
  volatile ADC_DBGCTRL_Type DBGCTRL;
} Adc;
# 227 "include/samd10c13a.h" 2
# 1 "include/component/dac.h" 1
# 61 "include/component/dac.h"
typedef union {
  struct {
    uint8_t SWRST:1;
    uint8_t ENABLE:1;
    uint8_t RUNSTDBY:1;
    uint8_t :5;
  } bit;
  uint8_t reg;
} DAC_CTRLA_Type;
# 85 "include/component/dac.h"
typedef union {
  struct {
    uint8_t EOEN:1;
    uint8_t IOEN:1;
    uint8_t LEFTADJ:1;
    uint8_t VPD:1;
    uint8_t BDWP:1;
    uint8_t :1;
    uint8_t REFSEL:2;
  } bit;
  uint8_t reg;
} DAC_CTRLB_Type;
# 125 "include/component/dac.h"
typedef union {
  struct {
    uint8_t STARTEI:1;
    uint8_t EMPTYEO:1;
    uint8_t :6;
  } bit;
  uint8_t reg;
} DAC_EVCTRL_Type;
# 146 "include/component/dac.h"
typedef union {
  struct {
    uint8_t UNDERRUN:1;
    uint8_t EMPTY:1;
    uint8_t SYNCRDY:1;
    uint8_t :5;
  } bit;
  uint8_t reg;
} DAC_INTENCLR_Type;
# 170 "include/component/dac.h"
typedef union {
  struct {
    uint8_t UNDERRUN:1;
    uint8_t EMPTY:1;
    uint8_t SYNCRDY:1;
    uint8_t :5;
  } bit;
  uint8_t reg;
} DAC_INTENSET_Type;
# 194 "include/component/dac.h"
typedef union {
  struct {
    volatile const uint8_t UNDERRUN:1;
    volatile const uint8_t EMPTY:1;
    volatile const uint8_t SYNCRDY:1;
    volatile const uint8_t :5;
  } bit;
  uint8_t reg;
} DAC_INTFLAG_Type;
# 218 "include/component/dac.h"
typedef union {
  struct {
    uint8_t :7;
    uint8_t SYNCBUSY:1;
  } bit;
  uint8_t reg;
} DAC_STATUS_Type;
# 236 "include/component/dac.h"
typedef union {
  struct {
    uint16_t DATA:16;
  } bit;
  uint16_t reg;
} DAC_DATA_Type;
# 254 "include/component/dac.h"
typedef union {
  struct {
    uint16_t DATABUF:16;
  } bit;
  uint16_t reg;
} DAC_DATABUF_Type;
# 272 "include/component/dac.h"
typedef struct {
  volatile DAC_CTRLA_Type CTRLA;
  volatile DAC_CTRLB_Type CTRLB;
  volatile DAC_EVCTRL_Type EVCTRL;
       RoReg8 Reserved1[0x1];
  volatile DAC_INTENCLR_Type INTENCLR;
  volatile DAC_INTENSET_Type INTENSET;
  volatile DAC_INTFLAG_Type INTFLAG;
  volatile const DAC_STATUS_Type STATUS;
  volatile DAC_DATA_Type DATA;
       RoReg8 Reserved2[0x2];
  volatile DAC_DATABUF_Type DATABUF;
} Dac;
# 228 "include/samd10c13a.h" 2
# 1 "include/component/dmac.h" 1
# 61 "include/component/dmac.h"
typedef union {
  struct {
    uint16_t SWRST:1;
    uint16_t DMAENABLE:1;
    uint16_t CRCENABLE:1;
    uint16_t :5;
    uint16_t LVLEN0:1;
    uint16_t LVLEN1:1;
    uint16_t LVLEN2:1;
    uint16_t LVLEN3:1;
    uint16_t :4;
  } bit;
  struct {
    uint16_t :8;
    uint16_t LVLEN:4;
    uint16_t :4;
  } vec;
  uint16_t reg;
} DMAC_CTRL_Type;
# 106 "include/component/dmac.h"
typedef union {
  struct {
    uint16_t CRCBEATSIZE:2;
    uint16_t CRCPOLY:2;
    uint16_t :4;
    uint16_t CRCSRC:6;
    uint16_t :2;
  } bit;
  uint16_t reg;
} DMAC_CRCCTRL_Type;
# 148 "include/component/dmac.h"
typedef union {
  struct {
    uint32_t CRCDATAIN:32;
  } bit;
  uint32_t reg;
} DMAC_CRCDATAIN_Type;
# 166 "include/component/dmac.h"
typedef union {
  struct {
    uint32_t CRCCHKSUM:32;
  } bit;
  uint32_t reg;
} DMAC_CRCCHKSUM_Type;
# 184 "include/component/dmac.h"
typedef union {
  struct {
    uint8_t CRCBUSY:1;
    uint8_t CRCZERO:1;
    uint8_t :6;
  } bit;
  uint8_t reg;
} DMAC_CRCSTATUS_Type;
# 205 "include/component/dmac.h"
typedef union {
  struct {
    uint8_t DBGRUN:1;
    uint8_t :7;
  } bit;
  uint8_t reg;
} DMAC_DBGCTRL_Type;
# 223 "include/component/dmac.h"
typedef union {
  struct {
    uint8_t WRBQOS:2;
    uint8_t FQOS:2;
    uint8_t DQOS:2;
    uint8_t :2;
  } bit;
  uint8_t reg;
} DMAC_QOSCTRL_Type;
# 274 "include/component/dmac.h"
typedef union {
  struct {
    uint32_t SWTRIG0:1;
    uint32_t SWTRIG1:1;
    uint32_t SWTRIG2:1;
    uint32_t SWTRIG3:1;
    uint32_t SWTRIG4:1;
    uint32_t SWTRIG5:1;
    uint32_t :26;
  } bit;
  struct {
    uint32_t SWTRIG:6;
    uint32_t :26;
  } vec;
  uint32_t reg;
} DMAC_SWTRIGCTRL_Type;
# 314 "include/component/dmac.h"
typedef union {
  struct {
    uint32_t LVLPRI0:3;
    uint32_t :4;
    uint32_t RRLVLEN0:1;
    uint32_t LVLPRI1:3;
    uint32_t :4;
    uint32_t RRLVLEN1:1;
    uint32_t LVLPRI2:3;
    uint32_t :4;
    uint32_t RRLVLEN2:1;
    uint32_t LVLPRI3:3;
    uint32_t :4;
    uint32_t RRLVLEN3:1;
  } bit;
  uint32_t reg;
} DMAC_PRICTRL0_Type;
# 360 "include/component/dmac.h"
typedef union {
  struct {
    uint16_t ID:3;
    uint16_t :5;
    uint16_t TERR:1;
    uint16_t TCMPL:1;
    uint16_t SUSP:1;
    uint16_t :2;
    uint16_t FERR:1;
    uint16_t BUSY:1;
    uint16_t PEND:1;
  } bit;
  uint16_t reg;
} DMAC_INTPEND_Type;
# 398 "include/component/dmac.h"
typedef union {
  struct {
    uint32_t CHINT0:1;
    uint32_t CHINT1:1;
    uint32_t CHINT2:1;
    uint32_t CHINT3:1;
    uint32_t CHINT4:1;
    uint32_t CHINT5:1;
    uint32_t :26;
  } bit;
  struct {
    uint32_t CHINT:6;
    uint32_t :26;
  } vec;
  uint32_t reg;
} DMAC_INTSTATUS_Type;
# 438 "include/component/dmac.h"
typedef union {
  struct {
    uint32_t BUSYCH0:1;
    uint32_t BUSYCH1:1;
    uint32_t BUSYCH2:1;
    uint32_t BUSYCH3:1;
    uint32_t BUSYCH4:1;
    uint32_t BUSYCH5:1;
    uint32_t :26;
  } bit;
  struct {
    uint32_t BUSYCH:6;
    uint32_t :26;
  } vec;
  uint32_t reg;
} DMAC_BUSYCH_Type;
# 478 "include/component/dmac.h"
typedef union {
  struct {
    uint32_t PENDCH0:1;
    uint32_t PENDCH1:1;
    uint32_t PENDCH2:1;
    uint32_t PENDCH3:1;
    uint32_t PENDCH4:1;
    uint32_t PENDCH5:1;
    uint32_t :26;
  } bit;
  struct {
    uint32_t PENDCH:6;
    uint32_t :26;
  } vec;
  uint32_t reg;
} DMAC_PENDCH_Type;
# 518 "include/component/dmac.h"
typedef union {
  struct {
    uint32_t LVLEX0:1;
    uint32_t LVLEX1:1;
    uint32_t LVLEX2:1;
    uint32_t LVLEX3:1;
    uint32_t :4;
    uint32_t ID:5;
    uint32_t :2;
    uint32_t ABUSY:1;
    uint32_t BTCNT:16;
  } bit;
  struct {
    uint32_t LVLEX:4;
    uint32_t :28;
  } vec;
  uint32_t reg;
} DMAC_ACTIVE_Type;
# 564 "include/component/dmac.h"
typedef union {
  struct {
    uint32_t BASEADDR:32;
  } bit;
  uint32_t reg;
} DMAC_BASEADDR_Type;
# 582 "include/component/dmac.h"
typedef union {
  struct {
    uint32_t WRBADDR:32;
  } bit;
  uint32_t reg;
} DMAC_WRBADDR_Type;
# 600 "include/component/dmac.h"
typedef union {
  struct {
    uint8_t ID:3;
    uint8_t :5;
  } bit;
  uint8_t reg;
} DMAC_CHID_Type;
# 619 "include/component/dmac.h"
typedef union {
  struct {
    uint8_t SWRST:1;
    uint8_t ENABLE:1;
    uint8_t :6;
  } bit;
  uint8_t reg;
} DMAC_CHCTRLA_Type;
# 640 "include/component/dmac.h"
typedef union {
  struct {
    uint32_t EVACT:3;
    uint32_t EVIE:1;
    uint32_t EVOE:1;
    uint32_t LVL:2;
    uint32_t :1;
    uint32_t TRIGSRC:5;
    uint32_t :9;
    uint32_t TRIGACT:2;
    uint32_t CMD:2;
    uint32_t :6;
  } bit;
  uint32_t reg;
} DMAC_CHCTRLB_Type;
# 711 "include/component/dmac.h"
typedef union {
  struct {
    uint8_t TERR:1;
    uint8_t TCMPL:1;
    uint8_t SUSP:1;
    uint8_t :5;
  } bit;
  uint8_t reg;
} DMAC_CHINTENCLR_Type;
# 735 "include/component/dmac.h"
typedef union {
  struct {
    uint8_t TERR:1;
    uint8_t TCMPL:1;
    uint8_t SUSP:1;
    uint8_t :5;
  } bit;
  uint8_t reg;
} DMAC_CHINTENSET_Type;
# 759 "include/component/dmac.h"
typedef union {
  struct {
    volatile const uint8_t TERR:1;
    volatile const uint8_t TCMPL:1;
    volatile const uint8_t SUSP:1;
    volatile const uint8_t :5;
  } bit;
  uint8_t reg;
} DMAC_CHINTFLAG_Type;
# 783 "include/component/dmac.h"
typedef union {
  struct {
    uint8_t PEND:1;
    uint8_t BUSY:1;
    uint8_t FERR:1;
    uint8_t :5;
  } bit;
  uint8_t reg;
} DMAC_CHSTATUS_Type;
# 807 "include/component/dmac.h"
typedef union {
  struct {
    uint16_t VALID:1;
    uint16_t EVOSEL:2;
    uint16_t BLOCKACT:2;
    uint16_t :3;
    uint16_t BEATSIZE:2;
    uint16_t SRCINC:1;
    uint16_t DSTINC:1;
    uint16_t STEPSEL:1;
    uint16_t STEPSIZE:3;
  } bit;
  uint16_t reg;
} DMAC_BTCTRL_Type;
# 889 "include/component/dmac.h"
typedef union {
  struct {
    uint16_t BTCNT:16;
  } bit;
  uint16_t reg;
} DMAC_BTCNT_Type;
# 906 "include/component/dmac.h"
typedef union {
  struct {
    uint32_t SRCADDR:32;
  } bit;
  uint32_t reg;
} DMAC_SRCADDR_Type;
# 923 "include/component/dmac.h"
typedef union {
  struct {
    uint32_t DSTADDR:32;
  } bit;
  uint32_t reg;
} DMAC_DSTADDR_Type;
# 940 "include/component/dmac.h"
typedef union {
  struct {
    uint32_t DESCADDR:32;
  } bit;
  uint32_t reg;
} DMAC_DESCADDR_Type;
# 957 "include/component/dmac.h"
typedef struct {
  volatile DMAC_CTRL_Type CTRL;
  volatile DMAC_CRCCTRL_Type CRCCTRL;
  volatile DMAC_CRCDATAIN_Type CRCDATAIN;
  volatile DMAC_CRCCHKSUM_Type CRCCHKSUM;
  volatile DMAC_CRCSTATUS_Type CRCSTATUS;
  volatile DMAC_DBGCTRL_Type DBGCTRL;
  volatile DMAC_QOSCTRL_Type QOSCTRL;
       RoReg8 Reserved1[0x1];
  volatile DMAC_SWTRIGCTRL_Type SWTRIGCTRL;
  volatile DMAC_PRICTRL0_Type PRICTRL0;
       RoReg8 Reserved2[0x8];
  volatile DMAC_INTPEND_Type INTPEND;
       RoReg8 Reserved3[0x2];
  volatile const DMAC_INTSTATUS_Type INTSTATUS;
  volatile const DMAC_BUSYCH_Type BUSYCH;
  volatile const DMAC_PENDCH_Type PENDCH;
  volatile const DMAC_ACTIVE_Type ACTIVE;
  volatile DMAC_BASEADDR_Type BASEADDR;
  volatile DMAC_WRBADDR_Type WRBADDR;
       RoReg8 Reserved4[0x3];
  volatile DMAC_CHID_Type CHID;
  volatile DMAC_CHCTRLA_Type CHCTRLA;
       RoReg8 Reserved5[0x3];
  volatile DMAC_CHCTRLB_Type CHCTRLB;
       RoReg8 Reserved6[0x4];
  volatile DMAC_CHINTENCLR_Type CHINTENCLR;
  volatile DMAC_CHINTENSET_Type CHINTENSET;
  volatile DMAC_CHINTFLAG_Type CHINTFLAG;
  volatile const DMAC_CHSTATUS_Type CHSTATUS;
} Dmac;




typedef struct {
  volatile DMAC_BTCTRL_Type BTCTRL;
  volatile DMAC_BTCNT_Type BTCNT;
  volatile DMAC_SRCADDR_Type SRCADDR;
  volatile DMAC_DSTADDR_Type DSTADDR;
  volatile DMAC_DESCADDR_Type DESCADDR;
} DmacDescriptor

  __attribute__ ((aligned (8)))

;
# 229 "include/samd10c13a.h" 2
# 1 "include/component/dsu.h" 1
# 61 "include/component/dsu.h"
typedef union {
  struct {
    uint8_t SWRST:1;
    uint8_t :1;
    uint8_t CRC:1;
    uint8_t MBIST:1;
    uint8_t CE:1;
    uint8_t :1;
    uint8_t ARR:1;
    uint8_t SMSA:1;
  } bit;
  uint8_t reg;
} DSU_CTRL_Type;
# 95 "include/component/dsu.h"
typedef union {
  struct {
    uint8_t DONE:1;
    uint8_t CRSTEXT:1;
    uint8_t BERR:1;
    uint8_t FAIL:1;
    uint8_t PERR:1;
    uint8_t :3;
  } bit;
  uint8_t reg;
} DSU_STATUSA_Type;
# 125 "include/component/dsu.h"
typedef union {
  struct {
    uint8_t PROT:1;
    uint8_t DBGPRES:1;
    uint8_t DCCD0:1;
    uint8_t DCCD1:1;
    uint8_t HPE:1;
    uint8_t :3;
  } bit;
  struct {
    uint8_t :2;
    uint8_t DCCD:2;
    uint8_t :4;
  } vec;
  uint8_t reg;
} DSU_STATUSB_Type;
# 163 "include/component/dsu.h"
typedef union {
  struct {
    uint32_t AMOD:2;
    uint32_t ADDR:30;
  } bit;
  uint32_t reg;
} DSU_ADDR_Type;
# 185 "include/component/dsu.h"
typedef union {
  struct {
    uint32_t :2;
    uint32_t LENGTH:30;
  } bit;
  uint32_t reg;
} DSU_LENGTH_Type;
# 204 "include/component/dsu.h"
typedef union {
  struct {
    uint32_t DATA:32;
  } bit;
  uint32_t reg;
} DSU_DATA_Type;
# 222 "include/component/dsu.h"
typedef union {
  struct {
    uint32_t DATA:32;
  } bit;
  uint32_t reg;
} DSU_DCC_Type;
# 240 "include/component/dsu.h"
typedef union {
  struct {
    uint32_t DEVSEL:8;
    uint32_t REVISION:4;
    uint32_t DIE:4;
    uint32_t SERIES:6;
    uint32_t :1;
    uint32_t FAMILY:5;
    uint32_t PROCESSOR:4;
  } bit;
  uint32_t reg;
} DSU_DID_Type;
# 294 "include/component/dsu.h"
typedef union {
  struct {
    uint32_t DCFG:32;
  } bit;
  uint32_t reg;
} DSU_DCFG_Type;
# 312 "include/component/dsu.h"
typedef union {
  struct {
    uint32_t EPRES:1;
    uint32_t FMT:1;
    uint32_t :10;
    uint32_t ADDOFF:20;
  } bit;
  uint32_t reg;
} DSU_ENTRY_Type;
# 336 "include/component/dsu.h"
typedef union {
  struct {
    uint32_t END:32;
  } bit;
  uint32_t reg;
} DSU_END_Type;
# 354 "include/component/dsu.h"
typedef union {
  struct {
    uint32_t SMEMP:1;
    uint32_t :31;
  } bit;
  uint32_t reg;
} DSU_MEMTYPE_Type;
# 372 "include/component/dsu.h"
typedef union {
  struct {
    uint32_t JEPCC:4;
    uint32_t FKBC:4;
    uint32_t :24;
  } bit;
  uint32_t reg;
} DSU_PID4_Type;
# 395 "include/component/dsu.h"
typedef union {
  uint32_t reg;
} DSU_PID5_Type;







typedef union {
  uint32_t reg;
} DSU_PID6_Type;







typedef union {
  uint32_t reg;
} DSU_PID7_Type;







typedef union {
  struct {
    uint32_t PARTNBL:8;
    uint32_t :24;
  } bit;
  uint32_t reg;
} DSU_PID0_Type;
# 444 "include/component/dsu.h"
typedef union {
  struct {
    uint32_t PARTNBH:4;
    uint32_t JEPIDCL:4;
    uint32_t :24;
  } bit;
  uint32_t reg;
} DSU_PID1_Type;
# 467 "include/component/dsu.h"
typedef union {
  struct {
    uint32_t JEPIDCH:3;
    uint32_t JEPU:1;
    uint32_t REVISION:4;
    uint32_t :24;
  } bit;
  uint32_t reg;
} DSU_PID2_Type;
# 493 "include/component/dsu.h"
typedef union {
  struct {
    uint32_t CUSMOD:4;
    uint32_t REVAND:4;
    uint32_t :24;
  } bit;
  uint32_t reg;
} DSU_PID3_Type;
# 516 "include/component/dsu.h"
typedef union {
  struct {
    uint32_t PREAMBLEB0:8;
    uint32_t :24;
  } bit;
  uint32_t reg;
} DSU_CID0_Type;
# 535 "include/component/dsu.h"
typedef union {
  struct {
    uint32_t PREAMBLE:4;
    uint32_t CCLASS:4;
    uint32_t :24;
  } bit;
  uint32_t reg;
} DSU_CID1_Type;
# 558 "include/component/dsu.h"
typedef union {
  struct {
    uint32_t PREAMBLEB2:8;
    uint32_t :24;
  } bit;
  uint32_t reg;
} DSU_CID2_Type;
# 577 "include/component/dsu.h"
typedef union {
  struct {
    uint32_t PREAMBLEB3:8;
    uint32_t :24;
  } bit;
  uint32_t reg;
} DSU_CID3_Type;
# 596 "include/component/dsu.h"
typedef struct {
  volatile DSU_CTRL_Type CTRL;
  volatile DSU_STATUSA_Type STATUSA;
  volatile const DSU_STATUSB_Type STATUSB;
       RoReg8 Reserved1[0x1];
  volatile DSU_ADDR_Type ADDR;
  volatile DSU_LENGTH_Type LENGTH;
  volatile DSU_DATA_Type DATA;
  volatile DSU_DCC_Type DCC[2];
  volatile const DSU_DID_Type DID;
       RoReg8 Reserved2[0xD4];
  volatile DSU_DCFG_Type DCFG[2];
       RoReg8 Reserved3[0xF08];
  volatile const DSU_ENTRY_Type ENTRY[2];
  volatile const DSU_END_Type END;
       RoReg8 Reserved4[0xFC0];
  volatile const DSU_MEMTYPE_Type MEMTYPE;
  volatile const DSU_PID4_Type PID4;
  volatile const DSU_PID5_Type PID5;
  volatile const DSU_PID6_Type PID6;
  volatile const DSU_PID7_Type PID7;
  volatile const DSU_PID0_Type PID0;
  volatile const DSU_PID1_Type PID1;
  volatile const DSU_PID2_Type PID2;
  volatile const DSU_PID3_Type PID3;
  volatile const DSU_CID0_Type CID0;
  volatile const DSU_CID1_Type CID1;
  volatile const DSU_CID2_Type CID2;
  volatile const DSU_CID3_Type CID3;
} Dsu;
# 230 "include/samd10c13a.h" 2
# 1 "include/component/eic.h" 1
# 61 "include/component/eic.h"
typedef union {
  struct {
    uint8_t SWRST:1;
    uint8_t ENABLE:1;
    uint8_t :6;
  } bit;
  uint8_t reg;
} EIC_CTRL_Type;
# 82 "include/component/eic.h"
typedef union {
  struct {
    uint8_t :7;
    uint8_t SYNCBUSY:1;
  } bit;
  uint8_t reg;
} EIC_STATUS_Type;
# 100 "include/component/eic.h"
typedef union {
  struct {
    uint8_t NMISENSE:3;
    uint8_t NMIFILTEN:1;
    uint8_t :4;
  } bit;
  uint8_t reg;
} EIC_NMICTRL_Type;
# 134 "include/component/eic.h"
typedef union {
  struct {
    uint8_t NMI:1;
    uint8_t :7;
  } bit;
  uint8_t reg;
} EIC_NMIFLAG_Type;
# 152 "include/component/eic.h"
typedef union {
  struct {
    uint32_t EXTINTEO0:1;
    uint32_t EXTINTEO1:1;
    uint32_t EXTINTEO2:1;
    uint32_t EXTINTEO3:1;
    uint32_t EXTINTEO4:1;
    uint32_t EXTINTEO5:1;
    uint32_t EXTINTEO6:1;
    uint32_t EXTINTEO7:1;
    uint32_t :24;
  } bit;
  struct {
    uint32_t EXTINTEO:8;
    uint32_t :24;
  } vec;
  uint32_t reg;
} EIC_EVCTRL_Type;
# 198 "include/component/eic.h"
typedef union {
  struct {
    uint32_t EXTINT0:1;
    uint32_t EXTINT1:1;
    uint32_t EXTINT2:1;
    uint32_t EXTINT3:1;
    uint32_t EXTINT4:1;
    uint32_t EXTINT5:1;
    uint32_t EXTINT6:1;
    uint32_t EXTINT7:1;
    uint32_t :24;
  } bit;
  struct {
    uint32_t EXTINT:8;
    uint32_t :24;
  } vec;
  uint32_t reg;
} EIC_INTENCLR_Type;
# 244 "include/component/eic.h"
typedef union {
  struct {
    uint32_t EXTINT0:1;
    uint32_t EXTINT1:1;
    uint32_t EXTINT2:1;
    uint32_t EXTINT3:1;
    uint32_t EXTINT4:1;
    uint32_t EXTINT5:1;
    uint32_t EXTINT6:1;
    uint32_t EXTINT7:1;
    uint32_t :24;
  } bit;
  struct {
    uint32_t EXTINT:8;
    uint32_t :24;
  } vec;
  uint32_t reg;
} EIC_INTENSET_Type;
# 290 "include/component/eic.h"
typedef union {
  struct {
    volatile const uint32_t EXTINT0:1;
    volatile const uint32_t EXTINT1:1;
    volatile const uint32_t EXTINT2:1;
    volatile const uint32_t EXTINT3:1;
    volatile const uint32_t EXTINT4:1;
    volatile const uint32_t EXTINT5:1;
    volatile const uint32_t EXTINT6:1;
    volatile const uint32_t EXTINT7:1;
    volatile const uint32_t :24;
  } bit;
  struct {
    volatile const uint32_t EXTINT:8;
    volatile const uint32_t :24;
  } vec;
  uint32_t reg;
} EIC_INTFLAG_Type;
# 336 "include/component/eic.h"
typedef union {
  struct {
    uint32_t WAKEUPEN0:1;
    uint32_t WAKEUPEN1:1;
    uint32_t WAKEUPEN2:1;
    uint32_t WAKEUPEN3:1;
    uint32_t WAKEUPEN4:1;
    uint32_t WAKEUPEN5:1;
    uint32_t WAKEUPEN6:1;
    uint32_t WAKEUPEN7:1;
    uint32_t :24;
  } bit;
  struct {
    uint32_t WAKEUPEN:8;
    uint32_t :24;
  } vec;
  uint32_t reg;
} EIC_WAKEUP_Type;
# 382 "include/component/eic.h"
typedef union {
  struct {
    uint32_t SENSE0:3;
    uint32_t FILTEN0:1;
    uint32_t SENSE1:3;
    uint32_t FILTEN1:1;
    uint32_t SENSE2:3;
    uint32_t FILTEN2:1;
    uint32_t SENSE3:3;
    uint32_t FILTEN3:1;
    uint32_t SENSE4:3;
    uint32_t FILTEN4:1;
    uint32_t SENSE5:3;
    uint32_t FILTEN5:1;
    uint32_t SENSE6:3;
    uint32_t FILTEN6:1;
    uint32_t SENSE7:3;
    uint32_t FILTEN7:1;
  } bit;
  uint32_t reg;
} EIC_CONFIG_Type;
# 548 "include/component/eic.h"
typedef struct {
  volatile EIC_CTRL_Type CTRL;
  volatile const EIC_STATUS_Type STATUS;
  volatile EIC_NMICTRL_Type NMICTRL;
  volatile EIC_NMIFLAG_Type NMIFLAG;
  volatile EIC_EVCTRL_Type EVCTRL;
  volatile EIC_INTENCLR_Type INTENCLR;
  volatile EIC_INTENSET_Type INTENSET;
  volatile EIC_INTFLAG_Type INTFLAG;
  volatile EIC_WAKEUP_Type WAKEUP;
  volatile EIC_CONFIG_Type CONFIG[1];
} Eic;
# 231 "include/samd10c13a.h" 2
# 1 "include/component/evsys.h" 1
# 61 "include/component/evsys.h"
typedef union {
  struct {
    uint8_t SWRST:1;
    uint8_t :3;
    uint8_t GCLKREQ:1;
    uint8_t :3;
  } bit;
  uint8_t reg;
} EVSYS_CTRL_Type;
# 83 "include/component/evsys.h"
typedef union {
  struct {
    uint32_t CHANNEL:3;
    uint32_t :5;
    uint32_t SWEVT:1;
    uint32_t :7;
    uint32_t EVGEN:6;
    uint32_t :2;
    uint32_t PATH:2;
    uint32_t EDGSEL:2;
    uint32_t :4;
  } bit;
  uint32_t reg;
} EVSYS_CHANNEL_Type;
# 134 "include/component/evsys.h"
typedef union {
  struct {
    uint16_t USER:5;
    uint16_t :3;
    uint16_t CHANNEL:4;
    uint16_t :4;
  } bit;
  uint16_t reg;
} EVSYS_USER_Type;
# 160 "include/component/evsys.h"
typedef union {
  struct {
    uint32_t USRRDY0:1;
    uint32_t USRRDY1:1;
    uint32_t USRRDY2:1;
    uint32_t USRRDY3:1;
    uint32_t USRRDY4:1;
    uint32_t USRRDY5:1;
    uint32_t :2;
    uint32_t CHBUSY0:1;
    uint32_t CHBUSY1:1;
    uint32_t CHBUSY2:1;
    uint32_t CHBUSY3:1;
    uint32_t CHBUSY4:1;
    uint32_t CHBUSY5:1;
    uint32_t :18;
  } bit;
  struct {
    uint32_t USRRDY:6;
    uint32_t :2;
    uint32_t CHBUSY:6;
    uint32_t :18;
  } vec;
  uint32_t reg;
} EVSYS_CHSTATUS_Type;
# 224 "include/component/evsys.h"
typedef union {
  struct {
    uint32_t OVR0:1;
    uint32_t OVR1:1;
    uint32_t OVR2:1;
    uint32_t OVR3:1;
    uint32_t OVR4:1;
    uint32_t OVR5:1;
    uint32_t :2;
    uint32_t EVD0:1;
    uint32_t EVD1:1;
    uint32_t EVD2:1;
    uint32_t EVD3:1;
    uint32_t EVD4:1;
    uint32_t EVD5:1;
    uint32_t :18;
  } bit;
  struct {
    uint32_t OVR:6;
    uint32_t :2;
    uint32_t EVD:6;
    uint32_t :18;
  } vec;
  uint32_t reg;
} EVSYS_INTENCLR_Type;
# 288 "include/component/evsys.h"
typedef union {
  struct {
    uint32_t OVR0:1;
    uint32_t OVR1:1;
    uint32_t OVR2:1;
    uint32_t OVR3:1;
    uint32_t OVR4:1;
    uint32_t OVR5:1;
    uint32_t :2;
    uint32_t EVD0:1;
    uint32_t EVD1:1;
    uint32_t EVD2:1;
    uint32_t EVD3:1;
    uint32_t EVD4:1;
    uint32_t EVD5:1;
    uint32_t :18;
  } bit;
  struct {
    uint32_t OVR:6;
    uint32_t :2;
    uint32_t EVD:6;
    uint32_t :18;
  } vec;
  uint32_t reg;
} EVSYS_INTENSET_Type;
# 352 "include/component/evsys.h"
typedef union {
  struct {
    volatile const uint32_t OVR0:1;
    volatile const uint32_t OVR1:1;
    volatile const uint32_t OVR2:1;
    volatile const uint32_t OVR3:1;
    volatile const uint32_t OVR4:1;
    volatile const uint32_t OVR5:1;
    volatile const uint32_t :2;
    volatile const uint32_t EVD0:1;
    volatile const uint32_t EVD1:1;
    volatile const uint32_t EVD2:1;
    volatile const uint32_t EVD3:1;
    volatile const uint32_t EVD4:1;
    volatile const uint32_t EVD5:1;
    volatile const uint32_t :18;
  } bit;
  struct {
    volatile const uint32_t OVR:6;
    volatile const uint32_t :2;
    volatile const uint32_t EVD:6;
    volatile const uint32_t :18;
  } vec;
  uint32_t reg;
} EVSYS_INTFLAG_Type;
# 416 "include/component/evsys.h"
typedef struct {
  volatile EVSYS_CTRL_Type CTRL;
       RoReg8 Reserved1[0x3];
  volatile EVSYS_CHANNEL_Type CHANNEL;
  volatile EVSYS_USER_Type USER;
       RoReg8 Reserved2[0x2];
  volatile const EVSYS_CHSTATUS_Type CHSTATUS;
  volatile EVSYS_INTENCLR_Type INTENCLR;
  volatile EVSYS_INTENSET_Type INTENSET;
  volatile EVSYS_INTFLAG_Type INTFLAG;
} Evsys;
# 232 "include/samd10c13a.h" 2
# 1 "include/component/gclk.h" 1
# 61 "include/component/gclk.h"
typedef union {
  struct {
    uint8_t SWRST:1;
    uint8_t :7;
  } bit;
  uint8_t reg;
} GCLK_CTRL_Type;
# 79 "include/component/gclk.h"
typedef union {
  struct {
    uint8_t :7;
    uint8_t SYNCBUSY:1;
  } bit;
  uint8_t reg;
} GCLK_STATUS_Type;
# 97 "include/component/gclk.h"
typedef union {
  struct {
    uint16_t ID:6;
    uint16_t :2;
    uint16_t GEN:4;
    uint16_t :2;
    uint16_t CLKEN:1;
    uint16_t WRTLOCK:1;
  } bit;
  uint16_t reg;
} GCLK_CLKCTRL_Type;
# 191 "include/component/gclk.h"
typedef union {
  struct {
    uint32_t ID:4;
    uint32_t :4;
    uint32_t SRC:5;
    uint32_t :3;
    uint32_t GENEN:1;
    uint32_t IDC:1;
    uint32_t OOV:1;
    uint32_t OE:1;
    uint32_t DIVSEL:1;
    uint32_t RUNSTDBY:1;
    uint32_t :10;
  } bit;
  uint32_t reg;
} GCLK_GENCTRL_Type;
# 252 "include/component/gclk.h"
typedef union {
  struct {
    uint32_t ID:4;
    uint32_t :4;
    uint32_t DIV:16;
    uint32_t :8;
  } bit;
  uint32_t reg;
} GCLK_GENDIV_Type;
# 276 "include/component/gclk.h"
typedef struct {
  volatile GCLK_CTRL_Type CTRL;
  volatile const GCLK_STATUS_Type STATUS;
  volatile GCLK_CLKCTRL_Type CLKCTRL;
  volatile GCLK_GENCTRL_Type GENCTRL;
  volatile GCLK_GENDIV_Type GENDIV;
} Gclk;
# 233 "include/samd10c13a.h" 2
# 1 "include/component/hmatrixb.h" 1
# 61 "include/component/hmatrixb.h"
typedef union {
  struct {
    uint32_t M0PR:4;
    uint32_t M1PR:4;
    uint32_t M2PR:4;
    uint32_t M3PR:4;
    uint32_t M4PR:4;
    uint32_t M5PR:4;
    uint32_t M6PR:4;
    uint32_t M7PR:4;
  } bit;
  uint32_t reg;
} HMATRIXB_PRAS_Type;
# 107 "include/component/hmatrixb.h"
typedef union {
  struct {
    uint32_t M8PR:4;
    uint32_t M9PR:4;
    uint32_t M10PR:4;
    uint32_t M11PR:4;
    uint32_t M12PR:4;
    uint32_t M13PR:4;
    uint32_t M14PR:4;
    uint32_t M15PR:4;
  } bit;
  uint32_t reg;
} HMATRIXB_PRBS_Type;
# 153 "include/component/hmatrixb.h"
typedef union {
  struct {
    uint32_t SFR:32;
  } bit;
  uint32_t reg;
} HMATRIXB_SFR_Type;
# 171 "include/component/hmatrixb.h"
typedef struct {
  volatile HMATRIXB_PRAS_Type PRAS;
  volatile HMATRIXB_PRBS_Type PRBS;
} HmatrixbPrs;




typedef struct {
       RoReg8 Reserved1[0x80];
       HmatrixbPrs Prs[16];
       RoReg8 Reserved2[0x10];
  volatile HMATRIXB_SFR_Type SFR[16];
} Hmatrixb;
# 234 "include/samd10c13a.h" 2
# 1 "include/component/mtb.h" 1
# 61 "include/component/mtb.h"
typedef union {
  struct {
    uint32_t :2;
    uint32_t WRAP:1;
    uint32_t POINTER:29;
  } bit;
  uint32_t reg;
} MTB_POSITION_Type;
# 82 "include/component/mtb.h"
typedef union {
  struct {
    uint32_t MASK:5;
    uint32_t TSTARTEN:1;
    uint32_t TSTOPEN:1;
    uint32_t SFRWPRIV:1;
    uint32_t RAMPRIV:1;
    uint32_t HALTREQ:1;
    uint32_t :21;
    uint32_t EN:1;
  } bit;
  uint32_t reg;
} MTB_MASTER_Type;
# 119 "include/component/mtb.h"
typedef union {
  struct {
    uint32_t AUTOSTOP:1;
    uint32_t AUTOHALT:1;
    uint32_t :1;
    uint32_t WATERMARK:29;
  } bit;
  uint32_t reg;
} MTB_FLOW_Type;
# 144 "include/component/mtb.h"
typedef union {
  uint32_t reg;
} MTB_BASE_Type;







typedef union {
  uint32_t reg;
} MTB_ITCTRL_Type;







typedef union {
  uint32_t reg;
} MTB_CLAIMSET_Type;







typedef union {
  uint32_t reg;
} MTB_CLAIMCLR_Type;







typedef union {
  uint32_t reg;
} MTB_LOCKACCESS_Type;







typedef union {
  uint32_t reg;
} MTB_LOCKSTATUS_Type;







typedef union {
  uint32_t reg;
} MTB_AUTHSTATUS_Type;







typedef union {
  uint32_t reg;
} MTB_DEVARCH_Type;







typedef union {
  uint32_t reg;
} MTB_DEVID_Type;







typedef union {
  uint32_t reg;
} MTB_DEVTYPE_Type;







typedef union {
  uint32_t reg;
} MTB_PID4_Type;







typedef union {
  uint32_t reg;
} MTB_PID5_Type;







typedef union {
  uint32_t reg;
} MTB_PID6_Type;







typedef union {
  uint32_t reg;
} MTB_PID7_Type;







typedef union {
  uint32_t reg;
} MTB_PID0_Type;







typedef union {
  uint32_t reg;
} MTB_PID1_Type;







typedef union {
  uint32_t reg;
} MTB_PID2_Type;







typedef union {
  uint32_t reg;
} MTB_PID3_Type;







typedef union {
  uint32_t reg;
} MTB_CID0_Type;







typedef union {
  uint32_t reg;
} MTB_CID1_Type;







typedef union {
  uint32_t reg;
} MTB_CID2_Type;







typedef union {
  uint32_t reg;
} MTB_CID3_Type;







typedef struct {
  volatile MTB_POSITION_Type POSITION;
  volatile MTB_MASTER_Type MASTER;
  volatile MTB_FLOW_Type FLOW;
  volatile const MTB_BASE_Type BASE;
       RoReg8 Reserved1[0xEF0];
  volatile MTB_ITCTRL_Type ITCTRL;
       RoReg8 Reserved2[0x9C];
  volatile MTB_CLAIMSET_Type CLAIMSET;
  volatile MTB_CLAIMCLR_Type CLAIMCLR;
       RoReg8 Reserved3[0x8];
  volatile MTB_LOCKACCESS_Type LOCKACCESS;
  volatile const MTB_LOCKSTATUS_Type LOCKSTATUS;
  volatile const MTB_AUTHSTATUS_Type AUTHSTATUS;
  volatile const MTB_DEVARCH_Type DEVARCH;
       RoReg8 Reserved4[0x8];
  volatile const MTB_DEVID_Type DEVID;
  volatile const MTB_DEVTYPE_Type DEVTYPE;
  volatile const MTB_PID4_Type PID4;
  volatile const MTB_PID5_Type PID5;
  volatile const MTB_PID6_Type PID6;
  volatile const MTB_PID7_Type PID7;
  volatile const MTB_PID0_Type PID0;
  volatile const MTB_PID1_Type PID1;
  volatile const MTB_PID2_Type PID2;
  volatile const MTB_PID3_Type PID3;
  volatile const MTB_CID0_Type CID0;
  volatile const MTB_CID1_Type CID1;
  volatile const MTB_CID2_Type CID2;
  volatile const MTB_CID3_Type CID3;
} Mtb;
# 235 "include/samd10c13a.h" 2
# 1 "include/component/nvmctrl.h" 1
# 61 "include/component/nvmctrl.h"
typedef union {
  struct {
    uint16_t CMD:7;
    uint16_t :1;
    uint16_t CMDEX:8;
  } bit;
  uint16_t reg;
} NVMCTRL_CTRLA_Type;
# 112 "include/component/nvmctrl.h"
typedef union {
  struct {
    uint32_t :1;
    uint32_t RWS:4;
    uint32_t :2;
    uint32_t MANW:1;
    uint32_t SLEEPPRM:2;
    uint32_t :6;
    uint32_t READMODE:2;
    uint32_t CACHEDIS:1;
    uint32_t :13;
  } bit;
  uint32_t reg;
} NVMCTRL_CTRLB_Type;
# 166 "include/component/nvmctrl.h"
typedef union {
  struct {
    uint32_t NVMP:16;
    uint32_t PSZ:3;
    uint32_t :13;
  } bit;
  uint32_t reg;
} NVMCTRL_PARAM_Type;
# 205 "include/component/nvmctrl.h"
typedef union {
  struct {
    uint8_t READY:1;
    uint8_t ERROR:1;
    uint8_t :6;
  } bit;
  uint8_t reg;
} NVMCTRL_INTENCLR_Type;
# 226 "include/component/nvmctrl.h"
typedef union {
  struct {
    uint8_t READY:1;
    uint8_t ERROR:1;
    uint8_t :6;
  } bit;
  uint8_t reg;
} NVMCTRL_INTENSET_Type;
# 247 "include/component/nvmctrl.h"
typedef union {
  struct {
    volatile const uint8_t READY:1;
    volatile const uint8_t ERROR:1;
    volatile const uint8_t :6;
  } bit;
  uint8_t reg;
} NVMCTRL_INTFLAG_Type;
# 268 "include/component/nvmctrl.h"
typedef union {
  struct {
    uint16_t PRM:1;
    uint16_t LOAD:1;
    uint16_t PROGE:1;
    uint16_t LOCKE:1;
    uint16_t NVME:1;
    uint16_t :3;
    uint16_t SB:1;
    uint16_t :7;
  } bit;
  uint16_t reg;
} NVMCTRL_STATUS_Type;
# 302 "include/component/nvmctrl.h"
typedef union {
  struct {
    uint32_t ADDR:22;
    uint32_t :10;
  } bit;
  uint32_t reg;
} NVMCTRL_ADDR_Type;
# 321 "include/component/nvmctrl.h"
typedef union {
  struct {
    uint16_t LOCK:16;
  } bit;
  uint16_t reg;
} NVMCTRL_LOCK_Type;
# 338 "include/component/nvmctrl.h"
typedef struct {
  volatile NVMCTRL_CTRLA_Type CTRLA;
       RoReg8 Reserved1[0x2];
  volatile NVMCTRL_CTRLB_Type CTRLB;
  volatile NVMCTRL_PARAM_Type PARAM;
  volatile NVMCTRL_INTENCLR_Type INTENCLR;
       RoReg8 Reserved2[0x3];
  volatile NVMCTRL_INTENSET_Type INTENSET;
       RoReg8 Reserved3[0x3];
  volatile NVMCTRL_INTFLAG_Type INTFLAG;
       RoReg8 Reserved4[0x3];
  volatile NVMCTRL_STATUS_Type STATUS;
       RoReg8 Reserved5[0x2];
  volatile NVMCTRL_ADDR_Type ADDR;
  volatile NVMCTRL_LOCK_Type LOCK;
} Nvmctrl;
# 236 "include/samd10c13a.h" 2
# 1 "include/component/pac.h" 1
# 61 "include/component/pac.h"
typedef union {
  struct {
    uint32_t :1;
    uint32_t WP:31;
  } bit;
  uint32_t reg;
} PAC_WPCLR_Type;
# 80 "include/component/pac.h"
typedef union {
  struct {
    uint32_t :1;
    uint32_t WP:31;
  } bit;
  uint32_t reg;
} PAC_WPSET_Type;
# 99 "include/component/pac.h"
typedef struct {
  volatile PAC_WPCLR_Type WPCLR;
  volatile PAC_WPSET_Type WPSET;
} Pac;
# 237 "include/samd10c13a.h" 2
# 1 "include/component/pm.h" 1
# 61 "include/component/pm.h"
typedef union {
  struct {
    uint8_t :2;
    uint8_t CFDEN:1;
    uint8_t :1;
    uint8_t BKUPCLK:1;
    uint8_t :3;
  } bit;
  uint8_t reg;
} PM_CTRL_Type;
# 84 "include/component/pm.h"
typedef union {
  struct {
    uint8_t IDLE:2;
    uint8_t :6;
  } bit;
  uint8_t reg;
} PM_SLEEP_Type;
# 109 "include/component/pm.h"
typedef union {
  struct {
    uint8_t SETDIS:1;
    uint8_t :7;
  } bit;
  uint8_t reg;
} PM_EXTCTRL_Type;
# 127 "include/component/pm.h"
typedef union {
  struct {
    uint8_t CPUDIV:3;
    uint8_t :5;
  } bit;
  uint8_t reg;
} PM_CPUSEL_Type;
# 162 "include/component/pm.h"
typedef union {
  struct {
    uint8_t APBADIV:3;
    uint8_t :5;
  } bit;
  uint8_t reg;
} PM_APBASEL_Type;
# 197 "include/component/pm.h"
typedef union {
  struct {
    uint8_t APBBDIV:3;
    uint8_t :5;
  } bit;
  uint8_t reg;
} PM_APBBSEL_Type;
# 232 "include/component/pm.h"
typedef union {
  struct {
    uint8_t APBCDIV:3;
    uint8_t :5;
  } bit;
  uint8_t reg;
} PM_APBCSEL_Type;
# 267 "include/component/pm.h"
typedef union {
  struct {
    uint32_t HPB0_:1;
    uint32_t HPB1_:1;
    uint32_t HPB2_:1;
    uint32_t DSU_:1;
    uint32_t NVMCTRL_:1;
    uint32_t DMAC_:1;
    uint32_t :26;
  } bit;
  uint32_t reg;
} PM_AHBMASK_Type;
# 300 "include/component/pm.h"
typedef union {
  struct {
    uint32_t PAC0_:1;
    uint32_t PM_:1;
    uint32_t SYSCTRL_:1;
    uint32_t GCLK_:1;
    uint32_t WDT_:1;
    uint32_t RTC_:1;
    uint32_t EIC_:1;
    uint32_t :25;
  } bit;
  uint32_t reg;
} PM_APBAMASK_Type;
# 336 "include/component/pm.h"
typedef union {
  struct {
    uint32_t PAC1_:1;
    uint32_t DSU_:1;
    uint32_t NVMCTRL_:1;
    uint32_t PORT_:1;
    uint32_t DMAC_:1;
    uint32_t :1;
    uint32_t HMATRIX_:1;
    uint32_t :25;
  } bit;
  uint32_t reg;
} PM_APBBMASK_Type;
# 370 "include/component/pm.h"
typedef union {
  struct {
    uint32_t PAC2_:1;
    uint32_t EVSYS_:1;
    uint32_t SERCOM0_:1;
    uint32_t SERCOM1_:1;
    uint32_t SERCOM2_:1;
    uint32_t TCC0_:1;
    uint32_t TC1_:1;
    uint32_t TC2_:1;
    uint32_t ADC_:1;
    uint32_t AC_:1;
    uint32_t DAC_:1;
    uint32_t PTC_:1;
    uint32_t :20;
  } bit;
  uint32_t reg;
} PM_APBCMASK_Type;
# 421 "include/component/pm.h"
typedef union {
  struct {
    uint8_t CKRDY:1;
    uint8_t CFD:1;
    uint8_t :6;
  } bit;
  uint8_t reg;
} PM_INTENCLR_Type;
# 442 "include/component/pm.h"
typedef union {
  struct {
    uint8_t CKRDY:1;
    uint8_t CFD:1;
    uint8_t :6;
  } bit;
  uint8_t reg;
} PM_INTENSET_Type;
# 463 "include/component/pm.h"
typedef union {
  struct {
    volatile const uint8_t CKRDY:1;
    volatile const uint8_t CFD:1;
    volatile const uint8_t :6;
  } bit;
  uint8_t reg;
} PM_INTFLAG_Type;
# 484 "include/component/pm.h"
typedef union {
  struct {
    uint8_t POR:1;
    uint8_t BOD12:1;
    uint8_t BOD33:1;
    uint8_t :1;
    uint8_t EXT:1;
    uint8_t WDT:1;
    uint8_t SYST:1;
    uint8_t :1;
  } bit;
  uint8_t reg;
} PM_RCAUSE_Type;
# 518 "include/component/pm.h"
typedef struct {
  volatile PM_CTRL_Type CTRL;
  volatile PM_SLEEP_Type SLEEP;
  volatile PM_EXTCTRL_Type EXTCTRL;
       RoReg8 Reserved1[0x5];
  volatile PM_CPUSEL_Type CPUSEL;
  volatile PM_APBASEL_Type APBASEL;
  volatile PM_APBBSEL_Type APBBSEL;
  volatile PM_APBCSEL_Type APBCSEL;
       RoReg8 Reserved2[0x8];
  volatile PM_AHBMASK_Type AHBMASK;
  volatile PM_APBAMASK_Type APBAMASK;
  volatile PM_APBBMASK_Type APBBMASK;
  volatile PM_APBCMASK_Type APBCMASK;
       RoReg8 Reserved3[0x10];
  volatile PM_INTENCLR_Type INTENCLR;
  volatile PM_INTENSET_Type INTENSET;
  volatile PM_INTFLAG_Type INTFLAG;
       RoReg8 Reserved4[0x1];
  volatile const PM_RCAUSE_Type RCAUSE;
} Pm;
# 238 "include/samd10c13a.h" 2
# 1 "include/component/port.h" 1
# 61 "include/component/port.h"
typedef union {
  struct {
    uint32_t DIR:32;
  } bit;
  uint32_t reg;
} PORT_DIR_Type;
# 79 "include/component/port.h"
typedef union {
  struct {
    uint32_t DIRCLR:32;
  } bit;
  uint32_t reg;
} PORT_DIRCLR_Type;
# 97 "include/component/port.h"
typedef union {
  struct {
    uint32_t DIRSET:32;
  } bit;
  uint32_t reg;
} PORT_DIRSET_Type;
# 115 "include/component/port.h"
typedef union {
  struct {
    uint32_t DIRTGL:32;
  } bit;
  uint32_t reg;
} PORT_DIRTGL_Type;
# 133 "include/component/port.h"
typedef union {
  struct {
    uint32_t OUT:32;
  } bit;
  uint32_t reg;
} PORT_OUT_Type;
# 151 "include/component/port.h"
typedef union {
  struct {
    uint32_t OUTCLR:32;
  } bit;
  uint32_t reg;
} PORT_OUTCLR_Type;
# 169 "include/component/port.h"
typedef union {
  struct {
    uint32_t OUTSET:32;
  } bit;
  uint32_t reg;
} PORT_OUTSET_Type;
# 187 "include/component/port.h"
typedef union {
  struct {
    uint32_t OUTTGL:32;
  } bit;
  uint32_t reg;
} PORT_OUTTGL_Type;
# 205 "include/component/port.h"
typedef union {
  struct {
    uint32_t IN:32;
  } bit;
  uint32_t reg;
} PORT_IN_Type;
# 223 "include/component/port.h"
typedef union {
  struct {
    uint32_t SAMPLING:32;
  } bit;
  uint32_t reg;
} PORT_CTRL_Type;
# 241 "include/component/port.h"
typedef union {
  struct {
    uint32_t PINMASK:16;
    uint32_t PMUXEN:1;
    uint32_t INEN:1;
    uint32_t PULLEN:1;
    uint32_t :3;
    uint32_t DRVSTR:1;
    uint32_t :1;
    uint32_t PMUX:4;
    uint32_t WRPMUX:1;
    uint32_t :1;
    uint32_t WRPINCFG:1;
    uint32_t HWSEL:1;
  } bit;
  uint32_t reg;
} PORT_WRCONFIG_Type;
# 287 "include/component/port.h"
typedef union {
  struct {
    uint8_t PMUXE:4;
    uint8_t PMUXO:4;
  } bit;
  uint8_t reg;
} PORT_PMUX_Type;
# 341 "include/component/port.h"
typedef union {
  struct {
    uint8_t PMUXEN:1;
    uint8_t INEN:1;
    uint8_t PULLEN:1;
    uint8_t :3;
    uint8_t DRVSTR:1;
    uint8_t :1;
  } bit;
  uint8_t reg;
} PORT_PINCFG_Type;
# 369 "include/component/port.h"
typedef struct {
  volatile PORT_DIR_Type DIR;
  volatile PORT_DIRCLR_Type DIRCLR;
  volatile PORT_DIRSET_Type DIRSET;
  volatile PORT_DIRTGL_Type DIRTGL;
  volatile PORT_OUT_Type OUT;
  volatile PORT_OUTCLR_Type OUTCLR;
  volatile PORT_OUTSET_Type OUTSET;
  volatile PORT_OUTTGL_Type OUTTGL;
  volatile const PORT_IN_Type IN;
  volatile PORT_CTRL_Type CTRL;
  volatile PORT_WRCONFIG_Type WRCONFIG;
       RoReg8 Reserved1[0x4];
  volatile PORT_PMUX_Type PMUX[16];
  volatile PORT_PINCFG_Type PINCFG[32];
       RoReg8 Reserved2[0x20];
} PortGroup;




typedef struct {
       PortGroup Group[1];
} Port;
# 239 "include/samd10c13a.h" 2
# 1 "include/component/rtc.h" 1
# 61 "include/component/rtc.h"
typedef union {
  struct {
    uint16_t SWRST:1;
    uint16_t ENABLE:1;
    uint16_t MODE:2;
    uint16_t :3;
    uint16_t MATCHCLR:1;
    uint16_t PRESCALER:4;
    uint16_t :4;
  } bit;
  uint16_t reg;
} RTC_MODE0_CTRL_Type;
# 122 "include/component/rtc.h"
typedef union {
  struct {
    uint16_t SWRST:1;
    uint16_t ENABLE:1;
    uint16_t MODE:2;
    uint16_t :4;
    uint16_t PRESCALER:4;
    uint16_t :4;
  } bit;
  uint16_t reg;
} RTC_MODE1_CTRL_Type;
# 180 "include/component/rtc.h"
typedef union {
  struct {
    uint16_t SWRST:1;
    uint16_t ENABLE:1;
    uint16_t MODE:2;
    uint16_t :2;
    uint16_t CLKREP:1;
    uint16_t MATCHCLR:1;
    uint16_t PRESCALER:4;
    uint16_t :4;
  } bit;
  uint16_t reg;
} RTC_MODE2_CTRL_Type;
# 244 "include/component/rtc.h"
typedef union {
  struct {
    uint16_t ADDR:6;
    uint16_t :8;
    uint16_t RCONT:1;
    uint16_t RREQ:1;
  } bit;
  uint16_t reg;
} RTC_READREQ_Type;
# 269 "include/component/rtc.h"
typedef union {
  struct {
    uint16_t PEREO0:1;
    uint16_t PEREO1:1;
    uint16_t PEREO2:1;
    uint16_t PEREO3:1;
    uint16_t PEREO4:1;
    uint16_t PEREO5:1;
    uint16_t PEREO6:1;
    uint16_t PEREO7:1;
    uint16_t CMPEO0:1;
    uint16_t :6;
    uint16_t OVFEO:1;
  } bit;
  struct {
    uint16_t PEREO:8;
    uint16_t CMPEO:1;
    uint16_t :7;
  } vec;
  uint16_t reg;
} RTC_MODE0_EVCTRL_Type;
# 325 "include/component/rtc.h"
typedef union {
  struct {
    uint16_t PEREO0:1;
    uint16_t PEREO1:1;
    uint16_t PEREO2:1;
    uint16_t PEREO3:1;
    uint16_t PEREO4:1;
    uint16_t PEREO5:1;
    uint16_t PEREO6:1;
    uint16_t PEREO7:1;
    uint16_t CMPEO0:1;
    uint16_t CMPEO1:1;
    uint16_t :5;
    uint16_t OVFEO:1;
  } bit;
  struct {
    uint16_t PEREO:8;
    uint16_t CMPEO:2;
    uint16_t :6;
  } vec;
  uint16_t reg;
} RTC_MODE1_EVCTRL_Type;
# 384 "include/component/rtc.h"
typedef union {
  struct {
    uint16_t PEREO0:1;
    uint16_t PEREO1:1;
    uint16_t PEREO2:1;
    uint16_t PEREO3:1;
    uint16_t PEREO4:1;
    uint16_t PEREO5:1;
    uint16_t PEREO6:1;
    uint16_t PEREO7:1;
    uint16_t ALARMEO0:1;
    uint16_t :6;
    uint16_t OVFEO:1;
  } bit;
  struct {
    uint16_t PEREO:8;
    uint16_t ALARMEO:1;
    uint16_t :7;
  } vec;
  uint16_t reg;
} RTC_MODE2_EVCTRL_Type;
# 440 "include/component/rtc.h"
typedef union {
  struct {
    uint8_t CMP0:1;
    uint8_t :5;
    uint8_t SYNCRDY:1;
    uint8_t OVF:1;
  } bit;
  struct {
    uint8_t CMP:1;
    uint8_t :7;
  } vec;
  uint8_t reg;
} RTC_MODE0_INTENCLR_Type;
# 471 "include/component/rtc.h"
typedef union {
  struct {
    uint8_t CMP0:1;
    uint8_t CMP1:1;
    uint8_t :4;
    uint8_t SYNCRDY:1;
    uint8_t OVF:1;
  } bit;
  struct {
    uint8_t CMP:2;
    uint8_t :6;
  } vec;
  uint8_t reg;
} RTC_MODE1_INTENCLR_Type;
# 505 "include/component/rtc.h"
typedef union {
  struct {
    uint8_t ALARM0:1;
    uint8_t :5;
    uint8_t SYNCRDY:1;
    uint8_t OVF:1;
  } bit;
  struct {
    uint8_t ALARM:1;
    uint8_t :7;
  } vec;
  uint8_t reg;
} RTC_MODE2_INTENCLR_Type;
# 536 "include/component/rtc.h"
typedef union {
  struct {
    uint8_t CMP0:1;
    uint8_t :5;
    uint8_t SYNCRDY:1;
    uint8_t OVF:1;
  } bit;
  struct {
    uint8_t CMP:1;
    uint8_t :7;
  } vec;
  uint8_t reg;
} RTC_MODE0_INTENSET_Type;
# 567 "include/component/rtc.h"
typedef union {
  struct {
    uint8_t CMP0:1;
    uint8_t CMP1:1;
    uint8_t :4;
    uint8_t SYNCRDY:1;
    uint8_t OVF:1;
  } bit;
  struct {
    uint8_t CMP:2;
    uint8_t :6;
  } vec;
  uint8_t reg;
} RTC_MODE1_INTENSET_Type;
# 601 "include/component/rtc.h"
typedef union {
  struct {
    uint8_t ALARM0:1;
    uint8_t :5;
    uint8_t SYNCRDY:1;
    uint8_t OVF:1;
  } bit;
  struct {
    uint8_t ALARM:1;
    uint8_t :7;
  } vec;
  uint8_t reg;
} RTC_MODE2_INTENSET_Type;
# 632 "include/component/rtc.h"
typedef union {
  struct {
    volatile const uint8_t CMP0:1;
    volatile const uint8_t :5;
    volatile const uint8_t SYNCRDY:1;
    volatile const uint8_t OVF:1;
  } bit;
  struct {
    volatile const uint8_t CMP:1;
    volatile const uint8_t :7;
  } vec;
  uint8_t reg;
} RTC_MODE0_INTFLAG_Type;
# 663 "include/component/rtc.h"
typedef union {
  struct {
    volatile const uint8_t CMP0:1;
    volatile const uint8_t CMP1:1;
    volatile const uint8_t :4;
    volatile const uint8_t SYNCRDY:1;
    volatile const uint8_t OVF:1;
  } bit;
  struct {
    volatile const uint8_t CMP:2;
    volatile const uint8_t :6;
  } vec;
  uint8_t reg;
} RTC_MODE1_INTFLAG_Type;
# 697 "include/component/rtc.h"
typedef union {
  struct {
    volatile const uint8_t ALARM0:1;
    volatile const uint8_t :5;
    volatile const uint8_t SYNCRDY:1;
    volatile const uint8_t OVF:1;
  } bit;
  struct {
    volatile const uint8_t ALARM:1;
    volatile const uint8_t :7;
  } vec;
  uint8_t reg;
} RTC_MODE2_INTFLAG_Type;
# 728 "include/component/rtc.h"
typedef union {
  struct {
    uint8_t :7;
    uint8_t SYNCBUSY:1;
  } bit;
  uint8_t reg;
} RTC_STATUS_Type;
# 746 "include/component/rtc.h"
typedef union {
  struct {
    uint8_t DBGRUN:1;
    uint8_t :7;
  } bit;
  uint8_t reg;
} RTC_DBGCTRL_Type;
# 764 "include/component/rtc.h"
typedef union {
  struct {
    uint8_t VALUE:7;
    uint8_t SIGN:1;
  } bit;
  uint8_t reg;
} RTC_FREQCORR_Type;
# 785 "include/component/rtc.h"
typedef union {
  struct {
    uint32_t COUNT:32;
  } bit;
  uint32_t reg;
} RTC_MODE0_COUNT_Type;
# 803 "include/component/rtc.h"
typedef union {
  struct {
    uint16_t COUNT:16;
  } bit;
  uint16_t reg;
} RTC_MODE1_COUNT_Type;
# 821 "include/component/rtc.h"
typedef union {
  struct {
    uint32_t SECOND:6;
    uint32_t MINUTE:6;
    uint32_t HOUR:5;
    uint32_t DAY:5;
    uint32_t MONTH:4;
    uint32_t YEAR:6;
  } bit;
  uint32_t reg;
} RTC_MODE2_CLOCK_Type;
# 861 "include/component/rtc.h"
typedef union {
  struct {
    uint16_t PER:16;
  } bit;
  uint16_t reg;
} RTC_MODE1_PER_Type;
# 879 "include/component/rtc.h"
typedef union {
  struct {
    uint32_t COMP:32;
  } bit;
  uint32_t reg;
} RTC_MODE0_COMP_Type;
# 897 "include/component/rtc.h"
typedef union {
  struct {
    uint16_t COMP:16;
  } bit;
  uint16_t reg;
} RTC_MODE1_COMP_Type;
# 915 "include/component/rtc.h"
typedef union {
  struct {
    uint32_t SECOND:6;
    uint32_t MINUTE:6;
    uint32_t HOUR:5;
    uint32_t DAY:5;
    uint32_t MONTH:4;
    uint32_t YEAR:6;
  } bit;
  uint32_t reg;
} RTC_MODE2_ALARM_Type;
# 953 "include/component/rtc.h"
typedef union {
  struct {
    uint8_t SEL:3;
    uint8_t :5;
  } bit;
  uint8_t reg;
} RTC_MODE2_MASK_Type;
# 986 "include/component/rtc.h"
typedef struct {
  volatile RTC_MODE2_ALARM_Type ALARM;
  volatile RTC_MODE2_MASK_Type MASK;
       RoReg8 Reserved1[0x3];
} RtcMode2Alarm;




typedef struct {
  volatile RTC_MODE0_CTRL_Type CTRL;
  volatile RTC_READREQ_Type READREQ;
  volatile RTC_MODE0_EVCTRL_Type EVCTRL;
  volatile RTC_MODE0_INTENCLR_Type INTENCLR;
  volatile RTC_MODE0_INTENSET_Type INTENSET;
  volatile RTC_MODE0_INTFLAG_Type INTFLAG;
       RoReg8 Reserved1[0x1];
  volatile RTC_STATUS_Type STATUS;
  volatile RTC_DBGCTRL_Type DBGCTRL;
  volatile RTC_FREQCORR_Type FREQCORR;
       RoReg8 Reserved2[0x3];
  volatile RTC_MODE0_COUNT_Type COUNT;
       RoReg8 Reserved3[0x4];
  volatile RTC_MODE0_COMP_Type COMP[1];
} RtcMode0;




typedef struct {
  volatile RTC_MODE1_CTRL_Type CTRL;
  volatile RTC_READREQ_Type READREQ;
  volatile RTC_MODE1_EVCTRL_Type EVCTRL;
  volatile RTC_MODE1_INTENCLR_Type INTENCLR;
  volatile RTC_MODE1_INTENSET_Type INTENSET;
  volatile RTC_MODE1_INTFLAG_Type INTFLAG;
       RoReg8 Reserved1[0x1];
  volatile RTC_STATUS_Type STATUS;
  volatile RTC_DBGCTRL_Type DBGCTRL;
  volatile RTC_FREQCORR_Type FREQCORR;
       RoReg8 Reserved2[0x3];
  volatile RTC_MODE1_COUNT_Type COUNT;
       RoReg8 Reserved3[0x2];
  volatile RTC_MODE1_PER_Type PER;
       RoReg8 Reserved4[0x2];
  volatile RTC_MODE1_COMP_Type COMP[2];
} RtcMode1;




typedef struct {
  volatile RTC_MODE2_CTRL_Type CTRL;
  volatile RTC_READREQ_Type READREQ;
  volatile RTC_MODE2_EVCTRL_Type EVCTRL;
  volatile RTC_MODE2_INTENCLR_Type INTENCLR;
  volatile RTC_MODE2_INTENSET_Type INTENSET;
  volatile RTC_MODE2_INTFLAG_Type INTFLAG;
       RoReg8 Reserved1[0x1];
  volatile RTC_STATUS_Type STATUS;
  volatile RTC_DBGCTRL_Type DBGCTRL;
  volatile RTC_FREQCORR_Type FREQCORR;
       RoReg8 Reserved2[0x3];
  volatile RTC_MODE2_CLOCK_Type CLOCK;
       RoReg8 Reserved3[0x4];
       RtcMode2Alarm Mode2Alarm[1];
} RtcMode2;



typedef union {
       RtcMode0 MODE0;
       RtcMode1 MODE1;
       RtcMode2 MODE2;
} Rtc;
# 240 "include/samd10c13a.h" 2
# 1 "include/component/sercom.h" 1
# 61 "include/component/sercom.h"
typedef union {
  struct {
    uint32_t SWRST:1;
    uint32_t ENABLE:1;
    uint32_t MODE:3;
    uint32_t :2;
    uint32_t RUNSTDBY:1;
    uint32_t :8;
    uint32_t PINOUT:1;
    uint32_t :3;
    uint32_t SDAHOLD:2;
    uint32_t MEXTTOEN:1;
    uint32_t SEXTTOEN:1;
    uint32_t SPEED:2;
    uint32_t :1;
    uint32_t SCLSM:1;
    uint32_t INACTOUT:2;
    uint32_t LOWTOUTEN:1;
    uint32_t :1;
  } bit;
  uint32_t reg;
} SERCOM_I2CM_CTRLA_Type;
# 132 "include/component/sercom.h"
typedef union {
  struct {
    uint32_t SWRST:1;
    uint32_t ENABLE:1;
    uint32_t MODE:3;
    uint32_t :2;
    uint32_t RUNSTDBY:1;
    uint32_t :8;
    uint32_t PINOUT:1;
    uint32_t :3;
    uint32_t SDAHOLD:2;
    uint32_t :1;
    uint32_t SEXTTOEN:1;
    uint32_t SPEED:2;
    uint32_t :1;
    uint32_t SCLSM:1;
    uint32_t :2;
    uint32_t LOWTOUTEN:1;
    uint32_t :1;
  } bit;
  uint32_t reg;
} SERCOM_I2CS_CTRLA_Type;
# 198 "include/component/sercom.h"
typedef union {
  struct {
    uint32_t SWRST:1;
    uint32_t ENABLE:1;
    uint32_t MODE:3;
    uint32_t :2;
    uint32_t RUNSTDBY:1;
    uint32_t IBON:1;
    uint32_t :7;
    uint32_t DOPO:2;
    uint32_t :2;
    uint32_t DIPO:2;
    uint32_t :2;
    uint32_t FORM:4;
    uint32_t CPHA:1;
    uint32_t CPOL:1;
    uint32_t DORD:1;
    uint32_t :1;
  } bit;
  uint32_t reg;
} SERCOM_SPI_CTRLA_Type;
# 266 "include/component/sercom.h"
typedef union {
  struct {
    uint32_t SWRST:1;
    uint32_t ENABLE:1;
    uint32_t MODE:3;
    uint32_t :2;
    uint32_t RUNSTDBY:1;
    uint32_t IBON:1;
    uint32_t :4;
    uint32_t SAMPR:3;
    uint32_t TXPO:2;
    uint32_t :2;
    uint32_t RXPO:2;
    uint32_t SAMPA:2;
    uint32_t FORM:4;
    uint32_t CMODE:1;
    uint32_t CPOL:1;
    uint32_t DORD:1;
    uint32_t :1;
  } bit;
  uint32_t reg;
} SERCOM_USART_CTRLA_Type;
# 341 "include/component/sercom.h"
typedef union {
  struct {
    uint32_t :8;
    uint32_t SMEN:1;
    uint32_t QCEN:1;
    uint32_t :6;
    uint32_t CMD:2;
    uint32_t ACKACT:1;
    uint32_t :13;
  } bit;
  uint32_t reg;
} SERCOM_I2CM_CTRLB_Type;
# 371 "include/component/sercom.h"
typedef union {
  struct {
    uint32_t :8;
    uint32_t SMEN:1;
    uint32_t GCMD:1;
    uint32_t AACKEN:1;
    uint32_t :3;
    uint32_t AMODE:2;
    uint32_t CMD:2;
    uint32_t ACKACT:1;
    uint32_t :13;
  } bit;
  uint32_t reg;
} SERCOM_I2CS_CTRLB_Type;
# 408 "include/component/sercom.h"
typedef union {
  struct {
    uint32_t CHSIZE:3;
    uint32_t :3;
    uint32_t PLOADEN:1;
    uint32_t :2;
    uint32_t SSDE:1;
    uint32_t :3;
    uint32_t MSSEN:1;
    uint32_t AMODE:2;
    uint32_t :1;
    uint32_t RXEN:1;
    uint32_t :14;
  } bit;
  uint32_t reg;
} SERCOM_SPI_CTRLB_Type;
# 447 "include/component/sercom.h"
typedef union {
  struct {
    uint32_t CHSIZE:3;
    uint32_t :3;
    uint32_t SBMODE:1;
    uint32_t :1;
    uint32_t COLDEN:1;
    uint32_t SFDE:1;
    uint32_t ENC:1;
    uint32_t :2;
    uint32_t PMODE:1;
    uint32_t :2;
    uint32_t TXEN:1;
    uint32_t RXEN:1;
    uint32_t :14;
  } bit;
  uint32_t reg;
} SERCOM_USART_CTRLB_Type;
# 491 "include/component/sercom.h"
typedef union {
  struct {
    uint32_t BAUD:8;
    uint32_t BAUDLOW:8;
    uint32_t HSBAUD:8;
    uint32_t HSBAUDLOW:8;
  } bit;
  uint32_t reg;
} SERCOM_I2CM_BAUD_Type;
# 521 "include/component/sercom.h"
typedef union {
  struct {
    uint8_t BAUD:8;
  } bit;
  uint8_t reg;
} SERCOM_SPI_BAUD_Type;
# 539 "include/component/sercom.h"
typedef union {
  struct {
    uint16_t BAUD:16;
  } bit;
  struct {
    uint16_t BAUD:13;
    uint16_t FP:3;
  } FRAC;
  struct {
    uint16_t BAUD:13;
    uint16_t FP:3;
  } FRACFP;
  struct {
    uint16_t BAUD:16;
  } USARTFP;
  uint16_t reg;
} SERCOM_USART_BAUD_Type;
# 592 "include/component/sercom.h"
typedef union {
  struct {
    uint8_t RXPL:8;
  } bit;
  uint8_t reg;
} SERCOM_USART_RXPL_Type;
# 610 "include/component/sercom.h"
typedef union {
  struct {
    uint8_t MB:1;
    uint8_t SB:1;
    uint8_t :5;
    uint8_t ERROR:1;
  } bit;
  uint8_t reg;
} SERCOM_I2CM_INTENCLR_Type;
# 634 "include/component/sercom.h"
typedef union {
  struct {
    uint8_t PREC:1;
    uint8_t AMATCH:1;
    uint8_t DRDY:1;
    uint8_t :4;
    uint8_t ERROR:1;
  } bit;
  uint8_t reg;
} SERCOM_I2CS_INTENCLR_Type;
# 661 "include/component/sercom.h"
typedef union {
  struct {
    uint8_t DRE:1;
    uint8_t TXC:1;
    uint8_t RXC:1;
    uint8_t SSL:1;
    uint8_t :3;
    uint8_t ERROR:1;
  } bit;
  uint8_t reg;
} SERCOM_SPI_INTENCLR_Type;
# 691 "include/component/sercom.h"
typedef union {
  struct {
    uint8_t DRE:1;
    uint8_t TXC:1;
    uint8_t RXC:1;
    uint8_t RXS:1;
    uint8_t CTSIC:1;
    uint8_t RXBRK:1;
    uint8_t :1;
    uint8_t ERROR:1;
  } bit;
  uint8_t reg;
} SERCOM_USART_INTENCLR_Type;
# 727 "include/component/sercom.h"
typedef union {
  struct {
    uint8_t MB:1;
    uint8_t SB:1;
    uint8_t :5;
    uint8_t ERROR:1;
  } bit;
  uint8_t reg;
} SERCOM_I2CM_INTENSET_Type;
# 751 "include/component/sercom.h"
typedef union {
  struct {
    uint8_t PREC:1;
    uint8_t AMATCH:1;
    uint8_t DRDY:1;
    uint8_t :4;
    uint8_t ERROR:1;
  } bit;
  uint8_t reg;
} SERCOM_I2CS_INTENSET_Type;
# 778 "include/component/sercom.h"
typedef union {
  struct {
    uint8_t DRE:1;
    uint8_t TXC:1;
    uint8_t RXC:1;
    uint8_t SSL:1;
    uint8_t :3;
    uint8_t ERROR:1;
  } bit;
  uint8_t reg;
} SERCOM_SPI_INTENSET_Type;
# 808 "include/component/sercom.h"
typedef union {
  struct {
    uint8_t DRE:1;
    uint8_t TXC:1;
    uint8_t RXC:1;
    uint8_t RXS:1;
    uint8_t CTSIC:1;
    uint8_t RXBRK:1;
    uint8_t :1;
    uint8_t ERROR:1;
  } bit;
  uint8_t reg;
} SERCOM_USART_INTENSET_Type;
# 844 "include/component/sercom.h"
typedef union {
  struct {
    volatile const uint8_t MB:1;
    volatile const uint8_t SB:1;
    volatile const uint8_t :5;
    volatile const uint8_t ERROR:1;
  } bit;
  uint8_t reg;
} SERCOM_I2CM_INTFLAG_Type;
# 868 "include/component/sercom.h"
typedef union {
  struct {
    volatile const uint8_t PREC:1;
    volatile const uint8_t AMATCH:1;
    volatile const uint8_t DRDY:1;
    volatile const uint8_t :4;
    volatile const uint8_t ERROR:1;
  } bit;
  uint8_t reg;
} SERCOM_I2CS_INTFLAG_Type;
# 895 "include/component/sercom.h"
typedef union {
  struct {
    volatile const uint8_t DRE:1;
    volatile const uint8_t TXC:1;
    volatile const uint8_t RXC:1;
    volatile const uint8_t SSL:1;
    volatile const uint8_t :3;
    volatile const uint8_t ERROR:1;
  } bit;
  uint8_t reg;
} SERCOM_SPI_INTFLAG_Type;
# 925 "include/component/sercom.h"
typedef union {
  struct {
    volatile const uint8_t DRE:1;
    volatile const uint8_t TXC:1;
    volatile const uint8_t RXC:1;
    volatile const uint8_t RXS:1;
    volatile const uint8_t CTSIC:1;
    volatile const uint8_t RXBRK:1;
    volatile const uint8_t :1;
    volatile const uint8_t ERROR:1;
  } bit;
  uint8_t reg;
} SERCOM_USART_INTFLAG_Type;
# 961 "include/component/sercom.h"
typedef union {
  struct {
    uint16_t BUSERR:1;
    uint16_t ARBLOST:1;
    uint16_t RXNACK:1;
    uint16_t :1;
    uint16_t BUSSTATE:2;
    uint16_t LOWTOUT:1;
    uint16_t CLKHOLD:1;
    uint16_t MEXTTOUT:1;
    uint16_t SEXTTOUT:1;
    uint16_t LENERR:1;
    uint16_t :5;
  } bit;
  uint16_t reg;
} SERCOM_I2CM_STATUS_Type;
# 1005 "include/component/sercom.h"
typedef union {
  struct {
    uint16_t BUSERR:1;
    uint16_t COLL:1;
    uint16_t RXNACK:1;
    uint16_t DIR:1;
    uint16_t SR:1;
    uint16_t :1;
    uint16_t LOWTOUT:1;
    uint16_t CLKHOLD:1;
    uint16_t :1;
    uint16_t SEXTTOUT:1;
    uint16_t HS:1;
    uint16_t :5;
  } bit;
  uint16_t reg;
} SERCOM_I2CS_STATUS_Type;
# 1049 "include/component/sercom.h"
typedef union {
  struct {
    uint16_t :2;
    uint16_t BUFOVF:1;
    uint16_t :13;
  } bit;
  uint16_t reg;
} SERCOM_SPI_STATUS_Type;
# 1068 "include/component/sercom.h"
typedef union {
  struct {
    uint16_t PERR:1;
    uint16_t FERR:1;
    uint16_t BUFOVF:1;
    uint16_t CTS:1;
    uint16_t ISF:1;
    uint16_t COLL:1;
    uint16_t :10;
  } bit;
  uint16_t reg;
} SERCOM_USART_STATUS_Type;
# 1101 "include/component/sercom.h"
typedef union {
  struct {
    uint32_t SWRST:1;
    uint32_t ENABLE:1;
    uint32_t SYSOP:1;
    uint32_t :29;
  } bit;
  uint32_t reg;
} SERCOM_I2CM_SYNCBUSY_Type;
# 1125 "include/component/sercom.h"
typedef union {
  struct {
    uint32_t SWRST:1;
    uint32_t ENABLE:1;
    uint32_t :30;
  } bit;
  uint32_t reg;
} SERCOM_I2CS_SYNCBUSY_Type;
# 1146 "include/component/sercom.h"
typedef union {
  struct {
    uint32_t SWRST:1;
    uint32_t ENABLE:1;
    uint32_t CTRLB:1;
    uint32_t :29;
  } bit;
  uint32_t reg;
} SERCOM_SPI_SYNCBUSY_Type;
# 1170 "include/component/sercom.h"
typedef union {
  struct {
    uint32_t SWRST:1;
    uint32_t ENABLE:1;
    uint32_t CTRLB:1;
    uint32_t :29;
  } bit;
  uint32_t reg;
} SERCOM_USART_SYNCBUSY_Type;
# 1194 "include/component/sercom.h"
typedef union {
  struct {
    uint32_t ADDR:11;
    uint32_t :2;
    uint32_t LENEN:1;
    uint32_t HS:1;
    uint32_t TENBITEN:1;
    uint32_t LEN:8;
    uint32_t :8;
  } bit;
  uint32_t reg;
} SERCOM_I2CM_ADDR_Type;
# 1227 "include/component/sercom.h"
typedef union {
  struct {
    uint32_t GENCEN:1;
    uint32_t ADDR:10;
    uint32_t :4;
    uint32_t TENBITEN:1;
    uint32_t :1;
    uint32_t ADDRMASK:10;
    uint32_t :5;
  } bit;
  uint32_t reg;
} SERCOM_I2CS_ADDR_Type;
# 1258 "include/component/sercom.h"
typedef union {
  struct {
    uint32_t ADDR:8;
    uint32_t :8;
    uint32_t ADDRMASK:8;
    uint32_t :8;
  } bit;
  uint32_t reg;
} SERCOM_SPI_ADDR_Type;
# 1282 "include/component/sercom.h"
typedef union {
  struct {
    uint8_t DATA:8;
  } bit;
  uint8_t reg;
} SERCOM_I2CM_DATA_Type;
# 1300 "include/component/sercom.h"
typedef union {
  struct {
    uint8_t DATA:8;
  } bit;
  uint8_t reg;
} SERCOM_I2CS_DATA_Type;
# 1318 "include/component/sercom.h"
typedef union {
  struct {
    uint32_t DATA:9;
    uint32_t :23;
  } bit;
  uint32_t reg;
} SERCOM_SPI_DATA_Type;
# 1337 "include/component/sercom.h"
typedef union {
  struct {
    uint16_t DATA:9;
    uint16_t :7;
  } bit;
  uint16_t reg;
} SERCOM_USART_DATA_Type;
# 1356 "include/component/sercom.h"
typedef union {
  struct {
    uint8_t DBGSTOP:1;
    uint8_t :7;
  } bit;
  uint8_t reg;
} SERCOM_I2CM_DBGCTRL_Type;
# 1374 "include/component/sercom.h"
typedef union {
  struct {
    uint8_t DBGSTOP:1;
    uint8_t :7;
  } bit;
  uint8_t reg;
} SERCOM_SPI_DBGCTRL_Type;
# 1392 "include/component/sercom.h"
typedef union {
  struct {
    uint8_t DBGSTOP:1;
    uint8_t :7;
  } bit;
  uint8_t reg;
} SERCOM_USART_DBGCTRL_Type;
# 1410 "include/component/sercom.h"
typedef struct {
  volatile SERCOM_I2CM_CTRLA_Type CTRLA;
  volatile SERCOM_I2CM_CTRLB_Type CTRLB;
       RoReg8 Reserved1[0x4];
  volatile SERCOM_I2CM_BAUD_Type BAUD;
       RoReg8 Reserved2[0x4];
  volatile SERCOM_I2CM_INTENCLR_Type INTENCLR;
       RoReg8 Reserved3[0x1];
  volatile SERCOM_I2CM_INTENSET_Type INTENSET;
       RoReg8 Reserved4[0x1];
  volatile SERCOM_I2CM_INTFLAG_Type INTFLAG;
       RoReg8 Reserved5[0x1];
  volatile SERCOM_I2CM_STATUS_Type STATUS;
  volatile const SERCOM_I2CM_SYNCBUSY_Type SYNCBUSY;
       RoReg8 Reserved6[0x4];
  volatile SERCOM_I2CM_ADDR_Type ADDR;
  volatile SERCOM_I2CM_DATA_Type DATA;
       RoReg8 Reserved7[0x7];
  volatile SERCOM_I2CM_DBGCTRL_Type DBGCTRL;
} SercomI2cm;




typedef struct {
  volatile SERCOM_I2CS_CTRLA_Type CTRLA;
  volatile SERCOM_I2CS_CTRLB_Type CTRLB;
       RoReg8 Reserved1[0xC];
  volatile SERCOM_I2CS_INTENCLR_Type INTENCLR;
       RoReg8 Reserved2[0x1];
  volatile SERCOM_I2CS_INTENSET_Type INTENSET;
       RoReg8 Reserved3[0x1];
  volatile SERCOM_I2CS_INTFLAG_Type INTFLAG;
       RoReg8 Reserved4[0x1];
  volatile SERCOM_I2CS_STATUS_Type STATUS;
  volatile const SERCOM_I2CS_SYNCBUSY_Type SYNCBUSY;
       RoReg8 Reserved5[0x4];
  volatile SERCOM_I2CS_ADDR_Type ADDR;
  volatile SERCOM_I2CS_DATA_Type DATA;
} SercomI2cs;




typedef struct {
  volatile SERCOM_SPI_CTRLA_Type CTRLA;
  volatile SERCOM_SPI_CTRLB_Type CTRLB;
       RoReg8 Reserved1[0x4];
  volatile SERCOM_SPI_BAUD_Type BAUD;
       RoReg8 Reserved2[0x7];
  volatile SERCOM_SPI_INTENCLR_Type INTENCLR;
       RoReg8 Reserved3[0x1];
  volatile SERCOM_SPI_INTENSET_Type INTENSET;
       RoReg8 Reserved4[0x1];
  volatile SERCOM_SPI_INTFLAG_Type INTFLAG;
       RoReg8 Reserved5[0x1];
  volatile SERCOM_SPI_STATUS_Type STATUS;
  volatile const SERCOM_SPI_SYNCBUSY_Type SYNCBUSY;
       RoReg8 Reserved6[0x4];
  volatile SERCOM_SPI_ADDR_Type ADDR;
  volatile SERCOM_SPI_DATA_Type DATA;
       RoReg8 Reserved7[0x4];
  volatile SERCOM_SPI_DBGCTRL_Type DBGCTRL;
} SercomSpi;




typedef struct {
  volatile SERCOM_USART_CTRLA_Type CTRLA;
  volatile SERCOM_USART_CTRLB_Type CTRLB;
       RoReg8 Reserved1[0x4];
  volatile SERCOM_USART_BAUD_Type BAUD;
  volatile SERCOM_USART_RXPL_Type RXPL;
       RoReg8 Reserved2[0x5];
  volatile SERCOM_USART_INTENCLR_Type INTENCLR;
       RoReg8 Reserved3[0x1];
  volatile SERCOM_USART_INTENSET_Type INTENSET;
       RoReg8 Reserved4[0x1];
  volatile SERCOM_USART_INTFLAG_Type INTFLAG;
       RoReg8 Reserved5[0x1];
  volatile SERCOM_USART_STATUS_Type STATUS;
  volatile const SERCOM_USART_SYNCBUSY_Type SYNCBUSY;
       RoReg8 Reserved6[0x8];
  volatile SERCOM_USART_DATA_Type DATA;
       RoReg8 Reserved7[0x6];
  volatile SERCOM_USART_DBGCTRL_Type DBGCTRL;
} SercomUsart;



typedef union {
       SercomI2cm I2CM;
       SercomI2cs I2CS;
       SercomSpi SPI;
       SercomUsart USART;
} Sercom;
# 241 "include/samd10c13a.h" 2
# 1 "include/component/sysctrl.h" 1
# 61 "include/component/sysctrl.h"
typedef union {
  struct {
    uint32_t XOSCRDY:1;
    uint32_t XOSC32KRDY:1;
    uint32_t OSC32KRDY:1;
    uint32_t OSC8MRDY:1;
    uint32_t DFLLRDY:1;
    uint32_t DFLLOOB:1;
    uint32_t DFLLLCKF:1;
    uint32_t DFLLLCKC:1;
    uint32_t DFLLRCS:1;
    uint32_t BOD33RDY:1;
    uint32_t BOD33DET:1;
    uint32_t B33SRDY:1;
    uint32_t :3;
    uint32_t DPLLLCKR:1;
    uint32_t DPLLLCKF:1;
    uint32_t DPLLLTO:1;
    uint32_t :14;
  } bit;
  uint32_t reg;
} SYSCTRL_INTENCLR_Type;
# 122 "include/component/sysctrl.h"
typedef union {
  struct {
    uint32_t XOSCRDY:1;
    uint32_t XOSC32KRDY:1;
    uint32_t OSC32KRDY:1;
    uint32_t OSC8MRDY:1;
    uint32_t DFLLRDY:1;
    uint32_t DFLLOOB:1;
    uint32_t DFLLLCKF:1;
    uint32_t DFLLLCKC:1;
    uint32_t DFLLRCS:1;
    uint32_t BOD33RDY:1;
    uint32_t BOD33DET:1;
    uint32_t B33SRDY:1;
    uint32_t :3;
    uint32_t DPLLLCKR:1;
    uint32_t DPLLLCKF:1;
    uint32_t DPLLLTO:1;
    uint32_t :14;
  } bit;
  uint32_t reg;
} SYSCTRL_INTENSET_Type;
# 183 "include/component/sysctrl.h"
typedef union {
  struct {
    volatile const uint32_t XOSCRDY:1;
    volatile const uint32_t XOSC32KRDY:1;
    volatile const uint32_t OSC32KRDY:1;
    volatile const uint32_t OSC8MRDY:1;
    volatile const uint32_t DFLLRDY:1;
    volatile const uint32_t DFLLOOB:1;
    volatile const uint32_t DFLLLCKF:1;
    volatile const uint32_t DFLLLCKC:1;
    volatile const uint32_t DFLLRCS:1;
    volatile const uint32_t BOD33RDY:1;
    volatile const uint32_t BOD33DET:1;
    volatile const uint32_t B33SRDY:1;
    volatile const uint32_t :3;
    volatile const uint32_t DPLLLCKR:1;
    volatile const uint32_t DPLLLCKF:1;
    volatile const uint32_t DPLLLTO:1;
    volatile const uint32_t :14;
  } bit;
  uint32_t reg;
} SYSCTRL_INTFLAG_Type;
# 244 "include/component/sysctrl.h"
typedef union {
  struct {
    uint32_t XOSCRDY:1;
    uint32_t XOSC32KRDY:1;
    uint32_t OSC32KRDY:1;
    uint32_t OSC8MRDY:1;
    uint32_t DFLLRDY:1;
    uint32_t DFLLOOB:1;
    uint32_t DFLLLCKF:1;
    uint32_t DFLLLCKC:1;
    uint32_t DFLLRCS:1;
    uint32_t BOD33RDY:1;
    uint32_t BOD33DET:1;
    uint32_t B33SRDY:1;
    uint32_t :3;
    uint32_t DPLLLCKR:1;
    uint32_t DPLLLCKF:1;
    uint32_t DPLLLTO:1;
    uint32_t :14;
  } bit;
  uint32_t reg;
} SYSCTRL_PCLKSR_Type;
# 305 "include/component/sysctrl.h"
typedef union {
  struct {
    uint16_t :1;
    uint16_t ENABLE:1;
    uint16_t XTALEN:1;
    uint16_t :3;
    uint16_t RUNSTDBY:1;
    uint16_t ONDEMAND:1;
    uint16_t GAIN:3;
    uint16_t AMPGC:1;
    uint16_t STARTUP:4;
  } bit;
  uint16_t reg;
} SYSCTRL_XOSC_Type;
# 354 "include/component/sysctrl.h"
typedef union {
  struct {
    uint16_t :1;
    uint16_t ENABLE:1;
    uint16_t XTALEN:1;
    uint16_t EN32K:1;
    uint16_t EN1K:1;
    uint16_t AAMPEN:1;
    uint16_t RUNSTDBY:1;
    uint16_t ONDEMAND:1;
    uint16_t STARTUP:3;
    uint16_t :1;
    uint16_t WRTLOCK:1;
    uint16_t :3;
  } bit;
  uint16_t reg;
} SYSCTRL_XOSC32K_Type;
# 399 "include/component/sysctrl.h"
typedef union {
  struct {
    uint32_t :1;
    uint32_t ENABLE:1;
    uint32_t EN32K:1;
    uint32_t EN1K:1;
    uint32_t :2;
    uint32_t RUNSTDBY:1;
    uint32_t ONDEMAND:1;
    uint32_t STARTUP:3;
    uint32_t :1;
    uint32_t WRTLOCK:1;
    uint32_t :3;
    uint32_t CALIB:7;
    uint32_t :9;
  } bit;
  uint32_t reg;
} SYSCTRL_OSC32K_Type;
# 444 "include/component/sysctrl.h"
typedef union {
  struct {
    uint8_t CALIB:5;
    uint8_t :2;
    uint8_t WRTLOCK:1;
  } bit;
  uint8_t reg;
} SYSCTRL_OSCULP32K_Type;
# 466 "include/component/sysctrl.h"
typedef union {
  struct {
    uint32_t :1;
    uint32_t ENABLE:1;
    uint32_t :4;
    uint32_t RUNSTDBY:1;
    uint32_t ONDEMAND:1;
    uint32_t PRESC:2;
    uint32_t :6;
    uint32_t CALIB:12;
    uint32_t :2;
    uint32_t FRANGE:2;
  } bit;
  uint32_t reg;
} SYSCTRL_OSC8M_Type;
# 521 "include/component/sysctrl.h"
typedef union {
  struct {
    uint16_t :1;
    uint16_t ENABLE:1;
    uint16_t MODE:1;
    uint16_t STABLE:1;
    uint16_t LLAW:1;
    uint16_t USBCRM:1;
    uint16_t RUNSTDBY:1;
    uint16_t ONDEMAND:1;
    uint16_t CCDIS:1;
    uint16_t QLDIS:1;
    uint16_t BPLCKC:1;
    uint16_t WAITLOCK:1;
    uint16_t :4;
  } bit;
  uint16_t reg;
} SYSCTRL_DFLLCTRL_Type;
# 570 "include/component/sysctrl.h"
typedef union {
  struct {
    uint32_t FINE:10;
    uint32_t COARSE:6;
    uint32_t DIFF:16;
  } bit;
  uint32_t reg;
} SYSCTRL_DFLLVAL_Type;
# 596 "include/component/sysctrl.h"
typedef union {
  struct {
    uint32_t MUL:16;
    uint32_t FSTEP:10;
    uint32_t CSTEP:6;
  } bit;
  uint32_t reg;
} SYSCTRL_DFLLMUL_Type;
# 622 "include/component/sysctrl.h"
typedef union {
  struct {
    uint8_t :7;
    uint8_t READREQ:1;
  } bit;
  uint8_t reg;
} SYSCTRL_DFLLSYNC_Type;
# 640 "include/component/sysctrl.h"
typedef union {
  struct {
    uint32_t :1;
    uint32_t ENABLE:1;
    uint32_t HYST:1;
    uint32_t ACTION:2;
    uint32_t :1;
    uint32_t RUNSTDBY:1;
    uint32_t :1;
    uint32_t MODE:1;
    uint32_t CEN:1;
    uint32_t :2;
    uint32_t PSEL:4;
    uint32_t LEVEL:6;
    uint32_t :10;
  } bit;
  uint32_t reg;
} SYSCTRL_BOD33_Type;
# 724 "include/component/sysctrl.h"
typedef union {
  struct {
    uint32_t :1;
    uint32_t TSEN:1;
    uint32_t BGOUTEN:1;
    uint32_t :13;
    uint32_t CALIB:11;
    uint32_t :5;
  } bit;
  uint32_t reg;
} SYSCTRL_VREF_Type;
# 751 "include/component/sysctrl.h"
typedef union {
  struct {
    uint8_t :1;
    uint8_t ENABLE:1;
    uint8_t :4;
    uint8_t RUNSTDBY:1;
    uint8_t ONDEMAND:1;
  } bit;
  uint8_t reg;
} SYSCTRL_DPLLCTRLA_Type;
# 776 "include/component/sysctrl.h"
typedef union {
  struct {
    uint32_t LDR:12;
    uint32_t :4;
    uint32_t LDRFRAC:4;
    uint32_t :12;
  } bit;
  uint32_t reg;
} SYSCTRL_DPLLRATIO_Type;
# 800 "include/component/sysctrl.h"
typedef union {
  struct {
    uint32_t FILTER:2;
    uint32_t LPEN:1;
    uint32_t WUF:1;
    uint32_t REFCLK:2;
    uint32_t :2;
    uint32_t LTIME:3;
    uint32_t :1;
    uint32_t LBYPASS:1;
    uint32_t :3;
    uint32_t DIV:11;
    uint32_t :5;
  } bit;
  uint32_t reg;
} SYSCTRL_DPLLCTRLB_Type;
# 867 "include/component/sysctrl.h"
typedef union {
  struct {
    uint8_t LOCK:1;
    uint8_t CLKRDY:1;
    uint8_t ENABLE:1;
    uint8_t DIV:1;
    uint8_t :4;
  } bit;
  uint8_t reg;
} SYSCTRL_DPLLSTATUS_Type;
# 894 "include/component/sysctrl.h"
typedef struct {
  volatile SYSCTRL_INTENCLR_Type INTENCLR;
  volatile SYSCTRL_INTENSET_Type INTENSET;
  volatile SYSCTRL_INTFLAG_Type INTFLAG;
  volatile const SYSCTRL_PCLKSR_Type PCLKSR;
  volatile SYSCTRL_XOSC_Type XOSC;
       RoReg8 Reserved1[0x2];
  volatile SYSCTRL_XOSC32K_Type XOSC32K;
       RoReg8 Reserved2[0x2];
  volatile SYSCTRL_OSC32K_Type OSC32K;
  volatile SYSCTRL_OSCULP32K_Type OSCULP32K;
       RoReg8 Reserved3[0x3];
  volatile SYSCTRL_OSC8M_Type OSC8M;
  volatile SYSCTRL_DFLLCTRL_Type DFLLCTRL;
       RoReg8 Reserved4[0x2];
  volatile SYSCTRL_DFLLVAL_Type DFLLVAL;
  volatile SYSCTRL_DFLLMUL_Type DFLLMUL;
  volatile SYSCTRL_DFLLSYNC_Type DFLLSYNC;
       RoReg8 Reserved5[0x3];
  volatile SYSCTRL_BOD33_Type BOD33;
       RoReg8 Reserved6[0x8];
  volatile SYSCTRL_VREF_Type VREF;
  volatile SYSCTRL_DPLLCTRLA_Type DPLLCTRLA;
       RoReg8 Reserved7[0x3];
  volatile SYSCTRL_DPLLRATIO_Type DPLLRATIO;
  volatile SYSCTRL_DPLLCTRLB_Type DPLLCTRLB;
  volatile const SYSCTRL_DPLLSTATUS_Type DPLLSTATUS;
} Sysctrl;
# 242 "include/samd10c13a.h" 2
# 1 "include/component/tc.h" 1
# 61 "include/component/tc.h"
typedef union {
  struct {
    uint16_t SWRST:1;
    uint16_t ENABLE:1;
    uint16_t MODE:2;
    uint16_t :1;
    uint16_t WAVEGEN:2;
    uint16_t :1;
    uint16_t PRESCALER:3;
    uint16_t RUNSTDBY:1;
    uint16_t PRESCSYNC:2;
    uint16_t :2;
  } bit;
  uint16_t reg;
} TC_CTRLA_Type;
# 139 "include/component/tc.h"
typedef union {
  struct {
    uint16_t ADDR:5;
    uint16_t :9;
    uint16_t RCONT:1;
    uint16_t RREQ:1;
  } bit;
  uint16_t reg;
} TC_READREQ_Type;
# 164 "include/component/tc.h"
typedef union {
  struct {
    uint8_t DIR:1;
    uint8_t :1;
    uint8_t ONESHOT:1;
    uint8_t :3;
    uint8_t CMD:2;
  } bit;
  uint8_t reg;
} TC_CTRLBCLR_Type;
# 196 "include/component/tc.h"
typedef union {
  struct {
    uint8_t DIR:1;
    uint8_t :1;
    uint8_t ONESHOT:1;
    uint8_t :3;
    uint8_t CMD:2;
  } bit;
  uint8_t reg;
} TC_CTRLBSET_Type;
# 228 "include/component/tc.h"
typedef union {
  struct {
    uint8_t INVEN0:1;
    uint8_t INVEN1:1;
    uint8_t :2;
    uint8_t CPTEN0:1;
    uint8_t CPTEN1:1;
    uint8_t :2;
  } bit;
  struct {
    uint8_t INVEN:2;
    uint8_t :2;
    uint8_t CPTEN:2;
    uint8_t :2;
  } vec;
  uint8_t reg;
} TC_CTRLC_Type;
# 268 "include/component/tc.h"
typedef union {
  struct {
    uint8_t DBGRUN:1;
    uint8_t :7;
  } bit;
  uint8_t reg;
} TC_DBGCTRL_Type;
# 286 "include/component/tc.h"
typedef union {
  struct {
    uint16_t EVACT:3;
    uint16_t :1;
    uint16_t TCINV:1;
    uint16_t TCEI:1;
    uint16_t :2;
    uint16_t OVFEO:1;
    uint16_t :3;
    uint16_t MCEO0:1;
    uint16_t MCEO1:1;
    uint16_t :2;
  } bit;
  struct {
    uint16_t :12;
    uint16_t MCEO:2;
    uint16_t :2;
  } vec;
  uint16_t reg;
} TC_EVCTRL_Type;
# 343 "include/component/tc.h"
typedef union {
  struct {
    uint8_t OVF:1;
    uint8_t ERR:1;
    uint8_t :1;
    uint8_t SYNCRDY:1;
    uint8_t MC0:1;
    uint8_t MC1:1;
    uint8_t :2;
  } bit;
  struct {
    uint8_t :4;
    uint8_t MC:2;
    uint8_t :2;
  } vec;
  uint8_t reg;
} TC_INTENCLR_Type;
# 382 "include/component/tc.h"
typedef union {
  struct {
    uint8_t OVF:1;
    uint8_t ERR:1;
    uint8_t :1;
    uint8_t SYNCRDY:1;
    uint8_t MC0:1;
    uint8_t MC1:1;
    uint8_t :2;
  } bit;
  struct {
    uint8_t :4;
    uint8_t MC:2;
    uint8_t :2;
  } vec;
  uint8_t reg;
} TC_INTENSET_Type;
# 421 "include/component/tc.h"
typedef union {
  struct {
    volatile const uint8_t OVF:1;
    volatile const uint8_t ERR:1;
    volatile const uint8_t :1;
    volatile const uint8_t SYNCRDY:1;
    volatile const uint8_t MC0:1;
    volatile const uint8_t MC1:1;
    volatile const uint8_t :2;
  } bit;
  struct {
    volatile const uint8_t :4;
    volatile const uint8_t MC:2;
    volatile const uint8_t :2;
  } vec;
  uint8_t reg;
} TC_INTFLAG_Type;
# 460 "include/component/tc.h"
typedef union {
  struct {
    uint8_t :3;
    uint8_t STOP:1;
    uint8_t SLAVE:1;
    uint8_t :2;
    uint8_t SYNCBUSY:1;
  } bit;
  uint8_t reg;
} TC_STATUS_Type;
# 485 "include/component/tc.h"
typedef union {
  struct {
    uint16_t COUNT:16;
  } bit;
  uint16_t reg;
} TC_COUNT16_COUNT_Type;
# 503 "include/component/tc.h"
typedef union {
  struct {
    uint32_t COUNT:32;
  } bit;
  uint32_t reg;
} TC_COUNT32_COUNT_Type;
# 521 "include/component/tc.h"
typedef union {
  struct {
    uint8_t COUNT:8;
  } bit;
  uint8_t reg;
} TC_COUNT8_COUNT_Type;
# 539 "include/component/tc.h"
typedef union {
  struct {
    uint8_t PER:8;
  } bit;
  uint8_t reg;
} TC_COUNT8_PER_Type;
# 557 "include/component/tc.h"
typedef union {
  struct {
    uint16_t CC:16;
  } bit;
  uint16_t reg;
} TC_COUNT16_CC_Type;
# 575 "include/component/tc.h"
typedef union {
  struct {
    uint32_t CC:32;
  } bit;
  uint32_t reg;
} TC_COUNT32_CC_Type;
# 593 "include/component/tc.h"
typedef union {
  struct {
    uint8_t CC:8;
  } bit;
  uint8_t reg;
} TC_COUNT8_CC_Type;
# 611 "include/component/tc.h"
typedef struct {
  volatile TC_CTRLA_Type CTRLA;
  volatile TC_READREQ_Type READREQ;
  volatile TC_CTRLBCLR_Type CTRLBCLR;
  volatile TC_CTRLBSET_Type CTRLBSET;
  volatile TC_CTRLC_Type CTRLC;
       RoReg8 Reserved1[0x1];
  volatile TC_DBGCTRL_Type DBGCTRL;
       RoReg8 Reserved2[0x1];
  volatile TC_EVCTRL_Type EVCTRL;
  volatile TC_INTENCLR_Type INTENCLR;
  volatile TC_INTENSET_Type INTENSET;
  volatile TC_INTFLAG_Type INTFLAG;
  volatile const TC_STATUS_Type STATUS;
  volatile TC_COUNT8_COUNT_Type COUNT;
       RoReg8 Reserved3[0x3];
  volatile TC_COUNT8_PER_Type PER;
       RoReg8 Reserved4[0x3];
  volatile TC_COUNT8_CC_Type CC[2];
} TcCount8;




typedef struct {
  volatile TC_CTRLA_Type CTRLA;
  volatile TC_READREQ_Type READREQ;
  volatile TC_CTRLBCLR_Type CTRLBCLR;
  volatile TC_CTRLBSET_Type CTRLBSET;
  volatile TC_CTRLC_Type CTRLC;
       RoReg8 Reserved1[0x1];
  volatile TC_DBGCTRL_Type DBGCTRL;
       RoReg8 Reserved2[0x1];
  volatile TC_EVCTRL_Type EVCTRL;
  volatile TC_INTENCLR_Type INTENCLR;
  volatile TC_INTENSET_Type INTENSET;
  volatile TC_INTFLAG_Type INTFLAG;
  volatile const TC_STATUS_Type STATUS;
  volatile TC_COUNT16_COUNT_Type COUNT;
       RoReg8 Reserved3[0x6];
  volatile TC_COUNT16_CC_Type CC[2];
} TcCount16;




typedef struct {
  volatile TC_CTRLA_Type CTRLA;
  volatile TC_READREQ_Type READREQ;
  volatile TC_CTRLBCLR_Type CTRLBCLR;
  volatile TC_CTRLBSET_Type CTRLBSET;
  volatile TC_CTRLC_Type CTRLC;
       RoReg8 Reserved1[0x1];
  volatile TC_DBGCTRL_Type DBGCTRL;
       RoReg8 Reserved2[0x1];
  volatile TC_EVCTRL_Type EVCTRL;
  volatile TC_INTENCLR_Type INTENCLR;
  volatile TC_INTENSET_Type INTENSET;
  volatile TC_INTFLAG_Type INTFLAG;
  volatile const TC_STATUS_Type STATUS;
  volatile TC_COUNT32_COUNT_Type COUNT;
       RoReg8 Reserved3[0x4];
  volatile TC_COUNT32_CC_Type CC[2];
} TcCount32;



typedef union {
       TcCount8 COUNT8;
       TcCount16 COUNT16;
       TcCount32 COUNT32;
} Tc;
# 243 "include/samd10c13a.h" 2
# 1 "include/component/tcc.h" 1
# 61 "include/component/tcc.h"
typedef union {
  struct {
    uint32_t SWRST:1;
    uint32_t ENABLE:1;
    uint32_t :3;
    uint32_t RESOLUTION:2;
    uint32_t :1;
    uint32_t PRESCALER:3;
    uint32_t RUNSTDBY:1;
    uint32_t PRESCSYNC:2;
    uint32_t ALOCK:1;
    uint32_t :9;
    uint32_t CPTEN0:1;
    uint32_t CPTEN1:1;
    uint32_t CPTEN2:1;
    uint32_t CPTEN3:1;
    uint32_t :4;
  } bit;
  struct {
    uint32_t :24;
    uint32_t CPTEN:4;
    uint32_t :4;
  } vec;
  uint32_t reg;
} TCC_CTRLA_Type;
# 153 "include/component/tcc.h"
typedef union {
  struct {
    uint8_t DIR:1;
    uint8_t LUPD:1;
    uint8_t ONESHOT:1;
    uint8_t IDXCMD:2;
    uint8_t CMD:3;
  } bit;
  uint8_t reg;
} TCC_CTRLBCLR_Type;
# 202 "include/component/tcc.h"
typedef union {
  struct {
    uint8_t DIR:1;
    uint8_t LUPD:1;
    uint8_t ONESHOT:1;
    uint8_t IDXCMD:2;
    uint8_t CMD:3;
  } bit;
  uint8_t reg;
} TCC_CTRLBSET_Type;
# 251 "include/component/tcc.h"
typedef union {
  struct {
    uint32_t SWRST:1;
    uint32_t ENABLE:1;
    uint32_t CTRLB:1;
    uint32_t STATUS:1;
    uint32_t COUNT:1;
    uint32_t PATT:1;
    uint32_t WAVE:1;
    uint32_t PER:1;
    uint32_t CC0:1;
    uint32_t CC1:1;
    uint32_t CC2:1;
    uint32_t CC3:1;
    uint32_t :4;
    uint32_t PATTB:1;
    uint32_t WAVEB:1;
    uint32_t PERB:1;
    uint32_t CCB0:1;
    uint32_t CCB1:1;
    uint32_t CCB2:1;
    uint32_t CCB3:1;
    uint32_t :9;
  } bit;
  struct {
    uint32_t :8;
    uint32_t CC:4;
    uint32_t :7;
    uint32_t CCB:4;
    uint32_t :9;
  } vec;
  uint32_t reg;
} TCC_SYNCBUSY_Type;
# 337 "include/component/tcc.h"
typedef union {
  struct {
    uint32_t SRC:2;
    uint32_t :1;
    uint32_t KEEP:1;
    uint32_t QUAL:1;
    uint32_t BLANK:2;
    uint32_t RESTART:1;
    uint32_t HALT:2;
    uint32_t CHSEL:2;
    uint32_t CAPTURE:3;
    uint32_t BLANKPRESC:1;
    uint32_t BLANKVAL:8;
    uint32_t FILTERVAL:4;
    uint32_t :4;
  } bit;
  uint32_t reg;
} TCC_FCTRLA_Type;
# 441 "include/component/tcc.h"
typedef union {
  struct {
    uint32_t SRC:2;
    uint32_t :1;
    uint32_t KEEP:1;
    uint32_t QUAL:1;
    uint32_t BLANK:2;
    uint32_t RESTART:1;
    uint32_t HALT:2;
    uint32_t CHSEL:2;
    uint32_t CAPTURE:3;
    uint32_t BLANKPRESC:1;
    uint32_t BLANKVAL:8;
    uint32_t FILTERVAL:4;
    uint32_t :4;
  } bit;
  uint32_t reg;
} TCC_FCTRLB_Type;
# 545 "include/component/tcc.h"
typedef union {
  struct {
    uint32_t OTMX:2;
    uint32_t :6;
    uint32_t DTIEN0:1;
    uint32_t DTIEN1:1;
    uint32_t DTIEN2:1;
    uint32_t DTIEN3:1;
    uint32_t :4;
    uint32_t DTLS:8;
    uint32_t DTHS:8;
  } bit;
  struct {
    uint32_t :8;
    uint32_t DTIEN:4;
    uint32_t :20;
  } vec;
  uint32_t reg;
} TCC_WEXCTRL_Type;
# 593 "include/component/tcc.h"
typedef union {
  struct {
    uint32_t NRE0:1;
    uint32_t NRE1:1;
    uint32_t NRE2:1;
    uint32_t NRE3:1;
    uint32_t NRE4:1;
    uint32_t NRE5:1;
    uint32_t NRE6:1;
    uint32_t NRE7:1;
    uint32_t NRV0:1;
    uint32_t NRV1:1;
    uint32_t NRV2:1;
    uint32_t NRV3:1;
    uint32_t NRV4:1;
    uint32_t NRV5:1;
    uint32_t NRV6:1;
    uint32_t NRV7:1;
    uint32_t INVEN0:1;
    uint32_t INVEN1:1;
    uint32_t INVEN2:1;
    uint32_t INVEN3:1;
    uint32_t INVEN4:1;
    uint32_t INVEN5:1;
    uint32_t INVEN6:1;
    uint32_t INVEN7:1;
    uint32_t FILTERVAL0:4;
    uint32_t FILTERVAL1:4;
  } bit;
  struct {
    uint32_t NRE:8;
    uint32_t NRV:8;
    uint32_t INVEN:8;
    uint32_t :8;
  } vec;
  uint32_t reg;
} TCC_DRVCTRL_Type;
# 702 "include/component/tcc.h"
typedef union {
  struct {
    uint8_t DBGRUN:1;
    uint8_t :1;
    uint8_t FDDBD:1;
    uint8_t :5;
  } bit;
  uint8_t reg;
} TCC_DBGCTRL_Type;
# 724 "include/component/tcc.h"
typedef union {
  struct {
    uint32_t EVACT0:3;
    uint32_t EVACT1:3;
    uint32_t CNTSEL:2;
    uint32_t OVFEO:1;
    uint32_t TRGEO:1;
    uint32_t CNTEO:1;
    uint32_t :1;
    uint32_t TCINV0:1;
    uint32_t TCINV1:1;
    uint32_t TCEI0:1;
    uint32_t TCEI1:1;
    uint32_t MCEI0:1;
    uint32_t MCEI1:1;
    uint32_t MCEI2:1;
    uint32_t MCEI3:1;
    uint32_t :4;
    uint32_t MCEO0:1;
    uint32_t MCEO1:1;
    uint32_t MCEO2:1;
    uint32_t MCEO3:1;
    uint32_t :4;
  } bit;
  struct {
    uint32_t :12;
    uint32_t TCINV:2;
    uint32_t TCEI:2;
    uint32_t MCEI:4;
    uint32_t :4;
    uint32_t MCEO:4;
    uint32_t :4;
  } vec;
  uint32_t reg;
} TCC_EVCTRL_Type;
# 857 "include/component/tcc.h"
typedef union {
  struct {
    uint32_t OVF:1;
    uint32_t TRG:1;
    uint32_t CNT:1;
    uint32_t ERR:1;
    uint32_t :7;
    uint32_t DFS:1;
    uint32_t FAULTA:1;
    uint32_t FAULTB:1;
    uint32_t FAULT0:1;
    uint32_t FAULT1:1;
    uint32_t MC0:1;
    uint32_t MC1:1;
    uint32_t MC2:1;
    uint32_t MC3:1;
    uint32_t :12;
  } bit;
  struct {
    uint32_t :16;
    uint32_t MC:4;
    uint32_t :12;
  } vec;
  uint32_t reg;
} TCC_INTENCLR_Type;
# 920 "include/component/tcc.h"
typedef union {
  struct {
    uint32_t OVF:1;
    uint32_t TRG:1;
    uint32_t CNT:1;
    uint32_t ERR:1;
    uint32_t :7;
    uint32_t DFS:1;
    uint32_t FAULTA:1;
    uint32_t FAULTB:1;
    uint32_t FAULT0:1;
    uint32_t FAULT1:1;
    uint32_t MC0:1;
    uint32_t MC1:1;
    uint32_t MC2:1;
    uint32_t MC3:1;
    uint32_t :12;
  } bit;
  struct {
    uint32_t :16;
    uint32_t MC:4;
    uint32_t :12;
  } vec;
  uint32_t reg;
} TCC_INTENSET_Type;
# 983 "include/component/tcc.h"
typedef union {
  struct {
    volatile const uint32_t OVF:1;
    volatile const uint32_t TRG:1;
    volatile const uint32_t CNT:1;
    volatile const uint32_t ERR:1;
    volatile const uint32_t :7;
    volatile const uint32_t DFS:1;
    volatile const uint32_t FAULTA:1;
    volatile const uint32_t FAULTB:1;
    volatile const uint32_t FAULT0:1;
    volatile const uint32_t FAULT1:1;
    volatile const uint32_t MC0:1;
    volatile const uint32_t MC1:1;
    volatile const uint32_t MC2:1;
    volatile const uint32_t MC3:1;
    volatile const uint32_t :12;
  } bit;
  struct {
    volatile const uint32_t :16;
    volatile const uint32_t MC:4;
    volatile const uint32_t :12;
  } vec;
  uint32_t reg;
} TCC_INTFLAG_Type;
# 1046 "include/component/tcc.h"
typedef union {
  struct {
    uint32_t STOP:1;
    uint32_t IDX:1;
    uint32_t :1;
    uint32_t DFS:1;
    uint32_t SLAVE:1;
    uint32_t PATTBV:1;
    uint32_t WAVEBV:1;
    uint32_t PERBV:1;
    uint32_t FAULTAIN:1;
    uint32_t FAULTBIN:1;
    uint32_t FAULT0IN:1;
    uint32_t FAULT1IN:1;
    uint32_t FAULTA:1;
    uint32_t FAULTB:1;
    uint32_t FAULT0:1;
    uint32_t FAULT1:1;
    uint32_t CCBV0:1;
    uint32_t CCBV1:1;
    uint32_t CCBV2:1;
    uint32_t CCBV3:1;
    uint32_t :4;
    uint32_t CMP0:1;
    uint32_t CMP1:1;
    uint32_t CMP2:1;
    uint32_t CMP3:1;
    uint32_t :4;
  } bit;
  struct {
    uint32_t :16;
    uint32_t CCBV:4;
    uint32_t :4;
    uint32_t CMP:4;
    uint32_t :4;
  } vec;
  uint32_t reg;
} TCC_STATUS_Type;
# 1145 "include/component/tcc.h"
typedef union {
  struct {
    uint32_t :4;
    uint32_t COUNT:20;
    uint32_t :8;
  } DITH4;
  struct {
    uint32_t :5;
    uint32_t COUNT:19;
    uint32_t :8;
  } DITH5;
  struct {
    uint32_t :6;
    uint32_t COUNT:18;
    uint32_t :8;
  } DITH6;
  struct {
    uint32_t COUNT:24;
    uint32_t :8;
  } bit;
  uint32_t reg;
} TCC_COUNT_Type;
# 1197 "include/component/tcc.h"
typedef union {
  struct {
    uint16_t PGE0:1;
    uint16_t PGE1:1;
    uint16_t PGE2:1;
    uint16_t PGE3:1;
    uint16_t PGE4:1;
    uint16_t PGE5:1;
    uint16_t PGE6:1;
    uint16_t PGE7:1;
    uint16_t PGV0:1;
    uint16_t PGV1:1;
    uint16_t PGV2:1;
    uint16_t PGV3:1;
    uint16_t PGV4:1;
    uint16_t PGV5:1;
    uint16_t PGV6:1;
    uint16_t PGV7:1;
  } bit;
  struct {
    uint16_t PGE:8;
    uint16_t PGV:8;
  } vec;
  uint16_t reg;
} TCC_PATT_Type;
# 1269 "include/component/tcc.h"
typedef union {
  struct {
    uint32_t WAVEGEN:3;
    uint32_t :1;
    uint32_t RAMP:2;
    uint32_t :1;
    uint32_t CIPEREN:1;
    uint32_t CICCEN0:1;
    uint32_t CICCEN1:1;
    uint32_t CICCEN2:1;
    uint32_t CICCEN3:1;
    uint32_t :4;
    uint32_t POL0:1;
    uint32_t POL1:1;
    uint32_t POL2:1;
    uint32_t POL3:1;
    uint32_t :4;
    uint32_t SWAP0:1;
    uint32_t SWAP1:1;
    uint32_t SWAP2:1;
    uint32_t SWAP3:1;
    uint32_t :4;
  } bit;
  struct {
    uint32_t :8;
    uint32_t CICCEN:4;
    uint32_t :4;
    uint32_t POL:4;
    uint32_t :4;
    uint32_t SWAP:4;
    uint32_t :4;
  } vec;
  uint32_t reg;
} TCC_WAVE_Type;
# 1375 "include/component/tcc.h"
typedef union {
  struct {
    uint32_t DITHERCY:4;
    uint32_t PER:20;
    uint32_t :8;
  } DITH4;
  struct {
    uint32_t DITHERCY:5;
    uint32_t PER:19;
    uint32_t :8;
  } DITH5;
  struct {
    uint32_t DITHERCY:6;
    uint32_t PER:18;
    uint32_t :8;
  } DITH6;
  struct {
    uint32_t PER:24;
    uint32_t :8;
  } bit;
  uint32_t reg;
} TCC_PER_Type;
# 1436 "include/component/tcc.h"
typedef union {
  struct {
    uint32_t DITHERCY:4;
    uint32_t CC:20;
    uint32_t :8;
  } DITH4;
  struct {
    uint32_t DITHERCY:5;
    uint32_t CC:19;
    uint32_t :8;
  } DITH5;
  struct {
    uint32_t DITHERCY:6;
    uint32_t CC:18;
    uint32_t :8;
  } DITH6;
  struct {
    uint32_t CC:24;
    uint32_t :8;
  } bit;
  uint32_t reg;
} TCC_CC_Type;
# 1497 "include/component/tcc.h"
typedef union {
  struct {
    uint16_t PGEB0:1;
    uint16_t PGEB1:1;
    uint16_t PGEB2:1;
    uint16_t PGEB3:1;
    uint16_t PGEB4:1;
    uint16_t PGEB5:1;
    uint16_t PGEB6:1;
    uint16_t PGEB7:1;
    uint16_t PGVB0:1;
    uint16_t PGVB1:1;
    uint16_t PGVB2:1;
    uint16_t PGVB3:1;
    uint16_t PGVB4:1;
    uint16_t PGVB5:1;
    uint16_t PGVB6:1;
    uint16_t PGVB7:1;
  } bit;
  struct {
    uint16_t PGEB:8;
    uint16_t PGVB:8;
  } vec;
  uint16_t reg;
} TCC_PATTB_Type;
# 1569 "include/component/tcc.h"
typedef union {
  struct {
    uint32_t WAVEGENB:3;
    uint32_t :1;
    uint32_t RAMPB:2;
    uint32_t :1;
    uint32_t CIPERENB:1;
    uint32_t CICCENB0:1;
    uint32_t CICCENB1:1;
    uint32_t CICCENB2:1;
    uint32_t CICCENB3:1;
    uint32_t :4;
    uint32_t POLB0:1;
    uint32_t POLB1:1;
    uint32_t POLB2:1;
    uint32_t POLB3:1;
    uint32_t :4;
    uint32_t SWAPB0:1;
    uint32_t SWAPB1:1;
    uint32_t SWAPB2:1;
    uint32_t SWAPB3:1;
    uint32_t :4;
  } bit;
  struct {
    uint32_t :8;
    uint32_t CICCENB:4;
    uint32_t :4;
    uint32_t POLB:4;
    uint32_t :4;
    uint32_t SWAPB:4;
    uint32_t :4;
  } vec;
  uint32_t reg;
} TCC_WAVEB_Type;
# 1673 "include/component/tcc.h"
typedef union {
  struct {
    uint32_t DITHERCYB:4;
    uint32_t PERB:20;
    uint32_t :8;
  } DITH4;
  struct {
    uint32_t DITHERCYB:5;
    uint32_t PERB:19;
    uint32_t :8;
  } DITH5;
  struct {
    uint32_t DITHERCYB:6;
    uint32_t PERB:18;
    uint32_t :8;
  } DITH6;
  struct {
    uint32_t PERB:24;
    uint32_t :8;
  } bit;
  uint32_t reg;
} TCC_PERB_Type;
# 1734 "include/component/tcc.h"
typedef union {
  struct {
    uint32_t DITHERCYB:4;
    uint32_t CCB:20;
    uint32_t :8;
  } DITH4;
  struct {
    uint32_t DITHERCYB:5;
    uint32_t CCB:19;
    uint32_t :8;
  } DITH5;
  struct {
    uint32_t DITHERCYB:6;
    uint32_t CCB:18;
    uint32_t :8;
  } DITH6;
  struct {
    uint32_t CCB:24;
    uint32_t :8;
  } bit;
  uint32_t reg;
} TCC_CCB_Type;
# 1795 "include/component/tcc.h"
typedef struct {
  volatile TCC_CTRLA_Type CTRLA;
  volatile TCC_CTRLBCLR_Type CTRLBCLR;
  volatile TCC_CTRLBSET_Type CTRLBSET;
       RoReg8 Reserved1[0x2];
  volatile const TCC_SYNCBUSY_Type SYNCBUSY;
  volatile TCC_FCTRLA_Type FCTRLA;
  volatile TCC_FCTRLB_Type FCTRLB;
  volatile TCC_WEXCTRL_Type WEXCTRL;
  volatile TCC_DRVCTRL_Type DRVCTRL;
       RoReg8 Reserved2[0x2];
  volatile TCC_DBGCTRL_Type DBGCTRL;
       RoReg8 Reserved3[0x1];
  volatile TCC_EVCTRL_Type EVCTRL;
  volatile TCC_INTENCLR_Type INTENCLR;
  volatile TCC_INTENSET_Type INTENSET;
  volatile TCC_INTFLAG_Type INTFLAG;
  volatile TCC_STATUS_Type STATUS;
  volatile TCC_COUNT_Type COUNT;
  volatile TCC_PATT_Type PATT;
       RoReg8 Reserved4[0x2];
  volatile TCC_WAVE_Type WAVE;
  volatile TCC_PER_Type PER;
  volatile TCC_CC_Type CC[4];
       RoReg8 Reserved5[0x10];
  volatile TCC_PATTB_Type PATTB;
       RoReg8 Reserved6[0x2];
  volatile TCC_WAVEB_Type WAVEB;
  volatile TCC_PERB_Type PERB;
  volatile TCC_CCB_Type CCB[4];
} Tcc;
# 244 "include/samd10c13a.h" 2
# 1 "include/component/wdt.h" 1
# 61 "include/component/wdt.h"
typedef union {
  struct {
    uint8_t :1;
    uint8_t ENABLE:1;
    uint8_t WEN:1;
    uint8_t :4;
    uint8_t ALWAYSON:1;
  } bit;
  uint8_t reg;
} WDT_CTRL_Type;
# 86 "include/component/wdt.h"
typedef union {
  struct {
    uint8_t PER:4;
    uint8_t WINDOW:4;
  } bit;
  uint8_t reg;
} WDT_CONFIG_Type;
# 156 "include/component/wdt.h"
typedef union {
  struct {
    uint8_t EWOFFSET:4;
    uint8_t :4;
  } bit;
  uint8_t reg;
} WDT_EWCTRL_Type;
# 199 "include/component/wdt.h"
typedef union {
  struct {
    uint8_t EW:1;
    uint8_t :7;
  } bit;
  uint8_t reg;
} WDT_INTENCLR_Type;
# 217 "include/component/wdt.h"
typedef union {
  struct {
    uint8_t EW:1;
    uint8_t :7;
  } bit;
  uint8_t reg;
} WDT_INTENSET_Type;
# 235 "include/component/wdt.h"
typedef union {
  struct {
    volatile const uint8_t EW:1;
    volatile const uint8_t :7;
  } bit;
  uint8_t reg;
} WDT_INTFLAG_Type;
# 253 "include/component/wdt.h"
typedef union {
  struct {
    uint8_t :7;
    uint8_t SYNCBUSY:1;
  } bit;
  uint8_t reg;
} WDT_STATUS_Type;
# 271 "include/component/wdt.h"
typedef union {
  struct {
    uint8_t CLEAR:8;
  } bit;
  uint8_t reg;
} WDT_CLEAR_Type;
# 291 "include/component/wdt.h"
typedef struct {
  volatile WDT_CTRL_Type CTRL;
  volatile WDT_CONFIG_Type CONFIG;
  volatile WDT_EWCTRL_Type EWCTRL;
       RoReg8 Reserved1[0x1];
  volatile WDT_INTENCLR_Type INTENCLR;
  volatile WDT_INTENSET_Type INTENSET;
  volatile WDT_INTFLAG_Type INTFLAG;
  volatile const WDT_STATUS_Type STATUS;
  volatile WDT_CLEAR_Type CLEAR;
} Wdt;
# 245 "include/samd10c13a.h" 2
# 253 "include/samd10c13a.h"
# 1 "include/instance/ac.h" 1
# 254 "include/samd10c13a.h" 2
# 1 "include/instance/adc.h" 1
# 255 "include/samd10c13a.h" 2
# 1 "include/instance/dac.h" 1
# 256 "include/samd10c13a.h" 2
# 1 "include/instance/dmac.h" 1
# 257 "include/samd10c13a.h" 2
# 1 "include/instance/dsu.h" 1
# 258 "include/samd10c13a.h" 2
# 1 "include/instance/eic.h" 1
# 259 "include/samd10c13a.h" 2
# 1 "include/instance/evsys.h" 1
# 260 "include/samd10c13a.h" 2
# 1 "include/instance/gclk.h" 1
# 261 "include/samd10c13a.h" 2
# 1 "include/instance/sbmatrix.h" 1
# 262 "include/samd10c13a.h" 2
# 1 "include/instance/mtb.h" 1
# 263 "include/samd10c13a.h" 2
# 1 "include/instance/nvmctrl.h" 1
# 264 "include/samd10c13a.h" 2
# 1 "include/instance/pac0.h" 1
# 265 "include/samd10c13a.h" 2
# 1 "include/instance/pac1.h" 1
# 266 "include/samd10c13a.h" 2
# 1 "include/instance/pac2.h" 1
# 267 "include/samd10c13a.h" 2
# 1 "include/instance/pm.h" 1
# 268 "include/samd10c13a.h" 2
# 1 "include/instance/port.h" 1
# 269 "include/samd10c13a.h" 2
# 1 "include/instance/rtc.h" 1
# 270 "include/samd10c13a.h" 2
# 1 "include/instance/sercom0.h" 1
# 271 "include/samd10c13a.h" 2
# 1 "include/instance/sercom1.h" 1
# 272 "include/samd10c13a.h" 2
# 1 "include/instance/sysctrl.h" 1
# 273 "include/samd10c13a.h" 2
# 1 "include/instance/tc1.h" 1
# 274 "include/samd10c13a.h" 2
# 1 "include/instance/tc2.h" 1
# 275 "include/samd10c13a.h" 2
# 1 "include/instance/tcc0.h" 1
# 276 "include/samd10c13a.h" 2
# 1 "include/instance/wdt.h" 1
# 277 "include/samd10c13a.h" 2
# 464 "include/samd10c13a.h"
# 1 "include/pio/samd10c13a.h" 1
# 465 "include/samd10c13a.h" 2
# 59 "include/samd10.h" 2
