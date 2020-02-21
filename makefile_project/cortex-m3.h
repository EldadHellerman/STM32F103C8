#ifndef CORTEX_M3_H
#define CORTEX_M3_H

#include <stdint.h>

typedef struct{ //MPU_TypeDef
  uint32_t TYPER;
  uint32_t CR;
  uint32_t RNR;
  uint32_t RBAR;
  uint32_t RASR;
}MPU_TypeDef;

typedef struct{ //NVIC_TypeDef
  uint32_t ISER0;
  uint32_t ISER1;
  uint32_t ISER2;
  uint32_t RESERVED_0[116];
  uint32_t ICER0;
  uint32_t ICER1;
  uint32_t ICER2;
  uint32_t RESERVED_1[116];
  uint32_t ISPR0;
  uint32_t ISPR1;
  uint32_t ISPR2;
  uint32_t RESERVED_2[116];
  uint32_t ICPR0;
  uint32_t ICPR1;
  uint32_t ICPR2;
  uint32_t RESERVED_3[116];
  uint32_t IABR0;
  uint32_t IABR1;
  uint32_t IABR2;
  uint32_t RESERVED_4[244];
  //this can be an array of IP[80] length
  uint32_t IPR0;
  uint32_t IPR1;
  uint32_t IPR2;
  uint32_t IPR3;
  uint32_t IPR4;
  uint32_t IPR5;
  uint32_t IPR6;
  uint32_t IPR7;
  uint32_t IPR8;
  uint32_t IPR9;
  uint32_t IPR10;
  uint32_t IPR11;
  uint32_t IPR12;
  uint32_t IPR13;
  uint32_t IPR14;
  uint32_t IPR15;
  uint32_t IPR16;
  uint32_t IPR17;
  uint32_t IPR18;
  uint32_t IPR19;
  uint32_t IPR20;
  uint32_t RESERVED_5[2732];
  uint32_t STIR;
}NVIC_TypeDef;

typedef struct{ //SCB_TypeDef
  uint32_t CPUID;
  uint32_t ICSR;
  uint32_t VTOR;
  uint32_t AIRCR;
  uint32_t SCR;
  uint32_t CCR;
  uint32_t SHPR1;
  uint32_t SHPR2;
  uint32_t SHPR3;
  uint32_t SHCRS;
  uint32_t CFSR;
  uint32_t HFSR;
  uint32_t MMAR;
  uint32_t BFAR;
}SCB_TypeDef;

typedef struct{ //STK_TypeDef
  uint32_t CTRL;
  uint32_t LOAD;
  uint32_t VAL;
  uint32_t CALIB;
}STK_TypeDef;


#define CORE_PERIPH_BASE                         0xE0000000U

#define STK_BASE                                 (CORE_PERIPH_BASE + 0x0000E010)
#define NVIC_BASE                                (CORE_PERIPH_BASE + 0x0000E100)
#define SCB_BASE                                 (CORE_PERIPH_BASE + 0x0000ED00)
#define MPU_BASE                                 (CORE_PERIPH_BASE + 0x0000ED90)

#define STK                                      ((STK_TypeDef *)STK_BASE)
#define NVIC                                     ((NVIC_TypeDef *)NVIC_BASE)
#define SCB                                      ((SCB_TypeDef *)SCB_BASE)
#define MPU                                      ((MPU_TypeDef *)MPU_BASE)


#define STK_CTRL_ENABLE                          (0x1U << 0U)
#define STK_CTRL_TICKINT                         (0x1U << 1U)
#define STK_CTRL_CLKSOURCE                       (0x1U << 2U)
#define STK_CTRL_COUNTFLAG                       (0x1U << 16U)

#define STK_LOAD_RELOAD                          (0xFFFFFFU << 0U)

#define STK_VAL_CURRENT                          (0xFFFFFFU << 0U)

#define STK_CALIB_TENMS                          (0xFFFFFFU << 0U)


#define SCB_CPUID_REVISION                       (0xFU << 0U)
#define SCB_CPUID_PARTNO                         (0xFFFU << 4U)
#define SCB_CPUID_CONSTANT                       (0xFU << 16U)
#define SCB_CPUID_VARIANT                        (0xFU << 20U)
#define SCB_CPUID_IMPLEMENTER                    (0xFFU << 24U)

#define SCB_ICSR_VECTACTIVE                      (0xFFU << 0U)
#define SCB_ICSR_RETTOBASE                       (0X1U << 11U)
#define SCB_ICSR_VECTPENDING                     (0x3FFU << 12U)
#define SCB_ICSR_ISRPENDING                      (0X1U << 22U)
#define SCB_ICSR_PENDSTCLR                       (0X1U << 25U)
#define SCB_ICSR_PENDSTSET                       (0X1U << 26U)
#define SCB_ICSR_PENDSVCLR                       (0X1U << 27U)
#define SCB_ICSR_PENDSVSET                       (0X1U << 28U)
#define SCB_ICSR_NMIPENDSET                      (0X1U << 31U)

#define SCB_VTOR_TABLEOFF                        (0x1FFFFFU << 9U)

#define SCB_AIRCR_VECTRESET                      (0X1U << 0U)
#define SCB_AIRCR_VECTCLRACTIVE                  (0X1U << 1U)
#define SCB_AIRCR_SYSRESETREQ                    (0X1U << 2U)
#define SCB_AIRCR_PRIGROUP                       (0X7U << 8U)
#define SCB_AIRCR_ENDIANESS                      (0X1U << 15U)
#define SCB_AIRCR_VECTKEY                        (0XFFU << 16U)

#define SCB_SCR_SLEEPONEXIT                      (0X1U << 1U)
#define SCB_SCR_SLEEPDEEP                        (0X1U << 2U)
#define SCB_SCR_SEVONPEND                        (0X1U << 4U)

#define SCB_CCR_NONBASETHRDEN                    (0X1U << 0U)
#define SCB_CCR_USERSETMPEND                     (0X1U << 1U)
#define SCB_CCR_UNALIGN_TRP                      (0X1U << 2U)
#define SCB_CCR_DIV_0_TRP                        (0X1U << 3U)
#define SCB_CCR_BFHFNIGN                         (0X1U << 8U)
#define SCB_CCR_STKALIGN                         (0X1U << 9U)

#define SCB_SHCRS_MEM_FAULT_ACT                  (0X1U << 0U)
#define SCB_SHCRS_BUS_FAULT_ACT                  (0X1U << 1U)
#define SCB_SHCRS_USG_FAULT_ACT                  (0X1U << 3U)
#define SCB_SHCRS_SV_CALL_ACT                    (0X1U << 7U)
#define SCB_SHCRS_MONITOR_ACT                    (0X1U << 8U)
#define SCB_SHCRS_PENDSV_ACT                     (0X1U << 10U)
#define SCB_SHCRS_SYSTICK_ACT                    (0X1U << 11U)
#define SCB_SHCRS_USG_FAULT_PENDED               (0X1U << 12U)
#define SCB_SHCRS_MEM_FAULT_PENDED               (0X1U << 13U)
#define SCB_SHCRS_BUS_FAULT_PENDED               (0X1U << 14U)
#define SCB_SHCRS_SV_CALL_PENDED                 (0X1U << 15U)
#define SCB_SHCRS_MEM_FAULT_ENA                  (0X1U << 16U)
#define SCB_SHCRS_BUS_FAULT_ENA                  (0X1U << 17U)
#define SCB_SHCRS_USG_FAULT_ENA                  (0X1U << 18U)

#define SCB_HFSR_VECTTBL                         (0x1U << 1U)
#define SCB_HFSR_FORCED                          (0x1U << 30U)
#define SCB_HFSR_DEBUG_VT                        (0x1U << 31U)


#define MPU_TYPER_SEPARATE

#define MPU_CR_ENABLE                            (0X1U << 0U)
#define MPU_CR_HFNMIENA                          (0X1U << 1U)
#define MPU_CR_PRIVDEFEN                         (0X1U << 2U)

#define MPU_RBAR_VALID                           (0X1U << 4U)

#define MPU_RASR_ENABLE                          (0X1U << 0U)
#define MPU_RASR_B                               (0X1U << 16U)
#define MPU_RASR_C                               (0X1U << 17U)
#define MPU_RASR_S                               (0X1U << 18U)
#define MPU_RASR_XN                              (0X1U << 28U)


#endif