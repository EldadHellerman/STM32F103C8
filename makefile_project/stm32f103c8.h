#ifndef STM32F103C8_H
#define STM32F103C8_H

#include <stdint.h>

typedef enum{ //IRQn_Type
  //Cortex-M3 Processor Exceptions Numbers:
  NonMaskableInt_IRQn         = -14,    // 2 - Non Maskable Interrupt
  HardFault_IRQn              = -13,    // 3 - Hard Fault Interrupt
  MemoryManagement_IRQn       = -12,    // 4 - Memory Management Interrup
  BusFault_IRQn               = -11,    // 5 - Bus Fault Interrupt
  UsageFault_IRQn             = -10,    // 6 - Usage Fault Interrupt
  SVCall_IRQn                 = -5,     // 11 - SV Call Interrupt
  DebugMonitor_IRQn           = -4,     // 12 - Debug Monitor Interrupt
  PendSV_IRQn                 = -2,     // 14 - Pend SV Interrupt
  SysTick_IRQn                = -1,     // 15 - System Tick Interrupt

  //STM32 specific Interrupt Numbers:
  WWDG_IRQn                   = 0,      //Window WatchDog Interrupt
  PVD_IRQn                    = 1,      //PVD through EXTI Line detection Interrupt
  TAMPER_IRQn                 = 2,      //Tamper Interrupt
  RTC_IRQn                    = 3,      //RTC global Interrupt
  FLASH_IRQn                  = 4,      //FLASH global Interrupt
  RCC_IRQn                    = 5,      //RCC global Interrupt
  EXTI0_IRQn                  = 6,      //EXTI Line0 Interrupt
  EXTI1_IRQn                  = 7,      //EXTI Line1 Interrupt
  EXTI2_IRQn                  = 8,      //EXTI Line2 Interrupt
  EXTI3_IRQn                  = 9,      //EXTI Line3 Interrupt
  EXTI4_IRQn                  = 10,     //EXTI Line4 Interrupt
  DMA1_Channel1_IRQn          = 11,     //DMA1 Channel 1 global Interrupt
  DMA1_Channel2_IRQn          = 12,     //DMA1 Channel 2 global Interrupt
  DMA1_Channel3_IRQn          = 13,     //DMA1 Channel 3 global Interrupt
  DMA1_Channel4_IRQn          = 14,     //DMA1 Channel 4 global Interrupt
  DMA1_Channel5_IRQn          = 15,     //DMA1 Channel 5 global Interrupt
  DMA1_Channel6_IRQn          = 16,     //DMA1 Channel 6 global Interrupt
  DMA1_Channel7_IRQn          = 17,     //DMA1 Channel 7 global Interrupt
  ADC1_2_IRQn                 = 18,     //ADC1 and ADC2 global Interrupt
  USB_HP_CAN1_TX_IRQn         = 19,     //USB Device High Priority or CAN1 TX Interrupts
  USB_LP_CAN1_RX0_IRQn        = 20,     //USB Device Low Priority or CAN1 RX0 Interrupts
  CAN1_RX1_IRQn               = 21,     //CAN1 RX1 Interrupt
  CAN1_SCE_IRQn               = 22,     //CAN1 SCE Interrupt
  EXTI9_5_IRQn                = 23,     //External Line[9:5] Interrupts
  TIM1_BRK_IRQn               = 24,     //TIM1 Break Interrupt
  TIM1_UP_IRQn                = 25,     //TIM1 Update Interrupt
  TIM1_TRG_COM_IRQn           = 26,     //TIM1 Trigger and Commutation Interrupt
  TIM1_CC_IRQn                = 27,     //TIM1 Capture Compare Interrupt
  TIM2_IRQn                   = 28,     //TIM2 global Interrupt
  TIM3_IRQn                   = 29,     //TIM3 global Interrupt
  TIM4_IRQn                   = 30,     //TIM4 global Interrupt
  I2C1_EV_IRQn                = 31,     //I2C1 Event Interrupt
  I2C1_ER_IRQn                = 32,     //I2C1 Error Interrupt
  I2C2_EV_IRQn                = 33,     //I2C2 Event Interrupt
  I2C2_ER_IRQn                = 34,     //I2C2 Error Interrupt
  SPI1_IRQn                   = 35,     //SPI1 global Interrupt
  SPI2_IRQn                   = 36,     //SPI2 global Interrupt
  USART1_IRQn                 = 37,     //USART1 global Interrupt
  USART2_IRQn                 = 38,     //USART2 global Interrupt
  USART3_IRQn                 = 39,     //USART3 global Interrupt
  EXTI15_10_IRQn              = 40,     //External Line[15:10] Interrupts
  RTC_Alarm_IRQn              = 41,     //RTC Alarm through EXTI Line Interrupt
  USBWakeUp_IRQn              = 42,     //USB Device WakeUp from suspend through EXTI Line Interrupt
} IRQn_Type;

typedef struct{ //FLASH_TypeDef
  uint32_t ACR;
  uint32_t KEYR;
  uint32_t OPTKEYR;
  uint32_t SR;
  uint32_t CR;
  uint32_t AR;
  uint32_t RESERVED;
  uint32_t OBR;
  uint32_t WRPR;
} FLASH_TypeDef;

typedef struct{ //OB_TypeDef
  uint16_t RDP;
  uint16_t USER;
  uint16_t Data0;
  uint16_t Data1;
  uint16_t WRP0;
  uint16_t WRP1;
  uint16_t WRP2;
  uint16_t WRP3;
} OB_TypeDef;

typedef struct{ //CRC_TypeDef
  uint32_t DR;
  uint8_t  IDR;
  uint8_t       RESERVED0;
  uint16_t      RESERVED1;
  uint32_t CR;
} CRC_TypeDef;

typedef struct{ //PWR_TypeDef
  uint32_t CR;
  uint32_t CSR;
} PWR_TypeDef;

typedef struct{ //BKP_TypeDef
  uint32_t RESERVED0;
  uint32_t DR1;
  uint32_t DR2;
  uint32_t DR3;
  uint32_t DR4;
  uint32_t DR5;
  uint32_t DR6;
  uint32_t DR7;
  uint32_t DR8;
  uint32_t DR9;
  uint32_t DR10;
  uint32_t RTCCR;
  uint32_t CR;
  uint32_t CSR;
} BKP_TypeDef;

typedef struct{ //RCC_TypeDef
  uint32_t CR;
  uint32_t CFGR;
  uint32_t CIR;
  uint32_t APB2RSTR;
  uint32_t APB1RSTR;
  uint32_t AHBENR;
  uint32_t APB2ENR;
  uint32_t APB1ENR;
  uint32_t BDCR;
  uint32_t CSR;
} RCC_TypeDef;

typedef struct{ //GPIO_TypeDef
  uint32_t CRL;
  uint32_t CRH;
  uint32_t IDR;
  uint32_t ODR;
  uint32_t BSRR;
  uint32_t BRR;
  uint32_t LCKR;
} GPIO_TypeDef;

typedef struct{ //AFIO_TypeDef
  uint32_t EVCR;
  uint32_t MAPR;
  uint32_t EXTICR[4];
  uint32_t RESERVED0;
  uint32_t MAPR2;
} AFIO_TypeDef;

typedef struct{ //EXTI_TypeDef
  uint32_t IMR;
  uint32_t EMR;
  uint32_t RTSR;
  uint32_t FTSR;
  uint32_t SWIER;
  uint32_t PR;
} EXTI_TypeDef;

typedef struct{ //ADC_TypeDef
  uint32_t SR;
  uint32_t CR1;
  uint32_t CR2;
  uint32_t SMPR1;
  uint32_t SMPR2;
  uint32_t JOFR1;
  uint32_t JOFR2;
  uint32_t JOFR3;
  uint32_t JOFR4;
  uint32_t HTR;
  uint32_t LTR;
  uint32_t SQR1;
  uint32_t SQR2;
  uint32_t SQR3;
  uint32_t JSQR;
  uint32_t JDR1;
  uint32_t JDR2;
  uint32_t JDR3;
  uint32_t JDR4;
  uint32_t DR;
} ADC_TypeDef;

typedef struct{ //DMA_Channel_TypeDef
  uint32_t CCR;
  uint32_t CNDTR;
  uint32_t CPAR;
  uint32_t CMAR;
  uint32_t RESERVED0;
} DMA_Channel_TypeDef;

typedef struct{ //DMA_TypeDef
  uint32_t ISR;
  uint32_t IFCR;
  DMA_Channel_TypeDef channel[7];
} DMA_TypeDef;

typedef struct{ //TIM_TypeDef
  uint32_t CR1;
  uint32_t CR2;
  uint32_t SMCR;
  uint32_t DIER;
  uint32_t SR;
  uint32_t EGR;
  uint32_t CCMR1;
  uint32_t CCMR2;
  uint32_t CCER;
  uint32_t CNT;
  uint32_t PSC;
  uint32_t ARR;
  uint32_t RCR;
  uint32_t CCR1;
  uint32_t CCR2;
  uint32_t CCR3;
  uint32_t CCR4;
  uint32_t BDTR;
  uint32_t DCR;
  uint32_t DMAR;
}TIM_TypeDef;

typedef struct{ //RTC_TypeDef
  uint32_t CRH;
  uint32_t CRL;
  uint32_t PRLH;
  uint32_t PRLL;
  uint32_t DIVH;
  uint32_t DIVL;
  uint32_t CNTH;
  uint32_t CNTL;
  uint32_t ALRH;
  uint32_t ALRL;
} RTC_TypeDef;

typedef struct{ //IWDG_TypeDef
  uint32_t KR;
  uint32_t PR;
  uint32_t RLR;
  uint32_t SR;
} IWDG_TypeDef;

typedef struct{ //WWDG_TypeDef
  uint32_t CR;
  uint32_t CFR;
  uint32_t SR;
} WWDG_TypeDef;

typedef struct{ //USB_TypeDef
  uint16_t EP0R;
  uint16_t RESERVED0;
  uint16_t EP1R;
  uint16_t RESERVED1;
  uint16_t EP2R;
  uint16_t RESERVED2;
  uint16_t EP3R;
  uint16_t RESERVED3;
  uint16_t EP4R;
  uint16_t RESERVED4;
  uint16_t EP5R;
  uint16_t RESERVED5;
  uint16_t EP6R;
  uint16_t RESERVED6;
  uint16_t EP7R;
  uint16_t RESERVED7[17];
  uint16_t CNTR;
  uint16_t RESERVED8;
  uint16_t ISTR;
  uint16_t RESERVED9;
  uint16_t FNR;
  uint16_t RESERVEDA;
  uint16_t DADDR;
  uint16_t RESERVEDB;
  uint16_t BTABLE;
  uint16_t RESERVEDC;
} USB_TypeDef;

typedef union{ //USBSRAM_TyepDef
  uint16_t buffer16[256];
  uint8_t buffer[512];
}USBSRAM_TyepDef;

typedef struct{ //CAN_TxMailBox_TypeDef
  uint32_t TIR;
  uint32_t TDTR;
  uint32_t TDLR;
  uint32_t TDHR;
} CAN_TxMailBox_TypeDef;

typedef struct{ //CAN_FIFOMailBox_TypeDef
  uint32_t RIR;
  uint32_t RDTR;
  uint32_t RDLR;
  uint32_t RDHR;
} CAN_FIFOMailBox_TypeDef;

typedef struct{ //CAN_FilterRegister_TypeDef
  uint32_t FR1;
  uint32_t FR2;
} CAN_FilterRegister_TypeDef;

typedef struct{ //CAN_TypeDef
  uint32_t MCR;
  uint32_t MSR;
  uint32_t TSR;
  uint32_t RF0R;
  uint32_t RF1R;
  uint32_t IER;
  uint32_t ESR;
  uint32_t BTR;
  uint32_t  RESERVED0[88];
  CAN_TxMailBox_TypeDef sTxMailBox[3];
  CAN_FIFOMailBox_TypeDef sFIFOMailBox[2];
  uint32_t  RESERVED1[12];
  uint32_t FMR;
  uint32_t FM1R;
  uint32_t  RESERVED2;
  uint32_t FS1R;
  uint32_t  RESERVED3;
  uint32_t FFA1R;
  uint32_t  RESERVED4;
  uint32_t FA1R;
  uint32_t  RESERVED5[8];
  CAN_FilterRegister_TypeDef sFilterRegister[14];
} CAN_TypeDef;

typedef struct{ //SPI_TypeDef
  uint32_t CR1;
  uint32_t CR2;
  uint32_t SR;
  uint32_t DR;
  uint32_t CRCPR;
  uint32_t RXCRCR;
  uint32_t TXCRCR;
} SPI_TypeDef;

typedef struct{ //I2C_TypeDef
  uint32_t CR1;
  uint32_t CR2;
  uint32_t OAR1;
  uint32_t OAR2;
  uint32_t DR;
  uint32_t SR1;
  uint32_t SR2;
  uint32_t CCR;
  uint32_t TRISE;
} I2C_TypeDef;

typedef struct{ //USART_TypeDef
  uint32_t SR;
  uint32_t DR;
  uint32_t BRR;
  uint32_t CR1;
  uint32_t CR2;
  uint32_t CR3;
  uint32_t GTPR;
} USART_TypeDef;

typedef struct{ //UID_TypeDef
  uint32_t UID[3];
}UID_TypeDef;

typedef struct{ //DBGMCU_TypeDef
  uint32_t IDCODE;
  uint32_t CR;
}DBGMCU_TypeDef;


#define FLASH_BASE            0x08000000U
#define SRAM_BASE             0x20000000U
#define PERIPH_BASE           0x40000000U
#define APB1_PERIPH_BASE      (PERIPH_BASE)
#define APB2_PERIPH_BASE      (PERIPH_BASE + 0x00010000U)
#define AHB_PERIPH_BASE       (PERIPH_BASE + 0x00020000U)

#define SRAM_BB_BASE          0x22000000U
#define PERIPH_BB_BASE        0x42000000U
#warning "unfinished code in line 407"
#define BIT_BAND_SRAM(WORD_ADDRESS, BIT)       (*(uint32_t *)(SRAM_BB_BASE + ))
#define BIT_BAND_PERIPH(WORD_ADDRESS, BIT)     (*(uint32_t *)(PERIPH_BB_BASE + ))

#define TIM2_BASE             (APB1_PERIPH_BASE + 0x00000000U)
#define TIM3_BASE             (APB1_PERIPH_BASE + 0x00000400U)
#define TIM4_BASE             (APB1_PERIPH_BASE + 0x00000800U)
#define RTC_BASE              (APB1_PERIPH_BASE + 0x00002800U)
#define WWDG_BASE             (APB1_PERIPH_BASE + 0x00002C00U)
#define IWDG_BASE             (APB1_PERIPH_BASE + 0x00003000U)
#define SPI2_BASE             (APB1_PERIPH_BASE + 0x00003800U)
#define USART2_BASE           (APB1_PERIPH_BASE + 0x00004400U)
#define USART3_BASE           (APB1_PERIPH_BASE + 0x00004800U)
#define I2C1_BASE             (APB1_PERIPH_BASE + 0x00005400U)
#define I2C2_BASE             (APB1_PERIPH_BASE + 0x00005800U)
#define USB_BASE              (APB1_PERIPH_BASE + 0x00005C00U)
#define USB_SRAM_BASE         (APB1_PERIPH_BASE + 0x00006000U)
#define CAN1_BASE             (APB1_PERIPH_BASE + 0x00006400U)
#define BKP_BASE              (APB1_PERIPH_BASE + 0x00006C00U)
#define PWR_BASE              (APB1_PERIPH_BASE + 0x00007000U)
#define AFIO_BASE             (APB2_PERIPH_BASE + 0x00000000U)
#define EXTI_BASE             (APB2_PERIPH_BASE + 0x00000400U)
#define GPIOA_BASE            (APB2_PERIPH_BASE + 0x00000800U)
#define GPIOB_BASE            (APB2_PERIPH_BASE + 0x00000C00U)
#define GPIOC_BASE            (APB2_PERIPH_BASE + 0x00001000U)
#define GPIOD_BASE            (APB2_PERIPH_BASE + 0x00001400U)
#define GPIOE_BASE            (APB2_PERIPH_BASE + 0x00001800U)
#define ADC1_BASE             (APB2_PERIPH_BASE + 0x00002400U)
#define ADC2_BASE             (APB2_PERIPH_BASE + 0x00002800U)
#define TIM1_BASE             (APB2_PERIPH_BASE + 0x00002C00U)
#define SPI1_BASE             (APB2_PERIPH_BASE + 0x00003000U)
#define USART1_BASE           (APB2_PERIPH_BASE + 0x00003800U)
#define DMA1_BASE             (AHB_PERIPH_BASE + 0x00000000U)
#define RCC_BASE              (AHB_PERIPH_BASE + 0x00001000U)
#define FLASH_R_BASE          (AHB_PERIPH_BASE + 0x00002000U)
#define CRC_BASE              (AHB_PERIPH_BASE + 0x00003000U)
#define FLASHSIZE_BASE        0x1FFFF7E0U
#define UID_BASE              0x1FFFF7E8U
#define DBGMCU_BASE           0xE0042000U
#define OB_BASE               0x1FFFF800U


#define TIM2                ((TIM_TypeDef *)TIM2_BASE)
#define TIM3                ((TIM_TypeDef *)TIM3_BASE)
#define TIM4                ((TIM_TypeDef *)TIM4_BASE)
#define RTC                 ((RTC_TypeDef *)RTC_BASE)
#define WWDG                ((WWDG_TypeDef *)WWDG_BASE)
#define IWDG                ((IWDG_TypeDef *)IWDG_BASE)
#define SPI2                ((SPI_TypeDef *)SPI2_BASE)
#define USART2              ((USART_TypeDef *)USART2_BASE)
#define USART3              ((USART_TypeDef *)USART3_BASE)
#define I2C1                ((I2C_TypeDef *)I2C1_BASE)
#define I2C2                ((I2C_TypeDef *)I2C2_BASE)
#define USB                 ((USB_TypeDef *)USB_BASE)
#define USB_SRAM            (*(USBSRAM_TyepDef *)USB_SRAM_BASE)
#define CAN1                ((CAN_TypeDef *)CAN1_BASE)
#define BKP                 ((BKP_TypeDef *)BKP_BASE)
#define PWR                 ((PWR_TypeDef *)PWR_BASE)
#define AFIO                ((AFIO_TypeDef *)AFIO_BASE)
#define EXTI                ((EXTI_TypeDef *)EXTI_BASE)
#define GPIOA               ((GPIO_TypeDef *)GPIOA_BASE)
#define GPIOB               ((GPIO_TypeDef *)GPIOB_BASE)
#define GPIOC               ((GPIO_TypeDef *)GPIOC_BASE)
#define GPIOD               ((GPIO_TypeDef *)GPIOD_BASE)
#define GPIOE               ((GPIO_TypeDef *)GPIOE_BASE)
#define ADC1                ((ADC_TypeDef *)ADC1_BASE)
#define ADC2                ((ADC_TypeDef *)ADC2_BASE)
#define TIM1                ((TIM_TypeDef *)TIM1_BASE)
#define SPI1                ((SPI_TypeDef *)SPI1_BASE)
#define USART1              ((USART_TypeDef *)USART1_BASE)
#define DMA                 ((DMA_TypeDef *)DMA1_BASE)
#define RCC                 ((RCC_TypeDef *)RCC_BASE)
#define FLASH               ((FLASH_TypeDef *)FLASH_R_BASE)
#define CRC                 ((CRC_TypeDef *)CRC_BASE)
#define FLASH_SIZE          (*(uint16_t *)(FLASHSIZE_BASE))
#define UID                 ((UID_TypeDef *)UID_BASE)
#define DBGMCU              ((DBGMCU_TypeDef *)DBGMCU_BASE)
#define OB                  ((OB_TypeDef *)OB_BASE)


#define CRC_CR_RESET                        0x1U                               /* RESET bit*/

#define PWR_CR_LPDS                         (0x1U << 0U)                       /* Low-Power Deepsleep*/
#define PWR_CR_PDDS                         (0x1U << 1U)                       /* Power Down Deepsleep*/
#define PWR_CR_CWUF                         (0x1U << 2U)                       /* Clear Wakeup Flag*/
#define PWR_CR_CSBF                         (0x1U << 3U)                       /* Clear Standby Flag*/
#define PWR_CR_PVDE                         (0x1U << 4U)                       /* Power Voltage Detector Enable*/
#define PWR_CR_PLS                          (0x7U << 5U)                       /* PLS[2:0] bits (PVD Level Selection)*/
#define PWR_CR_PLS_0                        (0x1U << 5U)
#define PWR_CR_PLS_1                        (0x2U << 5U)
#define PWR_CR_PLS_2                        (0x4U << 5U)
#define PWR_CR_PLS_LEV0                      0x00000000U                       /* PVD level 2.2V */
#define PWR_CR_PLS_LEV1                      0x00000020U                       /* PVD level 2.3V */
#define PWR_CR_PLS_LEV2                      0x00000040U                       /* PVD level 2.4V */
#define PWR_CR_PLS_LEV3                      0x00000060U                       /* PVD level 2.5V */
#define PWR_CR_PLS_LEV4                      0x00000080U                       /* PVD level 2.6V */
#define PWR_CR_PLS_LEV5                      0x000000A0U                       /* PVD level 2.7V */
#define PWR_CR_PLS_LEV6                      0x000000C0U                       /* PVD level 2.8V */
#define PWR_CR_PLS_LEV7                      0x000000E0U                       /* PVD level 2.9V */
#define PWR_CR_DBP                          (0x1U << 8U)                       /* Disable Backup Domain write protection*/

#define PWR_CSR_WUF                         (0x1U << 0U)                       /* Wakeup Flag*/
#define PWR_CSR_SBF                         (0x1U << 1U)                       /* Standby Flag*/
#define PWR_CSR_PVDO                        (0x1U << 2U)                       /* PVD Output*/
#define PWR_CSR_EWUP                        (0x1U << 8U)                       /* Enable WKUP pin*/


#define BKP_RTCCR_CAL                       (0x7FU << 0U)                      /* Calibration value*/
#define BKP_RTCCR_CCO                       (0x1U << 0U)                       /* Calibration Clock Output*/
#define BKP_RTCCR_ASOE                      (0x1U << 0U)                       /* Alarm or Second Output Enable*/
#define BKP_RTCCR_ASOS                      (0x1U << 0U)                       /* Alarm or Second Output Selection*/

#define BKP_CR_TPE                          (0x1U << 0U)                       /* TAMPER pin enable*/
#define BKP_CR_TPAL                         (0x1U << 1U)                       /* TAMPER pin active level*/

#define BKP_CSR_CTE                         (0x1U << 0U)                       /* Clear Tamper event*/
#define BKP_CSR_CTI                         (0x1U << 1U)                       /* Clear Tamper Interrupt*/
#define BKP_CSR_TPIE                        (0x1U << 2U)                       /* TAMPER Pin interrupt enable*/
#define BKP_CSR_TEF                         (0x1U << 8U)                       /* Tamper Event Flag*/
#define BKP_CSR_TIF                         (0x1U << 9U)                       /* Tamper Interrupt Flag*/


#define RCC_CR_HSION                         (0x1U << 0U)                      /* Internal High Speed clock enable*/
#define RCC_CR_HSIRDY                        (0x1U << 1U)                      /* Internal High Speed clock ready flag*/
#define RCC_CR_HSITRIM                       (0x1FU << 3U)                     /* Internal High Speed clock trimming*/
#define RCC_CR_HSICAL                        (0xFFU << 8U)                     /* Internal High Speed clock Calibration*/
#define RCC_CR_HSEON                         (0x1U << 16U)                     /* External High Speed clock enable*/
#define RCC_CR_HSERDY                        (0x1U << 17U)                     /* External High Speed clock ready flag*/
#define RCC_CR_HSEBYP                        (0x1U << 18U)                     /* External High Speed clock Bypass*/
#define RCC_CR_CSSON                         (0x1U << 19U)                     /* Clock Security System enable*/
#define RCC_CR_PLLON                         (0x1U << 24U)                     /* PLL enable*/
#define RCC_CR_PLLRDY                        (0x1U << 25U)                     /* PLL clock ready flag*/

#define RCC_CFGR_SW                          (0x3U << 0U)                      /* SW[1:0] bits (System clock Switch)*/
#define RCC_CFGR_SW_0                        (0x1U << 0U)
#define RCC_CFGR_SW_1                        (0x2U << 0U)
#define RCC_CFGR_SW_HSI                      0x00000000U                       /* HSI selected as system clock*/
#define RCC_CFGR_SW_HSE                      0x00000001U                       /* HSE selected as system clock*/
#define RCC_CFGR_SW_PLL                      0x00000002U                       /* PLL selected as system clock*/
#define RCC_CFGR_SWS                         (0x3U << 2U)                      /* SWS[1:0] bits (System Clock Switch Status)*/
#define RCC_CFGR_SWS_0                       (0x1U << 2U)
#define RCC_CFGR_SWS_1                       (0x2U << 2U)
#define RCC_CFGR_SWS_HSI                     0x00000000U                       /* HSI oscillator used as system clock*/
#define RCC_CFGR_SWS_HSE                     0x00000004U                       /* HSE oscillator used as system clock*/
#define RCC_CFGR_SWS_PLL                     0x00000008U                       /* PLL used as system clock*/
#define RCC_CFGR_HPRE                        (0xFU << 4U)                      /* HPRE[3:0] bits (AHB prescaler)*/
#define RCC_CFGR_HPRE_0                      (0x1U << 4U)
#define RCC_CFGR_HPRE_1                      (0x2U << 4U)
#define RCC_CFGR_HPRE_2                      (0x4U << 4U)
#define RCC_CFGR_HPRE_3                      (0x8U << 4U)
#define RCC_CFGR_HPRE_DIV1                   0x00000000U                       /* SYSCLK not divided*/
#define RCC_CFGR_HPRE_DIV2                   0x00000080U                       /* SYSCLK divided by 2*/
#define RCC_CFGR_HPRE_DIV4                   0x00000090U                       /* SYSCLK divided by 4*/
#define RCC_CFGR_HPRE_DIV8                   0x000000A0U                       /* SYSCLK divided by 8*/
#define RCC_CFGR_HPRE_DIV16                  0x000000B0U                       /* SYSCLK divided by 16*/
#define RCC_CFGR_HPRE_DIV64                  0x000000C0U                       /* SYSCLK divided by 64*/
#define RCC_CFGR_HPRE_DIV128                 0x000000D0U                       /* SYSCLK divided by 128*/
#define RCC_CFGR_HPRE_DIV256                 0x000000E0U                       /* SYSCLK divided by 256*/
#define RCC_CFGR_HPRE_DIV512                 0x000000F0U                       /* SYSCLK divided by 512*/
#define RCC_CFGR_PPRE1                       (0x7U << 8U)                      /* PRE1[2:0] bits (APB1 prescaler)*/
#define RCC_CFGR_PPRE1_0                     (0x1U << 8U)
#define RCC_CFGR_PPRE1_1                     (0x2U << 8U)
#define RCC_CFGR_PPRE1_2                     (0x4U << 8U)
#define RCC_CFGR_PPRE1_DIV1                  0x00000000U                       /* HCLK not divided*/
#define RCC_CFGR_PPRE1_DIV2                  0x00000400U                       /* HCLK divided by 2*/
#define RCC_CFGR_PPRE1_DIV4                  0x00000500U                       /* HCLK divided by 4*/
#define RCC_CFGR_PPRE1_DIV8                  0x00000600U                       /* HCLK divided by 8*/
#define RCC_CFGR_PPRE1_DIV16                 0x00000700U                       /* HCLK divided by 16*/
#define RCC_CFGR_PPRE2                       (0x7U << 11U)                     /* PRE2[2:0] bits (APB2 prescaler)*/
#define RCC_CFGR_PPRE2_0                     (0x1U << 11U)
#define RCC_CFGR_PPRE2_1                     (0x2U << 11U)
#define RCC_CFGR_PPRE2_2                     (0x4U << 11U)
#define RCC_CFGR_PPRE2_DIV1                  0x00000000U                       /* HCLK not divided*/
#define RCC_CFGR_PPRE2_DIV2                  0x00002000U                       /* HCLK divided by 2*/
#define RCC_CFGR_PPRE2_DIV4                  0x00002800U                       /* HCLK divided by 4*/
#define RCC_CFGR_PPRE2_DIV8                  0x00003000U                       /* HCLK divided by 8*/
#define RCC_CFGR_PPRE2_DIV16                 0x00003800U                       /* HCLK divided by 16*/
#define RCC_CFGR_ADCPRE                      (0x3U << 14U)                     /* ADCPRE[1:0] bits (ADC prescaler)*/
#define RCC_CFGR_ADCPRE_0                    (0x1U << 14U)
#define RCC_CFGR_ADCPRE_1                    (0x2U << 14U)
#define RCC_CFGR_ADCPRE_DIV2                 0x00000000U                       /* PCLK2 divided by 2*/
#define RCC_CFGR_ADCPRE_DIV4                 0x00004000U                       /* PCLK2 divided by 4*/
#define RCC_CFGR_ADCPRE_DIV6                 0x00008000U                       /* PCLK2 divided by 6*/
#define RCC_CFGR_ADCPRE_DIV8                 0x0000C000U                       /* PCLK2 divided by 8*/
#define RCC_CFGR_PLLSRC                      (0x1U << 16U)                     /* PLL entry clock source */
#define RCC_CFGR_PLLXTPRE                    (0x1U << 17U)                     /* HSE divider for PLL entry*/
#define RCC_CFGR_PLLXTPRE_HSE                0x00000000U                       /* HSE clock not divided for PLL entry*/
#define RCC_CFGR_PLLXTPRE_HSE_DIV2           0x00020000U                       /* HSE clock divided by 2 for PLL entry*/
#define RCC_CFGR_PLLMULL                     (0xFU << 18U)                     /* PLLMUL[3:0] bits (PLL multiplication factor)*/
#define RCC_CFGR_PLLMULL_0                   (0x1U << 18U)
#define RCC_CFGR_PLLMULL_1                   (0x2U << 18U)
#define RCC_CFGR_PLLMULL_2                   (0x4U << 18U)
#define RCC_CFGR_PLLMULL_3                   (0x8U << 18U)
#define RCC_CFGR_PLLMULL2                    (0x0U << 18U)                     /* PLL input clock*2 */
#define RCC_CFGR_PLLMULL3                    (0x1U << 18U)                     /* PLL input clock*3 */
#define RCC_CFGR_PLLMULL4                    (0x2U << 18U)                     /* PLL input clock*4 */
#define RCC_CFGR_PLLMULL5                    (0x3U << 18U)                     /* PLL input clock*5 */
#define RCC_CFGR_PLLMULL6                    (0x4U << 18U)                     /* PLL input clock*6 */
#define RCC_CFGR_PLLMULL7                    (0x5U << 18U)                     /* PLL input clock*7 */
#define RCC_CFGR_PLLMULL8                    (0x6U << 18U)                     /* PLL input clock*8 */
#define RCC_CFGR_PLLMULL9                    (0x7U << 18U)                     /* PLL input clock*9 */
#define RCC_CFGR_PLLMULL10                   (0x8U << 18U)                     /* PLL input clock10 */
#define RCC_CFGR_PLLMULL11                   (0x9U << 18U)                     /* PLL input clock*11 */
#define RCC_CFGR_PLLMULL12                   (0xAU << 18U)                     /* PLL input clock*12 */
#define RCC_CFGR_PLLMULL13                   (0xBU << 18U)                     /* PLL input clock*13 */
#define RCC_CFGR_PLLMULL14                   (0xCU << 18U)                     /* PLL input clock*14 */
#define RCC_CFGR_PLLMULL15                   (0xDU << 18U)                     /* PLL input clock*15 */
#define RCC_CFGR_PLLMULL16                   (0xEU << 18U)                     /* PLL input clock*16 */
#define RCC_CFGR_USBPRE                      (0x1U << 22U)                     /* USB Device prescaler*/
#define RCC_CFGR_MCO                         (0x7U << 24U)                     /* MCO[2:0] bits (Microcontroller Clock Output)*/
#define RCC_CFGR_MCO_0                       (0x1U << 24U)
#define RCC_CFGR_MCO_1                       (0x2U << 24U)
#define RCC_CFGR_MCO_2                       (0x4U << 24U)
#define RCC_CFGR_MCO_NOCLOCK                 0x00000000U                       /* No clock*/
#define RCC_CFGR_MCO_SYSCLK                  0x04000000U                       /* System clock selected as MCO source*/
#define RCC_CFGR_MCO_HSI                     0x05000000U                       /* HSI clock selected as MCO source*/
#define RCC_CFGR_MCO_HSE                     0x06000000U                       /* HSE clock selected as MCO source*/
#define RCC_CFGR_MCO_PLLCLK_DIV2             0x07000000U                       /* PLL clock divided by 2 selected as MCO source*/

#define RCC_CIR_LSIRDYF                      (0x1U << 0U)                      /* LSI Ready Interrupt flag*/
#define RCC_CIR_LSERDYF                      (0x1U << 1U)                      /* LSE Ready Interrupt flag*/
#define RCC_CIR_HSIRDYF                      (0x1U << 2U)                      /* HSI Ready Interrupt flag*/
#define RCC_CIR_HSERDYF                      (0x1U << 3U)                      /* HSE Ready Interrupt flag*/
#define RCC_CIR_PLLRDYF                      (0x1U << 4U)                      /* PLL Ready Interrupt flag*/
#define RCC_CIR_CSSF                         (0x1U << 7U)                      /* Clock Security System Interrupt flag*/
#define RCC_CIR_LSIRDYIE                     (0x1U << 8U)                      /* LSI Ready Interrupt Enable*/
#define RCC_CIR_LSERDYIE                     (0x1U << 9U)                      /* LSE Ready Interrupt Enable*/
#define RCC_CIR_HSIRDYIE                     (0x1U << 10U)                     /* HSI Ready Interrupt Enable*/
#define RCC_CIR_HSERDYIE                     (0x1U << 11U)                     /* HSE Ready Interrupt Enable*/
#define RCC_CIR_PLLRDYIE                     (0x1U << 12U)                     /* PLL Ready Interrupt Enable*/
#define RCC_CIR_LSIRDYC                      (0x1U << 16U)                     /* LSI Ready Interrupt Clear*/
#define RCC_CIR_LSERDYC                      (0x1U << 17U)                     /* LSE Ready Interrupt Clear*/
#define RCC_CIR_HSIRDYC                      (0x1U << 18U)                     /* HSI Ready Interrupt Clear*/
#define RCC_CIR_HSERDYC                      (0x1U << 19U)                     /* HSE Ready Interrupt Clear*/
#define RCC_CIR_PLLRDYC                      (0x1U << 20U)                     /* PLL Ready Interrupt Clear*/
#define RCC_CIR_CSSC                         (0x1U << 23U)                     /* Clock Security System Interrupt Clear*/

#define RCC_APB2RSTR_AFIORST                 (0x1U << 0U)                      /* Alternate Function I/O reset*/
#define RCC_APB2RSTR_IOPARST                 (0x1U << 2U)                      /* I/O port A reset*/
#define RCC_APB2RSTR_IOPBRST                 (0x1U << 3U)                      /* I/O port B reset*/
#define RCC_APB2RSTR_IOPCRST                 (0x1U << 4U)                      /* I/O port C reset*/
#define RCC_APB2RSTR_IOPDRST                 (0x1U << 5U)                      /* I/O port D reset*/
#define RCC_APB2RSTR_IOPERST                 (0x1U << 6U)                      /* I/O port E reset*/
#define RCC_APB2RSTR_ADC1RST                 (0x1U << 9U)                      /* ADC 1 interface reset*/
#define RCC_APB2RSTR_ADC2RST                 (0x1U << 10U)                     /* ADC 2 interface reset*/
#define RCC_APB2RSTR_TIM1RST                 (0x1U << 11U)                     /* TIM1 Timer reset*/
#define RCC_APB2RSTR_SPI1RST                 (0x1U << 12U)                     /* SPI 1 reset*/
#define RCC_APB2RSTR_USART1RST               (0x1U << 14U)                     /* USART1 reset*/

#define RCC_APB1RSTR_TIM2RST                 (0x1U << 0U)                      /* Timer 2 reset*/
#define RCC_APB1RSTR_TIM3RST                 (0x1U << 1U)                      /* Timer 3 reset*/
#define RCC_APB1RSTR_TIM4RST                 (0x1U << 2U)                      /* Timer 4 reset*/
#define RCC_APB1RSTR_WWDGRST                 (0x1U << 11U)                     /* Window Watchdog reset*/
#define RCC_APB1RSTR_SPI2RST                 (0x1U << 14U)                     /* SPI 2 reset*/
#define RCC_APB1RSTR_USART2RST               (0x1U << 17U)                     /* USART 2 reset*/
#define RCC_APB1RSTR_USART3RST               (0x1U << 18U)                     /* USART 3 reset*/
#define RCC_APB1RSTR_I2C1RST                 (0x1U << 21U)                     /* I2C 1 reset*/
#define RCC_APB1RSTR_I2C2RST                 (0x1U << 22U)                     /* I2C 2 reset*/
#define RCC_APB1RSTR_USBRST                  (0x1U << 23U)                     /* USB Device reset*/
#define RCC_APB1RSTR_CAN1RST                 (0x1U << 25U)                     /* CAN1 reset*/
#define RCC_APB1RSTR_BKPRST                  (0x1U << 27U)                     /* Backup interface reset*/
#define RCC_APB1RSTR_PWRRST                  (0x1U << 28U)                     /* Power interface reset*/

#define RCC_AHBENR_DMA1EN                    (0x1U << 0U)                      /* DMA1 clock enable*/
#define RCC_AHBENR_SRAMEN                    (0x1U << 2U)                      /* SRAM interface clock enable*/
#define RCC_AHBENR_FLITFEN                   (0x1U << 4U)                      /* FLITF clock enable*/
#define RCC_AHBENR_CRCEN                     (0x1U << 6U)                      /* CRC clock enable*/

#define RCC_APB2ENR_AFIOEN                   (0x1U << 0U)                      /* Alternate Function I/O clock enable*/
#define RCC_APB2ENR_IOPAEN                   (0x1U << 2U)                      /* I/O port A clock enable*/
#define RCC_APB2ENR_IOPBEN                   (0x1U << 3U)                      /* I/O port B clock enable*/
#define RCC_APB2ENR_IOPCEN                   (0x1U << 4U)                      /* I/O port C clock enable*/
#define RCC_APB2ENR_IOPDEN                   (0x1U << 5U)                      /* I/O port D clock enable*/
#define RCC_APB2ENR_IOPEEN                   (0x1U << 6U)                      /* I/O port E clock enable*/
#define RCC_APB2ENR_ADC1EN                   (0x1U << 9U)                      /* ADC1 interface clock enable*/
#define RCC_APB2ENR_ADC2EN                   (0x1U << 10U)                     /* ADC2 interface clock enable*/
#define RCC_APB2ENR_TIM1EN                   (0x1U << 11U)                     /* TIM1 Timer clock enable*/
#define RCC_APB2ENR_SPI1EN                   (0x1U << 12U)                     /* SPI1 clock enable*/
#define RCC_APB2ENR_USART1EN                 (0x1U << 14U)                     /* USART1 clock enable*/

#define RCC_APB1ENR_TIM2EN                   (0x1U << 0U)                      /* TIM2 clock enable*/
#define RCC_APB1ENR_TIM3EN                   (0x1U << 1U)                      /* TIM3 clock enable*/
#define RCC_APB1ENR_TIM4EN                   (0x1U << 2U)                      /* TIM4 clock enable*/
#define RCC_APB1ENR_WWDGEN                   (0x1U << 11U)                     /* Window Watchdog clock enable*/
#define RCC_APB1ENR_SPI2EN                   (0x1U << 14U)                     /* SPI 2 clock enable*/
#define RCC_APB1ENR_USART2EN                 (0x1U << 17U)                     /* USART2 clock enable*/
#define RCC_APB1ENR_USART3EN                 (0x1U << 18U)                     /* USART 3 clock enable*/
#define RCC_APB1ENR_I2C1EN                   (0x1U << 21U)                     /* I2C1 clock enable*/
#define RCC_APB1ENR_I2C2EN                   (0x1U << 22U)                     /* I2C 2 clock enable*/
#define RCC_APB1ENR_USBEN                    (0x1U << 23U)                     /* USB Device clock enable*/
#define RCC_APB1ENR_CAN1EN                   (0x1U << 25U)                     /* CAN1 clock enable*/
#define RCC_APB1ENR_BKPEN                    (0x1U << 27U)                     /* Backup interface clock enable*/
#define RCC_APB1ENR_PWREN                    (0x1U << 28U)                     /* Power interface clock enable*/

#define RCC_BDCR_LSEON                       (0x1U << 0U)                      /* External Low Speed oscillator enable*/
#define RCC_BDCR_LSERDY                      (0x1U << 1U)                      /* External Low Speed oscillator Ready*/
#define RCC_BDCR_LSEBYP                      (0x1U << 2U)                      /* External Low Speed oscillator Bypass*/
#define RCC_BDCR_RTCSEL                      (0x3U << 8U)                     /* RTCSEL[1:0] bits (RTC clock source selection)*/
#define RCC_BDCR_RTCSEL_0                    (0x1U << 8U)
#define RCC_BDCR_RTCSEL_1                    (0x2U << 8U)
#define RCC_BDCR_RTCSEL_NOCLOCK              0x00000000U                       /* No clock*/
#define RCC_BDCR_RTCSEL_LSE                  0x00000100U                       /* LSE oscillator clock used as RTC clock*/
#define RCC_BDCR_RTCSEL_LSI                  0x00000200U                       /* LSI oscillator clock used as RTC clock*/
#define RCC_BDCR_RTCSEL_HSE                  0x00000300U                       /* HSE oscillator clock divided by 128 used as RTC clock*/
#define RCC_BDCR_RTCEN                       (0x1U << 15U)                     /* RTC clock enable*/
#define RCC_BDCR_BDRST                       (0x1U << 16U)                     /* Backup domain software reset*/

#define RCC_CSR_LSION                        (0x1U << 0U)                   /* Internal Low Speed oscillator enable*/
#define RCC_CSR_LSIRDY                       (0x1U << 1U)                   /* Internal Low Speed oscillator Ready*/
#define RCC_CSR_RMVF                         (0x1U << 24U)                  /* Remove reset flags*/
#define RCC_CSR_PINRSTF                      (0x1U << 26U)                  /* PIN reset flag*/
#define RCC_CSR_PORRSTF                      (0x1U << 27U)                  /* POR/PDR reset flag*/
#define RCC_CSR_SFTRSTF                      (0x1U << 28U)                  /* Software Reset flag*/
#define RCC_CSR_IWDGRSTF                     (0x1U << 29U)                  /* Independent Watchdog reset flag*/
#define RCC_CSR_WWDGRSTF                     (0x1U << 30U)                  /* Window watchdog reset flag*/
#define RCC_CSR_LPWRRSTF                     (0x1U << 31U)                  /* Low-Power reset flag*/


#define GPIO_CRL_MODE                        (0x33333333U << 0U)               /* Port x mode bits*/
#define GPIO_CRL_MODE0                       (0x3U << 0U)                      /* MODE0[1:0] bits (Port x mode bits, pin 0)*/
#define GPIO_CRL_MODE0_0                     (0x1U << 0U)
#define GPIO_CRL_MODE0_1                     (0x2U << 0U)
#define GPIO_CRL_MODE1                       (0x3U << 4U)                      /* MODE1[1:0] bits (Port x mode bits, pin 1)*/
#define GPIO_CRL_MODE1_0                     (0x1U << 4U)
#define GPIO_CRL_MODE1_1                     (0x2U << 4U)
#define GPIO_CRL_MODE2                       (0x3U << 8U)                      /* MODE2[1:0] bits (Port x mode bits, pin 2)*/
#define GPIO_CRL_MODE2_0                     (0x1U << 8U)
#define GPIO_CRL_MODE2_1                     (0x2U << 8U)
#define GPIO_CRL_MODE3                       (0x3U << 12U)                     /* MODE3[1:0] bits (Port x mode bits, pin 3)*/
#define GPIO_CRL_MODE3_0                     (0x1U << 12U)
#define GPIO_CRL_MODE3_1                     (0x2U << 12U)
#define GPIO_CRL_MODE4                       (0x3U << 16U)                     /* MODE4[1:0] bits (Port x mode bits, pin 4)*/
#define GPIO_CRL_MODE4_0                     (0x1U << 16U)
#define GPIO_CRL_MODE4_1                     (0x2U << 16U)
#define GPIO_CRL_MODE5                       (0x3U << 20U)                     /* MODE5[1:0] bits (Port x mode bits, pin 5)*/
#define GPIO_CRL_MODE5_0                     (0x1U << 20U)
#define GPIO_CRL_MODE5_1                     (0x2U << 20U)
#define GPIO_CRL_MODE6                       (0x3U << 24U)                     /* MODE6[1:0] bits (Port x mode bits, pin 6)*/
#define GPIO_CRL_MODE6_0                     (0x1U << 24U)
#define GPIO_CRL_MODE6_1                     (0x2U << 24U)
#define GPIO_CRL_MODE7                       (0x3U << 28U)                     /* MODE7[1:0] bits (Port x mode bits, pin 7)*/
#define GPIO_CRL_MODE7_0                     (0x1U << 28U)
#define GPIO_CRL_MODE7_1                     (0x2U << 28U)
#define GPIO_CRL_CNF                         (0x33333333U << 2U)               /* Port x configuration bits*/
#define GPIO_CRL_CNF0                        (0x3U << 2U)                      /* CNF0[1:0] bits (Port x configuration bits, pin 0)*/
#define GPIO_CRL_CNF0_0                      (0x1U << 2U)
#define GPIO_CRL_CNF0_1                      (0x2U << 2U)
#define GPIO_CRL_CNF1                        (0x3U << 6U)                      /* CNF1[1:0] bits (Port x configuration bits, pin 1)*/
#define GPIO_CRL_CNF1_0                      (0x1U << 6U)
#define GPIO_CRL_CNF1_1                      (0x2U << 6U)
#define GPIO_CRL_CNF2                        (0x3U << 10U)                     /* CNF2[1:0] bits (Port x configuration bits, pin 2)*/
#define GPIO_CRL_CNF2_0                      (0x1U << 10U)
#define GPIO_CRL_CNF2_1                      (0x2U << 10U)
#define GPIO_CRL_CNF3                        (0x3U << 14U)                     /* CNF3[1:0] bits (Port x configuration bits, pin 3)*/
#define GPIO_CRL_CNF3_0                      (0x1U << 14U)
#define GPIO_CRL_CNF3_1                      (0x2U << 14U)
#define GPIO_CRL_CNF4                        (0x3U << 18U)                     /* CNF4[1:0] bits (Port x configuration bits, pin 4)*/
#define GPIO_CRL_CNF4_0                      (0x1U << 18U)
#define GPIO_CRL_CNF4_1                      (0x2U << 18U)
#define GPIO_CRL_CNF5                        (0x3U << 22U)                     /* CNF5[1:0] bits (Port x configuration bits, pin 5)*/
#define GPIO_CRL_CNF5_0                      (0x1U << 22U)
#define GPIO_CRL_CNF5_1                      (0x2U << 22U)
#define GPIO_CRL_CNF6                        (0x3U << 26U)                     /* CNF6[1:0] bits (Port x configuration bits, pin 6)*/
#define GPIO_CRL_CNF6_0                      (0x1U << 26U)
#define GPIO_CRL_CNF6_1                      (0x2U << 26U)
#define GPIO_CRL_CNF7                        (0x3U << 30U)                     /* CNF7[1:0] bits (Port x configuration bits, pin 7)*/
#define GPIO_CRL_CNF7_0                      (0x1U << 30U)
#define GPIO_CRL_CNF7_1                      (0x2U << 30U)

#define GPIO_CRH_MODE                        (0x33333333U << 0U)               /* Port x mode bits*/
#define GPIO_CRH_MODE8                       (0x3U << 0U)                      /* MODE8[1:0] bits (Port x mode bits, pin 8)*/
#define GPIO_CRH_MODE8_0                     (0x1U << 0U)
#define GPIO_CRH_MODE8_1                     (0x2U << 0U)
#define GPIO_CRH_MODE9                       (0x3U << 4U)                      /* MODE9[1:0] bits (Port x mode bits, pin 9)*/
#define GPIO_CRH_MODE9_0                     (0x1U << 4U)
#define GPIO_CRH_MODE9_1                     (0x2U << 4U)
#define GPIO_CRH_MODE10                      (0x3U << 8U)                      /* MODE10[1:0] bits (Port x mode bits, pin 10)*/
#define GPIO_CRH_MODE10_0                    (0x1U << 8U)
#define GPIO_CRH_MODE10_1                    (0x2U << 8U)
#define GPIO_CRH_MODE11                      (0x3U << 12U)                     /* MODE11[1:0] bits (Port x mode bits, pin 11)*/
#define GPIO_CRH_MODE11_0                    (0x1U << 12U)
#define GPIO_CRH_MODE11_1                    (0x2U << 12U)
#define GPIO_CRH_MODE12                      (0x3U << 16U)                     /* MODE12[1:0] bits (Port x mode bits, pin 12)*/
#define GPIO_CRH_MODE12_0                    (0x1U << 16U)
#define GPIO_CRH_MODE12_1                    (0x2U << 16U)
#define GPIO_CRH_MODE13                      (0x3U << 20U)                     /* MODE13[1:0] bits (Port x mode bits, pin 13)*/
#define GPIO_CRH_MODE13_0                    (0x1U << 20U)
#define GPIO_CRH_MODE13_1                    (0x2U << 20U)
#define GPIO_CRH_MODE14                      (0x3U << 24U)                     /* MODE14[1:0] bits (Port x mode bits, pin 14)*/
#define GPIO_CRH_MODE14_0                    (0x1U << 24U)
#define GPIO_CRH_MODE14_1                    (0x2U << 24U)
#define GPIO_CRH_MODE15                      (0x3U << 28U)                     /* MODE15[1:0] bits (Port x mode bits, pin 15)*/
#define GPIO_CRH_MODE15_0                    (0x1U << 28U)
#define GPIO_CRH_MODE15_1                    (0x2U << 28U)
#define GPIO_CRH_CNF                         (0x33333333U << 2U)               /* Port x configuration bits*/
#define GPIO_CRH_CNF8                        (0x3U << 2U)                      /* CNF8[1:0] bits (Port x configuration bits, pin 8)*/
#define GPIO_CRH_CNF8_0                      (0x1U << 2U)
#define GPIO_CRH_CNF8_1                      (0x2U << 2U)
#define GPIO_CRH_CNF9                        (0x3U << 6U)                      /* CNF9[1:0] bits (Port x configuration bits, pin 9)*/
#define GPIO_CRH_CNF9_0                      (0x1U << 6U)
#define GPIO_CRH_CNF9_1                      (0x2U << 6U)
#define GPIO_CRH_CNF10                       (0x3U << 10U)                     /* CNF10[1:0] bits (Port x configuration bits, pin 10)*/
#define GPIO_CRH_CNF10_0                     (0x1U << 10U)
#define GPIO_CRH_CNF10_1                     (0x2U << 10U)
#define GPIO_CRH_CNF11                       (0x3U << 14U)                     /* CNF11[1:0] bits (Port x configuration bits, pin 11)*/
#define GPIO_CRH_CNF11_0                     (0x1U << 14U)
#define GPIO_CRH_CNF11_1                     (0x2U << 14U)
#define GPIO_CRH_CNF12                       (0x3U << 18U)                     /* CNF12[1:0] bits (Port x configuration bits, pin 12)*/
#define GPIO_CRH_CNF12_0                     (0x1U << 18U)
#define GPIO_CRH_CNF12_1                     (0x2U << 18U)
#define GPIO_CRH_CNF13                       (0x3U << 22U)                     /* CNF13[1:0] bits (Port x configuration bits, pin 13)*/
#define GPIO_CRH_CNF13_0                     (0x1U << 22U)
#define GPIO_CRH_CNF13_1                     (0x2U << 22U)
#define GPIO_CRH_CNF14                       (0x3U << 26U)                     /* CNF14[1:0] bits (Port x configuration bits, pin 14)*/
#define GPIO_CRH_CNF14_0                     (0x1U << 26U)
#define GPIO_CRH_CNF14_1                     (0x2U << 26U)
#define GPIO_CRH_CNF15                       (0x3U << 30U)                     /* CNF15[1:0] bits (Port x configuration bits, pin 15)*/
#define GPIO_CRH_CNF15_0                     (0x1U << 30U)
#define GPIO_CRH_CNF15_1                     (0x2U << 30U)

#define GPIO_IDR_IDR0                        (0x1U << 0U)                      /* Port input data, bit 0*/
#define GPIO_IDR_IDR1                        (0x1U << 1U)                      /* Port input data, bit 1*/
#define GPIO_IDR_IDR2                        (0x1U << 2U)                      /* Port input data, bit 2*/
#define GPIO_IDR_IDR3                        (0x1U << 3U)                      /* Port input data, bit 3*/
#define GPIO_IDR_IDR4                        (0x1U << 4U)                      /* Port input data, bit 4*/
#define GPIO_IDR_IDR5                        (0x1U << 5U)                      /* Port input data, bit 5*/
#define GPIO_IDR_IDR6                        (0x1U << 6U)                      /* Port input data, bit 6*/
#define GPIO_IDR_IDR7                        (0x1U << 7U)                      /* Port input data, bit 7*/
#define GPIO_IDR_IDR8                        (0x1U << 8U)                      /* Port input data, bit 8*/
#define GPIO_IDR_IDR9                        (0x1U << 9U)                      /* Port input data, bit 9*/
#define GPIO_IDR_IDR10                       (0x1U << 10U)                     /* Port input data, bit 10*/
#define GPIO_IDR_IDR11                       (0x1U << 11U)                     /* Port input data, bit 11*/
#define GPIO_IDR_IDR12                       (0x1U << 12U)                     /* Port input data, bit 12*/
#define GPIO_IDR_IDR13                       (0x1U << 13U)                     /* Port input data, bit 13*/
#define GPIO_IDR_IDR14                       (0x1U << 14U)                     /* Port input data, bit 14*/
#define GPIO_IDR_IDR15                       (0x1U << 15U)                     /* Port input data, bit 15*/

#define GPIO_ODR_ODR0                        (0x1U << 0U)                      /* Port output data, bit 0*/
#define GPIO_ODR_ODR1                        (0x1U << 1U)                      /* Port output data, bit 1*/
#define GPIO_ODR_ODR2                        (0x1U << 2U)                      /* Port output data, bit 2*/
#define GPIO_ODR_ODR3                        (0x1U << 3U)                      /* Port output data, bit 3*/
#define GPIO_ODR_ODR4                        (0x1U << 4U)                      /* Port output data, bit 4*/
#define GPIO_ODR_ODR5                        (0x1U << 5U)                      /* Port output data, bit 5*/
#define GPIO_ODR_ODR6                        (0x1U << 6U)                      /* Port output data, bit 6*/
#define GPIO_ODR_ODR7                        (0x1U << 7U)                      /* Port output data, bit 7*/
#define GPIO_ODR_ODR8                        (0x1U << 8U)                      /* Port output data, bit 8*/
#define GPIO_ODR_ODR9                        (0x1U << 9U)                      /* Port output data, bit 9*/
#define GPIO_ODR_ODR10                       (0x1U << 10U)                     /*!Port output data, bit 10*/
#define GPIO_ODR_ODR11                       (0x1U << 11U)                     /*!Port output data, bit 11*/
#define GPIO_ODR_ODR12                       (0x1U << 12U)                     /*!Port output data, bit 12*/
#define GPIO_ODR_ODR13                       (0x1U << 13U)                     /*!Port output data, bit 13*/
#define GPIO_ODR_ODR14                       (0x1U << 14U)                     /*!Port output data, bit 14*/
#define GPIO_ODR_ODR15                       (0x1U << 15U)                     /*!Port output data, bit 15*/

#define GPIO_BSRR_BS0                        (0x1U << 0U)                      /* Port x Set bit 0*/
#define GPIO_BSRR_BS1                        (0x1U << 1U)                      /* Port x Set bit 1*/
#define GPIO_BSRR_BS2                        (0x1U << 2U)                      /* Port x Set bit 2*/
#define GPIO_BSRR_BS3                        (0x1U << 3U)                      /* Port x Set bit 3*/
#define GPIO_BSRR_BS4                        (0x1U << 4U)                      /* Port x Set bit 4*/
#define GPIO_BSRR_BS5                        (0x1U << 5U)                      /* Port x Set bit 5*/
#define GPIO_BSRR_BS6                        (0x1U << 6U)                      /* Port x Set bit 6*/
#define GPIO_BSRR_BS7                        (0x1U << 7U)                      /* Port x Set bit 7*/
#define GPIO_BSRR_BS8                        (0x1U << 8U)                      /* Port x Set bit 8*/
#define GPIO_BSRR_BS9                        (0x1U << 9U)                      /* Port x Set bit 9*/
#define GPIO_BSRR_BS10                       (0x1U << 10U)                     /* Port x Set bit 10*/
#define GPIO_BSRR_BS11                       (0x1U << 11U)                     /* Port x Set bit 11*/
#define GPIO_BSRR_BS12                       (0x1U << 12U)                     /* Port x Set bit 12*/
#define GPIO_BSRR_BS13                       (0x1U << 13U)                     /* Port x Set bit 13*/
#define GPIO_BSRR_BS14                       (0x1U << 14U)                     /* Port x Set bit 14*/
#define GPIO_BSRR_BS15                       (0x1U << 15U)                     /* Port x Set bit 15*/
#define GPIO_BSRR_BR0                        (0x1U << 16U)                     /* Port x Reset bit 0*/
#define GPIO_BSRR_BR1                        (0x1U << 17U)                     /* Port x Reset bit 1*/
#define GPIO_BSRR_BR2                        (0x1U << 18U)                     /* Port x Reset bit 2*/
#define GPIO_BSRR_BR3                        (0x1U << 19U)                     /* Port x Reset bit 3*/
#define GPIO_BSRR_BR4                        (0x1U << 20U)                     /* Port x Reset bit 4*/
#define GPIO_BSRR_BR5                        (0x1U << 21U)                     /* Port x Reset bit 5*/
#define GPIO_BSRR_BR6                        (0x1U << 22U)                     /* Port x Reset bit 6*/
#define GPIO_BSRR_BR7                        (0x1U << 23U)                     /* Port x Reset bit 7*/
#define GPIO_BSRR_BR8                        (0x1U << 24U)                     /* Port x Reset bit 8*/
#define GPIO_BSRR_BR9                        (0x1U << 25U)                     /* Port x Reset bit 9*/
#define GPIO_BSRR_BR10                       (0x1U << 26U)                     /* Port x Reset bit 10*/
#define GPIO_BSRR_BR11                       (0x1U << 27U)                     /* Port x Reset bit 11*/
#define GPIO_BSRR_BR12                       (0x1U << 28U)                     /* Port x Reset bit 12*/
#define GPIO_BSRR_BR13                       (0x1U << 29U)                     /* Port x Reset bit 13*/
#define GPIO_BSRR_BR14                       (0x1U << 30U)                     /* Port x Reset bit 14*/
#define GPIO_BSRR_BR15                       (0x1U << 31U)                     /* Port x Reset bit 15*/

#define GPIO_BRR_BR0                         (0x1U << 0U)                      /* Port x Reset bit 0*/
#define GPIO_BRR_BR1                         (0x1U << 1U)                      /* Port x Reset bit 1*/
#define GPIO_BRR_BR2                         (0x1U << 2U)                      /* Port x Reset bit 2*/
#define GPIO_BRR_BR3                         (0x1U << 3U)                      /* Port x Reset bit 3*/
#define GPIO_BRR_BR4                         (0x1U << 4U)                      /* Port x Reset bit 4*/
#define GPIO_BRR_BR5                         (0x1U << 5U)                      /* Port x Reset bit 5*/
#define GPIO_BRR_BR6                         (0x1U << 6U)                      /* Port x Reset bit 6*/
#define GPIO_BRR_BR7                         (0x1U << 7U)                      /* Port x Reset bit 7*/
#define GPIO_BRR_BR8                         (0x1U << 8U)                      /* Port x Reset bit 8*/
#define GPIO_BRR_BR9                         (0x1U << 9U)                      /* Port x Reset bit 9*/
#define GPIO_BRR_BR10                        (0x1U << 10U)                     /* Port x Reset bit 10*/
#define GPIO_BRR_BR11                        (0x1U << 11U)                     /* Port x Reset bit 11*/
#define GPIO_BRR_BR12                        (0x1U << 12U)                     /* Port x Reset bit 12*/
#define GPIO_BRR_BR13                        (0x1U << 13U)                     /* Port x Reset bit 13*/
#define GPIO_BRR_BR14                        (0x1U << 14U)                     /* Port x Reset bit 14*/
#define GPIO_BRR_BR15                        (0x1U << 15U)                     /* Port x Reset bit 15*/

#define GPIO_LCKR_LCK0                       (0x1U << 0U)                      /* Port x Lock bit 0*/
#define GPIO_LCKR_LCK1                       (0x1U << 1U)                      /* Port x Lock bit 1*/
#define GPIO_LCKR_LCK2                       (0x1U << 2U)                      /* Port x Lock bit 2*/
#define GPIO_LCKR_LCK3                       (0x1U << 3U)                      /* Port x Lock bit 3*/
#define GPIO_LCKR_LCK4                       (0x1U << 4U)                      /* Port x Lock bit 4*/
#define GPIO_LCKR_LCK5                       (0x1U << 5U)                      /* Port x Lock bit 5*/
#define GPIO_LCKR_LCK6                       (0x1U << 6U)                      /* Port x Lock bit 6*/
#define GPIO_LCKR_LCK7                       (0x1U << 7U)                      /* Port x Lock bit 7*/
#define GPIO_LCKR_LCK8                       (0x1U << 8U)                      /* Port x Lock bit 8*/
#define GPIO_LCKR_LCK9                       (0x1U << 9U)                      /* Port x Lock bit 9*/
#define GPIO_LCKR_LCK10                      (0x1U << 10U)                     /* Port x Lock bit 10*/
#define GPIO_LCKR_LCK11                      (0x1U << 11U)                     /* Port x Lock bit 11*/
#define GPIO_LCKR_LCK12                      (0x1U << 12U)                     /* Port x Lock bit 12*/
#define GPIO_LCKR_LCK13                      (0x1U << 13U)                     /* Port x Lock bit 13*/
#define GPIO_LCKR_LCK14                      (0x1U << 14U)                     /* Port x Lock bit 14*/
#define GPIO_LCKR_LCK15                      (0x1U << 15U)                     /* Port x Lock bit 15*/
#define GPIO_LCKR_LCKK                       (0x1U << 16U)                     /* Lock key*/


#define AFIO_EVCR_PIN                        (0xFU << 0U)                      /* PIN[3:0] bits (Pin selection)*/
#define AFIO_EVCR_PIN_0                      (0x1U << 0U)
#define AFIO_EVCR_PIN_1                      (0x2U << 0U)
#define AFIO_EVCR_PIN_2                      (0x4U << 0U)
#define AFIO_EVCR_PIN_3                      (0x8U << 0U)
#define AFIO_EVCR_PIN_PX0                    0x00000000U                       /* Pin 0 selected*/
#define AFIO_EVCR_PIN_PX1                    (0x1U << 0U)                      /*! Pin 1 selected*/
#define AFIO_EVCR_PIN_PX2                    (0x2U << 0U)                      /*! Pin 2 selected*/
#define AFIO_EVCR_PIN_PX3                    (0x3U << 0U)                      /*! Pin 3 selected*/
#define AFIO_EVCR_PIN_PX4                    (0x4U << 0U)                      /*! Pin 4 selected*/
#define AFIO_EVCR_PIN_PX5                    (0x5U << 0U)                      /*! Pin 5 selected*/
#define AFIO_EVCR_PIN_PX6                    (0x6U << 0U)                      /*! Pin 6 selected*/
#define AFIO_EVCR_PIN_PX7                    (0x7U << 0U)                      /*! Pin 7 selected*/
#define AFIO_EVCR_PIN_PX8                    (0x8U << 0U)                      /*! Pin 8 selected*/
#define AFIO_EVCR_PIN_PX9                    (0x9U << 0U)                      /*! Pin 9 selected*/
#define AFIO_EVCR_PIN_PX10                   (0xAU << 0U)                      /*! Pin 10 selected*/
#define AFIO_EVCR_PIN_PX11                   (0xBU << 0U)                      /*! Pin 11 selected*/
#define AFIO_EVCR_PIN_PX12                   (0xCU << 0U)                      /*! Pin 12 selected*/
#define AFIO_EVCR_PIN_PX13                   (0xDU << 0U)                      /*! Pin 13 selected*/
#define AFIO_EVCR_PIN_PX14                   (0xEU << 0U)                      /*! Pin 14 selected*/
#define AFIO_EVCR_PIN_PX15                   (0xFU << 0U)                      /*! Pin 15 selected*/
#define AFIO_EVCR_PORT                       (0x7U << 4U)                      /* PORT[2:0] bits (Port selection)*/
#define AFIO_EVCR_PORT_0                     (0x1U << 4U)
#define AFIO_EVCR_PORT_1                     (0x2U << 4U)
#define AFIO_EVCR_PORT_2                     (0x4U << 4U)
#define AFIO_EVCR_PORT_PA                    (0x0U << 4U)                      /* Port A selected*/
#define AFIO_EVCR_PORT_PB                    (0x1U << 4U)                      /* Port B selected*/
#define AFIO_EVCR_PORT_PC                    (0x2U << 4U)                      /* Port C selected*/
#define AFIO_EVCR_PORT_PD                    (0x3U << 4U)                      /* Port D selected*/
#define AFIO_EVCR_PORT_PE                    (0x4U << 4U)                      /* Port E selected*/
#define AFIO_EVCR_EVOE                       (0x1U << 7U)                      /* Event Output Enable*/

#define AFIO_MAPR_SPI1_REMAP                 (0x1U << 0U)                      /* SPI1 remapping*/
#define AFIO_MAPR_I2C1_REMAP                 (0x1U << 1U)                      /* I2C1 remapping*/
#define AFIO_MAPR_USART1_REMAP               (0x1U << 2U)                      /* USART1 remapping*/
#define AFIO_MAPR_USART2_REMAP               (0x1U << 3U)                      /* USART2 remapping*/
#define AFIO_MAPR_USART3_REMAP               (0x3U << 4U)                      /* USART3_REMAP[1:0] bits (USART3 remapping)*/
#define AFIO_MAPR_USART3_REMAP_0             (0x1U << 4U)
#define AFIO_MAPR_USART3_REMAP_1             (0x2U << 4U)
#define AFIO_MAPR_USART3_REMAP_NOREMAP       (0x0U << 4U)                      /* No remap (TX/PB10, RX/PB11, CK/PB12, CTS/PB13, RTS/PB14)*/
#define AFIO_MAPR_USART3_REMAP_PARTIALREMAP  (0x1U << 4U)                      /* Partial remap (TX/PC10, RX/PC11, CK/PC12, CTS/PB13, RTS/PB14)*/
#define AFIO_MAPR_USART3_REMAP_FULLREMAP     (0x3U << 4U)                      /* Full remap (TX/PD8, RX/PD9, CK/PD10, CTS/PD11, RTS/PD12)*/
#define AFIO_MAPR_TIM1_REMAP                 (0x3U << 6U)                      /* TIM1_REMAP[1:0] bits (TIM1 remapping)*/
#define AFIO_MAPR_TIM1_REMAP_0               (0x1U << 6U)
#define AFIO_MAPR_TIM1_REMAP_1               (0x2U << 6U)
#define AFIO_MAPR_TIM1_REMAP_NOREMAP         (0x0U << 6U)                      /* No remap (ETR/PA12, CH1/PA8, CH2/PA9, CH3/PA10, CH4/PA11, BKIN/PB12, CH1N/PB13, CH2N/PB14, CH3N/PB15)*/
#define AFIO_MAPR_TIM1_REMAP_PARTIALREMAP    (0x1U << 6U)                      /* Partial remap (ETR/PA12, CH1/PA8, CH2/PA9, CH3/PA10, CH4/PA11, BKIN/PA6, CH1N/PA7, CH2N/PB0, CH3N/PB1)*/
#define AFIO_MAPR_TIM1_REMAP_FULLREMAP       (0x3U << 6U)                      /* Full remap (ETR/PE7, CH1/PE9, CH2/PE11, CH3/PE13, CH4/PE14, BKIN/PE15, CH1N/PE8, CH2N/PE10, CH3N/PE12)*/
#define AFIO_MAPR_TIM2_REMAP                 (0x3U << 8U)                      /* TIM2_REMAP[1:0] bits (TIM2 remapping)*/
#define AFIO_MAPR_TIM2_REMAP_0               (0x1U << 8U)
#define AFIO_MAPR_TIM2_REMAP_1               (0x2U << 8U)
#define AFIO_MAPR_TIM2_REMAP_NOREMAP         (0x0U << 8U)                      /* No remap (CH1/ETR/PA0, CH2/PA1, CH3/PA2, CH4/PA3)*/
#define AFIO_MAPR_TIM2_REMAP_PARTIALREMAP1   (0x1U << 8U)                      /* Partial remap (CH1/ETR/PA15, CH2/PB3, CH3/PA2, CH4/PA3)*/
#define AFIO_MAPR_TIM2_REMAP_PARTIALREMAP2   (0x2U << 8U)                      /* Partial remap (CH1/ETR/PA0, CH2/PA1, CH3/PB10, CH4/PB11)*/
#define AFIO_MAPR_TIM2_REMAP_FULLREMAP       (0x3U << 8U)                      /* Full remap (CH1/ETR/PA15, CH2/PB3, CH3/PB10, CH4/PB11)*/
#define AFIO_MAPR_TIM3_REMAP                 (0x3U << 10U)                     /* TIM3_REMAP[1:0] bits (TIM3 remapping)*/
#define AFIO_MAPR_TIM3_REMAP_0               (0x1U << 10U)
#define AFIO_MAPR_TIM3_REMAP_1               (0x2U << 10U)
#define AFIO_MAPR_TIM3_REMAP_NOREMAP         (0x0U << 10U)                     /* No remap (CH1/PA6, CH2/PA7, CH3/PB0, CH4/PB1)*/
#define AFIO_MAPR_TIM3_REMAP_PARTIALREMAP    (0x2U << 10U)                     /* Partial remap (CH1/PB4, CH2/PB5, CH3/PB0, CH4/PB1)*/
#define AFIO_MAPR_TIM3_REMAP_FULLREMAP       (0x3U << 10U)                     /* Full remap (CH1/PC6, CH2/PC7, CH3/PC8, CH4/PC9)*/
#define AFIO_MAPR_TIM4_REMAP                 (0x1U << 12U)                     /* TIM4_REMAP bit (TIM4 remapping)*/
#define AFIO_MAPR_CAN_REMAP                  (0x3U << 13U)                     /* CAN_REMAP[1:0] bits (CAN Alternate function remapping)*/
#define AFIO_MAPR_CAN_REMAP_0                (0x1U << 13U)
#define AFIO_MAPR_CAN_REMAP_1                (0x2U << 13U)
#define AFIO_MAPR_CAN_REMAP_REMAP1           (0x0U << 13U)                     /* CANRX mapped to PA11, CANTX mapped to PA12*/
#define AFIO_MAPR_CAN_REMAP_REMAP2           (0x2U << 13U)                     /* CANRX mapped to PB8, CANTX mapped to PB9*/
#define AFIO_MAPR_CAN_REMAP_REMAP3           (0x3U << 13U)                     /* CANRX mapped to PD0, CANTX mapped to PD1*/
#define AFIO_MAPR_PD01_REMAP                 (0x1U << 15U)                     /* Port D0/Port D1 mapping on OSC_IN/OSC_OUT*/
#define AFIO_MAPR_SWJ_CFG                    (0x7U << 24U)                     /* SWJ_CFG[2:0] bits (Serial Wire JTAG configuration)*/
#define AFIO_MAPR_SWJ_CFG_0                  (0x1U << 24U)
#define AFIO_MAPR_SWJ_CFG_1                  (0x2U << 24U)
#define AFIO_MAPR_SWJ_CFG_2                  (0x4U << 24U)
#define AFIO_MAPR_SWJ_CFG_RESET              (0x0U << 24U)                     /* Full SWJ (JTAG-DP + SW-DP) : Reset State*/
#define AFIO_MAPR_SWJ_CFG_NOJNTRST           (0x1U << 24U)                     /* Full SWJ (JTAG-DP + SW-DP) but without JNTRST*/
#define AFIO_MAPR_SWJ_CFG_JTAGDISABLE        (0x2U << 24U)                     /* JTAG-DP Disabled and SW-DP Enabled*/
#define AFIO_MAPR_SWJ_CFG_DISABLE            (0x4U << 24U)                     /* JTAG-DP Disabled and SW-DP Disabled*/

#define AFIO_EXTICR1_EXTI0                   (0xFU << 0U)                      /* EXTI 0 configuration*/
#define AFIO_EXTICR1_EXTI0_0                 (0x1U << 0U)
#define AFIO_EXTICR1_EXTI0_1                 (0x2U << 0U)
#define AFIO_EXTICR1_EXTI0_2                 (0x4U << 0U)
#define AFIO_EXTICR1_EXTI0_3                 (0x8U << 0U)
#define AFIO_EXTICR1_EXTI0_PA                (0x0U << 0U)                      /* PA[0] pin*/
#define AFIO_EXTICR1_EXTI0_PB                (0x1U << 0U)                      /* PB[0] pin*/
#define AFIO_EXTICR1_EXTI0_PC                (0x2U << 0U)                      /* PC[0] pin*/
#define AFIO_EXTICR1_EXTI0_PD                (0x3U << 0U)                      /* PD[0] pin*/
#define AFIO_EXTICR1_EXTI0_PE                (0x4U << 0U)                      /* PE[0] pin*/
#define AFIO_EXTICR1_EXTI0_PF                (0x5U << 0U)                      /* PF[0] pin*/
#define AFIO_EXTICR1_EXTI0_PG                (0x6U << 0U)                      /* PG[0] pin*/
#define AFIO_EXTICR1_EXTI1                   (0xFU << 4U)                      /* EXTI 1 configuration*/
#define AFIO_EXTICR1_EXTI1_0                 (0x1U << 4U)
#define AFIO_EXTICR1_EXTI1_1                 (0x2U << 4U)
#define AFIO_EXTICR1_EXTI1_2                 (0x4U << 4U)
#define AFIO_EXTICR1_EXTI1_3                 (0x8U << 4U)
#define AFIO_EXTICR1_EXTI1_PA                (0x0U << 4U)                      /* PA[1] pin*/
#define AFIO_EXTICR1_EXTI1_PB                (0x1U << 4U)                      /* PB[1] pin*/
#define AFIO_EXTICR1_EXTI1_PC                (0x2U << 4U)                      /* PC[1] pin*/
#define AFIO_EXTICR1_EXTI1_PD                (0x3U << 4U)                      /* PD[1] pin*/
#define AFIO_EXTICR1_EXTI1_PE                (0x4U << 4U)                      /* PE[1] pin*/
#define AFIO_EXTICR1_EXTI1_PF                (0x5U << 4U)                      /* PF[1] pin*/
#define AFIO_EXTICR1_EXTI1_PG                (0x6U << 4U)                      /* PG[1] pin*/
#define AFIO_EXTICR1_EXTI2                   (0xFU << 8U)                      /* EXTI 2 configuration*/
#define AFIO_EXTICR1_EXTI2_0                 (0x1U << 8U)
#define AFIO_EXTICR1_EXTI2_1                 (0x2U << 8U)
#define AFIO_EXTICR1_EXTI2_2                 (0x4U << 8U)
#define AFIO_EXTICR1_EXTI2_3                 (0x8U << 8U)
#define AFIO_EXTICR1_EXTI2_PA                (0x0U << 8U)                      /* PA[2] pin*/
#define AFIO_EXTICR1_EXTI2_PB                (0x1U << 8U)                      /* PB[2] pin*/
#define AFIO_EXTICR1_EXTI2_PC                (0x2U << 8U)                      /* PC[2] pin*/
#define AFIO_EXTICR1_EXTI2_PD                (0x3U << 8U)                      /* PD[2] pin*/
#define AFIO_EXTICR1_EXTI2_PE                (0x4U << 8U)                      /* PE[2] pin*/
#define AFIO_EXTICR1_EXTI2_PF                (0x5U << 8U)                      /* PF[2] pin*/
#define AFIO_EXTICR1_EXTI2_PG                (0x6U << 8U)                      /* PG[2] pin*/
#define AFIO_EXTICR1_EXTI3                   (0xFU << 12U)                     /* EXTI 3 configuration*/
#define AFIO_EXTICR1_EXTI3_0                 (0x1U << 12U)
#define AFIO_EXTICR1_EXTI3_1                 (0x2U << 12U)
#define AFIO_EXTICR1_EXTI3_2                 (0x4U << 12U)
#define AFIO_EXTICR1_EXTI3_3                 (0x8U << 12U)
#define AFIO_EXTICR1_EXTI3_PA                (0x0U << 12U)                     /* PA[3] pin*/
#define AFIO_EXTICR1_EXTI3_PB                (0x1U << 12U)                     /* PB[3] pin*/
#define AFIO_EXTICR1_EXTI3_PC                (0x2U << 12U)                     /* PC[3] pin*/
#define AFIO_EXTICR1_EXTI3_PD                (0x3U << 12U)                     /* PD[3] pin*/
#define AFIO_EXTICR1_EXTI3_PE                (0x4U << 12U)                     /* PE[3] pin*/
#define AFIO_EXTICR1_EXTI3_PF                (0x5U << 12U)                     /* PF[3] pin*/
#define AFIO_EXTICR1_EXTI3_PG                (0x6U << 12U)                     /* PG[3] pin*/

#define AFIO_EXTICR2_EXTI4                   (0xFU << 0U)                      /* EXTI 4 configuration*/
#define AFIO_EXTICR2_EXTI4_0                 (0x1U << 0U)
#define AFIO_EXTICR2_EXTI4_1                 (0x2U << 0U)
#define AFIO_EXTICR2_EXTI4_2                 (0x4U << 0U)
#define AFIO_EXTICR2_EXTI4_3                 (0x8U << 0U)
#define AFIO_EXTICR2_EXTI4_PA                (0x0U << 0U)                      /* PA[4] pin*/
#define AFIO_EXTICR2_EXTI4_PB                (0x1U << 0U)                      /* PB[4] pin*/
#define AFIO_EXTICR2_EXTI4_PC                (0x2U << 0U)                      /* PC[4] pin*/
#define AFIO_EXTICR2_EXTI4_PD                (0x3U << 0U)                      /* PD[4] pin*/
#define AFIO_EXTICR2_EXTI4_PE                (0x4U << 0U)                      /* PE[4] pin*/
#define AFIO_EXTICR2_EXTI4_PF                (0x5U << 0U)                      /* PF[4] pin*/
#define AFIO_EXTICR2_EXTI4_PG                (0x6U << 0U)                      /* PG[4] pin*/
#define AFIO_EXTICR2_EXTI5                   (0xFU << 4U)                      /* EXTI 5 configuration*/
#define AFIO_EXTICR2_EXTI5_0                 (0x1U << 4U)
#define AFIO_EXTICR2_EXTI5_1                 (0x2U << 4U)
#define AFIO_EXTICR2_EXTI5_2                 (0x4U << 4U)
#define AFIO_EXTICR2_EXTI5_3                 (0x8U << 4U)
#define AFIO_EXTICR2_EXTI5_PA                (0x0U << 4U)                      /* PA[5] pin*/
#define AFIO_EXTICR2_EXTI5_PB                (0x1U << 4U)                      /* PB[5] pin*/
#define AFIO_EXTICR2_EXTI5_PC                (0x2U << 4U)                      /* PC[5] pin*/
#define AFIO_EXTICR2_EXTI5_PD                (0x3U << 4U)                      /* PD[5] pin*/
#define AFIO_EXTICR2_EXTI5_PE                (0x4U << 4U)                      /* PE[5] pin*/
#define AFIO_EXTICR2_EXTI5_PF                (0x5U << 4U)                      /* PF[5] pin*/
#define AFIO_EXTICR2_EXTI5_PG                (0x6U << 4U)                      /* PG[5] pin*/
#define AFIO_EXTICR2_EXTI6                   (0xFU << 8U)                      /* EXTI 6 configuration*/
#define AFIO_EXTICR2_EXTI6_0                 (0x1U << 8U)
#define AFIO_EXTICR2_EXTI6_1                 (0x2U << 8U)
#define AFIO_EXTICR2_EXTI6_2                 (0x4U << 8U)
#define AFIO_EXTICR2_EXTI6_3                 (0x8U << 8U)
#define AFIO_EXTICR2_EXTI6_PA                (0x0U << 8U)                      /* PA[6] pin*/
#define AFIO_EXTICR2_EXTI6_PB                (0x1U << 8U)                      /* PB[6] pin*/
#define AFIO_EXTICR2_EXTI6_PC                (0x2U << 8U)                      /* PC[6] pin*/
#define AFIO_EXTICR2_EXTI6_PD                (0x3U << 8U)                      /* PD[6] pin*/
#define AFIO_EXTICR2_EXTI6_PE                (0x4U << 8U)                      /* PE[6] pin*/
#define AFIO_EXTICR2_EXTI6_PF                (0x5U << 8U)                      /* PF[6] pin*/
#define AFIO_EXTICR2_EXTI6_PG                (0x6U << 8U)                      /* PG[6] pin*/
#define AFIO_EXTICR2_EXTI7                   (0xFU << 12U)                     /* EXTI 7 configuration*/
#define AFIO_EXTICR2_EXTI7_0                 (0x1U << 12U)
#define AFIO_EXTICR2_EXTI7_1                 (0x2U << 12U)
#define AFIO_EXTICR2_EXTI7_2                 (0x4U << 12U)
#define AFIO_EXTICR2_EXTI7_3                 (0x8U << 12U)
#define AFIO_EXTICR2_EXTI7_PA                (0x0U << 12U)                     /* PA[7] pin*/
#define AFIO_EXTICR2_EXTI7_PB                (0x1U << 12U)                     /* PB[7] pin*/
#define AFIO_EXTICR2_EXTI7_PC                (0x2U << 12U)                     /* PC[7] pin*/
#define AFIO_EXTICR2_EXTI7_PD                (0x3U << 12U)                     /* PD[7] pin*/
#define AFIO_EXTICR2_EXTI7_PE                (0x4U << 12U)                     /* PE[7] pin*/
#define AFIO_EXTICR2_EXTI7_PF                (0x5U << 12U)                     /* PF[7] pin*/
#define AFIO_EXTICR2_EXTI7_PG                (0x6U << 12U)                     /* PG[7] pin*/

#define AFIO_EXTICR3_EXTI8                   (0xFU << 0U)                      /* EXTI 8 configuration*/
#define AFIO_EXTICR3_EXTI8_0                 (0x1U << 0U)
#define AFIO_EXTICR3_EXTI8_1                 (0x2U << 0U)
#define AFIO_EXTICR3_EXTI8_2                 (0x4U << 0U)
#define AFIO_EXTICR3_EXTI8_3                 (0x8U << 0U)
#define AFIO_EXTICR3_EXTI8_PA                (0x0U << 0U)                      /* PA[8] pin*/
#define AFIO_EXTICR3_EXTI8_PB                (0x1U << 0U)                      /* PB[8] pin*/
#define AFIO_EXTICR3_EXTI8_PC                (0x2U << 0U)                      /* PC[8] pin*/
#define AFIO_EXTICR3_EXTI8_PD                (0x3U << 0U)                      /* PD[8] pin*/
#define AFIO_EXTICR3_EXTI8_PE                (0x4U << 0U)                      /* PE[8] pin*/
#define AFIO_EXTICR3_EXTI8_PF                (0x5U << 0U)                      /* PF[8] pin*/
#define AFIO_EXTICR3_EXTI8_PG                (0x6U << 0U)                      /* PG[8] pin*/
#define AFIO_EXTICR3_EXTI9                   (0xFU << 4U)                      /* EXTI 9 configuration*/
#define AFIO_EXTICR3_EXTI9_0                 (0x1U << 4U)
#define AFIO_EXTICR3_EXTI9_1                 (0x2U << 4U)
#define AFIO_EXTICR3_EXTI9_2                 (0x4U << 4U)
#define AFIO_EXTICR3_EXTI9_3                 (0x8U << 4U)
#define AFIO_EXTICR3_EXTI9_PA                (0x0U << 4U)                      /* PA[9] pin*/
#define AFIO_EXTICR3_EXTI9_PB                (0x1U << 4U)                      /* PB[9] pin*/
#define AFIO_EXTICR3_EXTI9_PC                (0x2U << 4U)                      /* PC[9] pin*/
#define AFIO_EXTICR3_EXTI9_PD                (0x3U << 4U)                      /* PD[9] pin*/
#define AFIO_EXTICR3_EXTI9_PE                (0x4U << 4U)                      /* PE[9] pin*/
#define AFIO_EXTICR3_EXTI9_PF                (0x5U << 4U)                      /* PF[9] pin*/
#define AFIO_EXTICR3_EXTI9_PG                (0x6U << 4U)                      /* PG[9] pin*/
#define AFIO_EXTICR3_EXTI10                  (0xFU << 8U)                      /* EXTI 10 configuration*/
#define AFIO_EXTICR3_EXTI10_0                (0x1U << 8U)
#define AFIO_EXTICR3_EXTI10_1                (0x2U << 8U)
#define AFIO_EXTICR3_EXTI10_2                (0x4U << 8U)
#define AFIO_EXTICR3_EXTI10_3                (0x8U << 8U)
#define AFIO_EXTICR3_EXTI10_PA               (0x0U << 8U)                      /* PA[10] pin*/
#define AFIO_EXTICR3_EXTI10_PB               (0x1U << 8U)                      /* PB[10] pin*/
#define AFIO_EXTICR3_EXTI10_PC               (0x2U << 8U)                      /* PC[10] pin*/
#define AFIO_EXTICR3_EXTI10_PD               (0x3U << 8U)                      /* PD[10] pin*/
#define AFIO_EXTICR3_EXTI10_PE               (0x4U << 8U)                      /* PE[10] pin*/
#define AFIO_EXTICR3_EXTI10_PF               (0x5U << 8U)                      /* PF[10] pin*/
#define AFIO_EXTICR3_EXTI10_PG               (0x6U << 8U)                      /* PG[10] pin*/
#define AFIO_EXTICR3_EXTI11                  (0xFU << 12U)                     /* EXTI 11 configuration*/
#define AFIO_EXTICR3_EXTI11_0                (0x1U << 12U)
#define AFIO_EXTICR3_EXTI11_1                (0x2U << 12U)
#define AFIO_EXTICR3_EXTI11_2                (0x4U << 12U)
#define AFIO_EXTICR3_EXTI11_3                (0x8U << 12U)
#define AFIO_EXTICR3_EXTI11_PA               (0x0U << 12U)                     /* PA[11] pin*/
#define AFIO_EXTICR3_EXTI11_PB               (0x1U << 12U)                     /* PB[11] pin*/
#define AFIO_EXTICR3_EXTI11_PC               (0x2U << 12U)                     /* PC[11] pin*/
#define AFIO_EXTICR3_EXTI11_PD               (0x3U << 12U)                     /* PD[11] pin*/
#define AFIO_EXTICR3_EXTI11_PE               (0x4U << 12U)                     /* PE[11] pin*/
#define AFIO_EXTICR3_EXTI11_PF               (0x5U << 12U)                     /* PF[11] pin*/
#define AFIO_EXTICR3_EXTI11_PG               (0x6U << 12U)                     /* PG[11] pin*/

#define AFIO_EXTICR4_EXTI12                  (0xFU << 0U)                      /* EXTI 12 configuration*/
#define AFIO_EXTICR4_EXTI12_0                (0x1U << 0U)
#define AFIO_EXTICR4_EXTI12_1                (0x2U << 0U)
#define AFIO_EXTICR4_EXTI12_2                (0x4U << 0U)
#define AFIO_EXTICR4_EXTI12_3                (0x8U << 0U)
#define AFIO_EXTICR4_EXTI12_PA               (0x0U << 0U)                      /* PA[12] pin*/
#define AFIO_EXTICR4_EXTI12_PB               (0x1U << 0U)                      /* PB[12] pin*/
#define AFIO_EXTICR4_EXTI12_PC               (0x2U << 0U)                      /* PC[12] pin*/
#define AFIO_EXTICR4_EXTI12_PD               (0x3U << 0U)                      /* PD[12] pin*/
#define AFIO_EXTICR4_EXTI12_PE               (0x4U << 0U)                      /* PE[12] pin*/
#define AFIO_EXTICR4_EXTI12_PF               (0x5U << 0U)                      /* PF[12] pin*/
#define AFIO_EXTICR4_EXTI12_PG               (0x6U << 0U)                      /* PG[12] pin*/
#define AFIO_EXTICR4_EXTI13                  (0xFU << 4U)                      /* EXTI 13 configuration*/
#define AFIO_EXTICR4_EXTI13_0                (0x1U << 4U)
#define AFIO_EXTICR4_EXTI13_1                (0x2U << 4U)
#define AFIO_EXTICR4_EXTI13_2                (0x4U << 4U)
#define AFIO_EXTICR4_EXTI13_3                (0x8U << 4U)
#define AFIO_EXTICR4_EXTI13_PA               (0x0U << 4U)                      /* PA[13] pin*/
#define AFIO_EXTICR4_EXTI13_PB               (0x1U << 4U)                      /* PB[13] pin*/
#define AFIO_EXTICR4_EXTI13_PC               (0x2U << 4U)                      /* PC[13] pin*/
#define AFIO_EXTICR4_EXTI13_PD               (0x3U << 4U)                      /* PD[13] pin*/
#define AFIO_EXTICR4_EXTI13_PE               (0x4U << 4U)                      /* PE[13] pin*/
#define AFIO_EXTICR4_EXTI13_PF               (0x5U << 4U)                      /* PF[13] pin*/
#define AFIO_EXTICR4_EXTI13_PG               (0x6U << 4U)                      /* PG[13] pin*/
#define AFIO_EXTICR4_EXTI14                  (0xFU << 8U)                      /* EXTI 14 configuration*/
#define AFIO_EXTICR4_EXTI14_0                (0x1U << 8U)
#define AFIO_EXTICR4_EXTI14_1                (0x2U << 8U)
#define AFIO_EXTICR4_EXTI14_2                (0x4U << 8U)
#define AFIO_EXTICR4_EXTI14_3                (0x8U << 8U)
#define AFIO_EXTICR4_EXTI14_PA               (0x0U << 8U)                      /* PA[14] pin*/
#define AFIO_EXTICR4_EXTI14_PB               (0x1U << 8U)                      /* PB[14] pin*/
#define AFIO_EXTICR4_EXTI14_PC               (0x2U << 8U)                      /* PC[14] pin*/
#define AFIO_EXTICR4_EXTI14_PD               (0x3U << 8U)                      /* PD[14] pin*/
#define AFIO_EXTICR4_EXTI14_PE               (0x4U << 8U)                      /* PE[14] pin*/
#define AFIO_EXTICR4_EXTI14_PF               (0x5U << 8U)                      /* PF[14] pin*/
#define AFIO_EXTICR4_EXTI14_PG               (0x6U << 8U)                      /* PG[14] pin*/
#define AFIO_EXTICR4_EXTI15                  (0xFU << 12U)                     /* EXTI 15 configuration*/
#define AFIO_EXTICR4_EXTI15_0                (0x1U << 12U)
#define AFIO_EXTICR4_EXTI15_1                (0x2U << 12U)
#define AFIO_EXTICR4_EXTI15_2                (0x4U << 12U)
#define AFIO_EXTICR4_EXTI15_3                (0x8U << 12U)
#define AFIO_EXTICR4_EXTI15_PA               (0x0U << 12U)                      /* PA[15] pin*/
#define AFIO_EXTICR4_EXTI15_PB               (0x1U << 12U)                      /* PB[15] pin*/
#define AFIO_EXTICR4_EXTI15_PC               (0x2U << 12U)                      /* PC[15] pin*/
#define AFIO_EXTICR4_EXTI15_PD               (0x3U << 12U)                      /* PD[15] pin*/
#define AFIO_EXTICR4_EXTI15_PE               (0x4U << 12U)                      /* PE[15] pin*/
#define AFIO_EXTICR4_EXTI15_PF               (0x5U << 12U)                      /* PF[15] pin*/
#define AFIO_EXTICR4_EXTI15_PG               (0x6U << 12U)                      /* PG[15] pin*/


#define EXTI_IMR_MR0                        (0x1U << 0U)                       /* Interrupt Mask on line 0*/
#define EXTI_IMR_MR1                        (0x1U << 1U)                       /* Interrupt Mask on line 1*/
#define EXTI_IMR_MR2                        (0x1U << 2U)                       /* Interrupt Mask on line 2*/
#define EXTI_IMR_MR3                        (0x1U << 3U)                       /* Interrupt Mask on line 3*/
#define EXTI_IMR_MR4                        (0x1U << 4U)                       /* Interrupt Mask on line 4*/
#define EXTI_IMR_MR5                        (0x1U << 5U)                       /* Interrupt Mask on line 5*/
#define EXTI_IMR_MR6                        (0x1U << 6U)                       /* Interrupt Mask on line 6*/
#define EXTI_IMR_MR7                        (0x1U << 7U)                       /* Interrupt Mask on line 7*/
#define EXTI_IMR_MR8                        (0x1U << 8U)                       /* Interrupt Mask on line 8*/
#define EXTI_IMR_MR9                        (0x1U << 9U)                       /* Interrupt Mask on line 9*/
#define EXTI_IMR_MR10                       (0x1U << 10U)                      /* Interrupt Mask on line 10*/
#define EXTI_IMR_MR11                       (0x1U << 11U)                      /* Interrupt Mask on line 11*/
#define EXTI_IMR_MR12                       (0x1U << 12U)                      /* Interrupt Mask on line 12*/
#define EXTI_IMR_MR13                       (0x1U << 13U)                      /* Interrupt Mask on line 13*/
#define EXTI_IMR_MR14                       (0x1U << 14U)                      /* Interrupt Mask on line 14*/
#define EXTI_IMR_MR15                       (0x1U << 15U)                      /* Interrupt Mask on line 15*/
#define EXTI_IMR_MR16                       (0x1U << 16U)                      /* Interrupt Mask on line 16*/
#define EXTI_IMR_MR17                       (0x1U << 17U)                      /* Interrupt Mask on line 17*/
#define EXTI_IMR_MR18                       (0x1U << 18U)                      /* Interrupt Mask on line 18*/

#define EXTI_EMR_MR0                        (0x1U << 0U)                       /* Event Mask on line 0*/
#define EXTI_EMR_MR1                        (0x1U << 1U)                       /* Event Mask on line 1*/
#define EXTI_EMR_MR2                        (0x1U << 2U)                       /* Event Mask on line 2*/
#define EXTI_EMR_MR3                        (0x1U << 3U)                       /* Event Mask on line 3*/
#define EXTI_EMR_MR4                        (0x1U << 4U)                       /* Event Mask on line 4*/
#define EXTI_EMR_MR5                        (0x1U << 5U)                       /* Event Mask on line 5*/
#define EXTI_EMR_MR6                        (0x1U << 6U)                       /* Event Mask on line 6*/
#define EXTI_EMR_MR7                        (0x1U << 7U)                       /* Event Mask on line 7*/
#define EXTI_EMR_MR8                        (0x1U << 8U)                       /* Event Mask on line 8*/
#define EXTI_EMR_MR9                        (0x1U << 9U)                       /* Event Mask on line 9*/
#define EXTI_EMR_MR10                       (0x1U << 10U)                      /* Event Mask on line 10*/
#define EXTI_EMR_MR11                       (0x1U << 11U)                      /* Event Mask on line 11*/
#define EXTI_EMR_MR12                       (0x1U << 12U)                      /* Event Mask on line 12*/
#define EXTI_EMR_MR13                       (0x1U << 13U)                      /* Event Mask on line 13*/
#define EXTI_EMR_MR14                       (0x1U << 14U)                      /* Event Mask on line 14*/
#define EXTI_EMR_MR15                       (0x1U << 15U)                      /* Event Mask on line 15*/
#define EXTI_EMR_MR16                       (0x1U << 16U)                      /* Event Mask on line 16*/
#define EXTI_EMR_MR17                       (0x1U << 17U)                      /* Event Mask on line 17*/
#define EXTI_EMR_MR18                       (0x1U << 18U)                      /* Event Mask on line 18*/

#define EXTI_RTSR_TR0                       (0x1U << 0U)                       /* Rising trigger event configuration bit of line 0*/
#define EXTI_RTSR_TR1                       (0x1U << 1U)                       /* Rising trigger event configuration bit of line 1*/
#define EXTI_RTSR_TR2                       (0x1U << 2U)                       /* Rising trigger event configuration bit of line 2*/
#define EXTI_RTSR_TR3                       (0x1U << 3U)                       /* Rising trigger event configuration bit of line 3*/
#define EXTI_RTSR_TR4                       (0x1U << 4U)                       /* Rising trigger event configuration bit of line 4*/
#define EXTI_RTSR_TR5                       (0x1U << 5U)                       /* Rising trigger event configuration bit of line 5*/
#define EXTI_RTSR_TR6                       (0x1U << 6U)                       /* Rising trigger event configuration bit of line 6*/
#define EXTI_RTSR_TR7                       (0x1U << 7U)                       /* Rising trigger event configuration bit of line 7*/
#define EXTI_RTSR_TR8                       (0x1U << 8U)                       /* Rising trigger event configuration bit of line 8*/
#define EXTI_RTSR_TR9                       (0x1U << 9U)                       /* Rising trigger event configuration bit of line 9*/
#define EXTI_RTSR_TR10                      (0x1U << 10U)                      /* Rising trigger event configuration bit of line 10*/
#define EXTI_RTSR_TR11                      (0x1U << 11U)                      /* Rising trigger event configuration bit of line 11*/
#define EXTI_RTSR_TR12                      (0x1U << 12U)                      /* Rising trigger event configuration bit of line 12*/
#define EXTI_RTSR_TR13                      (0x1U << 13U)                      /* Rising trigger event configuration bit of line 13*/
#define EXTI_RTSR_TR14                      (0x1U << 14U)                      /* Rising trigger event configuration bit of line 14*/
#define EXTI_RTSR_TR15                      (0x1U << 15U)                      /* Rising trigger event configuration bit of line 15*/
#define EXTI_RTSR_TR16                      (0x1U << 16U)                      /* Rising trigger event configuration bit of line 16*/
#define EXTI_RTSR_TR17                      (0x1U << 17U)                      /* Rising trigger event configuration bit of line 17*/
#define EXTI_RTSR_TR18                      (0x1U << 18U)                      /* Rising trigger event configuration bit of line 18*/

#define EXTI_FTSR_TR0                       (0x1U << 0U)                       /* Falling trigger event configuration bit of line 0*/
#define EXTI_FTSR_TR1                       (0x1U << 1U)                       /* Falling trigger event configuration bit of line 1*/
#define EXTI_FTSR_TR2                       (0x1U << 2U)                       /* Falling trigger event configuration bit of line 2*/
#define EXTI_FTSR_TR3                       (0x1U << 3U)                       /* Falling trigger event configuration bit of line 3*/
#define EXTI_FTSR_TR4                       (0x1U << 4U)                       /* Falling trigger event configuration bit of line 4*/
#define EXTI_FTSR_TR5                       (0x1U << 5U)                       /* Falling trigger event configuration bit of line 5*/
#define EXTI_FTSR_TR6                       (0x1U << 6U)                       /* Falling trigger event configuration bit of line 6*/
#define EXTI_FTSR_TR7                       (0x1U << 7U)                       /* Falling trigger event configuration bit of line 7*/
#define EXTI_FTSR_TR8                       (0x1U << 8U)                       /* Falling trigger event configuration bit of line 8*/
#define EXTI_FTSR_TR9                       (0x1U << 9U)                       /* Falling trigger event configuration bit of line 9*/
#define EXTI_FTSR_TR10                      (0x1U << 10U)                      /* Falling trigger event configuration bit of line 10*/
#define EXTI_FTSR_TR11                      (0x1U << 11U)                      /* Falling trigger event configuration bit of line 11*/
#define EXTI_FTSR_TR12                      (0x1U << 12U)                      /* Falling trigger event configuration bit of line 12*/
#define EXTI_FTSR_TR13                      (0x1U << 13U)                      /* Falling trigger event configuration bit of line 13*/
#define EXTI_FTSR_TR14                      (0x1U << 14U)                      /* Falling trigger event configuration bit of line 14*/
#define EXTI_FTSR_TR15                      (0x1U << 15U)                      /* Falling trigger event configuration bit of line 15*/
#define EXTI_FTSR_TR16                      (0x1U << 16U)                      /* Falling trigger event configuration bit of line 16*/
#define EXTI_FTSR_TR17                      (0x1U << 17U)                      /* Falling trigger event configuration bit of line 17*/
#define EXTI_FTSR_TR18                      (0x1U << 18U)                      /* Falling trigger event configuration bit of line 18*/

#define EXTI_SWIER_SWIER0                   (0x1U << 0U)                       /* Software Interrupt on line 0*/
#define EXTI_SWIER_SWIER1                   (0x1U << 1U)                       /* Software Interrupt on line 1*/
#define EXTI_SWIER_SWIER2                   (0x1U << 2U)                       /* Software Interrupt on line 2*/
#define EXTI_SWIER_SWIER3                   (0x1U << 3U)                       /* Software Interrupt on line 3*/
#define EXTI_SWIER_SWIER4                   (0x1U << 4U)                       /* Software Interrupt on line 4*/
#define EXTI_SWIER_SWIER5                   (0x1U << 5U)                       /* Software Interrupt on line 5*/
#define EXTI_SWIER_SWIER6                   (0x1U << 6U)                       /* Software Interrupt on line 6*/
#define EXTI_SWIER_SWIER7                   (0x1U << 7U)                       /* Software Interrupt on line 7*/
#define EXTI_SWIER_SWIER8                   (0x1U << 8U)                       /* Software Interrupt on line 8*/
#define EXTI_SWIER_SWIER9                   (0x1U << 9U)                       /* Software Interrupt on line 9*/
#define EXTI_SWIER_SWIER10                  (0x1U << 10U)                      /* Software Interrupt on line 10*/
#define EXTI_SWIER_SWIER11                  (0x1U << 11U)                      /* Software Interrupt on line 11*/
#define EXTI_SWIER_SWIER12                  (0x1U << 12U)                      /* Software Interrupt on line 12*/
#define EXTI_SWIER_SWIER13                  (0x1U << 13U)                      /* Software Interrupt on line 13*/
#define EXTI_SWIER_SWIER14                  (0x1U << 14U)                      /* Software Interrupt on line 14*/
#define EXTI_SWIER_SWIER15                  (0x1U << 15U)                      /* Software Interrupt on line 15*/
#define EXTI_SWIER_SWIER16                  (0x1U << 16U)                      /* Software Interrupt on line 16*/
#define EXTI_SWIER_SWIER17                  (0x1U << 17U)                      /* Software Interrupt on line 17*/
#define EXTI_SWIER_SWIER18                  (0x1U << 18U)                      /* Software Interrupt on line 18*/

#define EXTI_PR_PR0                         (0x1U << 0U)                       /* Pending bit for line 0*/
#define EXTI_PR_PR1                         (0x1U << 1U)                       /* Pending bit for line 1*/
#define EXTI_PR_PR2                         (0x1U << 2U)                       /* Pending bit for line 2*/
#define EXTI_PR_PR3                         (0x1U << 3U)                       /* Pending bit for line 3*/
#define EXTI_PR_PR4                         (0x1U << 4U)                       /* Pending bit for line 4*/
#define EXTI_PR_PR5                         (0x1U << 5U)                       /* Pending bit for line 5*/
#define EXTI_PR_PR6                         (0x1U << 6U)                       /* Pending bit for line 6*/
#define EXTI_PR_PR7                         (0x1U << 7U)                       /* Pending bit for line 7*/
#define EXTI_PR_PR8                         (0x1U << 8U)                       /* Pending bit for line 8*/
#define EXTI_PR_PR9                         (0x1U << 9U)                       /* Pending bit for line 9*/
#define EXTI_PR_PR10                        (0x1U << 10U)                      /* Pending bit for line 10*/
#define EXTI_PR_PR11                        (0x1U << 11U)                      /* Pending bit for line 11*/
#define EXTI_PR_PR12                        (0x1U << 12U)                      /* Pending bit for line 12*/
#define EXTI_PR_PR13                        (0x1U << 13U)                      /* Pending bit for line 13*/
#define EXTI_PR_PR14                        (0x1U << 14U)                      /* Pending bit for line 14*/
#define EXTI_PR_PR15                        (0x1U << 15U)                      /* Pending bit for line 15*/
#define EXTI_PR_PR16                        (0x1U << 16U)                      /* Pending bit for line 16*/
#define EXTI_PR_PR17                        (0x1U << 17U)                      /* Pending bit for line 17*/
#define EXTI_PR_PR18                        (0x1U << 18U)                      /* Pending bit for line 18*/


#define DMA_ISR_GIF1                        (0x1U << 0U)                       /* Channel 1 Global interrupt flag*/
#define DMA_ISR_TCIF1                       (0x1U << 1U)                       /* Channel 1 Transfer Complete flag*/
#define DMA_ISR_HTIF1                       (0x1U << 2U)                       /* Channel 1 Half Transfer flag*/
#define DMA_ISR_TEIF1                       (0x1U << 3U)                       /* Channel 1 Transfer Error flag*/
#define DMA_ISR_GIF2                        (0x1U << 4U)                       /* Channel 2 Global interrupt flag*/
#define DMA_ISR_TCIF2                       (0x1U << 5U)                       /* Channel 2 Transfer Complete flag*/
#define DMA_ISR_HTIF2                       (0x1U << 6U)                       /* Channel 2 Half Transfer flag*/
#define DMA_ISR_TEIF2                       (0x1U << 7U)                       /* Channel 2 Transfer Error flag*/
#define DMA_ISR_GIF3                        (0x1U << 8U)                       /* Channel 3 Global interrupt flag*/
#define DMA_ISR_TCIF3                       (0x1U << 9U)                       /* Channel 3 Transfer Complete flag*/
#define DMA_ISR_HTIF3                       (0x1U << 10U)                      /* Channel 3 Half Transfer flag*/
#define DMA_ISR_TEIF3                       (0x1U << 11U)                      /* Channel 3 Transfer Error flag*/
#define DMA_ISR_GIF4                        (0x1U << 12U)                      /* Channel 4 Global interrupt flag*/
#define DMA_ISR_TCIF4                       (0x1U << 13U)                      /* Channel 4 Transfer Complete flag*/
#define DMA_ISR_HTIF4                       (0x1U << 14U)                      /* Channel 4 Half Transfer flag*/
#define DMA_ISR_TEIF4                       (0x1U << 15U)                      /* Channel 4 Transfer Error flag*/
#define DMA_ISR_GIF5                        (0x1U << 16U)                      /* Channel 5 Global interrupt flag*/
#define DMA_ISR_TCIF5                       (0x1U << 17U)                      /* Channel 5 Transfer Complete flag*/
#define DMA_ISR_HTIF5                       (0x1U << 18U)                      /* Channel 5 Half Transfer flag*/
#define DMA_ISR_TEIF5                       (0x1U << 19U)                      /* Channel 5 Transfer Error flag*/
#define DMA_ISR_GIF6                        (0x1U << 20U)                      /* Channel 6 Global interrupt flag*/
#define DMA_ISR_TCIF6                       (0x1U << 21U)                      /* Channel 6 Transfer Complete flag*/
#define DMA_ISR_HTIF6                       (0x1U << 22U)                      /* Channel 6 Half Transfer flag*/
#define DMA_ISR_TEIF6                       (0x1U << 23U)                      /* Channel 6 Transfer Error flag*/
#define DMA_ISR_GIF7                        (0x1U << 24U)                      /* Channel 7 Global interrupt flag*/
#define DMA_ISR_TCIF7                       (0x1U << 25U)                      /* Channel 7 Transfer Complete flag*/
#define DMA_ISR_HTIF7                       (0x1U << 26U)                      /* Channel 7 Half Transfer flag*/
#define DMA_ISR_TEIF7                       (0x1U << 27U)                      /* Channel 7 Transfer Error flag*/

#define DMA_IFCR_CGIF1                      (0x1U << 0U)                       /* Channel 1 Global interrupt clear*/
#define DMA_IFCR_CTCIF1                     (0x1U << 1U)                       /* Channel 1 Transfer Complete clear*/
#define DMA_IFCR_CHTIF1                     (0x1U << 2U)                       /* Channel 1 Half Transfer clear*/
#define DMA_IFCR_CTEIF1                     (0x1U << 3U)                       /* Channel 1 Transfer Error clear*/
#define DMA_IFCR_CGIF2                      (0x1U << 4U)                       /* Channel 2 Global interrupt clear*/
#define DMA_IFCR_CTCIF2                     (0x1U << 5U)                       /* Channel 2 Transfer Complete clear*/
#define DMA_IFCR_CHTIF2                     (0x1U << 6U)                       /* Channel 2 Half Transfer clear*/
#define DMA_IFCR_CTEIF2                     (0x1U << 7U)                       /* Channel 2 Transfer Error clear*/
#define DMA_IFCR_CGIF3                      (0x1U << 8U)                       /* Channel 3 Global interrupt clear*/
#define DMA_IFCR_CTCIF3                     (0x1U << 9U)                       /* Channel 3 Transfer Complete clear*/
#define DMA_IFCR_CHTIF3                     (0x1U << 10U)                      /* Channel 3 Half Transfer clear*/
#define DMA_IFCR_CTEIF3                     (0x1U << 11U)                      /* Channel 3 Transfer Error clear*/
#define DMA_IFCR_CGIF4                      (0x1U << 12U)                      /* Channel 4 Global interrupt clear*/
#define DMA_IFCR_CTCIF4                     (0x1U << 13U)                      /* Channel 4 Transfer Complete clear*/
#define DMA_IFCR_CHTIF4                     (0x1U << 14U)                      /* Channel 4 Half Transfer clear*/
#define DMA_IFCR_CTEIF4                     (0x1U << 15U)                      /* Channel 4 Transfer Error clear*/
#define DMA_IFCR_CGIF5                      (0x1U << 16U)                      /* Channel 5 Global interrupt clear*/
#define DMA_IFCR_CTCIF5                     (0x1U << 17U)                      /* Channel 5 Transfer Complete clear*/
#define DMA_IFCR_CHTIF5                     (0x1U << 18U)                      /* Channel 5 Half Transfer clear*/
#define DMA_IFCR_CTEIF5                     (0x1U << 19U)                      /* Channel 5 Transfer Error clear*/
#define DMA_IFCR_CGIF6                      (0x1U << 20U)                      /* Channel 6 Global interrupt clear*/
#define DMA_IFCR_CTCIF6                     (0x1U << 21U)                      /* Channel 6 Transfer Complete clear*/
#define DMA_IFCR_CHTIF6                     (0x1U << 22U)                      /* Channel 6 Half Transfer clear*/
#define DMA_IFCR_CTEIF6                     (0x1U << 23U)                      /* Channel 6 Transfer Error clear*/
#define DMA_IFCR_CGIF7                      (0x1U << 24U)                      /* Channel 7 Global interrupt clear*/
#define DMA_IFCR_CTCIF7                     (0x1U << 25U)                      /* Channel 7 Transfer Complete clear*/
#define DMA_IFCR_CHTIF7                     (0x1U << 26U)                      /* Channel 7 Half Transfer clear*/
#define DMA_IFCR_CTEIF7                     (0x1U << 27U)                      /* Channel 7 Transfer Error clear*/

#define DMA_CCR_EN                          (0x1U << 0U)                       /* Channel enable*/
#define DMA_CCR_TCIE                        (0x1U << 1U)                       /* Transfer complete interrupt enable*/
#define DMA_CCR_HTIE                        (0x1U << 2U)                       /* Half Transfer interrupt enable*/
#define DMA_CCR_TEIE                        (0x1U << 3U)                       /* Transfer error interrupt enable*/
#define DMA_CCR_DIR                         (0x1U << 4U)                       /* Data transfer direction*/
#define DMA_CCR_CIRC                        (0x1U << 5U)                       /* Circular mode*/
#define DMA_CCR_PINC                        (0x1U << 6U)                       /* Peripheral increment mode*/
#define DMA_CCR_MINC                        (0x1U << 7U)                       /* Memory increment mode*/
#define DMA_CCR_PSIZE                       (0x3U << 8U)                       /* PSIZE[1:0] bits (Peripheral size)*/
#define DMA_CCR_PSIZE_0                     (0x1U << 8U)
#define DMA_CCR_PSIZE_1                     (0x2U << 8U)
#define DMA_CCR_MSIZE                       (0x3U << 10U)                      /* MSIZE[1:0] bits (Memory size)*/
#define DMA_CCR_MSIZE_0                     (0x1U << 10U)
#define DMA_CCR_MSIZE_1                     (0x2U << 10U)
#define DMA_CCR_PL                          (0x3U << 12U)                      /* PL[1:0] bits(Channel Priority level)*/
#define DMA_CCR_PL_0                        (0x1U << 12U)
#define DMA_CCR_PL_1                        (0x2U << 12U)
#define DMA_CCR_MEM2MEM                     (0x1U << 14U)                      /* Memory to memory mode*/

#define DMA_CNDTR_NDT                       (0xFFFFU << 0U)                    /* Number of data to Transfer*/

#define DMA_CPAR_PA                         (0xFFFFFFFFU << 0U)                /* Peripheral Address*/

#define DMA_CMAR_MA                         (0xFFFFFFFFU << 0U)                /* Memory Address*/


#define ADC_SR_AWD                          (0x1U << 0U)                       /* ADC analog watchdog 1 flag*/
#define ADC_SR_EOS                          (0x1U << 1U)                       /* ADC group regular end of sequence conversions flag*/
#define ADC_SR_JEOS                         (0x1U << 2U)                       /* ADC group injected end of sequence conversions flag*/
#define ADC_SR_JSTRT                        (0x1U << 3U)                       /* ADC group injected conversion start flag*/
#define ADC_SR_STRT                         (0x1U << 4U)                       /* ADC group regular conversion start flag*/

#define ADC_CR1_AWDCH                       (0x1FU << 0U)                      /* ADC analog watchdog 1 monitored channel selection*/
#define ADC_CR1_AWDCH_0                     (0x01U << 0U)
#define ADC_CR1_AWDCH_1                     (0x02U << 0U)
#define ADC_CR1_AWDCH_2                     (0x04U << 0U)
#define ADC_CR1_AWDCH_3                     (0x08U << 0U)
#define ADC_CR1_AWDCH_4                     (0x10U << 0U)
#define ADC_CR1_EOSIE                       (0x1U << 5U)                       /* ADC group regular end of sequence conversions interrupt*/
#define ADC_CR1_AWDIE                       (0x1U << 6U)                       /* ADC analog watchdog 1 interrupt*/
#define ADC_CR1_JEOSIE                      (0x1U << 7U)                       /* ADC group injected end of sequence conversions interrupt*/
#define ADC_CR1_SCAN                        (0x1U << 8U)                       /* ADC scan mode*/
#define ADC_CR1_AWDSGL                      (0x1U << 9U)                       /* ADC analog watchdog 1 monitoring a single channel or all channels*/
#define ADC_CR1_JAUTO                       (0x1U << 10U)                      /* ADC group injected automatic trigger mode*/
#define ADC_CR1_DISCEN                      (0x1U << 11U)                      /* ADC group regular sequencer discontinuous mode*/
#define ADC_CR1_JDISCEN                     (0x1U << 12U)                      /* ADC group injected sequencer discontinuous mode*/
#define ADC_CR1_DISCNUM                     (0x7U << 13U)                      /* ADC group regular sequencer discontinuous number of ranks*/
#define ADC_CR1_DISCNUM_0                   (0x1U << 13U)
#define ADC_CR1_DISCNUM_1                   (0x2U << 13U)
#define ADC_CR1_DISCNUM_2                   (0x4U << 13U)
#define ADC_CR1_DUALMOD                     (0xFU << 16U)                      /* ADC multimode mode selection*/
#define ADC_CR1_DUALMOD_0                   (0x1U << 16U)
#define ADC_CR1_DUALMOD_1                   (0x2U << 16U)
#define ADC_CR1_DUALMOD_2                   (0x4U << 16U)
#define ADC_CR1_DUALMOD_3                   (0x8U << 16U)
#define ADC_CR1_JAWDEN                      (0x1U << 22U)                      /* ADC analog watchdog 1 enable on scope ADC group injected*/
#define ADC_CR1_AWDEN                       (0x1U << 23U)                      /* ADC analog watchdog 1 enable on scope ADC group regular*/

#define ADC_CR2_ADON                        (0x1U << 0U)                       /* ADC enable*/
#define ADC_CR2_CONT                        (0x1U << 1U)                       /* ADC group regular continuous conversion mode*/
#define ADC_CR2_CAL                         (0x1U << 2U)                       /* ADC calibration start*/
#define ADC_CR2_RSTCAL                      (0x1U << 3U)                       /* ADC calibration reset*/
#define ADC_CR2_DMA                         (0x1U << 8U)                       /* ADC DMA transfer enable*/
#define ADC_CR2_ALIGN                       (0x1U << 11U)                      /* ADC data alignement*/
#define ADC_CR2_JEXTSEL                     (0x7U << 12U)                      /* ADC group injected external trigger source*/
#define ADC_CR2_JEXTSEL_0                   (0x1U << 12U)
#define ADC_CR2_JEXTSEL_1                   (0x2U << 12U)
#define ADC_CR2_JEXTSEL_2                   (0x4U << 12U)
#define ADC_CR2_JEXTTRIG                    (0x1U << 15U)                      /* ADC group injected external trigger enable*/
#define ADC_CR2_EXTSEL                      (0x7U << 17U)                      /* ADC group regular external trigger source*/
#define ADC_CR2_EXTSEL_0                    (0x1U << 17U)
#define ADC_CR2_EXTSEL_1                    (0x2U << 17U)
#define ADC_CR2_EXTSEL_2                    (0x4U << 17U)
#define ADC_CR2_EXTTRIG                     (0x1U << 20U)                      /* ADC group regular external trigger enable*/
#define ADC_CR2_JSWSTART                    (0x1U << 21U)                      /* ADC group injected conversion start*/
#define ADC_CR2_SWSTART                     (0x1U << 22U)                      /* ADC group regular conversion start*/
#define ADC_CR2_TSVREFE                     (0x1U << 23U)                      /* ADC internal path to VrefInt and temperature sensor enable*/

#define ADC_SMPR1_SMP10                     (0x7U << 0U)                       /* ADC channel 10 sampling time selection*/
#define ADC_SMPR1_SMP10_0                   (0x1U << 0U)
#define ADC_SMPR1_SMP10_1                   (0x2U << 0U)
#define ADC_SMPR1_SMP10_2                   (0x4U << 0U)
#define ADC_SMPR1_SMP11                     (0x7U << 3U)                       /* ADC channel 11 sampling time selection*/
#define ADC_SMPR1_SMP11_0                   (0x1U << 3U)
#define ADC_SMPR1_SMP11_1                   (0x2U << 3U)
#define ADC_SMPR1_SMP11_2                   (0x4U << 3U)
#define ADC_SMPR1_SMP12                     (0x7U << 6U)                       /* ADC channel 12 sampling time selection*/
#define ADC_SMPR1_SMP12_0                   (0x1U << 6U)
#define ADC_SMPR1_SMP12_1                   (0x2U << 6U)
#define ADC_SMPR1_SMP12_2                   (0x4U << 6U)
#define ADC_SMPR1_SMP13                     (0x7U << 9U)                       /* ADC channel 13 sampling time selection*/
#define ADC_SMPR1_SMP13_0                   (0x1U << 9U)
#define ADC_SMPR1_SMP13_1                   (0x2U << 9U)
#define ADC_SMPR1_SMP13_2                   (0x4U << 9U)
#define ADC_SMPR1_SMP14                     (0x7U << 12U)                      /* ADC channel 14 sampling time selection*/
#define ADC_SMPR1_SMP14_0                   (0x1U << 12U)
#define ADC_SMPR1_SMP14_1                   (0x2U << 12U)
#define ADC_SMPR1_SMP14_2                   (0x4U << 12U)
#define ADC_SMPR1_SMP15                     (0x7U << 15U)                      /* ADC channel 15 sampling time selection*/
#define ADC_SMPR1_SMP15_0                   (0x1U << 15U)
#define ADC_SMPR1_SMP15_1                   (0x2U << 15U)
#define ADC_SMPR1_SMP15_2                   (0x4U << 15U)
#define ADC_SMPR1_SMP16                     (0x7U << 18U)                      /* ADC channel 16 sampling time selection*/
#define ADC_SMPR1_SMP16_0                   (0x1U << 18U)
#define ADC_SMPR1_SMP16_1                   (0x2U << 18U)
#define ADC_SMPR1_SMP16_2                   (0x4U << 18U)
#define ADC_SMPR1_SMP17                     (0x7U << 21U)                      /* ADC channel 17 sampling time selection*/
#define ADC_SMPR1_SMP17_0                   (0x1U << 21U)
#define ADC_SMPR1_SMP17_1                   (0x2U << 21U)
#define ADC_SMPR1_SMP17_2                   (0x4U << 21U)

#define ADC_SMPR2_SMP0                      (0x7U << 0U)                       /* ADC channel 0 sampling time selection*/
#define ADC_SMPR2_SMP0_0                    (0x1U << 0U)
#define ADC_SMPR2_SMP0_1                    (0x2U << 0U)
#define ADC_SMPR2_SMP0_2                    (0x4U << 0U)
#define ADC_SMPR2_SMP1                      (0x7U << 3U)                       /* ADC channel 1 sampling time selection*/
#define ADC_SMPR2_SMP1_0                    (0x1U << 3U)
#define ADC_SMPR2_SMP1_1                    (0x2U << 3U)
#define ADC_SMPR2_SMP1_2                    (0x4U << 3U)
#define ADC_SMPR2_SMP2                      (0x7U << 6U)                       /* ADC channel 2 sampling time selection*/
#define ADC_SMPR2_SMP2_0                    (0x1U << 6U)
#define ADC_SMPR2_SMP2_1                    (0x2U << 6U)
#define ADC_SMPR2_SMP2_2                    (0x4U << 6U)
#define ADC_SMPR2_SMP3                      (0x7U << 9U)                       /* ADC channel 3 sampling time selection*/
#define ADC_SMPR2_SMP3_0                    (0x1U << 9U)
#define ADC_SMPR2_SMP3_1                    (0x2U << 9U)
#define ADC_SMPR2_SMP3_2                    (0x4U << 9U)
#define ADC_SMPR2_SMP4                      (0x7U << 12U)                      /* ADC channel 4 sampling time selection*/
#define ADC_SMPR2_SMP4_0                    (0x1U << 12U)
#define ADC_SMPR2_SMP4_1                    (0x2U << 12U)
#define ADC_SMPR2_SMP4_2                    (0x4U << 12U)
#define ADC_SMPR2_SMP5                      (0x7U << 15U)                      /* ADC channel 5 sampling time selection*/
#define ADC_SMPR2_SMP5_0                    (0x1U << 15U)
#define ADC_SMPR2_SMP5_1                    (0x2U << 15U)
#define ADC_SMPR2_SMP5_2                    (0x4U << 15U)
#define ADC_SMPR2_SMP6                      (0x7U << 18U)                      /* ADC channel 6 sampling time selection*/
#define ADC_SMPR2_SMP6_0                    (0x1U << 18U)
#define ADC_SMPR2_SMP6_1                    (0x2U << 18U)
#define ADC_SMPR2_SMP6_2                    (0x4U << 18U)
#define ADC_SMPR2_SMP7                      (0x7U << 21U)                      /* ADC channel 7 sampling time selection*/
#define ADC_SMPR2_SMP7_0                    (0x1U << 21U)
#define ADC_SMPR2_SMP7_1                    (0x2U << 21U)
#define ADC_SMPR2_SMP7_2                    (0x4U << 21U)
#define ADC_SMPR2_SMP8                      (0x7U << 24U)                      /* ADC channel 8 sampling time selection*/
#define ADC_SMPR2_SMP8_0                    (0x1U << 24U)
#define ADC_SMPR2_SMP8_1                    (0x2U << 24U)
#define ADC_SMPR2_SMP8_2                    (0x4U << 24U)
#define ADC_SMPR2_SMP9                      (0x7U << 27U)                      /* ADC channel 9 sampling time selection*/
#define ADC_SMPR2_SMP9_0                    (0x1U << 27U)
#define ADC_SMPR2_SMP9_1                    (0x2U << 27U)
#define ADC_SMPR2_SMP9_2                    (0x4U << 27U)

#define ADC_JOFR1_JOFFSET1                  (0xFFFU << 0U)                     /* ADC group injected sequencer rank 1 offset value*/

#define ADC_JOFR2_JOFFSET2                  (0xFFFU << 0U)                     /* ADC group injected sequencer rank 2 offset value*/

#define ADC_JOFR3_JOFFSET3                  (0xFFFU << 0U)                     /* ADC group injected sequencer rank 3 offset value*/

#define ADC_JOFR4_JOFFSET4                  (0xFFFU << 0U)                     /* ADC group injected sequencer rank 4 offset value*/

#define ADC_HTR_HT                          (0xFFFU << 0U)                     /* ADC analog watchdog 1 threshold high*/

#define ADC_LTR_LT                          (0xFFFU << 0U)                     /* ADC analog watchdog 1 threshold low*/

#define ADC_SQR1_SQ13                       (0x1FU << 0U)                      /* ADC group regular sequencer rank 13*/
#define ADC_SQR1_SQ13_0                     (0x01U << 0U)
#define ADC_SQR1_SQ13_1                     (0x02U << 0U)
#define ADC_SQR1_SQ13_2                     (0x04U << 0U)
#define ADC_SQR1_SQ13_3                     (0x08U << 0U)
#define ADC_SQR1_SQ13_4                     (0x10U << 0U)
#define ADC_SQR1_SQ14                       (0x1FU << 5U)                      /* ADC group regular sequencer rank 14*/
#define ADC_SQR1_SQ14_0                     (0x01U << 5U)
#define ADC_SQR1_SQ14_1                     (0x02U << 5U)
#define ADC_SQR1_SQ14_2                     (0x04U << 5U)
#define ADC_SQR1_SQ14_3                     (0x08U << 5U)
#define ADC_SQR1_SQ14_4                     (0x10U << 5U)
#define ADC_SQR1_SQ15                       (0x1FU << 10U)                     /* ADC group regular sequencer rank 15*/
#define ADC_SQR1_SQ15_0                     (0x01U << 10U)
#define ADC_SQR1_SQ15_1                     (0x02U << 10U)
#define ADC_SQR1_SQ15_2                     (0x04U << 10U)
#define ADC_SQR1_SQ15_3                     (0x08U << 10U)
#define ADC_SQR1_SQ15_4                     (0x10U << 10U)
#define ADC_SQR1_SQ16                       (0x1FU << 15U)                     /* ADC group regular sequencer rank 16*/
#define ADC_SQR1_SQ16_0                     (0x01U << 15U)
#define ADC_SQR1_SQ16_1                     (0x02U << 15U)
#define ADC_SQR1_SQ16_2                     (0x04U << 15U)
#define ADC_SQR1_SQ16_3                     (0x08U << 15U)
#define ADC_SQR1_SQ16_4                     (0x10U << 15U)
#define ADC_SQR1_L                          (0xFU << 20U)                      /* ADC group regular sequencer scan length*/
#define ADC_SQR1_L_0                        (0x1U << 20U)
#define ADC_SQR1_L_1                        (0x2U << 20U)
#define ADC_SQR1_L_2                        (0x4U << 20U)
#define ADC_SQR1_L_3                        (0x8U << 20U)

#define ADC_SQR2_SQ7                        (0x1FU << 0U)                      /* ADC group regular sequencer rank 7*/
#define ADC_SQR2_SQ7_0                      (0x01U << 0U)
#define ADC_SQR2_SQ7_1                      (0x02U << 0U)
#define ADC_SQR2_SQ7_2                      (0x04U << 0U)
#define ADC_SQR2_SQ7_3                      (0x08U << 0U)
#define ADC_SQR2_SQ7_4                      (0x10U << 0U)
#define ADC_SQR2_SQ8                        (0x1FU << 5U)                      /* ADC group regular sequencer rank 8*/
#define ADC_SQR2_SQ8_0                      (0x01U << 5U)
#define ADC_SQR2_SQ8_1                      (0x02U << 5U)
#define ADC_SQR2_SQ8_2                      (0x04U << 5U)
#define ADC_SQR2_SQ8_3                      (0x08U << 5U)
#define ADC_SQR2_SQ8_4                      (0x10U << 5U)
#define ADC_SQR2_SQ9                        (0x1FU << 10U)                     /* ADC group regular sequencer rank 9*/
#define ADC_SQR2_SQ9_0                      (0x01U << 10U)
#define ADC_SQR2_SQ9_1                      (0x02U << 10U)
#define ADC_SQR2_SQ9_2                      (0x04U << 10U)
#define ADC_SQR2_SQ9_3                      (0x08U << 10U)
#define ADC_SQR2_SQ9_4                      (0x10U << 10U)
#define ADC_SQR2_SQ10                       (0x1FU << 15U)                     /* ADC group regular sequencer rank 10*/
#define ADC_SQR2_SQ10_0                     (0x01U << 15U)
#define ADC_SQR2_SQ10_1                     (0x02U << 15U)
#define ADC_SQR2_SQ10_2                     (0x04U << 15U)
#define ADC_SQR2_SQ10_3                     (0x08U << 15U)
#define ADC_SQR2_SQ10_4                     (0x10U << 15U)
#define ADC_SQR2_SQ11                       (0x1FU << 20U)                     /* ADC group regular sequencer rank 1*/
#define ADC_SQR2_SQ11_0                     (0x01U << 20U)
#define ADC_SQR2_SQ11_1                     (0x02U << 20U)
#define ADC_SQR2_SQ11_2                     (0x04U << 20U)
#define ADC_SQR2_SQ11_3                     (0x08U << 20U)
#define ADC_SQR2_SQ11_4                     (0x10U << 20U)
#define ADC_SQR2_SQ12                       (0x1FU << 25U)                     /* ADC group regular sequencer rank 12*/
#define ADC_SQR2_SQ12_0                     (0x01U << 25U)
#define ADC_SQR2_SQ12_1                     (0x02U << 25U)
#define ADC_SQR2_SQ12_2                     (0x04U << 25U)
#define ADC_SQR2_SQ12_3                     (0x08U << 25U)
#define ADC_SQR2_SQ12_4                     (0x10U << 25U)

#define ADC_SQR3_SQ1                        (0x1FU << 0U)                      /* ADC group regular sequencer rank 1*/
#define ADC_SQR3_SQ1_0                      (0x01U << 0U)
#define ADC_SQR3_SQ1_1                      (0x02U << 0U)
#define ADC_SQR3_SQ1_2                      (0x04U << 0U)
#define ADC_SQR3_SQ1_3                      (0x08U << 0U)
#define ADC_SQR3_SQ1_4                      (0x10U << 0U)
#define ADC_SQR3_SQ2                        (0x1FU << 5U)                      /* ADC group regular sequencer rank 2*/
#define ADC_SQR3_SQ2_0                      (0x01U << 5U)
#define ADC_SQR3_SQ2_1                      (0x02U << 5U)
#define ADC_SQR3_SQ2_2                      (0x04U << 5U)
#define ADC_SQR3_SQ2_3                      (0x08U << 5U)
#define ADC_SQR3_SQ2_4                      (0x10U << 5U)
#define ADC_SQR3_SQ3                        (0x1FU << 10U)                     /* ADC group regular sequencer rank 3*/
#define ADC_SQR3_SQ3_0                      (0x01U << 10U)
#define ADC_SQR3_SQ3_1                      (0x02U << 10U)
#define ADC_SQR3_SQ3_2                      (0x04U << 10U)
#define ADC_SQR3_SQ3_3                      (0x08U << 10U)
#define ADC_SQR3_SQ3_4                      (0x10U << 10U)
#define ADC_SQR3_SQ4                        (0x1FU << 15U)                     /* ADC group regular sequencer rank 4*/
#define ADC_SQR3_SQ4_0                      (0x01U << 15U)
#define ADC_SQR3_SQ4_1                      (0x02U << 15U)
#define ADC_SQR3_SQ4_2                      (0x04U << 15U)
#define ADC_SQR3_SQ4_3                      (0x08U << 15U)
#define ADC_SQR3_SQ4_4                      (0x10U << 15U)
#define ADC_SQR3_SQ5                        (0x1FU << 20U)                     /* ADC group regular sequencer rank 5*/
#define ADC_SQR3_SQ5_0                      (0x01U << 20U)
#define ADC_SQR3_SQ5_1                      (0x02U << 20U)
#define ADC_SQR3_SQ5_2                      (0x04U << 20U)
#define ADC_SQR3_SQ5_3                      (0x08U << 20U)
#define ADC_SQR3_SQ5_4                      (0x10U << 20U)
#define ADC_SQR3_SQ6                        (0x1FU << 25U)                     /* ADC group regular sequencer rank 6*/
#define ADC_SQR3_SQ6_0                      (0x01U << 25U)
#define ADC_SQR3_SQ6_1                      (0x02U << 25U)
#define ADC_SQR3_SQ6_2                      (0x04U << 25U)
#define ADC_SQR3_SQ6_3                      (0x08U << 25U)
#define ADC_SQR3_SQ6_4                      (0x10U << 25U)

#define ADC_JSQR_JSQ1                       (0x1FU << 0U)                      /* ADC group injected sequencer rank 1*/
#define ADC_JSQR_JSQ1_0                     (0x01U << 0U)
#define ADC_JSQR_JSQ1_1                     (0x02U << 0U)
#define ADC_JSQR_JSQ1_2                     (0x04U << 0U)
#define ADC_JSQR_JSQ1_3                     (0x08U << 0U)
#define ADC_JSQR_JSQ1_4                     (0x10U << 0U)
#define ADC_JSQR_JSQ2                       (0x1FU << 5U)                      /* ADC group injected sequencer rank 2*/
#define ADC_JSQR_JSQ2_0                     (0x01U << 5U)
#define ADC_JSQR_JSQ2_1                     (0x02U << 5U)
#define ADC_JSQR_JSQ2_2                     (0x04U << 5U)
#define ADC_JSQR_JSQ2_3                     (0x08U << 5U)
#define ADC_JSQR_JSQ2_4                     (0x10U << 5U)
#define ADC_JSQR_JSQ3                       (0x1FU << 10U)                     /* ADC group injected sequencer rank 3*/
#define ADC_JSQR_JSQ3_0                     (0x01U << 10)
#define ADC_JSQR_JSQ3_1                     (0x02U << 10)
#define ADC_JSQR_JSQ3_2                     (0x04U << 10)
#define ADC_JSQR_JSQ3_3                     (0x08U << 10)
#define ADC_JSQR_JSQ3_4                     (0x10U << 10)
#define ADC_JSQR_JSQ4                       (0x1FU << 15U)                     /* ADC group injected sequencer rank 4*/
#define ADC_JSQR_JSQ4_0                     (0x01U << 15)
#define ADC_JSQR_JSQ4_1                     (0x02U << 15)
#define ADC_JSQR_JSQ4_2                     (0x04U << 15)
#define ADC_JSQR_JSQ4_3                     (0x08U << 15)
#define ADC_JSQR_JSQ4_4                     (0x10U << 15)
#define ADC_JSQR_JL                         (0x3U << 20U)                      /* ADC group injected sequencer scan length*/
#define ADC_JSQR_JL_0                       (0x1U << 20U)
#define ADC_JSQR_JL_1                       (0x2U << 20U)

#define ADC_JDR1_JDATA                      (0xFFFFU << 0U)                    /* ADC group injected sequencer rank 1 conversion data*/

#define ADC_JDR2_JDATA                      (0xFFFFU << 0U)                    /* ADC group injected sequencer rank 2 conversion data*/

#define ADC_JDR3_JDATA                      (0xFFFFU << 0U)                    /* ADC group injected sequencer rank 3 conversion data*/

#define ADC_JDR4_JDATA                      (0xFFFFU << 0U)                    /* ADC group injected sequencer rank 4 conversion data*/

#define ADC_DR_DATA                         (0xFFFFU << 0U)                    /* ADC group regular conversion data*/
#define ADC_DR_ADC2DATA                     (0xFFFFU << 16U)                   /* ADC group regular conversion data for ADC slave, in multimode*/






/*****************************************************************************/
/*                                                                           */
/*                               Timers (TIM)                                */
/*                                                                           */
/*****************************************************************************/
/*******************  Bit definition for TIM_CR1 register  *******************/
#define TIM_CR1_CEN_Pos                     (0U)
#define TIM_CR1_CEN_Msk                     (0x1U << TIM_CR1_CEN_Pos)          /*!< 0x00000001 */
#define TIM_CR1_CEN                         TIM_CR1_CEN_Msk                    /*!<Counter enable */
#define TIM_CR1_UDIS_Pos                    (1U)
#define TIM_CR1_UDIS_Msk                    (0x1U << TIM_CR1_UDIS_Pos)         /*!< 0x00000002 */
#define TIM_CR1_UDIS                        TIM_CR1_UDIS_Msk                   /*!<Update disable */
#define TIM_CR1_URS_Pos                     (2U)
#define TIM_CR1_URS_Msk                     (0x1U << TIM_CR1_URS_Pos)          /*!< 0x00000004 */
#define TIM_CR1_URS                         TIM_CR1_URS_Msk                    /*!<Update request source */
#define TIM_CR1_OPM_Pos                     (3U)
#define TIM_CR1_OPM_Msk                     (0x1U << TIM_CR1_OPM_Pos)          /*!< 0x00000008 */
#define TIM_CR1_OPM                         TIM_CR1_OPM_Msk                    /*!<One pulse mode */
#define TIM_CR1_DIR_Pos                     (4U)
#define TIM_CR1_DIR_Msk                     (0x1U << TIM_CR1_DIR_Pos)          /*!< 0x00000010 */
#define TIM_CR1_DIR                         TIM_CR1_DIR_Msk                    /*!<Direction */

#define TIM_CR1_CMS_Pos                     (5U)
#define TIM_CR1_CMS_Msk                     (0x3U << TIM_CR1_CMS_Pos)          /*!< 0x00000060 */
#define TIM_CR1_CMS                         TIM_CR1_CMS_Msk                    /*!<CMS[1:0] bits (Center-aligned mode selection) */
#define TIM_CR1_CMS_0                       (0x1U << TIM_CR1_CMS_Pos)          /*!< 0x00000020 */
#define TIM_CR1_CMS_1                       (0x2U << TIM_CR1_CMS_Pos)          /*!< 0x00000040 */

#define TIM_CR1_ARPE_Pos                    (7U)
#define TIM_CR1_ARPE_Msk                    (0x1U << TIM_CR1_ARPE_Pos)         /*!< 0x00000080 */
#define TIM_CR1_ARPE                        TIM_CR1_ARPE_Msk                   /*!<Auto-reload preload enable */

#define TIM_CR1_CKD_Pos                     (8U)
#define TIM_CR1_CKD_Msk                     (0x3U << TIM_CR1_CKD_Pos)          /*!< 0x00000300 */
#define TIM_CR1_CKD                         TIM_CR1_CKD_Msk                    /*!<CKD[1:0] bits (clock division) */
#define TIM_CR1_CKD_0                       (0x1U << TIM_CR1_CKD_Pos)          /*!< 0x00000100 */
#define TIM_CR1_CKD_1                       (0x2U << TIM_CR1_CKD_Pos)          /*!< 0x00000200 */

/*******************  Bit definition for TIM_CR2 register  *******************/
#define TIM_CR2_CCPC_Pos                    (0U)
#define TIM_CR2_CCPC_Msk                    (0x1U << TIM_CR2_CCPC_Pos)         /*!< 0x00000001 */
#define TIM_CR2_CCPC                        TIM_CR2_CCPC_Msk                   /*!<Capture/Compare Preloaded Control */
#define TIM_CR2_CCUS_Pos                    (2U)
#define TIM_CR2_CCUS_Msk                    (0x1U << TIM_CR2_CCUS_Pos)         /*!< 0x00000004 */
#define TIM_CR2_CCUS                        TIM_CR2_CCUS_Msk                   /*!<Capture/Compare Control Update Selection */
#define TIM_CR2_CCDS_Pos                    (3U)
#define TIM_CR2_CCDS_Msk                    (0x1U << TIM_CR2_CCDS_Pos)         /*!< 0x00000008 */
#define TIM_CR2_CCDS                        TIM_CR2_CCDS_Msk                   /*!<Capture/Compare DMA Selection */

#define TIM_CR2_MMS_Pos                     (4U)
#define TIM_CR2_MMS_Msk                     (0x7U << TIM_CR2_MMS_Pos)          /*!< 0x00000070 */
#define TIM_CR2_MMS                         TIM_CR2_MMS_Msk                    /*!<MMS[2:0] bits (Master Mode Selection) */
#define TIM_CR2_MMS_0                       (0x1U << TIM_CR2_MMS_Pos)          /*!< 0x00000010 */
#define TIM_CR2_MMS_1                       (0x2U << TIM_CR2_MMS_Pos)          /*!< 0x00000020 */
#define TIM_CR2_MMS_2                       (0x4U << TIM_CR2_MMS_Pos)          /*!< 0x00000040 */

#define TIM_CR2_TI1S_Pos                    (7U)
#define TIM_CR2_TI1S_Msk                    (0x1U << TIM_CR2_TI1S_Pos)         /*!< 0x00000080 */
#define TIM_CR2_TI1S                        TIM_CR2_TI1S_Msk                   /*!<TI1 Selection */
#define TIM_CR2_OIS1_Pos                    (8U)
#define TIM_CR2_OIS1_Msk                    (0x1U << TIM_CR2_OIS1_Pos)         /*!< 0x00000100 */
#define TIM_CR2_OIS1                        TIM_CR2_OIS1_Msk                   /*!<Output Idle state 1 (OC1 output) */
#define TIM_CR2_OIS1N_Pos                   (9U)
#define TIM_CR2_OIS1N_Msk                   (0x1U << TIM_CR2_OIS1N_Pos)        /*!< 0x00000200 */
#define TIM_CR2_OIS1N                       TIM_CR2_OIS1N_Msk                  /*!<Output Idle state 1 (OC1N output) */
#define TIM_CR2_OIS2_Pos                    (10U)
#define TIM_CR2_OIS2_Msk                    (0x1U << TIM_CR2_OIS2_Pos)         /*!< 0x00000400 */
#define TIM_CR2_OIS2                        TIM_CR2_OIS2_Msk                   /*!<Output Idle state 2 (OC2 output) */
#define TIM_CR2_OIS2N_Pos                   (11U)
#define TIM_CR2_OIS2N_Msk                   (0x1U << TIM_CR2_OIS2N_Pos)        /*!< 0x00000800 */
#define TIM_CR2_OIS2N                       TIM_CR2_OIS2N_Msk                  /*!<Output Idle state 2 (OC2N output) */
#define TIM_CR2_OIS3_Pos                    (12U)
#define TIM_CR2_OIS3_Msk                    (0x1U << TIM_CR2_OIS3_Pos)         /*!< 0x00001000 */
#define TIM_CR2_OIS3                        TIM_CR2_OIS3_Msk                   /*!<Output Idle state 3 (OC3 output) */
#define TIM_CR2_OIS3N_Pos                   (13U)
#define TIM_CR2_OIS3N_Msk                   (0x1U << TIM_CR2_OIS3N_Pos)        /*!< 0x00002000 */
#define TIM_CR2_OIS3N                       TIM_CR2_OIS3N_Msk                  /*!<Output Idle state 3 (OC3N output) */
#define TIM_CR2_OIS4_Pos                    (14U)
#define TIM_CR2_OIS4_Msk                    (0x1U << TIM_CR2_OIS4_Pos)         /*!< 0x00004000 */
#define TIM_CR2_OIS4                        TIM_CR2_OIS4_Msk                   /*!<Output Idle state 4 (OC4 output) */

/*******************  Bit definition for TIM_SMCR register  ******************/
#define TIM_SMCR_SMS_Pos                    (0U)
#define TIM_SMCR_SMS_Msk                    (0x7U << TIM_SMCR_SMS_Pos)         /*!< 0x00000007 */
#define TIM_SMCR_SMS                        TIM_SMCR_SMS_Msk                   /*!<SMS[2:0] bits (Slave mode selection) */
#define TIM_SMCR_SMS_0                      (0x1U << TIM_SMCR_SMS_Pos)         /*!< 0x00000001 */
#define TIM_SMCR_SMS_1                      (0x2U << TIM_SMCR_SMS_Pos)         /*!< 0x00000002 */
#define TIM_SMCR_SMS_2                      (0x4U << TIM_SMCR_SMS_Pos)         /*!< 0x00000004 */

#define TIM_SMCR_TS_Pos                     (4U)
#define TIM_SMCR_TS_Msk                     (0x7U << TIM_SMCR_TS_Pos)          /*!< 0x00000070 */
#define TIM_SMCR_TS                         TIM_SMCR_TS_Msk                    /*!<TS[2:0] bits (Trigger selection) */
#define TIM_SMCR_TS_0                       (0x1U << TIM_SMCR_TS_Pos)          /*!< 0x00000010 */
#define TIM_SMCR_TS_1                       (0x2U << TIM_SMCR_TS_Pos)          /*!< 0x00000020 */
#define TIM_SMCR_TS_2                       (0x4U << TIM_SMCR_TS_Pos)          /*!< 0x00000040 */

#define TIM_SMCR_MSM_Pos                    (7U)
#define TIM_SMCR_MSM_Msk                    (0x1U << TIM_SMCR_MSM_Pos)         /*!< 0x00000080 */
#define TIM_SMCR_MSM                        TIM_SMCR_MSM_Msk                   /*!<Master/slave mode */

#define TIM_SMCR_ETF_Pos                    (8U)
#define TIM_SMCR_ETF_Msk                    (0xFU << TIM_SMCR_ETF_Pos)         /*!< 0x00000F00 */
#define TIM_SMCR_ETF                        TIM_SMCR_ETF_Msk                   /*!<ETF[3:0] bits (External trigger filter) */
#define TIM_SMCR_ETF_0                      (0x1U << TIM_SMCR_ETF_Pos)         /*!< 0x00000100 */
#define TIM_SMCR_ETF_1                      (0x2U << TIM_SMCR_ETF_Pos)         /*!< 0x00000200 */
#define TIM_SMCR_ETF_2                      (0x4U << TIM_SMCR_ETF_Pos)         /*!< 0x00000400 */
#define TIM_SMCR_ETF_3                      (0x8U << TIM_SMCR_ETF_Pos)         /*!< 0x00000800 */

#define TIM_SMCR_ETPS_Pos                   (12U)
#define TIM_SMCR_ETPS_Msk                   (0x3U << TIM_SMCR_ETPS_Pos)        /*!< 0x00003000 */
#define TIM_SMCR_ETPS                       TIM_SMCR_ETPS_Msk                  /*!<ETPS[1:0] bits (External trigger prescaler) */
#define TIM_SMCR_ETPS_0                     (0x1U << TIM_SMCR_ETPS_Pos)        /*!< 0x00001000 */
#define TIM_SMCR_ETPS_1                     (0x2U << TIM_SMCR_ETPS_Pos)        /*!< 0x00002000 */

#define TIM_SMCR_ECE_Pos                    (14U)
#define TIM_SMCR_ECE_Msk                    (0x1U << TIM_SMCR_ECE_Pos)         /*!< 0x00004000 */
#define TIM_SMCR_ECE                        TIM_SMCR_ECE_Msk                   /*!<External clock enable */
#define TIM_SMCR_ETP_Pos                    (15U)
#define TIM_SMCR_ETP_Msk                    (0x1U << TIM_SMCR_ETP_Pos)         /*!< 0x00008000 */
#define TIM_SMCR_ETP                        TIM_SMCR_ETP_Msk                   /*!<External trigger polarity */

/*******************  Bit definition for TIM_DIER register  ******************/
#define TIM_DIER_UIE_Pos                    (0U)
#define TIM_DIER_UIE_Msk                    (0x1U << TIM_DIER_UIE_Pos)         /*!< 0x00000001 */
#define TIM_DIER_UIE                        TIM_DIER_UIE_Msk                   /*!<Update interrupt enable */
#define TIM_DIER_CC1IE_Pos                  (1U)
#define TIM_DIER_CC1IE_Msk                  (0x1U << TIM_DIER_CC1IE_Pos)       /*!< 0x00000002 */
#define TIM_DIER_CC1IE                      TIM_DIER_CC1IE_Msk                 /*!<Capture/Compare 1 interrupt enable */
#define TIM_DIER_CC2IE_Pos                  (2U)
#define TIM_DIER_CC2IE_Msk                  (0x1U << TIM_DIER_CC2IE_Pos)       /*!< 0x00000004 */
#define TIM_DIER_CC2IE                      TIM_DIER_CC2IE_Msk                 /*!<Capture/Compare 2 interrupt enable */
#define TIM_DIER_CC3IE_Pos                  (3U)
#define TIM_DIER_CC3IE_Msk                  (0x1U << TIM_DIER_CC3IE_Pos)       /*!< 0x00000008 */
#define TIM_DIER_CC3IE                      TIM_DIER_CC3IE_Msk                 /*!<Capture/Compare 3 interrupt enable */
#define TIM_DIER_CC4IE_Pos                  (4U)
#define TIM_DIER_CC4IE_Msk                  (0x1U << TIM_DIER_CC4IE_Pos)       /*!< 0x00000010 */
#define TIM_DIER_CC4IE                      TIM_DIER_CC4IE_Msk                 /*!<Capture/Compare 4 interrupt enable */
#define TIM_DIER_COMIE_Pos                  (5U)
#define TIM_DIER_COMIE_Msk                  (0x1U << TIM_DIER_COMIE_Pos)       /*!< 0x00000020 */
#define TIM_DIER_COMIE                      TIM_DIER_COMIE_Msk                 /*!<COM interrupt enable */
#define TIM_DIER_TIE_Pos                    (6U)
#define TIM_DIER_TIE_Msk                    (0x1U << TIM_DIER_TIE_Pos)         /*!< 0x00000040 */
#define TIM_DIER_TIE                        TIM_DIER_TIE_Msk                   /*!<Trigger interrupt enable */
#define TIM_DIER_BIE_Pos                    (7U)
#define TIM_DIER_BIE_Msk                    (0x1U << TIM_DIER_BIE_Pos)         /*!< 0x00000080 */
#define TIM_DIER_BIE                        TIM_DIER_BIE_Msk                   /*!<Break interrupt enable */
#define TIM_DIER_UDE_Pos                    (8U)
#define TIM_DIER_UDE_Msk                    (0x1U << TIM_DIER_UDE_Pos)         /*!< 0x00000100 */
#define TIM_DIER_UDE                        TIM_DIER_UDE_Msk                   /*!<Update DMA request enable */
#define TIM_DIER_CC1DE_Pos                  (9U)
#define TIM_DIER_CC1DE_Msk                  (0x1U << TIM_DIER_CC1DE_Pos)       /*!< 0x00000200 */
#define TIM_DIER_CC1DE                      TIM_DIER_CC1DE_Msk                 /*!<Capture/Compare 1 DMA request enable */
#define TIM_DIER_CC2DE_Pos                  (10U)
#define TIM_DIER_CC2DE_Msk                  (0x1U << TIM_DIER_CC2DE_Pos)       /*!< 0x00000400 */
#define TIM_DIER_CC2DE                      TIM_DIER_CC2DE_Msk                 /*!<Capture/Compare 2 DMA request enable */
#define TIM_DIER_CC3DE_Pos                  (11U)
#define TIM_DIER_CC3DE_Msk                  (0x1U << TIM_DIER_CC3DE_Pos)       /*!< 0x00000800 */
#define TIM_DIER_CC3DE                      TIM_DIER_CC3DE_Msk                 /*!<Capture/Compare 3 DMA request enable */
#define TIM_DIER_CC4DE_Pos                  (12U)
#define TIM_DIER_CC4DE_Msk                  (0x1U << TIM_DIER_CC4DE_Pos)       /*!< 0x00001000 */
#define TIM_DIER_CC4DE                      TIM_DIER_CC4DE_Msk                 /*!<Capture/Compare 4 DMA request enable */
#define TIM_DIER_COMDE_Pos                  (13U)
#define TIM_DIER_COMDE_Msk                  (0x1U << TIM_DIER_COMDE_Pos)       /*!< 0x00002000 */
#define TIM_DIER_COMDE                      TIM_DIER_COMDE_Msk                 /*!<COM DMA request enable */
#define TIM_DIER_TDE_Pos                    (14U)
#define TIM_DIER_TDE_Msk                    (0x1U << TIM_DIER_TDE_Pos)         /*!< 0x00004000 */
#define TIM_DIER_TDE                        TIM_DIER_TDE_Msk                   /*!<Trigger DMA request enable */

/********************  Bit definition for TIM_SR register  *******************/
#define TIM_SR_UIF_Pos                      (0U)
#define TIM_SR_UIF_Msk                      (0x1U << TIM_SR_UIF_Pos)           /*!< 0x00000001 */
#define TIM_SR_UIF                          TIM_SR_UIF_Msk                     /*!<Update interrupt Flag */
#define TIM_SR_CC1IF_Pos                    (1U)
#define TIM_SR_CC1IF_Msk                    (0x1U << TIM_SR_CC1IF_Pos)         /*!< 0x00000002 */
#define TIM_SR_CC1IF                        TIM_SR_CC1IF_Msk                   /*!<Capture/Compare 1 interrupt Flag */
#define TIM_SR_CC2IF_Pos                    (2U)
#define TIM_SR_CC2IF_Msk                    (0x1U << TIM_SR_CC2IF_Pos)         /*!< 0x00000004 */
#define TIM_SR_CC2IF                        TIM_SR_CC2IF_Msk                   /*!<Capture/Compare 2 interrupt Flag */
#define TIM_SR_CC3IF_Pos                    (3U)
#define TIM_SR_CC3IF_Msk                    (0x1U << TIM_SR_CC3IF_Pos)         /*!< 0x00000008 */
#define TIM_SR_CC3IF                        TIM_SR_CC3IF_Msk                   /*!<Capture/Compare 3 interrupt Flag */
#define TIM_SR_CC4IF_Pos                    (4U)
#define TIM_SR_CC4IF_Msk                    (0x1U << TIM_SR_CC4IF_Pos)         /*!< 0x00000010 */
#define TIM_SR_CC4IF                        TIM_SR_CC4IF_Msk                   /*!<Capture/Compare 4 interrupt Flag */
#define TIM_SR_COMIF_Pos                    (5U)
#define TIM_SR_COMIF_Msk                    (0x1U << TIM_SR_COMIF_Pos)         /*!< 0x00000020 */
#define TIM_SR_COMIF                        TIM_SR_COMIF_Msk                   /*!<COM interrupt Flag */
#define TIM_SR_TIF_Pos                      (6U)
#define TIM_SR_TIF_Msk                      (0x1U << TIM_SR_TIF_Pos)           /*!< 0x00000040 */
#define TIM_SR_TIF                          TIM_SR_TIF_Msk                     /*!<Trigger interrupt Flag */
#define TIM_SR_BIF_Pos                      (7U)
#define TIM_SR_BIF_Msk                      (0x1U << TIM_SR_BIF_Pos)           /*!< 0x00000080 */
#define TIM_SR_BIF                          TIM_SR_BIF_Msk                     /*!<Break interrupt Flag */
#define TIM_SR_CC1OF_Pos                    (9U)
#define TIM_SR_CC1OF_Msk                    (0x1U << TIM_SR_CC1OF_Pos)         /*!< 0x00000200 */
#define TIM_SR_CC1OF                        TIM_SR_CC1OF_Msk                   /*!<Capture/Compare 1 Overcapture Flag */
#define TIM_SR_CC2OF_Pos                    (10U)
#define TIM_SR_CC2OF_Msk                    (0x1U << TIM_SR_CC2OF_Pos)         /*!< 0x00000400 */
#define TIM_SR_CC2OF                        TIM_SR_CC2OF_Msk                   /*!<Capture/Compare 2 Overcapture Flag */
#define TIM_SR_CC3OF_Pos                    (11U)
#define TIM_SR_CC3OF_Msk                    (0x1U << TIM_SR_CC3OF_Pos)         /*!< 0x00000800 */
#define TIM_SR_CC3OF                        TIM_SR_CC3OF_Msk                   /*!<Capture/Compare 3 Overcapture Flag */
#define TIM_SR_CC4OF_Pos                    (12U)
#define TIM_SR_CC4OF_Msk                    (0x1U << TIM_SR_CC4OF_Pos)         /*!< 0x00001000 */
#define TIM_SR_CC4OF                        TIM_SR_CC4OF_Msk                   /*!<Capture/Compare 4 Overcapture Flag */

/*******************  Bit definition for TIM_EGR register  *******************/
#define TIM_EGR_UG_Pos                      (0U)
#define TIM_EGR_UG_Msk                      (0x1U << TIM_EGR_UG_Pos)           /*!< 0x00000001 */
#define TIM_EGR_UG                          TIM_EGR_UG_Msk                     /*!<Update Generation */
#define TIM_EGR_CC1G_Pos                    (1U)
#define TIM_EGR_CC1G_Msk                    (0x1U << TIM_EGR_CC1G_Pos)         /*!< 0x00000002 */
#define TIM_EGR_CC1G                        TIM_EGR_CC1G_Msk                   /*!<Capture/Compare 1 Generation */
#define TIM_EGR_CC2G_Pos                    (2U)
#define TIM_EGR_CC2G_Msk                    (0x1U << TIM_EGR_CC2G_Pos)         /*!< 0x00000004 */
#define TIM_EGR_CC2G                        TIM_EGR_CC2G_Msk                   /*!<Capture/Compare 2 Generation */
#define TIM_EGR_CC3G_Pos                    (3U)
#define TIM_EGR_CC3G_Msk                    (0x1U << TIM_EGR_CC3G_Pos)         /*!< 0x00000008 */
#define TIM_EGR_CC3G                        TIM_EGR_CC3G_Msk                   /*!<Capture/Compare 3 Generation */
#define TIM_EGR_CC4G_Pos                    (4U)
#define TIM_EGR_CC4G_Msk                    (0x1U << TIM_EGR_CC4G_Pos)         /*!< 0x00000010 */
#define TIM_EGR_CC4G                        TIM_EGR_CC4G_Msk                   /*!<Capture/Compare 4 Generation */
#define TIM_EGR_COMG_Pos                    (5U)
#define TIM_EGR_COMG_Msk                    (0x1U << TIM_EGR_COMG_Pos)         /*!< 0x00000020 */
#define TIM_EGR_COMG                        TIM_EGR_COMG_Msk                   /*!<Capture/Compare Control Update Generation */
#define TIM_EGR_TG_Pos                      (6U)
#define TIM_EGR_TG_Msk                      (0x1U << TIM_EGR_TG_Pos)           /*!< 0x00000040 */
#define TIM_EGR_TG                          TIM_EGR_TG_Msk                     /*!<Trigger Generation */
#define TIM_EGR_BG_Pos                      (7U)
#define TIM_EGR_BG_Msk                      (0x1U << TIM_EGR_BG_Pos)           /*!< 0x00000080 */
#define TIM_EGR_BG                          TIM_EGR_BG_Msk                     /*!<Break Generation */

/******************  Bit definition for TIM_CCMR1 register  ******************/
#define TIM_CCMR1_CC1S_Pos                  (0U)
#define TIM_CCMR1_CC1S_Msk                  (0x3U << TIM_CCMR1_CC1S_Pos)       /*!< 0x00000003 */
#define TIM_CCMR1_CC1S                      TIM_CCMR1_CC1S_Msk                 /*!<CC1S[1:0] bits (Capture/Compare 1 Selection) */
#define TIM_CCMR1_CC1S_0                    (0x1U << TIM_CCMR1_CC1S_Pos)       /*!< 0x00000001 */
#define TIM_CCMR1_CC1S_1                    (0x2U << TIM_CCMR1_CC1S_Pos)       /*!< 0x00000002 */

#define TIM_CCMR1_OC1FE_Pos                 (2U)
#define TIM_CCMR1_OC1FE_Msk                 (0x1U << TIM_CCMR1_OC1FE_Pos)      /*!< 0x00000004 */
#define TIM_CCMR1_OC1FE                     TIM_CCMR1_OC1FE_Msk                /*!<Output Compare 1 Fast enable */
#define TIM_CCMR1_OC1PE_Pos                 (3U)
#define TIM_CCMR1_OC1PE_Msk                 (0x1U << TIM_CCMR1_OC1PE_Pos)      /*!< 0x00000008 */
#define TIM_CCMR1_OC1PE                     TIM_CCMR1_OC1PE_Msk                /*!<Output Compare 1 Preload enable */

#define TIM_CCMR1_OC1M_Pos                  (4U)
#define TIM_CCMR1_OC1M_Msk                  (0x7U << TIM_CCMR1_OC1M_Pos)       /*!< 0x00000070 */
#define TIM_CCMR1_OC1M                      TIM_CCMR1_OC1M_Msk                 /*!<OC1M[2:0] bits (Output Compare 1 Mode) */
#define TIM_CCMR1_OC1M_0                    (0x1U << TIM_CCMR1_OC1M_Pos)       /*!< 0x00000010 */
#define TIM_CCMR1_OC1M_1                    (0x2U << TIM_CCMR1_OC1M_Pos)       /*!< 0x00000020 */
#define TIM_CCMR1_OC1M_2                    (0x4U << TIM_CCMR1_OC1M_Pos)       /*!< 0x00000040 */

#define TIM_CCMR1_OC1CE_Pos                 (7U)
#define TIM_CCMR1_OC1CE_Msk                 (0x1U << TIM_CCMR1_OC1CE_Pos)      /*!< 0x00000080 */
#define TIM_CCMR1_OC1CE                     TIM_CCMR1_OC1CE_Msk                /*!<Output Compare 1Clear Enable */

#define TIM_CCMR1_CC2S_Pos                  (8U)
#define TIM_CCMR1_CC2S_Msk                  (0x3U << TIM_CCMR1_CC2S_Pos)       /*!< 0x00000300 */
#define TIM_CCMR1_CC2S                      TIM_CCMR1_CC2S_Msk                 /*!<CC2S[1:0] bits (Capture/Compare 2 Selection) */
#define TIM_CCMR1_CC2S_0                    (0x1U << TIM_CCMR1_CC2S_Pos)       /*!< 0x00000100 */
#define TIM_CCMR1_CC2S_1                    (0x2U << TIM_CCMR1_CC2S_Pos)       /*!< 0x00000200 */

#define TIM_CCMR1_OC2FE_Pos                 (10U)
#define TIM_CCMR1_OC2FE_Msk                 (0x1U << TIM_CCMR1_OC2FE_Pos)      /*!< 0x00000400 */
#define TIM_CCMR1_OC2FE                     TIM_CCMR1_OC2FE_Msk                /*!<Output Compare 2 Fast enable */
#define TIM_CCMR1_OC2PE_Pos                 (11U)
#define TIM_CCMR1_OC2PE_Msk                 (0x1U << TIM_CCMR1_OC2PE_Pos)      /*!< 0x00000800 */
#define TIM_CCMR1_OC2PE                     TIM_CCMR1_OC2PE_Msk                /*!<Output Compare 2 Preload enable */

#define TIM_CCMR1_OC2M_Pos                  (12U)
#define TIM_CCMR1_OC2M_Msk                  (0x7U << TIM_CCMR1_OC2M_Pos)       /*!< 0x00007000 */
#define TIM_CCMR1_OC2M                      TIM_CCMR1_OC2M_Msk                 /*!<OC2M[2:0] bits (Output Compare 2 Mode) */
#define TIM_CCMR1_OC2M_0                    (0x1U << TIM_CCMR1_OC2M_Pos)       /*!< 0x00001000 */
#define TIM_CCMR1_OC2M_1                    (0x2U << TIM_CCMR1_OC2M_Pos)       /*!< 0x00002000 */
#define TIM_CCMR1_OC2M_2                    (0x4U << TIM_CCMR1_OC2M_Pos)       /*!< 0x00004000 */

#define TIM_CCMR1_OC2CE_Pos                 (15U)
#define TIM_CCMR1_OC2CE_Msk                 (0x1U << TIM_CCMR1_OC2CE_Pos)      /*!< 0x00008000 */
#define TIM_CCMR1_OC2CE                     TIM_CCMR1_OC2CE_Msk                /*!<Output Compare 2 Clear Enable */

/*---------------------------------------------------------------------------*/

#define TIM_CCMR1_IC1PSC_Pos                (2U)
#define TIM_CCMR1_IC1PSC_Msk                (0x3U << TIM_CCMR1_IC1PSC_Pos)     /*!< 0x0000000C */
#define TIM_CCMR1_IC1PSC                    TIM_CCMR1_IC1PSC_Msk               /*!<IC1PSC[1:0] bits (Input Capture 1 Prescaler) */
#define TIM_CCMR1_IC1PSC_0                  (0x1U << TIM_CCMR1_IC1PSC_Pos)     /*!< 0x00000004 */
#define TIM_CCMR1_IC1PSC_1                  (0x2U << TIM_CCMR1_IC1PSC_Pos)     /*!< 0x00000008 */

#define TIM_CCMR1_IC1F_Pos                  (4U)
#define TIM_CCMR1_IC1F_Msk                  (0xFU << TIM_CCMR1_IC1F_Pos)       /*!< 0x000000F0 */
#define TIM_CCMR1_IC1F                      TIM_CCMR1_IC1F_Msk                 /*!<IC1F[3:0] bits (Input Capture 1 Filter) */
#define TIM_CCMR1_IC1F_0                    (0x1U << TIM_CCMR1_IC1F_Pos)       /*!< 0x00000010 */
#define TIM_CCMR1_IC1F_1                    (0x2U << TIM_CCMR1_IC1F_Pos)       /*!< 0x00000020 */
#define TIM_CCMR1_IC1F_2                    (0x4U << TIM_CCMR1_IC1F_Pos)       /*!< 0x00000040 */
#define TIM_CCMR1_IC1F_3                    (0x8U << TIM_CCMR1_IC1F_Pos)       /*!< 0x00000080 */

#define TIM_CCMR1_IC2PSC_Pos                (10U)
#define TIM_CCMR1_IC2PSC_Msk                (0x3U << TIM_CCMR1_IC2PSC_Pos)     /*!< 0x00000C00 */
#define TIM_CCMR1_IC2PSC                    TIM_CCMR1_IC2PSC_Msk               /*!<IC2PSC[1:0] bits (Input Capture 2 Prescaler) */
#define TIM_CCMR1_IC2PSC_0                  (0x1U << TIM_CCMR1_IC2PSC_Pos)     /*!< 0x00000400 */
#define TIM_CCMR1_IC2PSC_1                  (0x2U << TIM_CCMR1_IC2PSC_Pos)     /*!< 0x00000800 */

#define TIM_CCMR1_IC2F_Pos                  (12U)
#define TIM_CCMR1_IC2F_Msk                  (0xFU << TIM_CCMR1_IC2F_Pos)       /*!< 0x0000F000 */
#define TIM_CCMR1_IC2F                      TIM_CCMR1_IC2F_Msk                 /*!<IC2F[3:0] bits (Input Capture 2 Filter) */
#define TIM_CCMR1_IC2F_0                    (0x1U << TIM_CCMR1_IC2F_Pos)       /*!< 0x00001000 */
#define TIM_CCMR1_IC2F_1                    (0x2U << TIM_CCMR1_IC2F_Pos)       /*!< 0x00002000 */
#define TIM_CCMR1_IC2F_2                    (0x4U << TIM_CCMR1_IC2F_Pos)       /*!< 0x00004000 */
#define TIM_CCMR1_IC2F_3                    (0x8U << TIM_CCMR1_IC2F_Pos)       /*!< 0x00008000 */

/******************  Bit definition for TIM_CCMR2 register  ******************/
#define TIM_CCMR2_CC3S_Pos                  (0U)
#define TIM_CCMR2_CC3S_Msk                  (0x3U << TIM_CCMR2_CC3S_Pos)       /*!< 0x00000003 */
#define TIM_CCMR2_CC3S                      TIM_CCMR2_CC3S_Msk                 /*!<CC3S[1:0] bits (Capture/Compare 3 Selection) */
#define TIM_CCMR2_CC3S_0                    (0x1U << TIM_CCMR2_CC3S_Pos)       /*!< 0x00000001 */
#define TIM_CCMR2_CC3S_1                    (0x2U << TIM_CCMR2_CC3S_Pos)       /*!< 0x00000002 */

#define TIM_CCMR2_OC3FE_Pos                 (2U)
#define TIM_CCMR2_OC3FE_Msk                 (0x1U << TIM_CCMR2_OC3FE_Pos)      /*!< 0x00000004 */
#define TIM_CCMR2_OC3FE                     TIM_CCMR2_OC3FE_Msk                /*!<Output Compare 3 Fast enable */
#define TIM_CCMR2_OC3PE_Pos                 (3U)
#define TIM_CCMR2_OC3PE_Msk                 (0x1U << TIM_CCMR2_OC3PE_Pos)      /*!< 0x00000008 */
#define TIM_CCMR2_OC3PE                     TIM_CCMR2_OC3PE_Msk                /*!<Output Compare 3 Preload enable */

#define TIM_CCMR2_OC3M_Pos                  (4U)
#define TIM_CCMR2_OC3M_Msk                  (0x7U << TIM_CCMR2_OC3M_Pos)       /*!< 0x00000070 */
#define TIM_CCMR2_OC3M                      TIM_CCMR2_OC3M_Msk                 /*!<OC3M[2:0] bits (Output Compare 3 Mode) */
#define TIM_CCMR2_OC3M_0                    (0x1U << TIM_CCMR2_OC3M_Pos)       /*!< 0x00000010 */
#define TIM_CCMR2_OC3M_1                    (0x2U << TIM_CCMR2_OC3M_Pos)       /*!< 0x00000020 */
#define TIM_CCMR2_OC3M_2                    (0x4U << TIM_CCMR2_OC3M_Pos)       /*!< 0x00000040 */

#define TIM_CCMR2_OC3CE_Pos                 (7U)
#define TIM_CCMR2_OC3CE_Msk                 (0x1U << TIM_CCMR2_OC3CE_Pos)      /*!< 0x00000080 */
#define TIM_CCMR2_OC3CE                     TIM_CCMR2_OC3CE_Msk                /*!<Output Compare 3 Clear Enable */

#define TIM_CCMR2_CC4S_Pos                  (8U)
#define TIM_CCMR2_CC4S_Msk                  (0x3U << TIM_CCMR2_CC4S_Pos)       /*!< 0x00000300 */
#define TIM_CCMR2_CC4S                      TIM_CCMR2_CC4S_Msk                 /*!<CC4S[1:0] bits (Capture/Compare 4 Selection) */
#define TIM_CCMR2_CC4S_0                    (0x1U << TIM_CCMR2_CC4S_Pos)       /*!< 0x00000100 */
#define TIM_CCMR2_CC4S_1                    (0x2U << TIM_CCMR2_CC4S_Pos)       /*!< 0x00000200 */

#define TIM_CCMR2_OC4FE_Pos                 (10U)
#define TIM_CCMR2_OC4FE_Msk                 (0x1U << TIM_CCMR2_OC4FE_Pos)      /*!< 0x00000400 */
#define TIM_CCMR2_OC4FE                     TIM_CCMR2_OC4FE_Msk                /*!<Output Compare 4 Fast enable */
#define TIM_CCMR2_OC4PE_Pos                 (11U)
#define TIM_CCMR2_OC4PE_Msk                 (0x1U << TIM_CCMR2_OC4PE_Pos)      /*!< 0x00000800 */
#define TIM_CCMR2_OC4PE                     TIM_CCMR2_OC4PE_Msk                /*!<Output Compare 4 Preload enable */

#define TIM_CCMR2_OC4M_Pos                  (12U)
#define TIM_CCMR2_OC4M_Msk                  (0x7U << TIM_CCMR2_OC4M_Pos)       /*!< 0x00007000 */
#define TIM_CCMR2_OC4M                      TIM_CCMR2_OC4M_Msk                 /*!<OC4M[2:0] bits (Output Compare 4 Mode) */
#define TIM_CCMR2_OC4M_0                    (0x1U << TIM_CCMR2_OC4M_Pos)       /*!< 0x00001000 */
#define TIM_CCMR2_OC4M_1                    (0x2U << TIM_CCMR2_OC4M_Pos)       /*!< 0x00002000 */
#define TIM_CCMR2_OC4M_2                    (0x4U << TIM_CCMR2_OC4M_Pos)       /*!< 0x00004000 */

#define TIM_CCMR2_OC4CE_Pos                 (15U)
#define TIM_CCMR2_OC4CE_Msk                 (0x1U << TIM_CCMR2_OC4CE_Pos)      /*!< 0x00008000 */
#define TIM_CCMR2_OC4CE                     TIM_CCMR2_OC4CE_Msk                /*!<Output Compare 4 Clear Enable */

/*---------------------------------------------------------------------------*/

#define TIM_CCMR2_IC3PSC_Pos                (2U)
#define TIM_CCMR2_IC3PSC_Msk                (0x3U << TIM_CCMR2_IC3PSC_Pos)     /*!< 0x0000000C */
#define TIM_CCMR2_IC3PSC                    TIM_CCMR2_IC3PSC_Msk               /*!<IC3PSC[1:0] bits (Input Capture 3 Prescaler) */
#define TIM_CCMR2_IC3PSC_0                  (0x1U << TIM_CCMR2_IC3PSC_Pos)     /*!< 0x00000004 */
#define TIM_CCMR2_IC3PSC_1                  (0x2U << TIM_CCMR2_IC3PSC_Pos)     /*!< 0x00000008 */

#define TIM_CCMR2_IC3F_Pos                  (4U)
#define TIM_CCMR2_IC3F_Msk                  (0xFU << TIM_CCMR2_IC3F_Pos)       /*!< 0x000000F0 */
#define TIM_CCMR2_IC3F                      TIM_CCMR2_IC3F_Msk                 /*!<IC3F[3:0] bits (Input Capture 3 Filter) */
#define TIM_CCMR2_IC3F_0                    (0x1U << TIM_CCMR2_IC3F_Pos)       /*!< 0x00000010 */
#define TIM_CCMR2_IC3F_1                    (0x2U << TIM_CCMR2_IC3F_Pos)       /*!< 0x00000020 */
#define TIM_CCMR2_IC3F_2                    (0x4U << TIM_CCMR2_IC3F_Pos)       /*!< 0x00000040 */
#define TIM_CCMR2_IC3F_3                    (0x8U << TIM_CCMR2_IC3F_Pos)       /*!< 0x00000080 */

#define TIM_CCMR2_IC4PSC_Pos                (10U)
#define TIM_CCMR2_IC4PSC_Msk                (0x3U << TIM_CCMR2_IC4PSC_Pos)     /*!< 0x00000C00 */
#define TIM_CCMR2_IC4PSC                    TIM_CCMR2_IC4PSC_Msk               /*!<IC4PSC[1:0] bits (Input Capture 4 Prescaler) */
#define TIM_CCMR2_IC4PSC_0                  (0x1U << TIM_CCMR2_IC4PSC_Pos)     /*!< 0x00000400 */
#define TIM_CCMR2_IC4PSC_1                  (0x2U << TIM_CCMR2_IC4PSC_Pos)     /*!< 0x00000800 */

#define TIM_CCMR2_IC4F_Pos                  (12U)
#define TIM_CCMR2_IC4F_Msk                  (0xFU << TIM_CCMR2_IC4F_Pos)       /*!< 0x0000F000 */
#define TIM_CCMR2_IC4F                      TIM_CCMR2_IC4F_Msk                 /*!<IC4F[3:0] bits (Input Capture 4 Filter) */
#define TIM_CCMR2_IC4F_0                    (0x1U << TIM_CCMR2_IC4F_Pos)       /*!< 0x00001000 */
#define TIM_CCMR2_IC4F_1                    (0x2U << TIM_CCMR2_IC4F_Pos)       /*!< 0x00002000 */
#define TIM_CCMR2_IC4F_2                    (0x4U << TIM_CCMR2_IC4F_Pos)       /*!< 0x00004000 */
#define TIM_CCMR2_IC4F_3                    (0x8U << TIM_CCMR2_IC4F_Pos)       /*!< 0x00008000 */

/*******************  Bit definition for TIM_CCER register  ******************/
#define TIM_CCER_CC1E_Pos                   (0U)
#define TIM_CCER_CC1E_Msk                   (0x1U << TIM_CCER_CC1E_Pos)        /*!< 0x00000001 */
#define TIM_CCER_CC1E                       TIM_CCER_CC1E_Msk                  /*!<Capture/Compare 1 output enable */
#define TIM_CCER_CC1P_Pos                   (1U)
#define TIM_CCER_CC1P_Msk                   (0x1U << TIM_CCER_CC1P_Pos)        /*!< 0x00000002 */
#define TIM_CCER_CC1P                       TIM_CCER_CC1P_Msk                  /*!<Capture/Compare 1 output Polarity */
#define TIM_CCER_CC1NE_Pos                  (2U)
#define TIM_CCER_CC1NE_Msk                  (0x1U << TIM_CCER_CC1NE_Pos)       /*!< 0x00000004 */
#define TIM_CCER_CC1NE                      TIM_CCER_CC1NE_Msk                 /*!<Capture/Compare 1 Complementary output enable */
#define TIM_CCER_CC1NP_Pos                  (3U)
#define TIM_CCER_CC1NP_Msk                  (0x1U << TIM_CCER_CC1NP_Pos)       /*!< 0x00000008 */
#define TIM_CCER_CC1NP                      TIM_CCER_CC1NP_Msk                 /*!<Capture/Compare 1 Complementary output Polarity */
#define TIM_CCER_CC2E_Pos                   (4U)
#define TIM_CCER_CC2E_Msk                   (0x1U << TIM_CCER_CC2E_Pos)        /*!< 0x00000010 */
#define TIM_CCER_CC2E                       TIM_CCER_CC2E_Msk                  /*!<Capture/Compare 2 output enable */
#define TIM_CCER_CC2P_Pos                   (5U)
#define TIM_CCER_CC2P_Msk                   (0x1U << TIM_CCER_CC2P_Pos)        /*!< 0x00000020 */
#define TIM_CCER_CC2P                       TIM_CCER_CC2P_Msk                  /*!<Capture/Compare 2 output Polarity */
#define TIM_CCER_CC2NE_Pos                  (6U)
#define TIM_CCER_CC2NE_Msk                  (0x1U << TIM_CCER_CC2NE_Pos)       /*!< 0x00000040 */
#define TIM_CCER_CC2NE                      TIM_CCER_CC2NE_Msk                 /*!<Capture/Compare 2 Complementary output enable */
#define TIM_CCER_CC2NP_Pos                  (7U)
#define TIM_CCER_CC2NP_Msk                  (0x1U << TIM_CCER_CC2NP_Pos)       /*!< 0x00000080 */
#define TIM_CCER_CC2NP                      TIM_CCER_CC2NP_Msk                 /*!<Capture/Compare 2 Complementary output Polarity */
#define TIM_CCER_CC3E_Pos                   (8U)
#define TIM_CCER_CC3E_Msk                   (0x1U << TIM_CCER_CC3E_Pos)        /*!< 0x00000100 */
#define TIM_CCER_CC3E                       TIM_CCER_CC3E_Msk                  /*!<Capture/Compare 3 output enable */
#define TIM_CCER_CC3P_Pos                   (9U)
#define TIM_CCER_CC3P_Msk                   (0x1U << TIM_CCER_CC3P_Pos)        /*!< 0x00000200 */
#define TIM_CCER_CC3P                       TIM_CCER_CC3P_Msk                  /*!<Capture/Compare 3 output Polarity */
#define TIM_CCER_CC3NE_Pos                  (10U)
#define TIM_CCER_CC3NE_Msk                  (0x1U << TIM_CCER_CC3NE_Pos)       /*!< 0x00000400 */
#define TIM_CCER_CC3NE                      TIM_CCER_CC3NE_Msk                 /*!<Capture/Compare 3 Complementary output enable */
#define TIM_CCER_CC3NP_Pos                  (11U)
#define TIM_CCER_CC3NP_Msk                  (0x1U << TIM_CCER_CC3NP_Pos)       /*!< 0x00000800 */
#define TIM_CCER_CC3NP                      TIM_CCER_CC3NP_Msk                 /*!<Capture/Compare 3 Complementary output Polarity */
#define TIM_CCER_CC4E_Pos                   (12U)
#define TIM_CCER_CC4E_Msk                   (0x1U << TIM_CCER_CC4E_Pos)        /*!< 0x00001000 */
#define TIM_CCER_CC4E                       TIM_CCER_CC4E_Msk                  /*!<Capture/Compare 4 output enable */
#define TIM_CCER_CC4P_Pos                   (13U)
#define TIM_CCER_CC4P_Msk                   (0x1U << TIM_CCER_CC4P_Pos)        /*!< 0x00002000 */
#define TIM_CCER_CC4P                       TIM_CCER_CC4P_Msk                  /*!<Capture/Compare 4 output Polarity */

/*******************  Bit definition for TIM_CNT register  *******************/
#define TIM_CNT_CNT_Pos                     (0U)
#define TIM_CNT_CNT_Msk                     (0xFFFFFFFFU << TIM_CNT_CNT_Pos)   /*!< 0xFFFFFFFF */
#define TIM_CNT_CNT                         TIM_CNT_CNT_Msk                    /*!<Counter Value */

/*******************  Bit definition for TIM_PSC register  *******************/
#define TIM_PSC_PSC_Pos                     (0U)
#define TIM_PSC_PSC_Msk                     (0xFFFFU << TIM_PSC_PSC_Pos)       /*!< 0x0000FFFF */
#define TIM_PSC_PSC                         TIM_PSC_PSC_Msk                    /*!<Prescaler Value */

/*******************  Bit definition for TIM_ARR register  *******************/
#define TIM_ARR_ARR_Pos                     (0U)
#define TIM_ARR_ARR_Msk                     (0xFFFFFFFFU << TIM_ARR_ARR_Pos)   /*!< 0xFFFFFFFF */
#define TIM_ARR_ARR                         TIM_ARR_ARR_Msk                    /*!<actual auto-reload Value */

/*******************  Bit definition for TIM_RCR register  *******************/
#define TIM_RCR_REP_Pos                     (0U)
#define TIM_RCR_REP_Msk                     (0xFFU << TIM_RCR_REP_Pos)         /*!< 0x000000FF */
#define TIM_RCR_REP                         TIM_RCR_REP_Msk                    /*!<Repetition Counter Value */

/*******************  Bit definition for TIM_CCR1 register  ******************/
#define TIM_CCR1_CCR1_Pos                   (0U)
#define TIM_CCR1_CCR1_Msk                   (0xFFFFU << TIM_CCR1_CCR1_Pos)     /*!< 0x0000FFFF */
#define TIM_CCR1_CCR1                       TIM_CCR1_CCR1_Msk                  /*!<Capture/Compare 1 Value */

/*******************  Bit definition for TIM_CCR2 register  ******************/
#define TIM_CCR2_CCR2_Pos                   (0U)
#define TIM_CCR2_CCR2_Msk                   (0xFFFFU << TIM_CCR2_CCR2_Pos)     /*!< 0x0000FFFF */
#define TIM_CCR2_CCR2                       TIM_CCR2_CCR2_Msk                  /*!<Capture/Compare 2 Value */

/*******************  Bit definition for TIM_CCR3 register  ******************/
#define TIM_CCR3_CCR3_Pos                   (0U)
#define TIM_CCR3_CCR3_Msk                   (0xFFFFU << TIM_CCR3_CCR3_Pos)     /*!< 0x0000FFFF */
#define TIM_CCR3_CCR3                       TIM_CCR3_CCR3_Msk                  /*!<Capture/Compare 3 Value */

/*******************  Bit definition for TIM_CCR4 register  ******************/
#define TIM_CCR4_CCR4_Pos                   (0U)
#define TIM_CCR4_CCR4_Msk                   (0xFFFFU << TIM_CCR4_CCR4_Pos)     /*!< 0x0000FFFF */
#define TIM_CCR4_CCR4                       TIM_CCR4_CCR4_Msk                  /*!<Capture/Compare 4 Value */

/*******************  Bit definition for TIM_BDTR register  ******************/
#define TIM_BDTR_DTG_Pos                    (0U)
#define TIM_BDTR_DTG_Msk                    (0xFFU << TIM_BDTR_DTG_Pos)        /*!< 0x000000FF */
#define TIM_BDTR_DTG                        TIM_BDTR_DTG_Msk                   /*!<DTG[0:7] bits (Dead-Time Generator set-up) */
#define TIM_BDTR_DTG_0                      (0x01U << TIM_BDTR_DTG_Pos)        /*!< 0x00000001 */
#define TIM_BDTR_DTG_1                      (0x02U << TIM_BDTR_DTG_Pos)        /*!< 0x00000002 */
#define TIM_BDTR_DTG_2                      (0x04U << TIM_BDTR_DTG_Pos)        /*!< 0x00000004 */
#define TIM_BDTR_DTG_3                      (0x08U << TIM_BDTR_DTG_Pos)        /*!< 0x00000008 */
#define TIM_BDTR_DTG_4                      (0x10U << TIM_BDTR_DTG_Pos)        /*!< 0x00000010 */
#define TIM_BDTR_DTG_5                      (0x20U << TIM_BDTR_DTG_Pos)        /*!< 0x00000020 */
#define TIM_BDTR_DTG_6                      (0x40U << TIM_BDTR_DTG_Pos)        /*!< 0x00000040 */
#define TIM_BDTR_DTG_7                      (0x80U << TIM_BDTR_DTG_Pos)        /*!< 0x00000080 */

#define TIM_BDTR_LOCK_Pos                   (8U)
#define TIM_BDTR_LOCK_Msk                   (0x3U << TIM_BDTR_LOCK_Pos)        /*!< 0x00000300 */
#define TIM_BDTR_LOCK                       TIM_BDTR_LOCK_Msk                  /*!<LOCK[1:0] bits (Lock Configuration) */
#define TIM_BDTR_LOCK_0                     (0x1U << TIM_BDTR_LOCK_Pos)        /*!< 0x00000100 */
#define TIM_BDTR_LOCK_1                     (0x2U << TIM_BDTR_LOCK_Pos)        /*!< 0x00000200 */

#define TIM_BDTR_OSSI_Pos                   (10U)
#define TIM_BDTR_OSSI_Msk                   (0x1U << TIM_BDTR_OSSI_Pos)        /*!< 0x00000400 */
#define TIM_BDTR_OSSI                       TIM_BDTR_OSSI_Msk                  /*!<Off-State Selection for Idle mode */
#define TIM_BDTR_OSSR_Pos                   (11U)
#define TIM_BDTR_OSSR_Msk                   (0x1U << TIM_BDTR_OSSR_Pos)        /*!< 0x00000800 */
#define TIM_BDTR_OSSR                       TIM_BDTR_OSSR_Msk                  /*!<Off-State Selection for Run mode */
#define TIM_BDTR_BKE_Pos                    (12U)
#define TIM_BDTR_BKE_Msk                    (0x1U << TIM_BDTR_BKE_Pos)         /*!< 0x00001000 */
#define TIM_BDTR_BKE                        TIM_BDTR_BKE_Msk                   /*!<Break enable */
#define TIM_BDTR_BKP_Pos                    (13U)
#define TIM_BDTR_BKP_Msk                    (0x1U << TIM_BDTR_BKP_Pos)         /*!< 0x00002000 */
#define TIM_BDTR_BKP                        TIM_BDTR_BKP_Msk                   /*!<Break Polarity */
#define TIM_BDTR_AOE_Pos                    (14U)
#define TIM_BDTR_AOE_Msk                    (0x1U << TIM_BDTR_AOE_Pos)         /*!< 0x00004000 */
#define TIM_BDTR_AOE                        TIM_BDTR_AOE_Msk                   /*!<Automatic Output enable */
#define TIM_BDTR_MOE_Pos                    (15U)
#define TIM_BDTR_MOE_Msk                    (0x1U << TIM_BDTR_MOE_Pos)         /*!< 0x00008000 */
#define TIM_BDTR_MOE                        TIM_BDTR_MOE_Msk                   /*!<Main Output enable */

/*******************  Bit definition for TIM_DCR register  *******************/
#define TIM_DCR_DBA_Pos                     (0U)
#define TIM_DCR_DBA_Msk                     (0x1FU << TIM_DCR_DBA_Pos)         /*!< 0x0000001F */
#define TIM_DCR_DBA                         TIM_DCR_DBA_Msk                    /*!<DBA[4:0] bits (DMA Base Address) */
#define TIM_DCR_DBA_0                       (0x01U << TIM_DCR_DBA_Pos)         /*!< 0x00000001 */
#define TIM_DCR_DBA_1                       (0x02U << TIM_DCR_DBA_Pos)         /*!< 0x00000002 */
#define TIM_DCR_DBA_2                       (0x04U << TIM_DCR_DBA_Pos)         /*!< 0x00000004 */
#define TIM_DCR_DBA_3                       (0x08U << TIM_DCR_DBA_Pos)         /*!< 0x00000008 */
#define TIM_DCR_DBA_4                       (0x10U << TIM_DCR_DBA_Pos)         /*!< 0x00000010 */

#define TIM_DCR_DBL_Pos                     (8U)
#define TIM_DCR_DBL_Msk                     (0x1FU << TIM_DCR_DBL_Pos)         /*!< 0x00001F00 */
#define TIM_DCR_DBL                         TIM_DCR_DBL_Msk                    /*!<DBL[4:0] bits (DMA Burst Length) */
#define TIM_DCR_DBL_0                       (0x01U << TIM_DCR_DBL_Pos)         /*!< 0x00000100 */
#define TIM_DCR_DBL_1                       (0x02U << TIM_DCR_DBL_Pos)         /*!< 0x00000200 */
#define TIM_DCR_DBL_2                       (0x04U << TIM_DCR_DBL_Pos)         /*!< 0x00000400 */
#define TIM_DCR_DBL_3                       (0x08U << TIM_DCR_DBL_Pos)         /*!< 0x00000800 */
#define TIM_DCR_DBL_4                       (0x10U << TIM_DCR_DBL_Pos)         /*!< 0x00001000 */

/*******************  Bit definition for TIM_DMAR register  ******************/
#define TIM_DMAR_DMAB_Pos                   (0U)
#define TIM_DMAR_DMAB_Msk                   (0xFFFFU << TIM_DMAR_DMAB_Pos)     /*!< 0x0000FFFF */
#define TIM_DMAR_DMAB                       TIM_DMAR_DMAB_Msk                  /*!<DMA register for burst accesses */

/******************************************************************************/
/*                                                                            */
/*                             Real-Time Clock                                */
/*                                                                            */
/******************************************************************************/

/*******************  Bit definition for RTC_CRH register  ********************/
#define RTC_CRH_SECIE_Pos                   (0U)
#define RTC_CRH_SECIE_Msk                   (0x1U << RTC_CRH_SECIE_Pos)        /*!< 0x00000001 */
#define RTC_CRH_SECIE                       RTC_CRH_SECIE_Msk                  /*!< Second Interrupt Enable */
#define RTC_CRH_ALRIE_Pos                   (1U)
#define RTC_CRH_ALRIE_Msk                   (0x1U << RTC_CRH_ALRIE_Pos)        /*!< 0x00000002 */
#define RTC_CRH_ALRIE                       RTC_CRH_ALRIE_Msk                  /*!< Alarm Interrupt Enable */
#define RTC_CRH_OWIE_Pos                    (2U)
#define RTC_CRH_OWIE_Msk                    (0x1U << RTC_CRH_OWIE_Pos)         /*!< 0x00000004 */
#define RTC_CRH_OWIE                        RTC_CRH_OWIE_Msk                   /*!< OverfloW Interrupt Enable */

/*******************  Bit definition for RTC_CRL register  ********************/
#define RTC_CRL_SECF_Pos                    (0U)
#define RTC_CRL_SECF_Msk                    (0x1U << RTC_CRL_SECF_Pos)         /*!< 0x00000001 */
#define RTC_CRL_SECF                        RTC_CRL_SECF_Msk                   /*!< Second Flag */
#define RTC_CRL_ALRF_Pos                    (1U)
#define RTC_CRL_ALRF_Msk                    (0x1U << RTC_CRL_ALRF_Pos)         /*!< 0x00000002 */
#define RTC_CRL_ALRF                        RTC_CRL_ALRF_Msk                   /*!< Alarm Flag */
#define RTC_CRL_OWF_Pos                     (2U)
#define RTC_CRL_OWF_Msk                     (0x1U << RTC_CRL_OWF_Pos)          /*!< 0x00000004 */
#define RTC_CRL_OWF                         RTC_CRL_OWF_Msk                    /*!< OverfloW Flag */
#define RTC_CRL_RSF_Pos                     (3U)
#define RTC_CRL_RSF_Msk                     (0x1U << RTC_CRL_RSF_Pos)          /*!< 0x00000008 */
#define RTC_CRL_RSF                         RTC_CRL_RSF_Msk                    /*!< Registers Synchronized Flag */
#define RTC_CRL_CNF_Pos                     (4U)
#define RTC_CRL_CNF_Msk                     (0x1U << RTC_CRL_CNF_Pos)          /*!< 0x00000010 */
#define RTC_CRL_CNF                         RTC_CRL_CNF_Msk                    /*!< Configuration Flag */
#define RTC_CRL_RTOFF_Pos                   (5U)
#define RTC_CRL_RTOFF_Msk                   (0x1U << RTC_CRL_RTOFF_Pos)        /*!< 0x00000020 */
#define RTC_CRL_RTOFF                       RTC_CRL_RTOFF_Msk                  /*!< RTC operation OFF */

/*******************  Bit definition for RTC_PRLH register  *******************/
#define RTC_PRLH_PRL_Pos                    (0U)
#define RTC_PRLH_PRL_Msk                    (0xFU << RTC_PRLH_PRL_Pos)         /*!< 0x0000000F */
#define RTC_PRLH_PRL                        RTC_PRLH_PRL_Msk                   /*!< RTC Prescaler Reload Value High */

/*******************  Bit definition for RTC_PRLL register  *******************/
#define RTC_PRLL_PRL_Pos                    (0U)
#define RTC_PRLL_PRL_Msk                    (0xFFFFU << RTC_PRLL_PRL_Pos)      /*!< 0x0000FFFF */
#define RTC_PRLL_PRL                        RTC_PRLL_PRL_Msk                   /*!< RTC Prescaler Reload Value Low */

/*******************  Bit definition for RTC_DIVH register  *******************/
#define RTC_DIVH_RTC_DIV_Pos                (0U)
#define RTC_DIVH_RTC_DIV_Msk                (0xFU << RTC_DIVH_RTC_DIV_Pos)     /*!< 0x0000000F */
#define RTC_DIVH_RTC_DIV                    RTC_DIVH_RTC_DIV_Msk               /*!< RTC Clock Divider High */

/*******************  Bit definition for RTC_DIVL register  *******************/
#define RTC_DIVL_RTC_DIV_Pos                (0U)
#define RTC_DIVL_RTC_DIV_Msk                (0xFFFFU << RTC_DIVL_RTC_DIV_Pos)  /*!< 0x0000FFFF */
#define RTC_DIVL_RTC_DIV                    RTC_DIVL_RTC_DIV_Msk               /*!< RTC Clock Divider Low */

/*******************  Bit definition for RTC_CNTH register  *******************/
#define RTC_CNTH_RTC_CNT_Pos                (0U)
#define RTC_CNTH_RTC_CNT_Msk                (0xFFFFU << RTC_CNTH_RTC_CNT_Pos)  /*!< 0x0000FFFF */
#define RTC_CNTH_RTC_CNT                    RTC_CNTH_RTC_CNT_Msk               /*!< RTC Counter High */

/*******************  Bit definition for RTC_CNTL register  *******************/
#define RTC_CNTL_RTC_CNT_Pos                (0U)
#define RTC_CNTL_RTC_CNT_Msk                (0xFFFFU << RTC_CNTL_RTC_CNT_Pos)  /*!< 0x0000FFFF */
#define RTC_CNTL_RTC_CNT                    RTC_CNTL_RTC_CNT_Msk               /*!< RTC Counter Low */

/*******************  Bit definition for RTC_ALRH register  *******************/
#define RTC_ALRH_RTC_ALR_Pos                (0U)
#define RTC_ALRH_RTC_ALR_Msk                (0xFFFFU << RTC_ALRH_RTC_ALR_Pos)  /*!< 0x0000FFFF */
#define RTC_ALRH_RTC_ALR                    RTC_ALRH_RTC_ALR_Msk               /*!< RTC Alarm High */

/*******************  Bit definition for RTC_ALRL register  *******************/
#define RTC_ALRL_RTC_ALR_Pos                (0U)
#define RTC_ALRL_RTC_ALR_Msk                (0xFFFFU << RTC_ALRL_RTC_ALR_Pos)  /*!< 0x0000FFFF */
#define RTC_ALRL_RTC_ALR                    RTC_ALRL_RTC_ALR_Msk               /*!< RTC Alarm Low */

/******************************************************************************/
/*                                                                            */
/*                        Independent WATCHDOG (IWDG)                         */
/*                                                                            */
/******************************************************************************/

/*******************  Bit definition for IWDG_KR register  ********************/
#define IWDG_KR_KEY_Pos                     (0U)
#define IWDG_KR_KEY_Msk                     (0xFFFFU << IWDG_KR_KEY_Pos)       /*!< 0x0000FFFF */
#define IWDG_KR_KEY                         IWDG_KR_KEY_Msk                    /*!< Key value (write only, read 0000h) */

/*******************  Bit definition for IWDG_PR register  ********************/
#define IWDG_PR_PR_Pos                      (0U)
#define IWDG_PR_PR_Msk                      (0x7U << IWDG_PR_PR_Pos)           /*!< 0x00000007 */
#define IWDG_PR_PR                          IWDG_PR_PR_Msk                     /*!< PR[2:0] (Prescaler divider) */
#define IWDG_PR_PR_0                        (0x1U << IWDG_PR_PR_Pos)           /*!< 0x00000001 */
#define IWDG_PR_PR_1                        (0x2U << IWDG_PR_PR_Pos)           /*!< 0x00000002 */
#define IWDG_PR_PR_2                        (0x4U << IWDG_PR_PR_Pos)           /*!< 0x00000004 */

/*******************  Bit definition for IWDG_RLR register  *******************/
#define IWDG_RLR_RL_Pos                     (0U)
#define IWDG_RLR_RL_Msk                     (0xFFFU << IWDG_RLR_RL_Pos)        /*!< 0x00000FFF */
#define IWDG_RLR_RL                         IWDG_RLR_RL_Msk                    /*!< Watchdog counter reload value */

/*******************  Bit definition for IWDG_SR register  ********************/
#define IWDG_SR_PVU_Pos                     (0U)
#define IWDG_SR_PVU_Msk                     (0x1U << IWDG_SR_PVU_Pos)          /*!< 0x00000001 */
#define IWDG_SR_PVU                         IWDG_SR_PVU_Msk                    /*!< Watchdog prescaler value update */
#define IWDG_SR_RVU_Pos                     (1U)
#define IWDG_SR_RVU_Msk                     (0x1U << IWDG_SR_RVU_Pos)          /*!< 0x00000002 */
#define IWDG_SR_RVU                         IWDG_SR_RVU_Msk                    /*!< Watchdog counter reload value update */

/******************************************************************************/
/*                                                                            */
/*                         Window WATCHDOG (WWDG)                             */
/*                                                                            */
/******************************************************************************/

/*******************  Bit definition for WWDG_CR register  ********************/
#define WWDG_CR_T_Pos                       (0U)
#define WWDG_CR_T_Msk                       (0x7FU << WWDG_CR_T_Pos)           /*!< 0x0000007F */
#define WWDG_CR_T                           WWDG_CR_T_Msk                      /*!< T[6:0] bits (7-Bit counter (MSB to LSB)) */
#define WWDG_CR_T_0                         (0x01U << WWDG_CR_T_Pos)           /*!< 0x00000001 */
#define WWDG_CR_T_1                         (0x02U << WWDG_CR_T_Pos)           /*!< 0x00000002 */
#define WWDG_CR_T_2                         (0x04U << WWDG_CR_T_Pos)           /*!< 0x00000004 */
#define WWDG_CR_T_3                         (0x08U << WWDG_CR_T_Pos)           /*!< 0x00000008 */
#define WWDG_CR_T_4                         (0x10U << WWDG_CR_T_Pos)           /*!< 0x00000010 */
#define WWDG_CR_T_5                         (0x20U << WWDG_CR_T_Pos)           /*!< 0x00000020 */
#define WWDG_CR_T_6                         (0x40U << WWDG_CR_T_Pos)           /*!< 0x00000040 */

/* Legacy defines */
#define  WWDG_CR_T0 WWDG_CR_T_0
#define  WWDG_CR_T1 WWDG_CR_T_1
#define  WWDG_CR_T2 WWDG_CR_T_2
#define  WWDG_CR_T3 WWDG_CR_T_3
#define  WWDG_CR_T4 WWDG_CR_T_4
#define  WWDG_CR_T5 WWDG_CR_T_5
#define  WWDG_CR_T6 WWDG_CR_T_6

#define WWDG_CR_WDGA_Pos                    (7U)
#define WWDG_CR_WDGA_Msk                    (0x1U << WWDG_CR_WDGA_Pos)         /*!< 0x00000080 */
#define WWDG_CR_WDGA                        WWDG_CR_WDGA_Msk                   /*!< Activation bit */

/*******************  Bit definition for WWDG_CFR register  *******************/
#define WWDG_CFR_W_Pos                      (0U)
#define WWDG_CFR_W_Msk                      (0x7FU << WWDG_CFR_W_Pos)          /*!< 0x0000007F */
#define WWDG_CFR_W                          WWDG_CFR_W_Msk                     /*!< W[6:0] bits (7-bit window value) */
#define WWDG_CFR_W_0                        (0x01U << WWDG_CFR_W_Pos)          /*!< 0x00000001 */
#define WWDG_CFR_W_1                        (0x02U << WWDG_CFR_W_Pos)          /*!< 0x00000002 */
#define WWDG_CFR_W_2                        (0x04U << WWDG_CFR_W_Pos)          /*!< 0x00000004 */
#define WWDG_CFR_W_3                        (0x08U << WWDG_CFR_W_Pos)          /*!< 0x00000008 */
#define WWDG_CFR_W_4                        (0x10U << WWDG_CFR_W_Pos)          /*!< 0x00000010 */
#define WWDG_CFR_W_5                        (0x20U << WWDG_CFR_W_Pos)          /*!< 0x00000020 */
#define WWDG_CFR_W_6                        (0x40U << WWDG_CFR_W_Pos)          /*!< 0x00000040 */

/* Legacy defines */
#define  WWDG_CFR_W0 WWDG_CFR_W_0
#define  WWDG_CFR_W1 WWDG_CFR_W_1
#define  WWDG_CFR_W2 WWDG_CFR_W_2
#define  WWDG_CFR_W3 WWDG_CFR_W_3
#define  WWDG_CFR_W4 WWDG_CFR_W_4
#define  WWDG_CFR_W5 WWDG_CFR_W_5
#define  WWDG_CFR_W6 WWDG_CFR_W_6

#define WWDG_CFR_WDGTB_Pos                  (7U)
#define WWDG_CFR_WDGTB_Msk                  (0x3U << WWDG_CFR_WDGTB_Pos)       /*!< 0x00000180 */
#define WWDG_CFR_WDGTB                      WWDG_CFR_WDGTB_Msk                 /*!< WDGTB[1:0] bits (Timer Base) */
#define WWDG_CFR_WDGTB_0                    (0x1U << WWDG_CFR_WDGTB_Pos)       /*!< 0x00000080 */
#define WWDG_CFR_WDGTB_1                    (0x2U << WWDG_CFR_WDGTB_Pos)       /*!< 0x00000100 */

/* Legacy defines */
#define  WWDG_CFR_WDGTB0 WWDG_CFR_WDGTB_0
#define  WWDG_CFR_WDGTB1 WWDG_CFR_WDGTB_1

#define WWDG_CFR_EWI_Pos                    (9U)
#define WWDG_CFR_EWI_Msk                    (0x1U << WWDG_CFR_EWI_Pos)         /*!< 0x00000200 */
#define WWDG_CFR_EWI                        WWDG_CFR_EWI_Msk                   /*!< Early Wakeup Interrupt */

/*******************  Bit definition for WWDG_SR register  ********************/
#define WWDG_SR_EWIF_Pos                    (0U)
#define WWDG_SR_EWIF_Msk                    (0x1U << WWDG_SR_EWIF_Pos)         /*!< 0x00000001 */
#define WWDG_SR_EWIF                        WWDG_SR_EWIF_Msk                   /*!< Early Wakeup Interrupt Flag */


/******************************************************************************/
/*                                                                            */
/*                                   USB Device FS                            */
/*                                                                            */
/******************************************************************************/

/*!< Endpoint-specific registers */
#define  USB_EP0R                            USB_BASE                      /*!< Endpoint 0 register address */
#define  USB_EP1R                            (USB_BASE + 0x00000004)       /*!< Endpoint 1 register address */
#define  USB_EP2R                            (USB_BASE + 0x00000008)       /*!< Endpoint 2 register address */
#define  USB_EP3R                            (USB_BASE + 0x0000000C)       /*!< Endpoint 3 register address */
#define  USB_EP4R                            (USB_BASE + 0x00000010)       /*!< Endpoint 4 register address */
#define  USB_EP5R                            (USB_BASE + 0x00000014)       /*!< Endpoint 5 register address */
#define  USB_EP6R                            (USB_BASE + 0x00000018)       /*!< Endpoint 6 register address */
#define  USB_EP7R                            (USB_BASE + 0x0000001C)       /*!< Endpoint 7 register address */

/* bit positions */
#define USB_EP_CTR_RX_Pos                       (15U)
#define USB_EP_CTR_RX_Msk                       (0x1U << USB_EP_CTR_RX_Pos)    /*!< 0x00008000 */
#define USB_EP_CTR_RX                           USB_EP_CTR_RX_Msk              /*!< EndPoint Correct TRansfer RX */
#define USB_EP_DTOG_RX_Pos                      (14U)
#define USB_EP_DTOG_RX_Msk                      (0x1U << USB_EP_DTOG_RX_Pos)   /*!< 0x00004000 */
#define USB_EP_DTOG_RX                          USB_EP_DTOG_RX_Msk             /*!< EndPoint Data TOGGLE RX */
#define USB_EPRX_STAT_Pos                       (12U)
#define USB_EPRX_STAT_Msk                       (0x3U << USB_EPRX_STAT_Pos)    /*!< 0x00003000 */
#define USB_EPRX_STAT                           USB_EPRX_STAT_Msk              /*!< EndPoint RX STATus bit field */
#define USB_EP_SETUP_Pos                        (11U)
#define USB_EP_SETUP_Msk                        (0x1U << USB_EP_SETUP_Pos)     /*!< 0x00000800 */
#define USB_EP_SETUP                            USB_EP_SETUP_Msk               /*!< EndPoint SETUP */
#define USB_EP_T_FIELD_Pos                      (9U)
#define USB_EP_T_FIELD_Msk                      (0x3U << USB_EP_T_FIELD_Pos)   /*!< 0x00000600 */
#define USB_EP_T_FIELD                          USB_EP_T_FIELD_Msk             /*!< EndPoint TYPE */
#define USB_EP_KIND_Pos                         (8U)
#define USB_EP_KIND_Msk                         (0x1U << USB_EP_KIND_Pos)      /*!< 0x00000100 */
#define USB_EP_KIND                             USB_EP_KIND_Msk                /*!< EndPoint KIND */
#define USB_EP_CTR_TX_Pos                       (7U)
#define USB_EP_CTR_TX_Msk                       (0x1U << USB_EP_CTR_TX_Pos)    /*!< 0x00000080 */
#define USB_EP_CTR_TX                           USB_EP_CTR_TX_Msk              /*!< EndPoint Correct TRansfer TX */
#define USB_EP_DTOG_TX_Pos                      (6U)
#define USB_EP_DTOG_TX_Msk                      (0x1U << USB_EP_DTOG_TX_Pos)   /*!< 0x00000040 */
#define USB_EP_DTOG_TX                          USB_EP_DTOG_TX_Msk             /*!< EndPoint Data TOGGLE TX */
#define USB_EPTX_STAT_Pos                       (4U)
#define USB_EPTX_STAT_Msk                       (0x3U << USB_EPTX_STAT_Pos)    /*!< 0x00000030 */
#define USB_EPTX_STAT                           USB_EPTX_STAT_Msk              /*!< EndPoint TX STATus bit field */
#define USB_EPADDR_FIELD_Pos                    (0U)
#define USB_EPADDR_FIELD_Msk                    (0xFU << USB_EPADDR_FIELD_Pos) /*!< 0x0000000F */
#define USB_EPADDR_FIELD                        USB_EPADDR_FIELD_Msk           /*!< EndPoint ADDRess FIELD */

/* EndPoint REGister MASK (no toggle fields) */
#define  USB_EPREG_MASK                      (USB_EP_CTR_RX|USB_EP_SETUP|USB_EP_T_FIELD|USB_EP_KIND|USB_EP_CTR_TX|USB_EPADDR_FIELD)
                                                                           /*!< EP_TYPE[1:0] EndPoint TYPE */
#define USB_EP_TYPE_MASK_Pos                    (9U)
#define USB_EP_TYPE_MASK_Msk                    (0x3U << USB_EP_TYPE_MASK_Pos) /*!< 0x00000600 */
#define USB_EP_TYPE_MASK                        USB_EP_TYPE_MASK_Msk           /*!< EndPoint TYPE Mask */
#define USB_EP_BULK                             0x00000000U                    /*!< EndPoint BULK */
#define USB_EP_CONTROL                          0x00000200U                    /*!< EndPoint CONTROL */
#define USB_EP_ISOCHRONOUS                      0x00000400U                    /*!< EndPoint ISOCHRONOUS */
#define USB_EP_INTERRUPT                        0x00000600U                    /*!< EndPoint INTERRUPT */
#define  USB_EP_T_MASK                          (~USB_EP_T_FIELD & USB_EPREG_MASK)

#define  USB_EPKIND_MASK                        (~USB_EP_KIND & USB_EPREG_MASK)  /*!< EP_KIND EndPoint KIND */
                                                                               /*!< STAT_TX[1:0] STATus for TX transfer */
#define USB_EP_TX_DIS                           0x00000000U                    /*!< EndPoint TX DISabled */
#define USB_EP_TX_STALL                         0x00000010U                    /*!< EndPoint TX STALLed */
#define USB_EP_TX_NAK                           0x00000020U                    /*!< EndPoint TX NAKed */
#define USB_EP_TX_VALID                         0x00000030U                    /*!< EndPoint TX VALID */
#define USB_EPTX_DTOG1                          0x00000010U                    /*!< EndPoint TX Data TOGgle bit1 */
#define USB_EPTX_DTOG2                          0x00000020U                    /*!< EndPoint TX Data TOGgle bit2 */
#define  USB_EPTX_DTOGMASK  (USB_EPTX_STAT|USB_EPREG_MASK)
                                                                               /*!< STAT_RX[1:0] STATus for RX transfer */
#define USB_EP_RX_DIS                           0x00000000U                    /*!< EndPoint RX DISabled */
#define USB_EP_RX_STALL                         0x00001000U                    /*!< EndPoint RX STALLed */
#define USB_EP_RX_NAK                           0x00002000U                    /*!< EndPoint RX NAKed */
#define USB_EP_RX_VALID                         0x00003000U                    /*!< EndPoint RX VALID */
#define USB_EPRX_DTOG1                          0x00001000U                    /*!< EndPoint RX Data TOGgle bit1 */
#define USB_EPRX_DTOG2                          0x00002000U                    /*!< EndPoint RX Data TOGgle bit1 */
#define  USB_EPRX_DTOGMASK  (USB_EPRX_STAT|USB_EPREG_MASK)

/*******************  Bit definition for USB_EP0R register  *******************/
#define USB_EP0R_EA_Pos                         (0U)
#define USB_EP0R_EA_Msk                         (0xFU << USB_EP0R_EA_Pos)      /*!< 0x0000000F */
#define USB_EP0R_EA                             USB_EP0R_EA_Msk                /*!< Endpoint Address */

#define USB_EP0R_STAT_TX_Pos                    (4U)
#define USB_EP0R_STAT_TX_Msk                    (0x3U << USB_EP0R_STAT_TX_Pos) /*!< 0x00000030 */
#define USB_EP0R_STAT_TX                        USB_EP0R_STAT_TX_Msk           /*!< STAT_TX[1:0] bits (Status bits, for transmission transfers) */
#define USB_EP0R_STAT_TX_0                      (0x1U << USB_EP0R_STAT_TX_Pos) /*!< 0x00000010 */
#define USB_EP0R_STAT_TX_1                      (0x2U << USB_EP0R_STAT_TX_Pos) /*!< 0x00000020 */

#define USB_EP0R_DTOG_TX_Pos                    (6U)
#define USB_EP0R_DTOG_TX_Msk                    (0x1U << USB_EP0R_DTOG_TX_Pos) /*!< 0x00000040 */
#define USB_EP0R_DTOG_TX                        USB_EP0R_DTOG_TX_Msk           /*!< Data Toggle, for transmission transfers */
#define USB_EP0R_CTR_TX_Pos                     (7U)
#define USB_EP0R_CTR_TX_Msk                     (0x1U << USB_EP0R_CTR_TX_Pos)  /*!< 0x00000080 */
#define USB_EP0R_CTR_TX                         USB_EP0R_CTR_TX_Msk            /*!< Correct Transfer for transmission */
#define USB_EP0R_EP_KIND_Pos                    (8U)
#define USB_EP0R_EP_KIND_Msk                    (0x1U << USB_EP0R_EP_KIND_Pos) /*!< 0x00000100 */
#define USB_EP0R_EP_KIND                        USB_EP0R_EP_KIND_Msk           /*!< Endpoint Kind */

#define USB_EP0R_EP_TYPE_Pos                    (9U)
#define USB_EP0R_EP_TYPE_Msk                    (0x3U << USB_EP0R_EP_TYPE_Pos) /*!< 0x00000600 */
#define USB_EP0R_EP_TYPE                        USB_EP0R_EP_TYPE_Msk           /*!< EP_TYPE[1:0] bits (Endpoint type) */
#define USB_EP0R_EP_TYPE_0                      (0x1U << USB_EP0R_EP_TYPE_Pos) /*!< 0x00000200 */
#define USB_EP0R_EP_TYPE_1                      (0x2U << USB_EP0R_EP_TYPE_Pos) /*!< 0x00000400 */

#define USB_EP0R_SETUP_Pos                      (11U)
#define USB_EP0R_SETUP_Msk                      (0x1U << USB_EP0R_SETUP_Pos)   /*!< 0x00000800 */
#define USB_EP0R_SETUP                          USB_EP0R_SETUP_Msk             /*!< Setup transaction completed */

#define USB_EP0R_STAT_RX_Pos                    (12U)
#define USB_EP0R_STAT_RX_Msk                    (0x3U << USB_EP0R_STAT_RX_Pos) /*!< 0x00003000 */
#define USB_EP0R_STAT_RX                        USB_EP0R_STAT_RX_Msk           /*!< STAT_RX[1:0] bits (Status bits, for reception transfers) */
#define USB_EP0R_STAT_RX_0                      (0x1U << USB_EP0R_STAT_RX_Pos) /*!< 0x00001000 */
#define USB_EP0R_STAT_RX_1                      (0x2U << USB_EP0R_STAT_RX_Pos) /*!< 0x00002000 */

#define USB_EP0R_DTOG_RX_Pos                    (14U)
#define USB_EP0R_DTOG_RX_Msk                    (0x1U << USB_EP0R_DTOG_RX_Pos) /*!< 0x00004000 */
#define USB_EP0R_DTOG_RX                        USB_EP0R_DTOG_RX_Msk           /*!< Data Toggle, for reception transfers */
#define USB_EP0R_CTR_RX_Pos                     (15U)
#define USB_EP0R_CTR_RX_Msk                     (0x1U << USB_EP0R_CTR_RX_Pos)  /*!< 0x00008000 */
#define USB_EP0R_CTR_RX                         USB_EP0R_CTR_RX_Msk            /*!< Correct Transfer for reception */

/*******************  Bit definition for USB_EP1R register  *******************/
#define USB_EP1R_EA_Pos                         (0U)
#define USB_EP1R_EA_Msk                         (0xFU << USB_EP1R_EA_Pos)      /*!< 0x0000000F */
#define USB_EP1R_EA                             USB_EP1R_EA_Msk                /*!< Endpoint Address */

#define USB_EP1R_STAT_TX_Pos                    (4U)
#define USB_EP1R_STAT_TX_Msk                    (0x3U << USB_EP1R_STAT_TX_Pos) /*!< 0x00000030 */
#define USB_EP1R_STAT_TX                        USB_EP1R_STAT_TX_Msk           /*!< STAT_TX[1:0] bits (Status bits, for transmission transfers) */
#define USB_EP1R_STAT_TX_0                      (0x1U << USB_EP1R_STAT_TX_Pos) /*!< 0x00000010 */
#define USB_EP1R_STAT_TX_1                      (0x2U << USB_EP1R_STAT_TX_Pos) /*!< 0x00000020 */

#define USB_EP1R_DTOG_TX_Pos                    (6U)
#define USB_EP1R_DTOG_TX_Msk                    (0x1U << USB_EP1R_DTOG_TX_Pos) /*!< 0x00000040 */
#define USB_EP1R_DTOG_TX                        USB_EP1R_DTOG_TX_Msk           /*!< Data Toggle, for transmission transfers */
#define USB_EP1R_CTR_TX_Pos                     (7U)
#define USB_EP1R_CTR_TX_Msk                     (0x1U << USB_EP1R_CTR_TX_Pos)  /*!< 0x00000080 */
#define USB_EP1R_CTR_TX                         USB_EP1R_CTR_TX_Msk            /*!< Correct Transfer for transmission */
#define USB_EP1R_EP_KIND_Pos                    (8U)
#define USB_EP1R_EP_KIND_Msk                    (0x1U << USB_EP1R_EP_KIND_Pos) /*!< 0x00000100 */
#define USB_EP1R_EP_KIND                        USB_EP1R_EP_KIND_Msk           /*!< Endpoint Kind */

#define USB_EP1R_EP_TYPE_Pos                    (9U)
#define USB_EP1R_EP_TYPE_Msk                    (0x3U << USB_EP1R_EP_TYPE_Pos) /*!< 0x00000600 */
#define USB_EP1R_EP_TYPE                        USB_EP1R_EP_TYPE_Msk           /*!< EP_TYPE[1:0] bits (Endpoint type) */
#define USB_EP1R_EP_TYPE_0                      (0x1U << USB_EP1R_EP_TYPE_Pos) /*!< 0x00000200 */
#define USB_EP1R_EP_TYPE_1                      (0x2U << USB_EP1R_EP_TYPE_Pos) /*!< 0x00000400 */

#define USB_EP1R_SETUP_Pos                      (11U)
#define USB_EP1R_SETUP_Msk                      (0x1U << USB_EP1R_SETUP_Pos)   /*!< 0x00000800 */
#define USB_EP1R_SETUP                          USB_EP1R_SETUP_Msk             /*!< Setup transaction completed */

#define USB_EP1R_STAT_RX_Pos                    (12U)
#define USB_EP1R_STAT_RX_Msk                    (0x3U << USB_EP1R_STAT_RX_Pos) /*!< 0x00003000 */
#define USB_EP1R_STAT_RX                        USB_EP1R_STAT_RX_Msk           /*!< STAT_RX[1:0] bits (Status bits, for reception transfers) */
#define USB_EP1R_STAT_RX_0                      (0x1U << USB_EP1R_STAT_RX_Pos) /*!< 0x00001000 */
#define USB_EP1R_STAT_RX_1                      (0x2U << USB_EP1R_STAT_RX_Pos) /*!< 0x00002000 */

#define USB_EP1R_DTOG_RX_Pos                    (14U)
#define USB_EP1R_DTOG_RX_Msk                    (0x1U << USB_EP1R_DTOG_RX_Pos) /*!< 0x00004000 */
#define USB_EP1R_DTOG_RX                        USB_EP1R_DTOG_RX_Msk           /*!< Data Toggle, for reception transfers */
#define USB_EP1R_CTR_RX_Pos                     (15U)
#define USB_EP1R_CTR_RX_Msk                     (0x1U << USB_EP1R_CTR_RX_Pos)  /*!< 0x00008000 */
#define USB_EP1R_CTR_RX                         USB_EP1R_CTR_RX_Msk            /*!< Correct Transfer for reception */

/*******************  Bit definition for USB_EP2R register  *******************/
#define USB_EP2R_EA_Pos                         (0U)
#define USB_EP2R_EA_Msk                         (0xFU << USB_EP2R_EA_Pos)      /*!< 0x0000000F */
#define USB_EP2R_EA                             USB_EP2R_EA_Msk                /*!< Endpoint Address */

#define USB_EP2R_STAT_TX_Pos                    (4U)
#define USB_EP2R_STAT_TX_Msk                    (0x3U << USB_EP2R_STAT_TX_Pos) /*!< 0x00000030 */
#define USB_EP2R_STAT_TX                        USB_EP2R_STAT_TX_Msk           /*!< STAT_TX[1:0] bits (Status bits, for transmission transfers) */
#define USB_EP2R_STAT_TX_0                      (0x1U << USB_EP2R_STAT_TX_Pos) /*!< 0x00000010 */
#define USB_EP2R_STAT_TX_1                      (0x2U << USB_EP2R_STAT_TX_Pos) /*!< 0x00000020 */

#define USB_EP2R_DTOG_TX_Pos                    (6U)
#define USB_EP2R_DTOG_TX_Msk                    (0x1U << USB_EP2R_DTOG_TX_Pos) /*!< 0x00000040 */
#define USB_EP2R_DTOG_TX                        USB_EP2R_DTOG_TX_Msk           /*!< Data Toggle, for transmission transfers */
#define USB_EP2R_CTR_TX_Pos                     (7U)
#define USB_EP2R_CTR_TX_Msk                     (0x1U << USB_EP2R_CTR_TX_Pos)  /*!< 0x00000080 */
#define USB_EP2R_CTR_TX                         USB_EP2R_CTR_TX_Msk            /*!< Correct Transfer for transmission */
#define USB_EP2R_EP_KIND_Pos                    (8U)
#define USB_EP2R_EP_KIND_Msk                    (0x1U << USB_EP2R_EP_KIND_Pos) /*!< 0x00000100 */
#define USB_EP2R_EP_KIND                        USB_EP2R_EP_KIND_Msk           /*!< Endpoint Kind */

#define USB_EP2R_EP_TYPE_Pos                    (9U)
#define USB_EP2R_EP_TYPE_Msk                    (0x3U << USB_EP2R_EP_TYPE_Pos) /*!< 0x00000600 */
#define USB_EP2R_EP_TYPE                        USB_EP2R_EP_TYPE_Msk           /*!< EP_TYPE[1:0] bits (Endpoint type) */
#define USB_EP2R_EP_TYPE_0                      (0x1U << USB_EP2R_EP_TYPE_Pos) /*!< 0x00000200 */
#define USB_EP2R_EP_TYPE_1                      (0x2U << USB_EP2R_EP_TYPE_Pos) /*!< 0x00000400 */

#define USB_EP2R_SETUP_Pos                      (11U)
#define USB_EP2R_SETUP_Msk                      (0x1U << USB_EP2R_SETUP_Pos)   /*!< 0x00000800 */
#define USB_EP2R_SETUP                          USB_EP2R_SETUP_Msk             /*!< Setup transaction completed */

#define USB_EP2R_STAT_RX_Pos                    (12U)
#define USB_EP2R_STAT_RX_Msk                    (0x3U << USB_EP2R_STAT_RX_Pos) /*!< 0x00003000 */
#define USB_EP2R_STAT_RX                        USB_EP2R_STAT_RX_Msk           /*!< STAT_RX[1:0] bits (Status bits, for reception transfers) */
#define USB_EP2R_STAT_RX_0                      (0x1U << USB_EP2R_STAT_RX_Pos) /*!< 0x00001000 */
#define USB_EP2R_STAT_RX_1                      (0x2U << USB_EP2R_STAT_RX_Pos) /*!< 0x00002000 */

#define USB_EP2R_DTOG_RX_Pos                    (14U)
#define USB_EP2R_DTOG_RX_Msk                    (0x1U << USB_EP2R_DTOG_RX_Pos) /*!< 0x00004000 */
#define USB_EP2R_DTOG_RX                        USB_EP2R_DTOG_RX_Msk           /*!< Data Toggle, for reception transfers */
#define USB_EP2R_CTR_RX_Pos                     (15U)
#define USB_EP2R_CTR_RX_Msk                     (0x1U << USB_EP2R_CTR_RX_Pos)  /*!< 0x00008000 */
#define USB_EP2R_CTR_RX                         USB_EP2R_CTR_RX_Msk            /*!< Correct Transfer for reception */

/*******************  Bit definition for USB_EP3R register  *******************/
#define USB_EP3R_EA_Pos                         (0U)
#define USB_EP3R_EA_Msk                         (0xFU << USB_EP3R_EA_Pos)      /*!< 0x0000000F */
#define USB_EP3R_EA                             USB_EP3R_EA_Msk                /*!< Endpoint Address */

#define USB_EP3R_STAT_TX_Pos                    (4U)
#define USB_EP3R_STAT_TX_Msk                    (0x3U << USB_EP3R_STAT_TX_Pos) /*!< 0x00000030 */
#define USB_EP3R_STAT_TX                        USB_EP3R_STAT_TX_Msk           /*!< STAT_TX[1:0] bits (Status bits, for transmission transfers) */
#define USB_EP3R_STAT_TX_0                      (0x1U << USB_EP3R_STAT_TX_Pos) /*!< 0x00000010 */
#define USB_EP3R_STAT_TX_1                      (0x2U << USB_EP3R_STAT_TX_Pos) /*!< 0x00000020 */

#define USB_EP3R_DTOG_TX_Pos                    (6U)
#define USB_EP3R_DTOG_TX_Msk                    (0x1U << USB_EP3R_DTOG_TX_Pos) /*!< 0x00000040 */
#define USB_EP3R_DTOG_TX                        USB_EP3R_DTOG_TX_Msk           /*!< Data Toggle, for transmission transfers */
#define USB_EP3R_CTR_TX_Pos                     (7U)
#define USB_EP3R_CTR_TX_Msk                     (0x1U << USB_EP3R_CTR_TX_Pos)  /*!< 0x00000080 */
#define USB_EP3R_CTR_TX                         USB_EP3R_CTR_TX_Msk            /*!< Correct Transfer for transmission */
#define USB_EP3R_EP_KIND_Pos                    (8U)
#define USB_EP3R_EP_KIND_Msk                    (0x1U << USB_EP3R_EP_KIND_Pos) /*!< 0x00000100 */
#define USB_EP3R_EP_KIND                        USB_EP3R_EP_KIND_Msk           /*!< Endpoint Kind */

#define USB_EP3R_EP_TYPE_Pos                    (9U)
#define USB_EP3R_EP_TYPE_Msk                    (0x3U << USB_EP3R_EP_TYPE_Pos) /*!< 0x00000600 */
#define USB_EP3R_EP_TYPE                        USB_EP3R_EP_TYPE_Msk           /*!< EP_TYPE[1:0] bits (Endpoint type) */
#define USB_EP3R_EP_TYPE_0                      (0x1U << USB_EP3R_EP_TYPE_Pos) /*!< 0x00000200 */
#define USB_EP3R_EP_TYPE_1                      (0x2U << USB_EP3R_EP_TYPE_Pos) /*!< 0x00000400 */

#define USB_EP3R_SETUP_Pos                      (11U)
#define USB_EP3R_SETUP_Msk                      (0x1U << USB_EP3R_SETUP_Pos)   /*!< 0x00000800 */
#define USB_EP3R_SETUP                          USB_EP3R_SETUP_Msk             /*!< Setup transaction completed */

#define USB_EP3R_STAT_RX_Pos                    (12U)
#define USB_EP3R_STAT_RX_Msk                    (0x3U << USB_EP3R_STAT_RX_Pos) /*!< 0x00003000 */
#define USB_EP3R_STAT_RX                        USB_EP3R_STAT_RX_Msk           /*!< STAT_RX[1:0] bits (Status bits, for reception transfers) */
#define USB_EP3R_STAT_RX_0                      (0x1U << USB_EP3R_STAT_RX_Pos) /*!< 0x00001000 */
#define USB_EP3R_STAT_RX_1                      (0x2U << USB_EP3R_STAT_RX_Pos) /*!< 0x00002000 */

#define USB_EP3R_DTOG_RX_Pos                    (14U)
#define USB_EP3R_DTOG_RX_Msk                    (0x1U << USB_EP3R_DTOG_RX_Pos) /*!< 0x00004000 */
#define USB_EP3R_DTOG_RX                        USB_EP3R_DTOG_RX_Msk           /*!< Data Toggle, for reception transfers */
#define USB_EP3R_CTR_RX_Pos                     (15U)
#define USB_EP3R_CTR_RX_Msk                     (0x1U << USB_EP3R_CTR_RX_Pos)  /*!< 0x00008000 */
#define USB_EP3R_CTR_RX                         USB_EP3R_CTR_RX_Msk            /*!< Correct Transfer for reception */

/*******************  Bit definition for USB_EP4R register  *******************/
#define USB_EP4R_EA_Pos                         (0U)
#define USB_EP4R_EA_Msk                         (0xFU << USB_EP4R_EA_Pos)      /*!< 0x0000000F */
#define USB_EP4R_EA                             USB_EP4R_EA_Msk                /*!< Endpoint Address */

#define USB_EP4R_STAT_TX_Pos                    (4U)
#define USB_EP4R_STAT_TX_Msk                    (0x3U << USB_EP4R_STAT_TX_Pos) /*!< 0x00000030 */
#define USB_EP4R_STAT_TX                        USB_EP4R_STAT_TX_Msk           /*!< STAT_TX[1:0] bits (Status bits, for transmission transfers) */
#define USB_EP4R_STAT_TX_0                      (0x1U << USB_EP4R_STAT_TX_Pos) /*!< 0x00000010 */
#define USB_EP4R_STAT_TX_1                      (0x2U << USB_EP4R_STAT_TX_Pos) /*!< 0x00000020 */

#define USB_EP4R_DTOG_TX_Pos                    (6U)
#define USB_EP4R_DTOG_TX_Msk                    (0x1U << USB_EP4R_DTOG_TX_Pos) /*!< 0x00000040 */
#define USB_EP4R_DTOG_TX                        USB_EP4R_DTOG_TX_Msk           /*!< Data Toggle, for transmission transfers */
#define USB_EP4R_CTR_TX_Pos                     (7U)
#define USB_EP4R_CTR_TX_Msk                     (0x1U << USB_EP4R_CTR_TX_Pos)  /*!< 0x00000080 */
#define USB_EP4R_CTR_TX                         USB_EP4R_CTR_TX_Msk            /*!< Correct Transfer for transmission */
#define USB_EP4R_EP_KIND_Pos                    (8U)
#define USB_EP4R_EP_KIND_Msk                    (0x1U << USB_EP4R_EP_KIND_Pos) /*!< 0x00000100 */
#define USB_EP4R_EP_KIND                        USB_EP4R_EP_KIND_Msk           /*!< Endpoint Kind */

#define USB_EP4R_EP_TYPE_Pos                    (9U)
#define USB_EP4R_EP_TYPE_Msk                    (0x3U << USB_EP4R_EP_TYPE_Pos) /*!< 0x00000600 */
#define USB_EP4R_EP_TYPE                        USB_EP4R_EP_TYPE_Msk           /*!< EP_TYPE[1:0] bits (Endpoint type) */
#define USB_EP4R_EP_TYPE_0                      (0x1U << USB_EP4R_EP_TYPE_Pos) /*!< 0x00000200 */
#define USB_EP4R_EP_TYPE_1                      (0x2U << USB_EP4R_EP_TYPE_Pos) /*!< 0x00000400 */

#define USB_EP4R_SETUP_Pos                      (11U)
#define USB_EP4R_SETUP_Msk                      (0x1U << USB_EP4R_SETUP_Pos)   /*!< 0x00000800 */
#define USB_EP4R_SETUP                          USB_EP4R_SETUP_Msk             /*!< Setup transaction completed */

#define USB_EP4R_STAT_RX_Pos                    (12U)
#define USB_EP4R_STAT_RX_Msk                    (0x3U << USB_EP4R_STAT_RX_Pos) /*!< 0x00003000 */
#define USB_EP4R_STAT_RX                        USB_EP4R_STAT_RX_Msk           /*!< STAT_RX[1:0] bits (Status bits, for reception transfers) */
#define USB_EP4R_STAT_RX_0                      (0x1U << USB_EP4R_STAT_RX_Pos) /*!< 0x00001000 */
#define USB_EP4R_STAT_RX_1                      (0x2U << USB_EP4R_STAT_RX_Pos) /*!< 0x00002000 */

#define USB_EP4R_DTOG_RX_Pos                    (14U)
#define USB_EP4R_DTOG_RX_Msk                    (0x1U << USB_EP4R_DTOG_RX_Pos) /*!< 0x00004000 */
#define USB_EP4R_DTOG_RX                        USB_EP4R_DTOG_RX_Msk           /*!< Data Toggle, for reception transfers */
#define USB_EP4R_CTR_RX_Pos                     (15U)
#define USB_EP4R_CTR_RX_Msk                     (0x1U << USB_EP4R_CTR_RX_Pos)  /*!< 0x00008000 */
#define USB_EP4R_CTR_RX                         USB_EP4R_CTR_RX_Msk            /*!< Correct Transfer for reception */

/*******************  Bit definition for USB_EP5R register  *******************/
#define USB_EP5R_EA_Pos                         (0U)
#define USB_EP5R_EA_Msk                         (0xFU << USB_EP5R_EA_Pos)      /*!< 0x0000000F */
#define USB_EP5R_EA                             USB_EP5R_EA_Msk                /*!< Endpoint Address */

#define USB_EP5R_STAT_TX_Pos                    (4U)
#define USB_EP5R_STAT_TX_Msk                    (0x3U << USB_EP5R_STAT_TX_Pos) /*!< 0x00000030 */
#define USB_EP5R_STAT_TX                        USB_EP5R_STAT_TX_Msk           /*!< STAT_TX[1:0] bits (Status bits, for transmission transfers) */
#define USB_EP5R_STAT_TX_0                      (0x1U << USB_EP5R_STAT_TX_Pos) /*!< 0x00000010 */
#define USB_EP5R_STAT_TX_1                      (0x2U << USB_EP5R_STAT_TX_Pos) /*!< 0x00000020 */

#define USB_EP5R_DTOG_TX_Pos                    (6U)
#define USB_EP5R_DTOG_TX_Msk                    (0x1U << USB_EP5R_DTOG_TX_Pos) /*!< 0x00000040 */
#define USB_EP5R_DTOG_TX                        USB_EP5R_DTOG_TX_Msk           /*!< Data Toggle, for transmission transfers */
#define USB_EP5R_CTR_TX_Pos                     (7U)
#define USB_EP5R_CTR_TX_Msk                     (0x1U << USB_EP5R_CTR_TX_Pos)  /*!< 0x00000080 */
#define USB_EP5R_CTR_TX                         USB_EP5R_CTR_TX_Msk            /*!< Correct Transfer for transmission */
#define USB_EP5R_EP_KIND_Pos                    (8U)
#define USB_EP5R_EP_KIND_Msk                    (0x1U << USB_EP5R_EP_KIND_Pos) /*!< 0x00000100 */
#define USB_EP5R_EP_KIND                        USB_EP5R_EP_KIND_Msk           /*!< Endpoint Kind */

#define USB_EP5R_EP_TYPE_Pos                    (9U)
#define USB_EP5R_EP_TYPE_Msk                    (0x3U << USB_EP5R_EP_TYPE_Pos) /*!< 0x00000600 */
#define USB_EP5R_EP_TYPE                        USB_EP5R_EP_TYPE_Msk           /*!< EP_TYPE[1:0] bits (Endpoint type) */
#define USB_EP5R_EP_TYPE_0                      (0x1U << USB_EP5R_EP_TYPE_Pos) /*!< 0x00000200 */
#define USB_EP5R_EP_TYPE_1                      (0x2U << USB_EP5R_EP_TYPE_Pos) /*!< 0x00000400 */

#define USB_EP5R_SETUP_Pos                      (11U)
#define USB_EP5R_SETUP_Msk                      (0x1U << USB_EP5R_SETUP_Pos)   /*!< 0x00000800 */
#define USB_EP5R_SETUP                          USB_EP5R_SETUP_Msk             /*!< Setup transaction completed */

#define USB_EP5R_STAT_RX_Pos                    (12U)
#define USB_EP5R_STAT_RX_Msk                    (0x3U << USB_EP5R_STAT_RX_Pos) /*!< 0x00003000 */
#define USB_EP5R_STAT_RX                        USB_EP5R_STAT_RX_Msk           /*!< STAT_RX[1:0] bits (Status bits, for reception transfers) */
#define USB_EP5R_STAT_RX_0                      (0x1U << USB_EP5R_STAT_RX_Pos) /*!< 0x00001000 */
#define USB_EP5R_STAT_RX_1                      (0x2U << USB_EP5R_STAT_RX_Pos) /*!< 0x00002000 */

#define USB_EP5R_DTOG_RX_Pos                    (14U)
#define USB_EP5R_DTOG_RX_Msk                    (0x1U << USB_EP5R_DTOG_RX_Pos) /*!< 0x00004000 */
#define USB_EP5R_DTOG_RX                        USB_EP5R_DTOG_RX_Msk           /*!< Data Toggle, for reception transfers */
#define USB_EP5R_CTR_RX_Pos                     (15U)
#define USB_EP5R_CTR_RX_Msk                     (0x1U << USB_EP5R_CTR_RX_Pos)  /*!< 0x00008000 */
#define USB_EP5R_CTR_RX                         USB_EP5R_CTR_RX_Msk            /*!< Correct Transfer for reception */

/*******************  Bit definition for USB_EP6R register  *******************/
#define USB_EP6R_EA_Pos                         (0U)
#define USB_EP6R_EA_Msk                         (0xFU << USB_EP6R_EA_Pos)      /*!< 0x0000000F */
#define USB_EP6R_EA                             USB_EP6R_EA_Msk                /*!< Endpoint Address */

#define USB_EP6R_STAT_TX_Pos                    (4U)
#define USB_EP6R_STAT_TX_Msk                    (0x3U << USB_EP6R_STAT_TX_Pos) /*!< 0x00000030 */
#define USB_EP6R_STAT_TX                        USB_EP6R_STAT_TX_Msk           /*!< STAT_TX[1:0] bits (Status bits, for transmission transfers) */
#define USB_EP6R_STAT_TX_0                      (0x1U << USB_EP6R_STAT_TX_Pos) /*!< 0x00000010 */
#define USB_EP6R_STAT_TX_1                      (0x2U << USB_EP6R_STAT_TX_Pos) /*!< 0x00000020 */

#define USB_EP6R_DTOG_TX_Pos                    (6U)
#define USB_EP6R_DTOG_TX_Msk                    (0x1U << USB_EP6R_DTOG_TX_Pos) /*!< 0x00000040 */
#define USB_EP6R_DTOG_TX                        USB_EP6R_DTOG_TX_Msk           /*!< Data Toggle, for transmission transfers */
#define USB_EP6R_CTR_TX_Pos                     (7U)
#define USB_EP6R_CTR_TX_Msk                     (0x1U << USB_EP6R_CTR_TX_Pos)  /*!< 0x00000080 */
#define USB_EP6R_CTR_TX                         USB_EP6R_CTR_TX_Msk            /*!< Correct Transfer for transmission */
#define USB_EP6R_EP_KIND_Pos                    (8U)
#define USB_EP6R_EP_KIND_Msk                    (0x1U << USB_EP6R_EP_KIND_Pos) /*!< 0x00000100 */
#define USB_EP6R_EP_KIND                        USB_EP6R_EP_KIND_Msk           /*!< Endpoint Kind */

#define USB_EP6R_EP_TYPE_Pos                    (9U)
#define USB_EP6R_EP_TYPE_Msk                    (0x3U << USB_EP6R_EP_TYPE_Pos) /*!< 0x00000600 */
#define USB_EP6R_EP_TYPE                        USB_EP6R_EP_TYPE_Msk           /*!< EP_TYPE[1:0] bits (Endpoint type) */
#define USB_EP6R_EP_TYPE_0                      (0x1U << USB_EP6R_EP_TYPE_Pos) /*!< 0x00000200 */
#define USB_EP6R_EP_TYPE_1                      (0x2U << USB_EP6R_EP_TYPE_Pos) /*!< 0x00000400 */

#define USB_EP6R_SETUP_Pos                      (11U)
#define USB_EP6R_SETUP_Msk                      (0x1U << USB_EP6R_SETUP_Pos)   /*!< 0x00000800 */
#define USB_EP6R_SETUP                          USB_EP6R_SETUP_Msk             /*!< Setup transaction completed */

#define USB_EP6R_STAT_RX_Pos                    (12U)
#define USB_EP6R_STAT_RX_Msk                    (0x3U << USB_EP6R_STAT_RX_Pos) /*!< 0x00003000 */
#define USB_EP6R_STAT_RX                        USB_EP6R_STAT_RX_Msk           /*!< STAT_RX[1:0] bits (Status bits, for reception transfers) */
#define USB_EP6R_STAT_RX_0                      (0x1U << USB_EP6R_STAT_RX_Pos) /*!< 0x00001000 */
#define USB_EP6R_STAT_RX_1                      (0x2U << USB_EP6R_STAT_RX_Pos) /*!< 0x00002000 */

#define USB_EP6R_DTOG_RX_Pos                    (14U)
#define USB_EP6R_DTOG_RX_Msk                    (0x1U << USB_EP6R_DTOG_RX_Pos) /*!< 0x00004000 */
#define USB_EP6R_DTOG_RX                        USB_EP6R_DTOG_RX_Msk           /*!< Data Toggle, for reception transfers */
#define USB_EP6R_CTR_RX_Pos                     (15U)
#define USB_EP6R_CTR_RX_Msk                     (0x1U << USB_EP6R_CTR_RX_Pos)  /*!< 0x00008000 */
#define USB_EP6R_CTR_RX                         USB_EP6R_CTR_RX_Msk            /*!< Correct Transfer for reception */

/*******************  Bit definition for USB_EP7R register  *******************/
#define USB_EP7R_EA_Pos                         (0U)
#define USB_EP7R_EA_Msk                         (0xFU << USB_EP7R_EA_Pos)      /*!< 0x0000000F */
#define USB_EP7R_EA                             USB_EP7R_EA_Msk                /*!< Endpoint Address */

#define USB_EP7R_STAT_TX_Pos                    (4U)
#define USB_EP7R_STAT_TX_Msk                    (0x3U << USB_EP7R_STAT_TX_Pos) /*!< 0x00000030 */
#define USB_EP7R_STAT_TX                        USB_EP7R_STAT_TX_Msk           /*!< STAT_TX[1:0] bits (Status bits, for transmission transfers) */
#define USB_EP7R_STAT_TX_0                      (0x1U << USB_EP7R_STAT_TX_Pos) /*!< 0x00000010 */
#define USB_EP7R_STAT_TX_1                      (0x2U << USB_EP7R_STAT_TX_Pos) /*!< 0x00000020 */

#define USB_EP7R_DTOG_TX_Pos                    (6U)
#define USB_EP7R_DTOG_TX_Msk                    (0x1U << USB_EP7R_DTOG_TX_Pos) /*!< 0x00000040 */
#define USB_EP7R_DTOG_TX                        USB_EP7R_DTOG_TX_Msk           /*!< Data Toggle, for transmission transfers */
#define USB_EP7R_CTR_TX_Pos                     (7U)
#define USB_EP7R_CTR_TX_Msk                     (0x1U << USB_EP7R_CTR_TX_Pos)  /*!< 0x00000080 */
#define USB_EP7R_CTR_TX                         USB_EP7R_CTR_TX_Msk            /*!< Correct Transfer for transmission */
#define USB_EP7R_EP_KIND_Pos                    (8U)
#define USB_EP7R_EP_KIND_Msk                    (0x1U << USB_EP7R_EP_KIND_Pos) /*!< 0x00000100 */
#define USB_EP7R_EP_KIND                        USB_EP7R_EP_KIND_Msk           /*!< Endpoint Kind */

#define USB_EP7R_EP_TYPE_Pos                    (9U)
#define USB_EP7R_EP_TYPE_Msk                    (0x3U << USB_EP7R_EP_TYPE_Pos) /*!< 0x00000600 */
#define USB_EP7R_EP_TYPE                        USB_EP7R_EP_TYPE_Msk           /*!< EP_TYPE[1:0] bits (Endpoint type) */
#define USB_EP7R_EP_TYPE_0                      (0x1U << USB_EP7R_EP_TYPE_Pos) /*!< 0x00000200 */
#define USB_EP7R_EP_TYPE_1                      (0x2U << USB_EP7R_EP_TYPE_Pos) /*!< 0x00000400 */

#define USB_EP7R_SETUP_Pos                      (11U)
#define USB_EP7R_SETUP_Msk                      (0x1U << USB_EP7R_SETUP_Pos)   /*!< 0x00000800 */
#define USB_EP7R_SETUP                          USB_EP7R_SETUP_Msk             /*!< Setup transaction completed */

#define USB_EP7R_STAT_RX_Pos                    (12U)
#define USB_EP7R_STAT_RX_Msk                    (0x3U << USB_EP7R_STAT_RX_Pos) /*!< 0x00003000 */
#define USB_EP7R_STAT_RX                        USB_EP7R_STAT_RX_Msk           /*!< STAT_RX[1:0] bits (Status bits, for reception transfers) */
#define USB_EP7R_STAT_RX_0                      (0x1U << USB_EP7R_STAT_RX_Pos) /*!< 0x00001000 */
#define USB_EP7R_STAT_RX_1                      (0x2U << USB_EP7R_STAT_RX_Pos) /*!< 0x00002000 */

#define USB_EP7R_DTOG_RX_Pos                    (14U)
#define USB_EP7R_DTOG_RX_Msk                    (0x1U << USB_EP7R_DTOG_RX_Pos) /*!< 0x00004000 */
#define USB_EP7R_DTOG_RX                        USB_EP7R_DTOG_RX_Msk           /*!< Data Toggle, for reception transfers */
#define USB_EP7R_CTR_RX_Pos                     (15U)
#define USB_EP7R_CTR_RX_Msk                     (0x1U << USB_EP7R_CTR_RX_Pos)  /*!< 0x00008000 */
#define USB_EP7R_CTR_RX                         USB_EP7R_CTR_RX_Msk            /*!< Correct Transfer for reception */

/*!< Common registers */
/*******************  Bit definition for USB_CNTR register  *******************/
#define USB_CNTR_FRES_Pos                       (0U)
#define USB_CNTR_FRES_Msk                       (0x1U << USB_CNTR_FRES_Pos)    /*!< 0x00000001 */
#define USB_CNTR_FRES                           USB_CNTR_FRES_Msk              /*!< Force USB Reset */
#define USB_CNTR_PDWN_Pos                       (1U)
#define USB_CNTR_PDWN_Msk                       (0x1U << USB_CNTR_PDWN_Pos)    /*!< 0x00000002 */
#define USB_CNTR_PDWN                           USB_CNTR_PDWN_Msk              /*!< Power down */
#define USB_CNTR_LP_MODE_Pos                    (2U)
#define USB_CNTR_LP_MODE_Msk                    (0x1U << USB_CNTR_LP_MODE_Pos) /*!< 0x00000004 */
#define USB_CNTR_LP_MODE                        USB_CNTR_LP_MODE_Msk           /*!< Low-power mode */
#define USB_CNTR_FSUSP_Pos                      (3U)
#define USB_CNTR_FSUSP_Msk                      (0x1U << USB_CNTR_FSUSP_Pos)   /*!< 0x00000008 */
#define USB_CNTR_FSUSP                          USB_CNTR_FSUSP_Msk             /*!< Force suspend */
#define USB_CNTR_RESUME_Pos                     (4U)
#define USB_CNTR_RESUME_Msk                     (0x1U << USB_CNTR_RESUME_Pos)  /*!< 0x00000010 */
#define USB_CNTR_RESUME                         USB_CNTR_RESUME_Msk            /*!< Resume request */
#define USB_CNTR_ESOFM_Pos                      (8U)
#define USB_CNTR_ESOFM_Msk                      (0x1U << USB_CNTR_ESOFM_Pos)   /*!< 0x00000100 */
#define USB_CNTR_ESOFM                          USB_CNTR_ESOFM_Msk             /*!< Expected Start Of Frame Interrupt Mask */
#define USB_CNTR_SOFM_Pos                       (9U)
#define USB_CNTR_SOFM_Msk                       (0x1U << USB_CNTR_SOFM_Pos)    /*!< 0x00000200 */
#define USB_CNTR_SOFM                           USB_CNTR_SOFM_Msk              /*!< Start Of Frame Interrupt Mask */
#define USB_CNTR_RESETM_Pos                     (10U)
#define USB_CNTR_RESETM_Msk                     (0x1U << USB_CNTR_RESETM_Pos)  /*!< 0x00000400 */
#define USB_CNTR_RESETM                         USB_CNTR_RESETM_Msk            /*!< RESET Interrupt Mask */
#define USB_CNTR_SUSPM_Pos                      (11U)
#define USB_CNTR_SUSPM_Msk                      (0x1U << USB_CNTR_SUSPM_Pos)   /*!< 0x00000800 */
#define USB_CNTR_SUSPM                          USB_CNTR_SUSPM_Msk             /*!< Suspend mode Interrupt Mask */
#define USB_CNTR_WKUPM_Pos                      (12U)
#define USB_CNTR_WKUPM_Msk                      (0x1U << USB_CNTR_WKUPM_Pos)   /*!< 0x00001000 */
#define USB_CNTR_WKUPM                          USB_CNTR_WKUPM_Msk             /*!< Wakeup Interrupt Mask */
#define USB_CNTR_ERRM_Pos                       (13U)
#define USB_CNTR_ERRM_Msk                       (0x1U << USB_CNTR_ERRM_Pos)    /*!< 0x00002000 */
#define USB_CNTR_ERRM                           USB_CNTR_ERRM_Msk              /*!< Error Interrupt Mask */
#define USB_CNTR_PMAOVRM_Pos                    (14U)
#define USB_CNTR_PMAOVRM_Msk                    (0x1U << USB_CNTR_PMAOVRM_Pos) /*!< 0x00004000 */
#define USB_CNTR_PMAOVRM                        USB_CNTR_PMAOVRM_Msk           /*!< Packet Memory Area Over / Underrun Interrupt Mask */
#define USB_CNTR_CTRM_Pos                       (15U)
#define USB_CNTR_CTRM_Msk                       (0x1U << USB_CNTR_CTRM_Pos)    /*!< 0x00008000 */
#define USB_CNTR_CTRM                           USB_CNTR_CTRM_Msk              /*!< Correct Transfer Interrupt Mask */

/*******************  Bit definition for USB_ISTR register  *******************/
#define USB_ISTR_EP_ID_Pos                      (0U)
#define USB_ISTR_EP_ID_Msk                      (0xFU << USB_ISTR_EP_ID_Pos)   /*!< 0x0000000F */
#define USB_ISTR_EP_ID                          USB_ISTR_EP_ID_Msk             /*!< Endpoint Identifier */
#define USB_ISTR_DIR_Pos                        (4U)
#define USB_ISTR_DIR_Msk                        (0x1U << USB_ISTR_DIR_Pos)     /*!< 0x00000010 */
#define USB_ISTR_DIR                            USB_ISTR_DIR_Msk               /*!< Direction of transaction */
#define USB_ISTR_ESOF_Pos                       (8U)
#define USB_ISTR_ESOF_Msk                       (0x1U << USB_ISTR_ESOF_Pos)    /*!< 0x00000100 */
#define USB_ISTR_ESOF                           USB_ISTR_ESOF_Msk              /*!< Expected Start Of Frame */
#define USB_ISTR_SOF_Pos                        (9U)
#define USB_ISTR_SOF_Msk                        (0x1U << USB_ISTR_SOF_Pos)     /*!< 0x00000200 */
#define USB_ISTR_SOF                            USB_ISTR_SOF_Msk               /*!< Start Of Frame */
#define USB_ISTR_RESET_Pos                      (10U)
#define USB_ISTR_RESET_Msk                      (0x1U << USB_ISTR_RESET_Pos)   /*!< 0x00000400 */
#define USB_ISTR_RESET                          USB_ISTR_RESET_Msk             /*!< USB RESET request */
#define USB_ISTR_SUSP_Pos                       (11U)
#define USB_ISTR_SUSP_Msk                       (0x1U << USB_ISTR_SUSP_Pos)    /*!< 0x00000800 */
#define USB_ISTR_SUSP                           USB_ISTR_SUSP_Msk              /*!< Suspend mode request */
#define USB_ISTR_WKUP_Pos                       (12U)
#define USB_ISTR_WKUP_Msk                       (0x1U << USB_ISTR_WKUP_Pos)    /*!< 0x00001000 */
#define USB_ISTR_WKUP                           USB_ISTR_WKUP_Msk              /*!< Wake up */
#define USB_ISTR_ERR_Pos                        (13U)
#define USB_ISTR_ERR_Msk                        (0x1U << USB_ISTR_ERR_Pos)     /*!< 0x00002000 */
#define USB_ISTR_ERR                            USB_ISTR_ERR_Msk               /*!< Error */
#define USB_ISTR_PMAOVR_Pos                     (14U)
#define USB_ISTR_PMAOVR_Msk                     (0x1U << USB_ISTR_PMAOVR_Pos)  /*!< 0x00004000 */
#define USB_ISTR_PMAOVR                         USB_ISTR_PMAOVR_Msk            /*!< Packet Memory Area Over / Underrun */
#define USB_ISTR_CTR_Pos                        (15U)
#define USB_ISTR_CTR_Msk                        (0x1U << USB_ISTR_CTR_Pos)     /*!< 0x00008000 */
#define USB_ISTR_CTR                            USB_ISTR_CTR_Msk               /*!< Correct Transfer */

/*******************  Bit definition for USB_FNR register  ********************/
#define USB_FNR_FN_Pos                          (0U)
#define USB_FNR_FN_Msk                          (0x7FFU << USB_FNR_FN_Pos)     /*!< 0x000007FF */
#define USB_FNR_FN                              USB_FNR_FN_Msk                 /*!< Frame Number */
#define USB_FNR_LSOF_Pos                        (11U)
#define USB_FNR_LSOF_Msk                        (0x3U << USB_FNR_LSOF_Pos)     /*!< 0x00001800 */
#define USB_FNR_LSOF                            USB_FNR_LSOF_Msk               /*!< Lost SOF */
#define USB_FNR_LCK_Pos                         (13U)
#define USB_FNR_LCK_Msk                         (0x1U << USB_FNR_LCK_Pos)      /*!< 0x00002000 */
#define USB_FNR_LCK                             USB_FNR_LCK_Msk                /*!< Locked */
#define USB_FNR_RXDM_Pos                        (14U)
#define USB_FNR_RXDM_Msk                        (0x1U << USB_FNR_RXDM_Pos)     /*!< 0x00004000 */
#define USB_FNR_RXDM                            USB_FNR_RXDM_Msk               /*!< Receive Data - Line Status */
#define USB_FNR_RXDP_Pos                        (15U)
#define USB_FNR_RXDP_Msk                        (0x1U << USB_FNR_RXDP_Pos)     /*!< 0x00008000 */
#define USB_FNR_RXDP                            USB_FNR_RXDP_Msk               /*!< Receive Data + Line Status */

/******************  Bit definition for USB_DADDR register  *******************/
#define USB_DADDR_ADD_Pos                       (0U)
#define USB_DADDR_ADD_Msk                       (0x7FU << USB_DADDR_ADD_Pos)   /*!< 0x0000007F */
#define USB_DADDR_ADD                           USB_DADDR_ADD_Msk              /*!< ADD[6:0] bits (Device Address) */
#define USB_DADDR_ADD0_Pos                      (0U)
#define USB_DADDR_ADD0_Msk                      (0x1U << USB_DADDR_ADD0_Pos)   /*!< 0x00000001 */
#define USB_DADDR_ADD0                          USB_DADDR_ADD0_Msk             /*!< Bit 0 */
#define USB_DADDR_ADD1_Pos                      (1U)
#define USB_DADDR_ADD1_Msk                      (0x1U << USB_DADDR_ADD1_Pos)   /*!< 0x00000002 */
#define USB_DADDR_ADD1                          USB_DADDR_ADD1_Msk             /*!< Bit 1 */
#define USB_DADDR_ADD2_Pos                      (2U)
#define USB_DADDR_ADD2_Msk                      (0x1U << USB_DADDR_ADD2_Pos)   /*!< 0x00000004 */
#define USB_DADDR_ADD2                          USB_DADDR_ADD2_Msk             /*!< Bit 2 */
#define USB_DADDR_ADD3_Pos                      (3U)
#define USB_DADDR_ADD3_Msk                      (0x1U << USB_DADDR_ADD3_Pos)   /*!< 0x00000008 */
#define USB_DADDR_ADD3                          USB_DADDR_ADD3_Msk             /*!< Bit 3 */
#define USB_DADDR_ADD4_Pos                      (4U)
#define USB_DADDR_ADD4_Msk                      (0x1U << USB_DADDR_ADD4_Pos)   /*!< 0x00000010 */
#define USB_DADDR_ADD4                          USB_DADDR_ADD4_Msk             /*!< Bit 4 */
#define USB_DADDR_ADD5_Pos                      (5U)
#define USB_DADDR_ADD5_Msk                      (0x1U << USB_DADDR_ADD5_Pos)   /*!< 0x00000020 */
#define USB_DADDR_ADD5                          USB_DADDR_ADD5_Msk             /*!< Bit 5 */
#define USB_DADDR_ADD6_Pos                      (6U)
#define USB_DADDR_ADD6_Msk                      (0x1U << USB_DADDR_ADD6_Pos)   /*!< 0x00000040 */
#define USB_DADDR_ADD6                          USB_DADDR_ADD6_Msk             /*!< Bit 6 */

#define USB_DADDR_EF_Pos                        (7U)
#define USB_DADDR_EF_Msk                        (0x1U << USB_DADDR_EF_Pos)     /*!< 0x00000080 */
#define USB_DADDR_EF                            USB_DADDR_EF_Msk               /*!< Enable Function */

/******************  Bit definition for USB_BTABLE register  ******************/
#define USB_BTABLE_BTABLE_Pos                   (3U)
#define USB_BTABLE_BTABLE_Msk                   (0x1FFFU << USB_BTABLE_BTABLE_Pos) /*!< 0x0000FFF8 */
#define USB_BTABLE_BTABLE                       USB_BTABLE_BTABLE_Msk          /*!< Buffer Table */

/*!< Buffer descriptor table */
/*****************  Bit definition for USB_ADDR0_TX register  *****************/
#define USB_ADDR0_TX_ADDR0_TX_Pos               (1U)
#define USB_ADDR0_TX_ADDR0_TX_Msk               (0x7FFFU << USB_ADDR0_TX_ADDR0_TX_Pos) /*!< 0x0000FFFE */
#define USB_ADDR0_TX_ADDR0_TX                   USB_ADDR0_TX_ADDR0_TX_Msk      /*!< Transmission Buffer Address 0 */

/*****************  Bit definition for USB_ADDR1_TX register  *****************/
#define USB_ADDR1_TX_ADDR1_TX_Pos               (1U)
#define USB_ADDR1_TX_ADDR1_TX_Msk               (0x7FFFU << USB_ADDR1_TX_ADDR1_TX_Pos) /*!< 0x0000FFFE */
#define USB_ADDR1_TX_ADDR1_TX                   USB_ADDR1_TX_ADDR1_TX_Msk      /*!< Transmission Buffer Address 1 */

/*****************  Bit definition for USB_ADDR2_TX register  *****************/
#define USB_ADDR2_TX_ADDR2_TX_Pos               (1U)
#define USB_ADDR2_TX_ADDR2_TX_Msk               (0x7FFFU << USB_ADDR2_TX_ADDR2_TX_Pos) /*!< 0x0000FFFE */
#define USB_ADDR2_TX_ADDR2_TX                   USB_ADDR2_TX_ADDR2_TX_Msk      /*!< Transmission Buffer Address 2 */

/*****************  Bit definition for USB_ADDR3_TX register  *****************/
#define USB_ADDR3_TX_ADDR3_TX_Pos               (1U)
#define USB_ADDR3_TX_ADDR3_TX_Msk               (0x7FFFU << USB_ADDR3_TX_ADDR3_TX_Pos) /*!< 0x0000FFFE */
#define USB_ADDR3_TX_ADDR3_TX                   USB_ADDR3_TX_ADDR3_TX_Msk      /*!< Transmission Buffer Address 3 */

/*****************  Bit definition for USB_ADDR4_TX register  *****************/
#define USB_ADDR4_TX_ADDR4_TX_Pos               (1U)
#define USB_ADDR4_TX_ADDR4_TX_Msk               (0x7FFFU << USB_ADDR4_TX_ADDR4_TX_Pos) /*!< 0x0000FFFE */
#define USB_ADDR4_TX_ADDR4_TX                   USB_ADDR4_TX_ADDR4_TX_Msk      /*!< Transmission Buffer Address 4 */

/*****************  Bit definition for USB_ADDR5_TX register  *****************/
#define USB_ADDR5_TX_ADDR5_TX_Pos               (1U)
#define USB_ADDR5_TX_ADDR5_TX_Msk               (0x7FFFU << USB_ADDR5_TX_ADDR5_TX_Pos) /*!< 0x0000FFFE */
#define USB_ADDR5_TX_ADDR5_TX                   USB_ADDR5_TX_ADDR5_TX_Msk      /*!< Transmission Buffer Address 5 */

/*****************  Bit definition for USB_ADDR6_TX register  *****************/
#define USB_ADDR6_TX_ADDR6_TX_Pos               (1U)
#define USB_ADDR6_TX_ADDR6_TX_Msk               (0x7FFFU << USB_ADDR6_TX_ADDR6_TX_Pos) /*!< 0x0000FFFE */
#define USB_ADDR6_TX_ADDR6_TX                   USB_ADDR6_TX_ADDR6_TX_Msk      /*!< Transmission Buffer Address 6 */

/*****************  Bit definition for USB_ADDR7_TX register  *****************/
#define USB_ADDR7_TX_ADDR7_TX_Pos               (1U)
#define USB_ADDR7_TX_ADDR7_TX_Msk               (0x7FFFU << USB_ADDR7_TX_ADDR7_TX_Pos) /*!< 0x0000FFFE */
#define USB_ADDR7_TX_ADDR7_TX                   USB_ADDR7_TX_ADDR7_TX_Msk      /*!< Transmission Buffer Address 7 */

/*----------------------------------------------------------------------------*/

/*****************  Bit definition for USB_COUNT0_TX register  ****************/
#define USB_COUNT0_TX_COUNT0_TX_Pos             (0U)
#define USB_COUNT0_TX_COUNT0_TX_Msk             (0x3FFU << USB_COUNT0_TX_COUNT0_TX_Pos) /*!< 0x000003FF */
#define USB_COUNT0_TX_COUNT0_TX                 USB_COUNT0_TX_COUNT0_TX_Msk    /*!< Transmission Byte Count 0 */

/*****************  Bit definition for USB_COUNT1_TX register  ****************/
#define USB_COUNT1_TX_COUNT1_TX_Pos             (0U)
#define USB_COUNT1_TX_COUNT1_TX_Msk             (0x3FFU << USB_COUNT1_TX_COUNT1_TX_Pos) /*!< 0x000003FF */
#define USB_COUNT1_TX_COUNT1_TX                 USB_COUNT1_TX_COUNT1_TX_Msk    /*!< Transmission Byte Count 1 */

/*****************  Bit definition for USB_COUNT2_TX register  ****************/
#define USB_COUNT2_TX_COUNT2_TX_Pos             (0U)
#define USB_COUNT2_TX_COUNT2_TX_Msk             (0x3FFU << USB_COUNT2_TX_COUNT2_TX_Pos) /*!< 0x000003FF */
#define USB_COUNT2_TX_COUNT2_TX                 USB_COUNT2_TX_COUNT2_TX_Msk    /*!< Transmission Byte Count 2 */

/*****************  Bit definition for USB_COUNT3_TX register  ****************/
#define USB_COUNT3_TX_COUNT3_TX_Pos             (0U)
#define USB_COUNT3_TX_COUNT3_TX_Msk             (0x3FFU << USB_COUNT3_TX_COUNT3_TX_Pos) /*!< 0x000003FF */
#define USB_COUNT3_TX_COUNT3_TX                 USB_COUNT3_TX_COUNT3_TX_Msk    /*!< Transmission Byte Count 3 */

/*****************  Bit definition for USB_COUNT4_TX register  ****************/
#define USB_COUNT4_TX_COUNT4_TX_Pos             (0U)
#define USB_COUNT4_TX_COUNT4_TX_Msk             (0x3FFU << USB_COUNT4_TX_COUNT4_TX_Pos) /*!< 0x000003FF */
#define USB_COUNT4_TX_COUNT4_TX                 USB_COUNT4_TX_COUNT4_TX_Msk    /*!< Transmission Byte Count 4 */

/*****************  Bit definition for USB_COUNT5_TX register  ****************/
#define USB_COUNT5_TX_COUNT5_TX_Pos             (0U)
#define USB_COUNT5_TX_COUNT5_TX_Msk             (0x3FFU << USB_COUNT5_TX_COUNT5_TX_Pos) /*!< 0x000003FF */
#define USB_COUNT5_TX_COUNT5_TX                 USB_COUNT5_TX_COUNT5_TX_Msk    /*!< Transmission Byte Count 5 */

/*****************  Bit definition for USB_COUNT6_TX register  ****************/
#define USB_COUNT6_TX_COUNT6_TX_Pos             (0U)
#define USB_COUNT6_TX_COUNT6_TX_Msk             (0x3FFU << USB_COUNT6_TX_COUNT6_TX_Pos) /*!< 0x000003FF */
#define USB_COUNT6_TX_COUNT6_TX                 USB_COUNT6_TX_COUNT6_TX_Msk    /*!< Transmission Byte Count 6 */

/*****************  Bit definition for USB_COUNT7_TX register  ****************/
#define USB_COUNT7_TX_COUNT7_TX_Pos             (0U)
#define USB_COUNT7_TX_COUNT7_TX_Msk             (0x3FFU << USB_COUNT7_TX_COUNT7_TX_Pos) /*!< 0x000003FF */
#define USB_COUNT7_TX_COUNT7_TX                 USB_COUNT7_TX_COUNT7_TX_Msk    /*!< Transmission Byte Count 7 */

/*----------------------------------------------------------------------------*/

/****************  Bit definition for USB_COUNT0_TX_0 register  ***************/
#define USB_COUNT0_TX_0_COUNT0_TX_0             0x000003FFU         /*!< Transmission Byte Count 0 (low) */

/****************  Bit definition for USB_COUNT0_TX_1 register  ***************/
#define USB_COUNT0_TX_1_COUNT0_TX_1             0x03FF0000U         /*!< Transmission Byte Count 0 (high) */

/****************  Bit definition for USB_COUNT1_TX_0 register  ***************/
#define USB_COUNT1_TX_0_COUNT1_TX_0             0x000003FFU         /*!< Transmission Byte Count 1 (low) */

/****************  Bit definition for USB_COUNT1_TX_1 register  ***************/
#define USB_COUNT1_TX_1_COUNT1_TX_1             0x03FF0000U         /*!< Transmission Byte Count 1 (high) */

/****************  Bit definition for USB_COUNT2_TX_0 register  ***************/
#define USB_COUNT2_TX_0_COUNT2_TX_0             0x000003FFU         /*!< Transmission Byte Count 2 (low) */

/****************  Bit definition for USB_COUNT2_TX_1 register  ***************/
#define USB_COUNT2_TX_1_COUNT2_TX_1             0x03FF0000U         /*!< Transmission Byte Count 2 (high) */

/****************  Bit definition for USB_COUNT3_TX_0 register  ***************/
#define USB_COUNT3_TX_0_COUNT3_TX_0             0x000003FFU         /*!< Transmission Byte Count 3 (low) */

/****************  Bit definition for USB_COUNT3_TX_1 register  ***************/
#define USB_COUNT3_TX_1_COUNT3_TX_1             0x03FF0000U         /*!< Transmission Byte Count 3 (high) */

/****************  Bit definition for USB_COUNT4_TX_0 register  ***************/
#define USB_COUNT4_TX_0_COUNT4_TX_0             0x000003FFU         /*!< Transmission Byte Count 4 (low) */

/****************  Bit definition for USB_COUNT4_TX_1 register  ***************/
#define USB_COUNT4_TX_1_COUNT4_TX_1             0x03FF0000U         /*!< Transmission Byte Count 4 (high) */

/****************  Bit definition for USB_COUNT5_TX_0 register  ***************/
#define USB_COUNT5_TX_0_COUNT5_TX_0             0x000003FFU         /*!< Transmission Byte Count 5 (low) */

/****************  Bit definition for USB_COUNT5_TX_1 register  ***************/
#define USB_COUNT5_TX_1_COUNT5_TX_1             0x03FF0000U         /*!< Transmission Byte Count 5 (high) */

/****************  Bit definition for USB_COUNT6_TX_0 register  ***************/
#define USB_COUNT6_TX_0_COUNT6_TX_0             0x000003FFU         /*!< Transmission Byte Count 6 (low) */

/****************  Bit definition for USB_COUNT6_TX_1 register  ***************/
#define USB_COUNT6_TX_1_COUNT6_TX_1             0x03FF0000U         /*!< Transmission Byte Count 6 (high) */

/****************  Bit definition for USB_COUNT7_TX_0 register  ***************/
#define USB_COUNT7_TX_0_COUNT7_TX_0             0x000003FFU         /*!< Transmission Byte Count 7 (low) */

/****************  Bit definition for USB_COUNT7_TX_1 register  ***************/
#define USB_COUNT7_TX_1_COUNT7_TX_1             0x03FF0000U         /*!< Transmission Byte Count 7 (high) */

/*----------------------------------------------------------------------------*/

/*****************  Bit definition for USB_ADDR0_RX register  *****************/
#define USB_ADDR0_RX_ADDR0_RX_Pos               (1U)
#define USB_ADDR0_RX_ADDR0_RX_Msk               (0x7FFFU << USB_ADDR0_RX_ADDR0_RX_Pos) /*!< 0x0000FFFE */
#define USB_ADDR0_RX_ADDR0_RX                   USB_ADDR0_RX_ADDR0_RX_Msk      /*!< Reception Buffer Address 0 */

/*****************  Bit definition for USB_ADDR1_RX register  *****************/
#define USB_ADDR1_RX_ADDR1_RX_Pos               (1U)
#define USB_ADDR1_RX_ADDR1_RX_Msk               (0x7FFFU << USB_ADDR1_RX_ADDR1_RX_Pos) /*!< 0x0000FFFE */
#define USB_ADDR1_RX_ADDR1_RX                   USB_ADDR1_RX_ADDR1_RX_Msk      /*!< Reception Buffer Address 1 */

/*****************  Bit definition for USB_ADDR2_RX register  *****************/
#define USB_ADDR2_RX_ADDR2_RX_Pos               (1U)
#define USB_ADDR2_RX_ADDR2_RX_Msk               (0x7FFFU << USB_ADDR2_RX_ADDR2_RX_Pos) /*!< 0x0000FFFE */
#define USB_ADDR2_RX_ADDR2_RX                   USB_ADDR2_RX_ADDR2_RX_Msk      /*!< Reception Buffer Address 2 */

/*****************  Bit definition for USB_ADDR3_RX register  *****************/
#define USB_ADDR3_RX_ADDR3_RX_Pos               (1U)
#define USB_ADDR3_RX_ADDR3_RX_Msk               (0x7FFFU << USB_ADDR3_RX_ADDR3_RX_Pos) /*!< 0x0000FFFE */
#define USB_ADDR3_RX_ADDR3_RX                   USB_ADDR3_RX_ADDR3_RX_Msk      /*!< Reception Buffer Address 3 */

/*****************  Bit definition for USB_ADDR4_RX register  *****************/
#define USB_ADDR4_RX_ADDR4_RX_Pos               (1U)
#define USB_ADDR4_RX_ADDR4_RX_Msk               (0x7FFFU << USB_ADDR4_RX_ADDR4_RX_Pos) /*!< 0x0000FFFE */
#define USB_ADDR4_RX_ADDR4_RX                   USB_ADDR4_RX_ADDR4_RX_Msk      /*!< Reception Buffer Address 4 */

/*****************  Bit definition for USB_ADDR5_RX register  *****************/
#define USB_ADDR5_RX_ADDR5_RX_Pos               (1U)
#define USB_ADDR5_RX_ADDR5_RX_Msk               (0x7FFFU << USB_ADDR5_RX_ADDR5_RX_Pos) /*!< 0x0000FFFE */
#define USB_ADDR5_RX_ADDR5_RX                   USB_ADDR5_RX_ADDR5_RX_Msk      /*!< Reception Buffer Address 5 */

/*****************  Bit definition for USB_ADDR6_RX register  *****************/
#define USB_ADDR6_RX_ADDR6_RX_Pos               (1U)
#define USB_ADDR6_RX_ADDR6_RX_Msk               (0x7FFFU << USB_ADDR6_RX_ADDR6_RX_Pos) /*!< 0x0000FFFE */
#define USB_ADDR6_RX_ADDR6_RX                   USB_ADDR6_RX_ADDR6_RX_Msk      /*!< Reception Buffer Address 6 */

/*****************  Bit definition for USB_ADDR7_RX register  *****************/
#define USB_ADDR7_RX_ADDR7_RX_Pos               (1U)
#define USB_ADDR7_RX_ADDR7_RX_Msk               (0x7FFFU << USB_ADDR7_RX_ADDR7_RX_Pos) /*!< 0x0000FFFE */
#define USB_ADDR7_RX_ADDR7_RX                   USB_ADDR7_RX_ADDR7_RX_Msk      /*!< Reception Buffer Address 7 */

/*----------------------------------------------------------------------------*/

/*****************  Bit definition for USB_COUNT0_RX register  ****************/
#define USB_COUNT0_RX_COUNT0_RX_Pos             (0U)
#define USB_COUNT0_RX_COUNT0_RX_Msk             (0x3FFU << USB_COUNT0_RX_COUNT0_RX_Pos) /*!< 0x000003FF */
#define USB_COUNT0_RX_COUNT0_RX                 USB_COUNT0_RX_COUNT0_RX_Msk    /*!< Reception Byte Count */

#define USB_COUNT0_RX_NUM_BLOCK_Pos             (10U)
#define USB_COUNT0_RX_NUM_BLOCK_Msk             (0x1FU << USB_COUNT0_RX_NUM_BLOCK_Pos) /*!< 0x00007C00 */
#define USB_COUNT0_RX_NUM_BLOCK                 USB_COUNT0_RX_NUM_BLOCK_Msk    /*!< NUM_BLOCK[4:0] bits (Number of blocks) */
#define USB_COUNT0_RX_NUM_BLOCK_0               (0x01U << USB_COUNT0_RX_NUM_BLOCK_Pos) /*!< 0x00000400 */
#define USB_COUNT0_RX_NUM_BLOCK_1               (0x02U << USB_COUNT0_RX_NUM_BLOCK_Pos) /*!< 0x00000800 */
#define USB_COUNT0_RX_NUM_BLOCK_2               (0x04U << USB_COUNT0_RX_NUM_BLOCK_Pos) /*!< 0x00001000 */
#define USB_COUNT0_RX_NUM_BLOCK_3               (0x08U << USB_COUNT0_RX_NUM_BLOCK_Pos) /*!< 0x00002000 */
#define USB_COUNT0_RX_NUM_BLOCK_4               (0x10U << USB_COUNT0_RX_NUM_BLOCK_Pos) /*!< 0x00004000 */

#define USB_COUNT0_RX_BLSIZE_Pos                (15U)
#define USB_COUNT0_RX_BLSIZE_Msk                (0x1U << USB_COUNT0_RX_BLSIZE_Pos) /*!< 0x00008000 */
#define USB_COUNT0_RX_BLSIZE                    USB_COUNT0_RX_BLSIZE_Msk       /*!< BLock SIZE */

/*****************  Bit definition for USB_COUNT1_RX register  ****************/
#define USB_COUNT1_RX_COUNT1_RX_Pos             (0U)
#define USB_COUNT1_RX_COUNT1_RX_Msk             (0x3FFU << USB_COUNT1_RX_COUNT1_RX_Pos) /*!< 0x000003FF */
#define USB_COUNT1_RX_COUNT1_RX                 USB_COUNT1_RX_COUNT1_RX_Msk    /*!< Reception Byte Count */

#define USB_COUNT1_RX_NUM_BLOCK_Pos             (10U)
#define USB_COUNT1_RX_NUM_BLOCK_Msk             (0x1FU << USB_COUNT1_RX_NUM_BLOCK_Pos) /*!< 0x00007C00 */
#define USB_COUNT1_RX_NUM_BLOCK                 USB_COUNT1_RX_NUM_BLOCK_Msk    /*!< NUM_BLOCK[4:0] bits (Number of blocks) */
#define USB_COUNT1_RX_NUM_BLOCK_0               (0x01U << USB_COUNT1_RX_NUM_BLOCK_Pos) /*!< 0x00000400 */
#define USB_COUNT1_RX_NUM_BLOCK_1               (0x02U << USB_COUNT1_RX_NUM_BLOCK_Pos) /*!< 0x00000800 */
#define USB_COUNT1_RX_NUM_BLOCK_2               (0x04U << USB_COUNT1_RX_NUM_BLOCK_Pos) /*!< 0x00001000 */
#define USB_COUNT1_RX_NUM_BLOCK_3               (0x08U << USB_COUNT1_RX_NUM_BLOCK_Pos) /*!< 0x00002000 */
#define USB_COUNT1_RX_NUM_BLOCK_4               (0x10U << USB_COUNT1_RX_NUM_BLOCK_Pos) /*!< 0x00004000 */

#define USB_COUNT1_RX_BLSIZE_Pos                (15U)
#define USB_COUNT1_RX_BLSIZE_Msk                (0x1U << USB_COUNT1_RX_BLSIZE_Pos) /*!< 0x00008000 */
#define USB_COUNT1_RX_BLSIZE                    USB_COUNT1_RX_BLSIZE_Msk       /*!< BLock SIZE */

/*****************  Bit definition for USB_COUNT2_RX register  ****************/
#define USB_COUNT2_RX_COUNT2_RX_Pos             (0U)
#define USB_COUNT2_RX_COUNT2_RX_Msk             (0x3FFU << USB_COUNT2_RX_COUNT2_RX_Pos) /*!< 0x000003FF */
#define USB_COUNT2_RX_COUNT2_RX                 USB_COUNT2_RX_COUNT2_RX_Msk    /*!< Reception Byte Count */

#define USB_COUNT2_RX_NUM_BLOCK_Pos             (10U)
#define USB_COUNT2_RX_NUM_BLOCK_Msk             (0x1FU << USB_COUNT2_RX_NUM_BLOCK_Pos) /*!< 0x00007C00 */
#define USB_COUNT2_RX_NUM_BLOCK                 USB_COUNT2_RX_NUM_BLOCK_Msk    /*!< NUM_BLOCK[4:0] bits (Number of blocks) */
#define USB_COUNT2_RX_NUM_BLOCK_0               (0x01U << USB_COUNT2_RX_NUM_BLOCK_Pos) /*!< 0x00000400 */
#define USB_COUNT2_RX_NUM_BLOCK_1               (0x02U << USB_COUNT2_RX_NUM_BLOCK_Pos) /*!< 0x00000800 */
#define USB_COUNT2_RX_NUM_BLOCK_2               (0x04U << USB_COUNT2_RX_NUM_BLOCK_Pos) /*!< 0x00001000 */
#define USB_COUNT2_RX_NUM_BLOCK_3               (0x08U << USB_COUNT2_RX_NUM_BLOCK_Pos) /*!< 0x00002000 */
#define USB_COUNT2_RX_NUM_BLOCK_4               (0x10U << USB_COUNT2_RX_NUM_BLOCK_Pos) /*!< 0x00004000 */

#define USB_COUNT2_RX_BLSIZE_Pos                (15U)
#define USB_COUNT2_RX_BLSIZE_Msk                (0x1U << USB_COUNT2_RX_BLSIZE_Pos) /*!< 0x00008000 */
#define USB_COUNT2_RX_BLSIZE                    USB_COUNT2_RX_BLSIZE_Msk       /*!< BLock SIZE */

/*****************  Bit definition for USB_COUNT3_RX register  ****************/
#define USB_COUNT3_RX_COUNT3_RX_Pos             (0U)
#define USB_COUNT3_RX_COUNT3_RX_Msk             (0x3FFU << USB_COUNT3_RX_COUNT3_RX_Pos) /*!< 0x000003FF */
#define USB_COUNT3_RX_COUNT3_RX                 USB_COUNT3_RX_COUNT3_RX_Msk    /*!< Reception Byte Count */

#define USB_COUNT3_RX_NUM_BLOCK_Pos             (10U)
#define USB_COUNT3_RX_NUM_BLOCK_Msk             (0x1FU << USB_COUNT3_RX_NUM_BLOCK_Pos) /*!< 0x00007C00 */
#define USB_COUNT3_RX_NUM_BLOCK                 USB_COUNT3_RX_NUM_BLOCK_Msk    /*!< NUM_BLOCK[4:0] bits (Number of blocks) */
#define USB_COUNT3_RX_NUM_BLOCK_0               (0x01U << USB_COUNT3_RX_NUM_BLOCK_Pos) /*!< 0x00000400 */
#define USB_COUNT3_RX_NUM_BLOCK_1               (0x02U << USB_COUNT3_RX_NUM_BLOCK_Pos) /*!< 0x00000800 */
#define USB_COUNT3_RX_NUM_BLOCK_2               (0x04U << USB_COUNT3_RX_NUM_BLOCK_Pos) /*!< 0x00001000 */
#define USB_COUNT3_RX_NUM_BLOCK_3               (0x08U << USB_COUNT3_RX_NUM_BLOCK_Pos) /*!< 0x00002000 */
#define USB_COUNT3_RX_NUM_BLOCK_4               (0x10U << USB_COUNT3_RX_NUM_BLOCK_Pos) /*!< 0x00004000 */

#define USB_COUNT3_RX_BLSIZE_Pos                (15U)
#define USB_COUNT3_RX_BLSIZE_Msk                (0x1U << USB_COUNT3_RX_BLSIZE_Pos) /*!< 0x00008000 */
#define USB_COUNT3_RX_BLSIZE                    USB_COUNT3_RX_BLSIZE_Msk       /*!< BLock SIZE */

/*****************  Bit definition for USB_COUNT4_RX register  ****************/
#define USB_COUNT4_RX_COUNT4_RX_Pos             (0U)
#define USB_COUNT4_RX_COUNT4_RX_Msk             (0x3FFU << USB_COUNT4_RX_COUNT4_RX_Pos) /*!< 0x000003FF */
#define USB_COUNT4_RX_COUNT4_RX                 USB_COUNT4_RX_COUNT4_RX_Msk    /*!< Reception Byte Count */

#define USB_COUNT4_RX_NUM_BLOCK_Pos             (10U)
#define USB_COUNT4_RX_NUM_BLOCK_Msk             (0x1FU << USB_COUNT4_RX_NUM_BLOCK_Pos) /*!< 0x00007C00 */
#define USB_COUNT4_RX_NUM_BLOCK                 USB_COUNT4_RX_NUM_BLOCK_Msk    /*!< NUM_BLOCK[4:0] bits (Number of blocks) */
#define USB_COUNT4_RX_NUM_BLOCK_0               (0x01U << USB_COUNT4_RX_NUM_BLOCK_Pos) /*!< 0x00000400 */
#define USB_COUNT4_RX_NUM_BLOCK_1               (0x02U << USB_COUNT4_RX_NUM_BLOCK_Pos) /*!< 0x00000800 */
#define USB_COUNT4_RX_NUM_BLOCK_2               (0x04U << USB_COUNT4_RX_NUM_BLOCK_Pos) /*!< 0x00001000 */
#define USB_COUNT4_RX_NUM_BLOCK_3               (0x08U << USB_COUNT4_RX_NUM_BLOCK_Pos) /*!< 0x00002000 */
#define USB_COUNT4_RX_NUM_BLOCK_4               (0x10U << USB_COUNT4_RX_NUM_BLOCK_Pos) /*!< 0x00004000 */

#define USB_COUNT4_RX_BLSIZE_Pos                (15U)
#define USB_COUNT4_RX_BLSIZE_Msk                (0x1U << USB_COUNT4_RX_BLSIZE_Pos) /*!< 0x00008000 */
#define USB_COUNT4_RX_BLSIZE                    USB_COUNT4_RX_BLSIZE_Msk       /*!< BLock SIZE */

/*****************  Bit definition for USB_COUNT5_RX register  ****************/
#define USB_COUNT5_RX_COUNT5_RX_Pos             (0U)
#define USB_COUNT5_RX_COUNT5_RX_Msk             (0x3FFU << USB_COUNT5_RX_COUNT5_RX_Pos) /*!< 0x000003FF */
#define USB_COUNT5_RX_COUNT5_RX                 USB_COUNT5_RX_COUNT5_RX_Msk    /*!< Reception Byte Count */

#define USB_COUNT5_RX_NUM_BLOCK_Pos             (10U)
#define USB_COUNT5_RX_NUM_BLOCK_Msk             (0x1FU << USB_COUNT5_RX_NUM_BLOCK_Pos) /*!< 0x00007C00 */
#define USB_COUNT5_RX_NUM_BLOCK                 USB_COUNT5_RX_NUM_BLOCK_Msk    /*!< NUM_BLOCK[4:0] bits (Number of blocks) */
#define USB_COUNT5_RX_NUM_BLOCK_0               (0x01U << USB_COUNT5_RX_NUM_BLOCK_Pos) /*!< 0x00000400 */
#define USB_COUNT5_RX_NUM_BLOCK_1               (0x02U << USB_COUNT5_RX_NUM_BLOCK_Pos) /*!< 0x00000800 */
#define USB_COUNT5_RX_NUM_BLOCK_2               (0x04U << USB_COUNT5_RX_NUM_BLOCK_Pos) /*!< 0x00001000 */
#define USB_COUNT5_RX_NUM_BLOCK_3               (0x08U << USB_COUNT5_RX_NUM_BLOCK_Pos) /*!< 0x00002000 */
#define USB_COUNT5_RX_NUM_BLOCK_4               (0x10U << USB_COUNT5_RX_NUM_BLOCK_Pos) /*!< 0x00004000 */

#define USB_COUNT5_RX_BLSIZE_Pos                (15U)
#define USB_COUNT5_RX_BLSIZE_Msk                (0x1U << USB_COUNT5_RX_BLSIZE_Pos) /*!< 0x00008000 */
#define USB_COUNT5_RX_BLSIZE                    USB_COUNT5_RX_BLSIZE_Msk       /*!< BLock SIZE */

/*****************  Bit definition for USB_COUNT6_RX register  ****************/
#define USB_COUNT6_RX_COUNT6_RX_Pos             (0U)
#define USB_COUNT6_RX_COUNT6_RX_Msk             (0x3FFU << USB_COUNT6_RX_COUNT6_RX_Pos) /*!< 0x000003FF */
#define USB_COUNT6_RX_COUNT6_RX                 USB_COUNT6_RX_COUNT6_RX_Msk    /*!< Reception Byte Count */

#define USB_COUNT6_RX_NUM_BLOCK_Pos             (10U)
#define USB_COUNT6_RX_NUM_BLOCK_Msk             (0x1FU << USB_COUNT6_RX_NUM_BLOCK_Pos) /*!< 0x00007C00 */
#define USB_COUNT6_RX_NUM_BLOCK                 USB_COUNT6_RX_NUM_BLOCK_Msk    /*!< NUM_BLOCK[4:0] bits (Number of blocks) */
#define USB_COUNT6_RX_NUM_BLOCK_0               (0x01U << USB_COUNT6_RX_NUM_BLOCK_Pos) /*!< 0x00000400 */
#define USB_COUNT6_RX_NUM_BLOCK_1               (0x02U << USB_COUNT6_RX_NUM_BLOCK_Pos) /*!< 0x00000800 */
#define USB_COUNT6_RX_NUM_BLOCK_2               (0x04U << USB_COUNT6_RX_NUM_BLOCK_Pos) /*!< 0x00001000 */
#define USB_COUNT6_RX_NUM_BLOCK_3               (0x08U << USB_COUNT6_RX_NUM_BLOCK_Pos) /*!< 0x00002000 */
#define USB_COUNT6_RX_NUM_BLOCK_4               (0x10U << USB_COUNT6_RX_NUM_BLOCK_Pos) /*!< 0x00004000 */

#define USB_COUNT6_RX_BLSIZE_Pos                (15U)
#define USB_COUNT6_RX_BLSIZE_Msk                (0x1U << USB_COUNT6_RX_BLSIZE_Pos) /*!< 0x00008000 */
#define USB_COUNT6_RX_BLSIZE                    USB_COUNT6_RX_BLSIZE_Msk       /*!< BLock SIZE */

/*****************  Bit definition for USB_COUNT7_RX register  ****************/
#define USB_COUNT7_RX_COUNT7_RX_Pos             (0U)
#define USB_COUNT7_RX_COUNT7_RX_Msk             (0x3FFU << USB_COUNT7_RX_COUNT7_RX_Pos) /*!< 0x000003FF */
#define USB_COUNT7_RX_COUNT7_RX                 USB_COUNT7_RX_COUNT7_RX_Msk    /*!< Reception Byte Count */

#define USB_COUNT7_RX_NUM_BLOCK_Pos             (10U)
#define USB_COUNT7_RX_NUM_BLOCK_Msk             (0x1FU << USB_COUNT7_RX_NUM_BLOCK_Pos) /*!< 0x00007C00 */
#define USB_COUNT7_RX_NUM_BLOCK                 USB_COUNT7_RX_NUM_BLOCK_Msk    /*!< NUM_BLOCK[4:0] bits (Number of blocks) */
#define USB_COUNT7_RX_NUM_BLOCK_0               (0x01U << USB_COUNT7_RX_NUM_BLOCK_Pos) /*!< 0x00000400 */
#define USB_COUNT7_RX_NUM_BLOCK_1               (0x02U << USB_COUNT7_RX_NUM_BLOCK_Pos) /*!< 0x00000800 */
#define USB_COUNT7_RX_NUM_BLOCK_2               (0x04U << USB_COUNT7_RX_NUM_BLOCK_Pos) /*!< 0x00001000 */
#define USB_COUNT7_RX_NUM_BLOCK_3               (0x08U << USB_COUNT7_RX_NUM_BLOCK_Pos) /*!< 0x00002000 */
#define USB_COUNT7_RX_NUM_BLOCK_4               (0x10U << USB_COUNT7_RX_NUM_BLOCK_Pos) /*!< 0x00004000 */

#define USB_COUNT7_RX_BLSIZE_Pos                (15U)
#define USB_COUNT7_RX_BLSIZE_Msk                (0x1U << USB_COUNT7_RX_BLSIZE_Pos) /*!< 0x00008000 */
#define USB_COUNT7_RX_BLSIZE                    USB_COUNT7_RX_BLSIZE_Msk       /*!< BLock SIZE */

/*----------------------------------------------------------------------------*/

/****************  Bit definition for USB_COUNT0_RX_0 register  ***************/
#define USB_COUNT0_RX_0_COUNT0_RX_0             0x000003FFU                    /*!< Reception Byte Count (low) */

#define USB_COUNT0_RX_0_NUM_BLOCK_0             0x00007C00U                    /*!< NUM_BLOCK_0[4:0] bits (Number of blocks) (low) */
#define USB_COUNT0_RX_0_NUM_BLOCK_0_0           0x00000400U                    /*!< Bit 0 */
#define USB_COUNT0_RX_0_NUM_BLOCK_0_1           0x00000800U                    /*!< Bit 1 */
#define USB_COUNT0_RX_0_NUM_BLOCK_0_2           0x00001000U                    /*!< Bit 2 */
#define USB_COUNT0_RX_0_NUM_BLOCK_0_3           0x00002000U                    /*!< Bit 3 */
#define USB_COUNT0_RX_0_NUM_BLOCK_0_4           0x00004000U                    /*!< Bit 4 */

#define USB_COUNT0_RX_0_BLSIZE_0                0x00008000U                    /*!< BLock SIZE (low) */

/****************  Bit definition for USB_COUNT0_RX_1 register  ***************/
#define USB_COUNT0_RX_1_COUNT0_RX_1             0x03FF0000U                    /*!< Reception Byte Count (high) */

#define USB_COUNT0_RX_1_NUM_BLOCK_1             0x7C000000U                    /*!< NUM_BLOCK_1[4:0] bits (Number of blocks) (high) */
#define USB_COUNT0_RX_1_NUM_BLOCK_1_0           0x04000000U                    /*!< Bit 1 */
#define USB_COUNT0_RX_1_NUM_BLOCK_1_1           0x08000000U                    /*!< Bit 1 */
#define USB_COUNT0_RX_1_NUM_BLOCK_1_2           0x10000000U                    /*!< Bit 2 */
#define USB_COUNT0_RX_1_NUM_BLOCK_1_3           0x20000000U                    /*!< Bit 3 */
#define USB_COUNT0_RX_1_NUM_BLOCK_1_4           0x40000000U                    /*!< Bit 4 */

#define USB_COUNT0_RX_1_BLSIZE_1                0x80000000U                    /*!< BLock SIZE (high) */

/****************  Bit definition for USB_COUNT1_RX_0 register  ***************/
#define USB_COUNT1_RX_0_COUNT1_RX_0             0x000003FFU                    /*!< Reception Byte Count (low) */

#define USB_COUNT1_RX_0_NUM_BLOCK_0             0x00007C00U                    /*!< NUM_BLOCK_0[4:0] bits (Number of blocks) (low) */
#define USB_COUNT1_RX_0_NUM_BLOCK_0_0           0x00000400U                    /*!< Bit 0 */
#define USB_COUNT1_RX_0_NUM_BLOCK_0_1           0x00000800U                    /*!< Bit 1 */
#define USB_COUNT1_RX_0_NUM_BLOCK_0_2           0x00001000U                    /*!< Bit 2 */
#define USB_COUNT1_RX_0_NUM_BLOCK_0_3           0x00002000U                    /*!< Bit 3 */
#define USB_COUNT1_RX_0_NUM_BLOCK_0_4           0x00004000U                    /*!< Bit 4 */

#define USB_COUNT1_RX_0_BLSIZE_0                0x00008000U                    /*!< BLock SIZE (low) */

/****************  Bit definition for USB_COUNT1_RX_1 register  ***************/
#define USB_COUNT1_RX_1_COUNT1_RX_1             0x03FF0000U                    /*!< Reception Byte Count (high) */

#define USB_COUNT1_RX_1_NUM_BLOCK_1             0x7C000000U                    /*!< NUM_BLOCK_1[4:0] bits (Number of blocks) (high) */
#define USB_COUNT1_RX_1_NUM_BLOCK_1_0           0x04000000U                    /*!< Bit 0 */
#define USB_COUNT1_RX_1_NUM_BLOCK_1_1           0x08000000U                    /*!< Bit 1 */
#define USB_COUNT1_RX_1_NUM_BLOCK_1_2           0x10000000U                    /*!< Bit 2 */
#define USB_COUNT1_RX_1_NUM_BLOCK_1_3           0x20000000U                    /*!< Bit 3 */
#define USB_COUNT1_RX_1_NUM_BLOCK_1_4           0x40000000U                    /*!< Bit 4 */

#define USB_COUNT1_RX_1_BLSIZE_1                0x80000000U                    /*!< BLock SIZE (high) */

/****************  Bit definition for USB_COUNT2_RX_0 register  ***************/
#define USB_COUNT2_RX_0_COUNT2_RX_0             0x000003FFU                    /*!< Reception Byte Count (low) */

#define USB_COUNT2_RX_0_NUM_BLOCK_0             0x00007C00U                    /*!< NUM_BLOCK_0[4:0] bits (Number of blocks) (low) */
#define USB_COUNT2_RX_0_NUM_BLOCK_0_0           0x00000400U                    /*!< Bit 0 */
#define USB_COUNT2_RX_0_NUM_BLOCK_0_1           0x00000800U                    /*!< Bit 1 */
#define USB_COUNT2_RX_0_NUM_BLOCK_0_2           0x00001000U                    /*!< Bit 2 */
#define USB_COUNT2_RX_0_NUM_BLOCK_0_3           0x00002000U                    /*!< Bit 3 */
#define USB_COUNT2_RX_0_NUM_BLOCK_0_4           0x00004000U                    /*!< Bit 4 */

#define USB_COUNT2_RX_0_BLSIZE_0                0x00008000U                    /*!< BLock SIZE (low) */

/****************  Bit definition for USB_COUNT2_RX_1 register  ***************/
#define USB_COUNT2_RX_1_COUNT2_RX_1             0x03FF0000U                    /*!< Reception Byte Count (high) */

#define USB_COUNT2_RX_1_NUM_BLOCK_1             0x7C000000U                    /*!< NUM_BLOCK_1[4:0] bits (Number of blocks) (high) */
#define USB_COUNT2_RX_1_NUM_BLOCK_1_0           0x04000000U                    /*!< Bit 0 */
#define USB_COUNT2_RX_1_NUM_BLOCK_1_1           0x08000000U                    /*!< Bit 1 */
#define USB_COUNT2_RX_1_NUM_BLOCK_1_2           0x10000000U                    /*!< Bit 2 */
#define USB_COUNT2_RX_1_NUM_BLOCK_1_3           0x20000000U                    /*!< Bit 3 */
#define USB_COUNT2_RX_1_NUM_BLOCK_1_4           0x40000000U                    /*!< Bit 4 */

#define USB_COUNT2_RX_1_BLSIZE_1                0x80000000U                    /*!< BLock SIZE (high) */

/****************  Bit definition for USB_COUNT3_RX_0 register  ***************/
#define USB_COUNT3_RX_0_COUNT3_RX_0             0x000003FFU                    /*!< Reception Byte Count (low) */

#define USB_COUNT3_RX_0_NUM_BLOCK_0             0x00007C00U                    /*!< NUM_BLOCK_0[4:0] bits (Number of blocks) (low) */
#define USB_COUNT3_RX_0_NUM_BLOCK_0_0           0x00000400U                    /*!< Bit 0 */
#define USB_COUNT3_RX_0_NUM_BLOCK_0_1           0x00000800U                    /*!< Bit 1 */
#define USB_COUNT3_RX_0_NUM_BLOCK_0_2           0x00001000U                    /*!< Bit 2 */
#define USB_COUNT3_RX_0_NUM_BLOCK_0_3           0x00002000U                    /*!< Bit 3 */
#define USB_COUNT3_RX_0_NUM_BLOCK_0_4           0x00004000U                    /*!< Bit 4 */

#define USB_COUNT3_RX_0_BLSIZE_0                0x00008000U                    /*!< BLock SIZE (low) */

/****************  Bit definition for USB_COUNT3_RX_1 register  ***************/
#define USB_COUNT3_RX_1_COUNT3_RX_1             0x03FF0000U                    /*!< Reception Byte Count (high) */

#define USB_COUNT3_RX_1_NUM_BLOCK_1             0x7C000000U                    /*!< NUM_BLOCK_1[4:0] bits (Number of blocks) (high) */
#define USB_COUNT3_RX_1_NUM_BLOCK_1_0           0x04000000U                    /*!< Bit 0 */
#define USB_COUNT3_RX_1_NUM_BLOCK_1_1           0x08000000U                    /*!< Bit 1 */
#define USB_COUNT3_RX_1_NUM_BLOCK_1_2           0x10000000U                    /*!< Bit 2 */
#define USB_COUNT3_RX_1_NUM_BLOCK_1_3           0x20000000U                    /*!< Bit 3 */
#define USB_COUNT3_RX_1_NUM_BLOCK_1_4           0x40000000U                    /*!< Bit 4 */

#define USB_COUNT3_RX_1_BLSIZE_1                0x80000000U                    /*!< BLock SIZE (high) */

/****************  Bit definition for USB_COUNT4_RX_0 register  ***************/
#define USB_COUNT4_RX_0_COUNT4_RX_0             0x000003FFU                    /*!< Reception Byte Count (low) */

#define USB_COUNT4_RX_0_NUM_BLOCK_0             0x00007C00U                    /*!< NUM_BLOCK_0[4:0] bits (Number of blocks) (low) */
#define USB_COUNT4_RX_0_NUM_BLOCK_0_0           0x00000400U                    /*!< Bit 0 */
#define USB_COUNT4_RX_0_NUM_BLOCK_0_1           0x00000800U                    /*!< Bit 1 */
#define USB_COUNT4_RX_0_NUM_BLOCK_0_2           0x00001000U                    /*!< Bit 2 */
#define USB_COUNT4_RX_0_NUM_BLOCK_0_3           0x00002000U                    /*!< Bit 3 */
#define USB_COUNT4_RX_0_NUM_BLOCK_0_4           0x00004000U                    /*!< Bit 4 */

#define USB_COUNT4_RX_0_BLSIZE_0                0x00008000U                    /*!< BLock SIZE (low) */

/****************  Bit definition for USB_COUNT4_RX_1 register  ***************/
#define USB_COUNT4_RX_1_COUNT4_RX_1             0x03FF0000U                    /*!< Reception Byte Count (high) */

#define USB_COUNT4_RX_1_NUM_BLOCK_1             0x7C000000U                    /*!< NUM_BLOCK_1[4:0] bits (Number of blocks) (high) */
#define USB_COUNT4_RX_1_NUM_BLOCK_1_0           0x04000000U                    /*!< Bit 0 */
#define USB_COUNT4_RX_1_NUM_BLOCK_1_1           0x08000000U                    /*!< Bit 1 */
#define USB_COUNT4_RX_1_NUM_BLOCK_1_2           0x10000000U                    /*!< Bit 2 */
#define USB_COUNT4_RX_1_NUM_BLOCK_1_3           0x20000000U                    /*!< Bit 3 */
#define USB_COUNT4_RX_1_NUM_BLOCK_1_4           0x40000000U                    /*!< Bit 4 */

#define USB_COUNT4_RX_1_BLSIZE_1                0x80000000U                    /*!< BLock SIZE (high) */

/****************  Bit definition for USB_COUNT5_RX_0 register  ***************/
#define USB_COUNT5_RX_0_COUNT5_RX_0             0x000003FFU                    /*!< Reception Byte Count (low) */

#define USB_COUNT5_RX_0_NUM_BLOCK_0             0x00007C00U                    /*!< NUM_BLOCK_0[4:0] bits (Number of blocks) (low) */
#define USB_COUNT5_RX_0_NUM_BLOCK_0_0           0x00000400U                    /*!< Bit 0 */
#define USB_COUNT5_RX_0_NUM_BLOCK_0_1           0x00000800U                    /*!< Bit 1 */
#define USB_COUNT5_RX_0_NUM_BLOCK_0_2           0x00001000U                    /*!< Bit 2 */
#define USB_COUNT5_RX_0_NUM_BLOCK_0_3           0x00002000U                    /*!< Bit 3 */
#define USB_COUNT5_RX_0_NUM_BLOCK_0_4           0x00004000U                    /*!< Bit 4 */

#define USB_COUNT5_RX_0_BLSIZE_0                0x00008000U                    /*!< BLock SIZE (low) */

/****************  Bit definition for USB_COUNT5_RX_1 register  ***************/
#define USB_COUNT5_RX_1_COUNT5_RX_1             0x03FF0000U                    /*!< Reception Byte Count (high) */

#define USB_COUNT5_RX_1_NUM_BLOCK_1             0x7C000000U                    /*!< NUM_BLOCK_1[4:0] bits (Number of blocks) (high) */
#define USB_COUNT5_RX_1_NUM_BLOCK_1_0           0x04000000U                    /*!< Bit 0 */
#define USB_COUNT5_RX_1_NUM_BLOCK_1_1           0x08000000U                    /*!< Bit 1 */
#define USB_COUNT5_RX_1_NUM_BLOCK_1_2           0x10000000U                    /*!< Bit 2 */
#define USB_COUNT5_RX_1_NUM_BLOCK_1_3           0x20000000U                    /*!< Bit 3 */
#define USB_COUNT5_RX_1_NUM_BLOCK_1_4           0x40000000U                    /*!< Bit 4 */

#define USB_COUNT5_RX_1_BLSIZE_1                0x80000000U                    /*!< BLock SIZE (high) */

/***************  Bit definition for USB_COUNT6_RX_0  register  ***************/
#define USB_COUNT6_RX_0_COUNT6_RX_0             0x000003FFU                    /*!< Reception Byte Count (low) */

#define USB_COUNT6_RX_0_NUM_BLOCK_0             0x00007C00U                    /*!< NUM_BLOCK_0[4:0] bits (Number of blocks) (low) */
#define USB_COUNT6_RX_0_NUM_BLOCK_0_0           0x00000400U                    /*!< Bit 0 */
#define USB_COUNT6_RX_0_NUM_BLOCK_0_1           0x00000800U                    /*!< Bit 1 */
#define USB_COUNT6_RX_0_NUM_BLOCK_0_2           0x00001000U                    /*!< Bit 2 */
#define USB_COUNT6_RX_0_NUM_BLOCK_0_3           0x00002000U                    /*!< Bit 3 */
#define USB_COUNT6_RX_0_NUM_BLOCK_0_4           0x00004000U                    /*!< Bit 4 */

#define USB_COUNT6_RX_0_BLSIZE_0                0x00008000U                    /*!< BLock SIZE (low) */

/****************  Bit definition for USB_COUNT6_RX_1 register  ***************/
#define USB_COUNT6_RX_1_COUNT6_RX_1             0x03FF0000U                   /*!< Reception Byte Count (high) */

#define USB_COUNT6_RX_1_NUM_BLOCK_1             0x7C000000U                   /*!< NUM_BLOCK_1[4:0] bits (Number of blocks) (high) */
#define USB_COUNT6_RX_1_NUM_BLOCK_1_0           0x04000000U                   /*!< Bit 0 */
#define USB_COUNT6_RX_1_NUM_BLOCK_1_1           0x08000000U                   /*!< Bit 1 */
#define USB_COUNT6_RX_1_NUM_BLOCK_1_2           0x10000000U                   /*!< Bit 2 */
#define USB_COUNT6_RX_1_NUM_BLOCK_1_3           0x20000000U                   /*!< Bit 3 */
#define USB_COUNT6_RX_1_NUM_BLOCK_1_4           0x40000000U                   /*!< Bit 4 */

#define USB_COUNT6_RX_1_BLSIZE_1                0x80000000U                   /*!< BLock SIZE (high) */

/***************  Bit definition for USB_COUNT7_RX_0 register  ****************/
#define USB_COUNT7_RX_0_COUNT7_RX_0             0x000003FFU                    /*!< Reception Byte Count (low) */

#define USB_COUNT7_RX_0_NUM_BLOCK_0             0x00007C00U                    /*!< NUM_BLOCK_0[4:0] bits (Number of blocks) (low) */
#define USB_COUNT7_RX_0_NUM_BLOCK_0_0           0x00000400U                    /*!< Bit 0 */
#define USB_COUNT7_RX_0_NUM_BLOCK_0_1           0x00000800U                    /*!< Bit 1 */
#define USB_COUNT7_RX_0_NUM_BLOCK_0_2           0x00001000U                    /*!< Bit 2 */
#define USB_COUNT7_RX_0_NUM_BLOCK_0_3           0x00002000U                    /*!< Bit 3 */
#define USB_COUNT7_RX_0_NUM_BLOCK_0_4           0x00004000U                    /*!< Bit 4 */

#define USB_COUNT7_RX_0_BLSIZE_0                0x00008000U                    /*!< BLock SIZE (low) */

/***************  Bit definition for USB_COUNT7_RX_1 register  ****************/
#define USB_COUNT7_RX_1_COUNT7_RX_1             0x03FF0000U                    /*!< Reception Byte Count (high) */

#define USB_COUNT7_RX_1_NUM_BLOCK_1             0x7C000000U                    /*!< NUM_BLOCK_1[4:0] bits (Number of blocks) (high) */
#define USB_COUNT7_RX_1_NUM_BLOCK_1_0           0x04000000U                    /*!< Bit 0 */
#define USB_COUNT7_RX_1_NUM_BLOCK_1_1           0x08000000U                    /*!< Bit 1 */
#define USB_COUNT7_RX_1_NUM_BLOCK_1_2           0x10000000U                    /*!< Bit 2 */
#define USB_COUNT7_RX_1_NUM_BLOCK_1_3           0x20000000U                    /*!< Bit 3 */
#define USB_COUNT7_RX_1_NUM_BLOCK_1_4           0x40000000U                    /*!< Bit 4 */

#define USB_COUNT7_RX_1_BLSIZE_1                0x80000000U                    /*!< BLock SIZE (high) */

















//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////






#define CAN_MCR_INRQ                         (0x1U << 0U)                      /* Initialization Request*/
#define CAN_MCR_SLEEP                        (0x1U << 1U)                      /* Sleep Mode Request*/
#define CAN_MCR_TXFP                         (0x1U << 2U)                      /* Transmit FIFO Priority*/
#define CAN_MCR_RFLM                         (0x1U << 3U)                      /* Receive FIFO Locked Mode*/
#define CAN_MCR_NART                         (0x1U << 4U)                      /* No Automatic Retransmission*/
#define CAN_MCR_AWUM                         (0x1U << 5U)                      /* Automatic Wakeup Mode*/
#define CAN_MCR_ABOM                         (0x1U << 6U)                      /* Automatic Bus-Off Management*/
#define CAN_MCR_TTCM                         (0x1U << 7U)                      /* Time Triggered Communication Mode*/
#define CAN_MCR_RESET                        (0x1U << 15U)                     /* CAN software master reset*/
#define CAN_MCR_DBF                          (0x1U << 16U)                     /* CAN Debug freeze*/

#define CAN_MSR_INAK                         (0x1U << 0U)                      /* Initialization Acknowledge*/
#define CAN_MSR_SLAK                         (0x1U << 1U)                      /* Sleep Acknowledge*/
#define CAN_MSR_ERRI                         (0x1U << 2U)                      /* Error Interrupt*/
#define CAN_MSR_WKUI                         (0x1U << 3U)                      /* Wakeup Interrupt*/
#define CAN_MSR_SLAKI                        (0x1U << 4U)                      /* Sleep Acknowledge Interrupt*/
#define CAN_MSR_TXM                          (0x1U << 8U)                      /* Transmit Mode*/
#define CAN_MSR_RXM                          (0x1U << 9U)                      /* Receive Mode*/
#define CAN_MSR_SAMP                         (0x1U << 10U)                     /* Last Sample Point*/
#define CAN_MSR_RX                           (0x1U << 11U)                     /* CAN Rx Signal*/

#define CAN_TSR_RQCP0                        (0x1U << 0U)                      /* Request Completed Mailbox0*/
#define CAN_TSR_TXOK0                        (0x1U << 1U)                      /* Transmission OK of Mailbox0*/
#define CAN_TSR_ALST0                        (0x1U << 2U)                      /* Arbitration Lost for Mailbox0*/
#define CAN_TSR_TERR0                        (0x1U << 3U)                      /* Transmission Error of Mailbox0*/
#define CAN_TSR_ABRQ0                        (0x1U << 7U)                      /* Abort Request for Mailbox0*/
#define CAN_TSR_RQCP1                        (0x1U << 8U)                      /* Request Completed Mailbox1*/
#define CAN_TSR_TXOK1                        (0x1U << 9U)                      /* Transmission OK of Mailbox1*/
#define CAN_TSR_ALST1                        (0x1U << 10U)                     /* Arbitration Lost for Mailbox1*/
#define CAN_TSR_TERR1                        (0x1U << 11U)                     /* Transmission Error of Mailbox1*/
#define CAN_TSR_ABRQ1                        (0x1U << 15U)                     /* Abort Request for Mailbox 1*/
#define CAN_TSR_RQCP2                        (0x1U << 16U)                     /* Request Completed Mailbox2*/
#define CAN_TSR_TXOK2                        (0x1U << 17U)                     /* Transmission OK of Mailbox 2*/
#define CAN_TSR_ALST2                        (0x1U << 18U)                     /* Arbitration Lost for mailbox 2*/
#define CAN_TSR_TERR2                        (0x1U << 19U)                     /* Transmission Error of Mailbox 2*/
#define CAN_TSR_ABRQ2                        (0x1U << 23U)                     /* Abort Request for Mailbox 2*/
#define CAN_TSR_CODE                         (0x3U << 24U)                     /* Mailbox Code*/
#define CAN_TSR_TME                          (0x7U << 26U)                     /* TME[2:0] bits*/
#define CAN_TSR_TME0                         (0x1U << 26U)                     /* Transmit Mailbox 0 Empty*/
#define CAN_TSR_TME1                         (0x1U << 27U)                     /* Transmit Mailbox 1 Empty*/
#define CAN_TSR_TME2                         (0x1U << 28U)                     /* Transmit Mailbox 2 Empty*/
#define CAN_TSR_LOW                          (0x7U << 29U)                     /* LOW[2:0] bits*/
#define CAN_TSR_LOW0                         (0x1U << 29U)                     /* Lowest Priority Flag for Mailbox 0*/
#define CAN_TSR_LOW1                         (0x1U << 30U)                     /* Lowest Priority Flag for Mailbox 1*/
#define CAN_TSR_LOW2                         (0x1U << 31U)                     /* Lowest Priority Flag for Mailbox 2*/

#define CAN_RF0R_FMP0                        (0x3U << 0U)                      /* FIFO 0 Message Pending*/
#define CAN_RF0R_FULL0                       (0x1U << 3U)                      /* FIFO 0 Full*/
#define CAN_RF0R_FOVR0                       (0x1U << 4U)                      /* FIFO 0 Overrun*/
#define CAN_RF0R_RFOM0                       (0x1U << 5U)                      /* Release FIFO 0 Output Mailbox*/

#define CAN_RF1R_FMP1                        (0x3U << 0U)                      /* FIFO 1 Message Pending*/
#define CAN_RF1R_FULL1                       (0x1U << 3U)                      /* FIFO 1 Full*/
#define CAN_RF1R_FOVR1                       (0x1U << 4U)                      /* FIFO 1 Overrun*/
#define CAN_RF1R_RFOM1                       (0x1U << 5U)                      /* Release FIFO 1 Output Mailbox*/

#define CAN_IER_TMEIE                        (0x1U << 0U)                      /* Transmit Mailbox Empty Interrupt Enable*/
#define CAN_IER_FMPIE0                       (0x1U << 1U)                      /* FIFO Message Pending Interrupt Enable*/
#define CAN_IER_FFIE0                        (0x1U << 2U)                      /* FIFO Full Interrupt Enable*/
#define CAN_IER_FOVIE0                       (0x1U << 3U)                      /* FIFO Overrun Interrupt Enable*/
#define CAN_IER_FMPIE1                       (0x1U << 4U)                      /* FIFO Message Pending Interrupt Enable*/
#define CAN_IER_FFIE1                        (0x1U << 5U)                      /* FIFO Full Interrupt Enable*/
#define CAN_IER_FOVIE1                       (0x1U << 6U)                      /* FIFO Overrun Interrupt Enable*/
#define CAN_IER_EWGIE                        (0x1U << 8U)                      /* Error Warning Interrupt Enable*/
#define CAN_IER_EPVIE                        (0x1U << 9U)                      /* Error Passive Interrupt Enable*/
#define CAN_IER_BOFIE                        (0x1U << 10U)                     /* Bus-Off Interrupt Enable*/
#define CAN_IER_LECIE                        (0x1U << 11U)                     /* Last Error Code Interrupt Enable*/
#define CAN_IER_ERRIE                        (0x1U << 15U)                     /* Error Interrupt Enable*/
#define CAN_IER_WKUIE                        (0x1U << 16U)                     /* Wakeup Interrupt Enable*/
#define CAN_IER_SLKIE                        (0x1U << 17U)                     /* Sleep Interrupt Enable*/

#define CAN_ESR_EWGF                         (0x1U << 0U)                      /* Error Warning Flag*/
#define CAN_ESR_EPVF                         (0x1U << 1U)                      /* Error Passive Flag*/
#define CAN_ESR_BOFF                         (0x1U << 2U)                      /* Bus-Off Flag*/
#define CAN_ESR_LEC                          (0x7U << 4U)                      /* LEC[2:0] bits (Last Error Code)*/
#define CAN_ESR_LEC_0                        (0x1U << 4U)
#define CAN_ESR_LEC_1                        (0x2U << 4U)
#define CAN_ESR_LEC_2                        (0x4U << 4U)
#define CAN_ESR_TEC                          (0xFFU << 16U)                    /* Least significant byte of the 9-bit Transmit Error Counter*/
#define CAN_ESR_REC                          (0xFFU << 24U)                    /* Receive Error Counter*/

#define CAN_BTR_BRP                          (0x3FFU << 0U)                    /*Baud Rate Prescaler*/
#define CAN_BTR_TS1                          (0xFU << 16U)                     /*Time Segment 1*/
#define CAN_BTR_TS1_0                        (0x1U << 16U)
#define CAN_BTR_TS1_1                        (0x2U << 16U)
#define CAN_BTR_TS1_2                        (0x4U << 16U)
#define CAN_BTR_TS1_3                        (0x8U << 16U)
#define CAN_BTR_TS2                          (0x7U << 20U)                     /*Time Segment 2*/
#define CAN_BTR_TS2_0                        (0x1U << 20U)
#define CAN_BTR_TS2_1                        (0x2U << 20U)
#define CAN_BTR_TS2_2                        (0x4U << 20U)
#define CAN_BTR_SJW                          (0x3U << 24U)                     /*Resynchronization Jump Width*/
#define CAN_BTR_SJW_0                        (0x1U << 24U)
#define CAN_BTR_SJW_1                        (0x2U << 24U)
#define CAN_BTR_LBKM                         (0x1U << 30U)                     /*Loop Back Mode (Debug)*/
#define CAN_BTR_SILM                         (0x1U << 31U)                     /*Silent Mode*/

#define CAN_TI0R_TXRQ                        (0x1U << 0U)                      /* Transmit Mailbox Request*/
#define CAN_TI0R_RTR                         (0x1U << 1U)                      /* Remote Transmission Request*/
#define CAN_TI0R_IDE                         (0x1U << 2U)                      /* Identifier Extension*/
#define CAN_TI0R_EXID                        (0x3FFFFU << 3U)                  /* Extended Identifier*/
#define CAN_TI0R_STID                        (0x7FFU << 21U)                   /* Standard Identifier or Extended Identifier*/

#define CAN_TDT0R_DLC                        (0xFU << 0U)                      /* Data Length Code*/
#define CAN_TDT0R_TGT                        (0x1U << 8U)                      /* Transmit Global Time*/
#define CAN_TDT0R_TIME                       (0xFFFFU << 16U)                  /* Message Time Stamp*/

#define CAN_TDL0R_DATA0                      (0xFFU << 0U)                     /* Data byte 0*/
#define CAN_TDL0R_DATA1                      (0xFFU << 8U)                     /* Data byte 1*/
#define CAN_TDL0R_DATA2                      (0xFFU << 16U)                    /* Data byte 2*/
#define CAN_TDL0R_DATA3                      (0xFFU << 24U)                    /* Data byte 3*/

#define CAN_TDH0R_DATA4                      (0xFFU << 0U)                     /* Data byte 4*/
#define CAN_TDH0R_DATA5                      (0xFFU << 8U)                     /* Data byte 5*/
#define CAN_TDH0R_DATA6                      (0xFFU << 16U)                    /* Data byte 6*/
#define CAN_TDH0R_DATA7                      (0xFFU << 24U)                    /* Data byte 7*/

#define CAN_TI1R_TXRQ                        (0x1U << 0U)                      /* Transmit Mailbox Request*/
#define CAN_TI1R_RTR                         (0x1U << 1U)                      /* Remote Transmission Request*/
#define CAN_TI1R_IDE                         (0x1U << 2U)                      /* Identifier Extension*/
#define CAN_TI1R_EXID                        (0x3FFFFU << 3U)                  /* Extended Identifier*/
#define CAN_TI1R_STID                        (0x7FFU << 21U)                   /* Standard Identifier or Extended Identifier*/

#define CAN_TDT1R_DLC                        (0xFU << 0U)                      /* Data Length Code*/
#define CAN_TDT1R_TGT                        (0x1U << 8U)                      /* Transmit Global Time*/
#define CAN_TDT1R_TIME                       (0xFFFFU << 16U)                  /* Message Time Stamp*/

#define CAN_TDL1R_DATA0                      (0xFFU << 0U)                     /* Data byte 0*/
#define CAN_TDL1R_DATA1                      (0xFFU << 8U)                     /* Data byte 1*/
#define CAN_TDL1R_DATA2                      (0xFFU << 16U)                    /* Data byte 2*/
#define CAN_TDL1R_DATA3                      (0xFFU << 24U)                    /* Data byte 3*/

#define CAN_TDH1R_DATA4                      (0xFFU << 0U)                     /* Data byte 4*/
#define CAN_TDH1R_DATA5                      (0xFFU << 8U)                     /* Data byte 5*/
#define CAN_TDH1R_DATA6                      (0xFFU << 16U)                    /* Data byte 6*/
#define CAN_TDH1R_DATA7                      (0xFFU << 24U)                    /* Data byte 7*/

#define CAN_TI2R_TXRQ                        (0x1U << 0U)                      /* Transmit Mailbox Request*/
#define CAN_TI2R_RTR                         (0x1U << 1U)                      /* Remote Transmission Request*/
#define CAN_TI2R_IDE                         (0x1U << 2U)                      /* Identifier Extension*/
#define CAN_TI2R_EXID                        (0x3FFFFU << 3U)                  /* Extended identifier*/
#define CAN_TI2R_STID                        (0x7FFU << 21U)                   /* Standard Identifier or Extended Identifier*/

#define CAN_TDT2R_DLC                        (0xFU << 0U)                      /* Data Length Code*/
#define CAN_TDT2R_TGT                        (0x1U << 8U)                      /* Transmit Global Time*/
#define CAN_TDT2R_TIME                       (0xFFFFU << 16U)                 /* Message Time Stamp*/

#define CAN_TDL2R_DATA0                      (0xFFU << 0U)                     /* Data byte 0*/
#define CAN_TDL2R_DATA1                      (0xFFU << 8U)                     /* Data byte 1*/
#define CAN_TDL2R_DATA2                      (0xFFU << 16U)                    /* Data byte 2*/
#define CAN_TDL2R_DATA3                      (0xFFU << 24U)                    /* Data byte 3*/

#define CAN_TDH2R_DATA4                      (0xFFU << 0U)                     /* Data byte 4*/
#define CAN_TDH2R_DATA5                      (0xFFU << 8U)                     /* Data byte 5*/
#define CAN_TDH2R_DATA6                      (0xFFU << 16U)                    /* Data byte 6*/
#define CAN_TDH2R_DATA7                      (0xFFU << 24U)                    /* Data byte 7*/

#define CAN_RI0R_RTR                         (0x1U << 1U)                      /* Remote Transmission Request*/
#define CAN_RI0R_IDE                         (0x1U << 2U)                      /* Identifier Extension*/
#define CAN_RI0R_EXID                        (0x3FFFFU << 3U)                  /* Extended Identifier*/
#define CAN_RI0R_STID                        (0x7FFU << 21U)                   /* Standard Identifier or Extended Identifier*/

#define CAN_RDT0R_DLC                        (0xFU << 0U)                      /* Data Length Code*/
#define CAN_RDT0R_FMI                        (0xFFU << 8U)                     /* Filter Match Index*/
#define CAN_RDT0R_TIME                       (0xFFFFU << 16U)                  /* Message Time Stamp*/

#define CAN_RDL0R_DATA0                      (0xFFU << 0U)                     /* Data byte 0*/
#define CAN_RDL0R_DATA1                      (0xFFU << 8U)                     /* Data byte 1*/
#define CAN_RDL0R_DATA2                      (0xFFU << 16U)                    /* Data byte 2*/
#define CAN_RDL0R_DATA3                      (0xFFU << 24U)                    /* Data byte 3*/

#define CAN_RDH0R_DATA4                      (0xFFU << 0U)                     /* Data byte 4*/
#define CAN_RDH0R_DATA5                      (0xFFU << 8U)                     /* Data byte 5*/
#define CAN_RDH0R_DATA6                      (0xFFU << 16U)                    /* Data byte 6*/
#define CAN_RDH0R_DATA7                      (0xFFU << 24U)                    /* Data byte 7*/

#define CAN_RI1R_RTR                         (0x1U << 1U)                      /* Remote Transmission Request*/
#define CAN_RI1R_IDE                         (0x1U << 2U)                      /* Identifier Extension*/
#define CAN_RI1R_EXID                        (0x3FFFFU << 3U)                  /* Extended identifier*/
#define CAN_RI1R_STID                        (0x7FFU << 21U)                   /* Standard Identifier or Extended Identifier*/

#define CAN_RDT1R_DLC                        (0xFU << 0U)                      /* Data Length Code*/
#define CAN_RDT1R_FMI                        (0xFFU << 8U)                     /* Filter Match Index*/
#define CAN_RDT1R_TIME                       (0xFFFFU << 16U)                  /* Message Time Stamp*/

#define CAN_RDL1R_DATA0                      (0xFFU << 0U)                    /* Data byte 0*/
#define CAN_RDL1R_DATA1                      (0xFFU << 8U)                    /* Data byte 1*/
#define CAN_RDL1R_DATA2                      (0xFFU << 16U)                   /* Data byte 2*/
#define CAN_RDL1R_DATA3                      (0xFFU << 24U)                   /* Data byte 3*/

#define CAN_RDH1R_DATA4                      (0xFFU << 0U)                     /* Data byte 4*/
#define CAN_RDH1R_DATA5                      (0xFFU << 8U)                     /* Data byte 5*/
#define CAN_RDH1R_DATA6                      (0xFFU << 16U)                    /* Data byte 6*/
#define CAN_RDH1R_DATA7                      (0xFFU << 24U)                    /* Data byte 7*/

#define CAN_FMR_FINIT                        (0x1U << 0U)                      /* Filter Init Mode*/
#define CAN_FMR_CAN2SB                       (0x3FU << 8U)                     /* CAN2 start bank*/

#define CAN_FM1R_FBM                         (0x3FFFU << 0U)                   /* Filter Mode */
#define CAN_FM1R_FBM0                        (0x1U << 0U)                      /* Filter Init Mode for filter 0*/
#define CAN_FM1R_FBM1                        (0x1U << 1U)                      /* Filter Init Mode for filter 1*/
#define CAN_FM1R_FBM2                        (0x1U << 2U)                      /* Filter Init Mode for filter 2*/
#define CAN_FM1R_FBM3                        (0x1U << 3U)                      /* Filter Init Mode for filter 3*/
#define CAN_FM1R_FBM4                        (0x1U << 4U)                      /* Filter Init Mode for filter 4*/
#define CAN_FM1R_FBM5                        (0x1U << 5U)                      /* Filter Init Mode for filter 5*/
#define CAN_FM1R_FBM6                        (0x1U << 6U)                      /* Filter Init Mode for filter 6*/
#define CAN_FM1R_FBM7                        (0x1U << 7U)                      /* Filter Init Mode for filter 7*/
#define CAN_FM1R_FBM8                        (0x1U << 8U)                      /* Filter Init Mode for filter 8*/
#define CAN_FM1R_FBM9                        (0x1U << 9U)                      /* Filter Init Mode for filter 9*/
#define CAN_FM1R_FBM10                       (0x1U << 10U)                     /* Filter Init Mode for filter 10*/
#define CAN_FM1R_FBM11                       (0x1U << 11U)                     /* Filter Init Mode for filter 11*/
#define CAN_FM1R_FBM12                       (0x1U << 12U)                     /* Filter Init Mode for filter 12*/
#define CAN_FM1R_FBM13                       (0x1U << 13U)                     /* Filter Init Mode for filter 13*/

#define CAN_FS1R_FSC                         (0x3FFFU << 0U)                   /* Filter Scale Configuration*/
#define CAN_FS1R_FSC0                        (0x1U << 0U)                      /* Filter Scale Configuration for filter 0*/
#define CAN_FS1R_FSC1                        (0x1U << 1U)                      /* Filter Scale Configuration for filter 1*/
#define CAN_FS1R_FSC2                        (0x1U << 2U)                      /* Filter Scale Configuration for filter 2*/
#define CAN_FS1R_FSC3                        (0x1U << 3U)                      /* Filter Scale Configuration for filter 3*/
#define CAN_FS1R_FSC4                        (0x1U << 4U)                      /* Filter Scale Configuration for filter 4*/
#define CAN_FS1R_FSC5                        (0x1U << 5U)                      /* Filter Scale Configuration for filter 5*/
#define CAN_FS1R_FSC6                        (0x1U << 6U)                      /* Filter Scale Configuration for filter 6*/
#define CAN_FS1R_FSC7                        (0x1U << 7U)                      /* Filter Scale Configuration for filter 7*/
#define CAN_FS1R_FSC8                        (0x1U << 8U)                      /* Filter Scale Configuration for filter 8*/
#define CAN_FS1R_FSC9                        (0x1U << 9U)                      /* Filter Scale Configuration for filter 9*/
#define CAN_FS1R_FSC10                       (0x1U << 10U)                     /* Filter Scale Configuration for filter 10*/
#define CAN_FS1R_FSC11                       (0x1U << 11U)                     /* Filter Scale Configuration for filter 11*/
#define CAN_FS1R_FSC12                       (0x1U << 12U)                     /* Filter Scale Configuration for filter 12*/
#define CAN_FS1R_FSC13                       (0x1U << 13U)                     /* Filter Scale Configuration for filter 13*/

#define CAN_FFA1R_FFA                        (0x3FFFU << 0U)                   /* Filter FIFO Assignment*/
#define CAN_FFA1R_FFA0                       (0x1U << 0U)                      /* Filter FIFO Assignment for filter 0*/
#define CAN_FFA1R_FFA1                       (0x1U << 1U)                      /* Filter FIFO Assignment for filter 1*/
#define CAN_FFA1R_FFA2                       (0x1U << 2U)                      /* Filter FIFO Assignment for filter 2*/
#define CAN_FFA1R_FFA3                       (0x1U << 3U)                      /* Filter FIFO Assignment for filter 3*/
#define CAN_FFA1R_FFA4                       (0x1U << 4U)                      /* Filter FIFO Assignment for filter 4*/
#define CAN_FFA1R_FFA5                       (0x1U << 5U)                      /* Filter FIFO Assignment for filter 5*/
#define CAN_FFA1R_FFA6                       (0x1U << 6U)                      /* Filter FIFO Assignment for filter 6*/
#define CAN_FFA1R_FFA7                       (0x1U << 7U)                      /* Filter FIFO Assignment for filter 7*/
#define CAN_FFA1R_FFA8                       (0x1U << 8U)                      /* Filter FIFO Assignment for filter 8*/
#define CAN_FFA1R_FFA9                       (0x1U << 9U)                      /* Filter FIFO Assignment for filter 9*/
#define CAN_FFA1R_FFA10                      (0x1U << 10U)                     /* Filter FIFO Assignment for filter 10*/
#define CAN_FFA1R_FFA11                      (0x1U << 11U)                     /* Filter FIFO Assignment for filter 11*/
#define CAN_FFA1R_FFA12                      (0x1U << 12U)                     /* Filter FIFO Assignment for filter 12*/
#define CAN_FFA1R_FFA13                      (0x1U << 13U)                     /* Filter FIFO Assignment for filter 13*/

#define CAN_FA1R_FACT                        (0x3FFFU << 0U)                   /* Filter Active*/
#define CAN_FA1R_FACT0                       (0x1U << 0U)                      /* Filter 0 Active*/
#define CAN_FA1R_FACT1                       (0x1U << 1U)                      /* Filter 1 Active*/
#define CAN_FA1R_FACT2                       (0x1U << 2U)                      /* Filter 2 Active*/
#define CAN_FA1R_FACT3                       (0x1U << 3U)                      /* Filter 3 Active*/
#define CAN_FA1R_FACT4                       (0x1U << 4U)                      /* Filter 4 Active*/
#define CAN_FA1R_FACT5                       (0x1U << 5U)                      /* Filter 5 Active*/
#define CAN_FA1R_FACT6                       (0x1U << 6U)                      /* Filter 6 Active*/
#define CAN_FA1R_FACT7                       (0x1U << 7U)                      /* Filter 7 Active*/
#define CAN_FA1R_FACT8                       (0x1U << 8U)                      /* Filter 8 Active*/
#define CAN_FA1R_FACT9                       (0x1U << 9U)                      /* Filter 9 Active*/
#define CAN_FA1R_FACT10                      (0x1U << 10U)                     /* Filter 10 Active*/
#define CAN_FA1R_FACT11                      (0x1U << 11U)                     /* Filter 11 Active*/
#define CAN_FA1R_FACT12                      (0x1U << 12U)                     /* Filter 12 Active*/
#define CAN_FA1R_FACT13                      (0x1U << 13U)                     /* Filter 13 Active*/

#define CAN_F0R1_FB0                         (0x1U << 0U)                      /* Filter bit 0*/
#define CAN_F0R1_FB1                         (0x1U << 1U)                      /* Filter bit 1*/
#define CAN_F0R1_FB2                         (0x1U << 2U)                      /* Filter bit 2*/
#define CAN_F0R1_FB3                         (0x1U << 3U)                      /* Filter bit 3*/
#define CAN_F0R1_FB4                         (0x1U << 4U)                      /* Filter bit 4*/
#define CAN_F0R1_FB5                         (0x1U << 5U)                      /* Filter bit 5*/
#define CAN_F0R1_FB6                         (0x1U << 6U)                      /* Filter bit 6*/
#define CAN_F0R1_FB7                         (0x1U << 7U)                      /* Filter bit 7*/
#define CAN_F0R1_FB8                         (0x1U << 8U)                      /* Filter bit 8*/
#define CAN_F0R1_FB9                         (0x1U << 9U)                      /* Filter bit 9*/
#define CAN_F0R1_FB10                        (0x1U << 10U)                     /* Filter bit 10*/
#define CAN_F0R1_FB11                        (0x1U << 11U)                     /* Filter bit 11*/
#define CAN_F0R1_FB12                        (0x1U << 12U)                     /* Filter bit 12*/
#define CAN_F0R1_FB13                        (0x1U << 13U)                     /* Filter bit 13*/
#define CAN_F0R1_FB14                        (0x1U << 14U)                     /* Filter bit 14*/
#define CAN_F0R1_FB15                        (0x1U << 15U)                     /* Filter bit 15*/
#define CAN_F0R1_FB16                        (0x1U << 16U)                     /* Filter bit 16*/
#define CAN_F0R1_FB17                        (0x1U << 17U)                     /* Filter bit 17*/
#define CAN_F0R1_FB18                        (0x1U << 18U)                     /* Filter bit 18*/
#define CAN_F0R1_FB19                        (0x1U << 19U)                     /* Filter bit 19*/
#define CAN_F0R1_FB20                        (0x1U << 20U)                     /* Filter bit 20*/
#define CAN_F0R1_FB21                        (0x1U << 21U)                     /* Filter bit 21*/
#define CAN_F0R1_FB22                        (0x1U << 22U)                     /* Filter bit 22*/
#define CAN_F0R1_FB23                        (0x1U << 23U)                     /* Filter bit 23*/
#define CAN_F0R1_FB24                        (0x1U << 24U)                     /* Filter bit 24*/
#define CAN_F0R1_FB25                        (0x1U << 25U)                     /* Filter bit 25*/
#define CAN_F0R1_FB26                        (0x1U << 26U)                     /* Filter bit 26*/
#define CAN_F0R1_FB27                        (0x1U << 27U)                     /* Filter bit 27*/
#define CAN_F0R1_FB28                        (0x1U << 28U)                     /* Filter bit 28*/
#define CAN_F0R1_FB29                        (0x1U << 29U)                     /* Filter bit 29*/
#define CAN_F0R1_FB30                        (0x1U << 30U)                     /* Filter bit 30*/
#define CAN_F0R1_FB31                        (0x1U << 31U)                     /* Filter bit 31*/

#define CAN_F1R1_FB0                         (0x1U << 0U)                      /* Filter bit 0*/
#define CAN_F1R1_FB1                         (0x1U << 1U)                      /* Filter bit 1*/
#define CAN_F1R1_FB2                         (0x1U << 2U)                      /* Filter bit 2*/
#define CAN_F1R1_FB3                         (0x1U << 3U)                      /* Filter bit 3*/
#define CAN_F1R1_FB4                         (0x1U << 4U)                      /* Filter bit 4*/
#define CAN_F1R1_FB5                         (0x1U << 5U)                      /* Filter bit 5*/
#define CAN_F1R1_FB6                         (0x1U << 6U)                      /* Filter bit 6*/
#define CAN_F1R1_FB7                         (0x1U << 7U)                      /* Filter bit 7*/
#define CAN_F1R1_FB8                         (0x1U << 8U)                      /* Filter bit 8*/
#define CAN_F1R1_FB9                         (0x1U << 9U)                      /* Filter bit 9*/
#define CAN_F1R1_FB10                        (0x1U << 10U)                     /* Filter bit 10*/
#define CAN_F1R1_FB11                        (0x1U << 11U)                     /* Filter bit 11*/
#define CAN_F1R1_FB12                        (0x1U << 12U)                     /* Filter bit 12*/
#define CAN_F1R1_FB13                        (0x1U << 13U)                     /* Filter bit 13*/
#define CAN_F1R1_FB14                        (0x1U << 14U)                     /* Filter bit 14*/
#define CAN_F1R1_FB15                        (0x1U << 15U)                     /* Filter bit 15*/
#define CAN_F1R1_FB16                        (0x1U << 16U)                     /* Filter bit 16*/
#define CAN_F1R1_FB17                        (0x1U << 17U)                     /* Filter bit 17*/
#define CAN_F1R1_FB18                        (0x1U << 18U)                     /* Filter bit 18*/
#define CAN_F1R1_FB19                        (0x1U << 19U)                     /* Filter bit 19*/
#define CAN_F1R1_FB20                        (0x1U << 20U)                     /* Filter bit 20*/
#define CAN_F1R1_FB21                        (0x1U << 21U)                     /* Filter bit 21*/
#define CAN_F1R1_FB22                        (0x1U << 22U)                     /* Filter bit 22*/
#define CAN_F1R1_FB23                        (0x1U << 23U)                     /* Filter bit 23*/
#define CAN_F1R1_FB24                        (0x1U << 24U)                     /* Filter bit 24*/
#define CAN_F1R1_FB25                        (0x1U << 25U)                     /* Filter bit 25*/
#define CAN_F1R1_FB26                        (0x1U << 26U)                     /* Filter bit 26*/
#define CAN_F1R1_FB27                        (0x1U << 27U)                     /* Filter bit 27*/
#define CAN_F1R1_FB28                        (0x1U << 28U)                     /* Filter bit 28*/
#define CAN_F1R1_FB29                        (0x1U << 29U)                     /* Filter bit 29*/
#define CAN_F1R1_FB30                        (0x1U << 30U)                     /* Filter bit 30*/
#define CAN_F1R1_FB31                        (0x1U << 31U)                     /* Filter bit 31*/

#define CAN_F2R1_FB0                         (0x1U << 0U)                      /* Filter bit 0*/
#define CAN_F2R1_FB1                         (0x1U << 1U)                      /* Filter bit 1*/
#define CAN_F2R1_FB2                         (0x1U << 2U)                      /* Filter bit 2*/
#define CAN_F2R1_FB3                         (0x1U << 3U)                      /* Filter bit 3*/
#define CAN_F2R1_FB4                         (0x1U << 4U)                      /* Filter bit 4*/
#define CAN_F2R1_FB5                         (0x1U << 5U)                      /* Filter bit 5*/
#define CAN_F2R1_FB6                         (0x1U << 6U)                      /* Filter bit 6*/
#define CAN_F2R1_FB7                         (0x1U << 7U)                      /* Filter bit 7*/
#define CAN_F2R1_FB8                         (0x1U << 8U)                      /* Filter bit 8*/
#define CAN_F2R1_FB9                         (0x1U << 9U)                      /* Filter bit 9*/
#define CAN_F2R1_FB10                        (0x1U << 10U)                     /* Filter bit 10*/
#define CAN_F2R1_FB11                        (0x1U << 11U)                     /* Filter bit 11*/
#define CAN_F2R1_FB12                        (0x1U << 12U)                     /* Filter bit 12*/
#define CAN_F2R1_FB13                        (0x1U << 13U)                     /* Filter bit 13*/
#define CAN_F2R1_FB14                        (0x1U << 14U)                     /* Filter bit 14*/
#define CAN_F2R1_FB15                        (0x1U << 15U)                     /* Filter bit 15*/
#define CAN_F2R1_FB16                        (0x1U << 16U)                     /* Filter bit 16*/
#define CAN_F2R1_FB17                        (0x1U << 17U)                     /* Filter bit 17*/
#define CAN_F2R1_FB18                        (0x1U << 18U)                     /* Filter bit 18*/
#define CAN_F2R1_FB19                        (0x1U << 19U)                     /* Filter bit 19*/
#define CAN_F2R1_FB20                        (0x1U << 20U)                     /* Filter bit 20*/
#define CAN_F2R1_FB21                        (0x1U << 21U)                     /* Filter bit 21*/
#define CAN_F2R1_FB22                        (0x1U << 22U)                     /* Filter bit 22*/
#define CAN_F2R1_FB23                        (0x1U << 23U)                     /* Filter bit 23*/
#define CAN_F2R1_FB24                        (0x1U << 24U)                     /* Filter bit 24*/
#define CAN_F2R1_FB25                        (0x1U << 25U)                     /* Filter bit 25*/
#define CAN_F2R1_FB26                        (0x1U << 26U)                     /* Filter bit 26*/
#define CAN_F2R1_FB27                        (0x1U << 27U)                     /* Filter bit 27*/
#define CAN_F2R1_FB28                        (0x1U << 28U)                     /* Filter bit 28*/
#define CAN_F2R1_FB29                        (0x1U << 29U)                     /* Filter bit 29*/
#define CAN_F2R1_FB30                        (0x1U << 30U)                     /* Filter bit 30*/
#define CAN_F2R1_FB31                        (0x1U << 31U)                     /* Filter bit 31*/

#define CAN_F3R1_FB0                         (0x1U << 0U)                      /* Filter bit 0*/
#define CAN_F3R1_FB1                         (0x1U << 1U)                      /* Filter bit 1*/
#define CAN_F3R1_FB2                         (0x1U << 2U)                      /* Filter bit 2*/
#define CAN_F3R1_FB3                         (0x1U << 3U)                      /* Filter bit 3*/
#define CAN_F3R1_FB4                         (0x1U << 4U)                      /* Filter bit 4*/
#define CAN_F3R1_FB5                         (0x1U << 5U)                      /* Filter bit 5*/
#define CAN_F3R1_FB6                         (0x1U << 6U)                      /* Filter bit 6*/
#define CAN_F3R1_FB7                         (0x1U << 7U)                      /* Filter bit 7*/
#define CAN_F3R1_FB8                         (0x1U << 8U)                      /* Filter bit 8*/
#define CAN_F3R1_FB9                         (0x1U << 9U)                      /* Filter bit 9*/
#define CAN_F3R1_FB10                        (0x1U << 10U)                     /* Filter bit 10*/
#define CAN_F3R1_FB11                        (0x1U << 11U)                     /* Filter bit 11*/
#define CAN_F3R1_FB12                        (0x1U << 12U)                     /* Filter bit 12*/
#define CAN_F3R1_FB13                        (0x1U << 13U)                     /* Filter bit 13*/
#define CAN_F3R1_FB14                        (0x1U << 14U)                     /* Filter bit 14*/
#define CAN_F3R1_FB15                        (0x1U << 15U)                     /* Filter bit 15*/
#define CAN_F3R1_FB16                        (0x1U << 16U)                     /* Filter bit 16*/
#define CAN_F3R1_FB17                        (0x1U << 17U)                     /* Filter bit 17*/
#define CAN_F3R1_FB18                        (0x1U << 18U)                     /* Filter bit 18*/
#define CAN_F3R1_FB19                        (0x1U << 19U)                     /* Filter bit 19*/
#define CAN_F3R1_FB20                        (0x1U << 20U)                     /* Filter bit 20*/
#define CAN_F3R1_FB21                        (0x1U << 21U)                     /* Filter bit 21*/
#define CAN_F3R1_FB22                        (0x1U << 22U)                     /* Filter bit 22*/
#define CAN_F3R1_FB23                        (0x1U << 23U)                     /* Filter bit 23*/
#define CAN_F3R1_FB24                        (0x1U << 24U)                     /* Filter bit 24*/
#define CAN_F3R1_FB25                        (0x1U << 25U)                     /* Filter bit 25*/
#define CAN_F3R1_FB26                        (0x1U << 26U)                     /* Filter bit 26*/
#define CAN_F3R1_FB27                        (0x1U << 27U)                     /* Filter bit 27*/
#define CAN_F3R1_FB28                        (0x1U << 28U)                     /* Filter bit 28*/
#define CAN_F3R1_FB29                        (0x1U << 29U)                     /* Filter bit 29*/
#define CAN_F3R1_FB30                        (0x1U << 30U)                     /* Filter bit 30*/
#define CAN_F3R1_FB31                        (0x1U << 31U)                     /* Filter bit 31*/

#define CAN_F4R1_FB0                         (0x1U << 0U)                      /* Filter bit 0*/
#define CAN_F4R1_FB1                         (0x1U << 1U)                      /* Filter bit 1*/
#define CAN_F4R1_FB2                         (0x1U << 2U)                      /* Filter bit 2*/
#define CAN_F4R1_FB3                         (0x1U << 3U)                      /* Filter bit 3*/
#define CAN_F4R1_FB4                         (0x1U << 4U)                      /* Filter bit 4*/
#define CAN_F4R1_FB5                         (0x1U << 5U)                      /* Filter bit 5*/
#define CAN_F4R1_FB6                         (0x1U << 6U)                      /* Filter bit 6*/
#define CAN_F4R1_FB7                         (0x1U << 7U)                      /* Filter bit 7*/
#define CAN_F4R1_FB8                         (0x1U << 8U)                      /* Filter bit 8*/
#define CAN_F4R1_FB9                         (0x1U << 9U)                      /* Filter bit 9*/
#define CAN_F4R1_FB10                        (0x1U << 10U)                     /* Filter bit 10*/
#define CAN_F4R1_FB11                        (0x1U << 11U)                     /* Filter bit 11*/
#define CAN_F4R1_FB12                        (0x1U << 12U)                     /* Filter bit 12*/
#define CAN_F4R1_FB13                        (0x1U << 13U)                     /* Filter bit 13*/
#define CAN_F4R1_FB14                        (0x1U << 14U)                     /* Filter bit 14*/
#define CAN_F4R1_FB15                        (0x1U << 15U)                     /* Filter bit 15*/
#define CAN_F4R1_FB16                        (0x1U << 16U)                     /* Filter bit 16*/
#define CAN_F4R1_FB17                        (0x1U << 17U)                     /* Filter bit 17*/
#define CAN_F4R1_FB18                        (0x1U << 18U)                     /* Filter bit 18*/
#define CAN_F4R1_FB19                        (0x1U << 19U)                     /* Filter bit 19*/
#define CAN_F4R1_FB20                        (0x1U << 20U)                     /* Filter bit 20*/
#define CAN_F4R1_FB21                        (0x1U << 21U)                     /* Filter bit 21*/
#define CAN_F4R1_FB22                        (0x1U << 22U)                     /* Filter bit 22*/
#define CAN_F4R1_FB23                        (0x1U << 23U)                     /* Filter bit 23*/
#define CAN_F4R1_FB24                        (0x1U << 24U)                     /* Filter bit 24*/
#define CAN_F4R1_FB25                        (0x1U << 25U)                     /* Filter bit 25*/
#define CAN_F4R1_FB26                        (0x1U << 26U)                     /* Filter bit 26*/
#define CAN_F4R1_FB27                        (0x1U << 27U)                     /* Filter bit 27*/
#define CAN_F4R1_FB28                        (0x1U << 28U)                     /* Filter bit 28*/
#define CAN_F4R1_FB29                        (0x1U << 29U)                     /* Filter bit 29*/
#define CAN_F4R1_FB30                        (0x1U << 30U)                     /* Filter bit 30*/
#define CAN_F4R1_FB31                        (0x1U << 31U)                     /* Filter bit 31*/

#define CAN_F5R1_FB0                         (0x1U << 0U)                      /* Filter bit 0*/
#define CAN_F5R1_FB1                         (0x1U << 1U)                      /* Filter bit 1*/
#define CAN_F5R1_FB2                         (0x1U << 2U)                      /* Filter bit 2*/
#define CAN_F5R1_FB3                         (0x1U << 3U)                      /* Filter bit 3*/
#define CAN_F5R1_FB4                         (0x1U << 4U)                      /* Filter bit 4*/
#define CAN_F5R1_FB5                         (0x1U << 5U)                      /* Filter bit 5*/
#define CAN_F5R1_FB6                         (0x1U << 6U)                      /* Filter bit 6*/
#define CAN_F5R1_FB7                         (0x1U << 7U)                      /* Filter bit 7*/
#define CAN_F5R1_FB8                         (0x1U << 8U)                      /* Filter bit 8*/
#define CAN_F5R1_FB9                         (0x1U << 9U)                      /* Filter bit 9*/
#define CAN_F5R1_FB10                        (0x1U << 10U)                     /* Filter bit 10*/
#define CAN_F5R1_FB11                        (0x1U << 11U)                     /* Filter bit 11*/
#define CAN_F5R1_FB12                        (0x1U << 12U)                     /* Filter bit 12*/
#define CAN_F5R1_FB13                        (0x1U << 13U)                     /* Filter bit 13*/
#define CAN_F5R1_FB14                        (0x1U << 14U)                     /* Filter bit 14*/
#define CAN_F5R1_FB15                        (0x1U << 15U)                     /* Filter bit 15*/
#define CAN_F5R1_FB16                        (0x1U << 16U)                     /* Filter bit 16*/
#define CAN_F5R1_FB17                        (0x1U << 17U)                     /* Filter bit 17*/
#define CAN_F5R1_FB18                        (0x1U << 18U)                     /* Filter bit 18*/
#define CAN_F5R1_FB19                        (0x1U << 19U)                     /* Filter bit 19*/
#define CAN_F5R1_FB20                        (0x1U << 20U)                     /* Filter bit 20*/
#define CAN_F5R1_FB21                        (0x1U << 21U)                     /* Filter bit 21*/
#define CAN_F5R1_FB22                        (0x1U << 22U)                     /* Filter bit 22*/
#define CAN_F5R1_FB23                        (0x1U << 23U)                     /* Filter bit 23*/
#define CAN_F5R1_FB24                        (0x1U << 24U)                     /* Filter bit 24*/
#define CAN_F5R1_FB25                        (0x1U << 25U)                     /* Filter bit 25*/
#define CAN_F5R1_FB26                        (0x1U << 26U)                     /* Filter bit 26*/
#define CAN_F5R1_FB27                        (0x1U << 27U)                     /* Filter bit 27*/
#define CAN_F5R1_FB28                        (0x1U << 28U)                     /* Filter bit 28*/
#define CAN_F5R1_FB29                        (0x1U << 29U)                     /* Filter bit 29*/
#define CAN_F5R1_FB30                        (0x1U << 30U)                     /* Filter bit 30*/
#define CAN_F5R1_FB31                        (0x1U << 31U)                     /* Filter bit 31*/

#define CAN_F6R1_FB0                         (0x1U << 0U)                      /* Filter bit 0*/
#define CAN_F6R1_FB1                         (0x1U << 1U)                      /* Filter bit 1*/
#define CAN_F6R1_FB2                         (0x1U << 2U)                      /* Filter bit 2*/
#define CAN_F6R1_FB3                         (0x1U << 3U)                      /* Filter bit 3*/
#define CAN_F6R1_FB4                         (0x1U << 4U)                      /* Filter bit 4*/
#define CAN_F6R1_FB5                         (0x1U << 5U)                      /* Filter bit 5*/
#define CAN_F6R1_FB6                         (0x1U << 6U)                      /* Filter bit 6*/
#define CAN_F6R1_FB7                         (0x1U << 7U)                      /* Filter bit 7*/
#define CAN_F6R1_FB8                         (0x1U << 8U)                      /* Filter bit 8*/
#define CAN_F6R1_FB9                         (0x1U << 9U)                      /* Filter bit 9*/
#define CAN_F6R1_FB10                        (0x1U << 10U)                     /* Filter bit 10*/
#define CAN_F6R1_FB11                        (0x1U << 11U)                     /* Filter bit 11*/
#define CAN_F6R1_FB12                        (0x1U << 12U)                     /* Filter bit 12*/
#define CAN_F6R1_FB13                        (0x1U << 13U)                     /* Filter bit 13*/
#define CAN_F6R1_FB14                        (0x1U << 14U)                     /* Filter bit 14*/
#define CAN_F6R1_FB15                        (0x1U << 15U)                     /* Filter bit 15*/
#define CAN_F6R1_FB16                        (0x1U << 16U)                     /* Filter bit 16*/
#define CAN_F6R1_FB17                        (0x1U << 17U)                     /* Filter bit 17*/
#define CAN_F6R1_FB18                        (0x1U << 18U)                     /* Filter bit 18*/
#define CAN_F6R1_FB19                        (0x1U << 19U)                     /* Filter bit 19*/
#define CAN_F6R1_FB20                        (0x1U << 20U)                     /* Filter bit 20*/
#define CAN_F6R1_FB21                        (0x1U << 21U)                     /* Filter bit 21*/
#define CAN_F6R1_FB22                        (0x1U << 22U)                     /* Filter bit 22*/
#define CAN_F6R1_FB23                        (0x1U << 23U)                     /* Filter bit 23*/
#define CAN_F6R1_FB24                        (0x1U << 24U)                     /* Filter bit 24*/
#define CAN_F6R1_FB25                        (0x1U << 25U)                     /* Filter bit 25*/
#define CAN_F6R1_FB26                        (0x1U << 26U)                     /* Filter bit 26*/
#define CAN_F6R1_FB27                        (0x1U << 27U)                     /* Filter bit 27*/
#define CAN_F6R1_FB28                        (0x1U << 28U)                     /* Filter bit 28*/
#define CAN_F6R1_FB29                        (0x1U << 29U)                     /* Filter bit 29*/
#define CAN_F6R1_FB30                        (0x1U << 30U)                     /* Filter bit 30*/
#define CAN_F6R1_FB31                        (0x1U << 31U)                     /* Filter bit 31*/

#define CAN_F7R1_FB0                         (0x1U << 0U)                      /* Filter bit 0*/
#define CAN_F7R1_FB1                         (0x1U << 1U)                      /* Filter bit 1*/
#define CAN_F7R1_FB2                         (0x1U << 2U)                      /* Filter bit 2*/
#define CAN_F7R1_FB3                         (0x1U << 3U)                      /* Filter bit 3*/
#define CAN_F7R1_FB4                         (0x1U << 4U)                      /* Filter bit 4*/
#define CAN_F7R1_FB5                         (0x1U << 5U)                      /* Filter bit 5*/
#define CAN_F7R1_FB6                         (0x1U << 6U)                      /* Filter bit 6*/
#define CAN_F7R1_FB7                         (0x1U << 7U)                      /* Filter bit 7*/
#define CAN_F7R1_FB8                         (0x1U << 8U)                      /* Filter bit 8*/
#define CAN_F7R1_FB9                         (0x1U << 9U)                      /* Filter bit 9*/
#define CAN_F7R1_FB10                        (0x1U << 10U)                     /* Filter bit 10*/
#define CAN_F7R1_FB11                        (0x1U << 11U)                     /* Filter bit 11*/
#define CAN_F7R1_FB12                        (0x1U << 12U)                     /* Filter bit 12*/
#define CAN_F7R1_FB13                        (0x1U << 13U)                     /* Filter bit 13*/
#define CAN_F7R1_FB14                        (0x1U << 14U)                     /* Filter bit 14*/
#define CAN_F7R1_FB15                        (0x1U << 15U)                     /* Filter bit 15*/
#define CAN_F7R1_FB16                        (0x1U << 16U)                     /* Filter bit 16*/
#define CAN_F7R1_FB17                        (0x1U << 17U)                     /* Filter bit 17*/
#define CAN_F7R1_FB18                        (0x1U << 18U)                     /* Filter bit 18*/
#define CAN_F7R1_FB19                        (0x1U << 19U)                     /* Filter bit 19*/
#define CAN_F7R1_FB20                        (0x1U << 20U)                     /* Filter bit 20*/
#define CAN_F7R1_FB21                        (0x1U << 21U)                     /* Filter bit 21*/
#define CAN_F7R1_FB22                        (0x1U << 22U)                     /* Filter bit 22*/
#define CAN_F7R1_FB23                        (0x1U << 23U)                     /* Filter bit 23*/
#define CAN_F7R1_FB24                        (0x1U << 24U)                     /* Filter bit 24*/
#define CAN_F7R1_FB25                        (0x1U << 25U)                     /* Filter bit 25*/
#define CAN_F7R1_FB26                        (0x1U << 26U)                     /* Filter bit 26*/
#define CAN_F7R1_FB27                        (0x1U << 27U)                     /* Filter bit 27*/
#define CAN_F7R1_FB28                        (0x1U << 28U)                     /* Filter bit 28*/
#define CAN_F7R1_FB29                        (0x1U << 29U)                     /* Filter bit 29*/
#define CAN_F7R1_FB30                        (0x1U << 30U)                     /* Filter bit 30*/
#define CAN_F7R1_FB31                        (0x1U << 31U)                     /* Filter bit 31*/

#define CAN_F8R1_FB0                         (0x1U << 0U)                      /* Filter bit 0*/
#define CAN_F8R1_FB1                         (0x1U << 1U)                      /* Filter bit 1*/
#define CAN_F8R1_FB2                         (0x1U << 2U)                      /* Filter bit 2*/
#define CAN_F8R1_FB3                         (0x1U << 3U)                      /* Filter bit 3*/
#define CAN_F8R1_FB4                         (0x1U << 4U)                      /* Filter bit 4*/
#define CAN_F8R1_FB5                         (0x1U << 5U)                      /* Filter bit 5*/
#define CAN_F8R1_FB6                         (0x1U << 6U)                      /* Filter bit 6*/
#define CAN_F8R1_FB7                         (0x1U << 7U)                      /* Filter bit 7*/
#define CAN_F8R1_FB8                         (0x1U << 8U)                      /* Filter bit 8*/
#define CAN_F8R1_FB9                         (0x1U << 9U)                      /* Filter bit 9*/
#define CAN_F8R1_FB10                        (0x1U << 10U)                     /* Filter bit 10*/
#define CAN_F8R1_FB11                        (0x1U << 11U)                     /* Filter bit 11*/
#define CAN_F8R1_FB12                        (0x1U << 12U)                     /* Filter bit 12*/
#define CAN_F8R1_FB13                        (0x1U << 13U)                     /* Filter bit 13*/
#define CAN_F8R1_FB14                        (0x1U << 14U)                     /* Filter bit 14*/
#define CAN_F8R1_FB15                        (0x1U << 15U)                     /* Filter bit 15*/
#define CAN_F8R1_FB16                        (0x1U << 16U)                     /* Filter bit 16*/
#define CAN_F8R1_FB17                        (0x1U << 17U)                     /* Filter bit 17*/
#define CAN_F8R1_FB18                        (0x1U << 18U)                     /* Filter bit 18*/
#define CAN_F8R1_FB19                        (0x1U << 19U)                     /* Filter bit 19*/
#define CAN_F8R1_FB20                        (0x1U << 20U)                     /* Filter bit 20*/
#define CAN_F8R1_FB21                        (0x1U << 21U)                     /* Filter bit 21*/
#define CAN_F8R1_FB22                        (0x1U << 22U)                     /* Filter bit 22*/
#define CAN_F8R1_FB23                        (0x1U << 23U)                     /* Filter bit 23*/
#define CAN_F8R1_FB24                        (0x1U << 24U)                     /* Filter bit 24*/
#define CAN_F8R1_FB25                        (0x1U << 25U)                     /* Filter bit 25*/
#define CAN_F8R1_FB26                        (0x1U << 26U)                     /* Filter bit 26*/
#define CAN_F8R1_FB27                        (0x1U << 27U)                     /* Filter bit 27*/
#define CAN_F8R1_FB28                        (0x1U << 28U)                     /* Filter bit 28*/
#define CAN_F8R1_FB29                        (0x1U << 29U)                     /* Filter bit 29*/
#define CAN_F8R1_FB30                        (0x1U << 30U)                     /* Filter bit 30*/
#define CAN_F8R1_FB31                        (0x1U << 31U)                     /* Filter bit 31*/

#define CAN_F9R1_FB0                         (0x1U << 0U)                      /* Filter bit 0*/
#define CAN_F9R1_FB1                         (0x1U << 1U)                      /* Filter bit 1*/
#define CAN_F9R1_FB2                         (0x1U << 2U)                      /* Filter bit 2*/
#define CAN_F9R1_FB3                         (0x1U << 3U)                      /* Filter bit 3*/
#define CAN_F9R1_FB4                         (0x1U << 4U)                      /* Filter bit 4*/
#define CAN_F9R1_FB5                         (0x1U << 5U)                      /* Filter bit 5*/
#define CAN_F9R1_FB6                         (0x1U << 6U)                      /* Filter bit 6*/
#define CAN_F9R1_FB7                         (0x1U << 7U)                      /* Filter bit 7*/
#define CAN_F9R1_FB8                         (0x1U << 8U)                      /* Filter bit 8*/
#define CAN_F9R1_FB9                         (0x1U << 9U)                      /* Filter bit 9*/
#define CAN_F9R1_FB10                        (0x1U << 10U)                     /* Filter bit 10*/
#define CAN_F9R1_FB11                        (0x1U << 11U)                     /* Filter bit 11*/
#define CAN_F9R1_FB12                        (0x1U << 12U)                     /* Filter bit 12*/
#define CAN_F9R1_FB13                        (0x1U << 13U)                     /* Filter bit 13*/
#define CAN_F9R1_FB14                        (0x1U << 14U)                     /* Filter bit 14*/
#define CAN_F9R1_FB15                        (0x1U << 15U)                     /* Filter bit 15*/
#define CAN_F9R1_FB16                        (0x1U << 16U)                     /* Filter bit 16*/
#define CAN_F9R1_FB17                        (0x1U << 17U)                     /* Filter bit 17*/
#define CAN_F9R1_FB18                        (0x1U << 18U)                     /* Filter bit 18*/
#define CAN_F9R1_FB19                        (0x1U << 19U)                     /* Filter bit 19*/
#define CAN_F9R1_FB20                        (0x1U << 20U)                     /* Filter bit 20*/
#define CAN_F9R1_FB21                        (0x1U << 21U)                     /* Filter bit 21*/
#define CAN_F9R1_FB22                        (0x1U << 22U)                     /* Filter bit 22*/
#define CAN_F9R1_FB23                        (0x1U << 23U)                     /* Filter bit 23*/
#define CAN_F9R1_FB24                        (0x1U << 24U)                     /* Filter bit 24*/
#define CAN_F9R1_FB25                        (0x1U << 25U)                     /* Filter bit 25*/
#define CAN_F9R1_FB26                        (0x1U << 26U)                     /* Filter bit 26*/
#define CAN_F9R1_FB27                        (0x1U << 27U)                     /* Filter bit 27*/
#define CAN_F9R1_FB28                        (0x1U << 28U)                     /* Filter bit 28*/
#define CAN_F9R1_FB29                        (0x1U << 29U)                     /* Filter bit 29*/
#define CAN_F9R1_FB30                        (0x1U << 30U)                     /* Filter bit 30*/
#define CAN_F9R1_FB31                        (0x1U << 31U)                     /* Filter bit 31*/

#define CAN_F10R1_FB0                        (0x1U << 0U)                      /* Filter bit 0*/
#define CAN_F10R1_FB1                        (0x1U << 1U)                      /* Filter bit 1*/
#define CAN_F10R1_FB2                        (0x1U << 2U)                      /* Filter bit 2*/
#define CAN_F10R1_FB3                        (0x1U << 3U)                      /* Filter bit 3*/
#define CAN_F10R1_FB4                        (0x1U << 4U)                      /* Filter bit 4*/
#define CAN_F10R1_FB5                        (0x1U << 5U)                      /* Filter bit 5*/
#define CAN_F10R1_FB6                        (0x1U << 6U)                      /* Filter bit 6*/
#define CAN_F10R1_FB7                        (0x1U << 7U)                      /* Filter bit 7*/
#define CAN_F10R1_FB8                        (0x1U << 8U)                      /* Filter bit 8*/
#define CAN_F10R1_FB9                        (0x1U << 9U)                      /* Filter bit 9*/
#define CAN_F10R1_FB10                       (0x1U << 10U)                     /*  Filter bit 10*/
#define CAN_F10R1_FB11                       (0x1U << 11U)                     /* Filter bit 11*/
#define CAN_F10R1_FB12                       (0x1U << 12U)                     /* Filter bit 12*/
#define CAN_F10R1_FB13                       (0x1U << 13U)                     /* Filter bit 13*/
#define CAN_F10R1_FB14                       (0x1U << 14U)                     /* Filter bit 14*/
#define CAN_F10R1_FB15                       (0x1U << 15U)                     /* Filter bit 15*/
#define CAN_F10R1_FB16                       (0x1U << 16U)                     /* Filter bit 16*/
#define CAN_F10R1_FB17                       (0x1U << 17U)                     /* Filter bit 17*/
#define CAN_F10R1_FB18                       (0x1U << 18U)                     /* Filter bit 18*/
#define CAN_F10R1_FB19                       (0x1U << 19U)                     /* Filter bit 19*/
#define CAN_F10R1_FB20                       (0x1U << 20U)                     /* Filter bit 20*/
#define CAN_F10R1_FB21                       (0x1U << 21U)                     /* Filter bit 21*/
#define CAN_F10R1_FB22                       (0x1U << 22U)                     /* Filter bit 22*/
#define CAN_F10R1_FB23                       (0x1U << 23U)                     /* Filter bit 23*/
#define CAN_F10R1_FB24                       (0x1U << 24U)                     /* Filter bit 24*/
#define CAN_F10R1_FB25                       (0x1U << 25U)                     /* Filter bit 25*/
#define CAN_F10R1_FB26                       (0x1U << 26U)                     /* Filter bit 26*/
#define CAN_F10R1_FB27                       (0x1U << 27U)                     /* Filter bit 27*/
#define CAN_F10R1_FB28                       (0x1U << 28U)                     /* Filter bit 28*/
#define CAN_F10R1_FB29                       (0x1U << 29U)                     /* Filter bit 29*/
#define CAN_F10R1_FB30                       (0x1U << 30U)                     /* Filter bit 30*/
#define CAN_F10R1_FB31                       (0x1U << 31U)                     /* Filter bit 31*/

#define CAN_F11R1_FB0                        (0x1U << 0U)                      /* Filter bit 0*/
#define CAN_F11R1_FB1                        (0x1U << 1U)                      /* Filter bit 1*/
#define CAN_F11R1_FB2                        (0x1U << 2U)                      /* Filter bit 2*/
#define CAN_F11R1_FB3                        (0x1U << 3U)                      /* Filter bit 3*/
#define CAN_F11R1_FB4                        (0x1U << 4U)                      /* Filter bit 4*/
#define CAN_F11R1_FB5                        (0x1U << 5U)                      /* Filter bit 5*/
#define CAN_F11R1_FB6                        (0x1U << 6U)                      /* Filter bit 6*/
#define CAN_F11R1_FB7                        (0x1U << 7U)                      /* Filter bit 7*/
#define CAN_F11R1_FB8                        (0x1U << 8U)                      /* Filter bit 8*/
#define CAN_F11R1_FB9                        (0x1U << 9U)                      /* Filter bit 9*/
#define CAN_F11R1_FB10                       (0x1U << 10U)                     /* Filter bit 10*/
#define CAN_F11R1_FB11                       (0x1U << 11U)                     /* Filter bit 11*/
#define CAN_F11R1_FB12                       (0x1U << 12U)                     /* Filter bit 12*/
#define CAN_F11R1_FB13                       (0x1U << 13U)                     /* Filter bit 13*/
#define CAN_F11R1_FB14                       (0x1U << 14U)                     /* Filter bit 14*/
#define CAN_F11R1_FB15                       (0x1U << 15U)                     /* Filter bit 15*/
#define CAN_F11R1_FB16                       (0x1U << 16U)                     /* Filter bit 16*/
#define CAN_F11R1_FB17                       (0x1U << 17U)                     /* Filter bit 17*/
#define CAN_F11R1_FB18                       (0x1U << 18U)                     /* Filter bit 18*/
#define CAN_F11R1_FB19                       (0x1U << 19U)                     /* Filter bit 19*/
#define CAN_F11R1_FB20                       (0x1U << 20U)                     /* Filter bit 20*/
#define CAN_F11R1_FB21                       (0x1U << 21U)                     /* Filter bit 21*/
#define CAN_F11R1_FB22                       (0x1U << 22U)                     /* Filter bit 22*/
#define CAN_F11R1_FB23                       (0x1U << 23U)                     /* Filter bit 23*/
#define CAN_F11R1_FB24                       (0x1U << 24U)                     /* Filter bit 24*/
#define CAN_F11R1_FB25                       (0x1U << 25U)                     /* Filter bit 25*/
#define CAN_F11R1_FB26                       (0x1U << 26U)                     /* Filter bit 26*/
#define CAN_F11R1_FB27                       (0x1U << 27U)                     /* Filter bit 27*/
#define CAN_F11R1_FB28                       (0x1U << 28U)                     /* Filter bit 28*/
#define CAN_F11R1_FB29                       (0x1U << 29U)                     /* Filter bit 29*/
#define CAN_F11R1_FB30                       (0x1U << 30U)                     /* Filter bit 30*/
#define CAN_F11R1_FB31                       (0x1U << 31U)                     /* Filter bit 31*/

#define CAN_F12R1_FB0                        (0x1U << 0U)                      /* Filter bit 0*/
#define CAN_F12R1_FB1                        (0x1U << 1U)                      /* Filter bit 1*/
#define CAN_F12R1_FB2                        (0x1U << 2U)                      /* Filter bit 2*/
#define CAN_F12R1_FB3                        (0x1U << 3U)                      /* Filter bit 3*/
#define CAN_F12R1_FB4                        (0x1U << 4U)                      /* Filter bit 4*/
#define CAN_F12R1_FB5                        (0x1U << 5U)                      /* Filter bit 5*/
#define CAN_F12R1_FB6                        (0x1U << 6U)                      /* Filter bit 6*/
#define CAN_F12R1_FB7                        (0x1U << 7U)                      /* Filter bit 7*/
#define CAN_F12R1_FB8                        (0x1U << 8U)                      /* Filter bit 8*/
#define CAN_F12R1_FB9                        (0x1U << 9U)                      /* Filter bit 9*/
#define CAN_F12R1_FB10                       (0x1U << 10U)                     /* Filter bit 10*/
#define CAN_F12R1_FB11                       (0x1U << 11U)                     /* Filter bit 11*/
#define CAN_F12R1_FB12                       (0x1U << 12U)                     /* Filter bit 12*/
#define CAN_F12R1_FB13                       (0x1U << 13U)                     /* Filter bit 13*/
#define CAN_F12R1_FB14                       (0x1U << 14U)                     /* Filter bit 14*/
#define CAN_F12R1_FB15                       (0x1U << 15U)                     /* Filter bit 15*/
#define CAN_F12R1_FB16                       (0x1U << 16U)                     /* Filter bit 16*/
#define CAN_F12R1_FB17                       (0x1U << 17U)                     /* Filter bit 17*/
#define CAN_F12R1_FB18                       (0x1U << 18U)                     /* Filter bit 18*/
#define CAN_F12R1_FB19                       (0x1U << 19U)                     /* Filter bit 19*/
#define CAN_F12R1_FB20                       (0x1U << 20U)                     /* Filter bit 20*/
#define CAN_F12R1_FB21                       (0x1U << 21U)                     /* Filter bit 21*/
#define CAN_F12R1_FB22                       (0x1U << 22U)                     /* Filter bit 22*/
#define CAN_F12R1_FB23                       (0x1U << 23U)                     /* Filter bit 23*/
#define CAN_F12R1_FB24                       (0x1U << 24U)                     /* Filter bit 24*/
#define CAN_F12R1_FB25                       (0x1U << 25U)                     /* Filter bit 25*/
#define CAN_F12R1_FB26                       (0x1U << 26U)                     /* Filter bit 26*/
#define CAN_F12R1_FB27                       (0x1U << 27U)                     /* Filter bit 27*/
#define CAN_F12R1_FB28                       (0x1U << 28U)                     /* Filter bit 28*/
#define CAN_F12R1_FB29                       (0x1U << 29U)                     /* Filter bit 29*/
#define CAN_F12R1_FB30                       (0x1U << 30U)                     /* Filter bit 30*/
#define CAN_F12R1_FB31                       (0x1U << 31U)                     /* Filter bit 31*/

#define CAN_F13R1_FB0                        (0x1U << 0U)                      /* Filter bit 0*/
#define CAN_F13R1_FB1                        (0x1U << 1U)                      /* Filter bit 1*/
#define CAN_F13R1_FB2                        (0x1U << 2U)                      /* Filter bit 2*/
#define CAN_F13R1_FB3                        (0x1U << 3U)                      /* Filter bit 3*/
#define CAN_F13R1_FB4                        (0x1U << 4U)                      /* Filter bit 4*/
#define CAN_F13R1_FB5                        (0x1U << 5U)                      /* Filter bit 5*/
#define CAN_F13R1_FB6                        (0x1U << 6U)                      /* Filter bit 6*/
#define CAN_F13R1_FB7                        (0x1U << 7U)                      /* Filter bit 7*/
#define CAN_F13R1_FB8                        (0x1U << 8U)                      /* Filter bit 8*/
#define CAN_F13R1_FB9                        (0x1U << 9U)                      /* Filter bit 9*/
#define CAN_F13R1_FB10                       (0x1U << 10U)                     /* Filter bit 10*/
#define CAN_F13R1_FB11                       (0x1U << 11U)                     /* Filter bit 11*/
#define CAN_F13R1_FB12                       (0x1U << 12U)                     /* Filter bit 12*/
#define CAN_F13R1_FB13                       (0x1U << 13U)                     /* Filter bit 13*/
#define CAN_F13R1_FB14                       (0x1U << 14U)                     /* Filter bit 14*/
#define CAN_F13R1_FB15                       (0x1U << 15U)                     /* Filter bit 15*/
#define CAN_F13R1_FB16                       (0x1U << 16U)                     /* Filter bit 16*/
#define CAN_F13R1_FB17                       (0x1U << 17U)                     /* Filter bit 17*/
#define CAN_F13R1_FB18                       (0x1U << 18U)                     /* Filter bit 18*/
#define CAN_F13R1_FB19                       (0x1U << 19U)                     /* Filter bit 19*/
#define CAN_F13R1_FB20                       (0x1U << 20U)                     /* Filter bit 20*/
#define CAN_F13R1_FB21                       (0x1U << 21U)                     /* Filter bit 21*/
#define CAN_F13R1_FB22                       (0x1U << 22U)                     /* Filter bit 22*/
#define CAN_F13R1_FB23                       (0x1U << 23U)                     /* Filter bit 23*/
#define CAN_F13R1_FB24                       (0x1U << 24U)                     /* Filter bit 24*/
#define CAN_F13R1_FB25                       (0x1U << 25U)                     /* Filter bit 25*/
#define CAN_F13R1_FB26                       (0x1U << 26U)                     /* Filter bit 26*/
#define CAN_F13R1_FB27                       (0x1U << 27U)                     /* Filter bit 27*/
#define CAN_F13R1_FB28                       (0x1U << 28U)                     /* Filter bit 28*/
#define CAN_F13R1_FB29                       (0x1U << 29U)                     /* Filter bit 29*/
#define CAN_F13R1_FB30                       (0x1U << 30U)                     /* Filter bit 30*/
#define CAN_F13R1_FB31                       (0x1U << 31U)                     /* Filter bit 31*/

#define CAN_F0R2_FB0                         (0x1U << 0U)                      /* Filter bit 0*/
#define CAN_F0R2_FB1                         (0x1U << 1U)                      /* Filter bit 1*/
#define CAN_F0R2_FB2                         (0x1U << 2U)                      /* Filter bit 2*/
#define CAN_F0R2_FB3                         (0x1U << 3U)                      /* Filter bit 3*/
#define CAN_F0R2_FB4                         (0x1U << 4U)                      /* Filter bit 4*/
#define CAN_F0R2_FB5                         (0x1U << 5U)                      /* Filter bit 5*/
#define CAN_F0R2_FB6                         (0x1U << 6U)                      /* Filter bit 6*/
#define CAN_F0R2_FB7                         (0x1U << 7U)                      /* Filter bit 7*/
#define CAN_F0R2_FB8                         (0x1U << 8U)                      /* Filter bit 8*/
#define CAN_F0R2_FB9                         (0x1U << 9U)                      /* Filter bit 9*/
#define CAN_F0R2_FB10                        (0x1U << 10U)                     /* Filter bit 10*/
#define CAN_F0R2_FB11                        (0x1U << 11U)                     /* Filter bit 11*/
#define CAN_F0R2_FB12                        (0x1U << 12U)                     /* Filter bit 12*/
#define CAN_F0R2_FB13                        (0x1U << 13U)                     /* Filter bit 13*/
#define CAN_F0R2_FB14                        (0x1U << 14U)                     /* Filter bit 14*/
#define CAN_F0R2_FB15                        (0x1U << 15U)                     /* Filter bit 15*/
#define CAN_F0R2_FB16                        (0x1U << 16U)                     /* Filter bit 16*/
#define CAN_F0R2_FB17                        (0x1U << 17U)                     /* Filter bit 17*/
#define CAN_F0R2_FB18                        (0x1U << 18U)                     /* Filter bit 18*/
#define CAN_F0R2_FB19                        (0x1U << 19U)                     /* Filter bit 19*/
#define CAN_F0R2_FB20                        (0x1U << 20U)                     /* Filter bit 20*/
#define CAN_F0R2_FB21                        (0x1U << 21U)                     /* Filter bit 21*/
#define CAN_F0R2_FB22                        (0x1U << 22U)                     /* Filter bit 22*/
#define CAN_F0R2_FB23                        (0x1U << 23U)                     /* Filter bit 23*/
#define CAN_F0R2_FB24                        (0x1U << 24U)                     /* Filter bit 24*/
#define CAN_F0R2_FB25                        (0x1U << 25U)                     /* Filter bit 25*/
#define CAN_F0R2_FB26                        (0x1U << 26U)                     /* Filter bit 26*/
#define CAN_F0R2_FB27                        (0x1U << 27U)                     /* Filter bit 27*/
#define CAN_F0R2_FB28                        (0x1U << 28U)                     /* Filter bit 28*/
#define CAN_F0R2_FB29                        (0x1U << 29U)                     /* Filter bit 29*/
#define CAN_F0R2_FB30                        (0x1U << 30U)                     /* Filter bit 30*/
#define CAN_F0R2_FB31                        (0x1U << 31U)                     /* Filter bit 31*/

#define CAN_F1R2_FB0                         (0x1U << 0U)                      /* Filter bit 0*/
#define CAN_F1R2_FB1                         (0x1U << 1U)                      /* Filter bit 1*/
#define CAN_F1R2_FB2                         (0x1U << 2U)                      /* Filter bit 2*/
#define CAN_F1R2_FB3                         (0x1U << 3U)                      /* Filter bit 3*/
#define CAN_F1R2_FB4                         (0x1U << 4U)                      /* Filter bit 4*/
#define CAN_F1R2_FB5                         (0x1U << 5U)                      /* Filter bit 5*/
#define CAN_F1R2_FB6                         (0x1U << 6U)                      /* Filter bit 6*/
#define CAN_F1R2_FB7                         (0x1U << 7U)                      /* Filter bit 7*/
#define CAN_F1R2_FB8                         (0x1U << 8U)                      /* Filter bit 8*/
#define CAN_F1R2_FB9                         (0x1U << 9U)                      /* Filter bit 9*/
#define CAN_F1R2_FB10                        (0x1U << 10U)                     /* Filter bit 10*/
#define CAN_F1R2_FB11                        (0x1U << 11U)                     /* Filter bit 11*/
#define CAN_F1R2_FB12                        (0x1U << 12U)                     /* Filter bit 12*/
#define CAN_F1R2_FB13                        (0x1U << 13U)                     /* Filter bit 13*/
#define CAN_F1R2_FB14                        (0x1U << 14U)                     /* Filter bit 14*/
#define CAN_F1R2_FB15                        (0x1U << 15U)                     /* Filter bit 15*/
#define CAN_F1R2_FB16                        (0x1U << 16U)                     /* Filter bit 16*/
#define CAN_F1R2_FB17                        (0x1U << 17U)                     /* Filter bit 17*/
#define CAN_F1R2_FB18                        (0x1U << 18U)                     /* Filter bit 18*/
#define CAN_F1R2_FB19                        (0x1U << 19U)                     /* Filter bit 19*/
#define CAN_F1R2_FB20                        (0x1U << 20U)                     /* Filter bit 20*/
#define CAN_F1R2_FB21                        (0x1U << 21U)                     /* Filter bit 21*/
#define CAN_F1R2_FB22                        (0x1U << 22U)                     /* Filter bit 22*/
#define CAN_F1R2_FB23                        (0x1U << 23U)                     /* Filter bit 23*/
#define CAN_F1R2_FB24                        (0x1U << 24U)                     /* Filter bit 24*/
#define CAN_F1R2_FB25                        (0x1U << 25U)                     /* Filter bit 25*/
#define CAN_F1R2_FB26                        (0x1U << 26U)                     /* Filter bit 26*/
#define CAN_F1R2_FB27                        (0x1U << 27U)                     /* Filter bit 27*/
#define CAN_F1R2_FB28                        (0x1U << 28U)                     /* Filter bit 28*/
#define CAN_F1R2_FB29                        (0x1U << 29U)                     /* Filter bit 29*/
#define CAN_F1R2_FB30                        (0x1U << 30U)                     /* Filter bit 30*/
#define CAN_F1R2_FB31                        (0x1U << 31U)                     /* Filter bit 31*/

#define CAN_F2R2_FB0                         (0x1U << 0U)                      /* Filter bit 0*/
#define CAN_F2R2_FB1                         (0x1U << 1U)                      /* Filter bit 1*/
#define CAN_F2R2_FB2                         (0x1U << 2U)                      /* Filter bit 2*/
#define CAN_F2R2_FB3                         (0x1U << 3U)                      /* Filter bit 3*/
#define CAN_F2R2_FB4                         (0x1U << 4U)                      /* Filter bit 4*/
#define CAN_F2R2_FB5                         (0x1U << 5U)                      /* Filter bit 5*/
#define CAN_F2R2_FB6                         (0x1U << 6U)                      /* Filter bit 6*/
#define CAN_F2R2_FB7                         (0x1U << 7U)                      /* Filter bit 7*/
#define CAN_F2R2_FB8                         (0x1U << 8U)                      /* Filter bit 8*/
#define CAN_F2R2_FB9                         (0x1U << 9U)                      /* Filter bit 9*/
#define CAN_F2R2_FB10                        (0x1U << 10U)                     /* Filter bit 10*/
#define CAN_F2R2_FB11                        (0x1U << 11U)                     /* Filter bit 11*/
#define CAN_F2R2_FB12                        (0x1U << 12U)                     /* Filter bit 12*/
#define CAN_F2R2_FB13                        (0x1U << 13U)                     /* Filter bit 13*/
#define CAN_F2R2_FB14                        (0x1U << 14U)                     /* Filter bit 14*/
#define CAN_F2R2_FB15                        (0x1U << 15U)                     /* Filter bit 15*/
#define CAN_F2R2_FB16                        (0x1U << 16U)                     /* Filter bit 16*/
#define CAN_F2R2_FB17                        (0x1U << 17U)                     /* Filter bit 17*/
#define CAN_F2R2_FB18                        (0x1U << 18U)                     /* Filter bit 18*/
#define CAN_F2R2_FB19                        (0x1U << 19U)                     /* Filter bit 19*/
#define CAN_F2R2_FB20                        (0x1U << 20U)                     /* Filter bit 20*/
#define CAN_F2R2_FB21                        (0x1U << 21U)                     /* Filter bit 21*/
#define CAN_F2R2_FB22                        (0x1U << 22U)                     /* Filter bit 22*/
#define CAN_F2R2_FB23                        (0x1U << 23U)                     /* Filter bit 23*/
#define CAN_F2R2_FB24                        (0x1U << 24U)                     /* Filter bit 24*/
#define CAN_F2R2_FB25                        (0x1U << 25U)                     /* Filter bit 25*/
#define CAN_F2R2_FB26                        (0x1U << 26U)                     /* Filter bit 26*/
#define CAN_F2R2_FB27                        (0x1U << 27U)                     /* Filter bit 27*/
#define CAN_F2R2_FB28                        (0x1U << 28U)                     /* Filter bit 28*/
#define CAN_F2R2_FB29                        (0x1U << 29U)                     /* Filter bit 29*/
#define CAN_F2R2_FB30                        (0x1U << 30U)                     /* Filter bit 30*/
#define CAN_F2R2_FB31                        (0x1U << 31U)                     /* Filter bit 31*/

#define CAN_F3R2_FB0                         (0x1U << 0U)                      /* Filter bit 0*/
#define CAN_F3R2_FB1                         (0x1U << 1U)                      /* Filter bit 1*/
#define CAN_F3R2_FB2                         (0x1U << 2U)                      /* Filter bit 2*/
#define CAN_F3R2_FB3                         (0x1U << 3U)                      /* Filter bit 3*/
#define CAN_F3R2_FB4                         (0x1U << 4U)                      /* Filter bit 4*/
#define CAN_F3R2_FB5                         (0x1U << 5U)                      /* Filter bit 5*/
#define CAN_F3R2_FB6                         (0x1U << 6U)                      /* Filter bit 6*/
#define CAN_F3R2_FB7                         (0x1U << 7U)                      /* Filter bit 7*/
#define CAN_F3R2_FB8                         (0x1U << 8U)                      /* Filter bit 8*/
#define CAN_F3R2_FB9                         (0x1U << 9U)                      /* Filter bit 9*/
#define CAN_F3R2_FB10                        (0x1U << 10U)                     /* Filter bit 10*/
#define CAN_F3R2_FB11                        (0x1U << 11U)                     /* Filter bit 11*/
#define CAN_F3R2_FB12                        (0x1U << 12U)                     /* Filter bit 12*/
#define CAN_F3R2_FB13                        (0x1U << 13U)                     /* Filter bit 13*/
#define CAN_F3R2_FB14                        (0x1U << 14U)                     /* Filter bit 14*/
#define CAN_F3R2_FB15                        (0x1U << 15U)                     /* Filter bit 15*/
#define CAN_F3R2_FB16                        (0x1U << 16U)                     /* Filter bit 16*/
#define CAN_F3R2_FB17                        (0x1U << 17U)                     /* Filter bit 17*/
#define CAN_F3R2_FB18                        (0x1U << 18U)                     /* Filter bit 18*/
#define CAN_F3R2_FB19                        (0x1U << 19U)                     /* Filter bit 19*/
#define CAN_F3R2_FB20                        (0x1U << 20U)                     /* Filter bit 20*/
#define CAN_F3R2_FB21                        (0x1U << 21U)                     /* Filter bit 21*/
#define CAN_F3R2_FB22                        (0x1U << 22U)                     /* Filter bit 22*/
#define CAN_F3R2_FB23                        (0x1U << 23U)                     /* Filter bit 23*/
#define CAN_F3R2_FB24                        (0x1U << 24U)                     /* Filter bit 24*/
#define CAN_F3R2_FB25                        (0x1U << 25U)                     /* Filter bit 25*/
#define CAN_F3R2_FB26                        (0x1U << 26U)                     /* Filter bit 26*/
#define CAN_F3R2_FB27                        (0x1U << 27U)                     /* Filter bit 27*/
#define CAN_F3R2_FB28                        (0x1U << 28U)                     /* Filter bit 28*/
#define CAN_F3R2_FB29                        (0x1U << 29U)                     /* Filter bit 29*/
#define CAN_F3R2_FB30                        (0x1U << 30U)                     /* Filter bit 30*/
#define CAN_F3R2_FB31                        (0x1U << 31U)                     /* Filter bit 31*/

#define CAN_F4R2_FB0                         (0x1U << 0U)                      /* Filter bit 0*/
#define CAN_F4R2_FB1                         (0x1U << 1U)                      /* Filter bit 1*/
#define CAN_F4R2_FB2                         (0x1U << 2U)                      /* Filter bit 2*/
#define CAN_F4R2_FB3                         (0x1U << 3U)                      /* Filter bit 3*/
#define CAN_F4R2_FB4                         (0x1U << 4U)                      /* Filter bit 4*/
#define CAN_F4R2_FB5                         (0x1U << 5U)                      /* Filter bit 5*/
#define CAN_F4R2_FB6                         (0x1U << 6U)                      /* Filter bit 6*/
#define CAN_F4R2_FB7                         (0x1U << 7U)                      /* Filter bit 7*/
#define CAN_F4R2_FB8                         (0x1U << 8U)                      /* Filter bit 8*/
#define CAN_F4R2_FB9                         (0x1U << 9U)                      /* Filter bit 9*/
#define CAN_F4R2_FB10                        (0x1U << 10U)                     /* Filter bit 10*/
#define CAN_F4R2_FB11                        (0x1U << 11U)                     /* Filter bit 11*/
#define CAN_F4R2_FB12                        (0x1U << 12U)                     /* Filter bit 12*/
#define CAN_F4R2_FB13                        (0x1U << 13U)                     /* Filter bit 13*/
#define CAN_F4R2_FB14                        (0x1U << 14U)                     /* Filter bit 14*/
#define CAN_F4R2_FB15                        (0x1U << 15U)                     /* Filter bit 15*/
#define CAN_F4R2_FB16                        (0x1U << 16U)                     /* Filter bit 16*/
#define CAN_F4R2_FB17                        (0x1U << 17U)                     /* Filter bit 17*/
#define CAN_F4R2_FB18                        (0x1U << 18U)                     /* Filter bit 18*/
#define CAN_F4R2_FB19                        (0x1U << 19U)                     /* Filter bit 19*/
#define CAN_F4R2_FB20                        (0x1U << 20U)                     /* Filter bit 20*/
#define CAN_F4R2_FB21                        (0x1U << 21U)                     /* Filter bit 21*/
#define CAN_F4R2_FB22                        (0x1U << 22U)                     /* Filter bit 22*/
#define CAN_F4R2_FB23                        (0x1U << 23U)                     /* Filter bit 23*/
#define CAN_F4R2_FB24                        (0x1U << 24U)                     /* Filter bit 24*/
#define CAN_F4R2_FB25                        (0x1U << 25U)                     /* Filter bit 25*/
#define CAN_F4R2_FB26                        (0x1U << 26U)                     /* Filter bit 26*/
#define CAN_F4R2_FB27                        (0x1U << 27U)                     /* Filter bit 27*/
#define CAN_F4R2_FB28                        (0x1U << 28U)                     /* Filter bit 28*/
#define CAN_F4R2_FB29                        (0x1U << 29U)                     /* Filter bit 29*/
#define CAN_F4R2_FB30                        (0x1U << 30U)                     /* Filter bit 30*/
#define CAN_F4R2_FB31                        (0x1U << 31U)                     /* Filter bit 31*/

#define CAN_F5R2_FB0                         (0x1U << 0U)                      /* Filter bit 0*/
#define CAN_F5R2_FB1                         (0x1U << 1U)                      /* Filter bit 1*/
#define CAN_F5R2_FB2                         (0x1U << 2U)                      /* Filter bit 2*/
#define CAN_F5R2_FB3                         (0x1U << 3U)                      /* Filter bit 3*/
#define CAN_F5R2_FB4                         (0x1U << 4U)                      /* Filter bit 4*/
#define CAN_F5R2_FB5                         (0x1U << 5U)                      /* Filter bit 5*/
#define CAN_F5R2_FB6                         (0x1U << 6U)                      /* Filter bit 6*/
#define CAN_F5R2_FB7                         (0x1U << 7U)                      /* Filter bit 7*/
#define CAN_F5R2_FB8                         (0x1U << 8U)                      /* Filter bit 8*/
#define CAN_F5R2_FB9                         (0x1U << 9U)                      /* Filter bit 9*/
#define CAN_F5R2_FB10                        (0x1U << 10U)                     /* Filter bit 10*/
#define CAN_F5R2_FB11                        (0x1U << 11U)                     /* Filter bit 11*/
#define CAN_F5R2_FB12                        (0x1U << 12U)                     /* Filter bit 12*/
#define CAN_F5R2_FB13                        (0x1U << 13U)                     /* Filter bit 13*/
#define CAN_F5R2_FB14                        (0x1U << 14U)                     /* Filter bit 14*/
#define CAN_F5R2_FB15                        (0x1U << 15U)                     /* Filter bit 15*/
#define CAN_F5R2_FB16                        (0x1U << 16U)                     /* Filter bit 16*/
#define CAN_F5R2_FB17                        (0x1U << 17U)                     /* Filter bit 17*/
#define CAN_F5R2_FB18                        (0x1U << 18U)                     /* Filter bit 18*/
#define CAN_F5R2_FB19                        (0x1U << 19U)                     /* Filter bit 19*/
#define CAN_F5R2_FB20                        (0x1U << 20U)                     /* Filter bit 20*/
#define CAN_F5R2_FB21                        (0x1U << 21U)                     /* Filter bit 21*/
#define CAN_F5R2_FB22                        (0x1U << 22U)                     /* Filter bit 22*/
#define CAN_F5R2_FB23                        (0x1U << 23U)                     /* Filter bit 23*/
#define CAN_F5R2_FB24                        (0x1U << 24U)                     /* Filter bit 24*/
#define CAN_F5R2_FB25                        (0x1U << 25U)                     /* Filter bit 25*/
#define CAN_F5R2_FB26                        (0x1U << 26U)                     /* Filter bit 26*/
#define CAN_F5R2_FB27                        (0x1U << 27U)                     /* Filter bit 27*/
#define CAN_F5R2_FB28                        (0x1U << 28U)                     /* Filter bit 28*/
#define CAN_F5R2_FB29                        (0x1U << 29U)                     /* Filter bit 29*/
#define CAN_F5R2_FB30                        (0x1U << 30U)                     /* Filter bit 30*/
#define CAN_F5R2_FB31                        (0x1U << 31U)                     /* Filter bit 31*/

#define CAN_F6R2_FB0                         (0x1U << 0U)                      /* Filter bit 0*/
#define CAN_F6R2_FB1                         (0x1U << 1U)                      /* Filter bit 1*/
#define CAN_F6R2_FB2                         (0x1U << 2U)                      /* Filter bit 2*/
#define CAN_F6R2_FB3                         (0x1U << 3U)                      /* Filter bit 3*/
#define CAN_F6R2_FB4                         (0x1U << 4U)                      /* Filter bit 4*/
#define CAN_F6R2_FB5                         (0x1U << 5U)                      /* Filter bit 5*/
#define CAN_F6R2_FB6                         (0x1U << 6U)                      /* Filter bit 6*/
#define CAN_F6R2_FB7                         (0x1U << 7U)                      /* Filter bit 7*/
#define CAN_F6R2_FB8                         (0x1U << 8U)                      /* Filter bit 8*/
#define CAN_F6R2_FB9                         (0x1U << 9U)                      /* Filter bit 9*/
#define CAN_F6R2_FB10                        (0x1U << 10U)                     /* Filter bit 10*/
#define CAN_F6R2_FB11                        (0x1U << 11U)                     /* Filter bit 11*/
#define CAN_F6R2_FB12                        (0x1U << 12U)                     /* Filter bit 12*/
#define CAN_F6R2_FB13                        (0x1U << 13U)                     /* Filter bit 13*/
#define CAN_F6R2_FB14                        (0x1U << 14U)                     /* Filter bit 14*/
#define CAN_F6R2_FB15                        (0x1U << 15U)                     /* Filter bit 15*/
#define CAN_F6R2_FB16                        (0x1U << 16U)                     /* Filter bit 16*/
#define CAN_F6R2_FB17                        (0x1U << 17U)                     /* Filter bit 17*/
#define CAN_F6R2_FB18                        (0x1U << 18U)                     /* Filter bit 18*/
#define CAN_F6R2_FB19                        (0x1U << 19U)                     /* Filter bit 19*/
#define CAN_F6R2_FB20                        (0x1U << 20U)                     /* Filter bit 20*/
#define CAN_F6R2_FB21                        (0x1U << 21U)                     /* Filter bit 21*/
#define CAN_F6R2_FB22                        (0x1U << 22U)                     /* Filter bit 22*/
#define CAN_F6R2_FB23                        (0x1U << 23U)                     /* Filter bit 23*/
#define CAN_F6R2_FB24                        (0x1U << 24U)                     /* Filter bit 24*/
#define CAN_F6R2_FB25                        (0x1U << 25U)                     /* Filter bit 25*/
#define CAN_F6R2_FB26                        (0x1U << 26U)                     /* Filter bit 26*/
#define CAN_F6R2_FB27                        (0x1U << 27U)                     /* Filter bit 27*/
#define CAN_F6R2_FB28                        (0x1U << 28U)                     /* Filter bit 28*/
#define CAN_F6R2_FB29                        (0x1U << 29U)                     /* Filter bit 29*/
#define CAN_F6R2_FB30                        (0x1U << 30U)                     /* Filter bit 30*/
#define CAN_F6R2_FB31                        (0x1U << 31U)                     /* Filter bit 31*/

#define CAN_F7R2_FB0                         (0x1U << 0U)                      /* Filter bit 0*/
#define CAN_F7R2_FB1                         (0x1U << 1U)                      /* Filter bit 1*/
#define CAN_F7R2_FB2                         (0x1U << 2U)                      /* Filter bit 2*/
#define CAN_F7R2_FB3                         (0x1U << 3U)                      /* Filter bit 3*/
#define CAN_F7R2_FB4                         (0x1U << 4U)                      /* Filter bit 4*/
#define CAN_F7R2_FB5                         (0x1U << 5U)                      /* Filter bit 5*/
#define CAN_F7R2_FB6                         (0x1U << 6U)                      /* Filter bit 6*/
#define CAN_F7R2_FB7                         (0x1U << 7U)                      /* Filter bit 7*/
#define CAN_F7R2_FB8                         (0x1U << 8U)                      /* Filter bit 8*/
#define CAN_F7R2_FB9                         (0x1U << 9U)                      /* Filter bit 9*/
#define CAN_F7R2_FB10                        (0x1U << 10U)                     /* Filter bit 10*/
#define CAN_F7R2_FB11                        (0x1U << 11U)                     /* Filter bit 11*/
#define CAN_F7R2_FB12                        (0x1U << 12U)                     /* Filter bit 12*/
#define CAN_F7R2_FB13                        (0x1U << 13U)                     /* Filter bit 13*/
#define CAN_F7R2_FB14                        (0x1U << 14U)                     /* Filter bit 14*/
#define CAN_F7R2_FB15                        (0x1U << 15U)                     /* Filter bit 15*/
#define CAN_F7R2_FB16                        (0x1U << 16U)                     /* Filter bit 16*/
#define CAN_F7R2_FB17                        (0x1U << 17U)                     /* Filter bit 17*/
#define CAN_F7R2_FB18                        (0x1U << 18U)                     /* Filter bit 18*/
#define CAN_F7R2_FB19                        (0x1U << 19U)                     /* Filter bit 19*/
#define CAN_F7R2_FB20                        (0x1U << 20U)                     /* Filter bit 20*/
#define CAN_F7R2_FB21                        (0x1U << 21U)                     /* Filter bit 21*/
#define CAN_F7R2_FB22                        (0x1U << 22U)                     /* Filter bit 22*/
#define CAN_F7R2_FB23                        (0x1U << 23U)                     /* Filter bit 23*/
#define CAN_F7R2_FB24                        (0x1U << 24U)                     /* Filter bit 24*/
#define CAN_F7R2_FB25                        (0x1U << 25U)                     /* Filter bit 25*/
#define CAN_F7R2_FB26                        (0x1U << 26U)                     /* Filter bit 26*/
#define CAN_F7R2_FB27                        (0x1U << 27U)                     /* Filter bit 27*/
#define CAN_F7R2_FB28                        (0x1U << 28U)                     /* Filter bit 28*/
#define CAN_F7R2_FB29                        (0x1U << 29U)                     /* Filter bit 29*/
#define CAN_F7R2_FB30                        (0x1U << 30U)                     /* Filter bit 30*/
#define CAN_F7R2_FB31                        (0x1U << 31U)                     /* Filter bit 31*/

#define CAN_F8R2_FB0                         (0x1U << 0U)                      /* Filter bit 0*/
#define CAN_F8R2_FB1                         (0x1U << 1U)                      /* Filter bit 1*/
#define CAN_F8R2_FB2                         (0x1U << 2U)                      /* Filter bit 2*/
#define CAN_F8R2_FB3                         (0x1U << 3U)                      /* Filter bit 3*/
#define CAN_F8R2_FB4                         (0x1U << 4U)                      /* Filter bit 4*/
#define CAN_F8R2_FB5                         (0x1U << 5U)                      /* Filter bit 5*/
#define CAN_F8R2_FB6                         (0x1U << 6U)                      /* Filter bit 6*/
#define CAN_F8R2_FB7                         (0x1U << 7U)                      /* Filter bit 7*/
#define CAN_F8R2_FB8                         (0x1U << 8U)                      /* Filter bit 8*/
#define CAN_F8R2_FB9                         (0x1U << 9U)                      /* Filter bit 9*/
#define CAN_F8R2_FB10                        (0x1U << 10U)                     /* Filter bit 10*/
#define CAN_F8R2_FB11                        (0x1U << 11U)                     /* Filter bit 11*/
#define CAN_F8R2_FB12                        (0x1U << 12U)                     /* Filter bit 12*/
#define CAN_F8R2_FB13                        (0x1U << 13U)                     /* Filter bit 13*/
#define CAN_F8R2_FB14                        (0x1U << 14U)                     /* Filter bit 14*/
#define CAN_F8R2_FB15                        (0x1U << 15U)                     /* Filter bit 15*/
#define CAN_F8R2_FB16                        (0x1U << 16U)                     /* Filter bit 16*/
#define CAN_F8R2_FB17                        (0x1U << 17U)                     /* Filter bit 17*/
#define CAN_F8R2_FB18                        (0x1U << 18U)                     /* Filter bit 18*/
#define CAN_F8R2_FB19                        (0x1U << 19U)                     /* Filter bit 19*/
#define CAN_F8R2_FB20                        (0x1U << 20U)                     /* Filter bit 20*/
#define CAN_F8R2_FB21                        (0x1U << 21U)                     /* Filter bit 21*/
#define CAN_F8R2_FB22                        (0x1U << 22U)                     /* Filter bit 22*/
#define CAN_F8R2_FB23                        (0x1U << 23U)                     /* Filter bit 23*/
#define CAN_F8R2_FB24                        (0x1U << 24U)                     /* Filter bit 24*/
#define CAN_F8R2_FB25                        (0x1U << 25U)                     /* Filter bit 25*/
#define CAN_F8R2_FB26                        (0x1U << 26U)                     /* Filter bit 26*/
#define CAN_F8R2_FB27                        (0x1U << 27U)                     /* Filter bit 27*/
#define CAN_F8R2_FB28                        (0x1U << 28U)                     /* Filter bit 28*/
#define CAN_F8R2_FB29                        (0x1U << 29U)                     /* Filter bit 29*/
#define CAN_F8R2_FB30                        (0x1U << 30U)                     /* Filter bit 30*/
#define CAN_F8R2_FB31                        (0x1U << 31U)                     /* Filter bit 31*/

#define CAN_F9R2_FB0                         (0x1U << 0U)                      /* Filter bit 0*/
#define CAN_F9R2_FB1                         (0x1U << 1U)                      /* Filter bit 1*/
#define CAN_F9R2_FB2                         (0x1U << 2U)                      /* Filter bit 2*/
#define CAN_F9R2_FB3                         (0x1U << 3U)                      /* Filter bit 3*/
#define CAN_F9R2_FB4                         (0x1U << 4U)                      /* Filter bit 4*/
#define CAN_F9R2_FB5                         (0x1U << 5U)                      /* Filter bit 5*/
#define CAN_F9R2_FB6                         (0x1U << 6U)                      /* Filter bit 6*/
#define CAN_F9R2_FB7                         (0x1U << 7U)                      /* Filter bit 7*/
#define CAN_F9R2_FB8                         (0x1U << 8U)                      /* Filter bit 8*/
#define CAN_F9R2_FB9                         (0x1U << 9U)                      /* Filter bit 9*/
#define CAN_F9R2_FB10                        (0x1U << 10U)                     /* Filter bit 10*/
#define CAN_F9R2_FB11                        (0x1U << 11U)                     /* Filter bit 11*/
#define CAN_F9R2_FB12                        (0x1U << 12U)                     /* Filter bit 12*/
#define CAN_F9R2_FB13                        (0x1U << 13U)                     /* Filter bit 13*/
#define CAN_F9R2_FB14                        (0x1U << 14U)                     /* Filter bit 14*/
#define CAN_F9R2_FB15                        (0x1U << 15U)                     /* Filter bit 15*/
#define CAN_F9R2_FB16                        (0x1U << 16U)                     /* Filter bit 16*/
#define CAN_F9R2_FB17                        (0x1U << 17U)                     /* Filter bit 17*/
#define CAN_F9R2_FB18                        (0x1U << 18U)                     /* Filter bit 18*/
#define CAN_F9R2_FB19                        (0x1U << 19U)                     /* Filter bit 19*/
#define CAN_F9R2_FB20                        (0x1U << 20U)                     /* Filter bit 20*/
#define CAN_F9R2_FB21                        (0x1U << 21U)                     /* Filter bit 21*/
#define CAN_F9R2_FB22                        (0x1U << 22U)                     /* Filter bit 22*/
#define CAN_F9R2_FB23                        (0x1U << 23U)                     /* Filter bit 23*/
#define CAN_F9R2_FB24                        (0x1U << 24U)                     /* Filter bit 24*/
#define CAN_F9R2_FB25                        (0x1U << 25U)                     /* Filter bit 25*/
#define CAN_F9R2_FB26                        (0x1U << 26U)                     /* Filter bit 26*/
#define CAN_F9R2_FB27                        (0x1U << 27U)                     /* Filter bit 27*/
#define CAN_F9R2_FB28                        (0x1U << 28U)                     /* Filter bit 28*/
#define CAN_F9R2_FB29                        (0x1U << 29U)                     /* Filter bit 29*/
#define CAN_F9R2_FB30                        (0x1U << 30U)                     /* Filter bit 30*/
#define CAN_F9R2_FB31                        (0x1U << 31U)                     /* Filter bit 31*/

#define CAN_F10R2_FB0                        (0x1U << 0U)                      /* Filter bit 0 */
#define CAN_F10R2_FB1                        (0x1U << 1U)                      /* Filter bit 1 */
#define CAN_F10R2_FB2                        (0x1U << 2U)                      /* Filter bit 2 */
#define CAN_F10R2_FB3                        (0x1U << 3U)                      /* Filter bit 3 */
#define CAN_F10R2_FB4                        (0x1U << 4U)                      /* Filter bit 4 */
#define CAN_F10R2_FB5                        (0x1U << 5U)                      /* Filter bit 5 */
#define CAN_F10R2_FB6                        (0x1U << 6U)                      /* Filter bit 6 */
#define CAN_F10R2_FB7                        (0x1U << 7U)                      /* Filter bit 7 */
#define CAN_F10R2_FB8                        (0x1U << 8U)                      /* Filter bit 8 */
#define CAN_F10R2_FB9                        (0x1U << 9U)                      /* Filter bit 9 */
#define CAN_F10R2_FB10                       (0x1U << 10U)                     /* Filter bit 10 */
#define CAN_F10R2_FB11                       (0x1U << 11U)                     /* Filter bit 11 */
#define CAN_F10R2_FB12                       (0x1U << 12U)                     /* Filter bit 12 */
#define CAN_F10R2_FB13                       (0x1U << 13U)                     /* Filter bit 13 */
#define CAN_F10R2_FB14                       (0x1U << 14U)                     /* Filter bit 14 */
#define CAN_F10R2_FB15                       (0x1U << 15U)                     /* Filter bit 15 */
#define CAN_F10R2_FB16                       (0x1U << 16U)                     /* Filter bit 16 */
#define CAN_F10R2_FB17                       (0x1U << 17U)                     /* Filter bit 17 */
#define CAN_F10R2_FB18                       (0x1U << 18U)                     /* Filter bit 18 */
#define CAN_F10R2_FB19                       (0x1U << 19U)                     /* Filter bit 19 */
#define CAN_F10R2_FB20                       (0x1U << 20U)                     /* Filter bit 20 */
#define CAN_F10R2_FB21                       (0x1U << 21U)                     /* Filter bit 21 */
#define CAN_F10R2_FB22                       (0x1U << 22U)                     /* Filter bit 22 */
#define CAN_F10R2_FB23                       (0x1U << 23U)                     /* Filter bit 23 */
#define CAN_F10R2_FB24                       (0x1U << 24U)                     /* Filter bit 24 */
#define CAN_F10R2_FB25                       (0x1U << 25U)                     /* Filter bit 25 */
#define CAN_F10R2_FB26                       (0x1U << 26U)                     /* Filter bit 26 */
#define CAN_F10R2_FB27                       (0x1U << 27U)                     /* Filter bit 27 */
#define CAN_F10R2_FB28                       (0x1U << 28U)                     /* Filter bit 28 */
#define CAN_F10R2_FB29                       (0x1U << 29U)                     /* Filter bit 29 */
#define CAN_F10R2_FB30                       (0x1U << 30U)                     /* Filter bit 30 */
#define CAN_F10R2_FB31                       (0x1U << 31U)                     /* Filter bit 31 */

#define CAN_F11R2_FB0                        (0x1U << 0U)                      /* Filter bit 0*/
#define CAN_F11R2_FB1                        (0x1U << 1U)                      /* Filter bit 1*/
#define CAN_F11R2_FB2                        (0x1U << 2U)                      /* Filter bit 2*/
#define CAN_F11R2_FB3                        (0x1U << 3U)                      /* Filter bit 3*/
#define CAN_F11R2_FB4                        (0x1U << 4U)                      /* Filter bit 4*/
#define CAN_F11R2_FB5                        (0x1U << 5U)                      /* Filter bit 5*/
#define CAN_F11R2_FB6                        (0x1U << 6U)                      /* Filter bit 6*/
#define CAN_F11R2_FB7                        (0x1U << 7U)                      /* Filter bit 7*/
#define CAN_F11R2_FB8                        (0x1U << 8U)                      /* Filter bit 8*/
#define CAN_F11R2_FB9                        (0x1U << 9U)                      /* Filter bit 9*/
#define CAN_F11R2_FB10                       (0x1U << 10U)                     /* Filter bit 10*/
#define CAN_F11R2_FB11                       (0x1U << 11U)                     /* Filter bit 11*/
#define CAN_F11R2_FB12                       (0x1U << 12U)                     /* Filter bit 12*/
#define CAN_F11R2_FB13                       (0x1U << 13U)                     /* Filter bit 13*/
#define CAN_F11R2_FB14                       (0x1U << 14U)                     /* Filter bit 14*/
#define CAN_F11R2_FB15                       (0x1U << 15U)                     /* Filter bit 15*/
#define CAN_F11R2_FB16                       (0x1U << 16U)                     /* Filter bit 16*/
#define CAN_F11R2_FB17                       (0x1U << 17U)                     /* Filter bit 17*/
#define CAN_F11R2_FB18                       (0x1U << 18U)                     /* Filter bit 18*/
#define CAN_F11R2_FB19                       (0x1U << 19U)                     /* Filter bit 19*/
#define CAN_F11R2_FB20                       (0x1U << 20U)                     /* Filter bit 20*/
#define CAN_F11R2_FB21                       (0x1U << 21U)                     /* Filter bit 21*/
#define CAN_F11R2_FB22                       (0x1U << 22U)                     /* Filter bit 22*/
#define CAN_F11R2_FB23                       (0x1U << 23U)                     /* Filter bit 23*/
#define CAN_F11R2_FB24                       (0x1U << 24U)                     /* Filter bit 24*/
#define CAN_F11R2_FB25                       (0x1U << 25U)                     /* Filter bit 25*/
#define CAN_F11R2_FB26                       (0x1U << 26U)                     /* Filter bit 26*/
#define CAN_F11R2_FB27                       (0x1U << 27U)                     /* Filter bit 27*/
#define CAN_F11R2_FB28                       (0x1U << 28U)                     /* Filter bit 28*/
#define CAN_F11R2_FB29                       (0x1U << 29U)                     /* Filter bit 29*/
#define CAN_F11R2_FB30                       (0x1U << 30U)                     /* Filter bit 30*/
#define CAN_F11R2_FB31                       (0x1U << 31U)                     /* Filter bit 31*/

#define CAN_F12R2_FB0                        (0x1U << 0U)                      /* Filter bit 0*/
#define CAN_F12R2_FB1                        (0x1U << 1U)                      /* Filter bit 1*/
#define CAN_F12R2_FB2                        (0x1U << 2U)                      /* Filter bit 2*/
#define CAN_F12R2_FB3                        (0x1U << 3U)                      /* Filter bit 3*/
#define CAN_F12R2_FB4                        (0x1U << 4U)                      /* Filter bit 4*/
#define CAN_F12R2_FB5                        (0x1U << 5U)                      /* Filter bit 5*/
#define CAN_F12R2_FB6                        (0x1U << 6U)                      /* Filter bit 6*/
#define CAN_F12R2_FB7                        (0x1U << 7U)                      /* Filter bit 7*/
#define CAN_F12R2_FB8                        (0x1U << 8U)                      /* Filter bit 8*/
#define CAN_F12R2_FB9                        (0x1U << 9U)                      /* Filter bit 9*/
#define CAN_F12R2_FB10                       (0x1U << 10U)                     /* Filter bit 10*/
#define CAN_F12R2_FB11                       (0x1U << 11U)                     /* Filter bit 11*/
#define CAN_F12R2_FB12                       (0x1U << 12U)                     /* Filter bit 12*/
#define CAN_F12R2_FB13                       (0x1U << 13U)                     /* Filter bit 13*/
#define CAN_F12R2_FB14                       (0x1U << 14U)                     /* Filter bit 14*/
#define CAN_F12R2_FB15                       (0x1U << 15U)                     /* Filter bit 15*/
#define CAN_F12R2_FB16                       (0x1U << 16U)                     /* Filter bit 16*/
#define CAN_F12R2_FB17                       (0x1U << 17U)                     /* Filter bit 17*/
#define CAN_F12R2_FB18                       (0x1U << 18U)                     /* Filter bit 18*/
#define CAN_F12R2_FB19                       (0x1U << 19U)                     /* Filter bit 19*/
#define CAN_F12R2_FB20                       (0x1U << 20U)                     /* Filter bit 20*/
#define CAN_F12R2_FB21                       (0x1U << 21U)                     /* Filter bit 21*/
#define CAN_F12R2_FB22                       (0x1U << 22U)                     /* Filter bit 22*/
#define CAN_F12R2_FB23                       (0x1U << 23U)                     /* Filter bit 23*/
#define CAN_F12R2_FB24                       (0x1U << 24U)                     /* Filter bit 24*/
#define CAN_F12R2_FB25                       (0x1U << 25U)                     /* Filter bit 25*/
#define CAN_F12R2_FB26                       (0x1U << 26U)                     /* Filter bit 26*/
#define CAN_F12R2_FB27                       (0x1U << 27U)                     /* Filter bit 27*/
#define CAN_F12R2_FB28                       (0x1U << 28U)                     /* Filter bit 28*/
#define CAN_F12R2_FB29                       (0x1U << 29U)                     /* Filter bit 29*/
#define CAN_F12R2_FB30                       (0x1U << 30U)                     /* Filter bit 30*/
#define CAN_F12R2_FB31                       (0x1U << 31U)                     /* Filter bit 31*/

#define CAN_F13R2_FB0                        (0x1U << 0U)                      /* Filter bit 0*/
#define CAN_F13R2_FB1                        (0x1U << 1U)                      /* Filter bit 1*/
#define CAN_F13R2_FB2                        (0x1U << 2U)                      /* Filter bit 2*/
#define CAN_F13R2_FB3                        (0x1U << 3U)                      /* Filter bit 3*/
#define CAN_F13R2_FB4                        (0x1U << 4U)                      /* Filter bit 4*/
#define CAN_F13R2_FB5                        (0x1U << 5U)                      /* Filter bit 5*/
#define CAN_F13R2_FB6                        (0x1U << 6U)                      /* Filter bit 6*/
#define CAN_F13R2_FB7                        (0x1U << 7U)                      /* Filter bit 7*/
#define CAN_F13R2_FB8                        (0x1U << 8U)                      /* Filter bit 8*/
#define CAN_F13R2_FB9                        (0x1U << 9U)                      /* Filter bit 9*/
#define CAN_F13R2_FB10                       (0x1U << 10U)                     /* Filter bit 10*/
#define CAN_F13R2_FB11                       (0x1U << 11U)                     /* Filter bit 11*/
#define CAN_F13R2_FB12                       (0x1U << 12U)                     /* Filter bit 12*/
#define CAN_F13R2_FB13                       (0x1U << 13U)                     /* Filter bit 13*/
#define CAN_F13R2_FB14                       (0x1U << 14U)                     /* Filter bit 14*/
#define CAN_F13R2_FB15                       (0x1U << 15U)                     /* Filter bit 15*/
#define CAN_F13R2_FB16                       (0x1U << 16U)                     /* Filter bit 16*/
#define CAN_F13R2_FB17                       (0x1U << 17U)                     /* Filter bit 17*/
#define CAN_F13R2_FB18                       (0x1U << 18U)                     /* Filter bit 18*/
#define CAN_F13R2_FB19                       (0x1U << 19U)                     /* Filter bit 19*/
#define CAN_F13R2_FB20                       (0x1U << 20U)                     /* Filter bit 20*/
#define CAN_F13R2_FB21                       (0x1U << 21U)                     /* Filter bit 21*/
#define CAN_F13R2_FB22                       (0x1U << 22U)                     /* Filter bit 22*/
#define CAN_F13R2_FB23                       (0x1U << 23U)                     /* Filter bit 23*/
#define CAN_F13R2_FB24                       (0x1U << 24U)                     /* Filter bit 24*/
#define CAN_F13R2_FB25                       (0x1U << 25U)                     /* Filter bit 25*/
#define CAN_F13R2_FB26                       (0x1U << 26U)                     /* Filter bit 26*/
#define CAN_F13R2_FB27                       (0x1U << 27U)                     /* Filter bit 27*/
#define CAN_F13R2_FB28                       (0x1U << 28U)                     /* Filter bit 28*/
#define CAN_F13R2_FB29                       (0x1U << 29U)                     /* Filter bit 29*/
#define CAN_F13R2_FB30                       (0x1U << 30U)                     /* Filter bit 30*/
#define CAN_F13R2_FB31                       (0x1U << 31U)                     /* Filter bit 31*/


#define SPI_CR1_CPHA                        (0x1U << 0U)                       /* Clock Phase*/
#define SPI_CR1_CPOL                        (0x1U << 1U)                       /* Clock Polarity*/
#define SPI_CR1_MSTR                        (0x1U << 2U)                       /* Master Selection*/
#define SPI_CR1_BR                          (0x7U << 3U)                       /* BR[2:0] bits (Baud Rate Control)*/
#define SPI_CR1_BR_0                        (0x1U << 3U)
#define SPI_CR1_BR_1                        (0x2U << 3U)
#define SPI_CR1_BR_2                        (0x4U << 3U)
#define SPI_CR1_SPE                         (0x1U << 6U)                       /* SPI Enable*/
#define SPI_CR1_LSBFIRST                    (0x1U << 7U)                       /* Frame Format*/
#define SPI_CR1_SSI                         (0x1U << 8U)                       /* Internal slave select*/
#define SPI_CR1_SSM                         (0x1U << 9U)                       /* Software slave management*/
#define SPI_CR1_RXONLY                      (0x1U << 10U)                      /* Receive only*/
#define SPI_CR1_DFF                         (0x1U << 11U)                      /* Data Frame Format*/
#define SPI_CR1_CRCNEXT                     (0x1U << 12U)                      /* Transmit CRC next*/
#define SPI_CR1_CRCEN                       (0x1U << 13U)                      /* Hardware CRC calculation enable*/
#define SPI_CR1_BIDIOE                      (0x1U << 14U)                      /* Output enable in bidirectional mode*/
#define SPI_CR1_BIDIMODE                    (0x1U << 15U)                      /* Bidirectional data mode enable*/

#define SPI_CR2_RXDMAEN                     (0x1U << 0U)                       /* Rx Buffer DMA Enable*/
#define SPI_CR2_TXDMAEN                     (0x1U << 1U)                       /* Tx Buffer DMA Enable*/
#define SPI_CR2_SSOE                        (0x1U << 2U)                       /* SS Output Enable*/
#define SPI_CR2_ERRIE                       (0x1U << 5U)                       /* Error Interrupt Enable*/
#define SPI_CR2_RXNEIE                      (0x1U << 6U)                       /* RX buffer Not Empty Interrupt Enable*/
#define SPI_CR2_TXEIE                       (0x1U << 7U)                       /* Tx buffer Empty Interrupt Enable*/

#define SPI_SR_RXNE                         (0x1U << 0U)                       /* Receive buffer Not Empty*/
#define SPI_SR_TXE                          (0x1U << 1U)                       /* Transmit buffer Empty*/
#define SPI_SR_CHSIDE                       (0x1U << 2U)                       /* Channel side*/
#define SPI_SR_UDR                          (0x1U << 3U)                       /* Underrun flag*/
#define SPI_SR_CRCERR                       (0x1U << 4U)                       /* CRC Error flag*/
#define SPI_SR_MODF                         (0x1U << 5U)                       /* Mode fault*/
#define SPI_SR_OVR                          (0x1U << 6U)                       /* Overrun flag*/
#define SPI_SR_BSY                          (0x1U << 7U)                       /* Busy flag*/

#define SPI_DR_DR                           (0xFFFFU << 0U)                    /* Data Register*/

#define SPI_CRCPR_CRCPOLY                   (0xFFFFU << 0U)                    /* CRC polynomial register*/

#define SPI_RXCRCR_RXCRC                    (0xFFFFU << 0U)                    /* Rx CRC Register*/

#define SPI_TXCRCR_TXCRC                    (0xFFFFU << 0U)                    /* Tx CRC Register*/

#define SPI_I2SCFGR_I2SMOD                  (0x1U << 11U)                      /* I2S mode selection*/


#define I2C_CR1_PE                          (0x1U << 0U)                       /* Peripheral Enable*/
#define I2C_CR1_SMBUS                       (0x1U << 1U)                       /* SMBus Mode*/
#define I2C_CR1_SMBTYPE                     (0x1U << 3U)                       /* SMBus Type*/
#define I2C_CR1_ENARP                       (0x1U << 4U)                       /* ARP Enable*/
#define I2C_CR1_ENPEC                       (0x1U << 5U)                       /* PEC Enable*/
#define I2C_CR1_ENGC                        (0x1U << 6U)                       /* General Call Enable*/
#define I2C_CR1_NOSTRETCH                   (0x1U << 7U)                       /* Clock Stretching Disable (Slave mode)*/
#define I2C_CR1_START                       (0x1U << 8U)                       /* Start Generation*/
#define I2C_CR1_STOP                        (0x1U << 9U)                       /* Stop Generation*/
#define I2C_CR1_ACK                         (0x1U << 10U)                      /* Acknowledge Enable*/
#define I2C_CR1_POS                         (0x1U << 11U)                      /* Acknowledge/PEC Position (for data reception)*/
#define I2C_CR1_PEC                         (0x1U << 12U)                      /* Packet Error Checking*/
#define I2C_CR1_ALERT                       (0x1U << 13U)                      /* SMBus Alert*/
#define I2C_CR1_SWRST                       (0x1U << 15U)                      /* Software Reset*/

#define I2C_CR2_FREQ                        (0x3FU << 0U)                      /* FREQ[5:0] bits (Peripheral Clock Frequency)*/
#define I2C_CR2_FREQ_0                      (0x01U << 0U)
#define I2C_CR2_FREQ_1                      (0x02U << 0U)
#define I2C_CR2_FREQ_2                      (0x04U << 0U)
#define I2C_CR2_FREQ_3                      (0x08U << 0U)
#define I2C_CR2_FREQ_4                      (0x10U << 0U)
#define I2C_CR2_FREQ_5                      (0x20U << 0U)
#define I2C_CR2_ITERREN                     (0x1U << 8U)                       /* Error Interrupt Enable*/
#define I2C_CR2_ITEVTEN                     (0x1U << 9U)                       /* Event Interrupt Enable*/
#define I2C_CR2_ITBUFEN                     (0x1U << 10U)                      /* Buffer Interrupt Enable*/
#define I2C_CR2_DMAEN                       (0x1U << 11U)                      /* DMA Requests Enable*/
#define I2C_CR2_LAST                        (0x1U << 12U)                      /* DMA Last Transfer*/

#define I2C_OAR1_ADD1_7                     0x000000FEU                        /* Interface Address*/
#define I2C_OAR1_ADD8_9                     0x00000300U                        /* Interface Address*/
#define I2C_OAR1_ADD0                       (0x1U << 0U)                       /* Bit 0*/
#define I2C_OAR1_ADD1                       (0x1U << 1U)                       /* Bit 1*/
#define I2C_OAR1_ADD2                       (0x1U << 2U)                       /* Bit 2*/
#define I2C_OAR1_ADD3                       (0x1U << 3U)                       /* Bit 3*/
#define I2C_OAR1_ADD4                       (0x1U << 4U)                       /* Bit 4*/
#define I2C_OAR1_ADD5                       (0x1U << 5U)                       /* Bit 5*/
#define I2C_OAR1_ADD6                       (0x1U << 6U)                       /* Bit 6*/
#define I2C_OAR1_ADD7                       (0x1U << 7U)                       /* Bit 7*/
#define I2C_OAR1_ADD8                       (0x1U << 8U)                       /* Bit 8*/
#define I2C_OAR1_ADD9                       (0x1U << 9U)                       /* Bit 9*/
#define I2C_OAR1_ADDMODE                    (0x1U << 15U)                      /* Addressing Mode (Slave mode)*/

#define I2C_OAR2_ENDUAL                     (0x1U << 0U)                       /* Dual addressing mode enable*/
#define I2C_OAR2_ADD2                       (0x7FU << 1U)                      /* Interface address*/

#define I2C_DR_DR                           (0xFFU << 0U)                      /* 8-bit Data Register*/

#define I2C_SR1_SB                          (0x1U << 0U)                       /* Start Bit (Master mode)*/
#define I2C_SR1_ADDR                        (0x1U << 1U)                       /* Address sent (master mode)/matched (slave mode)*/
#define I2C_SR1_BTF                         (0x1U << 2U)                       /* Byte Transfer Finished*/
#define I2C_SR1_ADD10                       (0x1U << 3U)                       /* 10-bit header sent (Master mode)*/
#define I2C_SR1_STOPF                       (0x1U << 4U)                       /* Stop detection (Slave mode)*/
#define I2C_SR1_RXNE                        (0x1U << 6U)                       /* Data Register not Empty (receivers)*/
#define I2C_SR1_TXE                         (0x1U << 7U)                       /* Data Register Empty (transmitters)*/
#define I2C_SR1_BERR                        (0x1U << 8U)                       /* Bus Error*/
#define I2C_SR1_ARLO                        (0x1U << 9U)                       /* Arbitration Lost (master mode)*/
#define I2C_SR1_AF                          (0x1U << 10U)                      /* Acknowledge Failure*/
#define I2C_SR1_OVR                         (0x1U << 11U)                      /* Overrun/Underrun*/
#define I2C_SR1_PECERR                      (0x1U << 12U)                      /* PEC Error in reception*/
#define I2C_SR1_TIMEOUT                     (0x1U << 14U)                      /* Timeout or Tlow Error*/
#define I2C_SR1_SMBALERT                    (0x1U << 15U)                      /* SMBus Alert*/

#define I2C_SR2_MSL                         (0x1U << 0U)                       /* Master/Slave*/
#define I2C_SR2_BUSY                        (0x1U << 1U)                       /* Bus Busy*/
#define I2C_SR2_TRA                         (0x1U << 2U)                       /* Transmitter/Receiver*/
#define I2C_SR2_GENCALL                     (0x1U << 4U)                       /* General Call Address (Slave mode)*/
#define I2C_SR2_SMBDEFAULT                  (0x1U << 5U)                       /* SMBus Device Default Address (Slave mode)*/
#define I2C_SR2_SMBHOST                     (0x1U << 6U)                       /* SMBus Host Header (Slave mode)*/
#define I2C_SR2_DUALF                       (0x1U << 7U)                       /* Dual Flag (Slave mode)*/
#define I2C_SR2_PEC                         (0xFFU << 8U)                      /* Packet Error Checking Register*/

#define I2C_CCR_CCR                         (0xFFFU << 0U)                     /* Clock Control Register in Fast/Standard mode (Master mode)*/
#define I2C_CCR_DUTY                        (0x1U << 14U)                      /* Fast Mode Duty Cycle*/
#define I2C_CCR_FS                          (0x1U << 15U)                      /* I2C Master Mode Selection*/

#define I2C_TRISE_TRISE                     (0x3FU << 0U)                      /* Maximum Rise Time in Fast/Standard mode (Master mode)*/


#define USART_SR_PE                         (0x1U << 0U)                       /* Parity Error*/
#define USART_SR_FE                         (0x1U << 1U)                       /* Framing Error*/
#define USART_SR_NE                         (0x1U << 2U)                       /* Noise Error Flag*/
#define USART_SR_ORE                        (0x1U << 3U)                       /* OverRun Error*/
#define USART_SR_IDLE                       (0x1U << 4U)                       /* IDLE line detected*/
#define USART_SR_RXNE                       (0x1U << 5U)                       /* Read Data Register Not Empty*/
#define USART_SR_TC                         (0x1U << 6U)                       /* Transmission Complete*/
#define USART_SR_TXE                        (0x1U << 7U)                       /* Transmit Data Register Empty*/
#define USART_SR_LBD                        (0x1U << 8U)                       /* LIN Break Detection Flag*/
#define USART_SR_CTS                        (0x1U << 9U)                       /* CTS Flag*/

#define USART_DR_DR                         (0x1FFU << 0U)                     /* Data value*/

#define USART_BRR_DIV_Fraction              (0xFU << 0U)                       /* Fraction of USARTDIV*/
#define USART_BRR_DIV_Mantissa              (0xFFFU << 4U)                     /* Mantissa of USARTDIV*/

#define USART_CR1_SBK                       (0x1U << 0U)                       /* Send Break*/
#define USART_CR1_RWU                       (0x1U << 1U)                       /* Receiver wakeup*/
#define USART_CR1_RE                        (0x1U << 2U)                       /* Receiver Enable*/
#define USART_CR1_TE                        (0x1U << 3U)                       /* Transmitter Enable*/
#define USART_CR1_IDLEIE                    (0x1U << 4U)                       /* IDLE Interrupt Enable*/
#define USART_CR1_RXNEIE                    (0x1U << 5U)                       /* RXNE Interrupt Enable*/
#define USART_CR1_TCIE                      (0x1U << 6U)                       /* Transmission Complete Interrupt Enable*/
#define USART_CR1_TXEIE                     (0x1U << 7U)                       /* PE Interrupt Enable*/
#define USART_CR1_PEIE                      (0x1U << 8U)                       /* PE Interrupt Enable*/
#define USART_CR1_PS                        (0x1U << 9U)                       /* Parity Selection*/
#define USART_CR1_PCE                       (0x1U << 10U)                      /* Parity Control Enable*/
#define USART_CR1_WAKE                      (0x1U << 11U)                      /* Wakeup method*/
#define USART_CR1_M                         (0x1U << 12U)                      /* Word length*/
#define USART_CR1_UE                        (0x1U << 13U)                      /* USART Enable*/

#define USART_CR2_ADD                       (0xFU << 0U)                       /* Address of the USART node*/
#define USART_CR2_LBDL                      (0x1U << 5U)                       /* LIN Break Detection Length*/
#define USART_CR2_LBDIE                     (0x1U << 6U)                       /* LIN Break Detection Interrupt Enable*/
#define USART_CR2_LBCL                      (0x1U << 8U)                       /* Last Bit Clock pulse*/
#define USART_CR2_CPHA                      (0x1U << 9U)                       /* Clock Phase*/
#define USART_CR2_CPOL                      (0x1U << 10U)                      /* Clock Polarity*/
#define USART_CR2_CLKEN                     (0x1U << 11U)                      /* Clock Enable*/
#define USART_CR2_STOP                      (0x3U << 12U)                      /* STOP[1:0] bits (STOP bits)*/
#define USART_CR2_STOP_0                    (0x1U << 12U)
#define USART_CR2_STOP_1                    (0x2U << 12U)
#define USART_CR2_LINEN                     (0x1U << 14U)                      /* LIN mode enable*/

#define USART_CR3_EIE                       (0x1U << 0U)                       /* Error Interrupt Enable*/
#define USART_CR3_IREN                      (0x1U << 1U)                       /* IrDA mode Enable*/
#define USART_CR3_IRLP                      (0x1U << 2U)                       /* IrDA Low-Power*/
#define USART_CR3_HDSEL                     (0x1U << 3U)                       /* Half-Duplex Selection*/
#define USART_CR3_NACK                      (0x1U << 4U)                       /* Smartcard NACK enable*/
#define USART_CR3_SCEN                      (0x1U << 5U)                       /* Smartcard mode enable*/
#define USART_CR3_DMAR                      (0x1U << 6U)                       /* DMA Enable Receiver*/
#define USART_CR3_DMAT                      (0x1U << 7U)                       /* DMA Enable Transmitter*/
#define USART_CR3_RTSE                      (0x1U << 8U)                       /* RTS Enable*/
#define USART_CR3_CTSE                      (0x1U << 9U)                       /* CTS Enable*/
#define USART_CR3_CTSIE                     (0x1U << 10U)                      /* CTS Interrupt Enable*/

#define USART_GTPR_PSC                      (0xFFU << 0U)                      /* PSC[7:0] bits (Prescaler value)*/
#define USART_GTPR_PSC_0                    (0x01U << 0U)
#define USART_GTPR_PSC_1                    (0x02U << 0U)
#define USART_GTPR_PSC_2                    (0x04U << 0U)
#define USART_GTPR_PSC_3                    (0x08U << 0U)
#define USART_GTPR_PSC_4                    (0x10U << 0U)
#define USART_GTPR_PSC_5                    (0x20U << 0U)
#define USART_GTPR_PSC_6                    (0x40U << 0U)
#define USART_GTPR_PSC_7                    (0x80U << 0U)
#define USART_GTPR_GT                       (0xFFU << 8U)                     /* Guard time value*/


#define DBGMCU_IDCODE_DEV_ID                (0xFFFU << 0U)                     /* Device Identifier*/
#define DBGMCU_IDCODE_REV_ID                (0xFFFFU << 16U)                   /* REV_ID[15:0] bits (Revision Identifier)*/
#define DBGMCU_IDCODE_REV_ID_0              (0x0001U << 16U)
#define DBGMCU_IDCODE_REV_ID_1              (0x0002U << 16U)
#define DBGMCU_IDCODE_REV_ID_2              (0x0004U << 16U)
#define DBGMCU_IDCODE_REV_ID_3              (0x0008U << 16U)
#define DBGMCU_IDCODE_REV_ID_4              (0x0010U << 16U)
#define DBGMCU_IDCODE_REV_ID_5              (0x0020U << 16U)
#define DBGMCU_IDCODE_REV_ID_6              (0x0040U << 16U)
#define DBGMCU_IDCODE_REV_ID_7              (0x0080U << 16U)
#define DBGMCU_IDCODE_REV_ID_8              (0x0100U << 16U)
#define DBGMCU_IDCODE_REV_ID_9              (0x0200U << 16U)
#define DBGMCU_IDCODE_REV_ID_10             (0x0400U << 16U)
#define DBGMCU_IDCODE_REV_ID_11             (0x0800U << 16U)
#define DBGMCU_IDCODE_REV_ID_12             (0x1000U << 16U)
#define DBGMCU_IDCODE_REV_ID_13             (0x2000U << 16U)
#define DBGMCU_IDCODE_REV_ID_14             (0x4000U << 16U)
#define DBGMCU_IDCODE_REV_ID_15             (0x8000U << 16U)

#define DBGMCU_CR_DBG_SLEEP                 (0x1U << 0U)                       /* Debug Sleep Mode*/
#define DBGMCU_CR_DBG_STOP                  (0x1U << 1U)                       /* Debug Stop Mode*/
#define DBGMCU_CR_DBG_STANDBY               (0x1U << 2U)                       /* Debug Standby mode*/
#define DBGMCU_CR_TRACE_IOEN                (0x1U << 5U)                       /* Trace Pin Assignment Control*/
#define DBGMCU_CR_TRACE_MODE                (0x3U << 6U)           /*!< TRACE_MODE[1:0] bits (Trace Pin Assignment Control) */
#define DBGMCU_CR_TRACE_MODE_0              (0x1U << 6U)
#define DBGMCU_CR_TRACE_MODE_1              (0x2U << 6U)
#define DBGMCU_CR_DBG_IWDG_STOP             (0x1U << 8U)                       /* Debug Independent Watchdog stopped when Core is halted*/
#define DBGMCU_CR_DBG_WWDG_STOP             (0x1U << 9U)                       /* Debug Window Watchdog stopped when Core is halted*/
#define DBGMCU_CR_DBG_TIM1_STOP             (0x1U << 10U)                      /* TIM1 counter stopped when core is halted*/
#define DBGMCU_CR_DBG_TIM2_STOP             (0x1U << 11U)                      /* TIM2 counter stopped when core is halted*/
#define DBGMCU_CR_DBG_TIM3_STOP             (0x1U << 12U)                      /* TIM3 counter stopped when core is halted*/
#define DBGMCU_CR_DBG_TIM4_STOP             (0x1U << 13U)                      /* TIM4 counter stopped when core is halted*/
#define DBGMCU_CR_DBG_CAN1_STOP             (0x1U << 14U)                      /* Debug CAN1 stopped when Core is halted*/
#define DBGMCU_CR_DBG_I2C1_SMBUS_TIMEOUT    (0x1U << 15U)                      /* SMBUS timeout mode stopped when Core is halted*/
#define DBGMCU_CR_DBG_I2C2_SMBUS_TIMEOUT    (0x1U << 16U)                      /* SMBUS timeout mode stopped when Core is halted*/


#define FLASH_ACR_LATENCY                   (0x7U << 0U)                       /* LATENCY[2:0] bits (Latency)*/
#define FLASH_ACR_LATENCY_0                 (0x1U << 0U)
#define FLASH_ACR_LATENCY_1                 (0x2U << 0U)
#define FLASH_ACR_LATENCY_2                 (0x4U << 0U)
#define FLASH_ACR_HLFCYA                    (0x1U << 3U)                       /* Flash Half Cycle Access Enable*/
#define FLASH_ACR_PRFTBE                    (0x1U << 4U)                       /* Prefetch Buffer Enable*/
#define FLASH_ACR_PRFTBS                    (0x1U << 5U)                       /* Prefetch Buffer Status*/

#define FLASH_KEYR_FKEYR                    (0xFFFFFFFFU << 0U)                /* FPEC Key*/

#define RDP_KEY                             (0xA5U << 0U)                      /* RDP Key*/
#define FLASH_KEY1                          (0x45670123U << 0U)                /* FPEC Key1*/
#define FLASH_KEY2                          (0xCDEF89ABU << 0U)                /* FPEC Key2*/

#define FLASH_OPTKEYR_OPTKEYR               (0xFFFFFFFFU << 0U)                /* Option Byte Key*/

#define FLASH_SR_BSY                        (0x1U << 0U)                       /* Busy*/
#define FLASH_SR_PGERR                      (0x1U << 2U)                       /* Programming Error*/
#define FLASH_SR_WRPRTERR                   (0x1U << 4U)                       /* Write Protection Error*/
#define FLASH_SR_EOP                        (0x1U << 5U)                       /* End of operation*/

#define FLASH_CR_PG                         (0x1U << 0U)                       /* Programming*/
#define FLASH_CR_PER                        (0x1U << 1U)                       /* Page Erase*/
#define FLASH_CR_MER                        (0x1U << 2U)                       /* Mass Erase*/
#define FLASH_CR_OPTPG                      (0x1U << 4U)                       /* Option Byte Programming*/
#define FLASH_CR_OPTER                      (0x1U << 5U)                       /* Option Byte Erase*/
#define FLASH_CR_STRT                       (0x1U << 6U)                       /* Start*/
#define FLASH_CR_LOCK                       (0x1U << 7U)                       /* Lock*/
#define FLASH_CR_OPTWRE                     (0x1U << 9U)                       /* Option Bytes Write Enable*/
#define FLASH_CR_ERRIE                      (0x1U << 10U)                      /* Error Interrupt Enable*/
#define FLASH_CR_EOPIE                      (0x1U << 12U)                      /* End of operation interrupt enable*/

#define FLASH_AR_FAR                        (0xFFFFFFFFU << 0U)                /* Flash Address*/

#define FLASH_OBR_OPTERR                    (0x1U << 0U)                       /* Option Byte Error*/
#define FLASH_OBR_RDPRT                     (0x1U << 1U)                       /* Read protection*/
#define FLASH_OBR_IWDG_SW                   (0x1U << 2U)                       /* IWDG SW*/
#define FLASH_OBR_nRST_STOP                 (0x1U << 3U)                       /* nRST_STOP*/
#define FLASH_OBR_nRST_STDBY                (0x1U << 4U)                       /* nRST_STDBY*/
#define FLASH_OBR_USER                      (0x7U << 2U)                       /* User Option Bytes*/
#define FLASH_OBR_DATA0                     (0xFFU << 10U)                     /* Data0*/
#define FLASH_OBR_DATA1                     (0xFFU << 18U)                     /* Data1*/

#define FLASH_WRPR_WRP                      (0xFFFFFFFFU << 0U)                /* Write Protect*/

#define FLASH_RDP_RDP                       (0xFFU << 0U)                      /* Read protection option byte*/
#define FLASH_RDP_nRDP                      (0xFFU << 8U)                      /* Read protection complemented option byte*/

#define FLASH_USER_USER                     (0xFFU << 16U)                     /* User option byte*/
#define FLASH_USER_nUSER                    (0xFFU << 24U)                     /* User complemented option byte*/

#define FLASH_DATA0_DATA0                   (0xFFU << 0U)                      /* User data storage option byte*/
#define FLASH_DATA0_nDATA0                  (0xFFU << 8U)                      /* User data storage complemented option byte*/

#define FLASH_DATA1_DATA1                   (0xFFU << 16U)                     /* User data storage option byte*/
#define FLASH_DATA1_nDATA1                  (0xFFU << 24U)                     /* User data storage complemented option byte*/

#define FLASH_WRP0_WRP0                     (0xFFU << 0U)                      /* Flash memory write protection option bytes*/
#define FLASH_WRP0_nWRP0                    (0xFFU << 8U)                      /* Flash memory write protection complemented option bytes*/

#define FLASH_WRP1_WRP1                     (0xFFU << 16U)                     /* Flash memory write protection option bytes*/
#define FLASH_WRP1_nWRP1                    (0xFFU << 24U)                     /* Flash memory write protection complemented option bytes*/

#define FLASH_WRP2_WRP2                     (0xFFU << 0U)                      /* Flash memory write protection option bytes*/
#define FLASH_WRP2_nWRP2                    (0xFFU << 8U)                      /* Flash memory write protection complemented option bytes*/

#define FLASH_WRP3_WRP3                     (0xFFU << 16U)                     /* Flash memory write protection option bytes*/
#define FLASH_WRP3_nWRP3                    (0xFFU << 24U)                     /* Flash memory write protection complemented option bytes*/


#endif
