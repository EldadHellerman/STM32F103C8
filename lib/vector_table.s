.syntax unified
.section .vector_table

.extern _start, _estack
.global _DefaultHandler, _VectorTable
.equ _RESET, _start
.weak _NMI, _HardFault, _MemManage, _BusFault, _UsageFault, _SVCall, _DebugMonitor, _PendSV, _Systick
.weak _IRQ0_WWDG, _IRQ1_PVD, _IRQ2_TAMPER, _IRQ3_RTC, _IRQ41_RTCAlarm, _IRQ4_FLASH, _IRQ5_RCC, _IRQ48_FSMC
.weak _IRQ6_EXTI0, _IRQ7_EXTI1, _IRQ8_EXTI2, _IRQ9_EXTI3, _IRQ10_EXTI4, _IRQ23_EXTI9_5, _IRQ40_EXTI15_10
.weak _IRQ18_ADC1_2, _IRQ47_ADC3
.weak _IRQ19_USB_HP_CAN_TX, _IRQ20_USB_LP_CAN_RX0, _IRQ42_USBWakeup, _IRQ21_CAN_RX1, _IRQ22_CAN_SCE
.weak _IRQ31_I2C1_EV, _IRQ32_I2C1_ER, _IRQ33_I2C2_EV, _IRQ34_I2C2_ER, _IRQ35_SPI1, _IRQ36_SPI2, _IRQ51_SPI3, _IRQ49_SDIO
.weak _IRQ37_USART1, _IRQ38_USART2, _IRQ39_USART3, _IRQ52_UART4, _IRQ53_UART5
.weak _IRQ24_TIM1_BRK, _IRQ25_TIM1_UP, _IRQ26_TIM1_TRG_COM, _IRQ27_TIM1_CC, _IRQ28_TIM2, _IRQ29_TIM3, _IRQ30_TIM4, _IRQ50_TIM5, _IRQ54_TIM6, _IRQ55_TIM7, _IRQ43_TIM8_BRK, _IRQ44_TIM8_UP, _IRQ45_TIM8_TRG_COM, _IRQ46_TIM8_CC
.weak _IRQ11_DMA1_Channel1, _IRQ12_DMA1_Channel2, _IRQ13_DMA1_Channel3, _IRQ14_DMA1_Channel4, _IRQ15_DMA1_Channel5, _IRQ16_DMA1_Channel6, _IRQ17_DMA1_Channel7, _IRQ56_DMA2_Channel1, _IRQ57_DMA2_Channel2, _IRQ58_DMA2_Channel3, _IRQ59_DMA2_Channel4_5

_VectorTable:

.word _estack             @initial SP value
.word _RESET              @Reset
.word _NMI                @NMI
.word _HardFault          @HardFault
.word _MemManage          @MemManage
.word _BusFault           @BusFault
.word _UsageFault         @UsageFault
.word 0x00000000          @Reserved
.word 0x00000000          @Reserved
.word 0x00000000          @Reserved
.word 0x00000000          @Reserved
.word _SVCall             @SVCall
.word _DebugMonitor       @DebugMonitor
.word 0x00000000          @Reserved
.word _PendSV             @PendSV
.word _Systick            @Systick

.word _IRQ0_WWDG                     @IRQ0 - WWDG
.word _IRQ1_PVD                      @IRQ1 - PVD
.word _IRQ2_TAMPER                   @IRQ2 - TAMPER
.word _IRQ3_RTC                      @IRQ3 - RTC
.word _IRQ4_FLASH                    @IRQ4 - FLASH
.word _IRQ5_RCC                      @IRQ5 - RCC
.word _IRQ6_EXTI0                    @IRQ6 - EXTI0
.word _IRQ7_EXTI1                    @IRQ7 - EXTI1
.word _IRQ8_EXTI2                    @IRQ8 - EXTI2
.word _IRQ9_EXTI3                    @IRQ9 - EXTI3
.word _IRQ10_EXTI4                   @IRQ10 - EXTI4
.word _IRQ11_DMA1_Channel1           @IRQ11 - DMA1_Channel1
.word _IRQ12_DMA1_Channel2           @IRQ12 - DMA1_Channel2
.word _IRQ13_DMA1_Channel3           @IRQ13 - DMA1_Channel3
.word _IRQ14_DMA1_Channel4           @IRQ14 - DMA1_Channel4
.word _IRQ15_DMA1_Channel5           @IRQ15 - DMA1_Channel5
.word _IRQ16_DMA1_Channel6           @IRQ16 - DMA1_Channel6
.word _IRQ17_DMA1_Channel7           @IRQ17 - DMA1_Channel7
.word _IRQ18_ADC1_2                  @IRQ18 - ADC1_2
.word _IRQ19_USB_HP_CAN_TX           @IRQ19 - USB_HP_CAN_TX
.word _IRQ20_USB_LP_CAN_RX0          @IRQ20 - USB_HP_CAN_RX0
.word _IRQ21_CAN_RX1                 @IRQ21 - CAN_RX1
.word _IRQ22_CAN_SCE                 @IRQ22 - CAN_SCE
.word _IRQ23_EXTI9_5                 @IRQ23 - EXTI9_5
.word _IRQ24_TIM1_BRK                @IRQ24 - TIM1_BRK
.word _IRQ25_TIM1_UP                 @IRQ25 - TIM1_UP
.word _IRQ26_TIM1_TRG_COM            @IRQ26 - TIM1_TRG_COM
.word _IRQ27_TIM1_CC                 @IRQ27 - TIM1_CC
.word _IRQ28_TIM2                    @IRQ28 - TIM2
.word _IRQ29_TIM3                    @IRQ29 - TIM3
.word _IRQ30_TIM4                    @IRQ30 - TIM4
.word _IRQ31_I2C1_EV                 @IRQ31 - I2C1_EV
.word _IRQ32_I2C1_ER                 @IRQ32 - I2C1_ER
.word _IRQ33_I2C2_EV                 @IRQ33 - I2C2_EV
.word _IRQ34_I2C2_ER                 @IRQ34 - I2C2_ER
.word _IRQ35_SPI1                    @IRQ35 - SPI1
.word _IRQ36_SPI2                    @IRQ36 - SPI2
.word _IRQ37_USART1                  @IRQ37 - USART1
.word _IRQ38_USART2                  @IRQ38 - USART2
.word _IRQ39_USART3                  @IRQ39 - USART3
.word _IRQ40_EXTI15_10               @IRQ40 - EXTI15_10
.word _IRQ41_RTCAlarm                @IRQ41 - RTCAlarm
.word _IRQ42_USBWakeup               @IRQ42 - USBWakeup
.word _IRQ43_TIM8_BRK                @IRQ43 - TIM8_BRK
.word _IRQ44_TIM8_UP                 @IRQ44 - TIM8_UP
.word _IRQ45_TIM8_TRG_COM            @IRQ45 - TIM8_TRG_COM
.word _IRQ46_TIM8_CC                 @IRQ46 - TIM8_CC
.word _IRQ47_ADC3                    @IRQ47 - ADC3
.word _IRQ48_FSMC                    @IRQ48 - FSMC
.word _IRQ49_SDIO                    @IRQ49 - SDIO
.word _IRQ50_TIM5                    @IRQ50 - TIM5
.word _IRQ51_SPI3                    @IRQ51 - SPI3
.word _IRQ52_UART4                   @IRQ52 - UART4
.word _IRQ53_UART5                   @IRQ53 - UART5
.word _IRQ54_TIM6                    @IRQ54 - TIM6
.word _IRQ55_TIM7                    @IRQ55 - TIM7
.word _IRQ56_DMA2_Channel1           @IRQ56 - DMA2_Channel1
.word _IRQ57_DMA2_Channel2           @IRQ57 - DMA2_Channel2
.word _IRQ58_DMA2_Channel3           @IRQ58 - DMA2_Channel3
.word _IRQ59_DMA2_Channel4_5           @IRQ59 - DMA2_Channel4_5
.word 0x00000000      @IRQ60
.word 0x00000000      @IRQ61
.word 0x00000000      @IRQ62
.word 0x00000000      @IRQ63
.word 0x00000000      @IRQ64
.word 0x00000000      @IRQ65
.word 0x00000000      @IRQ66
.word 0x00000000      @IRQ67

_NMI:
_HardFault:
_MemManage:
_BusFault:
_UsageFault:
_SVCall:
_DebugMonitor:
_PendSV:
_Systick:
_IRQ0_WWDG:
_IRQ1_PVD:
_IRQ2_TAMPER:
_IRQ3_RTC:
_IRQ4_FLASH:
_IRQ5_RCC:
_IRQ6_EXTI0:
_IRQ7_EXTI1:
_IRQ8_EXTI2:
_IRQ9_EXTI3:
_IRQ10_EXTI4:
_IRQ11_DMA1_Channel1:
_IRQ12_DMA1_Channel2:
_IRQ13_DMA1_Channel3:
_IRQ14_DMA1_Channel4:
_IRQ15_DMA1_Channel5:
_IRQ16_DMA1_Channel6:
_IRQ17_DMA1_Channel7:
_IRQ18_ADC1_2:
_IRQ19_USB_HP_CAN_TX:
_IRQ20_USB_LP_CAN_RX0:
_IRQ21_CAN_RX1:
_IRQ22_CAN_SCE:
_IRQ23_EXTI9_5:
_IRQ24_TIM1_BRK:
_IRQ25_TIM1_UP:
_IRQ26_TIM1_TRG_COM:
_IRQ27_TIM1_CC:
_IRQ28_TIM2:
_IRQ29_TIM3:
_IRQ30_TIM4:
_IRQ31_I2C1_EV:
_IRQ32_I2C1_ER:
_IRQ33_I2C2_EV:
_IRQ34_I2C2_ER:
_IRQ35_SPI1:
_IRQ36_SPI2:
_IRQ37_USART1:
_IRQ38_USART2:
_IRQ39_USART3:
_IRQ40_EXTI15_10:
_IRQ41_RTCAlarm:
_IRQ42_USBWakeup:
_IRQ43_TIM8_BRK:
_IRQ44_TIM8_UP:
_IRQ45_TIM8_TRG_COM:
_IRQ46_TIM8_CC:
_IRQ47_ADC3:
_IRQ48_FSMC:
_IRQ49_SDIO:
_IRQ50_TIM5:
_IRQ51_SPI3:
_IRQ52_UART4:
_IRQ53_UART5:
_IRQ54_TIM6:
_IRQ55_TIM7:
_IRQ56_DMA2_Channel1:
_IRQ57_DMA2_Channel2:
_IRQ58_DMA2_Channel3:
_IRQ59_DMA2_Channel4_5:
_RESET:
_DefaultHandler:
b _start
