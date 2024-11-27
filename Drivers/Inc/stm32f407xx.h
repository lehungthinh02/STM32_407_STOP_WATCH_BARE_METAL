/* stm32f407xx.h */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include <stdint.h>

#define __vo volatile
#define __weak __attribute__((weak))

/********************************** Processor Specific Details **********************************/

/* NVIC ISERx register Addresses */
#define NVIC_ISER0          ((__vo uint32_t*)0xE000E100)
#define NVIC_ISER1          ((__vo uint32_t*)0xE000E104)
#define NVIC_ISER2          ((__vo uint32_t*)0xE000E108)
#define NVIC_ISER3          ((__vo uint32_t*)0xE000E10C)

/* NVIC ICERx register Addresses */
#define NVIC_ICER0          ((__vo uint32_t*)0xE000E180)
#define NVIC_ICER1          ((__vo uint32_t*)0xE000E184)
#define NVIC_ICER2          ((__vo uint32_t*)0xE000E188)
#define NVIC_ICER3          ((__vo uint32_t*)0xE000E18C)

/* NVIC Priority Register Address Calculation */
#define NVIC_PR_BASE_ADDR   ((__vo uint32_t*)0xE000E400)

/* Number of priority bits implemented in Priority Register */
#define NO_PR_BITS_IMPLEMENTED  4

/* Base addresses of Flash and SRAM memories */
#define FLASH_BASEADDR      0x08000000U
#define SRAM1_BASEADDR      0x20000000U
#define SRAM2_BASEADDR      0x2001C000U
#define ROM_BASEADDR        0x1FFF0000U
#define SRAM                SRAM1_BASEADDR

/* AHBx and APBx Bus Peripheral base addresses */
#define PERIPH_BASEADDR     0x40000000U
#define APB1PERIPH_BASEADDR PERIPH_BASEADDR
#define APB2PERIPH_BASEADDR 0x40010000U
#define AHB1PERIPH_BASEADDR 0x40020000U
#define AHB2PERIPH_BASEADDR 0x50000000U

/* Base addresses of peripherals hanging on AHB1 bus */
#define GPIOA_BASEADDR      (AHB1PERIPH_BASEADDR + 0x0000)
#define GPIOB_BASEADDR      (AHB1PERIPH_BASEADDR + 0x0400)
#define GPIOC_BASEADDR      (AHB1PERIPH_BASEADDR + 0x0800)
#define GPIOD_BASEADDR      (AHB1PERIPH_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR      (AHB1PERIPH_BASEADDR + 0x1000)
#define GPIOF_BASEADDR      (AHB1PERIPH_BASEADDR + 0x1400)
#define GPIOG_BASEADDR      (AHB1PERIPH_BASEADDR + 0x1800)
#define GPIOH_BASEADDR      (AHB1PERIPH_BASEADDR + 0x1C00)
#define GPIOI_BASEADDR      (AHB1PERIPH_BASEADDR + 0x2000)
#define RCC_BASEADDR        (AHB1PERIPH_BASEADDR + 0x3800)

/* Base addresses of peripherals hanging on APB1 bus */
#define TIM2_BASEADDR       (APB1PERIPH_BASEADDR + 0x0000)
#define TIM3_BASEADDR       (APB1PERIPH_BASEADDR + 0x0400)
#define TIM4_BASEADDR       (APB1PERIPH_BASEADDR + 0x0800)
#define TIM5_BASEADDR       (APB1PERIPH_BASEADDR + 0x0C00)

/* Base addresses of peripherals hanging on APB2 bus */
#define EXTI_BASEADDR       (APB2PERIPH_BASEADDR + 0x3C00)
#define SYSCFG_BASEADDR     (APB2PERIPH_BASEADDR + 0x3800)

/********************************** Peripheral register definition structures **********************************/

/* GPIO Register Definition Structure */
typedef struct
{
    __vo uint32_t MODER;    /*!< GPIO port mode register,                   Address offset: 0x00 */
    __vo uint32_t OTYPER;   /*!< GPIO port output type register,            Address offset: 0x04 */
    __vo uint32_t OSPEEDR;  /*!< GPIO port output speed register,           Address offset: 0x08 */
    __vo uint32_t PUPDR;    /*!< GPIO port pull-up/pull-down register,      Address offset: 0x0C */
    __vo uint32_t IDR;      /*!< GPIO port input data register,             Address offset: 0x10 */
    __vo uint32_t ODR;      /*!< GPIO port output data register,            Address offset: 0x14 */
    __vo uint32_t BSRR;     /*!< GPIO port bit set/reset register,          Address offset: 0x18 */
    __vo uint32_t LCKR;     /*!< GPIO port configuration lock register,     Address offset: 0x1C */
    __vo uint32_t AFR[2];   /*!< GPIO alternate function registers,         Address offset: 0x20-0x24 */
} GPIO_RegDef_t;

/* RCC Register Definition Structure */
typedef struct
{
    __vo uint32_t CR;
    __vo uint32_t PLLCFGR;
    __vo uint32_t CFGR;
    __vo uint32_t CIR;
    __vo uint32_t AHB1RSTR;
    __vo uint32_t AHB2RSTR;
    __vo uint32_t AHB3RSTR;
    uint32_t      RESERVED0;
    __vo uint32_t APB1RSTR;
    __vo uint32_t APB2RSTR;
    uint32_t      RESERVED1[2];
    __vo uint32_t AHB1ENR;
    __vo uint32_t AHB2ENR;
    __vo uint32_t AHB3ENR;
    uint32_t      RESERVED2;
    __vo uint32_t APB1ENR;
    __vo uint32_t APB2ENR;
    uint32_t      RESERVED3[2];
    __vo uint32_t AHB1LPENR;
    __vo uint32_t AHB2LPENR;
    __vo uint32_t AHB3LPENR;
    uint32_t      RESERVED4;
    __vo uint32_t APB1LPENR;
    __vo uint32_t APB2LPENR;
    uint32_t      RESERVED5[2];
    __vo uint32_t BDCR;
    __vo uint32_t CSR;
    uint32_t      RESERVED6[2];
    __vo uint32_t SSCGR;
    __vo uint32_t PLLI2SCFGR;
    __vo uint32_t PLLSAICFGR;
    __vo uint32_t DCKCFGR;
    __vo uint32_t CKGATENR;
    __vo uint32_t DCKCFGR2;
} RCC_RegDef_t;

/* EXTI Register Definition Structure */
typedef struct
{
    __vo uint32_t IMR;
    __vo uint32_t EMR;
    __vo uint32_t RTSR;
    __vo uint32_t FTSR;
    __vo uint32_t SWIER;
    __vo uint32_t PR;
} EXTI_RegDef_t;

/* SYSCFG Register Definition Structure */
typedef struct
{
    __vo uint32_t MEMRMP;
    __vo uint32_t PMC;
    __vo uint32_t EXTICR[4];
    uint32_t      RESERVED1[2];
    __vo uint32_t CMPCR;
    uint32_t      RESERVED2[2];
    __vo uint32_t CFGR;
} SYSCFG_RegDef_t;

/* General Purpose Timer Register Definition Structure */
typedef struct
{
    __vo uint32_t CR1;
    __vo uint32_t CR2;
    __vo uint32_t SMCR;
    __vo uint32_t DIER;
    __vo uint32_t SR;
    __vo uint32_t EGR;
    __vo uint32_t CCMR1;
    __vo uint32_t CCMR2;
    __vo uint32_t CCER;
    __vo uint32_t CNT;
    __vo uint32_t PSC;
    __vo uint32_t ARR;
    uint32_t      RESERVED1;
    __vo uint32_t CCR1;
    __vo uint32_t CCR2;
    __vo uint32_t CCR3;
    __vo uint32_t CCR4;
    uint32_t      RESERVED2;
    __vo uint32_t DCR;
    __vo uint32_t DMAR;
} GP_TIM_RegDef_t;

/* Peripheral Definitions (Peripheral base addresses typecasted to xxx_RegDef_t) */
#define GPIOA               ((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB               ((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC               ((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD               ((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE               ((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF               ((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG               ((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH               ((GPIO_RegDef_t*)GPIOH_BASEADDR)
#define GPIOI               ((GPIO_RegDef_t*)GPIOI_BASEADDR)

#define RCC                 ((RCC_RegDef_t*)RCC_BASEADDR)
#define EXTI                ((EXTI_RegDef_t*)EXTI_BASEADDR)
#define SYSCFG              ((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)

#define TIM2                ((GP_TIM_RegDef_t*)TIM2_BASEADDR)
#define TIM3                ((GP_TIM_RegDef_t*)TIM3_BASEADDR)
#define TIM4                ((GP_TIM_RegDef_t*)TIM4_BASEADDR)
#define TIM5                ((GP_TIM_RegDef_t*)TIM5_BASEADDR)

/* Clock Enable Macros for GPIOx peripherals */
#define GPIOA_PCLK_EN()     (RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN()     (RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN()     (RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN()     (RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN()     (RCC->AHB1ENR |= (1 << 4))
#define GPIOF_PCLK_EN()     (RCC->AHB1ENR |= (1 << 5))
#define GPIOG_PCLK_EN()     (RCC->AHB1ENR |= (1 << 6))
#define GPIOH_PCLK_EN()     (RCC->AHB1ENR |= (1 << 7))
#define GPIOI_PCLK_EN()     (RCC->AHB1ENR |= (1 << 8))

/* Clock Enable Macros for SYSCFG peripheral */
#define SYSCFG_PCLK_EN()    (RCC->APB2ENR |= (1 << 14))

/* Clock Enable Macros for TIMx peripherals */
#define TIM2_PCLK_EN()      (RCC->APB1ENR |= (1 << 0))
#define TIM3_PCLK_EN()      (RCC->APB1ENR |= (1 << 1))
#define TIM4_PCLK_EN()      (RCC->APB1ENR |= (1 << 2))
#define TIM5_PCLK_EN()      (RCC->APB1ENR |= (1 << 3))

/* Clock Disable Macros for GPIOx peripherals */
#define GPIOA_PCLK_DI()     (RCC->AHB1ENR &= ~(1 << 0))
#define GPIOB_PCLK_DI()     (RCC->AHB1ENR &= ~(1 << 1))
#define GPIOC_PCLK_DI()     (RCC->AHB1ENR &= ~(1 << 2))
#define GPIOD_PCLK_DI()     (RCC->AHB1ENR &= ~(1 << 3))
#define GPIOE_PCLK_DI()     (RCC->AHB1ENR &= ~(1 << 4))
#define GPIOF_PCLK_DI()     (RCC->AHB1ENR &= ~(1 << 5))
#define GPIOG_PCLK_DI()     (RCC->AHB1ENR &= ~(1 << 6))
#define GPIOH_PCLK_DI()     (RCC->AHB1ENR &= ~(1 << 7))
#define GPIOI_PCLK_DI()     (RCC->AHB1ENR &= ~(1 << 8))

/* Clock Disable Macros for TIMx peripherals */
#define TIM2_PCLK_DI()      (RCC->APB1ENR &= ~(1 << 0))
#define TIM3_PCLK_DI()      (RCC->APB1ENR &= ~(1 << 1))
#define TIM4_PCLK_DI()      (RCC->APB1ENR &= ~(1 << 2))
#define TIM5_PCLK_DI()      (RCC->APB1ENR &= ~(1 << 3))

/* Macros to reset GPIOx peripherals */
#define GPIOA_REG_RESET()   do { (RCC->AHB1RSTR |= (1 << 0));  (RCC->AHB1RSTR &= ~(1 << 0)); } while(0)
#define GPIOB_REG_RESET()   do { (RCC->AHB1RSTR |= (1 << 1));  (RCC->AHB1RSTR &= ~(1 << 1)); } while(0)
#define GPIOC_REG_RESET()   do { (RCC->AHB1RSTR |= (1 << 2));  (RCC->AHB1RSTR &= ~(1 << 2)); } while(0)
#define GPIOD_REG_RESET()   do { (RCC->AHB1RSTR |= (1 << 3));  (RCC->AHB1RSTR &= ~(1 << 3)); } while(0)
#define GPIOE_REG_RESET()   do { (RCC->AHB1RSTR |= (1 << 4));  (RCC->AHB1RSTR &= ~(1 << 4)); } while(0)
#define GPIOF_REG_RESET()   do { (RCC->AHB1RSTR |= (1 << 5));  (RCC->AHB1RSTR &= ~(1 << 5)); } while(0)
#define GPIOG_REG_RESET()   do { (RCC->AHB1RSTR |= (1 << 6));  (RCC->AHB1RSTR &= ~(1 << 6)); } while(0)
#define GPIOH_REG_RESET()   do { (RCC->AHB1RSTR |= (1 << 7));  (RCC->AHB1RSTR &= ~(1 << 7)); } while(0)
#define GPIOI_REG_RESET()   do { (RCC->AHB1RSTR |= (1 << 8));  (RCC->AHB1RSTR &= ~(1 << 8)); } while(0)

/* Macros to get the port code for given GPIO base address */
#define GPIO_BASEADDR_TO_CODE(x)      ((x == GPIOA) ? 0 : \
                                       (x == GPIOB) ? 1 : \
                                       (x == GPIOC) ? 2 : \
                                       (x == GPIOD) ? 3 : \
                                       (x == GPIOE) ? 4 : \
                                       (x == GPIOF) ? 5 : \
                                       (x == GPIOG) ? 6 : \
                                       (x == GPIOH) ? 7 : \
                                       (x == GPIOI) ? 8 : 0)

/* IRQ(Interrupt Request) Numbers of STM32F407x MCU */
#define IRQ_NO_EXTI0        6
#define IRQ_NO_EXTI1        7
#define IRQ_NO_EXTI2        8
#define IRQ_NO_EXTI3        9
#define IRQ_NO_EXTI4        10
#define IRQ_NO_EXTI9_5      23
#define IRQ_NO_TIM2         28
#define IRQ_NO_TIM3         29
#define IRQ_NO_TIM4         30
#define IRQ_NO_TIM5         50
#define IRQ_NO_EXTI15_10    40

/* Macros for all possible priority levels */
#define NVIC_IRQ_PRI0       0
#define NVIC_IRQ_PRI15      15

/* Generic Macros */
#define ENABLE              1
#define DISABLE             0
#define SET                 ENABLE
#define RESET               DISABLE
#define GPIO_PIN_SET        SET
#define GPIO_PIN_RESET      RESET
#define FLAG_RESET          RESET
#define FLAG_SET            SET

/* Include driver header files */
#include  "stm32f4xx_gpio.h"
#include "stm32f4xx_gp_timer.h"

#endif /* INC_STM32F407XX_H_ */
