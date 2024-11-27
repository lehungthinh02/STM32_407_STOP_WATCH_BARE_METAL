/* stm32f4xx_gp_timer.h */

#ifndef INC_STM32F4XX_GP_TIMER_H_
#define INC_STM32F4XX_GP_TIMER_H_

#include "stm32f407xx.h"

/* Timer Modes */
#define GP_TIM_MODE_BASIC       0
#define GP_TIM_MODE_PWM         1

/* Timer Clock Sources */
#define GP_TIM_CK_INT           0

/* Timer Counter Modes */
#define GP_TIM_CNT_MODE_UP      0
#define GP_TIM_CNT_MODE_DOWN    1

/* Timer Config Structure */
typedef struct
{
    uint8_t TimerMode;         // GP_TIM_MODE_BASIC or GP_TIM_MODE_PWM
    uint8_t ClockSource;       // GP_TIM_CK_INT
    uint8_t CounterMode;       // GP_TIM_CNT_MODE_UP or GP_TIM_CNT_MODE_DOWN
    uint16_t Prescaler;
    uint32_t Autorealod;
} GP_TIM_Config_t;

/* Timer Handle Structure */
typedef struct
{
    GP_TIM_RegDef_t *pTIMx;    // Pointer to TIMx base address
    GP_TIM_Config_t TIMConfig;
} GP_TIM_Handle_t;

/* Bit position definitions for TIMx_CR1 register */
#define GP_TIM_CR1_CEN          0
#define GP_TIM_CR1_DIR          4
#define GP_TIM_CR1_ARPE         7

/* Bit position definitions for TIMx_DIER register */
#define GP_TIM_DIER_UIE         0

/* Bit position definitions for TIMx_SR register */
#define GP_TIM_SR_UIF           0

/* Bit position definitions for TIMx_EGR register */
#define GP_TIM_EGR_UG           0

/* APIs supported by this driver */
void GP_TIM_PeriClockControl(GP_TIM_RegDef_t* pTIMx, uint8_t EnorDi);
void GP_TIM_Init(GP_TIM_Handle_t* pGP_TIM_Handle);
void GP_TIM_Start(GP_TIM_Handle_t* pGP_TIM_Handle);
void GP_TIM_Stop(GP_TIM_Handle_t* pGP_TIM_Handle);

#endif /* INC_STM32F4XX_GP_TIMER_H_ */
