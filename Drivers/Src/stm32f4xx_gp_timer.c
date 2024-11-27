/* stm32f4xx_gp_timer.c */

#include "stm32f4xx_gp_timer.h"

/*********************************************************************
 * @fn      		  - GP_TIM_PeriClockControl
 *
 * @brief             - This function enables or disables peripheral clock for the given TIMx
 *
 * @param[in]         - base address of the TIMx peripheral
 * @param[in]         - ENABLE or DISABLE macros
 *
 * @return            - none
 *
 * @Note              - none
 */
void GP_TIM_PeriClockControl(GP_TIM_RegDef_t* pTIMx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if (pTIMx == TIM2)
		{
			TIM2_PCLK_EN();
		}
		else if (pTIMx == TIM3)
		{
			TIM3_PCLK_EN();
		}
		else if (pTIMx == TIM4)
		{
			TIM4_PCLK_EN();
		}
		else if (pTIMx == TIM5)
		{
			TIM5_PCLK_EN();
		}
	}
	else
	{
		if (pTIMx == TIM2)
		{
			TIM2_PCLK_DI();
		}
		else if (pTIMx == TIM3)
		{
			TIM3_PCLK_DI();
		}
		else if (pTIMx == TIM4)
		{
			TIM4_PCLK_DI();
		}
		else if (pTIMx == TIM5)
		{
			TIM5_PCLK_DI();
		}
	}
}

/*********************************************************************
 * @fn      		  - GP_TIM_Init
 *
 * @brief             - This function initializes the timer
 *
 * @param[in]         - Handle structure for TIMx
 *
 * @return            - none
 *
 * @Note              - none
 */
void GP_TIM_Init(GP_TIM_Handle_t* pGP_TIM_Handle)
{
	// Enable the peripheral clock
	GP_TIM_PeriClockControl(pGP_TIM_Handle->pTIMx, ENABLE);

	// Select clock source
	if (pGP_TIM_Handle->TIMConfig.ClockSource == GP_TIM_CK_INT)
	{
		// Internal Clock
		pGP_TIM_Handle->pTIMx->SMCR &= ~(0x7 << 0); // SMS = 000: Internal Clock
	}
	else
	{
		// Implement code for external clock if needed
	}

	// Set prescaler
	pGP_TIM_Handle->pTIMx->PSC = pGP_TIM_Handle->TIMConfig.Prescaler;

	// Set Auto-reload register (ARR)
	pGP_TIM_Handle->pTIMx->ARR = pGP_TIM_Handle->TIMConfig.Autorealod;

	// Select timer mode
	if (pGP_TIM_Handle->TIMConfig.TimerMode == GP_TIM_MODE_PWM)
	{
		// PWM mode (code remains as before)
	}
	else if (pGP_TIM_Handle->TIMConfig.TimerMode == GP_TIM_MODE_BASIC)
	{
		// Basic Timer mode

		// Enable update interrupt (UIE)
		pGP_TIM_Handle->pTIMx->DIER |= (1 << GP_TIM_DIER_UIE);

		// Configure counter mode
		if (pGP_TIM_Handle->TIMConfig.CounterMode == GP_TIM_CNT_MODE_UP)
		{
			pGP_TIM_Handle->pTIMx->CR1 &= ~(1 << GP_TIM_CR1_DIR); // DIR = 0: Upcounter
		}
		else if (pGP_TIM_Handle->TIMConfig.CounterMode == GP_TIM_CNT_MODE_DOWN)
		{
			pGP_TIM_Handle->pTIMx->CR1 |= (1 << GP_TIM_CR1_DIR);  // DIR = 1: Downcounter
		}

		// Enable Auto-reload preload (ARPE)
		pGP_TIM_Handle->pTIMx->CR1 |= (1 << GP_TIM_CR1_ARPE);

		// Generate an update event to reload the prescaler value immediately
		pGP_TIM_Handle->pTIMx->EGR |= (1 << GP_TIM_EGR_UG);
	}
}

/*********************************************************************
 * @fn      		  - GP_TIM_Start
 *
 * @brief             - This function starts the timer
 *
 * @param[in]         - Handle structure for TIMx
 *
 * @return            - none
 *
 * @Note              - none
 */
void GP_TIM_Start(GP_TIM_Handle_t* pGP_TIM_Handle)
{
	// Set TIMx_CR1 CEN bit
	pGP_TIM_Handle->pTIMx->CR1 |= (1 << GP_TIM_CR1_CEN);
}

/*********************************************************************
 * @fn      		  - GP_TIM_Stop
 *
 * @brief             - This function stops the timer
 *
 * @param[in]         - Handle structure for TIMx
 *
 * @return            - none
 *
 * @Note              - none
 */
void GP_TIM_Stop(GP_TIM_Handle_t* pGP_TIM_Handle)
{
	// Reset TIMx_CR1 CEN bit
	pGP_TIM_Handle->pTIMx->CR1 &= ~(1 << GP_TIM_CR1_CEN);
}
