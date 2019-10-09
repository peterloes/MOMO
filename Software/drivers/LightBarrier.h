/***************************************************************************//**
 * @file
 * @brief	Header file of module LightBarrier.c
 * @author	Ralf Gerhauser
 * @version	2017-01-27
 ****************************************************************************//*
Revision History:
2017-01-27,rage	Initialize LB2 only if LB2_ENABLE is set.
2016-04-13,rage	Removed LB_TRIG_MASK (no more required by EXTI module).
2016-04-05,rage	Made variable <g_LB_ActiveMask> of type "volatile".
2014-11-26,rage	Initial version.
*/

#ifndef __INC_LightBarrier_h
#define __INC_LightBarrier_h

/*=============================== Header Files ===============================*/

#include "em_device.h"
#include "em_gpio.h"
#include "config.h"		// include project configuration parameters

/*=============================== Definitions ================================*/

/*!@brief Enable switch for second light barrier.
 * MAPRDL has 2 inputs for light barriers.  If only one is used, the second
 * input can be left not connected since it has an intern pull-up resistor
 * and therefore stays high which means inactive.  There also exists the
 * possibility to disable the second light barrier (LB2) in software by setting
 * this define to 0.  The default is 1.
 */
#define LB2_ENABLE	0

/*!@brief Here follows the definition of the two light barriers and their
 * related hardware configuration.
 */
#define LB1_PORT	gpioPortE
#define LB1_PIN		13
#define LB1_EXTI_MASK	(1 << LB1_PIN)

#if LB2_ENABLE
 #define LB2_PORT	gpioPortE
 #define LB2_PIN	14
 #define LB2_EXTI_MASK	(1 << LB2_PIN)
#else
 #define LB2_EXTI_MASK	0
#endif

/*!@brief Bit mask of all affected external interrupts (EXTIs). */
#define LB_EXTI_MASK	(LB2_EXTI_MASK | LB1_EXTI_MASK)

/*================================ Global Data ===============================*/

extern volatile uint32_t  g_LB_ActiveMask;
extern int32_t   g_LB_FilterDuration;

/*================================ Prototypes ================================*/

/* Initialize Light Barrier hardware */
void	LB_Init (void);

/* Light Barrier Handler, called from interrupt service routine */
void	LB_Handler	(int extiNum, bool extiLvl, uint32_t timeStamp);


#endif /* __INC_LightBarrier_h */
