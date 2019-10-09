/***************************************************************************//**
 * @file
 * @brief	Light Barrier Logic
 * @author	Ralf Gerhauser
 * @version	2017-01-27
 *
 * This module receives the interrupts from the light barrier logic and
 * triggers the associated actions.  It contains an initialization routine
 * to set up the GPIOs, and an interrupt handler which processes the events.
 *
 ****************************************************************************//*
Revision History:
2017-05-02,rage	- Call generic Camera routines instead of CAM1 routines.
		- Detect recovery from power-fail or feeder off state to
		  re-enable the RFID reader and the camera.
2017-01-27,rage	Initialize LB2 only if LB2_ENABLE is set.
2016-04-05,rage	Made variable <g_LB_ActiveMask> of type "volatile".
2014-11-26,rage	Initial version.
*/

/*=============================== Header Files ===============================*/

#include "em_cmu.h"
#include "LightBarrier.h"
#include "PowerFail.h"
#include "AlarmClock.h"
#include "RFID.h"
#include "Logging.h"
#include "Control.h"

/*=============================== Definitions ================================*/

    // Module Debugging
#define MOD_DEBUG	0	// set 1 to enable debugging of this module
#if ! MOD_DEBUG
    #undef  DBG_PUTC
    #undef  DBG_PUTS
    #define DBG_PUTC(ch)	// define as empty
    #define DBG_PUTS(str)
#endif

/*========================= Global Data and Routines =========================*/

    /*!@brief Bit mask what Light Barriers are active. */
volatile uint32_t  g_LB_ActiveMask;

    /*!@brief Light barrier filter duration in seconds (0=inactive). */
int32_t  g_LB_FilterDuration = 0;

/*================================ Local Data ================================*/

    /*!@brief Timer handle for the light barrier filter. */
static volatile TIM_HDL	l_hdlLB_Filter = NONE;

    /*!@brief State of the light barrier filter output. */
static volatile bool	l_LB_FilterOutput;

/*=========================== Forward Declarations ===========================*/

static void InitiatePowerOff(void);


/***************************************************************************//**
 *
 * @brief	Initialize the light barrier hardware
 *
 * This routine initializes the board-specific hardware for the light
 * barriers.  This is restricted to the GPIO set up, NVIC interrupts will
 * be configured later by calling function ExtIntInit().
 *
 ******************************************************************************/
void	LB_Init (void)
{
    /* Be sure to enable clock to GPIO (should already be done) */
    CMU_ClockEnable (cmuClock_GPIO, true);

    /*
     * Initialize GPIOs for the light barriers.  The port pins must be
     * configured for input, and connected to the external interrupt (EXTI)
     * facility.  At this stage, the interrupts are not enabled, this is
     * done later by calling ExtIntInit().
     */
    GPIO_PinModeSet (LB1_PORT, LB1_PIN, gpioModeInputPull, 1);
    GPIO_IntConfig  (LB1_PORT, LB1_PIN, false, false, false);

#if LB2_ENABLE
    GPIO_PinModeSet (LB2_PORT, LB2_PIN, gpioModeInputPull, 1);
    GPIO_IntConfig  (LB2_PORT, LB2_PIN, false, false, false);
#endif

    /* Get a timer handle for the light barrier filter */
    if (l_hdlLB_Filter == NONE)
	l_hdlLB_Filter = sTimerCreate ((TIMER_FCT)InitiatePowerOff);
}


/***************************************************************************//**
 *
 * @brief	Light Barrier handler
 *
 * This handler is called by the EXTI interrupt service routine whenever the
 * state of a light barrier changes.  It controls the power state of the RFID
 * reader, i.e. this is enabled as long as a minimum of one light barrier
 * indicates an object.
 *
 * @param[in] extiNum
 *	EXTernal Interrupt number of a light barrier.  This is identical with
 *	the pin number, e.g. @ref LB1_PIN.
 *
 * @param[in] extiLvl
 *	EXTernal Interrupt level: 0 means falling edge, logic level is now 0,
 *	1 means rising edge, logic level is now 1.  The level of the light
 *	barriers shows if the light beam could be received, i.e. 1 means normal
 *	state, level 0 indicates an object.
 *
 * @param[in] timeStamp
 *	Time stamp when the event has been received.  This parameter is not
 *	used here.
 *
 ******************************************************************************/
void	LB_Handler (int extiNum, bool extiLvl, uint32_t timeStamp)
{
uint32_t  prevActiveMask;
static bool prevPowerFail, prevFeederOn;
bool	isPowerFail, isFeederOn;


    /* Save the current state of activity mask before changing it */
    prevActiveMask = g_LB_ActiveMask;

    /* Set or clear the corresponding bit in the activity mask */
    Bit(g_LB_ActiveMask, extiNum) = ! extiLvl;

    /* Generate Log Message for new state (not in case of "replay") */
#ifdef LOGGING
    if (timeStamp != 0)
	Log ("LB%c:%s", extiNum == LB1_PIN ? '1':'2',
			extiLvl == 0 ? "ON":"off");
#endif

    /* Get current state of power-fail and feeder */
    isPowerFail = IsPowerFail();
    isFeederOn  = IsFeederOn();

#if MOD_DEBUG
    char tmpBuf[90];
    sprintf (tmpBuf, " DBG LB_Handler: prevPowerFail=%d isPowerFail=%d"
		     " prevFeederOn=%d isFeederOn=%d\n",
		     prevPowerFail, isPowerFail, prevFeederOn, isFeederOn);
    DBG_PUTS(tmpBuf);
#endif

    /* Detect if we recover from power-fail or feeder off */
    if ((  prevPowerFail  &&  ! isPowerFail)
    ||  (! prevFeederOn   &&    isFeederOn))
    {
	/*
	 * There was a power-fail, or the feeder has been switched off.
	 * Both events will disable all power outputs, so we need to
	 * reset the <l_LB_FilterOutput> flag.
	 */
	l_LB_FilterOutput = false;	// clear filter flag
    }

    /* Save states for the next time */
    prevPowerFail = isPowerFail;
    prevFeederOn  = isFeederOn;

    /* If one or more Light Barriers are active, LB Filter Output is true */
    if (g_LB_ActiveMask)
    {
	/*
	 * Check if filter state is already set and for power-fail
	 * and if the feeder is in ON mode
	 */
	if (! l_LB_FilterOutput  &&  ! isPowerFail  &&  isFeederOn)
	{
	    l_LB_FilterOutput = true;
	    DBG_PUTS(" DBG LB_Handler: setting l_LB_FilterOutput=1\n");
	    RFID_Enable();		// immediately power-on the RFID reader
	    CameraEnable();		// and also the camera
	}
    }
    else
    {
	/*
	 * Light Barriers are (all) inactive. Check if inactivity happened now.
	 */
	if (prevActiveMask != 0)
	{
	    /*
	     * LB inactivity happened at this moment.  If LB_FILTER_DURATION
	     * has been specified, (re-)start the timer.  If not, immediately
	     * initiate a timed power-off for the RFID reader and the camera.
	     */
	    if (g_LB_FilterDuration > 0)
	    {
		/* (re-)start timer to switch-off RFID reader and the camera */
		if (l_hdlLB_Filter != NONE)
		    sTimerStart (l_hdlLB_Filter, g_LB_FilterDuration);
	    }
	    else
	    {
		/* Cancel possibly running timer */
		if (l_hdlLB_Filter != NONE)
		    sTimerCancel (l_hdlLB_Filter);

		/* Immediately initiate a timed power-off */
		InitiatePowerOff();
	    }
	}
    }

    g_flgIRQ = true;		// keep on running
}

/***************************************************************************//**
 *
 * @brief	Initiate a timed power-off
 *
 * This routine is called when the light barriers turn to inactive state.
 * If the configuration variable LB_FILTER_DURATION specifies a value of more
 * zero seconds, this happens after this duration, otherwise the routine is
 * called immediately.
 *
 ******************************************************************************/
static void InitiatePowerOff(void)
{
    l_LB_FilterOutput = false;	// clear filter flag
    DBG_PUTS(" DBG InitiatePowerOff: setting l_LB_FilterOutput=0\n");

    RFID_TimedDisable();	// initiate power-off after a while
    CameraTimedDisable();	// and also the camera
}
