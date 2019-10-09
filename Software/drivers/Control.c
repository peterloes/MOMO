/***************************************************************************//**
 * @file
 * @brief	Sequence Control
 * @author	Ralf Gerhauser
 * @version	2017-01-25
 *
 * This is the automatic sequence control module.  It controls peripheral
 * units like the camera or the shutter of the feeder.
 *
 ****************************************************************************//*
Revision History:
2017-05-02,rage	- Renamed CAM1_DURATION to CAM_DURATION, DFLT_CAM1_DURATION to
		  DFLT_CAM_DURATION, and dfltCam1Duration to dfltCamDuration.
		- ControlInit: Added CONTROL_INIT structure to specify the
		  power output for the camera (or PWR_OUT_NONE if no camera).
		- PWR_OUT_DEF contains the bit address and logical enable level
		  for all power output pins.
		- ExtIntEnableAll() is called when switching the feeder on again
		  to consider the current state of all EXTI interrupt inputs.
		- Implemented PowerOutput() and IsPowerOutputOn().
		- Removed CAM1 and CAM2 routines.
2017-01-25,rage	Initial version.
*/

/*=============================== Header Files ===============================*/

#include "em_gpio.h"
#include "ExtInt.h"
#include "Logging.h"
#include "LightBarrier.h"
#include "AlarmClock.h"
#include "RFID.h"
#include "CfgData.h"
#include "Control.h"

/*=============================== Definitions ================================*/

    /*!@name Hardware Configuration: Pulse output to open shutter */
//@{
#define SERVO_AUF_PORT		gpioPortE	//!< Port for servo control
#define SERVO_AUF_PIN		9		//!< Pulse Pin: 0=active, 1=idle
    //! Set level of the power enable pin
#define SERVO_AUF	IO_Bit(GPIO->P[SERVO_AUF_PORT].DOUT, SERVO_AUF_PIN)
//@}

    /*!@name Hardware Configuration: Pulse output to close shutter */
//@{
#define SERVO_ZU_PORT		gpioPortE	//!< Port for servo control
#define SERVO_ZU_PIN		10		//!< Pulse Pin: 0=active, 1=idle
    //! Set level of the power enable pin
#define SERVO_ZU	IO_Bit(GPIO->P[SERVO_ZU_PORT].DOUT, SERVO_ZU_PIN)
//@}

    /*!@brief Structure to define power outputs. */
typedef struct
{
    __IO uint32_t *BitBandAddr;	// Bit band address of GPIO power enable pin
    bool	   HighActive;	// true: The power enable pin is high-active
} PWR_OUT_DEF;

    /*!@brief Macro to calculate a GPIO bit address for a port and pin. */
#define GPIO_BIT_ADDR(port, pin)					\
	IO_BIT_ADDR((&(GPIO->P[(port)].DOUT)), (pin))

/*!@brief Macro to extract the port number from a GPIO bit address.  */
#define GPIO_BIT_ADDR_TO_PORT(bitAddr)		(GPIO_Port_TypeDef)	\
	(((((uint32_t)(bitAddr) - BITBAND_PER_BASE) >> 5)		\
	 + PER_MEM_BASE - GPIO_BASE) / sizeof(GPIO_P_TypeDef))

    /*!@brief Macro to extract the pin number from a GPIO bit address.  */
#define GPIO_BIT_ADDR_TO_PIN(bitAddr)					\
	(((uint32_t)(bitAddr) >> 2) & 0x1F)

/*================================ Local Data ================================*/

    /*!@brief Local pointer to control configuration. */
static const CONTROL_INIT *l_pControlCfg;

    /*!@brief Power output port and pin assignment. */
static const PWR_OUT_DEF  l_PwrOutDef[NUM_PWR_OUT] =
{   //   BitBandAddr,              HighActive
    { GPIO_BIT_ADDR(gpioPortA,  3), true },	// PWR_OUT_UA1
    { GPIO_BIT_ADDR(gpioPortA,  4), true },	// PWR_OUT_UA2
    { GPIO_BIT_ADDR(gpioPortC,  8), true },	// PWR_OUT_VDD_RFID1
    { GPIO_BIT_ADDR(gpioPortC,  9), true },	// PWR_OUT_VDD_RFID2
    { GPIO_BIT_ADDR(gpioPortC, 10), true },	// PWR_OUT_VDD_RFID3
    { GPIO_BIT_ADDR(gpioPortA,  6), true },	// PWR_OUT_RFID_GND_LB
};

    /*!@brief Time when feeder should be switched on, set by ON_TIME. */
static ALARM_TIME	l_OnTime  = { 06, 00 };

    /*!@brief Time when feeder should be switched off, set by OFF_TIME. */
static ALARM_TIME	l_OffTime = { 20, 30 };

    /*!@brief Default duration of the camera, set by CAM_DURATION. */
static int32_t		l_dfltCamDuration = DFLT_CAM_DURATION;

    /*!@brief Actual duration of the camera, set by ID. */
static int32_t		l_CamDuration = DFLT_CAM_DURATION;

    /*!@brief Default keep open duration of shutter, set by KEEP_OPEN. */
static int32_t		l_dfltKeepOpen = DFLT_KEEP_OPEN_DURATION;

    /*!@brief Actual keep open duration of shutter, set by ID. */
static int32_t		l_KeepOpen = DFLT_KEEP_OPEN_DURATION;

    /*!@brief Default keep closed duration of shutter, set by KEEP_CLOSED. */
static int32_t		l_dfltKeepClosed = DFLT_KEEP_CLOSED_DURATION;

    /*!@brief Actual keep closed duration of shutter, set by ID. */
static int32_t		l_KeepClosed = DFLT_KEEP_CLOSED_DURATION;

    /*!@brief List of configuration variables.
     * Alarm times, i.e. @ref CFG_VAR_TYPE_TIME must be defined first, because
     * the array index is used to specify the alarm number \<alarmNum\>,
     * starting with @ref ALARM_ON_TIME, when calling AlarmSet().
     */
static const CFG_VAR_DEF l_CfgVarList[] =
{
    { "ON_TIME",		CFG_VAR_TYPE_TIME,	&l_OnTime	      },
    { "OFF_TIME",		CFG_VAR_TYPE_TIME,	&l_OffTime	      },
    { "LB_FILTER_DURATION",	CFG_VAR_TYPE_INTEGER,	&g_LB_FilterDuration  },
    { "RFID_PWR_OFF_TIMEOUT",	CFG_VAR_TYPE_INT_GE_1,	&g_RFID_PwrOffTimeout },
    { "RFID_DETECT_TIMEOUT",	CFG_VAR_TYPE_INT_GE_1,	&g_RFID_DetectTimeout },
    { "KEEP_OPEN",		CFG_VAR_TYPE_DURATION,	&l_dfltKeepOpen       },
    { "KEEP_CLOSED",		CFG_VAR_TYPE_DURATION,	&l_dfltKeepClosed     },
    { "CAM_DURATION",		CFG_VAR_TYPE_INT_GE_1,	&l_dfltCamDuration    },
    { "ID",			CFG_VAR_TYPE_ID,	NULL		      },
    {  NULL,			END_CFG_VAR_TYPE,	NULL		      }
};

    /*!@brief Timer handle to keep the shutter open or closed for a while. */
static volatile TIM_HDL	l_hdlShutter = NONE;

    /*!@brief Timer handle for switching the camera off after a time. */
static volatile TIM_HDL	l_hdlCAM_Off = NONE;

    /*!@brief Flag if feeder should be switched on. */
static volatile bool	l_flgFeederOn = true;	// switch on after power-up

    /*!@brief Current state of the feeder: true means ON, false means OFF. */
static volatile bool	l_flgFeederIsOn;	// is false for default

    /*!@brief Flag if shutter should be opened. */
static volatile bool	l_flgOpenShutter = true;  // open shutter after power-up

    /*!@brief Flag if shutter is currently open. */
static volatile bool	l_flgShutterIsOpen;


/*=========================== Forward Declarations ===========================*/

static void	FeederOn (int alarmNum);
static void	FeederOff (int alarmNum);
static void	ShutterAction (TIM_HDL hdl);
static void	OpenShutter (void);
static void	CloseShutter (void);
void		CameraPowerDisable (void);


/***************************************************************************//**
 *
 * @brief	Initialize control module
 *
 * This routine initializes the sequence control module.
 *
 * @param[in] pInitStruct
 *	Address of an initialization structure of type CONTROL_INIT that defines
 *	the power output to switch the camera on or off.
 *
 * @note
 *	Parameter <b>pInitStruct</b> must point to a persistent data structure,
 *	i.e. this must be valid over the whole life time of the program.
 *
 ******************************************************************************/
void	ControlInit (const CONTROL_INIT *pInitStruct)
{
int	i;

    /* Parameter check */
    EFM_ASSERT(pInitStruct != NULL);

    /* Save configuration */
    l_pControlCfg = pInitStruct;

    /* Introduce variable list to configuration data module */
    CfgDataInit (l_CfgVarList);

    /* Initialize GPIOs */
    GPIO_PinModeSet (SERVO_AUF_PORT,  SERVO_AUF_PIN,  gpioModePushPull, 1);
    GPIO_PinModeSet (SERVO_ZU_PORT,   SERVO_ZU_PIN,   gpioModePushPull, 1);

    /* Initialize power output enable pins */
    for (i = 0;  i < NUM_PWR_OUT;  i++)
    {
	/* Configure Power Enable Pin, switch it OFF per default */
	GPIO_PinModeSet (GPIO_BIT_ADDR_TO_PORT(l_PwrOutDef[i].BitBandAddr),
			 GPIO_BIT_ADDR_TO_PIN (l_PwrOutDef[i].BitBandAddr),
			 gpioModePushPull, l_PwrOutDef[i].HighActive ? 0:1);
    }

    /* Get a timer handle to keep the shutter open or closed for a while */
    if (l_hdlShutter == NONE)
	l_hdlShutter = sTimerCreate (ShutterAction);

    /* Get a timer handle to switch the camera off after a time */
    if (l_hdlCAM_Off == NONE)
	l_hdlCAM_Off = sTimerCreate ((TIMER_FCT)CameraPowerDisable);

    /* Set up alarm default times when feeder should be switched on or off */
    AlarmAction (ALARM_ON_TIME, FeederOn);
    AlarmSet (ALARM_ON_TIME, l_OnTime.Hour, l_OnTime.Minute);
    AlarmEnable (ALARM_ON_TIME);

    AlarmAction (ALARM_OFF_TIME, FeederOff);
    AlarmSet (ALARM_OFF_TIME, l_OffTime.Hour, l_OffTime.Minute);
    AlarmEnable (ALARM_OFF_TIME);
}


/***************************************************************************//**
 *
 * @brief	Control
 *
 * This routine is periodically called by the main execution loop to perform
 * miscellaneous control tasks.
 *
 ******************************************************************************/
void	Control (void)
{
    /* feeder control */
    if (l_flgFeederOn)
    {
	/* Feeder should be switched ON */
	if (! l_flgFeederIsOn)
	{
#ifdef LOGGING
	    /* Generate Log Message */
	    Log ("Feeder is switched ON");
#endif
	    /* Deactivate timer */
	    if (l_hdlShutter != NONE)
		sTimerCancel (l_hdlShutter);

	    /* Open shutter */
	    OpenShutter();

	    l_flgFeederIsOn = true;

	    /* Replay external interrupts to consider new power state */
	    ExtIntReplay();
	}
    }
    else
    {
	/* Feeder should be switched OFF */
	if (l_flgFeederIsOn)
	{
	    /* Deactivate timer */
	    if (l_hdlShutter != NONE)
		sTimerCancel (l_hdlShutter);

	    /* Close shutter */
	    CloseShutter();

	    /* Switch off the camera */
	    CameraDisable();

	    /* Switch off RFID reader */
	    RFID_Disable();

	    l_flgFeederIsOn = false;

	    /* Replay external interrupts to consider new power state */
	    ExtIntReplay();

#ifdef LOGGING
	    /* Generate Log Message */
	    Log ("Feeder is switched off");
#endif
	}
    }

    /* shutter control */
    if (l_flgOpenShutter)
	OpenShutter();
    else
	CloseShutter();
}


/***************************************************************************//**
 *
 * @brief	Switch feeder on
 *
 * This routine is called by the RTC_IRQHandler() when @ref ALARM_ON_TIME is
 * reached.  It switches the feeder on, i.e. the shutter opens and the light
 * barrier will activate RFID reader and camera when it is triggered.
 *
 ******************************************************************************/
static void	FeederOn (int alarmNum)
{
    (void) alarmNum;		// suppress compiler warning "unused parameter"

    /* Set flag */
    l_flgFeederOn = true;

    g_flgIRQ = true;	// keep on running
}


/***************************************************************************//**
 *
 * @brief	Switch feeder off
 *
 * This routine is called by the RTC_IRQHandler() when @ref ALARM_OFF_TIME is
 * reached.  It switches the feeder off, i.e. the shutter will be closed and
 * information from the light barrier is ignored.
 *
 ******************************************************************************/
static void	FeederOff (int alarmNum)
{
    (void) alarmNum;		// suppress compiler warning "unused parameter"

    /* Clear flag */
    l_flgFeederOn = false;

    g_flgIRQ = true;	// keep on running
}


/***************************************************************************//**
 *
 * @brief	Determine if feeder is on
 *
 * The feeder can be switched on and off by setting alarm times.  This routine
 * is used to determine if the feeder is currently on.
 *
 * @return
 * 	The value <i>true</i> if the feeder is on, <i>false</i> if not.
 *
 ******************************************************************************/
bool	IsFeederOn (void)
{
    return l_flgFeederIsOn;
}


/***************************************************************************//**
 *
 * @brief	Inform the control module about a new transponder ID
 *
 * This routine must be called to inform the control module about a new
 * transponder ID.
 *
 * @warning
 * This function calls a blocking delay routine, therefore it must not be
 * called from interrupt context!
 *
 ******************************************************************************/
void	ControlUpdateID (char *transponderID)
{
char	 line[120];
char	*pStr;
ID_PARM	*pID;

    pStr = line;

    pID = CfgLookupID (transponderID);
    if (pID == NULL)
    {
	/* specified ID not found, look for an "ANY" entry */
	pID = CfgLookupID ("ANY");
	if (pID == NULL)
	{
	    /* no "ANY" entry defined, treat ID as "UNKNOWN" */
	    pID = CfgLookupID ("UNKNOWN");
	    if (pID == NULL)
	    {
		/* even no "UNKNOWN" entry exists - abort */
		Log ("Transponder: %s not found - aborting", transponderID);
		return;
	    }
	    else
	    {
		pStr += sprintf (pStr, "Transponder: %s not found -"
				 " using UNKNOWN", transponderID);
	    }
	}
	else
	{
	    pStr += sprintf (pStr, "Transponder: %s not found -"
			     " using ANY", transponderID);
	}
    }
    else
    {
	pStr += sprintf (pStr, "Transponder: %s", transponderID);
    }

    /* prepare the associated variables */
    l_KeepOpen     = (pID->KeepOpen == DUR_INVALID	? l_dfltKeepOpen
							: pID->KeepOpen);
    l_KeepClosed   = (pID->KeepClosed == DUR_INVALID	? l_dfltKeepClosed
							: pID->KeepClosed);
    l_CamDuration = (pID->CamDuration == DUR_INVALID	? l_dfltCamDuration
							: pID->CamDuration);
    /* append current parameters to ID */
    pStr += sprintf (pStr, l_KeepOpen == DUR_ALWAYS ? ":A":":%ld", l_KeepOpen);
    pStr += sprintf (pStr, l_KeepClosed==DUR_ALWAYS ? ":A":":%ld", l_KeepClosed);
    sprintf (pStr, ":%ld", l_CamDuration);
    Log (line);

    /* open or close the shutter (may already be done) */
    if (l_KeepOpen > 0  ||  l_KeepOpen == DUR_ALWAYS)
    {
	/*
	 * A KEEP_OPEN value of 1..n or "A" means "Open Shutter"
	 */
	OpenShutter();

	/* start KeepOpen timer */
	if (l_hdlShutter != NONE)
	{
	    if (l_KeepOpen > 0)
		sTimerStart (l_hdlShutter, l_KeepOpen);
	    else
		sTimerCancel (l_hdlShutter);
	}
    }
    else
    {
	/*
	 * A KEEP_OPEN value of 0 means "Close Shutter"
	 */
	/* be sure to cancel KeepOpen timer */
	if (l_hdlShutter != NONE)
	    sTimerCancel (l_hdlShutter);

	/* immediately close shutter */
	CloseShutter();

	/* it follows a KEEP_CLOSED duration, except in case of DUR_ALWAYS */
	if (l_KeepClosed > 0)
	{
	    /* start KeepClosed timer */
	    sTimerStart (l_hdlShutter, l_KeepClosed);
	}
    }
}


/***************************************************************************//**
 *
 * @brief	Shutter action
 *
 * This routine is called after the programmed KEEP_OPEN or KEEP_CLOSED
 * duration to initiate a close or open action for the shutter.
 *
 ******************************************************************************/
static void	ShutterAction (TIM_HDL hdl)
{
    (void) hdl;		// suppress compiler warning "unused parameter"

    /* determine current state */
    if (l_flgOpenShutter)
    {
	/* shutter is open now and should be closed */
	l_flgOpenShutter = false;

	/* it follows a KEEP_CLOSED duration, except in case of DUR_ALWAYS */
	if (l_KeepClosed > 0)
	{
	    /* start KeepClosed timer */
	    sTimerStart (l_hdlShutter, l_KeepClosed);
	}
    }
    else
    {
	/* shutter is closed and should now be opened again */
	l_flgOpenShutter = true;
    }
}


/***************************************************************************//**
 *
 * @brief	Open shutter
 *
 * This routine initiates to open the shutter of the feeder.  This is done by
 * generating a negative pulse on the SERVO_AUF output, which is connected to
 * the servo module.
 *
 * @warning
 * This function calls a blocking delay routine, therefore it must not be
 * called from interrupt context!
 *
 ******************************************************************************/
static void	OpenShutter (void)
{
    l_flgOpenShutter = true;

    if (! l_flgShutterIsOpen)
    {
	l_flgShutterIsOpen = true;

#ifdef LOGGING
	/* Generate Log Message */
	Log ("Shutter will be opened");
#endif
	SERVO_AUF = 0;	// generate low-active pulse
	msDelay(2);
	SERVO_AUF = 1;	// back to idle
    }
}


/***************************************************************************//**
 *
 * @brief	Close shutter
 *
 * This routine initiates to close the shutter of the feeder.  This is done by
 * generating a negative pulse on the SERVO_ZU output, which is connected to
 * the servo module.
 *
 * @warning
 * This function calls a blocking delay routine, therefore it must not be
 * called from interrupt context!
 *
 ******************************************************************************/
static void	CloseShutter (void)
{
    l_flgOpenShutter = false;

    if (l_flgShutterIsOpen)
    {
	l_flgShutterIsOpen = false;

#ifdef LOGGING
	/* Generate Log Message */
	Log ("Shutter will be closed");
#endif
	SERVO_ZU = 0;	// generate low-active pulse
	msDelay(2);
	SERVO_ZU = 1;	// back to idle
    }
}


/***************************************************************************//**
 *
 * @brief	Enable the camera
 *
 * This routine enables the camera immediately.
 *
 * @see CameraDisable(), CameraTimedDisable()
 *
 ******************************************************************************/
void	CameraEnable (void)
{
    /* Check if power output has been assigned */
    if (l_pControlCfg->CameraPwrOut == PWR_OUT_NONE)
	return;				// nothing to be done

    /* Cancel OFF-Timer */
    if (l_hdlCAM_Off != NONE)
	sTimerCancel (l_hdlCAM_Off);	// inhibit power-off of the camera

    /* Switch power ON */
    if (! IsPowerOutputOn (l_pControlCfg->CameraPwrOut))
    {
#ifdef LOGGING
	/* Generate Log Message */
	Log ("Camera is powered ON");
#endif
	PowerOutput (l_pControlCfg->CameraPwrOut, PWR_ON);
    }
}


/******************************************************************************
 *
 * @brief   Switch Camera Power OFF
 *
 * This routine is intended to disable the Power Output for the Camera.
 *
 *****************************************************************************/
void	CameraPowerDisable (void)
{
    /* Check if power output has been assigned */
    if (l_pControlCfg->CameraPwrOut == PWR_OUT_NONE)
	return;				// nothing to be done

    /* Switch Power OFF */
    if (IsPowerOutputOn (l_pControlCfg->CameraPwrOut))
    {
#ifdef LOGGING
	/* Generate Log Message */
	Log ("Camera is powered off");
#endif
	PowerOutput (l_pControlCfg->CameraPwrOut, PWR_OFF);
    }
}


/***************************************************************************//**
 *
 * @brief	Disable the camera
 *
 * This routine disables the camera immediately.
 *
 * @see CameraEnable(), CameraTimedDisable()
 *
 ******************************************************************************/
void	CameraDisable (void)
{
    /* Cancel OFF-Timer */
    if (l_hdlCAM_Off != NONE)
	sTimerCancel (l_hdlCAM_Off);	// cancel timer

    /* Switch power OFF */
    CameraPowerDisable();
}


/***************************************************************************//**
 *
 * @brief	Disable the camera after a while
 *
 * This routine disables the camera, i.e. it notifies the software to switch
 * off the assigned power output after a delay of @ref l_CamDuration seconds.
 *
 * @see CameraEnable(), CameraDisable()
 *
 ******************************************************************************/
void	CameraTimedDisable (void)
{
    /* (re-)start timer to switch OFF camera 1 after time */
    if (l_hdlCAM_Off != NONE)
	sTimerStart (l_hdlCAM_Off, l_CamDuration);
}


/***************************************************************************//**
 *
 * @brief	Power-Fail Handler for Control Module
 *
 * This function will be called in case of power-fail to switch off devices
 * that consume too much power, e.g. the camera.
 *
 ******************************************************************************/
void	ControlPowerFailHandler (void)
{
int	i;

    /* Deactivate timers */
    if (l_hdlCAM_Off != NONE)
	sTimerCancel (l_hdlCAM_Off);

    if (l_hdlShutter != NONE)
	sTimerCancel (l_hdlShutter);

#ifdef LOGGING
    /* Generate Log Message */
    Log ("Switching all power outputs OFF");
#endif

    /* Switch off all power outputs immediately */
    for (i = 0;  i < NUM_PWR_OUT;  i++)
	PowerOutput ((PWR_OUT)i, PWR_OFF);
}


/******************************************************************************
 *
 * @brief	Switch the specified power output on or off
 *
 * This routine enables or disables the specified power output.
 *
 * @param[in] output
 *	Power output to be changed.
 *
 * @param[in] enable
 *	If true (PWR_ON), the power output will be enabled, false (PWR_OFF)
 *	disables it.
 *
 *****************************************************************************/
void	PowerOutput (PWR_OUT output, bool enable)
{
    /* Parameter check */
    if (output == PWR_OUT_NONE)
	return;		// power output not assigned, nothing to be done

    if ((PWR_OUT)0 > output  ||  output >= NUM_PWR_OUT)
    {
#ifdef LOGGING
	/* Generate Error Log Message */
	LogError ("PowerOutput(%d, %d): Invalid output parameter",
		  output, enable);
#endif
	return;
    }

    /* Switch power output on or off */
    bool state = (l_PwrOutDef[output].HighActive == enable ? 1 : 0);
    *l_PwrOutDef[output].BitBandAddr = state;
}


/******************************************************************************
 *
 * @brief	Determine if the specified power output is switched on
 *
 * This routine determines the current state of a power output.
 *
 * @param[in] output
 *	Power output to be checked.
 *
 *****************************************************************************/
bool	IsPowerOutputOn (PWR_OUT output)
{
    /* Parameter check */
    if (output == PWR_OUT_NONE)
	return false;	// power output not assigned, return false (off)

    if ((PWR_OUT)0 > output  ||  output >= NUM_PWR_OUT)
    {
#ifdef LOGGING
	/* Generate Error Log Message */
	LogError ("IsPowerOutputOn(%d): Invalid output parameter", output);
#endif
	return false;
    }

    /* Determine the current state of this power output */
    unsigned int enableState = (l_PwrOutDef[output].HighActive ? 1 : 0);

    return (*l_PwrOutDef[output].BitBandAddr == enableState ? true : false);
}
