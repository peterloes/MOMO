/***************************************************************************//**
 * @file
 * @brief	Header file of module Control.c
 * @author	Ralf Gerhauser
 * @version	2017-01-25
 ****************************************************************************//*
Revision History:
2017-05-02,rage	- Renamed DFLT_CAM1_DURATION to DFLT_CAM_DURATION.
		- ControlInit: Added CONTROL_INIT structure.
		- Defined PWR_OUT enums to be able to assign power outputs.
		- Added prototypes for CameraEnable(), CameraDisable(),
		  CameraTimedDisable(), PowerOutput(), and IsPowerOutputOn().
		- Removed CAM1 and CAM2 related prototypes.
2017-01-25,rage	Initial version.
*/

#ifndef __INC_Control_h
#define __INC_Control_h

/*=============================== Header Files ===============================*/

#include "config.h"		// include project configuration parameters

/*=============================== Definitions ================================*/

    /*!@brief Show module "Control" exists in this project. */
#define MOD_CONTROL_EXISTS

#ifndef DFLT_CAM_DURATION
    /*!@brief Default camera recording duration after inactivity of the
     * light barrier (in seconds).*/
    #define DFLT_CAM_DURATION		30	// 30s
#endif

#ifndef DFLT_KEEP_OPEN_DURATION
    /*!@brief Default KEEP OPEN duration for the shutter (in seconds). */
    #define DFLT_KEEP_OPEN_DURATION	120	// 2min
#endif

#ifndef DFLT_KEEP_CLOSED_DURATION
    /*!@brief Default KEEP CLOSED duration for the shutter (in seconds). */
    #define DFLT_KEEP_CLOSED_DURATION	240	// 4min
#endif


    /*!@brief Power output selection. */
typedef enum
{
    PWR_OUT_NONE = NONE,	// (-1) for no output at all
    PWR_OUT_UA1,		// 0: DC/DC (3V3 to 12V) at pin UA1 (X10-1)
    PWR_OUT_UA2,		// 1: DC/DC (3V3 to 12V) at pin UA2 (X10-3)
    PWR_OUT_VDD_RFID1,		// 2: 3V3 at pin VDD_RFID1 (X8-3)
    PWR_OUT_VDD_RFID2,		// 3: 3V3 at pin VDD_RFID2 (X8-5)
    PWR_OUT_VDD_RFID3,		// 4: 3V3 at pin VDD_RFID3 (X8-7)
    PWR_OUT_RFID_GND_LB,	// 5: Gnd at pin RFID_GND_LB (X4-5)
    NUM_PWR_OUT
} PWR_OUT;

    /*!@brief Power control. */
//@{
#define PWR_OFF		false	//!< Switch power output off (disable power)
#define PWR_ON		true	//!< Switch power output on  (enable power)
//@}


/*! Initialization structure for the control module
 *
 * Select the appropriate power output for the camera.
 *
 * <b>Typical Example:</b>
 * @code
 * static const CONTROL_INIT  l_ControlCfg =
 * {
 *     PWR_OUT_NONE,		// no camera in use
 * };
 * @endcode
 */
typedef struct
{
    PWR_OUT	 CameraPwrOut;	//!< Power output to switch the camera on or off
} CONTROL_INIT;

/*================================ Prototypes ================================*/

    /* Initialize control module */
void	ControlInit (const CONTROL_INIT *pInitStruct);

    /* Perform miscellaneous control tasks */
void	Control (void);

    /* Determine if feeder is on */
bool	IsFeederOn (void);

    /* Inform the control module about a new transponder ID */
void	ControlUpdateID (char *transponderID);

    /* Enable or disable the camera */
void	CameraEnable (void);
void	CameraDisable (void);
void	CameraTimedDisable (void);

    /* Switch power output on or off */
void	PowerOutput	(PWR_OUT output, bool enable);
bool	IsPowerOutputOn (PWR_OUT output);

    /* Power Fail Handler of the control module */
void	ControlPowerFailHandler (void);


#endif /* __INC_Control_h */
