/***************************************************************************//**
 * @file
 * @brief	Header file of module RFID.c
 * @author	Ralf Gerhauser
 * @version	2016-02-24
 ****************************************************************************//*
Revision History:
2017-06-20,rage	Defined RFID_TYPE, extended RFID_CONFIG with it.
2017-05-02,rage	RFID_Init: Added RFID_PWR_OUT structure and NUM_RFID_READER.
2016-02-24,rage	Added prototype for RFID_PowerOff().
2014-11-25,rage	Initial version.
*/

#ifndef __INC_RFID_h
#define __INC_RFID_h

/*=============================== Header Files ===============================*/

#include <stdio.h>
#include <stdbool.h>
#include "em_device.h"
#include "config.h"		// include project configuration parameters
#include "Control.h"

/*=============================== Definitions ================================*/

    /*!@brief Number of RFID readers which are handled by this module. */
#define NUM_RFID_READER	2	//!< max. 2 RFID readers are supported

    /*!@brief Duration in [s] after which the RFID reader is powered-off. */
#ifndef DFLT_RFID_POWER_OFF_TIMEOUT
    #define DFLT_RFID_POWER_OFF_TIMEOUT	30
#endif

    /*!@brief Duration in [s] during the RFID reader tries to read an ID. */
#ifndef DFLT_RFID_DETECT_TIMEOUT
    #define DFLT_RFID_DETECT_TIMEOUT	10
#endif


    /*!@brief RFID types. */
typedef enum
{
    RFID_TYPE_NONE = NONE,	// (-1) for no RFID at all
    RFID_TYPE_FP,		// 0: Front Panel RFID reader
    RFID_TYPE_LR,		// 1: Long Range RFID reader
    NUM_RFID_TYPE
} RFID_TYPE;

/*!@brief Structure to specify the type of RFID readers and the power outputs */
typedef struct
{
    RFID_TYPE		RFID_Type;		//!< RFID type selection
    PWR_OUT		RFID_PwrOut;		//!< Power output selection
} RFID_CONFIG;

/*================================ Global Data ===============================*/

extern int32_t	 g_RFID_PwrOffTimeout;
extern int32_t	 g_RFID_DetectTimeout;
extern char	 g_Transponder[18];

/*================================ Prototypes ================================*/

    /* Initialize the RFID module */
void	RFID_Init (const RFID_CONFIG *pInitStruct);

    /* Enable RFID reader */
void	RFID_Enable (void);

    /* Disable RFID reader */
void	RFID_Disable (void);

    /* Disable RFID reader after a while */
void	RFID_TimedDisable (void);

    /* Check if to power-on/off RFID reader, get tranponder number */
void	RFID_Check (void);

    /* Power RFID reader Off */
void	RFID_PowerOff (void);

    /* RFID Power Fail Handler */
void	RFID_PowerFailHandler (void);


#endif /* __INC_RFID_h */
