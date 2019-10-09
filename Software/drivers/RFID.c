/***************************************************************************//**
 * @file
 * @brief	RFID Reader
 * @author	Ralf Gerhauser
 * @version	2018-11-12
 *
 * This module provides the functionality to communicate with the RFID reader.
 * It contains the following parts:
 * - Power management for RFID reader and UART
 * - UART driver to receive data from the RFID reader
 * - Decoder to handle the received data
 *
 * @see LightBarriers.c
 *
 ****************************************************************************//*
 *
 * Parts are Copyright 2013 Energy Micro AS, http://www.energymicro.com
 *
 *******************************************************************************
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 * 4. The source and compiled code may only be used on Energy Micro "EFM32"
 *    microcontrollers and "EFR4" radios.
 *
 * DISCLAIMER OF WARRANTY/LIMITATION OF REMEDIES: Energy Micro AS has no
 * obligation to support this Software. Energy Micro AS is providing the
 * Software "AS IS", with no express or implied warranties of any kind,
 * including, but not limited to, any implied warranties of merchantability
 * or fitness for any particular purpose or warranties against infringement
 * of any proprietary rights of a third party.
 *
 * Energy Micro AS will not be liable for any consequential, incidental, or
 * special damages, or any other relief, or for any claim by any third party,
 * arising from your use of this Software.
 *
 ****************************************************************************//*
Revision History:
2018-11-12,rage	- Set interrupt priority for UARTs.
2017-06-20,rage	- Separated USART settings from RFID settings to be able to
		  support different types of RFID readers.
		- Implemented decoding of Long Range Reader protocol, including
		  CRC calculation and verification.
2017-05-02,rage	- RFID_Init: Added RFID_PWR_OUT structure to specify the
		  power outputs for the two RFID readers.
		- RFID_Parms contains all relevant parameters to initialize
		  the related UART.
2017-04-09,rage	- Extended code to support two RFID readers.
		- Removed Tx part of UART code because it was never used.
2016-04-05,rage	Made all local variables of type "volatile".
2014-11-25,rage	Initial version.
*/

/*=============================== Header Files ===============================*/

#include <stdio.h>
#include <string.h>
#include "em_device.h"
#include "em_assert.h"
#include "em_cmu.h"
#include "em_gpio.h"
#include "em_usart.h"
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

/*=========================== Typedefs and Structs ===========================*/

/*!@brief Structure to hold UART specific parameters */
typedef struct
{
    USART_TypeDef *	const	UART;		//!< UART device to use
    CMU_Clock_TypeDef	const	cmuClock_UART;	//!< CMU clock for the UART
    IRQn_Type		const	UART_Rx_IRQn;	//!< Rx interrupt number
    GPIO_Port_TypeDef	const	UART_Rx_Port;	//!< Port for RX pin
    uint32_t		const	UART_Rx_Pin;	//!< Rx pin on this port
    uint32_t		const	UART_Route;	//!< Route location
} USART_Parms;

/*!@brief Structure to hold RFID reader type specific parameters. */
typedef struct
{
    uint32_t		   const Baudrate;	//!< Baudrate for RFID reader
    USART_Databits_TypeDef const DataBits;	//!< Number of data bits
    USART_Parity_TypeDef   const Parity;	//!< Parity mode
    USART_Stopbits_TypeDef const StopBits;	//!< Number of stop bits
} RFID_TYPE_PARMS;

/*========================= Global Data and Routines =========================*/

    /*!@brief Flags that determine which RFID readers should be activated. */
bool	 g_flgRFID_Activate[NUM_RFID_READER];

    /*!@brief Duration in [s] after which the RFID reader is powered-off. */
int32_t  g_RFID_PwrOffTimeout = DFLT_RFID_POWER_OFF_TIMEOUT;

    /*!@brief Duration in [s] during the RFID reader tries to read an ID. */
int32_t  g_RFID_DetectTimeout = DFLT_RFID_DETECT_TIMEOUT;

    /*!@brief Framing and Parity error counters of the USARTs. */
uint16_t g_FERR_Cnt[NUM_RFID_READER];
uint16_t g_PERR_Cnt[NUM_RFID_READER];

    /*!@brief Transponder number */
char	 g_Transponder[18];

/*================================ Local Data ================================*/

    /*! Local pointer to RFID power output configuration */
static const RFID_CONFIG *l_pRFID_Cfg;

    /*! RFID Reader specific parameters.  Entries are addresses via
     * enums @ref RFID_TYPE.
     */
static const RFID_TYPE_PARMS l_RFID_Type_Parms[NUM_RFID_TYPE] =
{
   {	// RFID_TYPE_FP - 0: Front Panel RFID reader
	  9600,  usartDatabits8,  usartEvenParity,  usartStopbits1
   },
   {	// RFID_TYPE_LR - 1: Long Range RFID reader
	 38400,  usartDatabits8,  usartNoParity,    usartStopbits1
   }
};

    /*! USART specific parameters for each RFID reader */
static const USART_Parms l_USART_Parms[NUM_RFID_READER] =
{
   {
	USART0, cmuClock_USART0, USART0_RX_IRQn,
	gpioPortE, 11, USART_ROUTE_LOCATION_LOC0
   },
   {
    USART1, cmuClock_USART1, USART1_RX_IRQn,
    gpioPortC,  1, USART_ROUTE_LOCATION_LOC0
   }
};

    /*! Timer handle for switching the RFID reader off after a time. */
static volatile TIM_HDL	l_hdlRFID_Off = NONE;

    /*! Flag if RFID reader should be powered on. */
static volatile bool	l_flgRFID_On;

    /*! Flag if RFID reader is currently powered on. */
static volatile bool	l_flgRFID_IsOn;

    /*! Flag indicates if an object is present. */
static volatile bool	l_flgObjectPresent;

    /*! Flag if a new run has been started, i.e. the module is prepared to
     *  receive a transponder number. */
static volatile bool	l_flgNewRun;

    /*! Flag to notify a new transponder ID */
static volatile bool	l_flgNewID;

    /*! Timer handler for the ID detection timeout */
static volatile TIM_HDL	l_hdlRFID_DetectTimeout = NONE;

    /*! State (index) variables for RFID_Decode. */
static volatile uint8_t	l_State[NUM_RFID_READER];

/*=========================== Forward Declarations ===========================*/

static void SwitchRFID_Off(TIM_HDL hdl);
static void RFID_DetectTimeout(TIM_HDL hdl);
static void uartSetup(int deviceNum);


/***************************************************************************//**
 *
 * @brief	Initialize the RFID Reader frame work
 *
 * This routine initializes the RFID reader and all the required functionality
 * around it, e.g. a timer to switch off the reader when it is not in use.
 *
 * @param[in] pInitStruct
 *	Address of an initialization structure of type RFID_PWR_OUT that defines
 *	the power outputs for the RFID readers.
 *
 * @note
 *	Parameter <b>pInitStruct</b> must point to a persistent data structure,
 *	i.e. this must be valid over the whole life time of the program.
 *
 ******************************************************************************/
void	RFID_Init (const RFID_CONFIG *pInitStruct)
{
int	i;

    /* Parameter check */
    EFM_ASSERT(pInitStruct != NULL);

    /* Save configuration */
    l_pRFID_Cfg = pInitStruct;

    /* Check which RFID readers should be activated */
    for (i = 0;  i < NUM_RFID_READER;  i++)
    {
	g_flgRFID_Activate[i] = (l_pRFID_Cfg[i].RFID_PwrOut == PWR_OUT_NONE ?
				 false : true);
    }

    /* Get a timer handle to switch the RFID reader off after a time */
    if (l_hdlRFID_Off == NONE)
	l_hdlRFID_Off = sTimerCreate (SwitchRFID_Off);

    /* Create another timer for the ID detection timeout */
    if (l_hdlRFID_DetectTimeout == NONE)
	l_hdlRFID_DetectTimeout = sTimerCreate (RFID_DetectTimeout);
}


/***************************************************************************//**
 *
 * @brief	Enable RFID reader
 *
 * This routine enables the RFID reader, i.e. it notifies the RFID software
 * module to power up and initialize the reader and the related hardware.
 * It is usually called by the light barrier logic when an object has been
 * detected.  Therefore the timer for the RFID Detect Timeout is also
 * (re-)started.
 *
 * @see RFID_Disable(), RFID_TimedDisable(), RFID_Check()
 *
 ******************************************************************************/
void RFID_Enable (void)
{
    /* set flag to notify there is an object present */
    l_flgObjectPresent = true;

    /* initiate power-on of the RFID reader */
    l_flgRFID_On = true;

    if (l_hdlRFID_Off != NONE)
	sTimerCancel (l_hdlRFID_Off);	// inhibit power-off of RFID reader

    /* re-trigger "new run" flag */
    l_flgNewRun = true;
    DBG_PUTS(" DBG RFID_Enable: setting l_flgNewRun=1\n");

    /* (re-)start timer for RFID timeout detection */
    DBG_PUTS(" DBG RFID_Enable: starting Detect Timeout\n");
    if (l_hdlRFID_DetectTimeout != NONE)
	sTimerStart (l_hdlRFID_DetectTimeout, g_RFID_DetectTimeout);
}


/***************************************************************************//**
 *
 * @brief	Disable RFID reader
 *
 * This routine immediately disables the RFID reader.
 *
 * @see RFID_Enable(), RFID_TimedDisable(), RFID_Check()
 *
 ******************************************************************************/
void RFID_Disable (void)
{
    /* no object present, clear flag */
    l_flgObjectPresent = false;

    /* be sure to cancel timeout timer */
    if (l_hdlRFID_DetectTimeout != NONE)
	sTimerCancel (l_hdlRFID_DetectTimeout);

    if (l_flgRFID_On)
    {
	l_flgRFID_On = false;		// mark RFID reader to be powered off

	if (l_hdlRFID_Off != NONE)
	    sTimerCancel (l_hdlRFID_Off);	// cancel timer

	/* RFID reader should be powered OFF */
	    if (l_flgRFID_IsOn)
	    {
		RFID_PowerOff();
		l_flgRFID_IsOn = false;
	    }
	}
}


/***************************************************************************//**
 *
 * @brief	Disable RFID reader after a while
 *
 * This routine disables the RFID reader, i.e. it notifies the RFID software
 * module to power down the reader after a delay of @ref g_RFID_PwrOffTimeout
 * seconds.
 *
 * @see RFID_Enable(), RFID_Disable(), RFID_Check()
 *
 ******************************************************************************/
void RFID_TimedDisable (void)
{
    /* no object present, clear flag */
    l_flgObjectPresent = false;

    /* be sure to cancel timeout timer */
    if (l_hdlRFID_DetectTimeout != NONE)
	sTimerCancel (l_hdlRFID_DetectTimeout);

    /* (re-)start timer to switch the RFID reader OFF after time */
    if (l_hdlRFID_Off != NONE)
	sTimerStart (l_hdlRFID_Off, g_RFID_PwrOffTimeout);
}


/***************************************************************************//**
 *
 * @brief	Power RFID reader On
 *
 * This routine powers the RFID reader on and initializes the related hardware.
 *
 ******************************************************************************/
void RFID_PowerOn (void)
{
int	i;

	/* Module RFID requires EM1, set bit in bit mask */
	Bit(g_EM1_ModuleMask, EM1_MOD_RFID) = 1;

    for (i = 0;  i < NUM_RFID_READER;  i++)
    {
	if (g_flgRFID_Activate[i])
	{
	/* Prepare UART to receive Transponder ID */
	    uartSetup(i);

	/* Set Power Enable Pin for the RFID receiver to ON */
	    PowerOutput (l_pRFID_Cfg[i].RFID_PwrOut, PWR_ON);
	}

	/* Reset indexes */
	l_State[i] = 0;
    }

#ifdef LOGGING
    /* Generate Log Message */
    Log ("RFID is powered ON");
#endif
}


/***************************************************************************//**
 *
 * @brief	Power RFID reader Off
 *
 * This routine powers all RFID readers immediately off.
 *
 ******************************************************************************/
void RFID_PowerOff (void)
{
int	i;


    for (i = 0;  i < NUM_RFID_READER;  i++)
    {
    /* Set Power Enable Pin for the RFID receiver to OFF */
	PowerOutput (l_pRFID_Cfg[i].RFID_PwrOut, PWR_OFF);

    /* Disable clock for USART module */
	CMU_ClockEnable(l_USART_Parms[i].cmuClock_UART, false);

    /* Disable Rx pin */
	GPIO_PinModeSet(l_USART_Parms[i].UART_Rx_Port,
			l_USART_Parms[i].UART_Rx_Pin, gpioModeDisabled, 0);

    /* Reset indexes */
	l_State[i] = 0;
    }

    /* Module RFID is no longer active, clear bit in bit mask */
    Bit(g_EM1_ModuleMask, EM1_MOD_RFID) = 0;

#ifdef LOGGING
    /* Generate Log Message */
    Log ("RFID is powered off");
#endif
}


/***************************************************************************//**
 *
 * @brief	RFID Check
 *
 * This function checks if the RFID reader needs to be powered on or off,
 * and if a transponder number has been set.
 *
 * @note
 * 	This function may only be called from standard program, usually the loop
 * 	in file main.c - it must not be called from interrupt routines!
 *
 ******************************************************************************/
void	RFID_Check (void)
{
    if (l_flgRFID_On)
    {
	/* RFID reader should be powered ON */
	if (! l_flgRFID_IsOn)
	{
	    RFID_PowerOn();
	    l_flgRFID_IsOn = true;
	}
    }
    else
    {
	/* RFID reader should be powered OFF */
	if (l_flgRFID_IsOn)
	{
		RFID_PowerOff();
		l_flgRFID_IsOn = false;
	    }
	}

    if (l_flgNewID)
    {
	l_flgNewID = false;

	/* New transponder ID has been set - inform control module */
	// Log ("ControlUpdateID(%s) - START", g_Transponder);
	ControlUpdateID(g_Transponder);
	// Log ("ControlUpdateID(%s) - END", g_Transponder);
    }
}


/***************************************************************************//**
 *
 * @brief	RFID Power Fail Handler
 *
 * This function will be called in case of power-fail to bring the RFID
 * hardware into a quiescent, power-saving state.
 *
 ******************************************************************************/
void	RFID_PowerFailHandler (void)
{
    /* Cancel timers */
    if (l_hdlRFID_Off != NONE)
	sTimerCancel (l_hdlRFID_Off);

    if (l_hdlRFID_DetectTimeout != NONE)
	sTimerCancel (l_hdlRFID_DetectTimeout);

    /* Switch RFID reader off */
    l_flgRFID_On = false;

    if (l_flgRFID_IsOn)
    {
	RFID_PowerOff();
	l_flgRFID_IsOn = false;
    }
}


/***************************************************************************//**
 *
 * @brief	Switch RFID Reader Off
 *
 * This routine is called from the RTC interrupt handler, after the specified
 * amount of time has elapsed, to trigger the power-off of the RFID reader.
 *
 ******************************************************************************/
static void SwitchRFID_Off(TIM_HDL hdl)
{
    (void) hdl;		// suppress compiler warning "unused parameter"

    l_flgRFID_On = false;

    g_flgIRQ = true;	// keep on running
}


/***************************************************************************//**
 *
 * @brief	RFID Detect Timeout occurred
 *
 * This routine is called from the RTC interrupt handler, after the specified
 * RFID timeout has elapsed.  This is the duration, the system waits for the
 * detection of a transponder ID.  If no ID could be received during this time,
 * it is set to "UNKNOWN".
 *
 ******************************************************************************/
static void RFID_DetectTimeout(TIM_HDL hdl)
{
    (void) hdl;		// suppress compiler warning "unused parameter"

    if (l_flgObjectPresent)
    {
	DBG_PUTS(" DBG RFID_DetectTimeout: Detect Timeout over, set UNKNOWN\n");

	strcpy (g_Transponder, "UNKNOWN");

#if defined(LOGGING)  &&  ! defined (MOD_CONTROL_EXISTS)
	/* Generate Log Message if there is no external module to handle this */
	Log ("Transponder: %s", g_Transponder);
#endif
	/* Set flag to notify new transponder ID */
	l_flgNewID = true;
    }
}


/***************************************************************************//**
 *
 * @brief	Decode RFID
 *
 * This routine is called from the UART interrupt handler, whenever a new
 * byte has been received.  It contains a state machine to extract a valid
 * transponder ID from the data stream, store it into the global variable
 * @ref g_Transponder, and initiate a display update.  The transponder number
 * is additionally logged, if logging is enabled.
 *
 * The following RFID reader types are supported:
 * - @ref RFID_TYPE_FP : Front Panel reader (14 byte frame)
 * - @ref RFID_TYPE_LR : Long Range reader (11 byte frame)
 *
 * @note
 * The RFID reader permanently sends the ID (i.e. about every 70ms) as long as
 * the transponder resides in its range.  To prevent a huge amount of log
 * messages, the received data is compared with the previous ID.  It will only
 * be logged if it differs, or the flag @ref l_flgNewRun is set, which indicates
 * a new assertion of the light barriers.
 *
 * To detect the absence of a transponder, configuration variable @ref
 * RFID_DETECT_TIMEOUT is used.  If no ID could be read for the amount of time,
 * @ref g_Transponder is set to "UNKNOWN", see RFID_DetectTimeout().
 *
 ******************************************************************************/
static void RFID_Decode(int devNum, uint32_t byte)
{
const  uint8_t	 v[5] = { 0x0E, 0x00, 0x11, 0x00, 0x05};
const  uint8_t	 InvNibble[16] = { 0x0, 0x8, 0x4, 0xC, 0x2, 0xA, 0x6, 0xE,
				   0x1, 0x9, 0x5, 0xD, 0x3, 0xB, 0x7, 0xF };
const  char	 HexChar[16] = {'0', '1', '2', '3', '4', '5', '6', '7',
 				'8', '9', 'A', 'B', 'C', 'D', 'E', 'F'};
static uint8_t	 xorsum[NUM_RFID_READER];  // checksum variables
static uint8_t	 w[NUM_RFID_READER][14];   // buffer for storing received bytes
static uint16_t	 crc[NUM_RFID_READER];     // checksum variables
uint16_t	 val;
char	 newTransponder[18];
int	 i;


    /* parameter check */
    if (devNum < 0  ||  devNum >= NUM_RFID_READER)
    {
	LogError ("RFID_Decode: devNum=%d", devNum);
	return;
    }

    /* count communication errors for debugging purposes */
    if (byte & USART_RXDATAX_FERR)
	g_FERR_Cnt[devNum]++;

    if (byte & USART_RXDATAX_PERR)
	g_PERR_Cnt[devNum]++;

    /* store current byte into receive buffer */
    byte &= 0xFF;		// only bit 7~0 contains the data
    w[devNum][l_State[devNum]] = (uint8_t)byte;

    /* handle data according to the respective RFID reader type */
    switch (l_pRFID_Cfg[devNum].RFID_Type)
    {
	case RFID_TYPE_FP:	// Front Panel reader (14 byte frame, 133kHz)
	    /*
	     * NOTE: Most of the code has been taken from file "RFID_tag.c"
	     */
	    switch (l_State[devNum])	// the state machine!
	    {
	    case 0:
		xorsum[devNum] = 0;
		/* no break */
	    case 1:
	    case 2:
	    case 3:
	    case 4:
		// Verify prefix
		if (byte != v[l_State[devNum]])
		{
		    l_State[devNum] = 0; // restart state machine
		    break;		// break!
		}
		/* no break */
	    case 5:
	    case 6:
	    case 7:
	    case 8:
	    case 9:
	    case 10:
	    case 11:
	    case 12:
		xorsum[devNum] ^= byte;		// build checksum
		l_State[devNum]++;		// go on
		break;			// break!

	    case 13:
		if (w[devNum][13] == xorsum[devNum])	//Checksumme vergleichen!
		{
		    for (i=0; i<8; i++)	// copy w to trans and convert to ASCII HEX
		    {
			newTransponder[2*i]   = HexChar[(w[devNum][12-i]>>4) & 0x0F];
			newTransponder[2*i+1] = HexChar[(w[devNum][12-i])    & 0x0F];
		    }
		    newTransponder[16] = '\0';

		    /* (re-)start timer for RFID timeout detection */
		    if (l_flgObjectPresent  &&  l_hdlRFID_DetectTimeout != NONE)
			sTimerStart (l_hdlRFID_DetectTimeout, g_RFID_DetectTimeout);

		    /* see if a new run - or Transponder Number has changed */
		    if (l_flgNewRun  ||  strcmp (newTransponder, g_Transponder))
		    {
			l_flgNewRun = false;	// clear flag

			/* store new Transponder Number */
			strcpy (g_Transponder, newTransponder);

#if defined(LOGGING)  &&  ! defined (MOD_CONTROL_EXISTS)
			/* Generate Log Message */
			Log ("Transponder: %s", g_Transponder);
#endif
			/* Set flag to notify new transponder ID */
			l_flgNewID = true;
		    }
		}
		/* no break */
	    default:
		l_State[devNum] = 0;	// restart state machine
		break;
	    }
	    break;

	case RFID_TYPE_LR:	// Long Range reader (11 byte frame, 133kHz)
	    switch (l_State[devNum])	// the state machine!
	    {
	    case 0:	// expect prefix 0x54 ('T')
		if (byte != 0x54)
		{
		    l_State[devNum] = 0; // restart state machine
		    break;		// break!
		}
		/* no break */
	    case 1:
		crc[devNum] = 0x0000;	// init w/ 0
		/* no break */
	    case 2:
	    case 3:
	    case 4:
	    case 5:
	    case 6:
	    case 7:
	    case 8:
		/*
		 * calculate CRC-CCITT as used by KERMIT (the protocol,
		 * not the frog ;-)  polynomial: x^16+x^12+x^5+1
		 * http://www.columbia.edu/kermit/ftp/e/kproto.doc
		 */
		val = crc[devNum];
		val = (val >> 4) ^ (((val ^ (byte >> 0)) & 0x0F) * 4225);
		val = (val >> 4) ^ (((val ^ (byte >> 4)) & 0x0F) * 4225);
		crc[devNum] = val;
		/* no break */
	    case 9:
		l_State[devNum]++;	// increase position in buffer
		break;			// break!

	    case 10:	// received complete frame
		val = (w[devNum][10] << 8) | w[devNum][9];
		if (val != crc[devNum])
		{
		    LogError("RFID_Decode(%d): recv.CRC=0x%04X, calc.CRC=0x%04X",
			     devNum, val, crc[devNum]);
		    l_State[devNum] = 0;	// restart state machine
		    break;
		}
		for (i=0; i<8; i++)	// copy w to trans and convert to ASCII HEX
		{
		    newTransponder[2*i]   = HexChar[InvNibble[(w[devNum][1+i]) & 0x0F]];
		    newTransponder[2*i+1] = HexChar[InvNibble[(w[devNum][1+i]>>4) & 0x0F]];
		}
		newTransponder[16] = '\0';

		/* (re-)start timer for RFID timeout detection */
		if (l_flgObjectPresent  &&  l_hdlRFID_DetectTimeout != NONE)
		    sTimerStart (l_hdlRFID_DetectTimeout, g_RFID_DetectTimeout);

		/* see if a new run - or Transponder Number has changed */
		if (l_flgNewRun  ||  strcmp (newTransponder, g_Transponder))
		{
		    l_flgNewRun = false;	// clear flag

		    /* store new Transponder Number */
		    strcpy (g_Transponder, newTransponder);

#if defined(LOGGING)  &&  ! defined (MOD_CONTROL_EXISTS)
		    /* Generate Log Message */
		    Log ("Transponder: %s", g_Transponder);
#endif
		    /* Set flag to notify new transponder ID */
		    l_flgNewID = true;
		}
		/* no break */
	    default:
		l_State[devNum] = 0;	// restart state machine
		break;
	    }
	    break;

	default:		// unknown RFID reader type
	    l_State[devNum] = 0;	// restart state machine
	    break;
    }
}


/*============================================================================*/
/*=============================== UART Routines ==============================*/
/*============================================================================*/

/* Setup UART in async mode for RS232*/
static USART_InitAsync_TypeDef uartInit = USART_INITASYNC_DEFAULT;


/******************************************************************************
* @brief  uartSetup function
*
******************************************************************************/
static void uartSetup(int devNum)
{
int	type = l_pRFID_Cfg[devNum].RFID_Type;

  /* Enable clock for USART module */
  CMU_ClockEnable(l_USART_Parms[devNum].cmuClock_UART, true);

  /* Configure GPIO Rx pin */
  GPIO_PinModeSet(l_USART_Parms[devNum].UART_Rx_Port,
		  l_USART_Parms[devNum].UART_Rx_Pin, gpioModeInput, 0);

  /* Prepare struct for initializing UART in asynchronous mode */
  uartInit.enable       = usartDisable;   // Don't enable UART upon initialization
  uartInit.refFreq      = 0;              // Set to 0 to use reference frequency
  uartInit.baudrate     = l_RFID_Type_Parms[type].Baudrate;
  uartInit.oversampling = usartOVS16;     // Oversampling. Range is 4x, 6x, 8x or 16x
  uartInit.databits     = l_RFID_Type_Parms[type].DataBits;
  uartInit.parity       = l_RFID_Type_Parms[type].Parity;
  uartInit.stopbits     = l_RFID_Type_Parms[type].StopBits;
#if defined( USART_INPUT_RXPRS ) && defined( USART_CTRL_MVDIS )
  uartInit.mvdis        = false;          // Disable majority voting
  uartInit.prsRxEnable  = false;          // Enable USART Rx via Peripheral Reflex System
  uartInit.prsRxCh      = usartPrsRxCh0;  // Select PRS channel if enabled
#endif

  /* Initialize USART with uartInit struct */
  USART_InitAsync(l_USART_Parms[devNum].UART, &uartInit);

  /* Prepare UART Rx interrupts */
  USART_IntClear(l_USART_Parms[devNum].UART, _USART_IF_MASK);
  USART_IntEnable(l_USART_Parms[devNum].UART, USART_IF_RXDATAV);
  NVIC_SetPriority(l_USART_Parms[devNum].UART_Rx_IRQn, INT_PRIO_UART);
  NVIC_ClearPendingIRQ(l_USART_Parms[devNum].UART_Rx_IRQn);
  NVIC_EnableIRQ(l_USART_Parms[devNum].UART_Rx_IRQn);

  /* Enable I/O pins at UART location #2 */
  l_USART_Parms[devNum].UART->ROUTE = USART_ROUTE_RXPEN
				    | l_USART_Parms[devNum].UART_Route;

  /* Enable UART receiver only */
  USART_Enable(l_USART_Parms[devNum].UART, usartEnableRx);
}


/**************************************************************************//**
 *
 * @brief UART 0 RX IRQ Handler
 *
 * This interrupt service routine is called whenever a byte has been received
 * from the RFID reader associated with UART 0.  It calls RFID_Decode() to
 * extract a valid ID from the data stream.
 *
 * NOTE:
 * Since both UARTs use the same interrupt priority, no interference is
 * possible between the two UARTs.
 *
 *****************************************************************************/
void USART0_RX_IRQHandler(void)
{
    /* Check for RX data valid interrupt */
    if (USART0->STATUS & USART_STATUS_RXDATAV)
    {
	/* Decode data */
	RFID_Decode (0, USART0->RXDATA);

	/* Clear RXDATAV interrupt */
	USART_IntClear(USART0, USART_IF_RXDATAV);
    }
}


/**************************************************************************//**
 *
 * @brief UART 1 RX IRQ Handler
 *
 * This interrupt service routine is called whenever a byte has been received
 * from the RFID reader associated with UART 1.  It calls RFID_Decode() to
 * extract a valid ID from the data stream.
 *
 * NOTE:
 * Since both UARTs use the same interrupt priority, no interference is
 * possible between the two UARTs.
 *
 *****************************************************************************/
void USART1_RX_IRQHandler(void)
{
    /* Check for RX data valid interrupt */
    if (USART1->STATUS & USART_STATUS_RXDATAV)
    {
	/* Decode data */
	RFID_Decode (1, USART1->RXDATA);

	/* Clear RXDATAV interrupt */
	USART_IntClear(USART1, USART_IF_RXDATAV);
    }
}
