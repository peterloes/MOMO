/***************************************************************************//**
 * @file
 * @brief	MAPRLD
 * @author	Ralf Gerhauser
 * @version	2018-11-13
 *
 * This application consists of the following modules:
 * - main.c - Initialization code and main execution loop.
 * - DMA_ControlBlock.c - Control structures for the DMA channels.
 * - Control.c - Sequence Control module.
 * - CfgData.c - Handling of configuration data.
 * - ExtInt.c - External interrupt handler.
 * - AlarmClock.c - Alarm clock and timers facility.
 * - DCF77.c - DCF77 Atomic Clock Decoder
 * - clock.c - An implementation of the POSIX time() function.
 * - LightBarrier.c - Interrupt logic for the two light barriers,
 *   enables the RFID reader.
 * - RFID.c - RFID reader to receive transponder IDs.
 * - BatteryMon.c - Battery monitor, periodically reads the state of the
 *   battery via the SMBus.
 * - LEUART.c - The Low-Energy UART can be used as monitoring and debugging
 *   connection to a host computer.
 * - microsd.c - Together with the files "diskio.c" and "ff.c", this module
 *   provides an implementation of a FAT file system on the @ref ss8.
 * - Logging.c - Logging facility to send messages to the LEUART and store
 *   them into a file on the SD-Card.
 * - PowerFail.c - Handler to switch off all loads in case of Power Fail.
 *
 * Parts of the code are based on the example code of AN0006 "tickless calender"
 * from Energy Micro AS.
 *
 ***************************************************************************//**
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
2020-05-12,rage	Version 1.4:
                - Call CheckAlarmTimes() after CONFIG.TXT has been read.
2018-11-13,rage Version 1.3:
		- Moved DMA related variables to module "DMA_ControlBlock.c".
		- Report available memory (debug version only).
2017-06-20,rage Version 1.2:
		- Specified RFID reader type in array l_RFID_Cfg[].
2017-05-12,rage	Version 1.1:
		- Support for single and double RFID reader configurations,
		  see DUAL_RFID_CONFIGURATION.
		- Power outputs are assigned to the RFID readers and the camera
		  by structures l_RFID_Cfg[] and l_ControlCfg.
		- Updated documentation.
2017-01-22,rage	Initial version for MAPRDL.
*/

/*!
 * @mainpage
 * @section desc Description
 * MAPRDL is an application to control a bird feeder.
 *
 * <b>It consists of the following components:</b>
 *
 * @subsection ssa00 Memory Map
 * FLASH and SRAM memory ranges:
 * <center><table>
 * <tr><th>Address</th> <th>Description</th></tr>
 * <tr><td>0x00000000</td> <td>Start of FLASH, here resides the Booter (32KB)</td></tr>
 * <tr><td>0x00008000</td> <td>Start of Application Software (up to 88KB)</td></tr>
 * <tr><td>0x0001E000</td> <td>EEPROM Emulation Area (8KB)</td></tr>
 * <tr><td>0x0001FFFF</td> <td>End of 128KB FLASH</td></tr>
 * <tr><td>0x10000000</td> <td>Start of SRAM when accessed as Code</td></tr>
 * <tr><td>0x10003FFF</td> <td>End of 16KB SRAM when accessed as Code</td></tr>
 * <tr><td>0x20000000</td> <td>Start of SRAM when accessed as Data</td></tr>
 * <tr><td>0x20003FFF</td> <td>End of 16KB SRAM when accessed as Data</td></tr>
 * </table></center>
 *
 * @subsection ssa01 Microcontroller
 * The heart of the board is an EFM32G230 microcontroller.  It provides two
 * different clock domains: All low-energy peripheral is clocked via a
 * 32.768kHz external XTAL.  The MCU and other high performance peripheral
 * uses a high-frequency clock.  The board can be configured to use the
 * internal RC-oscillator, or an external 32MHz XTAL for that purpose,
 * see define @ref USE_EXT_32MHZ_CLOCK.
 *
 * @subsection ssa02 DCF77 Atomic Clock
 * When powered on, the DCF77 hardware module is enabled to receive the time
 * information and set the system clock.  During this phase, the Power-On LED
 * (i.e. LED1) shows the current state of the DCF signal.  The further behavior
 * depends on the settings in the <i>config.h</i> file.  If the define @ref
 * DCF77_ONCE_PER_DAY is 1, the receiver (and also the LED) will be switched
 * off after time has been synchronized.  As the name of the define suggests,
 * it will be switched on once per day.  To properly detect a change between
 * winter and summer time, i.e. <i>normal time</i> (MEZ) and <i>daylight saving
 * time</i> (MESZ), this happens at 01:55 for MEZ, and 02:55 for MESZ.<br>
 * If the define is 0, the receiver remains switched on, but the LED will only
 * get active again when the time signal gets out-of-sync.<br>
 *
 * @subsection ssa03 Light Barriers
 * The firmware supports two light barriers, currently one is used to detect
 * a bird near the shutter of the feeder.
 *
 * @subsection ssa04 RFID Reader
 * The RFID reader is used to receive the transponder number of the bird, when
 * picking food.  It is enabled by the light barriers and shut down after @ref
 * RFID_PWR_OFF_TIMEOUT seconds when there is no more activity detected.  The
 * transponder @ref ID is looked up in the configuration file to get specific
 * settings for this bird.  When no transponder ID could be detected within
 * @ref RFID_DETECT_TIMEOUT seconds, the bird is treated as <b>UNKNOWN</b>.
 * If an ID is read that is not listed in the configuration file, the bird is
 * treated as <b>ANY</b>.  UNKNOWN and ANY are special IDs in the configuration
 * file to handle these groups of birds.
 * The default values of the timeout defines above may be overwritten by the
 * variables in <a href="../../CONFIG.TXT"><i>CONFIG.TXT</i></a>.
 * The power output for the RFID readers can be configured via @ref l_RFID_Cfg.
 *
 * @subsection ssa05 Feeder Shutter
 * This is one of the key elements of the system.  It controls when the feeder
 * offers food.  When and how long this happens can be specified via the
 * configuration file on the SD-Card.
 *
 * @subsection ssa06 Camera
 * If configured, a camera is switched on as soon as a light barrier detects an
 * object.  It will be switched off @ref CAM_DURATION seconds after the light
 * barriers go inactive.  The power output for the camera can be configured
 * via @ref l_ControlCfg.  Use @ref PWR_OUT_NONE if no camera is connected.
 *
 * @subsection ssa07 LEDs
 * There are two LEDs, one is red, the other one is green.
 * - The red LED is the Power-On LED.  It shows the current state of the
 *   DCF77 signal during receiving of the time information.  In normal
 *   condition the LED will be switched off.
 * - The green LED is the Log Flush LED.  It flashes whenever there are write
 *   accesses to SD-Card.
 *
 * @subsection ss8 SD-Card
 * The SD-Card is used to store configuration and logging data.  Only formatted
 * cards can be used, supported file systems are FAT12, FAT16, and FAT32.  The
 * filenames must follow the DOS schema 8+3, i.e. maximum 8 characters for
 * the basename, and 3 for the extension.  If the SD-Card contains a file
 * <b>BOX<i>nnnn</i>.TXT</b>, where <b><i>nnnn</i></b> can be any decimal
 * number, this will be used as the new log file.  In this way the filename
 * allows you to assign a specific box number to a dedicated unit.  If no
 * such file exists on the media, file "BOX0999.TXT" is created.
 * Another file on the SD-Card is <a href="../../CONFIG.TXT"><i>CONFIG.TXT</i></a>
 * which contains the configuration for the feeder.  Have a look at the example
 * configuration file for a description of all possible variables defined in
 * @ref l_CfgVarList.
 * - Removing an SD-Card
 *   -# Generate a log message by triggering the light barriers, optionally
 *      with a reference transponder.
 *   -# Wait about 5 seconds (@ref LOG_SAMPLE_TIMEOUT) until the green LED
 *      starts flashing.  Log data is now written to the SD-Card.
 *   -# When the green LED stops flashing, you have a guaranteed duration
 *      of 15 seconds (@ref LOG_FLUSH_PAUSE) where no further data will be
 *      written to the SD-Card.
 *   -# Remove the SD-Card within this time.  The message "SD-Card removed"
 *      will be displayed on the LCD for 20 seconds.  All newly generated log
 *      messages will be stored in memory until another media is available.
 *
 * - Inserting an SD-Card
 *   -# Just insert the SD-Card into the slot.  The message "SD-Card inserted"
 *      will be displayed on the LCD.
 *   -# The file system on the SD-Card will be mounted and the free space of
 *      the media is reported.
 *   -# If a firmware update file (*.UPD) exists on the SD-Card, a reboot will
 *      be initiated to pass control to the booter.
 *   -# Otherwise the firmware looks for a "BOX<n>.TXT" file to use as new log
 *      file.
 *
 * @subsection ssa08 Battery Monitor
 * The battery pack has its own controller.  It is connected to the EFM32
 * microcontroller via I2C-bus.  The battery monitor requests status
 * information from the battery pack and logs this on a regular basis, i.e.
 * every @ref BAT_MON_INTERVAL seconds.
 *
 * @subsection ssa09 Low-Energy UART
 * The Low-Power UART (LEUART) provides a connection to a host computer (PC).
 * It can be used as monitoring and debugging interface.  All log messages,
 * written to the SD-Card, are sent through this interface also.  This behaviour
 * can be changed by defining @ref LOG_MONITOR_FUNCTION to @ref NONE.
 * The format of this UART is 9600 baud, 8 data bits, no parity.
 *
 * @subsection ssa10 Firmware
 * The firmware consists of an initialization part and a main loop, also called
 * service execution loop.  The initialization part sets up all modules, enables
 * devices and interrupts.  The service execution loop handles all tasks that
 * must not be executed in interrupt context.
 *
 * After power-up or reset the following actions are performed:
 * -# Basic initialization of MCU and clocks
 * -# Low-Energy UART is set up
 * -# LEDs are switched on for test purposes (lamp test)
 * -# The logging facility is initialized and the firmware version is logged
 * -# Further hardware initialization (DCF77, RFID reader, Light
 *    Barriers, SD-Card interface, Interrupts, Alarm Clock, and Control Module)
 * -# LEDs are switched off after 4 seconds
 * -# The DCF77 Atomic Clock is enabled to receive the time signal
 *
 * The program then enters the Service Execution Loop which takes care of:
 * - Power management for the RFID reader
 * - SD-Card change detection, re-mounting the filesystem, and reading the
 *   configuration file <a href="../../CONFIG.TXT"><i>CONFIG.TXT</i></a>
 * - Battery monitoring
 * - Writing the log buffer to the SD-Card
 * - Entering the right energy mode
 *
 * @section cfg Configuration Variables
 * The behaviour of the system can be changed by setting configuration
 * variables in the file <a href="../../CONFIG.TXT"><i>CONFIG.TXT</i></a>
 * which resides on the SD-Card.  It follows a description of these variables.
 *
 * @subsection ssb01 ON_TIME, OFF_TIME
 * These variables determine the on and off time of the feeder.  When it
 * is in the off state, the feeder is closed and light barriers are ignored.
 *
 * @subsection LB_FILTER_DURATION
 * Light barrier (LB) filter duration in seconds.  The LB may change its
 * state several times during a bird is picking food.  To prevent the
 * underlying logic from assuming each time "a new bird", this filter has
 * been implemented.  The duration starts whenever the LB turns to inactive
 * state, i.e. no object is detected any more.  When the LB returns  active
 * before this time is over, this event will be ignored.  Timing diagram:
 * @code
                        ____        __
    LB activity  :  ___/    \______/  \______________________________
                        ____________________________
    Filter Output:  ___/              |------t------\________________
    (bird present)
   @endcode
 * A value of 0 disables the filter.
 *
 * @subsection RFID_PWR_OFF_TIMEOUT
 * Duration in seconds after which the RFID reader is powered-off.  It is
 * measured from the time when the light barrier filter output gets inactive.
 * Minimum value is 1 second.
 *
 * @subsection RFID_DETECT_TIMEOUT
 * Duration in seconds measured from the trigger, i.e. when the light barrier
 * filter output gets active or a transponder has been read already.
 * Within this period, the system tries to identify the bird by its
 * transponder ID.  If no transponder is detected, the bird is treated
 * as UNKNOWN and the feeder usually will be closed.
 * Minimum value is 1 second.
 *
 * @subsection KEEP_OPEN
 * Default duration in seconds how long the shutter keeps open.  Letter
 * <b>A</b> means <i>always</i> - use this if the feeder should be permanently
 * left open.  A value of 0 doesn't open the shutter at all, instead it is
 * closed immediately.
 *
 * @subsection KEEP_CLOSED
 * Default duration in seconds how long the shutter keeps closed.  Letter
 * <b>A</b> means <i>always</i> - use this if the feeder should be permanently
 * kept closed.
 *
 * @subsection CAM_DURATION
 * Duration in seconds, the camera is still powered after light barrier filter
 * output has been gone inactive.  Be aware, the camera itself may be
 * configured to have an additional time before powering off.
 * Minimum value is 1 second.
 *
 * @subsection ID
 * Transponder ID and optional parameters.
 *
 *     ID = 0123456789012345:{KEEP_OPEN}:{KEEP_CLOSED}:{CAM_DURATION}
 *
 * Fields may be left empty to use default values, for example
 *
 *     ID = 9E1CE7D001AF0001:::120
 *
 * specifies 120s (2min) for camera 1 duration, but uses the default settings
 * for @ref KEEP_OPEN and @ref KEEP_CLOSED.
 * There are two special IDs: <b>ANY</b> means there was a transponder
 * detected, but its ID is not listed in this file.  <b>UNKNOWN</b> means
 * that no transponder could be detected within @ref RFID_DETECT_TIMEOUT.
 *
 * @section scn Possible Scenarios
 * There are 3 types of RFID information that need to be handled: a known ID
 * with an entry in the configuration file, an ID that is not listed in the
 * file (referred to as <b>ANY</b>), and no ID could be detected at all.
 * Based on these 3 types, additional scenarios may happen when another bird
 * appears on the feeder while there is already a bird picking food.  This may
 * lead to a direct change between the cases described below.
 *
 * @subsection init Initial State
 * - No object is detected by the light barriers (LB).
 * - RFID reader is powered off.
 *
 * @subsection case_a Case A: Bird with known transponder ID
 * 1. Bird triggers light barriers.
 * 2. RFID reader is powered on.
 * 3. Transponder ID is read.
 * 4. ID could be found in the configuration file.
 * 5. ID-specific timing -or- default values are loaded.
 * 6. Feeder is left open for the @ref KEEP_OPEN duration.
 * 7. Feeder will then close and kept closed for the @ref KEEP_CLOSED duration.
 * 8. Feeder then opens again.
 *
 * General remarks:
 * - If the light barrier is no more triggered for @ref RFID_PWR_OFF_TIMEOUT,
 *   the RFID reader will be powered off.
 * - The RFID reader repeatedly reads (the same) transponder ID as long as the
 *   bird is located near the receiver, but the ID is reported only once.
 *
 * @subsection case_b Case B: Bird with transponder ID which is not in the list
 * 1. to 3. See @ref case_a.
 * 4. The ID could not be found in the configuration file, therefore the
 *    <b>ANY</b> entry is used.
 * 5. Timings of the ANY entry -or- default values are loaded.
 * 6. to 8. See @ref case_a.
 *
 * @subsection case_c Case C: Bird has no transponder
 * 1. to 2. See @ref case_a.
 * 3. After @ref RFID_DETECT_TIMEOUT seconds, the bird is treated as
 *    <b>UNKNOWN</b>.
 * 4. The UNKNOWN entry is looked-up in the configuration file.
 * 5. Timings of the UNKNOWN entry -or- default values are loaded.
 * 6. to 8. See @ref case_a.
 *
 * @subsection case_aa Change from case A or B to another case of type A or B
 * Another transponder ID is detected and looked-up in the configuration file.
 * The respective values are loaded and take effect.
 *
 * @subsection case_ac Change from case A or B to case C
 * If the light barriers are still triggered, the RFID reader is powered, but
 * no transponder ID could be read within @ref RFID_DETECT_TIMEOUT seconds, the
 * ID will internally change to <b>UNKNOWN</b> and the respective values are
 * loaded.
 *
 * @subsection case_ca Change from case C to case A or B
 * If the light barriers are triggered, the RFID reader is powered and suddenly
 * detects a transponder ID, this leads to case A or B.
 *
 * @subsection case_cc Change from case C to another case of type C
 * Since no transponders could be detected at all, the @ref LB_FILTER_DURATION
 * is an important parameter for this scenario.  If the light barriers are not
 * (re-)triggered within this duration, this is treated as absence of any bird.
 * A "new" bird without transponder may be detected as soon as the light
 * barriers report an object again.
 */
/*=============================== Header Files ===============================*/

#include <stdio.h>
#include "em_device.h"
#include "em_chip.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "config.h"		// include project configuration parameters
#include "ExtInt.h"
#include "DCF77.h"
#include "LightBarrier.h"
#include "RFID.h"
#include "AlarmClock.h"
#include "LEUART.h"
#include "BatteryMon.h"
#include "Logging.h"
#include "CfgData.h"
#include "Control.h"
#include "PowerFail.h"

#ifdef DEBUG
#include <malloc.h>
#endif

#include "ff.h"		// FS_FAT12/16/32
#include "diskio.h"	// DSTATUS
#include "microsd.h"

/*=============================== Definitions ================================*/

/*! @brief Dual RFID Reader Configuration.
 *
 * This source code currently provides two possible configurations:
 *
 * - Set DUAL_RFID_CONFIGURATION to 0 if there is just a single RFID reader
 *   which is connected to power output RFID_GND_LB.
 * - Set DUAL_RFID_CONFIGURATION to 1 for a dual RFID reader configuration.
 *   The two readers must be connected to the power outputs UA1 and UA2.
 *   Ensure that these provide 3.3 Volts.
 */
#define DUAL_RFID_CONFIGURATION	1	// set 1 to select dual RFID readers

/*================================ Global Data ===============================*/

extern PRJ_INFO const  prj;		// Project Information

/*! @brief Flag to indicate that an Interrupt occurred in the meantime.
 *
 * This flag must be set <b>true</b> by any interrupt service routine that
 * requires actions in the service execution loop of main().  This prevents
 * the system from entering sleep mode, so the action can be taken before.
 */
volatile bool		g_flgIRQ;

/*! @brief Modules that require EM1.
 *
 * This global variable is a bit mask for all modules that require EM1.
 * Standard peripherals would stop working in EM2 because clocks, etc. are
 * disabled.  Therefore it is required for software modules that make use
 * of such devices, to set the appropriate bit in this mask, as long as they
 * need EM1.  This prevents the power management of this application to enter
 * EM2.  The enumeration @ref EM1_MODULES lists those modules.
 * Low-Power peripherals, e.g. the LEUART still work in EM1.
 *
 * Examples:
 *
   @code
   // Module RFID requires EM1, set bit in bit mask
   Bit(g_EM1_ModuleMask, EM1_MOD_RFID) = 1;
   ...
   // Module RFID is no longer active, clear bit in bit mask
   Bit(g_EM1_ModuleMask, EM1_MOD_RFID) = 0;
   @endcode
 */
volatile uint16_t	g_EM1_ModuleMask;

/*================================ Local Data ================================*/

/*! EXTI initialization structure
 *
 * Connect the external interrupts of the push buttons to the key handler, the
 * DCF77 signal to the atomic clock module, the outer and inner light barrier
 * to their handler.
 */
static const EXTI_INIT  l_ExtIntCfg[] =
{   //	IntBitMask,	IntFct
    {	DCF_EXTI_MASK,	DCF77Handler		},	// DCF77
    {	LB_EXTI_MASK,	LB_Handler		},	// Light Barriers
    {	PF_EXTI_MASK,	PowerFailHandler	},	// Power Fail
    {	0,		NULL			}
};

/*! RFID reader initialization structure
 *
 * Select the appropriate power outputs to switch the RFID readers on or off.
 */
#if DUAL_RFID_CONFIGURATION	// dual RFID configuration
static const RFID_CONFIG  l_RFID_Cfg[NUM_RFID_READER] =
{   //	RFID_Type,	RFID_PwrOut
    {	RFID_TYPE_LR,	PWR_OUT_UA1	},	// (USART0)
    {	RFID_TYPE_FP,	PWR_OUT_UA2	},	// (UASRT1)
};
#else				// single RFID configuration
static const RFID_CONFIG  l_RFID_Cfg[NUM_RFID_READER] =
{   //	RFID_Type,	RFID_PwrOut
    {	RFID_TYPE_FP,	PWR_OUT_RFID_GND_LB },	// (USART0)
    {	RFID_TYPE_NONE,	PWR_OUT_NONE	},	// (not connected)
};
#endif

/*! Initialization structure for the control module
 *
 * Select the appropriate power output for the camera.
 */
#if DUAL_RFID_CONFIGURATION	// dual RFID configuration
static const CONTROL_INIT  l_ControlCfg =
{
    PWR_OUT_NONE,		// no camera in use
};
#else				// single RFID configuration
static const CONTROL_INIT  l_ControlCfg =
{
    PWR_OUT_UA1,		// the camera is controlled by UA1
};
#endif

/*!@brief Array of functions to be called in case of power-fail.
 *
 * Initialization array to define the power-fail handlers required for some
 * modules.
 * This array must be 0-terminated.
 */
static const POWER_FAIL_FCT l_PowerFailFct[] =
{
    RFID_PowerFailHandler,	// switch off RFID reader
    ControlPowerFailHandler,	// switch off power outputs
    NULL
};

/* Return code for CMU_Select_TypeDef as string */
static const char *CMU_Select_String[] =
{ "Error", "Disabled", "LFXO", "LFRCO", "HFXO", "HFRCO", "LEDIV2", "AUXHFRCO" };

/*=========================== Forward Declarations ===========================*/

static void cmuSetup(void);
static void Reboot(void);
#ifdef DEBUG
static void MemInfo (void);
#endif


/******************************************************************************
 * @brief  Main function
 *****************************************************************************/
int main( void )
{
    /* Initialize chip - handle erratas */
    CHIP_Init();

    /* Set up clocks */
    cmuSetup();

    /* Init Low Energy UART with 9600bd (this is the maximum) */
    drvLEUART_Init (9600);

#ifdef DEBUG
    dbgInit();
#endif

    /* Output version string to SWO or LEUART */
    drvLEUART_puts("\n***** MAPRDL V");
    drvLEUART_puts(prj.Version);
    drvLEUART_puts(" *****\n\n");

    /* Configure PA2 to drive the red Power-On LED (LED1) - show we are alive */
    GPIO_PinModeSet (POWER_LED_PORT, POWER_LED_PIN, gpioModePushPull, 1);

    /* Configure PA5 to drive the green Ready LED - show we are alive */
    GPIO_PinModeSet (LOG_FLUSH_LED_PORT, LOG_FLUSH_LED_PIN, gpioModePushPull, 1);

    /*
     * All modules that make use of external interrupts (EXTI) should be
     * initialized before calling ExtIntInit() because this enables the
     * interrupts, so IRQ handler may be executed immediately!
     */

    /* Initialize Logging (do this early) */
    LogInit();

    /* Log Firmware Revision and Clock Info */
    Log ("MAPRDL V%s (%s %s)", prj.Version, prj.Date, prj.Time);
    uint32_t freq = CMU_ClockFreqGet(cmuClock_HF);
    Log ("Using %s Clock at %ld.%03ldMHz",
	 CMU_Select_String[CMU_ClockSelectGet(cmuClock_HF)],
	 freq / 1000000L, (freq % 1000000L) / 1000L);

#ifdef DEBUG
    MemInfo();		// report available memory
#endif

     /* Initialize DCF77 hardware, configure Interrupt */
    DCF77Init();

    /* Initialize RFID reader */
    RFID_Init(l_RFID_Cfg);

    /* Initialize light barrier hardware, configure Interrupt */
    LB_Init();

    /* Initialize SD-Card Interface */
    DiskInit();

    /* Introduce Power-Fail Handlers, configure Interrupt */
    PowerFailInit (l_PowerFailFct);

    /* Initialize External Interrupts */
    ExtIntInit (l_ExtIntCfg);

    /* Initialize the Alarm Clock module */
    AlarmClockInit();

    /* Initialize control module */
    ControlInit(&l_ControlCfg);

    /* Switch Log Flush LED OFF */
    LOG_FLUSH_LED = 0;

    /* Initialize Battery Monitor */
    BatteryMonInit();

    /* Enable the DCF77 Atomic Clock Decoder */
    DCF77Enable();

    /* Enable all other External Interrupts */
    ExtIntEnableAll();


    /* ============================================ *
     * ========== Service Execution Loop ========== *
     * ============================================ */
    while (1)
    {
	/* Check for power-fail */
	if (! PowerFailCheck())
	{
#if ENABLE_LEUART_RECEIVER
	    /* Check for command from Debug Console */
	    CheckCommand();
#endif
	    /* Check if to power-on or off the RFID reader */
	    RFID_Check();

	    /* Check if SD-Card has been inserted or removed */
	    if (DiskCheck())
	    {
		/* First check if an "*.UPD" file exists on this SD-Card */
		if (FindFile ("/", "*.UPD") != NULL)
		{
		    /*
		     * In this case the SD-Card contains update images.  We must
		     * pass control to the booter to perform a firmware upgrade.
		     */
		    Reboot();
		}

		/* New File System mounted - (re-)open Log File */
		LogFileOpen("BOX*.TXT", "BOX0999.TXT");

		/* Be sure to flush current log buffer so it is empty */
		LogFlush(true);	// keep SD-Card power on!

		/* Log information about the MCU and the battery */
		uint32_t uniquHi = DEVINFO->UNIQUEH;
		Log ("MCU: %s HW-ID: 0x%08lX%08lX",
		     PART_NUMBER, uniquHi, DEVINFO->UNIQUEL);
		LogBatteryInfo (BAT_LOG_INFO_VERBOSE);

		/* Read and parse configuration file */
		CfgRead(CONFIG_FILE_NAME);

		/* Flush log buffer again and switch SD-Card power off */
		LogFlush(false);
                
                /* See if devices must be switched on at this time */
		CheckAlarmTimes();

#ifdef DEBUG
		MemInfo();	// report available memory
#endif
	    }

	    /* Check Battery State */
	    BatteryCheck();

	    /* Call sequence control module */
	    Control();

	    /* Check if to flush the log buffer */
	    LogFlushCheck();
	}

	/*
	 * Check for current power mode:  If a minimum of one active module
	 * requires EM1, i.e. <g_EM1_ModuleMask> is not 0, this will be
	 * entered.  If no one requires EM1 activity, EM2 is entered.
	 */
	if (! g_flgIRQ)		// enter EM only if no IRQ occurred
	{
	    if (g_EM1_ModuleMask)
		EMU_EnterEM1();		// EM1 - Sleep Mode
	    else
		EMU_EnterEM2(true);	// EM2 - Deep Sleep Mode
	}
	else
	{
	    g_flgIRQ = false;	// clear flag to enter EM the next time
	}
    }
}


/******************************************************************************
 * @brief   Configure Clocks
 *
 * This local routine is called once from main() to configure all required
 * clocks of the EFM32 device.
 *
 *****************************************************************************/
static void cmuSetup(void)
{
    /* Start LFXO and wait until it is stable */
    CMU_OscillatorEnable(cmuOsc_LFXO, true, true);

#if USE_EXT_32MHZ_CLOCK
    /* Start HFXO and wait until it is stable */
    CMU_OscillatorEnable(cmuOsc_HFXO, true, true);

    /* Select HFXO as clock source for HFCLK */
    CMU_ClockSelectSet(cmuClock_HF, cmuSelect_HFXO);

    /* Disable HFRCO */
    CMU_OscillatorEnable(cmuOsc_HFRCO, false, false);
#endif

    /* Route the LFXO clock to the RTC and set the prescaler */
    CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFXO);	// RTC, LETIMER
    CMU_ClockSelectSet(cmuClock_LFB, cmuSelect_LFXO);	// LEUART0/1
    CMU_ClockEnable(cmuClock_RTC, true);

    /* Prescaler of 1 = 30 us of resolution and overflow each 8 min */
    CMU_ClockDivSet(cmuClock_RTC, cmuClkDiv_1);

    /* Enable clock to low energy modules */
    CMU_ClockEnable(cmuClock_CORELE, true);

    /* Enable clock for HF peripherals (ADC, DAC, I2C, TIMER, and USART) */
    CMU_ClockEnable(cmuClock_HFPER, true);

    /* Enable clock to GPIO */
    CMU_ClockEnable(cmuClock_GPIO, true);
}


/******************************************************************************
 * @brief   Reboot
 *
 * This local routine brings the system into a quiescent state and then
 * generates a reset.  It is typically used to transfer control from the
 * application to the booter for firmware upgrades.
 *
 *****************************************************************************/
static void Reboot(void)
{
int	n, i;

     /* Disable external interrupts */
    ExtIntDisableAll();

    /* Shut down peripheral devices */
    BatteryMonDeinit();
    RFID_PowerOff();
    DCF77Disable();

    drvLEUART_puts ("Shutting down system for reboot\n");

    /*
     * Show LED Pattern before resetting:
     * 3x 5-short-pulses, separated by a pause,
     * finally a dimming LED from maximum brightness to off.
     */
    for (n = 0;  n < 3;  n++)		// 3x patterns
    {
	for (i = 0;  i < 5;  i++)	// a' 5 pulses
	{
	    POWER_LED = 1;
	    msDelay(100);
	    POWER_LED = 0;
	    msDelay(100);
	}
	msDelay(800);			// pause
    }

    for (n = 0;  n < 200;  n++)
    {
	POWER_LED = 1;
	for (i = 0;  i < (200 - n);  i++)
	    DelayTick();

	POWER_LED = 0;
	for (i = 0;  i < n;  i++)
	    DelayTick();
    }

    /* Perform RESET */
    NVIC_SystemReset();
}


/******************************************************************************
 * @brief   Show DCF77 Signal Indicator
 *
 * This routine is called by the DCF77 module during the synchronisation of
 * the clock to indicate the current state of the DCF77 signal.
 * On the APRDL board it sets the red power LED to the current state of
 * the DCF77 signal, LED on means high, LED off low level.
 *
 *****************************************************************************/
void	ShowDCF77Indicator (bool enable)
{
    POWER_LED = enable;
}

#ifdef DEBUG
// Debug routine to show remaining memory pool size
static void MemInfo (void)
{
    /* Report available memory */
    size_t size;
    for (size = 128;  size < 32 * 1024;  size += 128)
    {
	void *ptr = malloc (size);	// allocate memory
	if (ptr == NULL)
	    break;

	free (ptr);			// free memory
    }
    Log ("Memory Info: %ld bytes available", size - 128);
}
#endif
