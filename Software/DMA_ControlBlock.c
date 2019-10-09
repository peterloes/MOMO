/***************************************************************************//**
 * @file
 * @brief	DMA Control Block
 * @author	Ralf Gerhauser
 * @version	2018-10-09
 *
 * This file contains the DMA Control Blocks for all DMA channels.  It should
 * be linked as the first module in the list, so its data address is located
 * at the beginning of RAM.  This ensures no extra alignment is required which
 * will save memory.
 *
 ****************************************************************************//*
Revision History:
2018-10-09,rage	Initial version.
*/

/*=============================== Header Files ===============================*/

#include "em_device.h"
#include "em_dma.h"

/*================================ Global Data ===============================*/

/*! @brief Global DMA Control Block.
 *
 * It contains the configuration for all 8 DMA channels which may be used by
 * various peripheral devices, e.g. ADC, DAC, USART, LEUART, I2C, and others.
 * The entries of this array will be set by the initialization routines of the
 * driver, which was assigned to the respective channel.  Unused entries remain
 * zero.  There is a total of 16 entries in the array.  The first 8 are used
 * for the primary DMA structures, the second 8 for alternate DMA structures
 * as used for DMA scatter-gather mode, where one buffer is still available,
 * while the other can be re-configured.  This application uses only the first
 * 8 entries.
 *
 * @see  DMA Channel Assignment
 *
 * @note This array must be aligned to 256!
 */
#if defined (__ICCARM__)
    #pragma data_alignment=256
    DMA_DESCRIPTOR_TypeDef g_DMA_ControlBlock[DMA_CHAN_COUNT * 2]
	= { { .USER = 1 } };
#elif defined (__CC_ARM)
    DMA_DESCRIPTOR_TypeDef g_DMA_ControlBlock[DMA_CHAN_COUNT * 2] __attribute__ ((aligned(256)))
	= { { .USER = 1 } };
#elif defined (__GNUC__)
    DMA_DESCRIPTOR_TypeDef g_DMA_ControlBlock[DMA_CHAN_COUNT * 2] __attribute__ ((aligned(256)))
	= { { .USER = 1 } };
#else
    #error Undefined toolkit, need to define alignment
#endif


/*! @brief Global DMA Callback Structure.
 *
 * This array contains the addresses of the DMA callback functions, which are
 * executed for a dedicated DMA channel at the end of a DMA transfer.
 * The entries of this array will be set by the initialization routines of the
 * driver, which was assigned to the respective channel.  Unused entries remain
 * zero.
 */
DMA_CB_TypeDef g_DMA_Callback[DMA_CHAN_COUNT];

