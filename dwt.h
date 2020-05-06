/*
 * dwt.h
 *
 *  Created on: May 3, 2020
 *      Author: Lenovo310
 */

#ifndef DWT_H_
#define DWT_H_

/* DWT (Data Watchpoint and Trace) registers, only exists on ARM Cortex with a DWT unit */

#define DWT_CONTROL                 (*((volatile uint32_t*)0xE0001000))
/*!< DWT Control register */
#define DWT_CYCCNTENA_BIT           (1UL<<0)
/*!< CYCCNTENA bit in DWT_CONTROL register */
#define DWT_CYCCNT                  (*((volatile uint32_t*)0xE0001004))
/*!< DWT Cycle Counter register */
#define DEMCR                       (*((volatile uint32_t*)0xE000EDFC))
/*!< DEMCR: Debug Exception and Monitor Control Register */
#define TRCENA_BIT                  (1UL<<24)
/*!< Trace enable bit in DEMCR register */

#define DWT_InitCycleCounter()     DEMCR |= TRCENA_BIT
  /*!< TRCENA: Enable trace and debug block DEMCR (Debug Exception and Monitor Control Register */

#define DWT_ResetCycleCounter()    DWT_CYCCNT = 0
  /*!< Reset cycle counter */

#define DWT_EnableCycleCounter()   DWT_CONTROL |= DWT_CYCCNTENA_BIT
  /*!< Enable cycle counter */

#define DWT_DisableCycleCounter()  DWT_CONTROL &= ~DWT_CYCCNTENA_BIT
  /*!< Disable cycle counter */

#define DWT_GetCycleCounter()      DWT_CYCCNT
  /*!< Read cycle counter register */


#endif /* DWT_H_ */
