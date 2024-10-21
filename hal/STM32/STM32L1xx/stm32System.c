/*
 * system.c
 *
 *  Created on: Nov 12, 2023
 *      Author: ludovic
 */
#include <stdint.h>

#include "stm32l1xx.h"

#include "McuSdkConfig.h"
#include "mcuSdkSystem.h"

//static uint32_t nestedCriticalSessionCount = 0uL; //!< Stores the number of nested critical session requests

/*
 *
 */
void mcuSdkSystemInit(void)
{

}

/*
 *
 */
void mcuSdkSystemReset(void)
{
  __NVIC_SystemReset();
}

/*
 *
 */
//void systemEnterCriticalSession(void)
//{
//  nestedCriticalSessionCount++;
//  #if SYSTEM_CRITICAL_SESSION_LEVEL == 0uL
//    __disable_irq();
//    #else
//    __set_BASEPRI(SYSTEM_CRITICAL_SESSION_LEVEL);
//    #endif
//}
//
///*
// *
// */
//void systemExitCriticalSession(void)
//{
//  if( nestedCriticalSessionCount != 0uL )
//  {
//    nestedCriticalSessionCount --;
//    #if SYSTEM_CRITICAL_SESSION_LEVEL == 0uL
//      __enable_irq();
//    #else
//      __set_BASEPRI(0uL);
//    #endif
//  }
//
//}
