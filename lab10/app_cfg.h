/*
*********************************************************************************************************
*                              uC/OS-II Application Configuration
*
*                       DO NOT DELETE THIS FILE, IT IS REQUIRED FOR OS_VER > 2.80
*
*                                   CHANGE SETTINGS ACCORDINGLY
*
*
* File : app_cfg.h
*********************************************************************************************************
*/

#ifndef  APP_CFG_H
#define  APP_CFG_H


/*
*********************************************************************************************************
*                                       INCLUDES
*********************************************************************************************************
*/

#include <lib_def.h>


/*
*********************************************************************************************************
*                                       ADDITIONAL uC/MODULE ENABLES
*********************************************************************************************************
*/

#define  LIB_STR_CFG_FP_EN                DEF_DISABLED
#define  uC_PROBE_OS_PLUGIN               DEF_ENABLED
#define  uC_PROBE_COM_MODULE              DEF_ENABLED


/*
*********************************************************************************************************
*                                       TASK PRIORITIES
*********************************************************************************************************
*/

#define  APP_TASK_START_PRIO                    0                       /* Lower numbers are of higher priority                     */
// TODO declare task priority macros
#define APP_TASK_TOUCH_PRIO                     1
#define APP_TASK_MOTOR_PRIO                     2
#define APP_TASK_LCD_PRIO                       3
#define APP_TASK_LED_PRIO                       4

#define  OS_PROBE_TASK_PRIO                     8                       /* See probe_com_cfg for RS-232 communication task priority */
#define  OS_PROBE_TASK_ID                       8
#define  OS_TASK_TMR_PRIO                      10


/*
*********************************************************************************************************
*                                       TASK STACK SIZES
*
* Notes :   1) Warming, setting a stack size too small may result in the OS crashing. It the OS crashes
*              within a deep nested function call, the stack size may be to blame. The current maximum
*              stack usage for each task may be checked by using uC/OS-View or the stack checking
*              features of uC/OS-II.
*********************************************************************************************************
*/

#define  APP_TASK_START_STK_SIZE              128
// TODO declare task stack size macros
#define  APP_TASK_TOUCH_STK_SIZE              512
#define  APP_TASK_MOTOR_STK_SIZE              512
#define  APP_TASK_LCD_STK_SIZE                512
#define  APP_TASK_LED_STK_SIZE                128
#define  OS_PROBE_TASK_STK_SIZE               160                       /* See probe_com_cfg for RS-232 commication task stack size */


/*
*********************************************************************************************************
*                                          uC/LCD
*********************************************************************************************************
*/

#define  DISP_BUS_WIDTH                         8                       /* Data bus width: 4 or 8 bit                               */


/*
*********************************************************************************************************
*                               uC/Probe plug-in for uC/OS-II CONFIGURATION
*********************************************************************************************************
*/

#define  OS_PROBE_TASK                          1                       /* Task will be created for uC/Probe OS Plug-In             */
#define  OS_PROBE_TMR_32_BITS                   0                       /* Timer is 16 bits                                         */
#define  OS_PROBE_TIMER_SEL                     3                       /* Use TPM3 or TPM5                                         */
#define  OS_PROBE_HOOKS_EN                      1                       /* Enable uC/Probe application hooks                        */
#define  OS_PROBE_USE_FP                        1                       /* Override uC/Probe floating point support, use integers   */


#endif                                                                  /* End of file                                              */
