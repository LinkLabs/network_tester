
/* Scheduler includes. */
// #include "FreeRTOS.h"
// #include "task.h"

#include "debug_print.h"

#include "idle_task.h"

// #include "stack_check.h"
// #include "main.h"
// #include "timers.h"


/**************************************************************************//**
 * @brief vApplicationIdleHook
 * Override the default definition of vApplicationIdleHook()
 *****************************************************************************/
void vApplicationIdleHook(void)
{
#ifdef CHECK_STACK_USAGE
    static bool b_tasks_registered = false;

    if (!b_tasks_registered)
    {

        // Register Idle & Timer Task with stack check code
        TaskHandle_t task;
        task = xTaskGetIdleTaskHandle();

        // Task init complete. Register task and some metrics with main info structure
        task_info_t info;
        info.task_handle = task;
        info.stack_size = configMINIMAL_STACK_SIZE;
        info.stack_use_percentage = 0;

        register_task(TASK_IDLE, &info);

        task = xTimerGetTimerDaemonTaskHandle();
        info.task_handle = task;
        info.stack_size = configTIMER_TASK_STACK_DEPTH;
        info.stack_use_percentage = 0;

        register_task(TASK_TIMER, &info);

        b_tasks_registered = true;
    }
#endif
}

#if 0
/* Including only if tickless_idle is set to 1 or ( configUSE_TICKLESS_IDLE is set to 0  and  configUSE_SLEEP_MODE_IN_IDLE is set to 1 ) and EM2 or EM3 mode is choosed
 * in other hand standard Cortex M3 FreeRTOS functions are used. */
#if (((configUSE_TICKLESS_IDLE == 1) || ((configUSE_TICKLESS_IDLE == 0) && (configUSE_SLEEP_MODE_IN_IDLE == 1))) && (configSLEEP_MODE == 2 || configSLEEP_MODE == 3))
/* Constants required to pend a PendSV interrupt from the tick ISR if the
 * preemptive scheduler is being used.  These are just standard bits and registers
 * within the Cortex-M core itself. */
#define port_NVIC_INT_CTRL_REG     (*(( volatile unsigned long * ) 0xe000ed04))
#define port_NVIC_PENDSVSET_BIT    (1UL << 28UL)

#if (configUSE_TICKLESS_IDLE == 1)
/* Flag used only in EM2 and EM3 to get know whether
 * sleep mode was exited because of an interrupt */
static volatile bool intTickFlag = false;
#endif /* (configUSE_TICKLESS_IDLE == 1) */

/* Preload value for RTC and BURTC counter */
#define SYSTICK_LOAD_VALUE    ((configSYSTICK_CLOCK_HZ / configTICK_RATE_HZ))

/*
 * The number of SysTick increments that make up one tick period.
 */
static unsigned long ulTimerReloadValueForOneTick = 0;
/*
 * The maximum number of tick periods that can be suppressed is limited by the
 * 24 bit resolution RTC and 32 bit BURTC.
 */
#if (configUSE_TICKLESS_IDLE == 1)
static unsigned long xMaximumPossibleSuppressedTicks = 0;

/*
 * Compensate for the CPU cycles that pass while the SysTick is stopped (low
 * power functionality only.
 */

static unsigned long ulStoppedTimerCompensation = 0;
#endif /* (configUSE_TICKLESS_IDLE == 1) */

/* Functions which are used in EM2 mode*/
#if (configSLEEP_MODE == 2)
#define TIMER_CAPACITY        (0xFFFFFF)
#if (configUSE_TICKLESS_IDLE == 1)
#define TIMER_COMPENSATION    (45)
#endif /* (configUSE_TICKLESS_IDLE == 1) */

/**************************************************************************//**
 * @brief RTC_IRQHandler
 * Interrupt Service Routine for RTC which is used as system tick counter in EM2
 *****************************************************************************/
void RTC_IRQHandler(void)
{
  /* If using preemption, also force a context switch. */
#if (configUSE_PREEMPTION == 1)
  port_NVIC_INT_CTRL_REG = port_NVIC_PENDSVSET_BIT;
#endif /* (configUSE_PREEMPTION == 1) */
  /* Set RTC interrupt to one system tick period*/
  RTC_Enable(false);
  RTC_CompareSet(0, ulTimerReloadValueForOneTick);
  /* Restart the counter */
#if (configUSE_TICKLESS_IDLE == 1)
  /* Set flag that interrupt was made*/
  intTickFlag = true;
#endif /* (configUSE_TICKLESS_IDLE == 1) */

  /* Critical section which protect incrementing the tick*/
  ( void ) portSET_INTERRUPT_MASK_FROM_ISR();
  {
    xTaskIncrementTick();
  }
  portCLEAR_INTERRUPT_MASK_FROM_ISR(0);
  /* Clear interrupt */
  RTC_IntClear(_RTC_IFC_MASK);
  RTC_CounterReset();
}

/**************************************************************************//**
 * @brief vPortSetupTimerInterrupt
 * Override the default definition of vPortSetupTimerInterrupt() that is weakly
 * defined in the FreeRTOS Cortex-M3, which set source of system tick interrupt
 *****************************************************************************/
void vPortSetupTimerInterrupt(void)
{
  /* Set our data about timer used as system ticks*/
  ulTimerReloadValueForOneTick = SYSTICK_LOAD_VALUE ;
  #if (configUSE_TICKLESS_IDLE == 1)
  xMaximumPossibleSuppressedTicks = TIMER_CAPACITY / (SYSTICK_LOAD_VALUE);
  ulStoppedTimerCompensation      = TIMER_COMPENSATION / (configCPU_CLOCK_HZ / configSYSTICK_CLOCK_HZ);
#endif /* (configUSE_TICKLESS_IDLE == 1) */
  /* Configure RTC as system tick source */
  /* Structure of RTC init */
  RTC_Init_TypeDef init;
#if (configCRYSTAL_IN_EM2 == 1)
  /* LFXO setup */
  /* Starting LFXO and waiting until it is stable */
  CMU_OscillatorEnable(cmuOsc_LFXO, true, true);
#else
  /* RC oscillator */
  CMU_OscillatorEnable(cmuOsc_LFRCO, true, true);
#endif
  /* Ensure LE modules are accessible */
  CMU_ClockEnable(cmuClock_CORELE, true);
#if (configCRYSTAL_IN_EM2 == 1)
  /* Enable osc as LFACLK in CMU (will also enable oscillator if not enabled) */
  CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFXO);
#else
  /* Enable osc as LFACLK in CMU (will also enable oscillator if not enabled) */
  CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFRCO);
#endif
  /* Set 2 times divider to reduce energy*/
  CMU_ClockDivSet(cmuClock_RTC, cmuClkDiv_2);

  /* Enable clock to RTC module */
  CMU_ClockEnable(cmuClock_RTC, true);
  init.enable   = false;
  init.debugRun = false;
  init.comp0Top = false;         /* Count to max value before wrapping */
  /* Initialization of RTC */
  RTC_Init(&init);

  /* Disable interrupt generation from RTC0 */
  RTC_IntDisable(RTC_IFC_COMP0);

  /* Tick interrupt MUST execute at the lowest interrupt priority. */
  NVIC_SetPriority(RTC_IRQn, 255);

  /* Enable interrupts */
  NVIC_ClearPendingIRQ(RTC_IRQn);
  NVIC_EnableIRQ(RTC_IRQn);
  RTC_CompareSet(0, SYSTICK_LOAD_VALUE);
  RTC_IntClear(RTC_IFC_COMP0);
  RTC_IntEnable(RTC_IF_COMP0);
  RTC_Enable(true);
  //RTC_CounterReset();
}
#if (configUSE_TICKLESS_IDLE == 1)
/**************************************************************************//**
 * @brief vPortSuppressTicksAndSleep
 * Override the default definition of vPortSuppressTicksAndSleep() that is weakly
 * defined in the FreeRTOS Cortex-M3 port layer layer
 *****************************************************************************/
void vPortSuppressTicksAndSleep(portTickType xExpectedIdleTime)
{
  unsigned long ulReloadValue, ulCompleteTickPeriods;
  unsigned int ulRemainingCounter;

  portTickType  xModifiableIdleTime;
  /* Make sure the SysTick reload value does not overflow the counter. */
  if (xExpectedIdleTime > xMaximumPossibleSuppressedTicks)
  {
    xExpectedIdleTime = xMaximumPossibleSuppressedTicks;
  }

  /* Calculate the reload value required to wait xExpectedIdleTime
   * tick periods.
  */
  ulReloadValue = (ulTimerReloadValueForOneTick * (xExpectedIdleTime ));
  if (ulReloadValue > ulStoppedTimerCompensation)
  {
    ulReloadValue -= ulStoppedTimerCompensation;
  }

  /* Stop the System Tick momentarily.  The time the System Tick is stopped for
   * is accounted for as best it can be, but using the tickless mode will
   * inevitably result in some tiny drift of the time maintained by the
   * kernel with respect to calendar time. */

  /* Stop the RTC clock*/
  RTC_Enable(false);
/* Enter a critical section but don't use the taskENTER_CRITICAL()
 * method as that will mask interrupts that should exit sleep mode. */
  INT_Disable();

  /* The tick flag is set to false before sleeping.  If it is true when sleep
   * mode is exited then sleep mode was probably exited because the tick was
   * suppressed for the entire xExpectedIdleTime period. */
  intTickFlag = false;
  /* If a context switch is pending or a task is waiting for the scheduler
   * to be unsuspended then abandon the low power entry. */
  if (eTaskConfirmSleepModeStatus() == eAbortSleep)
  {
    RTC_Enable(true);
    /* Re-enable interrupts - see comments above __disable_interrupt()
     * call above. */
    INT_Enable();
  }
  else
  {
    /* Set the new reload value. */
    ulReloadValue -= RTC_CounterGet();
    RTC_CompareSet(0, ulReloadValue);
    /* Restart the counter*/
    RTC_CounterReset();
    /* Sleep until something happens.  configPRE_SLEEP_PROCESSING() can
     * set its parameter to 0 to indicate that its implementation contains
     * its own wait for interrupt or wait for event instruction, and so wfi
     * should not be executed again.  However, the original expected idle
     * time variable must remain unmodified, so a copy is taken. */
    xModifiableIdleTime = xExpectedIdleTime;
    configPRE_SLEEP_PROCESSING(xModifiableIdleTime);
    if (xModifiableIdleTime > 0)
    {
      SLEEP_Sleep();
      __DSB();
      __ISB();
    }
    configPOST_SLEEP_PROCESSING(xExpectedIdleTime);
    /* Stop SysTick.  Again, the time the SysTick is stopped for is
     * accounted for as best it can be, but using the tickless mode will
     * inevitably result in some tiny drift of the time maintained by the
     * kernel with respect to calendar time. */

    /* Store current counter value */
    ulRemainingCounter = RTC_CounterGet();
    /* Stop the RTC clock*/
    RTC_Enable(false);
    /* Re-enable interrupts */
    INT_Enable();
    if (intTickFlag != false)
    {
      /* The tick interrupt has already executed,
       * Reset the alarm value with whatever remains of this tick period. */
      RTC_CompareSet(0, TIMER_CAPACITY & (ulTimerReloadValueForOneTick - RTC_CounterGet()));

      /* The tick interrupt handler will already have pended the tick
       * processing in the kernel.  As the pending tick will be
       * processed as soon as this function exits, the tick value
       * maintained by the tick is stepped forward by one less than the
       * time spent waiting. */
      ulCompleteTickPeriods = xExpectedIdleTime - 1UL;
    }
    else
    {
      /* Some other interrupt than system tick ended the sleep.
       * Calculate how many tick periods passed while the processor
       * was waiting */
      ulCompleteTickPeriods = ulRemainingCounter / ulTimerReloadValueForOneTick;

      /* The reload value is set to whatever fraction of a single tick
       * period remains. */
      if (ulCompleteTickPeriods == 0)
      {
        ulReloadValue = ulTimerReloadValueForOneTick - ulRemainingCounter;
      }
      else
      {
        ulReloadValue = ulRemainingCounter - (ulCompleteTickPeriods * ulTimerReloadValueForOneTick);
      }
      RTC_CompareSet(0, ulReloadValue);
    }
    /* Restart the RTCounter */
    RTC_CounterReset();
    /* The tick forward by the number of tick periods that
     * remained in a low power state. */
    vTaskStepTick(ulCompleteTickPeriods);
  }
}
#endif /* (configUSE_TICKLESS_IDLE == 1) */
#endif /* (configSLEEP_MODE == 2) */

/* Functions which are used in EM3 mode*/
#if (configSLEEP_MODE == 3)

#define TIMER_CAPACITY        (0xFFFFFFFF)
#if (configUSE_TICKLESS_IDLE == 1)
#define TIMER_COMPENSATION    (45)
#endif /* (configUSE_TICKLESS_IDLE == 1) */
/**************************************************************************//**
 * @brief BURTC_IRQHandler
 * Interrupt Service Routine for RTC which is used as system tick counter in EM3
 *****************************************************************************/
void BURTC_IRQHandler(void)
{
  /* If using preemption, also force a context switch. */
#if (configUSE_PREEMPTION == 1)
  port_NVIC_INT_CTRL_REG = port_NVIC_PENDSVSET_BIT;
#endif /* (configUSE_PREEMPTION == 1) */
  /* Set BURTC interrupt to one system tick period*/
  BURTC_Enable(false);
  BURTC_CompareSet(0, ulTimerReloadValueForOneTick);
  /* Restart the counter */
  BURTC_CounterReset();
#if (configUSE_TICKLESS_IDLE == 1)
  /* Set flag that interrupt was made*/
  intTickFlag = true;
#endif /* (configUSE_TICKLESS_IDLE == 1) */
  /* Critical section which protect incrementing the tick*/
  ( void ) portSET_INTERRUPT_MASK_FROM_ISR();
  {
    xTaskIncrementTick();
  }
  portCLEAR_INTERRUPT_MASK_FROM_ISR(0);
  /* Clear interrupt */
  BURTC_IntClear(_RTC_IFC_MASK);
  BURTC_CounterReset();
}

/**************************************************************************//**
 * @brief vPortSetupTimerInterrupt
 * Override the default definition of vPortSetupTimerInterrupt() that is weakly
 * defined in the FreeRTOS Cortex-M3, which set source of system tick interrupt
 *****************************************************************************/
void vPortSetupTimerInterrupt(void)
{
  /* Set our timer's data used as system ticks*/
  ulTimerReloadValueForOneTick = SYSTICK_LOAD_VALUE;
#if (configUSE_TICKLESS_IDLE == 1)
  xMaximumPossibleSuppressedTicks = TIMER_CAPACITY / (SYSTICK_LOAD_VALUE);
  ulStoppedTimerCompensation      = TIMER_COMPENSATION / (configCPU_CLOCK_HZ / configSYSTICK_CLOCK_HZ);
#endif /* (configUSE_TICKLESS_IDLE == 1) */
  /* Ensure LE modules are accessible */
  CMU_ClockEnable(cmuClock_CORELE, true);

  /* Enable access to BURTC registers */
  RMU_ResetControl(rmuResetBU, false);

  /* Configure BURTC as system tick source */
  BURTC_Init_TypeDef burtcInit = BURTC_INIT_DEFAULT;
  burtcInit.mode   = burtcModeEM3;                      /* BURTC is enabled to EM3 */
  burtcInit.clkSel = burtcClkSelULFRCO;                 /* Select ULFRCO as clock source */
  burtcInit.clkDiv = burtcClkDiv_1;                     /* Choose 2kHz ULFRCO clock frequency */
  /* Initialization of BURTC */
  BURTC_Init(&burtcInit);

  /* Disable interrupt generation from BURTC */
  BURTC_IntDisable(BURTC_IF_COMP0);

  /* Tick interrupt MUST execute at the lowest interrupt priority. */
  NVIC_SetPriority(BURTC_IRQn, 255);

  /* Enable interrupts */
  NVIC_ClearPendingIRQ(BURTC_IRQn);
  NVIC_EnableIRQ(BURTC_IRQn);
  BURTC_CompareSet(0, SYSTICK_LOAD_VALUE);
  BURTC_IntClear(BURTC_IF_COMP0);
  BURTC_IntEnable(BURTC_IF_COMP0);
  BURTC_CounterReset();
}
#if (configUSE_TICKLESS_IDLE == 1)
/**************************************************************************//**
 * @brief vPortSetupTimerInterrupt
 * Override the default definition of vPortSuppressTicksAndSleep() that is weakly
 * defined in the FreeRTOS Cortex-M3 port layer layer
 *****************************************************************************/
void vPortSuppressTicksAndSleep(portTickType xExpectedIdleTime)
{
  unsigned long ulReloadValue, ulCompleteTickPeriods;
  portTickType  xModifiableIdleTime;
  /* Make sure the SysTick reload value does not overflow the counter. */
  if (xExpectedIdleTime > xMaximumPossibleSuppressedTicks)
  {
    xExpectedIdleTime = xMaximumPossibleSuppressedTicks;
  }

  /* Calculate the reload value required to wait xExpectedIdleTime
   * tick periods.  -1 is used because this code will execute part way
   * through one of the tick periods, and the fraction of a tick period is
   * accounted for later. */
  ulReloadValue = (ulTimerReloadValueForOneTick * (xExpectedIdleTime ));
  if (ulReloadValue > ulStoppedTimerCompensation)
  {
    ulReloadValue -= ulStoppedTimerCompensation;
  }

  /* Stop the SysTick momentarily.  The time the SysTick is stopped for
   * is accounted for as best it can be, but using the tickless mode will
   * inevitably result in some tiny drift of the time maintained by the
   * kernel with respect to calendar time. */
  /* Stop the RTC clock*/
  BURTC_Enable(false);
  /* Enter a critical section but don't use the taskENTER_CRITICAL()
   * method as that will mask interrupts that should exit sleep mode. */
  INT_Disable();
  /* The tick flag is set to false before sleeping.  If it is true when sleep
   * mode is exited then sleep mode was probably exited because the tick was
   * suppressed for the entire xExpectedIdleTime period. */
  intTickFlag = false;
  /* If a context switch is pending or a task is waiting for the scheduler
   * to be unsuspended then abandon the low power entry. */
  if (eTaskConfirmSleepModeStatus() == eAbortSleep)
  {
    BURTC_Enable(true);
    /* Re-enable interrupts */
    INT_Enable();
  }
  else
  {
    /* Set the new reload value. */
    ulReloadValue -= BURTC_CounterGet();
    BURTC_CompareSet(0, ulReloadValue);
    /* Restart the counter*/
    BURTC_CounterReset();
    /* Sleep until something happens.  configPRE_SLEEP_PROCESSING() can
     * set its parameter to 0 to indicate that its implementation contains
     * its own wait for interrupt or wait for event instruction, and so wfi
     * should not be executed again.  However, the original expected idle
     * time variable must remain unmodified, so a copy is taken. */
    xModifiableIdleTime = xExpectedIdleTime;
    configPRE_SLEEP_PROCESSING(xModifiableIdleTime);
    if (xModifiableIdleTime > 0)
    {
      SLEEP_Sleep();
      __DSB();
      __ISB();
    }
    configPOST_SLEEP_PROCESSING(xExpectedIdleTime);
    /* Stop SysTick.  Again, the time the SysTick is stopped for is
     * accounted for as best it can be, but using the tickless mode will
     * inevitably result in some tiny drift of the time maintained by the
     * kernel with respect to calendar time. */
    BURTC_Enable(false);
    /* Re-enable interrupts - see comments above __disable_interrupt()
     * call above. */
    INT_Enable();
    if (intTickFlag != false)
    {
      /* The tick interrupt has already executed,
       * Reset the alarm value with whatever remains of this tick period. */
      BURTC_CompareSet(0, TIMER_CAPACITY & (ulTimerReloadValueForOneTick - BURTC_CounterGet()));
      /* The tick interrupt handler will already have pended the tick
       * processing in the kernel.  As the pending tick will be
       * processed as soon as this function exits, the tick value
       * maintained by the tick is stepped forward by one less than the
       * time spent waiting. */
      ulCompleteTickPeriods = xExpectedIdleTime - 1UL;
    }
    else
    {
      /* Some other interrupt than system tick ended the sleep.
       * Calculate how many tick periods passed while the processor
       * was waiting */
      ulCompleteTickPeriods = BURTC_CounterGet() / ulTimerReloadValueForOneTick;

      /* The reload value is set to whatever fraction of a single tick
       * period remains. */
      if (ulCompleteTickPeriods == 0)
      {
        ulReloadValue = ulTimerReloadValueForOneTick - BURTC_CounterGet();
      }
      else
      {
        ulReloadValue = BURTC_CounterGet() - (ulCompleteTickPeriods * ulTimerReloadValueForOneTick);
      }
      BURTC_CompareSet(0, ulReloadValue);
    }
    /* Restart the RTCounter*/
    BURTC_CounterReset();
    /* The tick forward by the number of tick periods that
     * remained in a low power state. */
    vTaskStepTick(ulCompleteTickPeriods);
  }
}
#endif /* (configUSE_TICKLESS_IDLE == 1) */
#endif /* (configSLEEP_MODE == 3) */
#endif /* (((configUSE_TICKLESS_IDLE == 1) ||  (( configUSE_TICKLESS_IDLE == 0 ) && ( configUSE_SLEEP_MODE_IN_IDLE == 1 ))) && (configSLEEP_MODE == 2 || configSLEEP_MODE == 3)) */


#endif
