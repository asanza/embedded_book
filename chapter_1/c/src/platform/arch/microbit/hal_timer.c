#include <hal_timer.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <assert.h>

/* following defines should be used for structure members */
/*! Defines 'read only' structure member permissions */
#define __IM volatile const
/*! Defines 'write only' structure member permissions */
#define __OM volatile
/*! Defines 'read / write' structure member permissions */
#define __IOM volatile

#define __WFI()         __asm volatile ("wfi":::"memory")

/**
  \brief   Enable IRQ Interrupts
  \details Enables IRQ interrupts by clearing special-purpose register PRIMASK.
           Can only be executed in Privileged modes.
 */
static inline void __enable_irq(void)
{
  __asm volatile ("cpsie i" : : : "memory");
}


struct NRF_TIMER {           /*!< (@ 0x40008000) TIMER0 Structure           */
  __OM uint32_t TASKS_START; /*!< (@ 0x00000000) Start Timer. */
  __OM uint32_t TASKS_STOP;  /*!< (@ 0x00000004) Stop Timer.  */
  /*!< (@ 0x00000008) Increment Timer (In counter mode). */
  __OM uint32_t TASKS_COUNT;
  __OM uint32_t TASKS_CLEAR;    /*!< (@ 0x0000000C) Clear timer.    */
  __OM uint32_t TASKS_SHUTDOWN; /*!< (@ 0x00000010) Shutdown timer. */
  __IM uint32_t RESERVED[11];
  /*!< (@ 0x00000040) Capture Timer value to CC[n] registers. */
  __OM uint32_t TASKS_CAPTURE[4];
  __IM uint32_t RESERVED1[60];
  /*!< (@ 0x00000140) Compare event on CC[n] match. */
  __IOM uint32_t EVENTS_COMPARE[4];
  __IM uint32_t RESERVED2[44];
  __IOM uint32_t SHORTS; /*!< (@ 0x00000200) Shortcuts for Timer. */
  __IM uint32_t RESERVED3[64];
  __IOM uint32_t INTENSET; /*!< (@ 0x00000304) Interrupt enable set register. */
  __IOM uint32_t
      INTENCLR; /*!< (@ 0x00000308) Interrupt enable clear register. */
  __IM uint32_t RESERVED4[126];
  __IOM uint32_t MODE;    /*!< (@ 0x00000504) Timer Mode selection.    */
  __IOM uint32_t BITMODE; /*!< (@ 0x00000508) Sets timer behaviour. */
  __IM uint32_t RESERVED5;
  /*!< (@ 0x00000510) 4-bit prescaler to source clock frequency (max
                                                value 9). Source clock frequency
     is divided by 2^SCALE.                                                */
  __IOM uint32_t PRESCALER;
  __IM uint32_t RESERVED6[11];
  __IOM uint32_t CC[4]; /*!< (@ 0x00000540) Capture/compare registers. */
  __IM uint32_t RESERVED7[683];
  __IOM uint32_t POWER; /*!< (@ 0x00000FFC) Peripheral power control. */
};       /*!< Size = 4096 (0x1000)       */

/**
  \brief  Structure type to access the Nested Vectored Interrupt Controller (NVIC).
 */
struct NVIC
{
  __IOM uint32_t ISER[1U];               /*!< Offset: 0x000 (R/W)  Interrupt Set Enable Register */
        uint32_t RESERVED0[31U];
  __IOM uint32_t ICER[1U];               /*!< Offset: 0x080 (R/W)  Interrupt Clear Enable Register */
        uint32_t RESERVED1[31U];
  __IOM uint32_t ISPR[1U];               /*!< Offset: 0x100 (R/W)  Interrupt Set Pending Register */
        uint32_t RESERVED2[31U];
  __IOM uint32_t ICPR[1U];               /*!< Offset: 0x180 (R/W)  Interrupt Clear Pending Register */
        uint32_t RESERVED3[31U];
        uint32_t RESERVED4[64U];
  __IOM uint32_t IPR[8U];                /*!< Offset: 0x300 (R/W)  Interrupt Priority Register */
};

#define NRF_TIMER0_BASE 0x40008000UL
#define NRF_TIMER1_BASE 0x40009000UL
#define NRF_TIMER2_BASE 0x4000A000UL

#define TIMER0 ((struct NRF_TIMER* ) NRF_TIMER0_BASE)
#define TIMER1 ((struct NRF_TIMER* ) NRF_TIMER1_BASE)
#define TIMER2 ((struct NRF_TIMER* ) NRF_TIMER2_BASE)

#define TIMER_MODE_MODE_Timer (0UL) /*!< Timer in Normal mode. */
#define TIMER_MODE_MODE_Counter (1UL) /*!< Timer in Counter mode. */
#define TIMER_BITMODE_BITMODE_32Bit (0x03UL) /*!< 32-bit timer behaviour. */

#define TIMER_INTENSET_COMPARE0_Pos (16UL) /*!< Position of COMPARE0 field. */
#define TIMER_INTENSET_COMPARE0_Enabled (1UL) /*!< Interrupt enabled. */
#define TIMER_INTENSET_COMPARE0_Set (1UL) /*!< Enable interrupt on write. */

#define TIMER0_IRQn ( 8U)         /*!< 8  TIMER0                                                                 */
#define TIMER1_IRQn ( 9U)         /*!< 9  TIMER1                                                                 */
#define TIMER2_IRQn (10U)         /*!< 10 TIMER2                                                                 */

#define NVIC_BASE           (0xE000E000UL +  0x0100UL)                    /*!< NVIC Base Address */
#define NVIC0 ((struct NVIC *)(NVIC_BASE))

static inline void NVIC_EnableIRQ(uint32_t IRQn)
{
  if ((int32_t)(IRQn) >= 0)
  {
    __asm volatile("":::"memory");
    NVIC0->ISER[0U] |= (uint32_t)(1UL << (((uint32_t)IRQn) & 0x1FUL));
    __asm volatile("":::"memory");
  }
}

struct my_timer {
  struct NRF_TIMER *const hw_timer;
  bool one_shoot;
};

static struct my_timer instances[] = {
  {.hw_timer = TIMER0, .one_shoot = false },
  {.hw_timer = TIMER1, .one_shoot = false },
  {.hw_timer = TIMER2, .one_shoot = false },
};

void hal_timer_init(int timer_instance, bool one_shoot) {
    assert((unsigned)timer_instance < (sizeof(instances)/sizeof(instances[0])));

    struct my_timer *tmr = &instances[timer_instance];

    tmr->one_shoot = one_shoot;

    tmr->hw_timer->INTENSET =
        (TIMER_INTENSET_COMPARE0_Enabled << TIMER_INTENSET_COMPARE0_Pos);

    NVIC_EnableIRQ(TIMER0_IRQn + timer_instance);
    __enable_irq();
}

void hal_timer_delay_ms(int timer_instance, uint32_t period_ms) {
    assert((unsigned)timer_instance < (sizeof(instances)/sizeof(instances[0])));

    struct my_timer *tmr = &instances[timer_instance];

    tmr->hw_timer->MODE = TIMER_MODE_MODE_Timer; // Set the timer in Timer Mode
    tmr->hw_timer->TASKS_CLEAR = 1; // clear the task first to be usable for later
    tmr->hw_timer->PRESCALER   = 8; // Set prescaler. Higher number gives slower
               // timer. Prescaler = 0 gives 16MHz timer
    tmr->hw_timer->BITMODE = TIMER_BITMODE_BITMODE_32Bit; // Set counter to 32 bit resolution
    /* This is not that important right now. QEMU is not clock-accurate. */
    uint32_t period = (period_ms  * 50);
    tmr->hw_timer->CC[0] = period;
    (void) period_ms;

    tmr->hw_timer->TASKS_START = 1; // Start TIMER
    __WFI();
}

void
Timer0_IRQHandler(void)
{
    struct my_timer *tmr = &instances[0];
    assert(tmr->hw_timer == TIMER0);

    // Check if the interrupt was triggered by CC[0] or CC[1]
    if (tmr->hw_timer->EVENTS_COMPARE[0]) {
        // Clear the event
      tmr->hw_timer->EVENTS_COMPARE[0] = 0;
    }

    if(tmr->one_shoot){
      tmr->hw_timer->TASKS_STOP = 1;
    }
}
