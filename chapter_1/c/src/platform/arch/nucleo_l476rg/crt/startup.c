/*
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 * Copyright(c) 2023 Diego Asanza <f.asanza@gmail.com>
 */

#include <stdint.h>
#include <errno.h>
#include <sys/stat.h>
#include <unistd.h>


#define WEAK __attribute__((weak))
#define ALIAS(f) __attribute__((weak, alias(#f)))

#define CPACR 0xE000ED88
#define ICSR 0xE000ED04
#define VTOR 0xE000ED08
#define CCR 0xE000ED14
#define HFSR 0xE000ED2C

#define REGADDR(x) ((unsigned int *)x)

extern void __StackTop(void);

extern unsigned int __bss_start__;
extern unsigned int __bss_end__;
extern unsigned int __data_start__;
extern unsigned int __data_end__;
extern unsigned int __etext;

void fault_handler_c(unsigned int *stack);
extern void initialise_monitor_handles(void);

__attribute__((naked)) void HardFault_Handler(void);
void Default_Handler(void);

void NMI_Handler(void) ALIAS(Default_Handler);
void MemManage_Handler(void) ALIAS(Default_Handler);
void BusFault_Handler(void) ALIAS(Default_Handler);
void UsageFault_Handler(void) ALIAS(Default_Handler);
void SecureFault_Handler(void) ALIAS(Default_Handler);
void SVC_Handler(void) ALIAS(Default_Handler);
void DebugMon_Handler(void) ALIAS(Default_Handler);
void PendSV_Handler(void) ALIAS(Default_Handler);
void SysTick_Handler(void) ALIAS(Default_Handler);

/* STM32L476 external interrupts (fill to reach TIM2). */
void WWDG_Handler(void) ALIAS(Default_Handler);
void PVD_PVM_Handler(void) ALIAS(Default_Handler);
void TAMP_STAMP_Handler(void) ALIAS(Default_Handler);
void RTC_WKUP_Handler(void) ALIAS(Default_Handler);
void FLASH_Handler(void) ALIAS(Default_Handler);
void RCC_Handler(void) ALIAS(Default_Handler);
void EXTI0_Handler(void) ALIAS(Default_Handler);
void EXTI1_Handler(void) ALIAS(Default_Handler);
void EXTI2_Handler(void) ALIAS(Default_Handler);
void EXTI3_Handler(void) ALIAS(Default_Handler);
void EXTI4_Handler(void) ALIAS(Default_Handler);
void DMA1_CH1_Handler(void) ALIAS(Default_Handler);
void DMA1_CH2_Handler(void) ALIAS(Default_Handler);
void DMA1_CH3_Handler(void) ALIAS(Default_Handler);
void DMA1_CH4_Handler(void) ALIAS(Default_Handler);
void DMA1_CH5_Handler(void) ALIAS(Default_Handler);
void DMA1_CH6_Handler(void) ALIAS(Default_Handler);
void DMA1_CH7_Handler(void) ALIAS(Default_Handler);
void ADC1_2_Handler(void) ALIAS(Default_Handler);
void CAN1_TX_Handler(void) ALIAS(Default_Handler);
void CAN1_RX0_Handler(void) ALIAS(Default_Handler);
void CAN1_RX1_Handler(void) ALIAS(Default_Handler);
void CAN1_SCE_Handler(void) ALIAS(Default_Handler);
void EXTI9_5_Handler(void) ALIAS(Default_Handler);
void TIM1_BRK_TIM15_Handler(void) ALIAS(Default_Handler);
void TIM1_UP_TIM16_Handler(void) ALIAS(Default_Handler);
void TIM1_TRG_COM_TIM17_Handler(void) ALIAS(Default_Handler);
void TIM1_CC_Handler(void) ALIAS(Default_Handler);

extern void TIM2_IRQHandler(void);

void reset_handler(void);
void main(void);

__attribute__((
    used, section(".isr_vector"))) void (*const g_interrupt_vector[])(void) = {
    &__StackTop,                // 0: initial stack pointer
    reset_handler,              // 1: reset
    NMI_Handler,                // 2
    HardFault_Handler,          // 3
    MemManage_Handler,          // 4
    BusFault_Handler,           // 5
    UsageFault_Handler,         // 6
    SecureFault_Handler,        // 7
    0,                          // 8
    0,                          // 9
    0,                          // 10
    SVC_Handler,                // 11
    DebugMon_Handler,           // 12
    0,                          // 13
    PendSV_Handler,             // 14
    SysTick_Handler,            // 15
    WWDG_Handler,               // 16
    PVD_PVM_Handler,            // 17
    TAMP_STAMP_Handler,         // 18
    RTC_WKUP_Handler,           // 19
    FLASH_Handler,              // 20
    RCC_Handler,                // 21
    EXTI0_Handler,              // 22
    EXTI1_Handler,              // 23
    EXTI2_Handler,              // 24
    EXTI3_Handler,              // 25
    EXTI4_Handler,              // 26
    DMA1_CH1_Handler,           // 27
    DMA1_CH2_Handler,           // 28
    DMA1_CH3_Handler,           // 29
    DMA1_CH4_Handler,           // 30
    DMA1_CH5_Handler,           // 31
    DMA1_CH6_Handler,           // 32
    DMA1_CH7_Handler,           // 33
    ADC1_2_Handler,             // 34
    CAN1_TX_Handler,            // 35
    CAN1_RX0_Handler,           // 36
    CAN1_RX1_Handler,           // 37
    CAN1_SCE_Handler,           // 38
    EXTI9_5_Handler,            // 39
    TIM1_BRK_TIM15_Handler,     // 40
    TIM1_UP_TIM16_Handler,      // 41
    TIM1_TRG_COM_TIM17_Handler, // 42
    TIM1_CC_Handler,            // 43
    TIM2_IRQHandler,            // 44 (TIM2 global interrupt)
};

void reset_handler(void) {
  __asm__ volatile("cpsid i");

  /* Remap the interrupt vector */
  *REGADDR(VTOR) = (unsigned int)g_interrupt_vector;

  /* enable the FPU */
  *REGADDR(CPACR) = (0x0FU) << 20;

  /* enable div by zero trap */
  *REGADDR(CCR) |= (1 << 4);

  /* now we initialize data in ram from values saved in flash */
  volatile unsigned int *it = &__data_start__;
  volatile unsigned int *dr = &__etext;

  while (it < &__data_end__) {
    *it++ = *dr++;
  }

  /* clear the bss section */
  it = &__bss_start__;
  while (it < &__bss_end__) {
    *it++ = 0;
  }

  __asm__ volatile("cpsie i");

  main();

  while (1)
    ;
}

void HardFault_Handler(void) {
  while (1)
    ;
}

void Default_Handler(void) {
  /*
   * If we are here, chances are that we triggered an unhandled exception
   * handler. Read the active interrupt number bellow.
   */
  volatile __attribute((unused)) uint32_t vector = *REGADDR(ICSR) & 0xFFU;
  while (1)
    ;
}

/* stubs for library functions. Avoid libnano link warnings. */
int _close(int fd) {
  (void)fd;
  errno = ENOSYS;
  return -1;
}
int _fstat(int fd, struct stat *st) {
  (void)fd;
  st->st_mode = S_IFCHR;
  return 0;
}
int _isatty(int fd) {
  (void)fd;
  return 1;
}
off_t _lseek(int fd, off_t off, int whence) {
  (void)fd;
  (void)off;
  (void)whence;
  errno = ENOSYS;
  return -1;
}
ssize_t _read(int fd, void *buf, size_t len) {
  (void)fd;
  (void)buf;
  (void)len;
  errno = ENOSYS;
  return -1;
}
ssize_t _write(int fd, const void *buf, size_t len) {
  (void)fd;
  (void)buf;
  (void)len;
  errno = ENOSYS;
  return -1;
}
int _getpid(void) { return 1; }
int _kill(int pid, int sig) {
  (void)pid;
  (void)sig;
  errno = ENOSYS;
  return -1;
}