#include <hal_gpio.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

/* following defines should be used for structure members */
/*! Defines 'read only' structure member permissions */
#define __IM volatile const
/*! Defines 'write only' structure member permissions */
#define __OM volatile
/*! Defines 'read / write' structure member permissions */
#define __IOM volatile

#define NRF_GPIO_BASE 0x50000000UL

/**
 * @brief General purpose input and output. (GPIO)
 */

struct NRF_GPIO { /*!< (@ 0x50000000) GPIO Structure */
  __IM uint32_t RESERVED[321];
  __IOM uint32_t OUT; /*!< (@ 0x00000504) Write GPIO port. */
  /*!< (@ 0x00000508) Set individual bits in GPIO port. */
  __IOM uint32_t OUTSET;
  /*!< (@ 0x0000050C) Clear individual bits in GPIO port.         */
  __IOM uint32_t OUTCLR;
  __IM uint32_t IN;      /*!< (@ 0x00000510) Read GPIO port.   */
  __IOM uint32_t DIR;    /*!< (@ 0x00000514) Direction of GPIO pins. */
  __IOM uint32_t DIRSET; /*!< (@ 0x00000518) DIR set register. */
  __IOM uint32_t DIRCLR; /*!< (@ 0x0000051C) DIR clear register. */
  __IM uint32_t RESERVED1[120];
  __IOM uint32_t PIN_CNF[32]; /*!< (@ 0x00000700) Configuration of GPIO pins. */
};              /*!< Size = 1920 (0x780)              */

static struct NRF_GPIO *const GPIO0 = (struct NRF_GPIO*)(NRF_GPIO_BASE);

void hal_gpio_init_out(int pin, bool od, int value) {
    (void) od;
    GPIO0->DIR |= (1u << pin); // Set pin as output without disturbing others
    if (value) {
        GPIO0->OUTSET = (1u << pin);
    } else {
        GPIO0->OUTCLR = (1u << pin);
    }
}

void hal_gpio_write(int pin, int value) {
    if (value) {
        GPIO0->OUTSET = (1u << pin);
    } else {
        GPIO0->OUTCLR = (1u << pin);
    }
    printf("GPIO pin %d = %d\n", pin, 0 != (GPIO0->OUT & (1 << pin)));
}
