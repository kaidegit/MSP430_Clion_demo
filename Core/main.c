#include <stdint.h>
#include "driverlib/5xx_6xx/gpio.h"
#include "driverlib/5xx_6xx/wdt.h"
#include "inc/hw_memmap.h"

int main() {
    volatile uint32_t i;
    WDT_hold(WDT_A_BASE);
    GPIO_setAsOutputPin(P4_BASE, GPIO_PORT_P4, GPIO_PIN4 | GPIO_PIN5 | GPIO_PIN6);
    while (1) {
        GPIO_toggleOutputOnPin(P4_BASE, GPIO_PORT_P4, GPIO_PIN4 | GPIO_PIN5 | GPIO_PIN6);
        for (i = 0; i < 20000; i++) {}
    }
}