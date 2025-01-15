#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>

void delay(int count)
{
    for (int i = 0; i < count; i++)
    {
        __asm__("nop"); // add comment
    }
}

int main(void)
{
    // Enable GPIOC clock
    rcc_periph_clock_enable(RCC_GPIOC);

    // Set GPIOC pin 13 as output
    gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_10_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO13);

    while (1)
    {
        // Toggle the LED
        gpio_toggle(GPIOC, GPIO13);

        // Delay
        delay(1000000);
    }

    return 0;
}
