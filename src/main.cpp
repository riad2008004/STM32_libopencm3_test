#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/timer.h>

// Function to convert integer to string
void int_to_string(uint16_t value, char *buffer)
{
    int i = 0;
    if (value == 0)
    {
        buffer[i++] = '0';
    }
    else
    {
        while (value > 0)
        {
            buffer[i++] = '0' + (value % 10);
            value /= 10;
        }
    }
    buffer[i] = '\0';

    // Reverse the string
    for (int j = 0; j < i / 2; j++)
    {
        char temp = buffer[j];
        buffer[j] = buffer[i - j - 1];
        buffer[i - j - 1] = temp;
    }
}

void clock_setup(void)
{
    // Set up system clock to 72 MHz
    rcc_clock_setup_pll(&rcc_hse_configs[RCC_CLOCK_HSE8_72MHZ]);

    // Enable peripheral clocks
    rcc_periph_clock_enable(RCC_GPIOA);  // Enable GPIOA clock
    rcc_periph_clock_enable(RCC_GPIOB);  // Enable GPIOB clock
    rcc_periph_clock_enable(RCC_USART1); // Enable USART1 clock
    rcc_periph_clock_enable(RCC_ADC1);   // Enable ADC1 clock
    rcc_periph_clock_enable(RCC_TIM3);   // Enable Timer3 clock
}

void gpio_setup(void)
{
    // Digital Output (LED on PA5)
    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_2_MHZ,
                  GPIO_CNF_OUTPUT_PUSHPULL, GPIO5);

    // Digital Input (Button on PB10)
    gpio_set_mode(GPIOB, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO10);

    // Analog Input (ADC on PA0)
    gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_ANALOG, GPIO0);

    // USART1 TX (PA9) and RX (PA10)
    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
                  GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO9);                // TX
    gpio_set_mode(GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO10); // RX

    // PWM Output (PB6 for TIM3 CH1)
    gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ,
                  GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO6);
}

void usart_setup(void)
{
    usart_set_baudrate(USART1, 9600);
    usart_set_databits(USART1, 8);
    usart_set_stopbits(USART1, USART_STOPBITS_1);
    usart_set_mode(USART1, USART_MODE_TX_RX);
    usart_set_parity(USART1, USART_PARITY_NONE);
    usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);
    usart_enable(USART1);
}

void adc_setup(void)
{
    adc_power_off(ADC1);
    adc_enable_scan_mode(ADC1);
    adc_set_single_conversion_mode(ADC1);
    adc_set_sample_time_on_all_channels(ADC1, ADC_SMPR_SMP_239DOT5CYC);
    adc_power_on(ADC1);
    adc_reset_calibration(ADC1);
    adc_calibrate(ADC1);
}

void pwm_setup(void)
{
    // Set up Timer3 for PWM
    timer_set_mode(TIM3, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
    timer_set_prescaler(TIM3, 72 - 1); // 72 MHz / 72 = 1 MHz timer clock
    timer_set_period(TIM3, 1000 - 1);  // PWM frequency of 1 kHz

    // Configure Timer3 Channel 1 (PB6) for PWM
    timer_set_oc_mode(TIM3, TIM_OC1, TIM_OCM_PWM1);
    timer_enable_oc_output(TIM3, TIM_OC1);
    timer_set_oc_value(TIM3, TIM_OC1, 500); // Set initial duty cycle to 50%

    // Enable Timer3 counter
    timer_enable_counter(TIM3);
}

void delay_ms(uint32_t ms)
{
    for (uint32_t i = 0; i < ms * 8000; i++)
    {
        __asm__("nop");
    }
}

int main(void)
{
    clock_setup();
    gpio_setup();
    usart_setup();
    adc_setup();
    pwm_setup();

    uint16_t adc_value;
    char buffer[16];

    while (1)
    {
        // Read button state
        bool button_pressed = !gpio_get(GPIOB, GPIO10);

        // Control LED based on button state
        if (button_pressed)
        {
            gpio_set(GPIOA, GPIO5); // Turn LED on
        }
        else
        {
            gpio_clear(GPIOA, GPIO5); // Turn LED off
        }

        // Read ADC value
        uint8_t channel = 0;
        adc_set_regular_sequence(ADC1, 1, &channel);
        adc_start_conversion_direct(ADC1);
        while (!(adc_eoc(ADC1)))
            ;
        adc_value = adc_read_regular(ADC1);

        // Convert ADC value to string and send over USART
        int_to_string(adc_value, buffer);
        for (char *p = buffer; *p; p++)
        {
            usart_send_blocking(USART1, *p);
        }
        usart_send_blocking(USART1, '\r');
        usart_send_blocking(USART1, '\n');

        // Update PWM duty cycle based on ADC value
        uint16_t duty_cycle = (adc_value * 1000) / 4095; // Map ADC to 0–1000
        timer_set_oc_value(TIM3, TIM_OC1, duty_cycle);

        delay_ms(100);
    }

    return 0;
}
