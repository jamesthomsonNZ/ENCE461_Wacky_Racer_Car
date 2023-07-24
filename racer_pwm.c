/* File:   pwm_test2.c
   Author: M. P. Hayes, UCECE
   Date:   15 April 2013
   Descr:  This example starts two channels simultaneously; one inverted
           with respect to the other.
*/

/* File:   radio_rx_test1.c
   Author: M. P. Hayes, UCECE
   Date:   24 Feb 2018
*/
#include "nrf24.h"
#include "spi.h"


#include <stdio.h>
#include <string.h>
#include "pwm.h"
#include "pio.h"
#include "delay.h"
#include "panic.h"
#include "usb_serial.h"


#define RADIO_CHANNEL 1
#define RADIO_ADDRESS 0x0123456789LL
#define RADIO_PAYLOAD_SIZE 32

int main(void)
{
    spi_cfg_t spi_cfg =
        {
            .channel = 0,
            .clock_speed_kHz = 1000,
            .cs = RADIO_CS_PIO,
            .mode = SPI_MODE_0,
            .cs_mode = SPI_CS_MODE_FRAME,
            .bits = 8
        };
    nrf24_cfg_t nrf24_cfg =
        {
            .channel = RADIO_CHANNEL,
            .address = RADIO_ADDRESS,
            .payload_size = RADIO_PAYLOAD_SIZE,
            .ce_pio = RADIO_CE_PIO,
            .irq_pio = RADIO_IRQ_PIO,
            .spi = spi_cfg,
        };
    nrf24_t *nrf;

    // Configure LED PIO as output.
    pio_config_set (LED_ERROR_PIO, PIO_OUTPUT_HIGH);
    pio_config_set (LED_BLUE_PIO, PIO_OUTPUT_HIGH);
    pio_config_set (LED_STATUS_PIO, PIO_OUTPUT_HIGH);

    // Redirect stdio to USB serial.
    usb_serial_stdio_init ();

#ifdef RADIO_POWER_ENABLE_PIO
    // Enable radio regulator if present.
    pio_config_set (RADIO_POWER_ENABLE_PIO, PIO_OUTPUT_HIGH);
    delay_ms (10);
#endif

    nrf = nrf24_init (&nrf24_cfg);
    if (! nrf)
        panic (LED_ERROR_PIO, 2);

    while(1)
    {
        char buffer[RADIO_PAYLOAD_SIZE + 1];
        uint8_t bytes;

        bytes = nrf24_read (nrf, buffer, RADIO_PAYLOAD_SIZE);
        if (bytes != 0)
        {
            buffer[bytes] = 0;
            printf ("%s\n", buffer);
            pio_output_toggle (LED_STATUS_PIO);
        }
    }
}



#define PWM1_PIO PA20_PIO
#define PWM2_PIO PA2_PIO

#define PWM_FREQ_HZ 100e3

#define DELAY_MS 10

static const pwm_cfg_t pwm1_cfg =
{
    .pio = PWM1_PIO,
    .period = PWM_PERIOD_DIVISOR (PWM_FREQ_HZ),
    .duty = PWM_DUTY_DIVISOR (PWM_FREQ_HZ, 50),
    .align = PWM_ALIGN_LEFT,
    .polarity = PWM_POLARITY_HIGH,
    .stop_state = PIO_OUTPUT_LOW
};

static const pwm_cfg_t pwm2_cfg =
{
    .pio = PWM2_PIO,
    .period = PWM_PERIOD_DIVISOR (PWM_FREQ_HZ),
    .duty = PWM_DUTY_DIVISOR (PWM_FREQ_HZ, 50),
    .align = PWM_ALIGN_LEFT,
    .polarity = PWM_POLARITY_HIGH,
    .stop_state = PIO_OUTPUT_LOW
};


int
main (void)
{
    pwm_t pwm1;
    pwm_t pwm2;

    pio_config_set (LED_STATUS_PIO, PIO_OUTPUT_HIGH);

    // Redirect stdio to USB serial
    if (usb_serial_stdio_init () < 0)
        panic(LED_ERROR_PIO, 3);


    pwm1 = pwm_init (&pwm1_cfg);
    if (! pwm1)
        panic (LED_ERROR_PIO, 1);

    pwm2 = pwm_init (&pwm2_cfg);
    if (! pwm2)
        panic (LED_ERROR_PIO, 2);

    pwm_channels_start (pwm_channel_mask (pwm1) | pwm_channel_mask (pwm2));

    while (1)
    {
        delay_ms (500);
        pio_output_toggle (LED_STATUS_PIO);
        static uint32_t requested_duty_cycle = 0;
        static uint32_t requested_duty_percent = 0;
        static uint32_t duty_cycle;

        delay_ms (DELAY_MS);
        char buf[256];
        if (fgets(buf, sizeof(buf), stdin)) {

            // sscanf returns the number of input items successfully matched
            if (sscanf(buf, "%u",&requested_duty_percent) == 1) {
                if  (requested_duty_percent >= 0 && requested_duty_percent <= 100){
                    requested_duty_cycle = (requested_duty_percent*96)/10;
                  if (duty_cycle != requested_duty_cycle){
                    printf("Duty cycle set to: %u", requested_duty_percent);
                  }  
                  duty_cycle = requested_duty_cycle;
                  printf("%d", duty_cycle);
                  
                } else {
                    printf("Grim...");
                }
            } else {
                printf("Invalid input\n");
            }
        }

        pwm_duty_set(pwm1, duty_cycle);
        pwm_duty_set(pwm2, duty_cycle);

    }

    return 0;
}
