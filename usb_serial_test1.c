/* File:   pwm_test2.c
   Author: M. P. Hayes, UCECE
   Date:   15 April 2013
   Descr:  This example starts two channels simultaneously; one inverted
           with respect to the other.
*/

#include <stdio.h>
#include <string.h>
#include "pwm.h"
#include "pio.h"
#include "delay.h"
#include "panic.h"
#include "usb_serial.h"

#define PWM1_PIO PA0_PIO
#define PWM2_PIO PA2_PIO
#define BIN1 PA1_PIO
#define AIN1 PA7_PIO

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
    pio_config_set (AIN1, PIO_OUTPUT_LOW);
    pio_config_set (BIN1, PIO_OUTPUT_LOW);

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

    pwm_duty_set(pwm1, 0);
    pwm_duty_set(pwm2, 0);
    printf("Start\n");


    while (1)
    {
        delay_ms (500);
        pio_output_toggle (LED_STATUS_PIO);

        delay_ms (DELAY_MS);
        static uint32_t requested_duty_cycle = 0;
        static uint32_t requested_duty_percent = 0;
        static uint32_t duty_cycle = 0;
        char motor;
        char req_motor;
        char dir;
        char req_dir;
        char buf[256];
        if (fgets(buf, sizeof(buf), stdin)) {

            // sscanf returns the number of input items successfully matched
            if (sscanf(buf, "%c %u %c",&req_motor, &requested_duty_percent, &req_dir) == 3) {
                
                if  (requested_duty_percent >= 0 && requested_duty_percent <= 100){

                if (req_dir == 'f'){

                    requested_duty_cycle = (requested_duty_percent*96)/10;
                } else if (req_dir == 'r'){
                    requested_duty_cycle = 960-(requested_duty_percent*96)/10;
                }

                if (duty_cycle != requested_duty_cycle || motor != req_motor || dir != req_dir){

                    if (motor == 'a') {
                        printf("A duty cycle set to: %u\n", requested_duty_percent);
                    } else if (motor == 'b'){
                        printf("B duty cycle set to: %u\n", requested_duty_percent);
                    }
                }

                  duty_cycle = requested_duty_cycle;
                  motor = req_motor;
                  dir = req_dir;

                    if (motor == 'a') {
                        pwm_duty_set(pwm2, duty_cycle);

                        if (dir == 'f'){
                            pio_output_low(AIN1);
                        } else if (dir == 'r'){
                            pio_output_high(AIN1);
                        }

                    } else if (motor == 'b'){
                        pwm_duty_set(pwm1, duty_cycle);
                        if (dir == 'f'){
                            pio_output_low(BIN1);
                        } else if (dir == 'r'){
                            pio_output_high(BIN1);
                        }

                    } else {
                        printf("Invalid motor");
                    }

                } else {
                  printf("Grim...\n");
                }
            } else {
                printf("Invalid input\n");
            }
        }

        //pwm_duty_set(pwm1, duty_cycle);
        //pwm_duty_set(pwm2, duty_cycle);

    }

    return 0;
}
