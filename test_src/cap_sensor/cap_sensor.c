/* Turns on Sensor LED and BeagleBone LED when touch is detected */
/* Touch reading taken every 10 ms */
/* This program runs in an infinite loop */

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <unistd.h>
#include <stdint.h>
#include "gpio.h"

#define ON              (1)
#define OFF             (0)

#define BB_LED_GPIO     (53)
// #define CAP_LED_GPIO    (66)
#define CAP_DATA_GPIO   (69)

int main(void)
{
    int ret = 0;

    // holds capacitive sensor data
    int cap_data = 0;

    printf("CAPACITIVE TOUCH SENSOR TEST\n");
        
    // export beaglebone led gpio pin, capacitive sensor led and data pins
    if((ret = gpio_export(BB_LED_GPIO)) != 0)
    {
        perror("gpio_export");
        exit(1);
    }
    // if((ret = gpio_export(CAP_LED_GPIO)) != 0)
    // {
    //     perror("gpio_export");
    //     exit(1);
    // }
    if((ret = gpio_export(CAP_DATA_GPIO)) != 0)
    {
        perror("gpio_export");
        exit(1);
    }

    // set led pins as output and capacitive sensor data pin as input
    if((ret = gpio_set_dir(BB_LED_GPIO, GPIO_DIR_OUTPUT)) != 0)
    {
        perror("gpio_set_dir");
        exit(1);
    }
    // if((ret = gpio_set_dir(CAP_LED_GPIO, GPIO_DIR_OUTPUT)) != 0)
    // {
    //     perror("gpio_set_dir");
    //     exit(1);
    // }
    if((ret = gpio_set_dir(CAP_DATA_GPIO, GPIO_DIR_INPUT)) != 0)
    {
        perror("gpio_set_dir");
        exit(1);
    }
    
    // light up beaglebone and sensor leds if touch is detected
    for(;;)
    {
        // get sensor data
        if((ret = gpio_get_value(CAP_DATA_GPIO, &cap_data)) != 0)
        {
            perror("gpio_get_value");
            exit(1);
        }

        if(cap_data == 1)
        {
            printf("Touched\n");
            
            // beaglebone led on
            if((ret = gpio_set_value(BB_LED_GPIO, ON)) != 0)
            {
                perror("gpio_set_value");
                exit(1);
            }
            
            // // capacitive sensor led on
            // if((ret = gpio_set_value(CAP_LED_GPIO, ON)) != 0)
            // {
            //     perror("gpio_set_value");
            //     exit(1);
            // }
        }
        else
        {
            printf("Not Touched\n");
            
            // beaglebone led off
            if((ret = gpio_set_value(BB_LED_GPIO, OFF)) != 0)
            {
                perror("gpio_set_value");
                exit(1);
            }
            
            // // capacitive sensor led off
            // if((ret = gpio_set_value(CAP_LED_GPIO, OFF)) != 0)
            // {
            //     perror("gpio_set_value");
            //     exit(1);
            // } 
        }
       
        // sleep for 500 ms
        usleep(500000);
    }
   
    // unexport all exported pins
    if((ret = gpio_unexport(BB_LED_GPIO)) != 0)
    {
        perror("gpio_unexport");
        exit(1);
    }
    // if((ret = gpio_unexport(CAP_LED_GPIO)) != 0)
    // {
    //     perror("gpio_unexport");
    //     exit(1);
    // }
    if((ret = gpio_unexport(CAP_DATA_GPIO)) != 0)
    {
        perror("gpio_unexport");
        exit(1);
    }

    return 0;
}
