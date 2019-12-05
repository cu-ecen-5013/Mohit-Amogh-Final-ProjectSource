/* Blinks LED for a minute at 1 second intervals */
/* Also starts the buzzer */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include "gpio.h"

#define LED_GPIO    (53)
#define BUZ_GPIO    (67)
#define COUNT       (30)

int main(void)
{
    int ret = 0;

    printf("LED_GPIO BLINK TEST\n");
    
   // export gpio and buzzer pin
   if((ret = gpio_export(LED_GPIO)) != 0)
   {
       perror("gpio_export");
       exit(1);
   }
   if((ret = gpio_export(BUZ_GPIO)) != 0)
   {
       perror("gpio_export");
       exit(1);
   }

   // set led and buzzer gpio as output
   if((ret = gpio_set_dir(LED_GPIO, GPIO_DIR_OUTPUT)) != 0)
   {
       perror("gpio_set_dir");
       exit(1);
   }
   if((ret = gpio_set_dir(BUZ_GPIO, GPIO_DIR_OUTPUT)) != 0)
   {
       perror("gpio_set_dir");
       exit(1);
   }
   
   // blink led and start buzzer
   for(int i=0; i<COUNT; i++)
   {
       // led on
       if((ret = gpio_set_value(LED_GPIO, 1)) != 0)
       {
           perror("gpio_set_value");
           exit(1);
       }
       // buzzer on
       if((ret = gpio_set_value(BUZ_GPIO, 1)) != 0)
       {
           perror("gpio_set_value");
           exit(1);
       }

       sleep(1);
       
       // led off
       if((ret = gpio_set_value(LED_GPIO, 0)) != 0)
       {
           perror("gpio_set_value");
           exit(1);
       }
       // buzzer off
       if((ret = gpio_set_value(BUZ_GPIO, 0)) != 0)
       {
           perror("gpio_set_value");
           exit(1);
       }

       sleep(1);
   }
   
   // unexport gpio pin
   if((ret = gpio_unexport(LED_GPIO)) != 0)
   {
       perror("gpio_unexport");
       exit(1);
   }
   if((ret = gpio_unexport(BUZ_GPIO)) != 0)
   {
       perror("gpio_unexport");
       exit(1);
   }

    return 0;
}
