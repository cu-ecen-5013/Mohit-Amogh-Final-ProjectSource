/* Blinks LED for a minute at 1 second intervals */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include "gpio.h"

#define GPIO    (53)
#define COUNT   (30)

int main(void)
{
    int ret = 0;

    printf("GPIO BLINK TEST\n");
    
   // export gpio pin
   if((ret = gpio_export(GPIO)) != 0)
   {
       perror("gpio_export");
       exit(1);
   }

   // set gpio as output
   if((ret = gpio_set_dir(GPIO, GPIO_DIR_OUTPUT)) != 0)
   {
       perror("gpio_set_dir");
       exit(1);
   }
   
   // blink led
   for(int i=0; i<COUNT; i++)
   {
       // led on
       if((ret = gpio_set_value(GPIO, 1)) != 0)
       {
           perror("gpio_set_value");
           exit(1);
       }

       sleep(1);
       
       // led off
       if((ret = gpio_set_value(GPIO, 0)) != 0)
       {
           perror("gpio_set_value");
           exit(1);
       }
       
       sleep(1);
   }
   
   // unexport gpio pin
   if((ret = gpio_unexport(GPIO)) != 0)
   {
       perror("gpio_unexport");
       exit(1);
   }

    return 0;
}
