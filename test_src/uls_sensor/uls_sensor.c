/* Program to measure distance using HC-SR04 ultrasonic sensor
 * Distance is measured continuously
 */

/* References:
 * https://www.kernel.org/doc/Documentation/gpio/sysfs.txt
 * http://man7.org/linux/man-pages/man2/poll.2.html
 * https://cdn.sparkfun.com/datasheets/Sensors/Proximity/HCSR04.pdf
 */

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <unistd.h>
#include <stdint.h>
#include <string.h>
#include <time.h>
#include <poll.h>
#include <syslog.h>
#include "gpio.h"

#define ON              (1)
#define OFF             (0)

#define BB_LED_GPIO     (53)
#define TRIG_GPIO       (66)
#define ECHO_GPIO       (69)

int main(void)
{
    int ret = 0;
    struct timespec echo_start, echo_end;
    struct timespec trig_low = { 0, 2000 };
    struct timespec trig_high = { 0, 10000 };
    struct timespec cycle = { 0, 80000000 };
    struct pollfd echo_poll;
    int nfds = 1;
    int time_elapsed_ns = 0;
    int distance = 0;

    printf("ULTRASONIC SENSOR TEST\n\n");
    
    // create gpio files
    if((ret = gpio_export(TRIG_GPIO)) != 0) { perror("gpio_export"); exit(1); }
    if((ret = gpio_export(ECHO_GPIO)) != 0) { perror("gpio_export"); exit(1); }
    if((ret = gpio_export(BB_LED_GPIO)) != 0) { perror("gpio_export"); exit(1); }

    // set direction
    if((ret = gpio_set_dir(TRIG_GPIO, GPIO_DIR_OUTPUT)) != 0) { perror("gpio_set_dir"); exit(1); }
    if((ret = gpio_set_dir(ECHO_GPIO, GPIO_DIR_INPUT)) != 0) { perror("gpio_set_dir"); exit(1); }
    if((ret = gpio_set_dir(BB_LED_GPIO, GPIO_DIR_OUTPUT)) != 0) { perror("gpio_set_dir"); exit(1); }

    // set echo edge to falling (interrupt would be generated on falling edge)
    if((ret = gpio_set_edge(ECHO_GPIO, kPollEdge_falling)) != 0) { perror("gpio_set_edge"); exit(1); }

    // stuff the poll structure
    memset((void*) &echo_poll, 0, sizeof(echo_poll));
    echo_poll.fd = gpio_fd_open(ECHO_GPIO);
    echo_poll.events = POLLPRI;

    printf("Distance:\n");

    while(1)
    {
        // set trig pin low
        if((ret = gpio_set_value(TRIG_GPIO, OFF)) != 0) { perror("gpio_set_value"); exit(1); }
        
        // wait 2 us then set trig pin high
        nanosleep(&trig_low, NULL);
        if((ret = gpio_set_value(TRIG_GPIO, ON)) != 0) { perror("gpio_set_value"); exit(1); }

        // wait 10 us then set trig pin low
        nanosleep(&trig_high, NULL);
        if((ret = gpio_set_value(TRIG_GPIO, OFF)) != 0) { perror("gpio_set_value"); exit(1); }

        // record echo start time (echo pin gets high)
        clock_gettime(CLOCK_MONOTONIC, &echo_start);

        // poll for falling edge; timeout = 1 sec
        ret = poll(&echo_poll, nfds, 1);
        if(ret < 0) { perror("\rerror: poll"); continue; }      // error check
        else if(ret == 0) { printf("\rtimeout"); continue; }     // poll timeout
        if(echo_poll.revents & POLLPRI)                         // echo falling edge occurs
        {
            // record echo stop time (echo pin gets low)
            clock_gettime(CLOCK_MONOTONIC, &echo_end);

            // calculate time difference
            time_elapsed_ns = (echo_end.tv_nsec > echo_start.tv_nsec) ? (echo_end.tv_nsec - echo_start.tv_nsec) : (echo_end.tv_nsec - echo_start.tv_nsec + 1000000000);

            // calculate distance based on formula given in datasheet
            distance = time_elapsed_ns/(58 * 1000);

            // print time difference
            printf("\r%d", distance);
            printf(" cm");

            // set led if distance is less than 50 cm
            if(distance <= 50) {
                if((ret = gpio_set_value(BB_LED_GPIO, ON)) != 0) { perror("gpio_set_value"); exit(1); }
            }
            else {
                if((ret = gpio_set_value(BB_LED_GPIO, OFF)) != 0) { perror("gpio_set_value"); exit(1); }
            }
        }

        // measurement cycle should be atleast 60 ms according to datasheet
        nanosleep(&cycle, NULL);
    }

    // remove gpio files
    if((ret = gpio_unexport(TRIG_GPIO)) != 0) { perror("gpio_unexport"); exit(1); }
    if((ret = gpio_unexport(ECHO_GPIO)) != 0) { perror("gpio_unexport"); exit(1); }
    if((ret = gpio_unexport(BB_LED_GPIO)) != 0) { perror("gpio_unexport"); exit(1); }

    return 0;
}