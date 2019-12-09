// References taken from Exploring BeagleBone book by Derek Molloy and termios man page
// Parent opens UART1 and transmits a string
// Child opens UART4 and receives the sring

#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <stdint.h>

int main(void)
{
#if 1
    int fd, cnt;
    struct termios options;

    if ((fd = open("/dev/ttyO1", O_RDWR | O_NOCTTY)) < 0)
    {
        perror("open\n");
        return -1;
    }

    // fcntl(fd, F_SETFL, 0);
    tcgetattr(fd, &options);
    if((cfsetispeed(&options, B115200)) == -1)
    {
        perror("input baud\n");
        return -1;
    }
    if((cfsetospeed(&options, B115200)) == -1)
    {
        perror("output baud\n");
        return -1;
    } 

    options.c_cflag |= (CLOCAL | CS8);
    options.c_iflag &= ~(ISTRIP | IXON | INLCR | PARMRK | ICRNL | IGNBRK);
    options.c_oflag &=  ~(OPOST);
    options.c_lflag &= ~(ICANON | ECHO | ECHONL | ISIG | IEXTEN);

    tcsetattr(fd, TCSAFLUSH, &options);
    
    printf("Basic UART - 33\n");

    // if(fork() == 0)     // parent
    // {
        printf("[P]: Sending string\n");

        // send string
        // unsigned char string_tx[13] = "Hello World!";
        // printf("[P]: sending-> '%s'\n", string_tx);

        while(1)
        {
            unsigned char send = 'g';

            if ((cnt = write(fd, (void*)&send, 1)) < 0)
            {
                perror("[P]: write\n");
                return -1;
            }
            usleep(10000);
        }        
    // }
    // else                // child
    // {
        // let parent transmit the string before receiving
        usleep(100000);

        // printf("[C]: Receiving string\n");

        // // receive string
        // unsigned char string_rx[13];
        // if ((cnt = read(fd, (void*)string_rx, 1)) < 0)
        // {
        //     perror("[C]: read\n");
        //     return -1;
        // }

        // if(cnt == 0)
        // {
        //     printf("[C]: no data to read!\n");
        // }
        // else
        // {
        //     printf("[C]: received-> '%c'", string_rx[0]);
        // }
    // }

    close(fd);

    return 0;
#else
    int fd, cnt;
    struct termios options;

    // printf("Basic UART test\n");
    printf("TEST WITH TIVA - RECEIVE TIVA DATA - 5\n");

    if ((fd = open("/dev/ttyO1", O_RDWR | O_NOCTTY)) < 0)
    {
        perror("open\n");
        return -1;
    }

    // fcntl(fd, F_SETFL, 0);
    tcgetattr(fd, &options);
    if((cfsetispeed(&options, B115200)) == -1)
    {
        perror("input baud\n");
        return -1;
    }
    if((cfsetospeed(&options, B115200)) == -1)
    {
        perror("output baud\n");
        return -1;
    } 

    options.c_cflag |= (CLOCAL | CS8);
    options.c_iflag &= ~(ISTRIP | IXON | INLCR | PARMRK | ICRNL | IGNBRK);
    options.c_oflag &= ~(OPOST);
    options.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);

    tcsetattr(fd, TCSAFLUSH, &options);

    // if(fork() == 0)     // parent
    // {
        // printf("[P]: Sending string\n");

        // // send string
        // unsigned char string_tx[13] = "Hello World!";
        // printf("[P]: sending-> '%s'\n", string_tx);
        // if ((cnt = write(fd, &string_tx, 13)) < 0)
        // {
        //     perror("[P]: write\n");
        //     return -1;
        // }
    // }
    // else                // child
    // {
        // let parent transmit the string before receiving
        // usleep(10000000);

        printf("[C]: Sending characters\n");

        // receive string
        unsigned char char_rx;
        // unsigned char char_tx = '5';

        struct sensor {
            uint8_t led_value;
            uint8_t buzz_value;
        } test;

        printf("sizeof test = %ld\n", sizeof(test));
        test.led_value = 1;
        test.buzz_value = 2;

        while(1)
        {
            // if ((cnt = write(fd, &char_tx, 1)) < 0)
            if ((cnt = write(fd, (void*)&test, sizeof(test))) < 0)
            {
                perror("[C]: write\n");
                return -1;
            }

            test.led_value += 2;
            test.buzz_value += 2;

            usleep(1000000);

            // if(cnt == 0)
            // {
            //     printf("\n[C]: no data to read!\n");
            // }
            // else
            // {
            //     printf("test.led_value = %u\n", test.led_value);
            //     printf("test.buzz_value = %u\n", test.buzz_value);
            //     // printf("[C]: received-> '%c'\n", char_rx);
            // }
        }
    // }

    close(fd);

    return 0;
#endif
}