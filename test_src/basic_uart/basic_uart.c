// References taken from Exploring BeagleBone book by Derek Molloy and termios man page
// Parent opens UART1 and transmits a string
// Child opens UART4 and receives the sring

#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

int main(void)
{
    int fd, cnt;
    struct termios options;

    // printf("Basic UART test\n");
    printf("TEST WITH TIVA - RECEIVE TIVA DATA\n");

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
        usleep(10000000);

        printf("[C]: Receiving string\n");

        // receive string
        unsigned char string_rx[13];
        if ((cnt = read(fd, (void*)string_rx, 13)) < 0)
        {
            perror("[C]: read\n");
            return -1;
        }

        if(cnt == 0)
        {
            printf("[C]: no data to read!\n");
        }
        else
        {
            printf("[C]: received-> '%s'", string_rx);
        }
    // }

    close(fd);

    return 0;
}