// References taken from Exploring BeagleBone book by Derek Molloy
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

    printf("111\n\n");

    if ((fd = open("/dev/ttyO1", O_RDWR | O_NOCTTY)) < 0)
    {
        perror("open\n");
        return -1;
    }

    tcgetattr(fd, &options); // sets termios parameters
    
    // communication parameters
    // 9600 baud, 8-bit, enable receiver, no modem control lines
    options.c_cflag = B115200 | CS8 | CREAD | CLOCAL;
    // ignore partity errors, CR -> newline
    options.c_iflag = IGNPAR | ICRNL;
    // discard file information not transmitted
    tcflush(fd, TCIFLUSH);
    // changes occur immmediately
    tcsetattr(fd, TCSANOW, &options);
    
    printf("Basic UART test\n");

    // if(fork() == 0)     // parent
    // {
        printf("[P]: Sending string\n");

        // send string
        unsigned char string_tx[13] = "Hello World!";
        printf("[P]: sending-> '%s'\n", string_tx);
        if ((cnt = write(fd, &string_tx, 13)) < 0)
        {
            perror("[P]: write\n");
            return -1;
        }
    // }
    // else                // child
    // {
    //     // let parent transmit the string before receiving
        usleep(100000);

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