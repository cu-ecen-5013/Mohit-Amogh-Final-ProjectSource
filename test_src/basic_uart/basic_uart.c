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
    
    printf("Basic UART test\n");

    if(fork() == 0)
    {
        printf("[P]: Sending string\n");

        if ((fd = open("/dev/ttyO1", O_WRONLY | O_NOCTTY | O_NDELAY)) < 0)
        {
            perror("[P]: open\n");
            return -1;
        }

        tcgetattr(fd, &options); // sets termios parameters
        
        // communication parameters
        // 9600 baud, 8-bit, enable receiver, no modem control lines
        options.c_cflag = B9600 | CS8 | CREAD | CLOCAL;
        // ignore partity errors, CR -> newline
        options.c_iflag = IGNPAR | ICRNL;
        // discard file information not transmitted
        tcflush(fd, TCIFLUSH);
        // changes occur immmediately
        tcsetattr(fd, TCSANOW, &options);

        // send string
        unsigned char string_tx[13] = "Hello World!";
        printf("[P]: sending-> '%s'\n", string_tx);
        if ((cnt = write(fd, &string_tx, 13)) < 0)
        {
            perror("[P]: write\n");
            return -1;
        }

        close(fd);
    }
    else
    {
        printf("[C]: Receiving string\n");
        if ((fd = open("/dev/ttyO4", O_RDONLY | O_NOCTTY | O_NDELAY)) < 0)
        {
            perror("open\n");
            return -1;
        }

        tcgetattr(fd, &options); // sets termios parameters
        
        // communication parameters
        // 9600 baud, 8-bit, enable receiver, no modem control lines
        options.c_cflag = B9600 | CS8 | CREAD | CLOCAL;
        // ignore partity errors, CR -> newline
        options.c_iflag = IGNPAR | ICRNL;
        // changes occur immmediately
        tcsetattr(fd, TCSANOW, &options);

        // let parent transmit the string before receiving
        usleep(100000);

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

        close(fd);
    }

    return 0;
}