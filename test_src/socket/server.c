// Previous assignment code

/*
References:
Sockets: https://www.youtube.com/watch?v=eVYsIolL2gE&t=422s
Daemons: http://www2.lawrence.edu/fast/GREGGJ/CMSC480/Daemons.html
*/

// Header Files
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <errno.h>
#include <syslog.h>
#include <fcntl.h>
#include <signal.h> 
#include <sys/stat.h>
#include <linux/fs.h>

// Port address
#define PORT 9000

// buffer size
#define MAX 200

#if 0
volatile sig_atomic_t signal_flag;

// SIGINT signal handler
void sigint_handler(int sig)
{
    signal_flag = 1;
}

// SIGTERM signal handler
void sigterm_handler(int sig)
{
    signal_flag = 1;
}
#endif

// function to return size of file
off_t fsize(const char *filename) {
    struct stat st; 

    if (stat(filename, &st) == 0)
        return st.st_size;

    return -1; 
}

// Main Function
int main(int argc, char *argv[])
{
    // if this is set, process will run as daemon
    int daemon_flag = 0;

#if 0
    int opt;
    while((opt = getopt(argc, argv, "d")) != -1)  
    {  
        switch(opt)  
        { 
            case 'd':  
                daemon_flag = 1;
                break;
            case '?':  
                printf("unknown option: %c\n", optopt);
                printf("Usage: ./aesdsocket [-d]\n");
                printf("-d : run the process as daemon\n");
                break;
        }  
    }
#endif

#if 0
    // Setting signal handling
    struct sigaction saint;
    struct sigaction saterm;

    signal_flag = 0;

    saint.sa_handler = sigint_handler;
    saint.sa_flags = 0;
    sigemptyset(&saint.sa_mask);

    saterm.sa_handler = sigint_handler;
    saterm.sa_flags = 0;
    sigemptyset(&saterm.sa_mask);

    if (sigaction(SIGINT, &saint, NULL) == -1) {
        perror("sigaction");
        exit(1);
    }

    if (sigaction(SIGTERM, &saterm, NULL) == -1) {
        perror("sigaction");
        exit(1);
    }
#endif

    // socket descriptor
    int sock, cli;
    struct sockaddr_in server, client;
    unsigned int len;
    char recvbuff[MAX];
    char *sendbuff = (char *)calloc(200, sizeof(char));

    // SOCKET
    if((sock = socket(AF_INET, SOCK_STREAM, 0)) == -1)
    {
        perror("socket");
        exit(-1);
    }

    // BIND
    server.sin_family = AF_INET;
    server.sin_port = htons(PORT);
    server.sin_addr.s_addr = INADDR_ANY;
    bzero(&server.sin_zero, 8);

    len = sizeof(struct sockaddr_in);

    if((bind(sock, (struct sockaddr *)&server, len)) == -1)
    {
        perror("bind");
        exit(-1);
    }

#if 0
    // check if port is open, then run process as deamon if flag is set
    if(daemon_flag == 1)
    {
        printf("creating daemon\n");
        
        pid_t pid;

        /* create new process */
        pid = fork ();
        if (pid == -1)
            return -1;
        else if (pid != 0)
            exit (EXIT_SUCCESS);
        
        /* create new session and process group */
        if (setsid() == -1)
            return -1;

        /* set the working directory to the root directory */
        if (chdir ("/") == -1)
            return -1;

        /* redirect fd's 0,1,2 to /dev/null */
        open ("/dev/null", O_RDWR);     /* stdin */
        dup (0);                        /* stdout */
        dup (0);                        /* stderror */

        // a check to determine if daemon was created
        syslog(LOG_DEBUG, "daemon created by aesdsocket\n");
    }
#endif

    // LISTEN
    if((listen(sock, 5)) == -1)
    {
        perror("listen");
        exit(-1);
    }

    // while(signal_flag == 0)
    while (1)
    {
        // ACCEPT
        if((cli = accept(sock, (struct sockaddr *)&client, &len)) == -1)
        {
            // if (errno == EINTR) break;
            perror("accept");
            exit(-1);
        }

        // log ip address of connected client
        syslog(LOG_INFO, "Accepted connection from %s", inet_ntoa(client.sin_addr));

        // SEND/RECV
        // read client message
        bzero(recvbuff, MAX);
        read(cli, recvbuff, sizeof(recvbuff));

        // open a file descriptor to write message to file
        int fd1 = open("/var/tmp/aesdsocketdata", O_WRONLY | O_CREAT | O_APPEND, 0644); 
        if (fd1 < 0) 
        {
            perror("r1");
            exit(1);
        }
        write(fd1, recvbuff, (strlen(recvbuff)));
        fsync(fd1);
        close(fd1);

        // open a file descriptor to read whole content of file
        int fd2 = open("/var/tmp/aesdsocketdata", O_RDONLY | O_CREAT, 0644); 
        if (fd2 < 0) 
        {
            perror("r1");
            exit(1);
        }
        off_t ret = fsize("/var/tmp/aesdsocketdata");
        read(fd2, sendbuff, ret);

        // send buffer data to client
        send(cli, sendbuff, strlen(sendbuff), 0);
        close(fd2);

        // CLOSE SOCKET
        close(cli);

        // log ip address of connected client
        syslog(LOG_INFO, "Closed connection to %s", inet_ntoa(client.sin_addr));
    }

    // close socket
    close(sock);

    // remove file where received client data is stored
    system("rm /var/tmp/aesdsocketdata");
    syslog(LOG_CRIT, "Caught signal, exiting");

    return 0;
}