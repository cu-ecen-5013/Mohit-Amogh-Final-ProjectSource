#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <syslog.h>
#include "../inc/main.h"
#include "../inc/lux_task.h"

/* Lux Sensor Task */
void lux_task(void)
{
    syslog(LOG_DEBUG, "[LUX ] Task Started - PID %ld\n", (long)getpid());
    PDEBUG("[LUX ] Task Started - PID %ld\n", (long)getpid());
}

/* Capacitive Sensor Task */
void cap_task(void)
{
    syslog(LOG_DEBUG, "[CAP ] Task Started - PID %ld\n", (long)getpid());
    PDEBUG("[CAP ] Task Started - PID %ld\n", (long)getpid());
}

/* UART Tx Task */
void utx_task(void)
{
    syslog(LOG_DEBUG, "[UTX ] Task Started - PID %ld\n", (long)getpid());
    PDEBUG("[UTX ] Task Started - PID %ld\n", (long)getpid());
}

/* Logger Task */
void log_task(void)
{
    syslog(LOG_DEBUG, "[LOG ] Task Started - PID %ld\n", (long)getpid());
    PDEBUG("[LOG ] Task Started - PID %ld\n", (long)getpid());
}

int main(void)
{
    // array of function pointers
    void(*fun_ptr_arr[])(void) = { lux_task, 
                                    cap_task,
                                    utx_task,
                                    log_task };
    pid_t fork_pid[NUM_OF_TASKS];
    pid_t wait_pid;
    int task_cnt = NUM_OF_TASKS;
    int status;
    int i;

    /* Create each task using fork */
    syslog(LOG_DEBUG, "[MAIN] Starting Tasks\n");
    PDEBUG("[MAIN] Starting Tasks\n");
    for(i=0; i<task_cnt; i++) {
        if((fork_pid[i] = fork()) < 0)      // error check
        {
            perror("fork");
            exit(EXIT_FAILURE);
        }
        else if (fork_pid[i] == 0)          // in child
        {
            (*fun_ptr_arr[i])();
            exit(EXIT_SUCCESS);
        }
    }
    syslog(LOG_DEBUG, "[MAIN] All Tasks Started\n");
    PDEBUG("[MAIN] All Tasks Started\n");

    /* Wait for children to exit */
    syslog(LOG_DEBUG, "[MAIN] Waiting for children to exit\n");
    PDEBUG("[MAIN] Waiting for children to exit\n");
    while(task_cnt > 0)
    {
        wait_pid = wait(&status);
        --task_cnt;

        syslog(LOG_DEBUG, "[MAIN] Child with PID %ld exited with status 0x%x\n", (long)wait_pid, status);
        PDEBUG("[MAIN] Child with PID %ld exited with status 0x%x\n", (long)wait_pid, status);
    }

    syslog(LOG_DEBUG, "[MAIN] Program Complete\n");
    PDEBUG("[MAIN] Program Complete\n");

    return EXIT_SUCCESS;
}