/* References:
 * 1. http://man7.org/training/download/posix_shm_slides.pdf
 * 2. http://www.cse.psu.edu/~deh25/cmpsc473/notes/OSC/Processes/shm-posix-producer.c
 * 3. http://www.cse.psu.edu/~deh25/cmpsc473/notes/OSC/Processes/shm-posix-consumer.c */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <string.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <sys/mman.h>
#include <syslog.h>
#include "../inc/main.h"
#include "../inc/lux_task.h"


/* Print system error and exit */
void error (char *msg)
{
    perror(msg);
    exit(EXIT_FAILURE);
}


/* Lux Sensor Task 
 * Producer 1 for Shared Memory */
void lux_task(void)
{
    syslog(LOG_DEBUG, "[LUX ] Task Started - PID %ld\n", (long)getpid());
    PDEBUG("[LUX ] Task Started - PID %ld\n", (long)getpid());

    // SHMSEG *shmseg_lux;
    // shmseg_lux->sensor = LUX;    // fixed

}


/* Capacitive Sensor Task 
 * Producer 2 for Shared Memory */
void cap_task(void)
{
    syslog(LOG_DEBUG, "[CAP ] Task Started - PID %ld\n", (long)getpid());
    PDEBUG("[CAP ] Task Started - PID %ld\n", (long)getpid());

    int pshm_1_fd;
    SHMSEG shmseg_cap;
    SHMSEG* shmseg_cap_ptr = &shmseg_cap;
    SHMSEG *pshm_1_base = NULL;
    shmseg_cap_ptr->sensor = 1;    // fixed

    /* Open shared memory */
    if ((pshm_1_fd = shm_open(PSHM_1_NAME, O_RDWR, 0600)) < 0) { error("[CAP ] shm_open"); }

    /* Map shared memory segment to address space of process */
    if ((pshm_1_base = (SHMSEG *)mmap(NULL, sizeof(SHMSEG), PROT_READ | PROT_WRITE, MAP_SHARED, pshm_1_fd, 0)) < 0) { error("[CAP ] mmap"); }

    /* Close shared memory file descriptor */
    if (close(pshm_1_fd) < 0) { error("[CAP ] close"); }

    shmseg_cap_ptr->data = 23;

    /* Write data to shared memory */
    memcpy((void*)pshm_1_base, (void*)shmseg_cap_ptr, sizeof(SHMSEG));

    /* Remove the mapped shared memory segment from the address space of the process */
    if (munmap(pshm_1_base, sizeof(SHMSEG)) < 0) { error("[CAP ] munmap"); }
}


/* UART Tx Task 
 * Consumer 1 for Shared Memory */
void utx_task(void)
{
    syslog(LOG_DEBUG, "[UTX ] Task Started - PID %ld\n", (long)getpid());
    PDEBUG("[UTX ] Task Started - PID %ld\n", (long)getpid());

    int pshm_1_fd;
    SHMSEG shmseg_utx;
    SHMSEG* shmseg_utx_ptr = &shmseg_utx;
    SHMSEG *pshm_1_base = NULL;

    /* Open shared memory */
    if ((pshm_1_fd = shm_open(PSHM_1_NAME, O_RDONLY, 0600)) < 0) { error("[UTX ] shm_open"); }

    /* Map shared memory segment to address space of process */
    if ((pshm_1_base = mmap(NULL, sizeof(SHMSEG), PROT_READ, MAP_SHARED, pshm_1_fd, 0)) < 0) { error("[UTX ] mmap"); }

    /* Close shared memory file descriptor */
    if (close(pshm_1_fd) < 0) { error("[UTX ] close"); }

    /* Read data from shared memory */
    sleep(1);
    memcpy((void*)shmseg_utx_ptr, (void*)pshm_1_base, sizeof(SHMSEG));
    PDEBUG("[UTX ] shmseg_utx.sensor = %d\n", shmseg_utx.sensor);
    PDEBUG("[UTX ] shmseg_utx.data = %d\n", shmseg_utx.data);

    /* Remove the mapped shared memory segment from the address space of the process */
    if (munmap(pshm_1_base, sizeof(SHMSEG)) < 0) { error("[UTX ] munmap"); }
}


/* Logger Task 
 * Consumer 2 for Shared Memory */
void log_task(void)
{
    syslog(LOG_DEBUG, "[LOG ] Task Started - PID %ld\n", (long)getpid());
    PDEBUG("[LOG ] Task Started - PID %ld\n", (long)getpid());

    // SHMSEG *shmseg_log;
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

    int pshm_1_fd, pshm_2_fd;               // file descriptor, from shm_open

    /* Create POSIX shared memory */
    if ((pshm_1_fd = shm_open(PSHM_1_NAME, O_CREAT | O_EXCL | O_RDWR, 0600)) < 0) { error("[MAIN] shm_open"); }
    if ((pshm_2_fd = shm_open(PSHM_2_NAME, O_CREAT | O_EXCL | O_RDWR, 0600)) < 0) { error("[MAIN] shm_open"); }

    /* Set size of shared memory */
    if (ftruncate(pshm_1_fd, sizeof(SHMSEG)) < 0) { error("[MAIN] ftruncate"); }
    if (ftruncate(pshm_2_fd, sizeof(SHMSEG)) < 0) { error("[MAIN] ftruncate"); }

    /* Close shared memory file descriptors */
    if (close(pshm_1_fd) < 0) { error("[MAIN] close"); }
    if (close(pshm_2_fd) < 0) { error("[MAIN] close"); }

    /* Create each task using fork */
    syslog(LOG_DEBUG, "[MAIN] Starting Tasks\n");
    PDEBUG("[MAIN] Starting Tasks\n");
    for (i=0; i<task_cnt; i++)
    {
        if ((fork_pid[i] = fork()) < 0)                      // error check
        {
            error("fork");
        }
        else if (fork_pid[i] == 0)                          // in child
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
    while (task_cnt > 0)
    {
        wait_pid = wait(&status);
        --task_cnt;

        syslog(LOG_DEBUG, "[MAIN] Child with PID %ld exited with status %d\n", (long)wait_pid, status);
        PDEBUG("[MAIN] Child with PID %ld exited with status %d\n", (long)wait_pid, status);
    }

    /* Unlink shared memory */
    if (shm_unlink(PSHM_1_NAME) < 0) { error("[MAIN] shm_unlink"); }
    if (shm_unlink(PSHM_2_NAME) < 0) { error("[MAIN] shm_unlink"); }

    syslog(LOG_DEBUG, "[MAIN] Program Complete\n");
    PDEBUG("[MAIN] Program Complete\n");

    return EXIT_SUCCESS;
}