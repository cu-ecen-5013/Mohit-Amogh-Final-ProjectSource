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
#include <semaphore.h>
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

    int pshm_1_fd;
    sem_t *lux_sem;
    SHMSEG shmseg_lux = { 0, 0 };
    SHMSEG *shmseg_lux_ptr = &shmseg_lux;
    SHMSEG *pshm_1_base = NULL;
    shmseg_lux_ptr->sensor = LUX;

    /* Open shared memory */
    if ((pshm_1_fd = shm_open(PSHM_1_NAME,                                  /* name of object */
                              O_RDWR,                                       /* open object for read-write access */
                              0                                             /* mode - specified 0 for opening existing object */
                              )) < 0) { error("[LUX ] shm_open"); }

    /* Map shared memory segment to address space of process */
    if ((pshm_1_base = (SHMSEG *)mmap(NULL,                                 /* system chooses where to place shm in virtual address space */
                                      PSHM_1_NUM_OF_PROD * sizeof(SHMSEG),  /* size of mapping */
                                      PROT_READ|PROT_WRITE,                 /* read-write mapping */
                                      MAP_SHARED,                           /* make modifications to shm visible to other processes */
                                      pshm_1_fd,                            /* file descriptor specifying file to map */
                                      0                                     /* offset of mapping in shm file */
                                      )) < 0) { error("[LUX ] mmap"); }

    /* Close shared memory file descriptor */
    if (close(pshm_1_fd) < 0) { error("[LUX ] close"); }

    /* Open the semaphore */
    if ((lux_sem = sem_open(lux_sem_name, 0, 0600, 0)) < 0) { error("[LUX ] sem_open"); }

    /* Set mmap address for writing data - lux sensor writes data in 1st sub-segment of shared memory */
    pshm_1_base += (LUX - 1) * sizeof(SHMSEG);

    // while(1)
    // {
        // TODO: take sensor readings
        shmseg_lux_ptr->data = 10;

        /* Write data to shared memory */
        memcpy((void*)pshm_1_base, (void*)shmseg_lux_ptr, sizeof(SHMSEG));

        /* Post semaphore */
        sem_post(lux_sem);

        // sleep
    // }

    /* Unmap the mapped shared memory segment from the address space of the process */
    pshm_1_base -= (LUX - 1) * sizeof(SHMSEG);
    if (munmap(pshm_1_base, PSHM_1_NUM_OF_PROD * sizeof(SHMSEG)) < 0) { error("[LUX ] munmap"); }

    /* Close the semaphore */
    if (sem_close(lux_sem) < 0) { error("[LUX ] sem_close"); }
}


/* Capacitive Sensor Task 
 * Producer 2 for Shared Memory */
void cap_task(void)
{
    syslog(LOG_DEBUG, "[CAP ] Task Started - PID %ld\n", (long)getpid());
    PDEBUG("[CAP ] Task Started - PID %ld\n", (long)getpid());

    int pshm_1_fd;
    sem_t *cap_sem;
    SHMSEG shmseg_cap = { 0, 0 };
    SHMSEG *shmseg_cap_ptr = &shmseg_cap;
    SHMSEG *pshm_1_base = NULL;
    shmseg_cap_ptr->sensor = CAP;

    /* Open shared memory */
    if ((pshm_1_fd = shm_open(PSHM_1_NAME,                                  /* name of object */
                              O_RDWR,                                       /* open object for read-write access */
                              0                                             /* mode - specified 0 for opening existing object */
                              )) < 0) { error("[CAP ] shm_open"); }

    /* Map shared memory segment to address space of process */
    if ((pshm_1_base = (SHMSEG *)mmap(NULL,                                 /* system chooses where to place shm in virtual address space */
                                      PSHM_1_NUM_OF_PROD * sizeof(SHMSEG),  /* size of mapping */
                                      PROT_READ|PROT_WRITE,                 /* read-write mapping */
                                      MAP_SHARED,                           /* make modifications to shm visible to other processes */
                                      pshm_1_fd,                            /* file descriptor specifying file to map */
                                      0                                     /* offset of mapping in shm file */
                                      )) < 0) { error("[CAP ] mmap"); }

    /* Close shared memory file descriptor */
    if (close(pshm_1_fd) < 0) { error("[CAP ] close"); }

    /* Open the semaphore */
    if ((cap_sem = sem_open(cap_sem_name, 0, 0600, 0)) < 0) { error("[CAP ] sem_open"); }

    /* Set mmap address for writing data - cap sensor writes data in 2nd sub-segment of shared memory */
    pshm_1_base += (CAP - 1) * sizeof(SHMSEG);

    // while(1)
    // {
        // TODO: take sensor readings
        shmseg_cap_ptr->data = 46;

        /* Write data to shared memory */
        memcpy((void*)pshm_1_base, (void*)shmseg_cap_ptr, sizeof(SHMSEG));

        /* Post semaphore */
        sem_post(cap_sem);

        // sleep
    // }

    /* Unmap the mapped shared memory segment from the address space of the process */
    pshm_1_base -= (CAP - 1) * sizeof(SHMSEG);
    if (munmap(pshm_1_base, PSHM_1_NUM_OF_PROD * sizeof(SHMSEG)) < 0) { error("[CAP ] munmap"); }

    /* Close the semaphore */
    if (sem_close(cap_sem) < 0) { error("[CAP ] sem_close"); }
}


/* UART Tx Task 
 * Consumer 1 for Shared Memory */
void utx_task(void)
{
    syslog(LOG_DEBUG, "[UTX ] Task Started - PID %ld\n", (long)getpid());
    PDEBUG("[UTX ] Task Started - PID %ld\n", (long)getpid());

    int pshm_1_fd;
    sem_t *lux_sem, *cap_sem;
    SHMSEG shmseg_utx;
    SHMSEG *shmseg_utx_ptr = &shmseg_utx;
    SHMSEG *pshm_1_base = NULL;

    /* Open shared memory */
    if ((pshm_1_fd = shm_open(PSHM_1_NAME, O_RDONLY, 0)) < 0) { error("[UTX ] shm_open"); }

    /* Map shared memory segment to address space of process */
    if ((pshm_1_base = mmap(NULL, PSHM_1_NUM_OF_PROD * sizeof(SHMSEG), PROT_READ, MAP_SHARED, pshm_1_fd, 0)) < 0) { error("[UTX ] mmap"); }

    /* Close shared memory file descriptor */
    if (close(pshm_1_fd) < 0) { error("[UTX ] close"); }

    /* Open the semaphore */
    if ((lux_sem = sem_open(lux_sem_name, 0, 0600, 0)) < 0) { error("[UTX ] sem_open"); }
    if ((cap_sem = sem_open(cap_sem_name, 0, 0600, 0)) < 0) { error("[UTX ] sem_open"); }

    /* Read data from shared memory */
    // sleep(1);

    // wait for semaphore
    sem_wait(cap_sem);
    sem_wait(lux_sem);

    memcpy((void*)shmseg_utx_ptr, (void*)pshm_1_base, sizeof(SHMSEG));
    PDEBUG("[UTX ] shmseg_lux.sensor = %d\n", shmseg_utx.sensor);
    PDEBUG("[UTX ] shmseg_lux.data = %d\n", shmseg_utx.data);

    pshm_1_base += sizeof(SHMSEG);

    memcpy((void*)shmseg_utx_ptr, (void*)pshm_1_base, sizeof(SHMSEG));
    PDEBUG("[UTX ] shmseg_cap.sensor = %d\n", shmseg_utx.sensor);
    PDEBUG("[UTX ] shmseg_cap.data = %d\n", shmseg_utx.data);

    pshm_1_base -= sizeof(SHMSEG);

    /* Remove the mapped shared memory segment from the address space of the process */
    if (munmap(pshm_1_base, PSHM_1_NUM_OF_PROD * sizeof(SHMSEG)) < 0) { error("[UTX ] munmap"); }

    /* Close the semaphore */
    if (sem_close(lux_sem) < 0) { error("[UTX ] sem_close"); }
    if (sem_close(cap_sem) < 0) { error("[UTX ] sem_close"); }
}


#if 1
/* Logger Task 
 * Consumer 2 for Shared Memory */
void log_task(void)
{
    syslog(LOG_DEBUG, "[LOG ] Task Started - PID %ld\n", (long)getpid());
    PDEBUG("[LOG ] Task Started - PID %ld\n", (long)getpid());

    // SHMSEG *shmseg_log;
}
#endif


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
    sem_t *sem_des;

    int pshm_1_fd, pshm_2_fd;               // file descriptor, from shm_open

    /* Create POSIX shared memory */
    if ((pshm_1_fd = shm_open(PSHM_1_NAME, O_CREAT | O_EXCL | O_RDWR, 0600)) < 0) { error("[MAIN] shm_open"); }
    if ((pshm_2_fd = shm_open(PSHM_2_NAME, O_CREAT | O_EXCL | O_RDWR, 0600)) < 0) { error("[MAIN] shm_open"); }

    /* Set size of shared memory */
    if (ftruncate(pshm_1_fd, PSHM_1_NUM_OF_PROD * sizeof(SHMSEG)) < 0) { error("[MAIN] ftruncate"); }
    if (ftruncate(pshm_2_fd, PSHM_2_NUM_OF_PROD * sizeof(SHMSEG)) < 0) { error("[MAIN] ftruncate"); }

    /* Close shared memory file descriptors */
    if (close(pshm_1_fd) < 0) { error("[MAIN] close"); }
    if (close(pshm_2_fd) < 0) { error("[MAIN] close"); }

    /* Create semaphores for shared memory synchronization */
    if ((sem_des = sem_open(lux_sem_name, O_CREAT, 0600, 0)) < 0) { error("[MAIN] sem_open"); }
    if (sem_close(sem_des) < 0) { error("[MAIN] sem_close"); }
    if ((sem_des = sem_open(cap_sem_name, O_CREAT, 0600, 0)) < 0) { error("[MAIN] sem_open"); }
    if (sem_close(sem_des) < 0) { error("[MAIN] sem_close"); }

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