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


/* Function declarations */
void error (char *msg);

void lux_task(void);
uint8_t lux_sensor_init(void);
uint8_t get_lux_value(void);
int apds9301_read_reg_byte(int file, unsigned char addr, unsigned char cmd_reg, unsigned char* data);
int apds9301_write_reg_byte(int file, unsigned char addr, unsigned char cmd_reg, unsigned char* data);
float calculate_lux(uint16_t ch0, uint16_t ch1);

void cap_task(void);
void utx_task(void);
void log_task(void);


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

////////////////////////////////////////////////////////////////////////////////////////////

/* Print system error and exit */
void error (char *msg)
{
    perror(msg);
    exit(EXIT_FAILURE);
}

////////////////////////////////////////////////////////////////////////////////////////////

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

    /* Initialize lux sensor */
    // if(lux_sensor_init() < 0) { error("[LUX ] Initialization Failure"); }

    while(1)
    {
        /* Take sensor readings */
        // shmseg_lux_ptr->data = get_lux_value();
        shmseg_lux_ptr->data = 45;

        /* Write data to shared memory */
        memcpy((void*)(&pshm_1_base[LUX]), (void*)shmseg_lux_ptr, sizeof(SHMSEG));

        /* Post semaphore */
        sem_post(lux_sem);

        /* Sleep - period of measurement */
        sleep(2);
    }

    /* Unmap the mapped shared memory segment from the address space of the process */
    if (munmap(pshm_1_base, PSHM_1_NUM_OF_PROD * sizeof(SHMSEG)) < 0) { error("[LUX ] munmap"); }

    /* Close the semaphore */
    if (sem_close(lux_sem) < 0) { error("[LUX ] sem_close"); }
}

////////////////////////////////////////////////////////////////////////////////////////////

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

    while(1)
    {
        // TODO: take sensor readings
        shmseg_cap_ptr->data = 46;

        /* Write data to shared memory */
        memcpy((void*)(&pshm_1_base[CAP]), (void*)shmseg_cap_ptr, sizeof(SHMSEG));

        /* Post semaphore */
        sem_post(cap_sem);

        // sleep
        sleep(2);
    }

    /* Unmap the mapped shared memory segment from the address space of the process */
    if (munmap(pshm_1_base, PSHM_1_NUM_OF_PROD * sizeof(SHMSEG)) < 0) { error("[CAP ] munmap"); }

    /* Close the semaphore */
    if (sem_close(cap_sem) < 0) { error("[CAP ] sem_close"); }
}

////////////////////////////////////////////////////////////////////////////////////////////

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
    if ((pshm_1_fd = shm_open(PSHM_1_NAME,                                  /* name of object */
                              O_RDONLY,                                     /* open object for read only access */
                              0                                             /* mode - specified 0 for opening existing object */
                              )) < 0) { error("[UTX ] shm_open"); }

    /* Map shared memory segment to address space of process */
    if ((pshm_1_base = (SHMSEG *)mmap(NULL,                                 /* system chooses where to place shm in virtual address space */
                                      PSHM_1_NUM_OF_PROD * sizeof(SHMSEG),  /* size of mapping */
                                      PROT_READ,                            /* read mapping */
                                      MAP_SHARED,                           /* make modifications to shm visible to other processes */
                                      pshm_1_fd,                            /* file descriptor specifying file to map */
                                      0                                     /* offset of mapping in shm file */
                                      )) < 0) { error("[UTX ] mmap"); }

    /* Close shared memory file descriptor */
    if (close(pshm_1_fd) < 0) { error("[UTX ] close"); }

    /* Open the semaphore */
    if ((lux_sem = sem_open(lux_sem_name, 0, 0600, 0)) < 0) { error("[UTX ] sem_open"); }
    if ((cap_sem = sem_open(cap_sem_name, 0, 0600, 0)) < 0) { error("[UTX ] sem_open"); }

    while(1)
    {
        // wait for semaphore
        sem_wait(cap_sem);
        sem_wait(lux_sem);

        memcpy((void*)shmseg_utx_ptr, (void*)(&pshm_1_base[LUX]), sizeof(SHMSEG));
        PDEBUG("[UTX ] shmseg_lux.sensor = %d\n", shmseg_utx.sensor);
        PDEBUG("[UTX ] shmseg_lux.data = %d\n", shmseg_utx.data);

        memcpy((void*)shmseg_utx_ptr, (void*)(&pshm_1_base[CAP]), sizeof(SHMSEG));
        PDEBUG("[UTX ] shmseg_cap.sensor = %d\n", shmseg_utx.sensor);
        PDEBUG("[UTX ] shmseg_cap.data = %d\n", shmseg_utx.data);
    }

    /* Unmap the mapped shared memory segment from the address space of the process */
    if (munmap(pshm_1_base, PSHM_1_NUM_OF_PROD * sizeof(SHMSEG)) < 0) { error("[UTX ] munmap"); }

    /* Close the semaphore */
    if (sem_close(lux_sem) < 0) { error("[UTX ] sem_close"); }
    if (sem_close(cap_sem) < 0) { error("[UTX ] sem_close"); }
}

////////////////////////////////////////////////////////////////////////////////////////////

/* Logger Task 
 * Consumer 2 for Shared Memory */
void log_task(void)
{
    syslog(LOG_DEBUG, "[LOG ] Task Started - PID %ld\n", (long)getpid());
    PDEBUG("[LOG ] Task Started - PID %ld\n", (long)getpid());

    // SHMSEG *shmseg_log;
}

////////////////////////////////////////////////////////////////////////////////////////////

uint8_t lux_sensor_init(void)
{
    char filename[20];
    unsigned char apds9301_id;
    unsigned char apds9301_power_state;
    unsigned char apds9301_power_on = APDS9301_POWER_ON;
    // unsigned char apds9301_power_off = APDS9301_POWER_OFF;

    /* Open i2c file */
    snprintf(filename, 19, "/dev/i2c-%d", I2C_ADAPTER_NR);
    if((i2c_fd = open(filename, O_RDWR)) < 0) {
        perror("[LUX ] [APDS9301] i2c - file open error");
        return EXIT_FAILURE;
    }


    /* Select slave address */
    if(ioctl(i2c_fd, I2C_SLAVE, APDS9301_SLAVE_ADDR) < 0) {
        perror("[LUX ] [APDS9301] i2c - ioctl error");
        return EXIT_FAILURE;
    }


    /* Power on the sensor if off */
    if(apds9301_read_reg_byte(i2c_fd, APDS9301_SLAVE_ADDR, (APDS9301_CMD_REG | APDS9301_CTRL_REG), &apds9301_power_state) != 0) {
        perror("[LUX ] [APDS9301] apds9301_read_reg_byte");
        return EXIT_FAILURE;
    }
    printf("[LUX ] [APDS9301] CTRL reg value: 0x%02x\n", apds9301_power_state);
    if(apds9301_power_state == 0x00) {
        printf("[LUX ] [APDS9301] Powering up Lux sensor\n");

        if(apds9301_write_reg_byte(i2c_fd, APDS9301_SLAVE_ADDR, (APDS9301_CMD_REG | APDS9301_CTRL_REG), &apds9301_power_on) != 0) {
            perror("apds9301_write_reg_byte");
            return EXIT_FAILURE;
        }
    }
    printf("[LUX ] [APDS9301] Lux sensor powered up\n");


    /* Read device ID */
    if(apds9301_read_reg_byte(i2c_fd, APDS9301_SLAVE_ADDR, (APDS9301_CMD_REG | APDS9301_ID_REG), &apds9301_id) != 0) {
        perror("[LUX ] [APDS9301] apds9301_read_reg_byte");
        return EXIT_FAILURE;
    }
    printf("[LUX ] [APDS9301] Lux Sensor Device ID: 0x%02x\n", apds9301_id);
    printf("[LUX ] [APDS9301] Lux Sensor Initialized\n");

    return EXIT_SUCCESS;
}

uint8_t get_lux_value(void)
{
    unsigned char ch0_data_low, ch0_data_high;
    unsigned char ch1_data_low, ch1_data_high;
    uint16_t ch0_data, ch1_data;
    float lux_flt;
    uint8_t lux_ret;
    
    PDEBUG("\n[LUX ] [APDS9301] Reading CH0 byte by byte\n");
    if(apds9301_read_reg_byte(i2c_fd, APDS9301_SLAVE_ADDR, (APDS9301_CMD_REG | APDS9301_CH0_DATA_LOW), &ch0_data_low) != 0) {
        perror("[LUX ] [APDS9301] apds9301_read_reg_byte");
        return EXIT_FAILURE;
    }
    if(apds9301_read_reg_byte(i2c_fd, APDS9301_SLAVE_ADDR, (APDS9301_CMD_REG | APDS9301_CH0_DATA_HIGH), &ch0_data_high) != 0) {
        perror("[LUX ] [APDS9301] apds9301_read_reg_byte");
        return EXIT_FAILURE;
    }
    PDEBUG("[LUX ] [APDS9301] CH0_DATA_LOW  = 0x%02x\n", ch0_data_low);
    PDEBUG("[LUX ] [APDS9301] CH0_DATA_HIGH = 0x%02x\n", ch0_data_high);

    ch0_data = (ch0_data_high << 8) | (ch0_data_low);
    PDEBUG("[LUX ] [APDS9301] CH0_VALUE     = 0x%04x = %d\n", ch0_data, ch0_data);

    PDEBUG("[LUX ] [APDS9301] Reading CH1 byte by byte\n");
    if(apds9301_read_reg_byte(i2c_fd, APDS9301_SLAVE_ADDR, (APDS9301_CMD_REG | APDS9301_CH1_DATA_LOW), &ch1_data_low) != 0) {
        perror("[LUX ] [APDS9301] apds9301_read_reg_byte");
        return EXIT_FAILURE;
    }
    if(apds9301_read_reg_byte(i2c_fd, APDS9301_SLAVE_ADDR, (APDS9301_CMD_REG | APDS9301_CH1_DATA_HIGH), &ch1_data_high) != 0) {
        perror("[LUX ] [APDS9301] apds9301_read_reg_byte");
        return EXIT_FAILURE;
    }
    PDEBUG("[LUX ] [APDS9301] CH1_DATA_LOW  = 0x%02x\n", ch1_data_low);
    PDEBUG("[LUX ] [APDS9301] CH1_DATA_HIGH = 0x%02x\n", ch1_data_high);

    ch1_data = (ch1_data_high << 8) | (ch1_data_low);
    PDEBUG("[LUX ] [APDS9301] CH1_VALUE     = 0x%04x = %d\n", ch1_data, ch1_data);

    lux_flt = calculate_lux(ch0_data, ch1_data);
    PDEBUG("[LUX ] [APDS9301] lux flt value = %0.2f\n", lux_flt);

    lux_ret = (lux_flt < 255.00) ? (uint8_t)lux_flt : 255;
    PDEBUG("[LUX ] [APDS9301] lux ret value = %d\n", lux_ret);

    return lux_ret;
}

int apds9301_read_reg_byte(int file, unsigned char addr, unsigned char cmd_reg, unsigned char* data)
{
    unsigned char ipbuff;
    struct i2c_rdwr_ioctl_data packets;
    struct i2c_msg messages[2];

    /* Set transaction segment 1 message - write command to APDS9301 */
    messages[0].addr  = addr;               // slave address
    messages[0].flags = 0x00;               // write bit
    messages[0].len   = sizeof(cmd_reg);    // size of command buffer
    messages[0].buf   = &cmd_reg;           // command + address of register of interest

    /* Set transaction segment 2 message - read register data from APDS9301 */
    messages[1].addr  = addr;               // slave address
    messages[1].flags = I2C_M_RD;           // read bit - I2C_M_RD defined as 0x0001 in i2c.h
    messages[1].len   = sizeof(ipbuff);     // size of input buffer
    messages[1].buf   = &ipbuff;            // buffer to store register data

    /* Perform combined W/R transaction */
    packets.msgs      = messages;
    packets.nmsgs     = 2;
    if(ioctl(file, I2C_RDWR, &packets) < 0) {
        perror("[LUX ] [APDS9301] i2c - apds9301_read_reg_byte - ioctl rdwr");
        return EXIT_FAILURE;
    }

    /* Store register data in calling thread */
    *data = ipbuff;

    return EXIT_SUCCESS;
}

int apds9301_write_reg_byte(int file, unsigned char addr, unsigned char cmd_reg, unsigned char* data)
{
    unsigned char wrbuff[2] = { cmd_reg, *data };
    struct i2c_rdwr_ioctl_data packets;
    struct i2c_msg message;

    /* Set transaction segment message - write command and data to APDS9301 */
    message.addr    = addr;                 // slave address
    message.flags   = 0x00;                 // write bit
    message.len     = sizeof(wrbuff);       // size of write buffer
    message.buf     = wrbuff;               // command + address of register of interest and data to be written

    /* Perform 2 writes transaction */
    packets.msgs    = &message;
    packets.nmsgs   = 1;
    if(ioctl(file, I2C_RDWR, &packets) < 0) {
        perror("[LUX ] [APDS9301] i2c - apds9301_write_reg_byte - ioctl rdwr");
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}

float calculate_lux(uint16_t ch0, uint16_t ch1)
{
    float ratio, lux;
    
    // ratio = CH1/CH0
    ratio = (float)ch1 / ch0;

    if((ratio > 0) && (ratio <= 0.50)) {
        lux = (0.0304 * ch0) - (0.062 * ch0 * (float)pow((float)ch1/ch0, 1.4));
    }
    else if (ratio <= 0.61) {
        lux = (0.0224 * ch0) - (0.031 * ch1);
    }
    else if (ratio <= 0.80) {
        lux = (0.0128 * ch0) - (0.0153 * ch1);
    }
    else if (ratio <= 1.30) {
        lux = (0.00146 * ch0) - (0.00112 * ch1);
    }
    else {
        lux = 0;
    }

    return lux;
}