#define NUM_OF_TASKS          (7)

/* Shared memory defines */
#define PSHM_1_NAME           ("/pshm_1")
#define PSHM_2_NAME           ("/pshm_2")
#define PSHM_1_NUM_OF_PROD    (2)
#define PSHM_2_NUM_OF_PROD    (1)

#define AESD_DEBUG 1  //Remove comment on this line to enable debug

#undef PDEBUG             /* undef it, just in case */
#ifdef AESD_DEBUG
#  ifdef __KERNEL__
     /* This one if debugging is on, and kernel space */
#    define PDEBUG(fmt, args...) printk( KERN_DEBUG "aesdchar: " fmt, ## args)
#  else
     /* This one for user space */
#    define PDEBUG(fmt, args...) fprintf(stderr, fmt, ## args)
#  endif
#else
#  define PDEBUG(fmt, args...) /* not debugging: nothing */
#endif

/* Enum for sensor */
enum { LUX = 0, 
       CAP };

/* Enum for actuator */
enum { LED = 0, 
       BUZ };

/* Shared memory 1 segment data */
typedef struct {
     uint8_t sensor;
     uint8_t error;
     uint16_t data;
} SHMSEG_1;

/* Shared memory 2 segment data */
typedef struct {
     uint8_t actuator; 
     uint8_t value;
} SHMSEG_2;

/* Shared memory synchronization semaphores */
char* cap_sem_name = "cap_sem";
char* lux_sem_name = "lux_sem";
char* act_sem_name = "act_sem";
char* log_sem_name = "log_sem";
char* urx_sem_1_name = "urx_sem_1";
// char* urx_sem_2_name = "urx_sem_2";

volatile sig_atomic_t signal_flag;

/*** Lux sensor ***/
#include <math.h>
#include <sys/ioctl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>

#define I2C_ADAPTER_NR              (2)
#define APDS9301_SLAVE_ADDR         (0x39)

#define APDS9301_POWER_OFF          (0x00)
#define APDS9301_POWER_ON           (0x03)

/* Registers */
#define APDS9301_CMD_REG            (0x80)
#define APDS9301_CTRL_REG           (0x00)
#define APDS9301_TIMING_REG         (0x01)
#define APDS9301_ID_REG             (0x0A)
#define APDS9301_CH0_DATA_LOW       (0x0C)
#define APDS9301_CH0_DATA_HIGH      (0x0D)
#define APDS9301_CH1_DATA_LOW       (0x0E)
#define APDS9301_CH1_DATA_HIGH      (0x0F)

int i2c_fd;

/*** UART Tx ***/
#include <termios.h>

int uart_fd;

/*** Capacitive sensor ***/
#include "../inc/gpio.h"

#define ON              (1)
#define OFF             (0)

// #define CAP_LED_GPIO    (53)       // USR LED 0
#define CAP_DATA_GPIO   (69)       // P8_09

/*** Actuators ***/
#define ACT_LED_GPIO    (53)       // USR LED 0
#define ACT_BUZ_GPIO    (67)       // P8_08

/*** Socket ***/
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <linux/fs.h>