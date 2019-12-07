#define NUM_OF_TASKS     (4)

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

enum { LUX = 1, 
       CAP };

/* Shared memory segment data */
typedef struct {
     uint8_t sensor; 
     uint8_t data;
} SHMSEG;

/* Shared memory synchronization semaphores */
char* cap_sem_name = "cap_sem";
char* lux_sem_name = "lux_sem";