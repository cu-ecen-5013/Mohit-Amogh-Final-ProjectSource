/*
 * aesdchar.h
 *
 *  Created on: Oct 23, 2019
 *      Author: Dan Walkes
 */

#ifndef AESD_CHAR_DRIVER_AESDCHAR_H_
#define AESD_CHAR_DRIVER_AESDCHAR_H_

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

// Circular Buffer Size
#define CB_SIZE         (10)

// Circular Buffer Structure
struct CB_struct
{
	int head;				/* index for new write in buffer		*/
	int tail;				/* index for read location				*/
    int fill_size;          /* stores current filled size of CB     */
	int size[CB_SIZE];		/* stores size of each buffer element 	*/
	char* data[CB_SIZE];	/* stores string						*/
};

// structure used by ioctl and read call
// ioctl populates this structure which is later accessed by read
struct ioc_struct
{
	int id;					/* id retrieved from ioctl command		*/
	int head;				/* index for head for ioctl command		*/
	int tail;				/* index for tail for ioctl command		*/
};

struct aesd_dev
{
    unsigned long size;		// unused - size of device file
	struct CB_struct CB[2];
	struct ioc_struct ioc; 
	struct mutex lock;
	struct cdev cdev;	  /* Char device structure		*/
};


#endif /* AESD_CHAR_DRIVER_AESDCHAR_H_ */
