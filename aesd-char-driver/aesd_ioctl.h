/*
 * aesd_ioctl.h
 *
 *  Created on: Oct 23, 2019
 *      Author: Dan Walkes
 *
 *  @brief Definitins for the ioctl used on aesd char devices for assignment 9
 */

#ifndef AESD_IOCTL_H
#define AESD_IOCTL_H

#ifdef __KERNEL__
#include <asm-generic/ioctl.h>
#include <linux/types.h>
#else
#include <sys/ioctl.h>
#include <stdint.h>
#endif

/**
 * A structure to be passed by IOCTL from user space to kernel space, describing the type
 * of data that needs to be retrieved
 */
struct aesd_read_data {
	/**
	 * The id of data to be retrieved
	 */
	uint8_t id;
	/**
	 * The number of strings to be retrieved
	 */
	uint16_t num;
};

// Pick an arbitrary unused value from https://github.com/torvalds/linux/blob/master/Documentation/ioctl/ioctl-number.rst
#define AESD_IOC_MAGIC 0x16

// Define a write command from the user point of view, use command number 1
#define IOC_GET_DATA _IOWR(AESD_IOC_MAGIC, 1, struct aesd_read_data)
/**
 * The maximum number of commands supported, used for bounds checking
 */
#define AESDCHAR_IOC_MAXNR 1

#endif /* AESD_IOCTL_H */
