/**
 * @file aesdchar.c
 * @brief Functions and data related to the AESD char driver implementation
 *
 * Based on the implementation of the "scull" device driver, found in
 * Linux Device Drivers example code.
 *
 * @author Dan Walkes
 * @date 2019-10-22
 * @copyright Copyright (c) 2019
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/printk.h>
#include <linux/types.h>
#include <linux/cdev.h>
#include <linux/fs.h> 	// file_operations
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/mutex.h>
#include <linux/string.h>
#include "aesdchar.h"
#include "aesd_ioctl.h"

int aesd_major =   0; 	// use dynamic major
int aesd_minor =   0;

MODULE_AUTHOR("Mohit Rane");
MODULE_LICENSE("Dual BSD/GPL");

struct aesd_dev aesd_device;

/* OPEN METHOD */
int aesd_open(struct inode *inode, struct file *filp)
{
    struct aesd_dev *dev; /* device information */
    
	PDEBUG("[DRV ] [OPEN ] in open\n");
    
    dev = container_of(inode->i_cdev, struct aesd_dev, cdev);
    filp->private_data = dev; /* for other methods */

    return 0;
}

/* RELEASE METHOD */
int aesd_release(struct inode *inode, struct file *filp)
{
	PDEBUG("[DRV ] [CLOSE] in close\n");
    return 0;
}

/* IOCTL METHOD */
long aesd_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int retval = 0;
    int j = 0;
	// int size = 0;
	// loff_t newpos;
	// int res;
	struct aesd_read_data recv_data;
	int cnt = 0;

	struct aesd_dev *dev = filp->private_data;

	// if(mutex_lock_interruptible(&dev->lock)) { return -ERESTARTSYS; }
	// PDEBUG("MUTEX LOCKED\n");

	if (_IOC_TYPE(cmd) != AESD_IOC_MAGIC) return -ENOTTY;
	if (_IOC_NR(cmd) > AESDCHAR_IOC_MAXNR) return -ENOTTY;

	switch(cmd) {
	  case IOC_GET_DATA:
		copy_from_user(&recv_data, (const void __user*)arg, sizeof(recv_data));
		PDEBUG("[DRV ] [ICOTL] received id  : %u\n", recv_data.id);
		PDEBUG("[DRV ] [ICOTL] received num : %u\n", recv_data.num);

        // store id
        dev->ioc.id = recv_data.id;

        // store current head
        dev->ioc.head = dev->CB[recv_data.id].head;
		
        // check how filled the circular buffer is
        for(j=0; j<CB_SIZE; j++)
		{
			if(dev->CB[recv_data.id].data[j] != NULL) { cnt++; }
			else { break; }
		}

        // store tail according to how much buffer is currently filled (cnt) and num
        if(cnt == CB_SIZE)
            dev->ioc.tail = (dev->CB[recv_data.id].tail + (CB_SIZE - recv_data.num)) % CB_SIZE;
        else if(recv_data.num <= cnt)
            dev->ioc.tail = cnt - recv_data.num;
        else
            dev->ioc.tail = 0;

        PDEBUG("[DRV ] [ICOTL] CB[%d] ioc_head : %u\n", dev->ioc.id, dev->ioc.head);
        PDEBUG("[DRV ] [ICOTL] CB[%d] ioc_tail : %u\n", dev->ioc.id, dev->ioc.tail);

		break;

	  default:
		retval = -ENOTTY;
		goto out;
	}

	out:
		// mutex_unlock(&dev->lock);
		// PDEBUG("MUTEX UNLOCKED\n");
		return retval;
}

/* READ METHOD */
ssize_t aesd_read(struct file *filp, char __user *buf, size_t count,
                loff_t *f_pos)
{
	ssize_t retval = 0;
    struct aesd_dev *dev = filp->private_data;
    int i;                      // for iterations
    unsigned long send_size;    // store size of data that needs to be sent to user
    int id;
    int l_head;
	int l_tail;

	if(mutex_lock_interruptible(&dev->lock)) { return -ERESTARTSYS; }
	PDEBUG("[DRV ] [READ ] mutex locked\n");

    // store ioc variables locally
	id = dev->ioc.id;
	l_head = dev->ioc.head;
	l_tail = dev->ioc.tail;

    PDEBUG("[DRV ] [READ ] id     : %d\n", id);
    PDEBUG("[DRV ] [READ ] l_head : %d\n", l_head);
    PDEBUG("[DRV ] [READ ] l_tail : %d\n", l_tail);

    PDEBUG("[DRV ] [READ ] dev->CB[%d].fill_size = %d and f_pos = %lld\n", id, dev->CB[id].fill_size, *f_pos);

    // calculate size of data that needs to be send to user
	send_size = 0;
    for(i=0; i<CB_SIZE; i++)
    {
        // break if local tail reaches head where new data will be stored
        if(l_tail == l_head) { break; }

        // update size
        send_size += dev->CB[id].size[l_tail];

		// increment l_tail
		l_tail = (l_tail + 1) % CB_SIZE;
    }
    PDEBUG("[DRV ] [READ ] send_size = %ld\n", send_size);

	// return when there is no more content left to read
	if (*f_pos >= send_size)
		goto out;

    // restore l_tail value
    l_tail = dev->ioc.tail;

	// send device file contents
	for(i=0; i<CB_SIZE; i++)
	{
		PDEBUG("[DRV ] [READ ] l_tail = %d\n", l_tail);

        // break if local tail reaches head where new data will be stored
        if(l_tail == l_head) { break; }

		// send data to user	
		if (copy_to_user(buf + *f_pos, dev->CB[id].data[l_tail], dev->CB[id].size[l_tail])) { return -EFAULT; }
		PDEBUG("[DRV ] [READ ] CB[%d][%d] = %s; size = %d\n", id, l_tail, dev->CB[id].data[l_tail], dev->CB[id].size[l_tail]);
		
		// update file offset and retval
		*f_pos += dev->CB[id].size[l_tail];
		retval += dev->CB[id].size[l_tail];
		PDEBUG("[DRV ] [READ ] f_pos = %lld\n", *f_pos);

		// increment l_tail
		l_tail = (l_tail + 1) % CB_SIZE;
	}

	PDEBUG("[DRV ] [READ ] retval = %ld\n", retval);

	out:
		mutex_unlock(&dev->lock);
		PDEBUG("[DRV ] [READ ] mutex unlocked\n");
		return retval;
}

/* WRITE METHOD */
ssize_t aesd_write(struct file *filp, const char __user *buf, size_t count,
                loff_t *f_pos)
{
	struct aesd_dev *dev = filp->private_data;
    ssize_t retval = -ENOMEM;
	int id = 0;
    int j = 0;
    retval = count;

	if(mutex_lock_interruptible(&dev->lock)) { return -ERESTARTSYS; }
	PDEBUG("[DRV ] [WRITE] mutex locked\n");

    PDEBUG("[DRV ] [WRITE] write %zu bytes with offset %lld\n",count,*f_pos);

    // compare string to get id number
    // loop will break after id matches
    for(id=0; id<NUM_OF_IDS; id++)
    {
        if ((strncmp(id_str[id], &buf[28], 3) == 0))
		{
            break;
		}
    }

	// if overwrite then free previous data
	if (dev->CB[id].data[dev->CB[id].head] != NULL)
	{
		dev->CB[id].tail = dev->CB[id].head + 1;
		kfree(dev->CB[id].data[dev->CB[id].head]);
	}

	// insert received data
	// update size of character array (CB[id].data)
    dev->CB[id].data[dev->CB[id].head] = (char*)kmalloc(count * sizeof(char), GFP_KERNEL);
    if (copy_from_user(dev->CB[id].data[dev->CB[id].head], buf, count)) { retval = -EFAULT; }
    dev->CB[id].size[dev->CB[id].head] = count;

	PDEBUG("[DRV ] [WRITE] CB[%d][%d]	    : %s\n", id, dev->CB[id].head, dev->CB[id].data[dev->CB[id].head]);
	PDEBUG("[DRV ] [WRITE] CB[%d] size  	: %d\n", id, dev->CB[id].size[dev->CB[id].head]);

	// increment head pointer
	dev->CB[id].head = (dev->CB[id].head + 1) % CB_SIZE;
	PDEBUG("[DRV ] [WRITE] CB[%d] head  	: %d\n", id, dev->CB[id].head);
    PDEBUG("[DRV ] [WRITE] CB[%d] tail  	: %d\n", id, dev->CB[id].tail);

	// update current filled circular buffer size
    dev->CB[id].fill_size = 0;
	for(j=0; j<CB_SIZE; j++)
		dev->CB[id].fill_size += dev->CB[id].size[j];
	PDEBUG("[DRV ] [WRITE] CB[%d] fill_size : %d\n", id, dev->CB[id].fill_size);

	mutex_unlock(&dev->lock);
	PDEBUG("[DRV ] [WRITE] mutex unlocked\n");

    return retval;
}

/* file operations structure for character driver */
struct file_operations aesd_fops = {
	.owner =    THIS_MODULE,
	.read =     aesd_read,
	.write =    aesd_write,
    .unlocked_ioctl  =	aesd_ioctl,
	.open =     aesd_open,
	.release =  aesd_release,
};

/* character device setup called during module initialization */
static int aesd_setup_cdev(struct aesd_dev *dev)
{
	int err, devno = MKDEV(aesd_major, aesd_minor);

	cdev_init(&dev->cdev, &aesd_fops);
	dev->cdev.owner = THIS_MODULE;
	dev->cdev.ops = &aesd_fops;
	err = cdev_add (&dev->cdev, devno, 1);
	if (err) {
		printk(KERN_ERR "Error %d adding aesd cdev", err);
	}
	return err;
}


/* This function is called when module is inserted in kernel */
int aesd_init_module(void)
{
	dev_t dev = 0;
	int result;
	result = alloc_chrdev_region(&dev, aesd_minor, 1,
			"aesdchar");
	aesd_major = MAJOR(dev);
	if (result < 0) {
		printk(KERN_WARNING "Can't get major %d\n", aesd_major);
		return result;
	}

	mutex_init(&aesd_device.lock);

	memset(&aesd_device, 0, sizeof(struct aesd_dev));

	result = aesd_setup_cdev(&aesd_device);

	if( result ) {
		unregister_chrdev_region(dev, 1);
	}
	return result;

}

/* This function is called when module is removed from kernel */
void aesd_cleanup_module(void)
{
	dev_t devno = MKDEV(aesd_major, aesd_minor);
    int i, j;

	cdev_del(&aesd_device.cdev);

    // free up allocated circular buffers
    for(i=0; i<NUM_OF_IDS; i++)
    {
        for(j=0; j<CB_SIZE; j++)
        {
            if(aesd_device.CB[i].data[j] != NULL)
                kfree(aesd_device.CB[i].data[j]);
        }
    }

	unregister_chrdev_region(devno, 1);
}

module_init(aesd_init_module);
module_exit(aesd_cleanup_module);