/* APDS9301 Lux Sensor Code */

/* This code reads APDS9301 Lux Sensor ID register */

/* References:
 * 1. Exploring BeagleBone by Derek Molloy 
 * 2. https://elinux.org/Interfacing_with_I2C_Devices
 * 3. https://www.kernel.org/doc/Documentation/i2c/dev-interface
 * 4. https://cdn.sparkfun.com/assets/3/2/c/0/8/AV02-2315EN0.pdf
 */

#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <errno.h>

#define I2C_ADAPTER_NR          (2)

#define APDS9301_READ_BIT       (1)
#define APDS9301_WRITE_BIT      (0)

#define APDS9301_SLAVE_ADDR     (0x39)
#define APDS9301_CMD_REG        (0x80)
#define APDS9301_ID_REG         (0x0A | APDS9301_CMD_REG)

#define APDS9301_SLAVE_READ     ((APDS9301_SLAVE_ADDR << 1) | APDS9301_READ_BIT)
#define APDS9301_SLAVE_WRITE    ((APDS9301_SLAVE_ADDR << 1) | APDS9301_WRITE_BIT)


int main(void)
{
    printf("APDS9301 Lux Sensor Test\n");

    int i2c_fd;
    char filename[20];
    
    // open i2c file
    snprintf(filename, 19, "/dev/i2c-%d", I2C_ADAPTER_NR);
    if((i2c_fd = open(filename, O_RDWR)) < 0) {
        perror("i2c - file open error");
        return -1;
    }

    // select slave address
    if(ioctl(i2c_fd, I2C_SLAVE, APDS9301_SLAVE_ADDR) < 0) {
        perror("i2c - ioctl error");
        return -1;
    }

    // reset read address
    char write_buff[3] = { APDS9301_SLAVE_WRITE, APDS9301_ID_REG, APDS9301_SLAVE_READ };
    if(write(i2c_fd, write_buff, 3) != 3) {
        perror("i2c - failed read initialization");
        return -1;
    }

    // read i2c register
    char read_byte;
    if(read(i2c_fd, &read_byte, 1) != 1) {
        perror("i2c - failed to read in buffer");
        return -1;
    }

    printf("Lux Sensor Device ID: 0x%02x\n", read_byte);

    // close i2c file
    if(close(i2c_fd) < 0) {
        perror("i2c - file close error");
        return -1;
    }

    return 0;
}
