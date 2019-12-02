/* I2C Lux sensor code */
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

#define I2C_ADDR    (0x39)
#define DEVID       (0x8A)
#define BUFF_SIZE   (1)

int main(void)
{
    int i2c_fd;
    printf("APDS-9301 Lux Sensor Test\n");
    
    // open i2c file
    if((i2c_fd = open("/dev/i2c-2", O_RDWR)) < 0) {
        perror("file open error\n");
        return -1;
    }

    // select slave address
    if(ioctl(i2c_fd, I2C_SLAVE, I2C_ADDR) < 0) {
        perror("i2c ioctl error\n");
        return -1;
    }

    // reset read address
    char write_buff[1] = { 0x8A };
    if(write(i2c_fd, write_buff, 1) != 1) {
        perror("failed resetting read address\n");
        return -1;
    }

    // read i2c-2 registers
    char read_buff[BUFF_SIZE];
    if(read(i2c_fd, read_buff, BUFF_SIZE) != BUFF_SIZE) {
        perror("failed to read in buffer\n");
        return -1;
    }

    printf("Lux Sensor Device ID: 0x%02x\n", read_buff[DEVID]);
    close(i2c_fd);
    return 0;
}
