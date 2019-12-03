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
#define APDS9301_ID_REG         (0x0A)
// #define APDS9301_ID_REG         (0x0A | APDS9301_CMD_REG)

#define APDS9301_SLAVE_READ     ((APDS9301_SLAVE_ADDR << 1) | APDS9301_READ_BIT)
#define APDS9301_SLAVE_WRITE    ((APDS9301_SLAVE_ADDR << 1) | APDS9301_WRITE_BIT)


int main(void)
{
    printf("APDS9301 Lux Sensor Test - 2\n");

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

#if 0
    // reset read address
    char write_buff[3] = { APDS9301_SLAVE_WRITE, APDS9301_ID_REG, APDS9301_SLAVE_READ };
    if(write(i2c_fd, write_buff, 3) != 3) {
        perror("i2c - read transaction failed");
        return -1;
    }

    // read i2c register
    char read_byte;
    if(read(i2c_fd, &read_byte, 1) != 1) {
        perror("i2c - failed to read in buffer");
        return -1;
    } else {
        printf("Lux Sensor Device ID: 0x%02x\n", read_byte);
    }
#endif

#if 1
    unsigned char in_buff, out_buff;
    struct i2c_rdwr_ioctl_data packets;
    struct i2c_msg messages[2];

    out_buff = APDS9301_CMD_REG | APDS9301_ID_REG;

    messages[0].addr  = APDS9301_SLAVE_ADDR;
    messages[0].flags = 0;
    messages[0].len   = sizeof(out_buff);
    messages[0].buf   = &out_buff;

    messages[1].addr  = APDS9301_SLAVE_ADDR;
    messages[1].flags = I2C_M_RD;
    messages[1].len   = sizeof(in_buff);
    messages[1].buf   = &in_buff;

    packets.msgs      = messages;
    packets.nmsgs     = 2;
    if(ioctl(i2c_fd, I2C_RDWR, &packets) < 0) {
        perror("i2c - ioctl rdwr");
        return -1;
    }

    printf("Lux Sensor Device ID: 0x%02x\n", in_buff);

#endif

#if 0
    // read all address
    char read_buff[150];
    if(read(i2c_fd, read_buff, 150) != 150) {
        perror("i2c - read failed");
        return -1;
    } else {
        for(int i=0; i<150; i++)
            printf("0x%02x\n", read_buff[i]);
    }
#endif

    // close i2c file
    if(close(i2c_fd) < 0) {
        perror("i2c - file close error");
        return -1;
    }

    return 0;
}
