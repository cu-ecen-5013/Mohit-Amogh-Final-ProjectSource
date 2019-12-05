/* APDS9301 Lux Sensor Code */

/* This code reads APDS9301 Lux Sensor ID register */

/* References:
 * 1. https://cdn.sparkfun.com/assets/3/2/c/0/8/AV02-2315EN0.pdf
 * 2. Exploring BeagleBone by Derek Molloy
 * 3. https://www.kernel.org/doc/Documentation/i2c/dev-interface
 * 4. https://elinux.org/Interfacing_with_I2C_Devices
 * 5. https://stackoverflow.com/questions/52975817/setup-i2c-reading-and-writing-in-c-language
 * 6. https://docs.huihoo.com/doxygen/linux/kernel/3.7/structi2c__msg.html
 * 7. https://stackoverflow.com/questions/25159064/why-are-i2c-smbus-function-not-available-i2c-embedded-linux
 */

#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <math.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <errno.h>

#define I2C_ADAPTER_NR              (2)
#define APDS9301_SLAVE_ADDR         (0x39)

#define APDS9301_POWER_OFF          (0x00)
#define APDS9301_POWER_ON           (0x03)

#define APDS9301_WORD_BIT           (1<<5)

/* Registers */
#define APDS9301_CMD_REG            (0x80)
#define APDS9301_CTRL_REG           (0x00)
#define APDS9301_TIMING_REG         (0x01)
#define APDS9301_ID_REG             (0x0A)
#define APDS9301_CH0_DATA_LOW       (0x0C)
#define APDS9301_CH0_DATA_HIGH      (0x0D)
#define APDS9301_CH1_DATA_LOW       (0x0E)
#define APDS9301_CH1_DATA_HIGH      (0x0F)


#if 1
/* Function declarations */
int apds9301_read_reg_byte(int file, unsigned char addr, unsigned char cmd_reg, unsigned char* data);
int apds9301_write_reg_byte(int file, unsigned char addr, unsigned char cmd_reg, unsigned char* data);
int apds9301_read_reg_word(int file, unsigned char addr, unsigned char cmd_reg, uint16_t* data);
int apds9301_write_reg_word(int file, unsigned char addr, unsigned char cmd_reg, unsigned char* data);
float calculate_lux(uint16_t ch0, uint16_t ch1);
#endif

#if 0
__s32 i2c_smbus_access(int file, char read_write, __u8 command,
		       int size, union i2c_smbus_data *data)
{
	struct i2c_smbus_ioctl_data args;
	__s32 err;

	args.read_write = read_write;
	args.command = command;
	args.size = size;
	args.data = data;

	err = ioctl(file, I2C_SMBUS, &args);
	if (err == -1)
		err = -errno;
	return err;
}

__s32 i2c_smbus_read_byte_data(int file, __u8 command)
{
	union i2c_smbus_data data;
	int err;

	err = i2c_smbus_access(file, I2C_SMBUS_READ, command,
			       I2C_SMBUS_BYTE_DATA, &data);
	if (err < 0)
		return err;

	return 0x0FF & data.byte;
}

__s32 i2c_smbus_read_word_data(int file, __u8 command)
{
	union i2c_smbus_data data;
	int err;

	err = i2c_smbus_access(file, I2C_SMBUS_READ, command,
			       I2C_SMBUS_WORD_DATA, &data);
	if (err < 0)
		return err;

	return 0x0FFFF & data.word;
}
#endif

int main(void)
{
    printf("APDS9301 Lux Sensor Test - X0\n");

    /***************************************************/
#if 0
    int ret;
    int i2c_fd;
    char filename[20];

    /* Open i2c file */
    snprintf(filename, 19, "/dev/i2c-%d", I2C_ADAPTER_NR);
    if((i2c_fd = open(filename, O_RDWR)) < 0) {
        perror("i2c - file open error");
        return -1;
    }

    /* Select slave address */
    if(ioctl(i2c_fd, I2C_SLAVE, 0x39) < 0) {
        perror("i2c - ioctl error");
        return -1;
    }

    // read byte -> 0x8C
    // read word -> 0xAC
    uint8_t byte;
    uint16_t word;

    printf("CH0\n");
    byte = i2c_smbus_read_byte_data(i2c_fd, 0x8c);
    printf("0x8c - byte = %d = 0x%02x\n", byte, byte);

    byte = i2c_smbus_read_byte_data(i2c_fd, 0x8d);
    printf("0x8d - byte = %d = 0x%02x\n", byte, byte);

    word = i2c_smbus_read_word_data(i2c_fd, 0xac);
    printf("0xac - word = %d = 0x%04x\n", word, word);

    printf("******************************\n");
    printf("CH1\n");
    byte = i2c_smbus_read_byte_data(i2c_fd, 0x8e);
    printf("0x8e - byte = %d = 0x%02x\n", byte, byte);

    byte = i2c_smbus_read_byte_data(i2c_fd, 0x8f);
    printf("0x8f - byte = %d = 0x%02x\n", byte, byte);

    word = i2c_smbus_read_word_data(i2c_fd, 0xae);
    printf("0xae - word = %d = 0x%04x\n", word, word);

#endif
    /***************************************************/

    int ret;
    int i2c_fd;
    char filename[20];
    unsigned char apds9301_id;
    unsigned char apds9301_power_state;
    uint16_t ch0_data, ch1_data;
    unsigned char ch0_data_low, ch0_data_high;
    unsigned char ch1_data_low, ch1_data_high;
    float lux;

    unsigned char apds9301_power_on = APDS9301_POWER_ON;
    unsigned char apds9301_power_off = APDS9301_POWER_OFF;
    
    /* Open i2c file */
    snprintf(filename, 19, "/dev/i2c-%d", I2C_ADAPTER_NR);
    if((i2c_fd = open(filename, O_RDWR)) < 0) {
        perror("i2c - file open error");
        return -1;
    }


    /* Select slave address */
    if(ioctl(i2c_fd, I2C_SLAVE, APDS9301_SLAVE_ADDR) < 0) {
        perror("i2c - ioctl error");
        return -1;
    }


    /* Power on the sensor if off */
    if((ret = apds9301_read_reg_byte(i2c_fd, APDS9301_SLAVE_ADDR, (APDS9301_CMD_REG | APDS9301_CTRL_REG), &apds9301_power_state)) != 0) {
        perror("apds9301_read_reg_byte");
        return -1;
    }
    printf("[APDS9301] CTRL reg value: 0x%02x\n", apds9301_power_state);
    if(apds9301_power_state == 0x00) {
        printf("[APDS9301] Powering up Lux sensor\n");

        if((ret = apds9301_write_reg_byte(i2c_fd, APDS9301_SLAVE_ADDR, (APDS9301_CMD_REG | APDS9301_CTRL_REG), &apds9301_power_on)) != 0) {
            perror("apds9301_write_reg_byte");
            return -1;
        }
    }
    printf("[APDS9301] Lux sensor powered up\n");


    /* Read device ID */
    if((ret = apds9301_read_reg_byte(i2c_fd, APDS9301_SLAVE_ADDR, (APDS9301_CMD_REG | APDS9301_ID_REG), &apds9301_id)) != 0) {
        perror("apds9301_read_reg_byte");
        return -1;
    }
    printf("[APDS9301] Lux Sensor Device ID: 0x%02x\n", apds9301_id);


    /* Read sensor values in an infinite loop */
    while(1)
    {
        // printf("\n[APDS9301] Reading CH0 word\n");
        // if((ret = apds9301_read_reg_word(i2c_fd, APDS9301_SLAVE_ADDR, (APDS9301_CMD_REG | APDS9301_WORD_BIT | APDS9301_CH0_DATA_LOW), &ch0_data)) != 0) {
        //     perror("apds9301_read_reg_word");
        //     return -1;
        // }
        // printf("[APDS9301] CH0_VALUE     = 0x%04x = %d\n", ch0_data, ch0_data);


        printf("\n[APDS9301] Reading CH0 byte by byte\n");
        if((ret = apds9301_read_reg_byte(i2c_fd, APDS9301_SLAVE_ADDR, (APDS9301_CMD_REG | APDS9301_CH0_DATA_LOW), &ch0_data_low)) != 0) {
            perror("apds9301_read_reg_byte");
            return -1;
        }
        if((ret = apds9301_read_reg_byte(i2c_fd, APDS9301_SLAVE_ADDR, (APDS9301_CMD_REG | APDS9301_CH0_DATA_HIGH), &ch0_data_high)) != 0) {
            perror("apds9301_read_reg_byte");
            return -1;
        }
        printf("[APDS9301] CH0_DATA_LOW  = 0x%02x\n", ch0_data_low);
        printf("[APDS9301] CH0_DATA_HIGH = 0x%02x\n", ch0_data_high);

        ch0_data = (ch0_data_high << 8) | (ch0_data_low);
        printf("[APDS9301] CH0_VALUE     = 0x%04x = %d\n", ch0_data, ch0_data);


        printf("\n");


        // printf("\n[APDS9301] Reading CH1 word\n");
        // if((ret = apds9301_read_reg_word(i2c_fd, APDS9301_SLAVE_ADDR, (APDS9301_CMD_REG | APDS9301_WORD_BIT | APDS9301_CH1_DATA_LOW), &ch1_data)) != 0) {
        //     perror("apds9301_read_reg_word");
        //     return -1;
        // }
        // printf("[APDS9301] CH1_VALUE     = 0x%04x = %d\n", ch1_data, ch1_data);


        printf("\n[APDS9301] Reading CH1 byte by byte\n");
        if((ret = apds9301_read_reg_byte(i2c_fd, APDS9301_SLAVE_ADDR, (APDS9301_CMD_REG | APDS9301_CH1_DATA_LOW), &ch1_data_low)) != 0) {
            perror("apds9301_read_reg_byte");
            return -1;
        }
        if((ret = apds9301_read_reg_byte(i2c_fd, APDS9301_SLAVE_ADDR, (APDS9301_CMD_REG | APDS9301_CH1_DATA_HIGH), &ch1_data_high)) != 0) {
            perror("apds9301_read_reg_byte");
            return -1;
        }
        printf("[APDS9301] CH1_DATA_LOW  = 0x%02x\n", ch1_data_low);
        printf("[APDS9301] CH1_DATA_HIGH = 0x%02x\n", ch1_data_high);

        ch1_data = (ch1_data_high << 8) | (ch1_data_low);
        printf("[APDS9301] CH1_VALUE     = 0x%04x = %d\n", ch1_data, ch1_data);

        lux = calculate_lux(ch0_data, ch1_data);
        printf("\nLUX = %0.2f\n", lux);

        printf("\n***************************************\n");
        sleep(2);
    }


    /* Close i2c file */
    if(close(i2c_fd) < 0) {
        perror("i2c - file close error");
        return -1;
    }

    printf("[APDS9301] Test Complete\n");
    return 0;
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
        perror("i2c - apds9301_read_reg_byte - ioctl rdwr");
        return -1;
    }

    /* Store register data in calling thread */
    *data = ipbuff;

    return 0;
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
        perror("i2c - apds9301_write_reg_byte - ioctl rdwr");
        return -1;
    }

    return 0;
}


int apds9301_read_reg_word(int file, unsigned char addr, unsigned char cmd_reg, uint16_t* data)
{
    unsigned char ipbuff[2];
    struct i2c_rdwr_ioctl_data packets;
    struct i2c_msg messages[4];

    unsigned char cmd_reg_2 = cmd_reg + 1;

    /* Set transaction segment 1 message - write command to APDS9301 */
    messages[0].addr  = addr;               // slave address
    messages[0].flags = 0x00;               // write bit
    messages[0].len   = sizeof(cmd_reg);    // size of command buffer
    messages[0].buf   = &cmd_reg;           // command + address of register of interest

    /* Set transaction segment 2 message - read register data from APDS9301 */
    messages[1].addr  = addr;               // slave address
    messages[1].flags = I2C_M_RD;           // read bit - I2C_M_RD defined as 0x0001 in i2c.h
    messages[1].len   = sizeof(ipbuff[0]);  // size of input buffer
    messages[1].buf   = &ipbuff[0];          // buffer to store register data

    /* Set transaction segment 3 message - write command to APDS9301 */
    messages[2].addr  = addr;               // slave address
    messages[2].flags = 0x00;               // write bit
    messages[2].len   = sizeof(cmd_reg_2);    // size of command buffer
    messages[2].buf   = &cmd_reg_2;           // command + address of register of interest

    /* Set transaction segment 4 message - read register data from APDS9301 */
    messages[3].addr  = addr;               // slave address
    messages[3].flags = I2C_M_RD;           // read bit - I2C_M_RD defined as 0x0001 in i2c.h
    messages[3].len   = sizeof(ipbuff[1]);  // size of input buffer
    messages[3].buf   = &ipbuff[1];          // buffer to store register data

    /* Perform combined W/R transaction */
    packets.msgs      = messages;
    packets.nmsgs     = 4;
    if(ioctl(file, I2C_RDWR, &packets) < 0) {
        perror("i2c - apds9301_read_reg_word - ioctl rdwr");
        return -1;
    }

    /* Store register data in calling thread */
    data[0] = ipbuff[0];
    data[1] = ipbuff[1];

    return 0;
}


int apds9301_write_reg_word(int file, unsigned char addr, unsigned char cmd_reg, unsigned char* data)
{
    unsigned char wrbuff[3] = { cmd_reg, data[0], data[1] };
    struct i2c_rdwr_ioctl_data packets;
    struct i2c_msg message;

    /* Set transaction segment message - write command and data to APDS9301 */
    message.addr    = addr;                 // slave address
    message.flags   = 0x00;                 // write bit
    message.len     = sizeof(wrbuff);       // size of write buffer
    message.buf     = wrbuff;               // command + address of register of interest and data to be written

    /* Perform 3 writes transaction */
    packets.msgs    = &message;
    packets.nmsgs   = 1;
    if(ioctl(file, I2C_RDWR, &packets) < 0) {
        perror("i2c - apds9301_write_reg_word - ioctl rdwr");
        return -1;
    }

    return 0;
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