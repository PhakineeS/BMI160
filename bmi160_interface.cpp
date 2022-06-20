#include "bmi160_interface.h"

int file;
int adapter_nr = 2;
char filename[40];

BMI160::BMI160(){   //the main sensor interface class constructor
    sprintf(filename,"/dev/i2c-3");  //i2c interface 1  --> if needed, change with (i2c-0,i2c-2,..)
    file = open(filename, O_RDWR);//, O_NONBLOCK,O_NDELAY); // open the channel
    if (file < 0)
    {
        printf("Failed to open the bus.\n");
        /* ERROR HANDLING; you can check error */
        exit(1);
    }

    /* The I2C address */
    

    // BMI160_I2C_ADDR -> BMI160_I2C_ADDR
    if (ioctl(file, I2C_SLAVE, BMI160_I2C_ADDR) < 0)
    {
        printf("Failed to acquire bus access and/or talk to slave.\n");
        /* ERROR HANDLING; you can check errno to see what went wrong */
        exit(1);
    }
    else{
        printf("BMI160 found at 0x%02X\n", BMI160_I2C_ADDR );
    }
}


int8_t i2c_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len){

    uint8_t buffer_write[2];
    memset(buffer_write,'\0',2);
    int n_writ;
    n_writ = 0;
    // Request data
    buffer_write[0] = reg_addr;

    n_writ = write(file,buffer_write,1);
    if (n_writ != 1)
    {
        /* ERROR HANDLING: i2c transaction failed */
        printf("BMI160 Reading Error (cannot request data): Failed to write\n");
        return -1;
    }
    int n_read;
    // Read data
    n_read = read(file,data,len);
    if (n_read != len)
    {
        /* ERROR HANDLING: i2c transaction failed */
        printf("BMI160 Reading Error (not enough data readed) :Failed to read\n");
        return -1;
    }
    return BMI160_OK;
}

int8_t i2c_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len){
    uint8_t buffer_write[len+1]; // cast needed???!?
    memset(buffer_write,'\0',len+1);
    int n_writ;
    buffer_write[0] = reg_addr;
    for(int i = 0; i<len; i++)
    {
        buffer_write[i+1] = data[i];
    }
    n_writ = write(file,buffer_write, len+1);
    if( n_writ < len+1)
    {
        /* ERROR HANDLING: i2c transaction failed */
        printf("BMI160 Writing Error :Failed to write\n");
        return -1;
    }
    return BMI160_OK;
}

void delay_ms(uint32_t period){
    usleep(period*1000);
}

void delay_ms2(uint32_t period){
    usleep(period*1000);
}

bmi160_dev BMI160::initialize(int8_t &rslt){
    struct bmi160_dev dev;
    
    dev.id = BMI160_I2C_ADDR;
    dev.interface = BMI160_I2C_INTF;
    dev.read = i2c_read;
    dev.write = i2c_write;
    dev.delay_ms = delay_ms;
    
    rslt = bmi160_init(&dev);
    
    return dev;
}

void BMI160::set_sensor_settings(struct bmi160_dev *dev, int mode, int8_t &rslt)
{
    // if(mode == 0){
    //     /* Setting the power mode as normal */
    //     dev->settings.pwr_mode = BMI160_NORMAL_MODE;
    //     rslt = BMI160_set_op_mode(dev);
    // }
    // else{
    //     /* Setting the preset mode as Low power mode 
    //     i.e. data rate = 10Hz XY-rep = 1 Z-rep = 2*/
    //     dev->settings.preset_mode = BMI160_PRESETMODE_LOWPOWER;
    //     rslt = BMI160_set_presetmode(dev);
    // }
    /* Select the Output data rate, range of accelerometer sensor */
    dev->accel_cfg.odr = BMI160_ACCEL_ODR_1600HZ;
    dev->accel_cfg.range = BMI160_ACCEL_RANGE_16G;
    dev->accel_cfg.bw = BMI160_ACCEL_BW_NORMAL_AVG4;

    /* Select the power mode of accelerometer sensor */
    dev->accel_cfg.power = BMI160_ACCEL_NORMAL_MODE;
    dev->delay_ms(100);
    /* Select the Output data rate, range of Gyroscope sensor */
    dev->gyro_cfg.odr = BMI160_GYRO_ODR_800HZ;
    dev->gyro_cfg.range = BMI160_GYRO_RANGE_2000_DPS;
    dev->gyro_cfg.bw = BMI160_GYRO_BW_NORMAL_MODE;

    /* Select the power mode of Gyroscope sensor */
    dev->gyro_cfg.power = BMI160_GYRO_NORMAL_MODE; 
    dev->delay_ms(100);
    /* Set the sensor configuration */
    rslt = bmi160_set_sens_conf(dev);

}

void BMI160::read_sensor_data(struct bmi160_dev *dev, int8_t &rslt, struct bmi160_sensor_data &accel, struct bmi160_sensor_data &gyro)
{

    /* Mag data for X,Y,Z axis are stored inside the
    BMI160_dev structure in int16_t format */
    // rslt = BMI160_read_mag_data(dev);

    /* Print the Mag data */
    // printf("\n Magnetometer data \n");
    // printf("MAG X : %d \t MAG Y : %d \t MAG Z : %d \n",dev.data.x, dev->data.y, dev->data.z);
    bmi160_get_sensor_data((BMI160_ACCEL_SEL | BMI160_GYRO_SEL | BMI160_TIME_SEL), &accel, &gyro, dev);
}

