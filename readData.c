#include <stdio.h>
// #include <iostream>
#include "../bmi160.h"
#include "../bmi160_interface.h"

// using namespace std;

// BMI160 bmi = BMI160();

int main(){
    /////// Sensor interface over I2C ////////

    struct bmi160_dev sensor;
    int8_t rslt = BMI160_OK;

    sensor.id = BMI160_I2C_ADDR;
    sensor.interface = BMI160_I2C_INTF;
    sensor.read = &(i2c_read);
    sensor.write = &(i2c_write);
    sensor.delay_ms = &(delay_ms);

    rslt = bmi160_init(&sensor);
    /* After the above function call, accel and gyro parameters in the device structure 
    are set with default values, found in the datasheet of the sensor */



    ///////// Configuring accel and gyro sensor /////////
    ///////// Example for configuring accel and gyro sensors in normal mode  /////////

    // int8_t rslt = BMI160_OK;

    /* Select the Output data rate, range of accelerometer sensor */
    sensor.accel_cfg.odr = BMI160_ACCEL_ODR_1600HZ;
    sensor.accel_cfg.range = BMI160_ACCEL_RANGE_2G;
    sensor.accel_cfg.bw = BMI160_ACCEL_BW_NORMAL_AVG4;

    /* Select the power mode of accelerometer sensor */
    sensor.accel_cfg.power = BMI160_ACCEL_NORMAL_MODE;

    /* Select the Output data rate, range of Gyroscope sensor */
    sensor.gyro_cfg.odr = BMI160_GYRO_ODR_3200HZ;
    sensor.gyro_cfg.range = BMI160_GYRO_RANGE_2000_DPS;
    sensor.gyro_cfg.bw = BMI160_GYRO_BW_NORMAL_MODE;

    /* Select the power mode of Gyroscope sensor */
    sensor.gyro_cfg.power = BMI160_GYRO_NORMAL_MODE; 

    /* Set the sensor configuration */
    rslt = bmi160_set_sens_conf(&sensor);


    ////// Reading sensor data /////////
    ////// Example for reading sensor data ////////

    // int8_t rslt = BMI160_OK;
    struct bmi160_sensor_data accel;
    struct bmi160_sensor_data gyro;

    /* To read only Accel data */
    // rslt = bmi160_get_sensor_data(BMI160_ACCEL_SEL, &accel, NULL, &sensor);

    /* To read only Gyro data */
    // rslt = bmi160_get_sensor_data(BMI160_GYRO_SEL, NULL, &gyro, &sensor);

    /* To read both Accel and Gyro data */
    // bmi160_get_sensor_data((BMI160_ACCEL_SEL | BMI160_GYRO_SEL), &accel, &gyro, &sensor);

    /* To read Accel data along with time */
    // rslt = bmi160_get_sensor_data((BMI160_ACCEL_SEL | BMI160_TIME_SEL) , &accel, NULL, &sensor);

    /* To read Gyro data along with time */
    // rslt = bmi160_get_sensor_data((BMI160_GYRO_SEL | BMI160_TIME_SEL), NULL, &gyro, &sensor);

    /* To read both Accel and Gyro data along with time*/
    bmi160_get_sensor_data((BMI160_ACCEL_SEL | BMI160_GYRO_SEL | BMI160_TIME_SEL), &accel, &gyro, &sensor);
    printf("accel : x -> %d y -> %d z -> %d time -> %u \n",accel.x,accel.y,accel.z,accel.sensortime);
    printf("gyro : x -> %d y -> %d z -> %d time -> %u \n",gyro.x,gyro.y,gyro.z,gyro.sensortime);
 
    return 0;
}
