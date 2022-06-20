#include <stdio.h>
#include <iostream>
#include "../bmi160.h"
#include "../bmi160_interface.h"
#include <thread>
#include <chrono>
#include <atomic>
#include <sys/time.h>
#include <fstream>

using namespace std;
using namespace chrono_literals;

BMI160 bmi = BMI160();

/* An example to read the Gyro data in header mode along with sensor time (if available)
 * Configure the gyro sensor as prerequisite and follow the below example to read and
 * obtain the gyro data from FIFO */
	
/* Declare instances of the sensor data structure to store the parsed FIFO data */
struct bmi160_sensor_data gyro_data[1024]; // 300 bytes / ~7bytes per frame ~ 42 data frames
uint8_t gyro_frames_req; 
uint8_t gyro_index;
atomic_int count{0};

// file pointer
fstream csvFile;

int8_t fifo_gyro_header_time_data(struct bmi160_dev *dev)
{

	int8_t rslt = 0;

	/* Declare memory to store the raw FIFO buffer information */
	uint8_t fifo_buff[1024];
	
	/* Modify the FIFO buffer instance and link to the device instance */
	struct bmi160_fifo_frame fifo_frame;
	fifo_frame.data = fifo_buff;
	fifo_frame.length = 1024;
	dev->fifo = &fifo_frame;
	uint16_t index = 0;
	
	
	/* Configure the sensor's FIFO settings */
	rslt = bmi160_set_fifo_config(BMI160_FIFO_GYRO | BMI160_FIFO_HEADER, // | BMI160_FIFO_TIME
					BMI160_ENABLE, dev);
					
	if (rslt == BMI160_OK) {
		/* At ODR of 100 Hz ,1 frame gets updated in 1/100 = 0.01s
		i.e. for 42 frames we need 42 * 0.01 = 0.42s = 420ms delay */
		dev->delay_ms(300); 
	
		/* Read data from the sensor's FIFO and store it the FIFO buffer,"fifo_buff" */
		// printf("\n USER REQUESTED FIFO LENGTH : %d\n",dev->fifo->length);

		while(1) {
			fifo_frame.length = 1024;
			rslt = bmi160_get_fifo_data(dev);

			if (rslt == BMI160_OK) {
				// printf("\n AVAILABLE FIFO LENGTH : %d\n",dev->fifo->length);
				/* Print the raw FIFO data */
				for (index = 0; index < dev->fifo->length; index++) {
						// printf("\n FIFO DATA INDEX[%d] = %d", index,dev->fifo->data[index]);
				}
				/* Parse the FIFO data to extract gyro data from the FIFO buffer */
					// printf("\n REQUESTED GYRO DATA FRAMES : %d\n ",gyro_frames_req);
				rslt = bmi160_extract_gyro(gyro_data, &gyro_frames_req, dev);
				// rslt = bmi160_extract_accel(gyro_data, &gyro_frames_req, dev);
				if (rslt == BMI160_OK) {
						// printf("\n AVAILABLE GYRO DATA FRAMES : %d\n ",gyro_frames_req);
						// printf("\n-------------------------\n");
					/* Print the parsed gyro data from the FIFO buffer */
					// printf("%u %u\n",&gyro_index,gyro_frames_req);
					for (gyro_index = 0; gyro_index < gyro_frames_req; gyro_index++) {
							csvFile << gyro_data[gyro_index].x1 << ", "
								<< gyro_data[gyro_index].y1 << ", "
								<< gyro_data[gyro_index].z1 << ", "
								<< gyro_data[gyro_index].x2 << ", "
								<< gyro_data[gyro_index].y2 << ", "
								<< gyro_data[gyro_index].z2
								<< "\n";
							printf("\nFIFO ACCEL & GYRO  FRAME[%d]",gyro_index);
							printf("\nGYRO X-DATA : %d \t Y-DATA : %d \t Z-DATA : %d"
								,gyro_data[gyro_index].x1 ,gyro_data[gyro_index].y1
								,gyro_data[gyro_index].z1);
							printf("\nACCEL X-DATA : %d \t Y-DATA : %d \t Z-DATA : %d\n"
								,gyro_data[gyro_index].x2 ,gyro_data[gyro_index].y2
								,gyro_data[gyro_index].z2);
							// printf("------------------------------------(%d)\n",tmp);
							count++;
					}
						// printf("\n-------------------------\n");
					/* Print the special FIFO frame data like sensortime */
					// printf("\n SENSOR TIME DATA : %d \n",dev->fifo->sensor_time);
					// printf("SKIPPED FRAME COUNT : %d\n",dev->fifo->skipped_frame_count);
				} else {
						// printf("\n Gyro data extraction failed\n");
				}
			} else {
					// printf("\n Reading FIFO data failed\n");
				}
		}
	} else {
		// printf("\n Setting FIFO configuration failed\n");
	}

	return rslt;
}


int main(){
	// open csv
	remove("imuData.csv");
    csvFile.open("imuData.csv", ios::out | ios::trunc);

	csvFile << "anglvel_x" << ", "
			<< "anglvel_y" << ", "
			<< "anglvel_z" << ", "
			<< "accel_x" << ", "
			<< "accel_y" << ", "
			<< "accel_z"
			<< "\n";

    /////// Sensor interface over I2C ////////
    struct bmi160_dev sensor;
    int8_t rslt = BMI160_OK;
	bool running = true;

    sensor.id = BMI160_I2C_ADDR;
    sensor.interface = BMI160_I2C_INTF;
    sensor.read = &(i2c_read);
    sensor.write = &(i2c_write);
    sensor.delay_ms = &(delay_ms);
    // sensor = bmi.initialize(rslt);
    bmi160_dev* p_sensor = &sensor;

    // rslt = bmi160_init(&sensor);

    /* After the above function call, accel and gyro parameters in the device structure 
    are set with default values, found in the datasheet of the sensor */

    // bmi.set_sensor_settings(p_sensor, 0, rslt);
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
	sensor.gyro_cfg.odr = BMI160_GYRO_ODR_1600HZ;
    sensor.gyro_cfg.range = BMI160_GYRO_RANGE_2000_DPS;
    sensor.gyro_cfg.bw = BMI160_GYRO_BW_NORMAL_MODE;

    /* Select the power mode of Gyroscope sensor */
    sensor.gyro_cfg.power = BMI160_GYRO_NORMAL_MODE; 

    /* Set the sensor configuration */
    rslt = bmi160_set_sens_conf(&sensor);
    struct bmi160_sensor_data gyro;



    ////// Reading sensor data /////////
    ////// Example for reading sensor data ////////

    // int8_t rslt = BMI160_OK;

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
    // while(1){
        // bmi.read_sensor_data(p_sensor, rslt, accel, gyro);
        // printf("accel : x -> %d y -> %d z -> %d time -> %u \n",accel.x,accel.y,accel.z,accel.sensortime);
        // printf("gyro : x -> %d y -> %d z -> %d time -> %u \n",gyro.x,gyro.y,gyro.z,gyro.sensortime);
        // printf("\n");
    // }

	struct timeval stop, start;

	std::thread count_samples_thread{
		[&]() {
			int last_count{0};
			while(running) {
				int current_count = count.load();
				int n_samples = current_count - last_count;
				printf("%d\n", n_samples);
				last_count = current_count;
				this_thread::sleep_for(1000ms);
			}
		}
	};
	std::thread read_fifo_thread{
		[&] () {	
			fifo_gyro_header_time_data(p_sensor);
		}
		
	};

	

	// count_samples_thread.join();
	read_fifo_thread.join();
	running = false;
	// printf("\n-------------------------------------\n");
	// for (int i = 0 ; i< 42 ;i++){
	// 	printf("gyro : x -> %d y -> %d z -> %d time -> %u \n",gyro_data[i].x,gyro_data[i].y,gyro_data[i].z,gyro_data[i].sensortime);
	// }
	// printf("\n-------------------------------------\n");
    
	return 0;
}

