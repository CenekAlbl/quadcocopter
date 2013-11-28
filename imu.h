/*
 * imu.h
 *
 *  Created on: 22.5.2013
 *      Author: Vincek
 */

#ifndef IMU_H_
#define IMU_H_




// Gyro
//
#define ITG3205_ADDR 0x68    // The address of ITG3205
#define ITG3205_X_ADDR 0x1D  // Start address for x-axis
#define SCALING_FACTOR 14.375     // Scaling factor - used when converting to angle

// Accelerometer
//
#define ADXL345_ADDR (0x53)  // The adress of ADXL345
#define ADXL345_X_ADDR (0x32)// Start address for x-axis       // Sensitivity. 13 bit adc, +/- 16 g. Calculated as: (2^13)/(16*2)
#define ACC_SENS 2048
#define ASSUME_1G_ACC 0


// Magnetometer
//
#define HMC_ADDR 0x1E        // The address of HMC5883
#define HMC_X_ADDR (0x03)    // Start address for x-axis.

#define SAMPLERATE 128       // Samplerate of sensors (in hz, samples per second)

#define HMC5883L_RA_CONFIG_A	    0x00
#define HMC5883L_RA_CONFIG_B	    0x01
#define HMC5883L_RA_MODE		    0x02

#define HMC5883L_RA_STATUS          0x09
#define HMC5883L_RA_ID_A            0x0A
#define HMC5883L_RA_ID_B            0x0B
#define HMC5883L_RA_ID_C            0x0C

#define HMC5883L_AVERAGING_1        0x00<<5
#define HMC5883L_AVERAGING_2        0x01<<5
#define HMC5883L_AVERAGING_4        0x02<<5
#define HMC5883L_AVERAGING_8        0x03<<5

#define HMC5883L_RATE_0P75          0x00<<2
#define HMC5883L_RATE_1P5           0x01<<2
#define HMC5883L_RATE_3             0x02<<2
#define HMC5883L_RATE_7P5           0x03<<2
#define HMC5883L_RATE_15            0x04<<2
#define HMC5883L_RATE_30            0x05<<2
#define HMC5883L_RATE_75            0x06<<2

#define HMC5883L_BIAS_NORMAL        0x00
#define HMC5883L_BIAS_POSITIVE      0x01
#define HMC5883L_BIAS_NEGATIVE      0x02

#define HMC5883L_GAIN_1370          0x00<<5
#define HMC5883L_GAIN_1090          0x01<<5
#define HMC5883L_GAIN_820           0x02<<5
#define HMC5883L_GAIN_660           0x03<<5
#define HMC5883L_GAIN_440           0x04<<5
#define HMC5883L_GAIN_390           0x05<<5
#define HMC5883L_GAIN_330           0x06<<5
#define HMC5883L_GAIN_220           0x07<<5

#define HMC5883L_MODE_CONTINUOUS    0x00
#define HMC5883L_MODE_SINGLE        0x01
#define HMC5883L_MODE_IDLE          0x02

#define HMC5883L_STATUS_LOCK_BIT    1
#define HMC5883L_STATUS_READY_BIT   0

//#define HMC5883L_MAX_5				575
//#define HMC5883L_MIN_5				243
//#define HMC5883L_MAX_6				487
//#define HMC5883L_MIN_6				206
//#define HMC5883L_MAX_7				339
//#define HMC5883L_MIN_7				143

#define MAG0MAX 625
#define MAG1MAX 625
#define MAG2MAX 625
#define MAG0MIN -625
#define MAG1MIN -625
#define MAG2MIN -625

void read_sensors();
int read_acc(uint8_t * buffer);
int read_gyro(uint8_t * buffer);
int read_mag(uint8_t * buffer);
void acc_get_g_raw(uint8_t * buffer, short * acc_offset, float * acc_g);
void acc_get_g_filtered(float * acc_g_raw, float * last_acc_g_raw, float beta, float * acc_g_filtered);
void acc_get_angles(float * acc_g, float * angles);
void gyro_get_rates(uint8_t * buffer, short * gyro_offset, float * rates);
void transform_gyro_rates(float * gyro_rates, float * angles);
void calibrate_gyro(short * gyro_offset);

void calibrate_mag(float * mag_gain);
void get_angles_mag(uint8_t * buffer, short * mag_offset, float * mag_gain, float * angles, float * mag_angles);
//void process_gyro();
//void process_mag();


#endif /* IMU_H_ */
