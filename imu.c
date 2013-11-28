/*
 * imu.c
 *
 *  Created on: 22.5.2013
 *      Author: Vincek
 */


#include "includes.h"
#include "imu.h"
#include "I2C.h"
#include <math.h>

void read_sensors(){


}

int read_acc(uint8_t * buffer){
	if(read_from_I2C(ADXL345_ADDR, ADXL345_X_ADDR,buffer,6)==6)return 0; else return 1;
}

int read_gyro(uint8_t * buffer){
	if(read_from_I2C(ITG3205_ADDR, ITG3205_X_ADDR, buffer, 6)==6)return 0; else return 1;
}

int read_mag(uint8_t * buffer){
	if(read_from_I2C(HMC_ADDR, HMC_X_ADDR, buffer, 6)==6)return 0; else return 1;
}

void acc_get_g_raw(uint8_t * buffer, short * acc_offset, float * acc_g){
	short acc_raw[3];
	unsigned char i;
	acc_raw[0] = ((short)buffer[0] | ((short)buffer[1] << 8))*-1;
	acc_raw[1] = ((short)buffer[2] | ((short)buffer[3] << 8))*-1;
	acc_raw[2] = (short)buffer[4] | ((short)buffer[5] << 8);
	acc_raw[0] += acc_offset[0];
	acc_raw[1] += acc_offset[1];
	acc_raw[2] += acc_offset[2];

	for ( i = 0; i<3; i++)
	{
		acc_g[i] = (float)acc_raw[i] / ACC_SENS;          // G-force in each direction
	}


}

void acc_get_g_filtered(float * acc_g_raw, float * last_acc_g_raw, float beta, float * acc_g_filtered){
	unsigned char i;
	for(i=0;i<3;i++){
		acc_g_filtered[i] = acc_g_raw[i]*beta + (1-beta)*last_acc_g_raw[i];
		last_acc_g_raw[i]=acc_g_filtered[i];
	}

}

void acc_get_angles(float * acc_g, float * angles){
	float g;
	unsigned char i;
	#if (ASSUME_1G_ACC == 0)
		g = sqrt((acc_g[0] * acc_g[0]) + (acc_g[1] * acc_g[1]) + (acc_g[2] * acc_g[2]));
	#else // Otherwise, just assume total G = 1.
		g = 1;
	#endif

	// Calculate final angles:
	// WARNING - THIS LIMITS THE UPDATE OF ACC ANGLE TO SMALL G's
	if (g < 1.3 && g > 0.7)
	{
		for (i = 0; i<3; i++)
		{
			//angles[i] = acc_g[i];
			angles[i] = acos(acc_g[i]/g)*57.3;
			//angles[i] = acos(acc_g[i] / g) * 57.3;
		}
	}
}

void gyro_get_rates(uint8_t * buffer, short * gyro_offset, float * rates){
	short gyro_raw[3];
	uint8_t i;
	gyro_raw[0] = (short)buffer[1] | ((short)buffer[0] << 8);
	gyro_raw[1] = ( (short)buffer[3] | ((short)buffer[2] << 8) ) * -1;
	gyro_raw[2] = ( (short)buffer[5] | ((short)buffer[4] << 8) )*-1;

	for (i=0; i<3; i++)
	{
		rates[i]=(float)(gyro_raw[i]-gyro_offset[i])/SCALING_FACTOR;
	}

}


//see http://www.chrobotics.com/library/understanding-euler-angles
// roll = pitch, pitch = roll from acc!!!!
void transform_gyro_rates(float * gyro_rates, float * angles){
	//if calculated from acc
//	float roll_rate = gyro_rates[0] + gyro_rates[2]*cos((angles[1]-90)/ 57.3)*tan((angles[0]-90)/ 57.3)+gyro_rates[1]*sin((angles[1]-90)/ 57.3)*tan((angles[0]-90)/ 57.3);
//	float pitch_rate = gyro_rates[1]*cos((angles[1]-90)/ 57.3)-gyro_rates[2]*sin((angles[1]-90)/ 57.3);
//	float yaw_rate = gyro_rates[1]*sin((angles[1]-90)/ 57.3)/cos((angles[0]-90)/ 57.3)+gyro_rates[2]*cos((angles[1]-90)/ 57.3)/cos((angles[0]-90)/ 57.3);
	//if calculated from total angle
	float roll_rate = gyro_rates[0] + gyro_rates[2]*cos((angles[0])/ 57.3)*tan((angles[1])/ 57.3)+gyro_rates[1]*sin((angles[0])/ 57.3)*tan((angles[1])/ 57.3);
	float pitch_rate = gyro_rates[1]*cos((angles[0])/ 57.3)-gyro_rates[2]*sin((angles[0])/ 57.3);
	float yaw_rate = gyro_rates[1]*sin((angles[0])/ 57.3)/cos((angles[1])/ 57.3)+gyro_rates[2]*cos((angles[0])/ 57.3)/cos((angles[1])/ 57.3);
	gyro_rates[0] = roll_rate;
	gyro_rates[1] = pitch_rate;
	gyro_rates[2] = yaw_rate;
}

void calibrate_gyro(short * gyro_offset){
	int i,j;
	uint8_t buffer[6];
	short gyro_raw[3];
	//collect 200 samples
	for(i=0;i<200;i++){
		if(!read_gyro(buffer)){
			gyro_raw[0] = (short)buffer[1] | ((short)buffer[0] << 8);
			gyro_raw[1] = ( (short)buffer[3] | ((short)buffer[2] << 8) ) * -1;
			gyro_raw[2] = ( (short)buffer[5] | ((short)buffer[4] << 8) ) * -1;
			for(j=0;j<3;j++){
				gyro_offset[j]+=gyro_raw[j];
			}
		}
		SysCtlDelay(SysCtlClockGet()/100);
	}
	//average
	for(j=0;j<3;j++){
		gyro_offset[j]/=200;
	}



}

void calibrate_mag(float * mag_gain)
{
	int i;
	short mag_pos_off[3];
	short mag_neg_off[3];
	short mag_raw[3];
	uint8_t buffer[6];
	write_to_I2C(HMC_ADDR, 0x00, 17);
	SysCtlDelay(SysCtlClockGet()/10);

    // MM: Again with the loops. Not sure what purpose this serves, Dennis.
    for (i = 0; i < 10; i++)
    {
    	read_mag(buffer);
    	SysCtlDelay(SysCtlClockGet()/10);
    }

    for (i =0; i < 3; i++)
	{
	   mag_raw[i] = (short)buffer[(i * 2) + 1] | ((short)buffer[i * 2] << 8);
	}

    mag_pos_off[0] = mag_raw[0];
    mag_pos_off[1] = mag_raw[1];
    mag_pos_off[2] = mag_raw[2];


    write_to_I2C(HMC_ADDR, 0x00, 18);
    SysCtlDelay(SysCtlClockGet()/10);

    for (i = 0; i < 10; i++)
    {
    	read_mag(buffer);
    	SysCtlDelay(SysCtlClockGet()/10);

    }


    for (i =0; i < 3; i++)
	{
	   mag_raw[i] = (short)buffer[(i * 2) + 1] | ((short)buffer[i * 2] << 8);
	}


    mag_neg_off[0] = mag_raw[0];
    mag_neg_off[1] = mag_raw[1];
    mag_neg_off[2] = mag_raw[2];




    mag_gain[0] =-2500/(float)(mag_neg_off[0] - mag_pos_off[0]);
    mag_gain[1] =-2500/ (float)(mag_neg_off[1] - mag_pos_off[1]);
    mag_gain[2] =-2500/ (float)(mag_neg_off[2] - mag_pos_off[2]);


    write_to_I2C(HMC_ADDR, 0x00, 16);

    for (i = 0; i < 10; i++)
    {
    	read_mag(buffer);
    	SysCtlDelay(SysCtlClockGet()/10);
    }

}

void get_angles_mag(uint8_t * buffer, short * mag_offset, float * mag_gain, float * angles, float * mag_angles)
{
	int i;
	float mx,my;
	short mag_raw[3];

	for (i =0; i < 3; i++)
	{
	   mag_raw[i] = (short)buffer[(i * 2) + 1] | ((short)buffer[i * 2] << 8);
	}

    // Invert 2 axis
	mag_raw[1] *= -1;
	mag_raw[2] *= -1;

    // Set gain:
	mag_raw[0] *= mag_gain[0];
	mag_raw[1] *= mag_gain[1];
	mag_raw[2] *= mag_gain[2];

	mag_raw[0] -= mag_offset[0];
	mag_raw[1] -= mag_offset[1];
	mag_raw[2] -= mag_offset[2];

    float test_angle = angles[0]; // tilt = pitch = angles[0]
    mx = mag_raw[0] * cos((test_angle) / 57.3)
        + mag_raw[1] * sin(test_angle / 57.3);

    my = mag_raw[0] * sin((angles[1]) / 57.3) // roll = angles[1]
        * sin((angles[0] - 90) / 57.3)
        + mag_raw[2] * cos((angles[1]) / 57.3)
        - mag_raw[1] * sin((angles[1]) / 57.3)
        * cos((angles[0]) / 57.3);

    // Calculate yaw-angle from magnetometer.
    mag_angles[2] = (atan(mx / my) * 57.3 + 90);

    // Get full 0-360 degrees.
    if (my < 0)
    {
    	mag_angles[2] += 180;
    }

    float temp_angle = - mag_angles[2];

    if (temp_angle > 180)
    {
    	temp_angle -= 360;
    }
    else if (temp_angle < -180)
    {
    	temp_angle += 360;
    }

    mag_angles[2] = temp_angle * -1;
}


