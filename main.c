/*
 * main.c
 */
#include "includes.h"
#include "pwm.h"
#include "uart.h"
#include "I2C.h"
#include "imu.h"

#ifdef DEBUG
void
__error__(char *pcFilename, unsigned long ulLine)
{
}
#endif

#define TESTING
#define REALTIME_READOUT

//#define PID_ANGLE
//#define PID_RATE
#define PID_RATE_AND_ANGLE

typedef struct{
	volatile long channel1_start;
	volatile long channel2_start;
	volatile long channel3_start;
	volatile long channel4_start;
	volatile long channel1_end;
	volatile long channel2_end;
	volatile long channel3_end;
	volatile long channel4_end;
	long channel1_pulse_width;
	long channel2_pulse_width;
	long channel3_pulse_width;
	long channel4_pulse_width;
	long channel1_measuring;
	long channel2_measuring;
	long channel3_measuring;
	long channel4_measuring;
	float motor1_speed;
	float motor2_speed;
	float motor3_speed;
	float motor4_speed;
	float speed_adjust;
	float roll_adjust;
	float pitch_adjust;
	float yaw_adjust;
#if defined(PID_RATE) || defined(PID_RATE_AND_ANGLE)
	float roll_rate_adjust;
	float pitch_rate_adjust;
	float yaw_rate_adjust;
#endif

	uint8_t ready;
	uint8_t engines_on;

}Kokopter;




#if defined(PID_ANGLE) || defined(PID_RATE_AND_ANGLE)
	float I_term_pitch=0,I_term_roll=0;
	float error_pitch;
	float error_roll;
	float d_input_pitch =0;
	float d_input_roll =0;
	float last_input_pitch=0;
	float last_input_roll=0;
	float err_sum_pitch=0;
	float err_sum_roll=0;
	float set_point_roll = 0;
	float set_point_pitch = 0;
#endif

#if defined(PID_RATE) || defined(PID_RATE_AND_ANGLE)
	float I_term_pitch_rate=0,I_term_roll_rate=0;
	float error_pitch_rate;
	float error_roll_rate;
	float d_input_pitch_rate=0;
	float d_input_roll_rate=0;
	float last_input_pitch_rate=0;
	float last_input_roll_rate=0;
#endif

	//inner loop constants
#if defined(PID_RATE) || defined(PID_RATE_AND_ANGLE)
	float kp1=0.05, ki1=0, kd1=0;
#endif

#if defined(PID_ANGLE) || defined(PID_RATE_AND_ANGLE)
	//outer loop constants
	float kp2=5, ki2=0, kd2=0;
#endif

float PID_output_max = 7;
float PID_output_rate_max = 100;



uint8_t radio_thrust_prev = 0;

static long motor_set_timer,pwm_frequency;

volatile unsigned char command[5];
volatile int buff_id=0;

uint8_t button1_pressed=0;

long button1_time1 = 0;
long total  =0;

//ACC DATA
float acc_angle[3];
float acc_angle_filtered[3];
float acc_g[3];
float acc_g_filtered[3];
float acc_g_last[3] = {0,0,0};
short acc_offset[3] = {0, 0, 0};
float acc_angles_offset[3] = {4.5,0,0};
//GYRO DATA
float gyro_rates_body[3];
float gyro_rates_euler[3];
float gyro_rates_filtered[3];
float gyro_angles[3] = {0,0,0};

short gyro_offset[3] = {0, 0, 0};
short mag_offset[3] = {(MAG0MAX + MAG0MIN) / 2, (MAG1MAX + MAG1MIN) / 2, (MAG2MAX + MAG2MIN) / 2};
float mag_gain[3];
float mag_angles[3];



float sample_rate = 100; //sample rate in hz


float gyro_weight = 0.98;

float angles[3] = {0,0,0};
float angles_LP[3] = {0,0,0};
float last_angles[3] = {0,0,0};

float tilt_roll_beta = 0.75;

uint8_t buffer[6];

static Kokopter kokopter;
static uint8_t done;

void set_PID_constants(float P, float I, float D, int loop){
	double sample_time_in_sec = 1.0/((double)sample_rate);

	switch(loop){
	case 1:
		kp1 = P;
		ki1 = I*sample_time_in_sec;
		kd1 = D/sample_time_in_sec;

	break;
#if defined(PID_ANGLE) || defined (PID_RATE_AND_ANGLE)
	case 2:
		kp2 = P;
		ki2 = I*sample_time_in_sec;
		kd2 = D/sample_time_in_sec;
	break;
#endif

	}



}

void set_motor_speed(uint8_t motor,  uint8_t percent){
	float duty_cycle = 0.8+(100-percent)*0.001;
	set_pwm_duty_cycle(motor,pwm_frequency,duty_cycle);
}

process_command(){
	double sample_time_in_sec = 1.0/((double)sample_rate);
	//handle different types of commands
	switch(command[0]){
	// change outer loop P
	case 0:
#if defined( PID_RATE) || defined(PID_RATE_AND_ANGLE)
		kp1=*(float *)(command+1);
#endif
#if defined( PID_ANGLE) || defined(PID_RATE_AND_ANGLE)
		kp2=*(float *)(command+1);
#endif
	break;
	// change outer loop I
	case 1:
#if defined( PID_RATE) || defined(PID_RATE_AND_ANGLE)
		ki1=(*(float *)(command+1))*sample_time_in_sec;
#endif
#if defined( PID_ANGLE) || defined(PID_RATE_AND_ANGLE)
		ki2=(*(float *)(command+1))*sample_time_in_sec;
#endif
	break;
	// change outer loop D
	case 2:
#if defined( PID_RATE) || defined(PID_RATE_AND_ANGLE)
		kd1=(*(float *)(command+1))/sample_time_in_sec;
#endif
#if defined( PID_ANGLE) || defined(PID_RATE_AND_ANGLE)
		kd2=(*(float *)(command+1))/sample_time_in_sec;
#endif
	break;
	case 100:
		{
#ifdef PID_RATE
		float ki_send=ki1/sample_time_in_sec;
		float kd_send=kd1*sample_time_in_sec;
		UARTSend((unsigned char *)&kp1, 4);
		SysCtlDelay(10000);
		UARTSend((unsigned char *)&ki_send, 4);
		SysCtlDelay(10000);
		UARTSend((unsigned char *)&kd_send, 4);
		SysCtlDelay(10000);
#elif defined( PID_ANGLE) || defined(PID_RATE_AND_ANGLE)
		float ki_send=ki2/sample_time_in_sec;
		float kd_send=kd2*sample_time_in_sec;
		UARTSend((unsigned char *)&kp2, 4);
		SysCtlDelay(10000);
		UARTSend((unsigned char *)&ki_send, 4);
		SysCtlDelay(10000);
		UARTSend((unsigned char *)&kd_send, 4);
		SysCtlDelay(10000);
#endif

		break;
		}
	}
	buff_id=0;

}

void radio_interrupt_handler(void){
	long status,temp,time;
	time = TimerValueGet(TIMER2_BASE,TIMER_A);
	status = GPIOPinRead(GPIO_PORTE_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4);
	//status2 = GPIOPinIntStatus(GPIO_PORTF_BASE,0);
	GPIOPinIntClear(GPIO_PORTE_BASE, 0xFF);
	//status3 = GPIOPinIntStatus(GPIO_PORTF_BASE,0);
	if(status&GPIO_PIN_2){
		if(kokopter.channel2_measuring==0){
			kokopter.channel2_measuring=1;
			kokopter.channel2_start = time;
		}

	}else{
		if(kokopter.channel2_measuring==1){
			kokopter.channel2_measuring=0;
			kokopter.channel2_end = time;
			if(kokopter.channel2_start<kokopter.channel2_end)kokopter.channel2_start+=4294967296;
			temp = kokopter.channel2_start-kokopter.channel2_end;
			if(temp<160000&&temp>80000){
				kokopter.channel2_pulse_width=temp;
			}
		}
	}

	if(status&GPIO_PIN_1){
			if(kokopter.channel1_measuring==0){
				kokopter.channel1_measuring=1;
				kokopter.channel1_start = time;
			}

		}else{
			if(kokopter.channel1_measuring==1){
				kokopter.channel1_measuring=0;
				kokopter.channel1_end = time;
				if(kokopter.channel1_start<kokopter.channel1_end)kokopter.channel1_start+=4294967296;
				temp = kokopter.channel1_start-kokopter.channel1_end;
				if(temp<160000&&temp>80000){
					kokopter.channel1_pulse_width=temp;
				}
			}
		}

	if(status&GPIO_PIN_3){
			if(kokopter.channel3_measuring==0){
				kokopter.channel3_measuring=1;
				kokopter.channel3_start = time;
			}

		}else{
			if(kokopter.channel3_measuring==1){
				kokopter.channel3_measuring=0;
				kokopter.channel3_end = time;
				if(kokopter.channel3_start<kokopter.channel3_end)kokopter.channel3_start+=4294967296;
				temp = kokopter.channel3_start-kokopter.channel3_end;
				if(temp<160000&&temp>80000){
					kokopter.channel3_pulse_width=temp;
				}
			}
		}

	if(status&GPIO_PIN_4){
			if(kokopter.channel4_measuring==0){
				kokopter.channel4_measuring=1;
				kokopter.channel4_start = time;
			}

		}else{
			if(kokopter.channel4_measuring==1){
				kokopter.channel4_measuring=0;
				kokopter.channel4_end = time;
				if(kokopter.channel4_start<kokopter.channel4_end)kokopter.channel4_start+=4294967296;
				temp = kokopter.channel4_start-kokopter.channel4_end;
				if(temp<160000&&temp>80000){
					kokopter.channel4_pulse_width=temp;
				}
			}
		}




};

void Systick_handler(void)
{
	float range;
	int mid,min,sensitivity;
	long button1_status;


	button1_status = GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_4);
	if((button1_status&GPIO_PIN_4)==0){
		button1_time1++;
	}else{
		button1_time1=0;
		button1_pressed=0;
	}

	if(button1_time1>5&&!button1_pressed){
		if(kokopter.ready){
			kokopter.ready=0;
		}else{
			kokopter.ready=1;
		}
		button1_pressed=1;
	}

	//THRUST
	range = 64000;
	min = 152000;
	kokopter.speed_adjust = (min-kokopter.channel2_pulse_width)/range*100;

#if defined(PID_ANGLE) || defined(PID_RATE_AND_ANGLE)
	//YAW
	range = 40000;
	mid = 120000;
	sensitivity = 40;
	kokopter.yaw_adjust = (kokopter.channel3_pulse_width-mid)/range*sensitivity;
#endif

#if defined(PID_RATE)
	//YAW RATE
	range = 40000;
	mid = 120000;
	sensitivity = 40;
	kokopter.yaw_rate_adjust = (kokopter.channel3_pulse_width-mid)/range*sensitivity;
#endif
#if defined(PID_ANGLE) || defined(PID_RATE_AND_ANGLE)
	//PITCH
	range = 48000;
	mid = 120000;
	sensitivity = 40;
	kokopter.pitch_adjust = (kokopter.channel1_pulse_width-mid)/range*sensitivity;
#endif
#if defined(PID_RATE)
	//PITCH RATE
	range = 48000;
	mid = 120000;
	sensitivity = 40;
	kokopter.pitch_rate_adjust = (kokopter.channel1_pulse_width-mid)/range*sensitivity;
#endif
#if defined(PID_ANGLE) || defined(PID_RATE_AND_ANGLE)
	//ROLL
	range = 32000;
	mid = 120000;
	sensitivity = 40;
	kokopter.roll_adjust = (kokopter.channel4_pulse_width-mid)/range*sensitivity;
#endif

#if defined(PID_RATE)
	//ROLL RATE
	range = 32000;
	mid = 120000;
	sensitivity = 40;
	kokopter.roll_rate_adjust = (kokopter.channel4_pulse_width-mid)/range*sensitivity;
#endif

#ifdef TESTING
	#if defined(PID_RATE)
		kokopter.pitch_rate_adjust = 0;
		kokopter.roll_rate_adjust = 0;
		kokopter.yaw_rate_adjust = 0;
	#endif
	#if defined( PID_ANGLE) || defined(PID_RATE_AND_ANGLE)
		//kokopter.pitch_adjust = 0;
		//kokopter.roll_adjust = 0;
		//kokopter.yaw_adjust = 0;
	#endif
#endif

	if(buff_id>=5)process_command();

	read_acc(buffer);
	acc_get_g_raw(buffer,acc_offset,acc_g);
	acc_get_g_filtered(acc_g,acc_g_last,0.1,acc_g_filtered);
	acc_get_angles(acc_g_filtered, acc_angle_filtered);
#ifdef TESTING
	acc_get_angles(acc_g,acc_angle);
	acc_angle[0]-=acc_angles_offset[0];
	acc_angle[1]-=acc_angles_offset[1];
#endif
	acc_angle_filtered[0]-=acc_angles_offset[0];
	acc_angle_filtered[1]-=acc_angles_offset[1];

	read_gyro(buffer);
	gyro_get_rates(buffer, gyro_offset, gyro_rates_body);

#if defined( PID_ANGLE) || defined(PID_RATE_AND_ANGLE)
	transform_angular_rates(gyro_rates_body,gyro_rates_euler, angles_LP);

	int i;
	for(i=0;i<3;i++){
			gyro_rates_filtered[i] = gyro_rates_euler[i]*0.3 + (1-0.3)*gyro_rates_filtered[i];
	}

	read_mag(buffer);
	get_angles_mag(buffer, mag_offset, mag_gain, angles, mag_angles);

	angles[0] = (angles_LP[0] + gyro_rates_euler[0]/100)*gyro_weight + (acc_angle_filtered[1])*(1-gyro_weight);
	angles[1] = (angles_LP[1] + gyro_rates_euler[1]/100)*gyro_weight + (acc_angle_filtered[0])*(1-gyro_weight);
	angles[2] = (angles_LP[2] + gyro_rates_euler[2]/100)*gyro_weight;// + mag_angles[2]*(1-gyro_weight);

	//Low pass filter this shit
	for(i=0;i<3;i++){
		angles_LP[i] = angles[i]*tilt_roll_beta + (1-tilt_roll_beta)*last_angles[i];
		last_angles[i]=angles[i];
	}
#endif

#ifdef REALTIME_READOUT
	UARTSend((unsigned char *)&acc_angle[0], 4);
	SysCtlDelay(1000);
	UARTSend((unsigned char *)&gyro_rates_body[0], 4);
	SysCtlDelay(1000);
#endif

	if(kokopter.ready){
		GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_3,8);
		//
		if(kokopter.speed_adjust>20){
			kokopter.engines_on=1;
			kokopter.speed_adjust-=20;
		}else{
			kokopter.engines_on=0;
			kokopter.speed_adjust-=20;
		}

#ifdef PID_ANGLE
		error_roll = kokopter.roll_adjust - angles_LP[0];
	    error_pitch = kokopter.pitch_adjust - angles_LP[1];
	    d_input_roll = (angles_LP[0] - last_input_roll);
	    d_input_pitch = (angles_LP[1] - last_input_pitch);
#endif

#ifdef PID_RATE
	    error_roll_rate = kokopter.roll_rate_adjust - gyro_rates_body[0];
	    error_pitch_rate = kokopter.pitch_rate_adjust - gyro_rates_body[1];
#endif

#ifdef PID_RATE_AND_ANGLE
	    error_roll = kokopter.roll_adjust - angles_LP[0];
		error_pitch = kokopter.pitch_adjust - angles_LP[1];
		d_input_roll = (angles_LP[0] - last_input_roll);
		d_input_pitch = (angles_LP[1] - last_input_pitch);

#endif

		if(kokopter.engines_on){

			//PID
		   /*Compute all the working error variables*/


#ifdef PID_ANGLE

		   I_term_roll += ki2*error_roll;
		   I_term_pitch += ki2*error_pitch;

		   if(I_term_pitch>PID_output_max){
			   I_term_pitch = PID_output_max;
		   }else if(I_term_pitch<-PID_output_max){
			   I_term_pitch = -PID_output_max;
		   }

		   if(I_term_roll>PID_output_max){
			   I_term_roll = PID_output_max;
		   }else if(I_term_roll<-PID_output_max){
			   I_term_roll = -PID_output_max;
		   }


		   /*Compute PID Output*/
		   kokopter.roll_adjust = kp2 * error_roll + I_term_roll + kd2 * d_input_roll;
		   kokopter.pitch_adjust = kp2 * error_pitch + I_term_pitch + kd2 * d_input_pitch;
		   //trim to max values
		   if(kokopter.pitch_adjust>PID_output_max)kokopter.pitch_adjust=PID_output_max;
		   else if(kokopter.pitch_adjust<-PID_output_max)kokopter.pitch_adjust=-PID_output_max;
		   if(kokopter.roll_adjust>PID_output_max)kokopter.roll_adjust=PID_output_max;
		   if(kokopter.roll_adjust<-PID_output_max)kokopter.roll_adjust=-PID_output_max;
#endif
#ifdef PID_RATE
		   I_term_roll_rate+=ki1*error_roll_rate;
		   I_term_pitch_rate+=ki1*error_pitch_rate;

		   if(I_term_pitch_rate>PID_output_max){
			   I_term_pitch_rate = PID_output_max;
		   }else if(I_term_pitch_rate<-PID_output_max){
				I_term_pitch_rate = -PID_output_max;
		   }

		   if(I_term_roll_rate>PID_output_max){
			   I_term_roll_rate = PID_output_max;
		   }else if(I_term_roll_rate<-PID_output_max){
			   I_term_roll_rate = -PID_output_max;
		   }

		   /*Compute PID Output*/
		   kokopter.roll_rate_adjust = kp1 * error_roll_rate + I_term_roll_rate + kd1 * d_input_roll_rate;
		   kokopter.pitch_rate_adjust = kp1 * error_pitch_rate + I_term_pitch_rate + kd1 * d_input_pitch_rate;

		   if(kokopter.pitch_rate_adjust>PID_output_max)kokopter.pitch_rate_adjust=PID_output_max;
		   else if(kokopter.pitch_rate_adjust<-PID_output_max)kokopter.pitch_rate_adjust=-PID_output_max;

		   if(kokopter.roll_rate_adjust>PID_output_max)kokopter.roll_rate_adjust=PID_output_max;
		   if(kokopter.roll_rate_adjust<-PID_output_max)kokopter.roll_rate_adjust=-PID_output_max;

#endif
#ifdef PID_RATE_AND_ANGLE

		   //first the outer control loop
		   //driven by the reference angle and measured angle from both gyros and acc
		   //provides angular velocity output for the inner loop
		   I_term_roll += ki2*error_roll;
		   I_term_pitch += ki2*error_pitch;

		   if(I_term_pitch>PID_output_max){
			   I_term_pitch = PID_output_max;
		   }else if(I_term_pitch<-PID_output_max){
			   I_term_pitch = -PID_output_max;
		   }

		   if(I_term_roll>PID_output_max){
			   I_term_roll = PID_output_max;
		   }else if(I_term_roll<-PID_output_max){
			   I_term_roll = -PID_output_max;
		   }


		   /*Compute PID Output*/
		   kokopter.roll_adjust = kp2 * error_roll + I_term_roll + kd2 * d_input_roll;
		   kokopter.pitch_adjust = kp2 * error_pitch + I_term_pitch + kd2 * d_input_pitch;
		   //trim to max values
		   if(kokopter.pitch_adjust>PID_output_rate_max)kokopter.pitch_adjust=PID_output_rate_max;
		   else if(kokopter.pitch_adjust<-PID_output_rate_max-30)kokopter.pitch_adjust=-PID_output_rate_max;
		   if(kokopter.roll_adjust>PID_output_rate_max)kokopter.roll_adjust=PID_output_rate_max;
		   if(kokopter.roll_adjust<-PID_output_rate_max)kokopter.roll_adjust=-PID_output_rate_max;

		   //now the inner loop
		   //driven by the desired angular rate which is the output of outer loop and the measured angular rate from gyros
		   error_roll_rate = kokopter.roll_adjust - gyro_rates_euler[0];
		   error_pitch_rate = kokopter.pitch_adjust - gyro_rates_euler[1];

		   I_term_roll_rate+=ki1*error_roll_rate;
		   I_term_pitch_rate+=ki1*error_pitch_rate;

		   if(I_term_pitch_rate>PID_output_max){
			   I_term_pitch_rate = PID_output_max;
		   }else if(I_term_pitch_rate<-PID_output_max){
				I_term_pitch_rate = -PID_output_max;
		   }

		   if(I_term_roll_rate>PID_output_max){
			   I_term_roll_rate = PID_output_max;
		   }else if(I_term_roll_rate<-PID_output_max){
			   I_term_roll_rate = -PID_output_max;
		   }

		   /*Compute PID Output*/
		   kokopter.roll_rate_adjust = kp1 * error_roll_rate + I_term_roll_rate + kd1 * d_input_roll_rate;
		   kokopter.pitch_rate_adjust = kp1 * error_pitch_rate + I_term_pitch_rate + kd1 * d_input_pitch_rate;

		   if(kokopter.pitch_rate_adjust>PID_output_max)kokopter.pitch_rate_adjust=PID_output_max;
		   else if(kokopter.pitch_rate_adjust<-PID_output_max)kokopter.pitch_rate_adjust=-PID_output_max;

		   if(kokopter.roll_rate_adjust>PID_output_max)kokopter.roll_rate_adjust=PID_output_max;
		   if(kokopter.roll_rate_adjust<-PID_output_max)kokopter.roll_rate_adjust=-PID_output_max;

#endif



#if defined(PID_ANGLE) || defined(PID_RATE_AND_ANGLE)
		   /*Remember some variables for next time*/
			last_input_roll = angles_LP[0];
			last_input_pitch = angles_LP[1];
#endif
#ifdef PID_RATE
			last_input_roll_rate = gyro_rates_body[0];
			last_input_pitch_rate = gyro_rates_body[1];
			kokopter.motor1_speed = kokopter.speed_adjust-kokopter.pitch_rate_adjust-kokopter.roll_rate_adjust-kokopter.yaw_rate_adjust;
			kokopter.motor2_speed = kokopter.speed_adjust+kokopter.pitch_rate_adjust-kokopter.roll_rate_adjust+kokopter.yaw_rate_adjust;
			kokopter.motor3_speed = kokopter.speed_adjust-kokopter.pitch_rate_adjust+kokopter.roll_rate_adjust+kokopter.yaw_rate_adjust;
			kokopter.motor4_speed = kokopter.speed_adjust+kokopter.pitch_rate_adjust+kokopter.roll_rate_adjust-kokopter.yaw_rate_adjust;
#endif
#if defined(PID_ANGLE) || defined(PID_RATE_AND_ANGLE)
			kokopter.motor1_speed = kokopter.speed_adjust-kokopter.pitch_rate_adjust-kokopter.roll_rate_adjust-kokopter.yaw_rate_adjust;
			kokopter.motor2_speed = kokopter.speed_adjust+kokopter.pitch_rate_adjust-kokopter.roll_rate_adjust+kokopter.yaw_rate_adjust;
			kokopter.motor3_speed = kokopter.speed_adjust-kokopter.pitch_rate_adjust+kokopter.roll_rate_adjust+kokopter.yaw_rate_adjust;
			kokopter.motor4_speed = kokopter.speed_adjust+kokopter.pitch_rate_adjust+kokopter.roll_rate_adjust-kokopter.yaw_rate_adjust;
#endif
			if(kokopter.motor1_speed<0)kokopter.motor1_speed=0;
			else if(kokopter.motor1_speed>100)kokopter.motor1_speed=100;
			if(kokopter.motor4_speed<0)kokopter.motor4_speed=0;
			else if(kokopter.motor4_speed>100)kokopter.motor4_speed=100;
			if(kokopter.motor2_speed<0)kokopter.motor2_speed=0;
			else if(kokopter.motor2_speed>100)kokopter.motor2_speed=100;
			if(kokopter.motor3_speed<0)kokopter.motor3_speed=0;
			else if(kokopter.motor3_speed>100)kokopter.motor3_speed=100;
			set_motor_speed(0, kokopter.motor1_speed);
			set_motor_speed(1, kokopter.motor2_speed);
			set_motor_speed(2, kokopter.motor3_speed);
			set_motor_speed(3, kokopter.motor4_speed);

		}else{
			set_motor_speed(2, 0);
			set_motor_speed(0, 0);
			set_motor_speed(3, 0);
			set_motor_speed(1, 0);
#if defined(PID_ANGLE) || defined(PID_RATE_AND_ANGLE)
			I_term_pitch = 0;
			I_term_roll = 0;
#endif
#if defined(PID_RATE) || defined(PID_RATE_AND_ANGLE)
			I_term_pitch_rate = 0;
			I_term_roll_rate = 0;
#endif

		}

	}else{
		GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_3,0);
			set_motor_speed(2, 0);
			set_motor_speed(0, 0);
			set_motor_speed(3, 0);
			set_motor_speed(1, 0);
#if defined(PID_ANGLE) || defined(PID_RATE_AND_ANGLE)
			I_term_pitch = 0;
			I_term_roll = 0;
#endif
#if defined(PID_RATE) || defined(PID_RATE_AND_ANGLE)
			I_term_pitch_rate = 0;
			I_term_roll_rate = 0;
#endif
	}










}




int main(void) {
	float duty_cycle=0.0;
	kokopter.channel1_pulse_width = 0;
	kokopter.channel1_measuring = 0;
	kokopter.channel2_pulse_width = 0;
	kokopter.channel2_measuring = 0;
	kokopter.channel3_pulse_width = 0;
	kokopter.channel3_measuring = 0;
	kokopter.channel4_pulse_width = 0;
	kokopter.channel4_measuring = 0;
	kokopter.motor1_speed=0;
	kokopter.motor2_speed=0;
	kokopter.motor3_speed=0;
	kokopter.motor4_speed=0;
	kokopter.engines_on=0;
	kokopter.ready=0;
	buff_id=0;

#if defined(PID_ANGLE) || defined(PID_RATE_AND_ANGLE)
	set_PID_constants(kp2,ki2,kd2,2);
#endif
	set_PID_constants(kp1,ki1,kd1,1);
	pwm_frequency = 100;
	motor_set_timer =0;

	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);

	SysCtlClockSet(SYSCTL_SYSDIV_2_5|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN);

	duty_cycle=0.9;

	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE,GPIO_PIN_1);
	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE,GPIO_PIN_3);
	GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1,2);


	//set_pwm_duty_cycle(2,50,duty_cycle);

//	//set PORTE PIN1 as external interrupt

	GPIOPinTypeGPIOInput(GPIO_PORTE_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4);
	GPIOIntTypeSet(GPIO_PORTE_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4, GPIO_BOTH_EDGES);
	GPIOPinIntEnable(GPIO_PORTE_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4);
	IntEnable(INT_GPIOE);



	GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_PIN_4);
	GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_4, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);



	//
	// Enable lazy stacking for interrupt handlers.  This allows floating-point
	// instructions to be used within interrupt handlers, but at the expense of
	// extra stack usage.
	//
	FPUEnable();
	FPULazyStackingEnable();



	//
	// Enable the GPIO pins for the LED (PF2).
	//
	//GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_2);

	//
	// Enable the peripherals used by this example.
	//
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

	//set systick to 10us
	SysTickPeriodSet(SysCtlClockGet()/100);

	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);
	TimerConfigure(TIMER2_BASE, TIMER_CFG_PERIODIC);
	TimerLoadSet(TIMER2_BASE, TIMER_A, SysCtlClockGet());
	TimerEnable(TIMER2_BASE, TIMER_A);
	//
	// Enable processor interrupts.
	//


	//set PB5 as +3.3V and PE4 as gnd for powering the gps
//	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
//	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
//	GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_5);
//	GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_4);
//	GPIOPinWrite(GPIO_PORTB_BASE,GPIO_PIN_5,32);
//	GPIOPinWrite(GPIO_PORTE_BASE,GPIO_PIN_4,0);

	//
	// Set GPIO A0 and A1 as UART pins.
	//
	GPIOPinConfigure(GPIO_PA0_U0RX);
	GPIOPinConfigure(GPIO_PA1_U0TX);
	GPIOPinConfigure(GPIO_PB0_U1RX);
	GPIOPinConfigure(GPIO_PB1_U1TX);
	ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
	ROM_GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);

	//
	// Configure the UART for 115,200, 8-N-1 operation.
	//
	ROM_UARTConfigSetExpClk(UART0_BASE, ROM_SysCtlClockGet(), 115200,
							(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
							 UART_CONFIG_PAR_NONE));

	ROM_UARTConfigSetExpClk(UART1_BASE, ROM_SysCtlClockGet(), 9600,
								(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
								 UART_CONFIG_PAR_NONE));

	//
	// Enable the UART interrupt.
	//
	ROM_IntEnable(INT_UART0);
	ROM_IntEnable(INT_UART1);
	ROM_UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);
	ROM_UARTIntEnable(UART1_BASE, UART_INT_RX | UART_INT_RT);


	set_timer_pwm(0,pwm_frequency,duty_cycle);
	set_timer_pwm(1,pwm_frequency,duty_cycle);
	//
	// Prompt for text to be entered.
	//

	setup_I2C();


	int i,x,y,z;

	write_to_I2C(HMC_ADDR, 0x02, 0x00);
//		ROM_SysCtlDelay(6*(ROM_SysCtlClockGet()/3000));
//		read_from_I2C(HMC_ADDR, HMC_X_ADDR, buffer,6);
//		x = ((int16_t)(((uint16_t)buffer[0]<<8) | (uint16_t)buffer[1]));
//		y = ((int16_t)(((uint16_t)buffer[2]<<8) | (uint16_t)buffer[3]));
//		z = ((int16_t)(((uint16_t)buffer[4]<<8) | (uint16_t)buffer[5]));

	uint8_t ADXL345_ID,ITG3205_ID;

	read_from_I2C(ADXL345_ADDR, 0x00, &ADXL345_ID, 1);

	write_to_I2C(ADXL345_ADDR, 0x31, 0x07);
	//  ADXL345 POWER_CTL
	write_to_I2C(ADXL345_ADDR, 0x2D, 0);
	write_to_I2C(ADXL345_ADDR, 0x2D, 16);
	write_to_I2C(ADXL345_ADDR, 0x2D, 8);


	//read gyro ID
	read_from_I2C(ITG3205_ADDR, 0x00, &ITG3205_ID, 1);
	//set gyro full scale
	write_to_I2C(ITG3205_ADDR, 22, 24);

	calibrate_gyro(gyro_offset);
//
//	calibrate_mag(mag_gain);

	//set systick priority
	HWREG(0xE000ED20) |= 4<<29;
	IntPrioritySet(INT_GPIOE,0);

	read_from_I2C(ITG3205_ADDR, ITG3205_X_ADDR, buffer, 6);
	x = (int)buffer[1] | ((int)buffer[0] << 8);
	y = ( (int)buffer[3] | ((int)buffer[2] << 8) ) * -1;
	z = ( (int)buffer[5] | ((int)buffer[4] << 8) ) * -1;
	SysTickEnable();
	SysTickIntEnable();



	//ROM_IntMasterEnable();
	//UARTSend((unsigned char *)"\033[2JEnter text: ", 16);
	IntMasterEnable();
	done =0;
	int neco =5;

	//clear any characters accidentaly appearing in UART buffer
	while(ROM_UARTCharsAvail(UART0_BASE))
	{
		unsigned char dummy = UARTCharGetNonBlocking(UART0_BASE);
	}

	GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1,0);



	while(1){

		long delay = SysCtlClockGet()/100;
		SysCtlDelay(delay);

#ifdef TESTING
#ifndef REALTIME_READOUT
		//send sync flag
		uint8_t sync_flag[10] = {255,255,255,255,255};
		UARTSend((unsigned char *)&sync_flag[0], 5);
		SysCtlDelay(10000);

		//sned non-filtered acc angles
		UARTSend((unsigned char *)&acc_angle[0], 4);
		SysCtlDelay(10000);
		UARTSend((unsigned char *)&acc_angle[1], 4);
		SysCtlDelay(10000);
		UARTSend((unsigned char *)&acc_angle[2], 4);
		//sned flitered acc angles
		UARTSend((unsigned char *)&acc_angle_filtered[0], 4);
		SysCtlDelay(10000);
		UARTSend((unsigned char *)&acc_angle_filtered[1], 4);
		SysCtlDelay(10000);
		UARTSend((unsigned char *)&acc_angle_filtered[2], 4);
		//send filtered gyro rates
#ifdef PID_RATE
		SysCtlDelay(10000);
		UARTSend((unsigned char *)&gyro_rates_body[0], 4);
		SysCtlDelay(10000);
		UARTSend((unsigned char *)&gyro_rates_body[1], 4);
		SysCtlDelay(10000);
		UARTSend((unsigned char *)&gyro_rates_body[2], 4);
#elif defined( PID_ANGLE) || defined(PID_RATE_AND_ANGLE)
		SysCtlDelay(10000);
		UARTSend((unsigned char *)&gyro_rates_euler[0], 4);
		SysCtlDelay(10000);
		UARTSend((unsigned char *)&gyro_rates_euler[1], 4);
		SysCtlDelay(10000);
		UARTSend((unsigned char *)&gyro_rates_euler[2], 4);
#endif

		UARTSend((unsigned char *)&gyro_rates_filtered[0], 4);
		SysCtlDelay(10000);
		UARTSend((unsigned char *)&gyro_rates_filtered[1], 4);
		SysCtlDelay(10000);
		UARTSend((unsigned char *)&gyro_rates_filtered[2], 4);


		UARTSend((unsigned char *)&angles_LP[0], 4);
		SysCtlDelay(10000);
		UARTSend((unsigned char *)&angles_LP[1], 4);
		SysCtlDelay(10000);
		UARTSend((unsigned char *)&angles_LP[2], 4);
		SysCtlDelay(10000);
#ifdef PID_RATE
		UARTSend((unsigned char *)&kokopter.pitch_rate_adjust, 4);
		SysCtlDelay(1000);
		UARTSend((unsigned char *)&kokopter.roll_rate_adjust, 4);
		SysCtlDelay(1000);
		UARTSend((unsigned char *)&kokopter.yaw_rate_adjust, 4);
		SysCtlDelay(10000);
#elif defined( PID_ANGLE) || defined(PID_RATE_AND_ANGLE)
//		UARTSend((unsigned char *)&kokopter.pitch_adjust, 4);
//		SysCtlDelay(10000);
//		UARTSend((unsigned char *)&kokopter.roll_adjust, 4);
//		SysCtlDelay(10000);
//		UARTSend((unsigned char *)&kokopter.yaw_adjust, 4);
		SysCtlDelay(10000);
		UARTSend((unsigned char *)&kokopter.pitch_rate_adjust, 4);
		SysCtlDelay(10000);
		UARTSend((unsigned char *)&kokopter.roll_rate_adjust, 4);
		SysCtlDelay(1000);
		UARTSend((unsigned char *)&kokopter.yaw_rate_adjust, 4);
		SysCtlDelay(10000);
//		UARTSend((unsigned char *)&kokopter.motor1_speed, 4);
//		SysCtlDelay(10000);
//		UARTSend((unsigned char *)&kokopter.motor2_speed, 4);
//		SysCtlDelay(1000);
//		UARTSend((unsigned char *)&kokopter.motor3_speed, 4);
//		SysCtlDelay(10000);
#endif


#endif
#endif

	};

	return 0;
}
