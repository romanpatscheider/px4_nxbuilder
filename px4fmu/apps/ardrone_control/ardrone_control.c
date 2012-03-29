/****************************************************************************
 * px4/sensors/tests_main.c
 *
 *   Copyright (C) 2012 Michael Smith. All rights reserved.
 *   Authors: Michael Smith <DrZiplok@me.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "ardrone_control.h"




/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

void read_sensors_raw(void)
{

	if(0 == global_data_trylock(&global_data_sensors_raw.access_conf));//TODO: check if trylock is the right choice, maybe only lock?
	{
		memcpy(sensors_raw.gyro_raw, global_data_sensors_raw.gyro_raw, sizeof(sensors_raw.gyro_raw));
		memcpy(sensors_raw.accelerometer_raw, global_data_sensors_raw.accelerometer_raw, sizeof(sensors_raw.accelerometer_raw));
		memcpy(sensors_raw.magnetometer_raw, global_data_sensors_raw.magnetometer_raw, sizeof(sensors_raw.magnetometer_raw));
		memcpy(sensors_raw.pressure_sensor_raw, global_data_sensors_raw.pressure_sensor_raw, sizeof(sensors_raw.pressure_sensor_raw));
		global_data_unlock(&global_data_sensors_raw.access_conf);
	}

}

void read_quad_motors_setpoint(void)
{

	if(0 == global_data_trylock(&global_data_quad_motors_setpoint.access_conf));//TODO: check if trylock is the right choice, maybe only lock?
	{
		quad_motors_setpoint_desired.motor_front_nw = global_data_quad_motors_setpoint.motor_front_nw;
		quad_motors_setpoint_desired.motor_right_ne = global_data_quad_motors_setpoint.motor_right_ne;
		quad_motors_setpoint_desired.motor_back_se = global_data_quad_motors_setpoint.motor_back_se;
		quad_motors_setpoint_desired.motor_left_sw = global_data_quad_motors_setpoint.motor_left_sw;

		global_data_unlock(&global_data_quad_motors_setpoint.access_conf);
	}
}

void read_gps(void)
{
	if(0 == global_data_trylock(&global_data_gps.access_conf));
	{
		//TODO: determine which gps data the controller needs and copy these here...
		global_data_unlock(&global_data_gps.access_conf);
	}
}

void control_step(void)
{
	int16_t outputBand=0;

	float setpointXrate;
	float setpointYrate;
	float setpointZrate;

	float setpointRateCast[3]={0,0,0};
	float Kp=0;
	float Ki=0;
	float setpointThrustCast=0;
	float startpointFullControll=0;
	float maxThrustSetpoints=0;

	float gyro_filtered[3] = {0,0,0};
	float gyro_filtered_offset[3] = {0,0,0};
	float gyro_alpha = 0;
	float gyro_alpha_offset = 0;
	float errXrate=0;
	float attRatesScaled[3]={0,0,0};

	uint16_t offsetCnt=0;
	float antiwindup=0;


	gyro_filtered_offset[0] = 0.00026631611f*sensors_raw.gyro_raw[0];
	gyro_filtered_offset[1] = 0.00026631611f*sensors_raw.gyro_raw[1];
	gyro_filtered_offset[2] = 0.00026631611f*sensors_raw.gyro_raw[2];

	gyro_filtered[0] = 0.00026631611f*sensors_raw.gyro_raw[1];
	gyro_filtered[1] = 0.00026631611f*sensors_raw.gyro_raw[0];
	gyro_filtered[2] = 0.00026631611f*sensors_raw.gyro_raw[2];

	outputBand=0;
	startpointFullControll = 150.0f;
	maxThrustSetpoints = 511.0f;
	//Kp=60;
	Kp=40.0f;
	//Kp=45;
	Ki=0.0f;
	antiwindup=50.0f;


	//Rate Controller
	setpointRateCast[0]=((float)quad_motors_setpoint_desired.motor_right_ne-9999.0f)*0.01f/180.0f*3.141f;
	setpointRateCast[1]=((float)quad_motors_setpoint_desired.motor_front_nw-9999.0f)*0.01f/180.0f*3.141f;
	setpointRateCast[2]=-((float)quad_motors_setpoint_desired.motor_back_se-127.0f)/180.0f*3.141f;
	//Ki=actuatorDesired.motorRight_NE*0.001f;
	setpointThrustCast=quad_motors_setpoint_desired.motor_left_sw;

	attRatesScaled[0]=0.00026631611f*sensors_raw.gyro_raw[1];
	attRatesScaled[1]=0.00026631611f*sensors_raw.gyro_raw[0];
	attRatesScaled[2]=0.00026631611f*sensors_raw.gyro_raw[2];


	//filtering of the gyroscope values

	//compute filter coefficient alpha

	//gyro_alpha=0.005/(2.0f*3.1415f*200.0f+0.005f);
	//gyro_alpha=0.009;
	gyro_alpha=0.09f;
	gyro_alpha_offset=0.001f;
	//gyro_alpha=0.001;
	//offset estimation and filtering
	offsetCnt++;
	uint8_t i;
	for(i=0; i< 3; i++)
	{	if(offsetCnt<5000){
		gyro_filtered_offset[i] = attRatesScaled[i] * gyro_alpha_offset +  gyro_filtered_offset[i]* (1 - gyro_alpha_offset);
	}
	gyro_filtered[i] = 1.0f*((attRatesScaled[i]-gyro_filtered_offset[i]) * gyro_alpha + gyro_filtered[i] * (1 - gyro_alpha))-0*setpointRateCast[i];
	}

	//rate controller

	//X-axis
	if((Ki*errXrate<antiwindup)&&(Ki*errXrate>-antiwindup)){
		errXrate=errXrate+(setpointRateCast[0]-gyro_filtered[1]);
	}


	setpointXrate=Kp*(setpointRateCast[0]-gyro_filtered[0])+Ki*errXrate;

	//Y-axis
	setpointYrate=Kp*(setpointRateCast[1]-gyro_filtered[1])+Ki*errXrate;
	//Z-axis
	setpointZrate=Kp*(setpointRateCast[2]-gyro_filtered[2])+Ki*errXrate;

	//Mixing
	if (setpointThrustCast<=0){
		setpointThrustCast=0;
		outputBand=0;
	}

	if ((setpointThrustCast<startpointFullControll)&&(setpointThrustCast>0)){
		outputBand=0.75f*setpointThrustCast;
	}

	if((setpointThrustCast>=startpointFullControll)&&(setpointThrustCast<maxThrustSetpoints-0.75f*startpointFullControll)){
		outputBand=0.75f*startpointFullControll;
	}

	if(setpointThrustCast>=maxThrustSetpoints-0.75f*startpointFullControll){
		setpointThrustCast=0.75f*startpointFullControll;
		outputBand=0.75f*startpointFullControll;
	}

	quad_motors_setpoint_desired.motor_front_nw=setpointThrustCast+(setpointXrate+setpointYrate+setpointZrate);
	quad_motors_setpoint_desired.motor_right_ne=setpointThrustCast+(-setpointXrate+setpointYrate-setpointZrate);
	quad_motors_setpoint_desired.motor_back_se=setpointThrustCast+(-setpointXrate-setpointYrate+setpointZrate);
	quad_motors_setpoint_desired.motor_left_sw=setpointThrustCast+(setpointXrate-setpointYrate-setpointZrate);


	if((setpointThrustCast+setpointXrate+setpointYrate+setpointZrate)>(setpointThrustCast+outputBand)){
		quad_motors_setpoint_desired.motor_front_nw=setpointThrustCast+outputBand;
	}
	if((setpointThrustCast+setpointXrate+setpointYrate+setpointZrate)<(setpointThrustCast-outputBand)){
		quad_motors_setpoint_desired.motor_front_nw=setpointThrustCast-outputBand;
	}

	if((setpointThrustCast+(-setpointXrate)+setpointYrate-setpointZrate)>(setpointThrustCast+outputBand)){
		quad_motors_setpoint_desired.motor_right_ne=setpointThrustCast+outputBand;
	}
	if((setpointThrustCast+(-setpointXrate)+setpointYrate-setpointZrate)<(setpointThrustCast-outputBand)){
		quad_motors_setpoint_desired.motor_right_ne=setpointThrustCast-outputBand;
	}

	if((setpointThrustCast+(-setpointXrate)+(-setpointYrate)+setpointZrate)>(setpointThrustCast+outputBand)){
		quad_motors_setpoint_desired.motor_back_se=setpointThrustCast+outputBand;
	}
	if((setpointThrustCast+(-setpointXrate)+(-setpointYrate)+setpointZrate)<(setpointThrustCast-outputBand)){
		quad_motors_setpoint_desired.motor_back_se=setpointThrustCast-outputBand;
	}

	if((setpointThrustCast+setpointXrate+(-setpointYrate)-setpointZrate)>(setpointThrustCast+outputBand)){
		quad_motors_setpoint_desired.motor_left_sw=setpointThrustCast+outputBand;
	}
	if((setpointThrustCast+setpointXrate+(-setpointYrate)-setpointZrate)<(setpointThrustCast-outputBand)){
		quad_motors_setpoint_desired.motor_left_sw=setpointThrustCast-outputBand;
	}

	static int motor_skip_counter = 0;
	if (motor_skip_counter == 5)
	{
		//uint8_t* motorSpeedBuf = ar_get_motor_packet(actuatorDesired.motorFront_NW, actuatorDesired.motorRight_NE, actuatorDesired.motorBack_SE, actuatorDesired.motorLeft_SW);
		uint8_t* motorSpeedBuf = ar_get_motor_packet(15, 15, 15, 15);

		write(ardrone_write, motorSpeedBuf, 5);
		motor_skip_counter = 0;
	}
	motor_skip_counter++;

	usleep(2000);
}

static void *controlloop(void * arg)
{
	while(1)
	{
		/* Get sensor data */

		read_sensors_raw();
		read_quad_motors_setpoint();
//		read_gps(); //TODO: uncomment once gps is used by the controller, read_gps is not yet fully implemented!

		/* Control */
		control_step();

		sleep(CONTROL_LOOP_USLEEP);
	}
}


/****************************************************************************
 * Name: ardrone_control
 ****************************************************************************/

int ardrone_control_main(int argc, char *argv[])
{
    // print text
    printf("Hello, Ardrone Control!\n");
    usleep(100000);

    //default values for arguments
	char * ardrone_uart_name = "/dev/ttyS1";
    char * commandline_usage = "\tusage: ardrone_control -d ardrone-devicename\n";


    //read arguments
	int i;
	if (strcmp(argv[i], "-d") == 0 || strcmp(argv[i], "--device") == 0)  //ardrone set
	{
		if(argc > i+1)
		{
			ardrone_uart_name = argv[i+1];
		}
		else
		{
			printf(commandline_usage);
			return 0;
		}
	}



    /* initialize shared data structures */
	global_data_init(&global_data_gps.access_conf);
	global_data_init(&global_data_sensors_raw.access_conf);
	global_data_init(&global_data_quad_motors_setpoint.access_conf);

	/*open uarts */
	printf("ARDrone UART is %s\n", ardrone_uart_name);
	ardrone_write = open(ardrone_uart_name, O_RDWR | O_NOCTTY | O_NDELAY);

	/* initialize motors */
	ar_init_motors(ardrone_write, &gpios);

	//create pthreads
	pthread_create (&control_thread, NULL, controlloop, NULL);

	//wait for threads to complete:
	pthread_join(control_thread, NULL);

	/* close uarts */
	close(ardrone_uart_name);
	ar_multiplexing_deinit(gpios);



	return 0;
}

