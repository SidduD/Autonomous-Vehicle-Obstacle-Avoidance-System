#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/log.h>

#include <drivers/drv_hrt.h>
#include <uORB/Publication.hpp>
#include <uORB/topics/test_motor.h>
#include <uORB/topics/rc_channels.h>
#include <uORB/topics/debug_value.h>
#include <stdio.h>
#include <string.h>


#define DC_MOTOR 0
#define SERVO_MOTOR 1

extern "C" __EXPORT int hello_world_main(int argc, char *argv[]);

int hello_world_main(int argc, char *argv[])
{
	px4_sleep(2);
	
	test_motor_s dc_motor;
	test_motor_s servo_motor;

	double motor_value = 0.5; // a number between 0 to 1
	double servo_value = 0.5; // a number between 0 to 1
	
	

	int debug_handle = orb_subscribe(ORB_ID(debug_value));
	orb_set_interval(debug_handle, 200);
	
	debug_value_s debug_data;
	
	uORB::Publication<test_motor_s> dc_motor_pub(ORB_ID(test_motor));
	uORB::Publication<test_motor_s> servo_motor_pub(ORB_ID(test_motor));
	
	int rc_channels_handle;

	rc_channels_handle = orb_subscribe(ORB_ID(rc_channels));
	orb_set_interval(rc_channels_handle, 200);
	/*
	struct{
		int s_value;
		int a_value;
			
	}unpacked_data;*/

	while(1)
	{
		

		dc_motor.timestamp = hrt_absolute_time();
		dc_motor.motor_number = DC_MOTOR;
		dc_motor.value = (float)motor_value;
		dc_motor.action = test_motor_s::ACTION_RUN;
		dc_motor.driver_instance = 0;
		dc_motor.timeout_ms = 0;

		servo_motor.timestamp = hrt_absolute_time();
		servo_motor.motor_number = SERVO_MOTOR;
		servo_motor.value = (float)servo_value;
		servo_motor.action = test_motor_s::ACTION_RUN;
		servo_motor.driver_instance = 0;
		servo_motor.timeout_ms = 0;


		dc_motor_pub.publish(dc_motor);
		servo_motor_pub.publish(servo_motor);
		px4_sleep(1);
		
		orb_copy(ORB_ID(debug_value), debug_handle, &debug_data);
		
		int angle = (int)debug_data.ind%10;
		int speed = debug_data.ind - angle;
		//memcpy(&unpacked_data,&debug_data.value,sizeof(unpacked_data));
		
		double speedf = (double)speed;
		motor_value = (((speedf)/100)*0.5)+0.5;
	
		
		if (angle == 1){
			servo_value = 0;
		}
		else if (angle == 2){
			servo_value = 0.5;
		}
		else if (angle ==3){
			servo_value = 1;
		}

		px4_sleep(1);
		
		
		
	}


	PX4_INFO("The motor will be stopped");
	dc_motor.timestamp = hrt_absolute_time();
	dc_motor.motor_number = DC_MOTOR;
	dc_motor.value = 0.5;
	dc_motor.driver_instance = 0;
	dc_motor.timeout_ms = 0;
	dc_motor_pub.publish(dc_motor);
	return 0;
}
