#include "mpu_tool.h"
#include <stdio.h>
#include <math.h>
#include <string.h>
#include "AP_Math.h"
#include "Vector3f.h"
#include "Matrix3f.h"
#include "TM4C.h"
#include "AP_AHRS_DCM.h"
#include "AP_Baro.h"
#include "AP_BufferFloat.h"
#include "AP_Inertial_Nav.h"
#include "MPU6050.h"
#include "FC_PI.h"
#include "FC_PID.h"
#include "RC_Channel.h"
#include "Parameters.h"
#include "ArduCopter.h"
#include "confige.h"
#include "PWM.h"
#include "Timer_user.h"
#include "Delay.h"
#include "rate_controller.h"
#include "utils/uartstdio.h"
#include "utils/ustdlib.h"
#include "Sys.h"
#include "debug.h"
long pid_p=0,pid_d=0;


//current rate (unit: radians)
Vector3f current_omega = {0, 0, 0};

//targe attitude (100 * degree)
long target_roll_degree = 0, target_pitch_degree = 0, target_yaw_degree = 0;
#define DEFAULT_MOVE_ANGLE 1000

//current attitude
AP_AHRS_DCM ap_ahrs_dcm;

//calculate longerval (unit: s)
float G_Dt = 0.01f;//0.01f

//targe rate(100 * degree/s)
long roll_rate_target_ef, pitch_rate_target_ef, yaw_rate_target_ef;
long roll_rate_target_bf, pitch_rate_target_bf, yaw_rate_target_bf;

float cos_roll_x         = 1;
float cos_pitch_x        = 1;
float cos_yaw_x          = 1;
float sin_yaw_y;
float sin_roll;
float sin_pitch;

//motor
AP_MotorsQuard ap_motors_quard;

//height (cm)
long height_current, height_target = 0, height_last = -1;
int height_update_count;
int alt_hold=0;  //高度控制开关
#define HEIGHT_UPDATE_PERIOD 3

//position
long target_point_x ;
long target_point_y ;
long current_point_x = 0;
long current_point_y = 0;
#define POINT_UPDATE_PERIOD 5
int hover_hold=0;   //悬停控制开关

//task reference
enum TaskType task_type;
int begin_fly = 0;
int height_base =0;

void arducopter_init(void)
{
	current_omega.x = 0;
	current_omega.y = 0;
	current_omega.z = 0;
	
	//init motor
	_Parameters_init();
	ap_motors_quard_init(&ap_motors_quard, &rc_1, &rc_2, &rc_3, &rc_4);
	height_base = 300;
	rc_3.servo_out = height_base ;//给定了油门通道的servo_out=720,calculate the rc_throtle.radio_out=1100+542
 
	
	//current attitude

	//zero rate
	roll_rate_target_ef = pitch_rate_target_ef = yaw_rate_target_ef = 0;
	roll_rate_target_bf = pitch_rate_target_bf = yaw_rate_target_bf = 0;

	height_target = TARGET_HEIGHT;
	height_current = -1;
	
	target_point_x = 0;
	target_point_y = 0;
	
	height_last = -1;
	height_update_count = 0;

	target_roll_degree = 0;
	target_pitch_degree = 0;
	target_yaw_degree = (Q_ANGLE.Z_OFFSET)*100;
	//alt_hold =1;
	//hover_hold = 1;
}


void arducopter_fast_loop(void)
{ 

	
	// IMU DCM Algorithm
	// --------------------
	read_AHRS();//根据欧拉角计算出目标角速度从大地坐标系变换到机体坐标系的目标角速度
  
	read_inertia();//当有位置变化时控制目标姿态

	// custom code/exceptions for flight modes
	// ---------------------------------------
	update_yaw_mode();
	
	update_roll_pitch_mode();

	// update targets to rate controllers
 	update_rate_contoller_targets();//根据欧拉角计算出相对大地坐标系的目标角速度
 
	run_rate_controllers();//输出四个电机的三个通道的角速度控制量
 	
  	//control the motor
	set_servos_4();
 	
}


void read_AHRS(void)
{
	Matrix3f *temp_matrix = &ap_ahrs_dcm.dcm_matrix;
	float temp;

	//rotate
  matrix3f_from_euler(temp_matrix,current_omega.x ,current_omega.y,current_omega.z);//update current rate

	//update triangle value in order to calculate current goal jiaosudu 
	cos_pitch_x = safe_sqrt(1 - (temp_matrix->c.x * temp_matrix->c.x));     // level = 1
	cos_roll_x = temp_matrix->c.z / cos_pitch_x;                       // level = 1
	cos_pitch_x = (float)constrain(cos_pitch_x, 0, 1.0);
	cos_roll_x  = (float)constrain(cos_roll_x, -1.0, 1.0);

	sin_yaw_y = temp_matrix->a.x; // 1y = north
	cos_yaw_x = temp_matrix->b.x; // 0x = north
	temp = (float)sqrt(sin_yaw_y * sin_yaw_y + cos_yaw_x * cos_yaw_x);
	sin_yaw_y /= temp;
	cos_yaw_x /= temp;

	sin_pitch = -temp_matrix->c.x;
	sin_roll = temp_matrix->c.y / cos_pitch_x;
  
}

extern uint32_t last_update;
void read_inertia(void)
{
	 static uint32_t last_of_update;
	 if(height_current <= 5000 && last_of_update != last_update && hover_hold ==1)
	 {
		 last_of_update = last_update;
		 ADNS3080_update_position(radians(Q_ANGLE.X),radians(Q_ANGLE.Y),(float)height_current*0.1);
     update_point_control();			
		
  }
	//ap_inertial_nav_update(&ap_inertial_nav, 0.01F);
}

void update_yaw_mode(void)
{
	get_stabilize_yaw(target_yaw_degree);
}

void update_roll_pitch_mode(void)
{
	get_stabilize_roll(target_roll_degree);
	get_stabilize_pitch(target_pitch_degree);
}

void update_rate_contoller_targets(void)
{

	roll_rate_target_bf = (long)(roll_rate_target_ef - sin_pitch * yaw_rate_target_ef);//目标角速度
	pitch_rate_target_bf = (long)(cos_roll_x  * pitch_rate_target_ef + sin_roll * cos_pitch_x * yaw_rate_target_ef);
	yaw_rate_target_bf = (long)(cos_pitch_x * cos_roll_x * yaw_rate_target_ef - sin_roll * pitch_rate_target_ef);

}

void run_rate_controllers(void)
{

	rc_1.servo_out = get_rate_roll(roll_rate_target_bf);//角速度控制量，范围-5000到+5000
	rc_2.servo_out = get_rate_pitch(pitch_rate_target_bf);
	rc_4.servo_out = get_rate_yaw(yaw_rate_target_bf);

	if(alt_hold ==1 && height_current != -1)    //
	{
		height_update_count++;
		if(height_update_count == HEIGHT_UPDATE_PERIOD)
		{
			height_update_count = 0;

			update_throttle_servo_out();
		}
  }
}

void update_throttle_servo_out(void)
{
	long pid;
	RC_Channel* rc_thr = ap_motors_quard.rc_throttle;
	
	pid = fc_pid_get_pid(&ac_pid_height, height_target - height_current, G_Dt * HEIGHT_UPDATE_PERIOD);

	pid += height_base;
	pid = constrain(pid, rc_thr->low_out, rc_thr->high_out);//高度油门输出[130,1000],最终Motor_out=[130,850]
	rc_thr->servo_out = (int)pid;
}

void update_point_control(void)
{
    long pid[2];
//     long target_vectory_x=0,target_vectory_y=0;
// 	  target_vectory_x = fc_get_p(target_point_x - current_point_x);
// 	  target_vectory_y = fc_get_p(target_point_y - current_point_y);
// 	 	pid[0]=fc_pid_get_pid(&ac_pid_point_x,(target_vectory_x-dx)*100,G_Dt*POINT_UPDATE_PERIOD);
// 		pid[1]=fc_pid_get_pid(&ac_pid_point_y,(target_vectory_y-dy)*100,G_Dt*POINT_UPDATE_PERIOD);
      pid[0]=fc_pid_get_pid(&ac_pid_point_x,(target_point_x-current_point_x)*100,G_Dt*POINT_UPDATE_PERIOD);
		  pid[1]=fc_pid_get_pid(&ac_pid_point_y,(target_point_y-current_point_y)*100,G_Dt*POINT_UPDATE_PERIOD);
// 	  long p1,i1,d1,p2,i2,d2;
// 	  p1=fc_pid_get_p(&ac_pid_point_x, (target_point_x-current_point_x)*100);
// 	  i1=fc_pid_get_i(&ac_pid_point_x, (target_point_x-current_point_x)*100, G_Dt*POINT_UPDATE_PERIOD);
// 	  d1=fc_pid_get_d2(&ac_pid_point_x, (target_point_x-current_point_x)*100, G_Dt*POINT_UPDATE_PERIOD);
// 	  p2=fc_pid_get_p(&ac_pid_point_y, (target_point_y-current_point_y)*100);
// 	  i2=fc_pid_get_i(&ac_pid_point_y, (target_point_y-current_point_y)*100, G_Dt*POINT_UPDATE_PERIOD);
// 	  d2=fc_pid_get_d2(&ac_pid_point_y, (target_point_y-current_point_y)*100, G_Dt*POINT_UPDATE_PERIOD);
// 		pid[0]=(p1+i1+d1);
// 		pid[1]=(p2+i2+d2);

	  //pid_x=calc_pid_position(&ac_pid_point_x,(target_point_x-current_point_x)*100,G_Dt*POINT_UPDATE_PERIOD);
	  //pid_y=calc_pid_position(&ac_pid_point_y,(target_point_y-current_point_y)*100,G_Dt*POINT_UPDATE_PERIOD);
	
// 	  debug_printf("BP:%d$%d\n",current_point_x,current_point_y);
// 	  debug_printf("X_P:%d\n",p1);
// 	  debug_printf("X_I:%d\n",i1);
// 	  debug_printf("X_D:%d\n",d1);
// 	  debug_printf("Y_P:%d\n",p2);
// 	  debug_printf("Y_I:%d\n",i2);
// 	  debug_printf("Y_D:%d\n",d2);
// 	  debug_printf("pid_x:%d\n",pid[0]);
//     debug_printf("pid_y:%d\n",pid[1]);
	    pid[0] = constrain(pid[0],-1000,1000);
	    pid[1] = constrain(pid[1],-1000,1000);
	
    target_roll_degree = pid[0];
    target_pitch_degree = pid[1];
	  
    
    //target_roll_degree = -asin(pid[1]/GRAVITY);
    //target_pitch_degree = pid[0]/(sqrt(GRAVITY*GRAVITY-pid[1]*pid[1]));
	  //debug_printf("BP:%d$%d\n",current_point_x,current_point_y);
	
}

void set_servos_4(void)
{
	ap_motors_quard_output_armed(&ap_motors_quard);
}

//height cm
void goto_height(int height)
{
	height_target = height;
}

//move reference
void move_forward(void)
{
	target_pitch_degree = -DEFAULT_MOVE_ANGLE;
}

void move_backward(void)
{
	target_pitch_degree = DEFAULT_MOVE_ANGLE;
}

void middle_pitch(void)
{
	target_pitch_degree = 0;
}

void move_left(void)
{
	target_roll_degree = -DEFAULT_MOVE_ANGLE;
}

void middle_roll(void)
{
	target_roll_degree = 0;
}

void move_right(void)
{
	target_roll_degree = DEFAULT_MOVE_ANGLE;
}


void do_basic_low_height_task(void)
{
	
	//go up
	goto_height(40);
	//forward
	move_forward();

	delay_millis(1500);
	goto_height(50);
	delay_millis(700);
	
	middle_pitch();
	goto_height(0);
	delay_millis(200);
	
	PWM_Set(1000,1000,1000,1000); 
	//Timer0Stop();
	begin_fly = 0;
}