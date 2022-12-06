/*
 * File:          my_controller.c
 * Date:
 * Description:
 * Author:
 * Modifications:
 */

/*
 * You may need to add include files like <webots/distance_sensor.h> or
 * <webots/motor.h>, etc.
 */
#include <webots/robot.h>
#include <webots/inertial_unit.h>
#include <webots/motor.h>
#include <stdio.h>

/*
 * You may want to add macros here.
 */
#define TIME_STEP 32
#define ANG2RAD(x) (x/180.0f*M_PI)
#define RAD2ANG(x) (x/M_PI*180.0f)

typedef struct
{
  float error;
  float error_last;

  float desired_ang;

  float kp;
  float ki;
  float kd;

  float output;
  float max_output;
}pos_pid_t;

typedef struct
{
  float error;
  float error_last;

  float desired_vel;
  float now_vel;

  float kp;
  float ki;
  float kd;

  float i_out;
  float output;
  float max_output;
}vel_pid_t;

pos_pid_t pos_pid;
vel_pid_t vel_pid;

float pos_out;
float vel_out;

float Pos_PID_Cal(pos_pid_t *pid, const double *imu_data);
float Vel_PID_Cal(vel_pid_t *pid, float now_speed);
void Pos_PID_Init(pos_pid_t *pid);
void Vel_PID_Init(vel_pid_t *pid);

float OutPutLimit(float val1, float max_val);
/*
 * This is the main program.
 * The arguments of the main function can be specified by the
 * "controllerArgs" field of the Robot node
 */
int main(int argc, char **argv) {
  /* necessary to initialize webots stuff */
  wb_robot_init();
  Pos_PID_Init(&pos_pid);
  Vel_PID_Init(&vel_pid);

  // 获取imu的tag并使能imu
  WbDeviceTag imu = wb_robot_get_device("imu");
  wb_inertial_unit_enable(imu, TIME_STEP);

  const double *imu_data;
  

  /*
   * You should declare here WbDeviceTag variables for storing
   * robot devices like this:
   *  WbDeviceTag my_sensor = wb_robot_get_device("my_sensor");
   *  WbDeviceTag my_actuator = wb_robot_get_device("my_actuator");
   */
  WbDeviceTag motor1 = wb_robot_get_device("motor1");
  WbDeviceTag motor2 = wb_robot_get_device("motor2");

  wb_motor_set_position(motor1, INFINITY);
  wb_motor_set_position(motor2, INFINITY);

  wb_motor_set_velocity(motor1, 0.0f);
  wb_motor_set_velocity(motor2, 0.0f);
  /* main loop
   * Perform simulation steps of TIME_STEP milliseconds
   * and leave the loop when the simulation is over
   */
  while (wb_robot_step(TIME_STEP) != -1) {
    /*
     * Read the sensors :
     * Enter here functions to read sensor data, like:
     *  double val = wb_distance_sensor_get_value(my_sensor);
     */
    imu_data = wb_inertial_unit_get_roll_pitch_yaw(imu); 
    vel_pid.now_vel = wb_motor_get_velocity(motor1);

    //printf("%f, %f, %f \n",imu_data[0],imu_data[1], imu_data[2]);
    /* Process sensor data here */
    vel_out = Vel_PID_Cal(&vel_pid, vel_pid.now_vel);
    pos_out = Pos_PID_Cal(&pos_pid, imu_data);
    
    printf("vel_out = %f , pos_out = %f \n",vel_out,pos_out);

    wb_motor_set_velocity(motor1, pos_out);
    wb_motor_set_velocity(motor2, pos_out);
    /*
     * Enter here functions to send actuator commands, like:
     * wb_motor_set_position(my_actuator, 10.0);
     */
  };

  /* Enter your cleanup code here */

  /* This is necessary to cleanup webots resources */
  wb_robot_cleanup();

  return 0;
}

float Pos_PID_Cal(pos_pid_t *pos_pid, const double *imu_data)
{
  pos_pid->error = (RAD2ANG(imu_data[0]) + vel_out) - pos_pid->desired_ang;
  
  pos_pid->output = pos_pid->kp * pos_pid->error + pos_pid->kd * (pos_pid->error - pos_pid->error_last);

  pos_pid->error_last = pos_pid->error;

  pos_pid->output = OutPutLimit(pos_pid->output, pos_pid->max_output);

  return pos_pid->output;
}

float Vel_PID_Cal(vel_pid_t *vel_pid, float now_speed)
{
  vel_pid->error =  now_speed - vel_pid->desired_vel;

  vel_pid->i_out += vel_pid->ki * vel_pid->error;

  vel_pid->i_out = OutPutLimit(vel_pid->i_out, vel_pid->max_output);

  vel_pid->output = vel_pid->kp * vel_pid->error + vel_pid->ki * vel_pid->i_out;

  vel_pid->output = OutPutLimit(vel_pid->output, vel_pid->max_output);

  return vel_pid->output;
}

void Pos_PID_Init(pos_pid_t *pid)
{
  pid->kp = 2.0f;
  pid->kd = 0.5f;
  pid->ki = 0.0f;

  pid->error = 0;
  pid->error_last = 0;
  pid->output = 0;
  pid->desired_ang = 0;
  pid->max_output = 100;
}

float OutPutLimit(float val1, float max_val)
{
  if (val1 > max_val)
  {
    return max_val;
  }
  else if (val1 < -max_val)
  {
    return -max_val;
  }
  else
  {
    return val1;
  }
  
}

void Vel_PID_Init(vel_pid_t *pid)
{
  pid->desired_vel = 0.0f; 
  pid->error = 0;
  pid->error_last = 0;

  pid->kp = 0.4f;
  pid->ki = 0.1f;
  pid->kd = 0.0f;

  pid->i_out = 0.0f;
  pid->max_output = 100;
  pid->output = 0.0f;
}  