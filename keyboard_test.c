/*
 * File:
 * Date:
 * Description:
 * Author: 
 */

/*
 * You may need to add include files like <webots/distance_sensor.h> or
 * <webots/motor.h>, etc.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/inertial_unit.h>
#include <webots/distance_sensor.h>
#include <webots/position_sensor.h>

#include <webots/keyboard.h>

#define TIME_STEP 32
#define MAX_SPEED 6.28


/*
 * This is the main program.
 * The arguments of the main function can be specified by the
 * "controllerArgs" field of the Robot node
 */
int main(int argc, char **argv) {
  /* necessary to initialize webots stuff */
  wb_robot_init();

  /*
   * You should declare here WbDeviceTag variables for storing
   * robot devices like this:
   *  WbDeviceTag my_sensor = wb_robot_get_device("my_sensor");
   *  WbDeviceTag my_actuator = wb_robot_get_device("my_actuator");
   */
   // internal variables

  WbDeviceTag left_motor = wb_robot_get_device("left wheel motor");
  WbDeviceTag right_motor = wb_robot_get_device("right wheel motor");
  wb_keyboard_enable(TIME_STEP);

  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);
  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(right_motor, 0.0);
  
  WbDeviceTag left_encoder = wb_robot_get_device("left wheel sensor");
  wb_position_sensor_enable(left_encoder, TIME_STEP);

  int forwardFlag = 0;
  int backwardFlag = 0;
  int leftFlag = 0;
  int rightFlag = 0;

  double left_encoder_pre = 0;

  double left_speed = MAX_SPEED * 0.3; // 0.7
  double right_speed = MAX_SPEED * 0.3; // 0.7
  
  void check_keyboard() {
    int i = wb_keyboard_get_key();
    switch (i) {
      case WB_KEYBOARD_UP:
        wb_motor_set_velocity(left_motor, left_speed);
        wb_motor_set_velocity(right_motor, right_speed);
        forwardFlag = 1;
        break;
      case WB_KEYBOARD_DOWN:
        wb_motor_set_velocity(left_motor, -left_speed);
        wb_motor_set_velocity(right_motor, -right_speed);
        backwardFlag = 1;
        break;
      case WB_KEYBOARD_LEFT:
        wb_motor_set_velocity(left_motor, -left_speed);
        wb_motor_set_velocity(right_motor, +right_speed);
        leftFlag = 1;
        break;
      case WB_KEYBOARD_RIGHT:
        wb_motor_set_velocity(left_motor, +left_speed);
        wb_motor_set_velocity(right_motor, -right_speed);
        rightFlag = 1;
        break;
      }
  }
  
  while (wb_robot_step(TIME_STEP) != -1) {
  
    double left_encoder_value = wb_position_sensor_get_value(left_encoder);
    double gap = left_encoder_value - left_encoder_pre;
    
    if (forwardFlag == 1) {
      if (gap >= 8.23) {
        wb_motor_set_velocity(left_motor, 0);
        wb_motor_set_velocity(right_motor, 0);
        forwardFlag = 0;
        left_encoder_pre = left_encoder_value;
      }
      
    } else if  (backwardFlag == 1) {
      if (gap <= -8.23) {
        wb_motor_set_velocity(left_motor, 0);
        wb_motor_set_velocity(right_motor, 0);
        backwardFlag = 0;
        left_encoder_pre = left_encoder_value;
      }
    
    } else if (leftFlag == 1) {
      if (gap <= -2.2) {
        wb_motor_set_velocity(left_motor, 0);
        wb_motor_set_velocity(right_motor, 0);
        leftFlag = 0;
        left_encoder_pre = left_encoder_value;
      }
      
    } else if (rightFlag == 1) {
      if (gap >= 2.2) {
        wb_motor_set_velocity(left_motor, 0);
        wb_motor_set_velocity(right_motor, 0);
        rightFlag = 0;
        left_encoder_pre = left_encoder_value;
      }
    }


    double velocity = wb_motor_get_velocity(left_motor);
    if (velocity == 0) {
      check_keyboard();
      if (forwardFlag == 1) {
        printf("You've pressed FORWARD!\n");
      
      } else if  (backwardFlag == 1) {
        printf("You've pressed BACKWARD!\n");
      
      } else if (leftFlag == 1) {
        printf("You've pressed LEFT!\n");
        
      } else if (rightFlag == 1) {
        printf("You've pressed RIGHT!\n");
        
      }
    }
  

     } // robot step

  /* Enter your cleanup code here */
  /* This is necessary to cleanup webots resources */
  wb_robot_cleanup();

  return 0;
}

