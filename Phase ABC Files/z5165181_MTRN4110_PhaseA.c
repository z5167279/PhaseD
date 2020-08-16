// MTRN4110 PhaseA
// Author: z5165181 Yang CHEN
// Date: 21/06/20

#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/distance_sensor.h>
#include <stdio.h> 
#include <unistd.h>

#define TIME_STEP 64
#define SPEED 6.28*0.4   
#define PI 3.14159
#define POSITION_STEP_LINEAR 165.0/20.0 
#define POSITION_STEP_ROTATION PI/2 * 28.3 / 20.0
//#define PATH_PLAN_FILE_NAME "C:/Users/yangc/OneDrive/Desktop/UNSW/2020/T2/MTRN4110/Phase A/z5165181_MTRN4110_PhaseA/PathPlan.txt" 
#define PATH_PLAN_FILE_NAME "z5165181_MTRN4110_PhaseA/PathPlan.txt" 


int main(int argc, char **argv) {
  /* necessary to initialize webots stuff */
  wb_robot_init();
  double left_position = 0.0;
  double right_position = 0.0; 
  double left_encoder_value = 0.0;
  double right_encoder_value = 0.0;
  double dsF_value;
  double dsR_value;
  double dsL_value;
  
  WbDeviceTag left_motor = wb_robot_get_device("left wheel motor");
  WbDeviceTag right_motor = wb_robot_get_device("right wheel motor");
  
  wb_motor_set_velocity(left_motor, SPEED);
  wb_motor_set_velocity(right_motor, SPEED);
  
  wb_motor_set_position(left_motor, left_position);
  wb_motor_set_position(right_motor, right_position);
  
  // front, left and right distance sensors (added on)  
  WbDeviceTag dsF = wb_robot_get_device("dsF");
  WbDeviceTag dsL = wb_robot_get_device("dsL");
  WbDeviceTag dsR = wb_robot_get_device("dsR");

  // used to determine how much the wheels have rotated 
  WbDeviceTag left_encoder = wb_robot_get_device("left wheel sensor");
  WbDeviceTag right_encoder = wb_robot_get_device("right wheel sensor");
  
  // enable the sensors 
  wb_distance_sensor_enable(dsF, TIME_STEP);
  wb_distance_sensor_enable(dsR, TIME_STEP);
  wb_distance_sensor_enable(dsL, TIME_STEP);
  wb_position_sensor_enable(left_encoder, TIME_STEP);
  wb_position_sensor_enable(right_encoder, TIME_STEP);
  
  // open PathFile, obtain and print sequence
  FILE *fp = fopen(PATH_PLAN_FILE_NAME, "r");
  char seq[100];
  if (fp == NULL){
      printf("Could not open file %s",PATH_PLAN_FILE_NAME);
      return 1;
  }
  fgets(seq, 255, (FILE*)fp);
  printf("Sequence: %s\n", seq);
  fclose(fp);
  
  int Row = seq[0];
  int Column = seq[1];
  char Heading = seq[2];
  char FW[] = "";
  char LW[] = "";
  char RW[] = "";
  int i = 0;
  double tempL = 0;
  double tempR = 0;

  while (wb_robot_step(TIME_STEP) != -1) {
      dsF_value = wb_distance_sensor_get_value(dsF);
      if (dsF_value < 1000) {
        FW[0] = 'Y';
      } else {
        FW[0] = 'N';
      }
      dsR_value = wb_distance_sensor_get_value(dsR);
      if (dsR_value < 1000) {
        RW[0] = 'Y';
      } else {
        RW[0] = 'N';
      }
      dsL_value = wb_distance_sensor_get_value(dsL);
      if (dsL_value < 1000) {
        LW[0] = 'Y';
      } else {
        LW[0] = 'N';
      }
      left_encoder_value = wb_position_sensor_get_value(left_encoder); 
      right_encoder_value = wb_position_sensor_get_value(right_encoder);
      
      if (tempL - left_encoder_value == 0 && tempR - right_encoder_value == 0) {
        if (i < 10) {
          printf("Step: 0%d | ", i); 
        } else {
          printf("Step: %d | ", i); 
        }
        printf("Row: %c, Column: %c, Heading: %c | ", Row, Column, Heading);
        printf("Left Wall: %c, Front Wall: %c, Right Wall: %c\n", LW[0], FW[0], RW[0]);       
        if (seq[i+3] == 'F') {
          left_position += POSITION_STEP_LINEAR;
          right_position += POSITION_STEP_LINEAR;
          switch (Heading){
            case 'N':
              Row -= 1;
              break;           
            case 'S':
              Row += 1;
              break;          
            case 'E':
              Column += 1;
              break;       
            case 'W':
              Column -= 1;
          }         
        } else if (seq[i+3] == 'L') {
          left_position -= POSITION_STEP_ROTATION;
          right_position += POSITION_STEP_ROTATION;
          switch (Heading){
            case 'N':
              Heading = 'W';
              break;              
            case 'E':
              Heading = 'N';
              break;          
            case 'S':
              Heading = 'E';
              break;            
            case 'W':
              Heading = 'S';
          }          
        } else if (seq[i+3] == 'R') {
          left_position += POSITION_STEP_ROTATION;
          right_position -= POSITION_STEP_ROTATION;
          switch (Heading){
            case 'N':
              Heading = 'E';
              break;              
            case 'E':
              Heading = 'S';
              break;            
            case 'S':
              Heading = 'W';
              break;             
            case 'W':
              Heading = 'N';
          }
        } else {
          break;
        }        
        i++;
        wb_motor_set_position(left_motor, left_position);
        wb_motor_set_position(right_motor, right_position);
      }
      tempL = left_encoder_value;
      tempR = right_encoder_value;
  };
  wb_robot_cleanup();
  printf("Done! - Path plan executed.\n");
  return 0;
}
