/*
 * File:          z5206447_MTRN4110_PhaseA.c
 * Date:          21/06/2020
 * Description:   MTRN4110 Phase A
 * Author:        Patrick Pho z5206447
 * Modifications:
 */

/*
 * You may need to add include files like <webots/distance_sensor.h> or
 * <webots/motor.h>, etc.
 */
#include <webots/robot.h>
#include <webots/distance_sensor.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/inertial_unit.h>


#include <stdio.h>
#include <time.h>
#include <math.h>

/*
 * You may want to add macros here.
 */
#define TIME_STEP 64

#define PATH_PLAN_FILE_NAME "../../PathPlan.txt"
#define MAXPATH 100

#define MAX_SPEED 6.28
#define PI 3.14159265358979323846
#define Forward_Position 33/4.0
//#define Turn_Position 0.0566*PI/(0.02)
#define Turn_Position 0.0566*PI/0.02 + 1.43


// Function for making robot move forwards 1 cell
void move_forwards(WbDeviceTag move_tags[]) {
  WbDeviceTag left_motor = move_tags[0];
  WbDeviceTag right_motor = move_tags[1];
  WbDeviceTag left_sensor = move_tags[2];
  WbDeviceTag right_sensor = move_tags[3];
  //WbDeviceTag inertial_unit = move_tags[4];

  const double DELTA = 0.01;
  // Turn on position sensor
  wb_position_sensor_enable(left_sensor, TIME_STEP);
  wb_position_sensor_enable(right_sensor, TIME_STEP);
  while (wb_robot_step(TIME_STEP) == -1) {
    continue;
  }

  // Get Desired positions
  double left_desired = wb_position_sensor_get_value(left_sensor) + Forward_Position;
  double right_desired = wb_position_sensor_get_value(right_sensor) + Forward_Position;
  // Set Motor to forwards
  //printf("%f %f \n", wb_position_sensor_get_value(left_sensor), wb_position_sensor_get_value(right_sensor));
  wb_motor_set_position(left_motor, left_desired);
  wb_motor_set_position(right_motor, right_desired);

  double left_actual = 0;
  double right_actual = 0;

  do {
    if (wb_robot_step(TIME_STEP) == -1) {
      break;
    }
    // Get actual positions
    left_actual = wb_position_sensor_get_value(left_sensor);
    right_actual = wb_position_sensor_get_value(right_sensor);

  } while (fabs(left_desired - left_actual) > DELTA && fabs(right_desired - right_actual) > DELTA);
  // Turn Off to save processing Time
  wb_position_sensor_disable(left_sensor);
  wb_position_sensor_disable(right_sensor);

}


// Find the new pole from a current pole and direction of turning
void find_new_pole(char *pole, char direction) {
  char poles[4] = {'N', 'E', 'S', 'W'};
  int i = 0;
  while ((*pole != poles[i]) && (i < 4)) {
    i++;
  }
  if (direction == 'F' ) {
    // Do nothing
  } else if (i == 0 && direction == 'L') {
    *pole = poles[3];
  } else if (i == 3 && direction == 'R') {
    *pole = poles[0];
  } else {
    *pole = poles[i + ((direction == 'R') - (direction == 'L'))];
  }
}

// Convert pole to rad values
double pole_to_rad(char pole){
  double desired_dir = 0;
  switch (pole) {
    case 'N':
      desired_dir = 0;
      break;
    case 'E':
      desired_dir = -PI/2.0;
      break;
    case 'S':
      desired_dir = PI;
      break;
    default:
      desired_dir = PI/2.0;
      break;
  }
  return desired_dir;
}


// Function for find the desired dir in rads from current dir rads
double find_desired_dir(char direction, char *pole) {
  // Find Desired pole
  find_new_pole(pole, direction);

  // Convert to radians
  double desired_dir = pole_to_rad(*pole);
  return desired_dir;
}

// Function for making robot turn left or right
void turn(char direction, char *curr_pole, WbDeviceTag move_tags[]) {
  WbDeviceTag left_motor = move_tags[0];
  WbDeviceTag right_motor = move_tags[1];
  WbDeviceTag left_sensor = move_tags[2];
  WbDeviceTag right_sensor = move_tags[3];
  WbDeviceTag inertial_unit = move_tags[4];

  // Turn on sensors
  wb_inertial_unit_enable(inertial_unit, TIME_STEP);
  wb_position_sensor_enable(left_sensor, TIME_STEP);
  wb_position_sensor_enable(right_sensor, TIME_STEP);

  while (wb_robot_step(TIME_STEP) == -1) {
    continue;
  }

  // Find inital motor pos
  double left_position = wb_position_sensor_get_value(left_sensor);
  double right_position = wb_position_sensor_get_value(right_sensor);

  const double *roll_pitch_yaw;
  // Find Desired Dir
  double desired_yaw = find_desired_dir(direction, curr_pole);
  double adjustment = 0.025;
  const double DELTA = 0.025;

  // Keep turning while not at desired_yaw
  do {
    if (wb_robot_step(TIME_STEP) == -1) {
      break;
    }
    // Get curr yaw
    roll_pitch_yaw = wb_inertial_unit_get_roll_pitch_yaw(inertial_unit);

    // Adjust motors to turn slightly
    if (direction == 'L') {
      left_position -= adjustment;
      right_position += adjustment;
    } else {
      left_position += adjustment;
      right_position -= adjustment;
    }

    wb_motor_set_position(left_motor, left_position);
    wb_motor_set_position(right_motor, right_position);

  } while (fabs(fabs(desired_yaw) - fabs(*(roll_pitch_yaw+2))) > DELTA);
  // Turn Off sensors
  wb_inertial_unit_disable(inertial_unit);
  wb_position_sensor_disable(left_sensor);
  wb_position_sensor_disable(right_sensor);
}

void find_new_position(char *curr_pole, int *position) {
  switch(*curr_pole) {
    case 'N':
      position[0] -= 1;
      break;
    case 'E':
      position[1] += 1;
      break;
    case 'W':
      position[1] -= 1;
      break;
    default:
      position[0] += 1;
      break;
  }
}


void move_robot(char *curr_pole, int *position, char movement, WbDeviceTag move_tags[]) {
  if (movement == 'F') {
    move_forwards(move_tags);
    find_new_position(curr_pole, position);
    //find_position(pole);
  } else {
    turn(movement, curr_pole, move_tags);
  }
}

void check_walls(WbDeviceTag dis_sensor[], char *walls) {
  while (wb_robot_step(TIME_STEP) == -1) {
    continue;
  }
  // Get values from sensors
  double Ds_values[3];
  for (int i = 0; i < 3; i++) {
    Ds_values[i] = wb_distance_sensor_get_value(dis_sensor[i]);
  }



  // detect obstacles
  // walls 0 - Left, 1 - Right, 2 - Front
  for (int i = 0; i < 3; i++) {
    if (Ds_values[i] == 50) {
      walls[i] = 'Y';
    } else {
      walls[i] = 'N';
    }
  }
}


int read_file_path(char *path_string) {
  FILE *fp;

  fp = fopen(PATH_PLAN_FILE_NAME, "r");
  if (fp == NULL){
    printf("Could not open file %s", PATH_PLAN_FILE_NAME);
    return -1;
  }
  while (fgets(path_string, MAXPATH, fp) != NULL) {
    printf("%s\n", path_string);
  }
  fclose(fp);
  printf("Done - Path plan read!\n");
  return 0;
}


/*
 * This is the main program.
 * The arguments of the main function can be specified by the
 * "controllerArgs" field of the Robot node
 */
int main(int argc, char **argv) {
  wb_robot_init();

  printf("Start - Read path plan from %s:\n", PATH_PLAN_FILE_NAME);
  char path_string[MAXPATH];
  if (read_file_path(path_string) == -1) {
    return -1;
  }

  printf("Start - Execute path plan!\n");

   // Get devices needed for movement
  WbDeviceTag move_tags[5];
  char move_tag_name[5][25] = {"left wheel motor", "right wheel motor", "left wheel sensor", "right wheel sensor", "inertial unit"};
  for (int i = 0; i < 5; i++) {
    move_tags[i] = wb_robot_get_device(move_tag_name[i]);
  }

  // Initialise zero position
  for (int i = 0; i < 2; i++) {
    wb_motor_set_position(move_tags[i], 0);
  }

  // Get devices for wall sensing
  WbDeviceTag dis_sensor[3];
  char dis_sensor_name[3][5] = {"Ds_L", "Ds_R", "Ds_F"};
  for (int i = 0; i < 3; i++) {
    dis_sensor[i] = wb_robot_get_device(dis_sensor_name[i]);
    wb_distance_sensor_enable(dis_sensor[i], TIME_STEP);
  }

  char walls[3] = {0};
  char curr_dir = path_string[2];
  int position[2] = {path_string[0] - '0', path_string[1] - '0'};

  check_walls(dis_sensor, walls);

  printf("Step: 00, Row: %d, Column: %d, Heading %c, ", position[0], position[1], curr_dir);
  printf("Left wall: %c, Front wall: %c, Right wall: %c\n", walls[0], walls[2], walls[1]);

  for (int i = 1; ((i+2) < MAXPATH) && (path_string[i+2] != 0); i++) {
    // Move Robot
    move_robot(&curr_dir, position, path_string[i+2], move_tags);

    // Find walls
    check_walls(dis_sensor, walls);
    // Print Msg
    printf("Step: %02d, Row: %d, Column: %d, Heading %c, ", i, position[0], position[1], curr_dir);
    printf("Left wall: %c, Front wall: %c, Right wall: %c\n", walls[0], walls[2], walls[1]);
    //printf("Walls, L %d, R %d, F %d \n", walls[0], walls[1], walls[2]);
    //printf("Finished Movement\n");
  }

  wb_robot_cleanup();
  return 0;
}
