/*
 * File:          z5206447_MTRN4110_PhaseB.c
 * Date:
 * Description:
 * Author:
 * Modifications:
 */
#include <webots/robot.h>
#include <webots/distance_sensor.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/inertial_unit.h>
#include <webots/keyboard.h>


#include <stdio.h>
#include <string.h>
#include <Windows.h>
#include <conio.h>

#include <time.h>
#include <math.h>

#define MAXTEXTCHAR 39
#define MAP_HEIGHT 5
#define MAP_WIDTH 36 // Last Vertical wall
#define center_row 2
#define center_col 4
#define row_len 5
#define col_len 9
#define MAXPATHSIZE 45
#define MAXPATHNO 2000

#define TIME_STEP 64
#define MAX_SPEED 6.28
#define PI 3.14159265358979323846
#define Forward_Position 33/4.0
//#define Turn_Position 0.0566*PI/(0.02)
#define Turn_Position 0.0566*PI/0.02 + 1.43

typedef struct {
  int h_walls[row_len+1][col_len];
  int v_walls[row_len][col_len+1];
  int cell_value[row_len][col_len];
  int curr_row;
  int curr_col;
  char pole;
} Map;

typedef struct {
  int row_locs[MAXPATHSIZE];
  int col_locs[MAXPATHSIZE];
  int value[MAXPATHSIZE];
  int counter;
} Stack;

typedef struct {
  int row_locs[MAXPATHNO][MAXPATHSIZE];
  int col_locs[MAXPATHNO][MAXPATHSIZE];
  int value[MAXPATHNO][MAXPATHSIZE];
  int counter;
} SolutionArray;

typedef struct {
  char PathStrings[MAXPATHNO][MAXPATHSIZE * 2];
  int PathLength[MAXPATHNO];
  int counter;
  int ShortestPathIndex;
} PathStringArray;



void FillAllMaze (Map *Maze, int value) {
  for (int i = 0; i < row_len; i++) {
    for (int j = 0; j < col_len; j++) {
      Maze->cell_value[i][j] = value;
    }
  }
  Maze->cell_value[center_row][center_col] = 0;
}

void SetEmptyMaze(Map *Maze) {
  // Set top and bottom h_walls
  for (int i = 0; i < col_len; i++) {
    Maze->h_walls[0][i] = 1;
    Maze->h_walls[5][i] = 1;
  }
  // Set left and right most v_walls
  for (int i = 0; i < row_len; i++) {
    Maze->v_walls[i][0] = 1;
    Maze->v_walls[i][col_len] = 1;
  }

}

void SetInitalLocation(Map *Maze, int row, int col, char pole) {
  Maze->curr_row = row;
  Maze->curr_col = col;
  Maze->pole = pole;
}

int Possiblepath(int i, int j, char pole, Map *Maze) {
  int row_modifier = 0;
  int col_modifier = 0;
  // Change coords if these poles
  if (pole == 'S') {
    row_modifier += 1;
  }
  if (pole == 'E') {
    col_modifier += 1;
  }

  // Check for walls
  int wall = 0;
  if (pole == 'S' || pole == 'N') {
    wall = (Maze->h_walls[i+row_modifier][j+col_modifier] == 1);
  } else {
    wall = (Maze->v_walls[i+row_modifier][j+col_modifier] == 1);
  }
  return !wall;
}

int get_new_row (int row, char pole) {
  int new_row;
  switch (pole) {
    case 'N':
      new_row = row - 1;
      break;
    case 'S':
      new_row = row + 1;
      break;
    default:
      new_row = row;
  }
  return new_row;
}

int get_new_col (int col, char pole) {
  int new_col;
  switch (pole) {
    case 'E':
      new_col = col + 1;
      break;
    case 'W':
      new_col = col - 1;
      break;
    default:
      new_col = col;
  }
  return new_col;
}

int Possiblepathvalue (int row, int col, char pole, Map *Maze) {
  int row_neighbour = get_new_row(row, pole);
  int col_neighbour = get_new_col(col, pole);
  return Maze->cell_value[row_neighbour][col_neighbour];
}

void Fillpossiblepath(int row, int col, char pole, Map *Maze, int CurrentExploredValue) {
  if (Possiblepathvalue(row, col, pole, Maze) == 45) {
    int row_neighbour = get_new_row(row, pole);
    int col_neighbour = get_new_col(col, pole);
    Maze->cell_value[row_neighbour][col_neighbour] = CurrentExploredValue + 1;
  }
}

void Floodfill(Map *Maze) {
  int CurrentExploredValue = 0;
  int MazeValueChanged = 1;
  char poles[4] = {'N', 'E', 'S', 'W'};
  while (MazeValueChanged) {
    MazeValueChanged = 0;
    for (int i = 0; i < row_len; i++) {
      for (int j = 0; j < col_len; j++) {
        if (Maze->cell_value[i][j] == CurrentExploredValue) {
          for (int k = 0; k < 4; k++) {
            // No neighbour wall and cell = 45
            if (Possiblepath(i, j, poles[k], Maze)) {
              Fillpossiblepath(i, j, poles[k], Maze, CurrentExploredValue);
              MazeValueChanged = 1;
            }
          }
        }
      }
    }
    CurrentExploredValue++;
  }
}

// Display Map
void Displaycellvalues (Map *Maze) {
  char dir;
  for (int i = 0; i < row_len; i++) {
    // Print the horizontal walls
    for (int j = 0; j < col_len; j++) {
      printf(" ");
      if (Maze->h_walls[i][j] == 1) {
        printf("---");
      } else {
        printf("   ");
      }
    }

    printf("\n");
    // Prints the vertical walls and step numbers
    for (int j = 0; j < col_len; j++) {
      if (Maze->v_walls[i][j] == 1) {
        printf("|");
      } else {
        printf(" ");
      }
      if (Maze->cell_value[i][j] != 45) {
        if ((i == Maze->curr_row) && (j == Maze->curr_col)) {
          switch (Maze->pole) {
            case 'N':
              dir = '^';
              break;
            case 'E':
              dir = '>';
              break;
            case 'S':
              dir = 'v';
              break;
            default:
              dir = '<';
              break;
          }
          printf(" %c ", dir);
        } else {
          printf(" %-2d", Maze->cell_value[i][j]);
        }
      } else {
        printf("   ");
      }
    }
    // Print Rightmost vertical wall
    if (Maze->v_walls[i][col_len] == 1) {
      printf("|");
    } else {
      printf(" ");
    }
    printf("\n");
  }

  // Print Last horizontal wall
  for (int j = 0; j < col_len; j++) {
    printf(" ");
    if (Maze->h_walls[row_len][j] == 1) {
      printf("---");
    } else {
      printf("   ");
    }
  }
  printf("\n");
}

int startingvalue (Map *Maze) {
  return Maze->cell_value[Maze->curr_row][Maze->curr_col];
}

void InitialiseStack (Stack *StackPath) {
  for (int i = 0; i < MAXPATHSIZE; i++) {
      StackPath->row_locs[i] = -1;
      StackPath->col_locs[i] = -1;
      StackPath->value[i] = -1;
  }
  StackPath->counter = 0;
}

void InitialisePathSolutions (SolutionArray *Maze_solutions) {
  for (int i = 0; i < MAXPATHNO; i++) {
    for (int j = 0; j < MAXPATHSIZE; j++) {
      Maze_solutions->row_locs[i][j] = -1;
      Maze_solutions->col_locs[i][j] = -1;
      Maze_solutions->value[i][j] = -1;
    }
  }
  Maze_solutions->counter = 0;
}

void PushStack(Stack *StackPath, int row, int col, int value) {
  StackPath->row_locs[StackPath->counter] = row;
  StackPath->col_locs[StackPath->counter] = col;
  StackPath->value[StackPath->counter] = value;
  StackPath->counter += 1;
}

void PopStack(Stack *StackPath) {
  int count = StackPath->counter;
  StackPath->row_locs[count] = -1;
  StackPath->col_locs[count] = -1;
  StackPath->value[count] = -1;
  StackPath->counter -= 1;
}

int IsStackEmpty (Stack *StackPath) {
  return (StackPath->value[0] == -1);
}

void AddPathToSolutions(Stack *StackPath, SolutionArray *Maze_solutions) {
  int SolutionNo = Maze_solutions->counter;
  //printf("Solution No: %d ", SolutionNo);
  for (int i = 0; i < StackPath->counter; i++) {
    Maze_solutions->row_locs[SolutionNo][i] = StackPath->row_locs[i];
    Maze_solutions->col_locs[SolutionNo][i] = StackPath->col_locs[i];
    Maze_solutions->value[SolutionNo][i] = StackPath->value[i];
    //printf("(%d, %d, %d), ", StackPath->row_locs[i], StackPath->col_locs[i], StackPath->value[i]);
  }
  //printf("\n");
  Maze_solutions->counter++;
}

void FindPaths (int row, int col, int visited[row_len][col_len], Map *Maze, Stack *StackPath, SolutionArray *Maze_solutions, int CurrentExploredValue) {
  // Add current cell to stack path
  PushStack(StackPath, row, col, CurrentExploredValue);

  // If found the target destination add that path
  if (CurrentExploredValue == 0) {
    AddPathToSolutions(StackPath, Maze_solutions);
    //return;
  }

  int next_row = row;
  int next_col = col;
  char poles[4] = {'N', 'E', 'S', 'W'};

  // For each direction from current cell
  for (int i = 0; i < 4; i++) {
    // If can go that dir
    //printf("%d, %d, %c \n", row, col, poles[i]);
    if (Possiblepath(row, col, poles[i], Maze)) {
      //printf("possible\n");
      next_row = get_new_row(row, poles[i]);
      next_col = get_new_col(col, poles[i]);
      // If correct CurrentExploredValue
      if (Possiblepathvalue(row, col, poles[i], Maze) == CurrentExploredValue - 1) {
        // If not visited yet
        if (visited[next_row][next_col] == 0) {
          // Set visited
          visited[row][col] = 1;
          // Search Along path
          FindPaths(next_row, next_col, visited, Maze, StackPath, Maze_solutions, CurrentExploredValue - 1);
          // After finished searching mark it unvisited
          visited[row][col] = 0;
        }
      }
    }
  }
  // Remove current cell from path
  PopStack(StackPath);
}

void FindPossibleSolutions (SolutionArray *Maze_solutions, Stack *StackPath, Map *Maze) {
  int curr_row = Maze->curr_row;
  int curr_col = Maze->curr_col;
  int CurrentExploredValue = startingvalue(Maze);
  int visited[5][9] = {0};

  FindPaths(curr_row, curr_col, visited, Maze, StackPath, Maze_solutions, CurrentExploredValue);
}

char FindTurn (int curr_dir, int new_dir) {
  char turn = 'F';
  // If not change dir
  //printf("\n%d %d\n", curr_dir, new_dir);
  if (curr_dir == new_dir) {
    turn = 'F';
  } else if (abs(curr_dir - new_dir) == 2) {
    turn = 'X';
  // If at North and going West
  } else if ((curr_dir == 0) && (new_dir == 3)) {
    turn = 'L';
  // If going from west to north
  } else if ((curr_dir == 3) && (new_dir == 0)) {
    turn = 'R';
  } else {
    if (curr_dir > new_dir) {
      turn = 'L';
    } else {
      turn = 'R';
    }
  }
  return turn;
}

// Converts Maze_solutions to StringPath, from array of row/cols to the string
void FindPathStrings (PathStringArray *StringPath, SolutionArray *Maze_solutions, Map *Maze) {
  int curr_dir = 0;

  // Convert curr direction to number notation
  int poles[4] = {'N', 'E', 'S', 'W'};
  for (int i = 0; i < 4; i++) {
    if (poles[i] == Maze->pole) {
        curr_dir = i;
    }
  }
  int start_dir = curr_dir;
  int new_dir = curr_dir;
  char turn = 'L';
  int StringNo = 0;
  int row_diff = 0;
  int col_diff = 0;
  StringPath->counter = 0;
  // For each valid soln
  for (int i = 0; i < Maze_solutions->counter; i++) {
    // Reset Variables
    StringNo = 3;
    curr_dir = start_dir;
    // Get first 3 chars
    StringPath->PathStrings[i][0] = *Maze_solutions->row_locs[0] + '0';
    StringPath->PathStrings[i][1] = *Maze_solutions->col_locs[0] + '0';
    StringPath->PathStrings[i][2] = Maze->pole;
    // For rest of the path
    for (int j = 1; j < startingvalue(Maze); j++) {
      // Get the movements for each change it path and increment movement no
      row_diff = Maze_solutions->row_locs[i][j] - Maze_solutions->row_locs[i][j-1];
      col_diff = Maze_solutions->col_locs[i][j] - Maze_solutions->col_locs[i][j-1];
      // Find new direction
      if (row_diff == 0) {
        if (col_diff < 0) {
          new_dir = 3;
        } else {
          new_dir =  1;
        }
      }
      if (col_diff == 0) {
        if (row_diff < 0) {
          new_dir = 0;
        } else {
          new_dir = 2;
        }
      }
      // Find which direction and add it if turning
      turn = FindTurn(curr_dir, new_dir);
      if  (turn != 'F') {
        // Print R twice if turning 180degs
        if (turn == 'X') {
          StringPath->PathStrings[i][StringNo] = 'R';
          StringNo++;
          StringPath->PathStrings[i][StringNo] = 'R';
          StringNo++;
        } else {
          StringPath->PathStrings[i][StringNo] = turn;
          StringNo++;
        }
      }
      // Print the forward movement
      StringPath->PathStrings[i][StringNo] = 'F';
      StringNo++;
      // Update current direction
      curr_dir = new_dir;
    }
    // Add newline at the end
    StringPath->PathStrings[i][StringNo++] = 'F';
    StringPath->PathStrings[i][StringNo++] = '\n';
    StringPath->PathLength[i] = StringNo;
  }
  StringPath->counter += 1;
}

void InitalisePathString(PathStringArray *StringPath) {
  StringPath->counter = 0;
}

void FindShortestPathString(PathStringArray *StringPath) {
  StringPath->ShortestPathIndex = 0;
  // Find shortest paths
  int ShortestSolution = MAXPATHSIZE * 2;
  for (int i = 0; i < StringPath->counter; i++) {
    if (StringPath->PathLength[i] < ShortestSolution) {
      ShortestSolution = i;
    }
  }
  StringPath->ShortestPathIndex = ShortestSolution;
}

void PrintShortestPath(PathStringArray *StringPath) {
  printf("Printing Shortest Path\n");
  for (int i = 0; i < StringPath->PathLength[StringPath->ShortestPathIndex]; i++) {
    printf("%c", StringPath->PathStrings[StringPath->ShortestPathIndex][i]);
  }
  //printf("\n");
}

void CopyMap (Map *TempMaze, Map *Maze) {
  for (int i = 0; i < row_len; i++) {
    for (int j = 0; j < col_len; j++) {
      TempMaze->h_walls[i][j] = Maze->h_walls[i][j];
      TempMaze->v_walls[i][j] = Maze->v_walls[i][j];
    }
  }
  for (int j = 0; j < col_len; j++) {
    TempMaze->h_walls[row_len][j] = Maze->h_walls[row_len][j];
  }
  for (int i = 0; i < row_len; i++) {
    TempMaze->v_walls[i][col_len] = Maze->v_walls[i][col_len];
  }
  TempMaze->curr_row = Maze->curr_row;
  TempMaze->curr_col = Maze->curr_col;
  TempMaze->pole = Maze->pole;
}

void ConvertPathstoMazePaths(Map *TempMaze, SolutionArray *Maze_solutions, int PathNo) {
  int row = 0;
  int col = 0;
  int value = 0;

  for (int i = 0; i < MAXPATHSIZE; i++) {
    if (Maze_solutions->value[PathNo][i] != -1) {
      row = Maze_solutions->row_locs[PathNo][i];
      col = Maze_solutions->col_locs[PathNo][i];
      value = Maze_solutions->value[PathNo][i];
      TempMaze->cell_value[row][col] = value;
    }
  }
}

void PrintShortestMazePath (Map *TempMaze, SolutionArray *Maze_solutions, Map *Maze, PathStringArray *StringPath) {
  CopyMap(TempMaze, Maze);
  FillAllMaze(TempMaze, 45);
  ConvertPathstoMazePaths(TempMaze, Maze_solutions, StringPath->ShortestPathIndex);
  Displaycellvalues(TempMaze);
}

void GetMazePath(Map *Maze, Stack *StackPath, SolutionArray *Maze_solutions, Map *TempMaze, PathStringArray *StringPath) {
  // Start Loop
  InitialiseStack(StackPath);
  InitialisePathSolutions(Maze_solutions);
  //InitalisePathString(&StringPath);
  // Initalise Maze Cells for FloodFill
  FillAllMaze(Maze, 45);

  // Fill Map
  Floodfill(Maze);
  //Displaycellvalues(Maze);
  // Finds solutions and puts them into Maze_solutions
  FindPossibleSolutions(Maze_solutions, StackPath, Maze);
  FindPathStrings(StringPath, Maze_solutions, Maze);
  FindShortestPathString(StringPath);
}


// Function for making robot move forwards 1 cell
void move_forwards(WbDeviceTag move_tags[], Map *Maze) {
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

  // Update position
  switch (Maze->pole) {
    case 'N':
      Maze->curr_row -= 1;
      break;
    case 'E':
      Maze->curr_col += 1;
      break;
    case 'W':
      Maze->curr_col -= 1;
      break;
    default:
      Maze->curr_row += 1;
      break;
  }
}

// Very similar to function FindTurn
char find_new_pole(char pole, char Action) {
  char poles[4] = {'N', 'E', 'S', 'W'};
  int i = 0;
  int new_pole = 'N';
  while ((pole != poles[i]) && (i < 4)) {
    i++;
  }
  if (Action == 'F' ) {
    new_pole = pole;
    // Do nothing
  } else if (i == 0 && Action == 'L') {
    new_pole = poles[3];
  } else if (i == 3 && Action == 'R') {
    new_pole = poles[0];
  } else {
    new_pole = poles[i + ((Action == 'R') - (Action == 'L'))];
  }
  return new_pole;
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
double find_desired_dir(char Action, Map *Maze) {
  // Find Desired pole
  Maze->pole = find_new_pole(Maze->pole, Action);

  // Convert to radians
  double desired_dir = pole_to_rad(Maze->pole);
  return desired_dir;
}

// Function for making robot turn left or right
void turn(char Action, WbDeviceTag move_tags[], Map *Maze) {
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
  double desired_yaw = find_desired_dir(Action, Maze);

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
    if (Action == 'L') {
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

void MoveOneStep(char Action, WbDeviceTag move_tags[], Map *Maze) {
  if (Action == 'F') {
    move_forwards(move_tags, Maze);
    //find_position(pole);
  } else {
    turn(Action, move_tags, Maze);
  }
}

void UpdateMap(Map *Maze, WbDeviceTag dis_sensor[]) {
  while (wb_robot_step(TIME_STEP) == -1) {
    continue;
  }
  // Get values from sensors L,R,F
  double Ds_values[3];
  for (int i = 0; i < 3; i++) {
    Ds_values[i] = wb_distance_sensor_get_value(dis_sensor[i]);
  }
  int walls[3];
  for (int i = 0; i < 3; i++) {
    if (Ds_values[i] == 50) {
      walls[i] = 1;
    } else {
      walls[i] = 0;
    }
  }

  char Direction[3] = {'L', 'R', 'F'};
  char New_Direction = 'N';
  int row_modifier = 0;
  int col_modifier = 0;
  // Add Wall at relevant location
  for (int i = 0; i < 3; i++) {
    New_Direction = find_new_pole(Maze->pole, Direction[i]);
    row_modifier = 0;
    col_modifier = 0;
    if (New_Direction == 'S') {
      row_modifier += 1;
    }
    if (New_Direction == 'E') {
      col_modifier += 1;
    }
    if (walls[i]) {
      if (New_Direction == 'S' || New_Direction == 'N') {
        Maze->h_walls[Maze->curr_row+row_modifier][Maze->curr_col+col_modifier] = 1;
      }
      else {
        Maze->v_walls[Maze->curr_row+row_modifier][Maze->curr_col+col_modifier] = 1;
      }
    }
  }
}

char check_keyboard() {
  char action;
  int i = wb_keyboard_get_key();
  switch (i) {
    case WB_KEYBOARD_UP:
      action = 'F';
      break;
    case WB_KEYBOARD_LEFT:
      action = 'L';
      break;
    case WB_KEYBOARD_RIGHT:
      action = 'R';
      break;
    default:
      action = '\0';
      break;
  }
  return action;
}


int main(int argc, char **argv) {

  wb_robot_init();
  // Initalise Planning Stuff
  Map Maze;
  Map TempMaze;
  Stack StackPath;
  SolutionArray Maze_solutions;
  PathStringArray StringPath;
  SetEmptyMaze(&Maze);
  SetInitalLocation(&Maze, 0, 0, 'S');
  char Action;

  // Initalise Moving Stuff
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
  // Initialise keyboard control
  wb_keyboard_enable(TIME_STEP);
  printf("Press UP/LEFT/RIGHT to control manually.\n");
   
  //Start Loop 27 = ESC
  while ((Maze.curr_row != center_row) || (Maze.curr_col != center_col)) {
    printf("--- Finding Optimal Path ---\n");
    GetMazePath(&Maze, &StackPath, &Maze_solutions, &TempMaze, &StringPath);
    PrintShortestMazePath(&TempMaze, &Maze_solutions, &Maze, &StringPath);
    PrintShortestPath(&StringPath);

    Action = check_keyboard();
    if (Action == '\0'){
      Action = StringPath.PathStrings[StringPath.ShortestPathIndex][3];
    }
    printf("Finished Planning\n");

    // Move Based on Path
    printf("--- Moving Based on Path ---\n");
    
    MoveOneStep(Action, move_tags, &Maze);
    printf("Finished Moving\n");


    printf("--- Updating Map ---\n");
    UpdateMap(&Maze, dis_sensor);
    printf("Finished Updating\n");
  }
  printf("Finished Program\n--- Exiting ---\n");
  wb_robot_cleanup();
  return 0;

}
