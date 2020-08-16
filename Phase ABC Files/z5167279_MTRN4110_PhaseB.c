/*
 * File:          z5167279_MTRN4110_PhaseB.c
 * Date:          11/07/2020
 * Description:
 * Author:        Xinning Guo
 * Modifications:
 * Hard-coding:   The map is hard-coded.
 */

/*
 * You may need to add include files like <webots/distance_sensor.h> or
 * <webots/motor.h>, etc.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <webots/robot.h>

#define MAP_FILE_NAME "C:/Users/g2754/Desktop/Map4.txt"
#define PATH_PLAN_FILE_NAME "C:/Users/g2754/Documents/A/20T2/4110/PhaseB/z5167279_MTRN4110_PhaseB/PathPlanFound.txt"
#define TIME_STEP 64
// add these two define variables for re-submission
// layernum and pathnum in the code are the only things changed 
#define layernum 2000
#define pathnum 100

/*
 * This is the main program.
 * The arguments of the main function can be specified by the
 * "controllerArgs" field of the Robot node
 */
int main(int argc, char **argv) {
  /* necessary to initialize webots stuff */
  wb_robot_init();


  // load the map txt file
  printf("Start - Reading map:\n");
  int x = 0;
  int y = 0;
  int length;
  char map[100];
  char *maze[100];
  int mazeNum[5][9];
  int mazeHwall[6][9] = {0};
  int mazeVwall[5][10] = {0};
  FILE *file = fopen(MAP_FILE_NAME, "r");

  while(fgets(map, 100, file) != NULL) {
      length = strlen(map);
      maze[x] = malloc(length);
      for (y = 0; y < (length-1); y++) {
          maze[x][y] = map[y];
          printf("%c",maze[x][y]);
      }
      printf("\n");
      x++;
  }
  
  fclose(file);
  
  if (map != NULL) {
      printf("Done - Map read!\n");
      printf("Start - Finding shortest path:\n");
  }
  
  
  // find the initial location and heading of the robot
  int r = 0;
  int c = 0;
  char *space = " ";
  char *vwall = "|";
  char *hwall = "-";
  char *S = "v";
  char *N = "^";
  char *W = "<";
  char *E = ">";
  char *south = "S";
  char *north = "N";
  char *west = "W";
  char *east = "E";
  
  int robotfound = 0;
  while ((r < 11) & (robotfound == 0)) {
    while ((c < 37) & (robotfound == 0)){
      if ((maze[r][c] != *space) & (maze[r][c] != *hwall) & (maze[r][c] != *vwall)){
        robotfound = 1;
      } else {
        c++;
      }
    }
    if (robotfound == 0) {
      c = 0;
    }
    r++;
  }
  r = r - 1; 
  
  int rRobot = r;
  int cRobot = c;
  int row = r/2;
  int col = c/4;
  int rowRobot = row;
  int colRobot = col;
  char robotHeading = 0;

  
  if (maze[rRobot][cRobot] == *S) {
    robotHeading = *south;
  } else if (maze[rRobot][cRobot] == *N) {
    robotHeading = *north;
  } else if (maze[rRobot][cRobot] == *W) {
    robotHeading = *west;
  } else if (maze[rRobot][cRobot] == *E) {
    robotHeading = *east;
  }
  


  // flood-fill
  row = 0;
  while (row < 5) {
    for (col = 0; col < 9; col++) {
      mazeNum[row][col] = 45;
    }
    row++;
  }

  
  r = 0;
  c = 2;
  while (r <= 11) {
    while (c < 37){
      if (maze[r][c] == *hwall){
        row = r/2;
        col = c/4;
        mazeHwall[row][col] = 1;
      }
      c = c + 4;
      }
    r = r + 2;
    c = 2;
  }
  
  r = 1;
  c = 0;
  while (r < 11) {
    while (c < 37){
      if (maze[r][c] == *vwall){
        row = r/2;
        col = c/4;
        mazeVwall[row][col] = 1;
      }
      c = c + 4;
      }
    r = r + 2;
    c = 0;
  }
  
  r = 5;
  c = 18;
  row = r/2;
  col = c/4;
  mazeNum[row][col] = 0;

  int CurrentExploredValue = 0;
  int MazeValueChanged = 1;
  while (MazeValueChanged != 0){
    MazeValueChanged = 0;
    row = 0;
    col = 0;
    while (row<5){
      while(col<9){
        if (mazeNum[row][col] == CurrentExploredValue){
          if (mazeHwall[row][col] == 0) {
            if (mazeNum[row-1][col] > mazeNum[row][col]) {
              mazeNum[row-1][col] = CurrentExploredValue + 1;
              MazeValueChanged = 1;
            }
          }
          if (mazeHwall[row+1][col] == 0){
            if (mazeNum[row+1][col] > mazeNum[row][col]) {
              mazeNum[row+1][col] = CurrentExploredValue + 1;
              MazeValueChanged = 1;
            }
          }
          if (mazeVwall[row][col] == 0){
            if (mazeNum[row][col-1] > mazeNum[row][col]) {
              mazeNum[row][col-1] = CurrentExploredValue + 1;
              MazeValueChanged = 1;
            }
          }
          if (mazeVwall[row][col+1] == 0){
            if (mazeNum[row][col+1] > mazeNum[row][col]) {
              mazeNum[row][col+1] = CurrentExploredValue + 1;
              MazeValueChanged = 1;    
            }        
          }
        }
        col++;
      }
      row++;
      col=0;
    }
    CurrentExploredValue++;
  }
  
  // find paths

  int i = 0;
  int left = 0;
  int right = 0;
  int up = 0;
  int down = 0;

  int layer = 0;
  int mazePath[layernum][5][9];
  int mazeMarker[2][layernum];

  row = 0;
  col = 0;
  while (layer<layernum){
    while (row<5){
      while (col<9){
        mazePath[layer][row][col] = 45;
        col++;
      }
      row++;
      col = 0;
    }
    layer++;
    row = 0;
    col = 0;
  }
  

    while (row<2){
      while (col<layernum){
        mazeMarker[row][col] = 45;
        col++;
      }
      row++;
      col = 0;
    }

  
  int m = 0;
  int n = 0;
  int k = 0;
  int noPathFlag = 0;
  int counter = 0;
  int repeatLayer = 0;
  int layerChanged = 0;
  
  int step = mazeNum[rowRobot][colRobot];
  row = rowRobot;
  col = colRobot;
  mazePath[0][row][col] = step;

  while (k<mazeNum[rowRobot][colRobot]){

    // scan 'step' in path. find the num of cols and rows
    layer = 0;
    row = 0;
    col = 0;
    while (layer<layernum){
      while (row<5){
        while (col<9){
          if (mazePath[layer][row][col] == step){
              mazeMarker[0][layer] = row;
              mazeMarker[1][layer] = col;
          }
          col++;
        }
        row++;
        col = 0;
      }
      layer++;
      row = 0;
      col = 0;
    }

    // put it back to floodfill
    // scan neighbours (if not, clear whole path; 
    // if 2, insert the whole path in new layer)
    while (col<layernum){
      if (mazeMarker[0][col] != 45){
        i = 0;
        r = mazeMarker[0][col];
        c = mazeMarker[1][col];
        
        if (mazeVwall[r][c] == 0){
          if(mazeNum[r][c-1] == step-1){
            left = 1;
            i++;
          }
        }
        if (mazeVwall[r][c+1] == 0){
          if (mazeNum[r][c+1] == step-1){
            right = 1;
            i++;
          }
        }
        if (mazeHwall[r][c] == 0){
          if (mazeNum[r-1][c] == step-1){
            up = 1;
            i++;
          }
        }
        if (mazeHwall[r+1][c] == 0){
          if (mazeNum[r+1][c] == step-1) {
            down = 1;
            i++;
          }
        }

        if (i == 0){
          while (m<5){
            while (n<9){
              mazePath[col][m][n] = 45;
              n++;
            }
            m++;
            n = 0;
          }
          m = 0;
        }
        
        if (i == 0) {

          printf("No path found.\n");
          noPathFlag = 1;
          break;

        }

        if (i == 1){

          if (left == 1) {
            mazePath[col][r][c-1] = step-1;
          }
          if (right == 1) {
            mazePath[col][r][c+1] = step-1;
          }
          if (up == 1) {
            mazePath[col][r-1][c] = step-1;
          }
          if (down == 1) {
            mazePath[col][r+1][c] = step-1;
          }
        }
        
        if (i == 2){

          counter++;
          
          while (mazeMarker[0][m] != 45){
            m++;
          }
          layer = m;
          if (repeatLayer != 0){
            layer = layer + repeatLayer;
          }
          repeatLayer++;
          
          m = 0;
          while (m<5){
            while (n<9){
              mazePath[layer][m][n] = mazePath[col][m][n];
              n++;
            }
            m++;
            n = 0;
          }
          m = 0;
          
          if (left == 1) {
            mazePath[col][r][c-1] = step-1;
            layerChanged = 1;
          }
          if (right == 1) {
            if (layerChanged == 0) {
              mazePath[col][r][c+1] = step-1;
              layerChanged = 1;
            } else {
              mazePath[layer][r][c+1] = step-1;
            }
          }
          if (up == 1) {
            if (layerChanged == 0) {
              mazePath[col][r-1][c] = step-1;
              layerChanged = 1;
            } else {
              mazePath[layer][r-1][c] = step-1;
            }
          }
          if (down == 1) {
            if (layerChanged == 0) {
              mazePath[col][r+1][c] = step-1;
              layerChanged = 1;
            } else {
              mazePath[layer][r+1][c] = step-1;
            }
          }
          layerChanged = 0;          
          
          
        }
        
      } 
      col++;
      down = 0;
      up = 0;
      left = 0;
      right = 0;
    }
    
    // clean up variables
    step--;
    row = 0;
    col = 0;
    repeatLayer = 0;
    while (row<2){
      while (col<layernum){
        mazeMarker[row][col] = 45;
        col++;
      }
      row++;
      col = 0;
    }    

    k++;
  }
        
  // display all the paths
  
  if (noPathFlag == 1) {

  } else {
    m = 0;
    n = 0;
    layer = 0;
    int mazeDisplay[counter+1][11][37];
    while (layer<=counter) {
      while (m<11){
        while (n<37){
          mazeDisplay[layer][m][n] = maze[m][n]; 
          n++;
        }
        m++;
        n = 0;
      }
      m = 0;
      n = 0;
      layer++;
    }
  
  
  m = 0;
  layer = 0;     
  while (layer <= counter){
    printf("path - %d:\n",layer+1);
    while (m<5){
      while (n<9){
        if ((mazePath[layer][m][n] != 45)&(mazePath[layer][m][n] != mazeNum[rowRobot][colRobot])){
          r = m*2+1;
          c = n*4+2;
          mazeDisplay[layer][r][c] = mazePath[layer][m][n] + 48;
          
          if (mazeDisplay[layer][r][c] >= 88){
            mazeDisplay[layer][r][c+1] = mazeDisplay[layer][r][c] - 40;
            mazeDisplay[layer][r][c] = 52;
          }
          if (mazeDisplay[layer][r][c] >= 78){
            mazeDisplay[layer][r][c+1] = mazeDisplay[layer][r][c] - 30;
            mazeDisplay[layer][r][c] = 51;
          }
          if (mazeDisplay[layer][r][c] >= 68){
            mazeDisplay[layer][r][c+1] = mazeDisplay[layer][r][c] - 20;
            mazeDisplay[layer][r][c] = 50;
          }
          if (mazeDisplay[layer][r][c] >= 58){
            mazeDisplay[layer][r][c+1] = mazeDisplay[layer][r][c] - 10;
            mazeDisplay[layer][r][c] = 49;
          }
        }
        n++;
      }
      m++;
      n = 0;
    }

     x = 0;
     y = 0;
     while (x<11){
       while (y<37){
         printf("%c", mazeDisplay[layer][x][y]);
         y++;
       }
       printf("\n");
       x++;
       y = 0;
     }
     
     m = 0;
     layer++;
  }
  
  printf("Done - %d shortest paths found!\n",counter+1);
  printf("Start - Finding shortest path with least turns:\n");
  
  // find the least turns path
  
  int turn;
  int prevTurn = 100;
  
  int heading = 0;
  char direction[pathnum] = {0};
  char prevDirection[pathnum] = {0};
  char *forward = "F";
  char *rightTurn = "R";
  char *leftTurn = "L";
  
  int stepCounter = 0;
  int prevStep = 100;
  
  int shortest = 0;
  layer = 0;

  while (layer<=counter) {
    
    if (maze[rRobot][cRobot] == *S) {
      heading = 4;
    } else if (maze[rRobot][cRobot] == *N) {
      heading = 2;
    } else if (maze[rRobot][cRobot] == *W) {
      heading = 1;
    } else if (maze[rRobot][cRobot] == *E) {
      heading = 3;
    }
    
    row = rowRobot;
    col = colRobot;    
    step = mazeNum[row][col];

    
    while (step != 0){
      if (mazePath[layer][row][col+1] == step-1) {
        col++;
        stepCounter++;
        if (heading != 3) {
          if (heading == 2) {
            direction[stepCounter-1] = *rightTurn;
          } else if (heading == 4) {
            direction[stepCounter-1] = *leftTurn;
          } else if (heading == 1) {
            turn++;
            stepCounter++;
            direction[stepCounter-2] = *rightTurn;
            direction[stepCounter-1] = *rightTurn;
          }
          direction[stepCounter] = *forward;
          turn++;
          stepCounter++;
          heading = 3;
        } else {
          direction[stepCounter-1] = *forward;
        }
      } else if (mazePath[layer][row][col-1] == step-1) {
        col--;
        stepCounter++;
        if (heading != 1) {
          if (heading == 2) {
            direction[stepCounter-1] = *leftTurn;
          } else if (heading == 4) {
            direction[stepCounter-1] = *rightTurn;
          } else if (heading == 3) {
            turn++;
            stepCounter++;
            direction[stepCounter-2] = *rightTurn;
            direction[stepCounter-1] = *rightTurn;
          }
          direction[stepCounter] = *forward;
          turn++;
          stepCounter++;
          heading = 1;
        } else {
          direction[stepCounter-1] = *forward;
        }
      } else if (mazePath[layer][row-1][col] == step-1) {
        row--;
        stepCounter++;
        if (heading != 2) {
          if (heading == 1) {
            direction[stepCounter-1] = *rightTurn;
          } else if (heading == 3) {
            direction[stepCounter-1] = *leftTurn;
          } else if (heading == 4) {
            turn++;
            stepCounter++;
            direction[stepCounter-2] = *rightTurn;
            direction[stepCounter-1] = *rightTurn;
          }
          direction[stepCounter] = *forward;
          turn++;
          stepCounter++;
          heading = 2;
        } else {
          direction[stepCounter-1] = *forward;
        }
      } else if (mazePath[layer][row+1][col] == step-1){
        row++;
        stepCounter++;
        if (heading != 4) {
          if (heading == 1) {
            direction[stepCounter-1] = *leftTurn;
          } else if (heading == 3) {
            direction[stepCounter-1] = *rightTurn;
          } else if (heading == 2) {
            turn++;
            stepCounter++;
            direction[stepCounter-2] = *rightTurn;
            direction[stepCounter-1] = *rightTurn;
          }
          direction[stepCounter] = *forward;
          turn++;
          stepCounter++;
          heading = 4;
        } else {
          direction[stepCounter-1] = *forward;
        } 
      } 
      step--;
    }
      
    if (turn <= prevTurn) {
      for(i=0;i<pathnum;i++){
        prevDirection[i] = 0;
      }
      for(i=0;i<pathnum;i++){
        prevDirection[i] = direction[i];
      }
      prevTurn = turn;
      prevStep = stepCounter;
      shortest = layer;
    }

    turn = 0;
    stepCounter = 0;
    for(i=0;i<pathnum;i++){
        direction[i] = 0;
    }
    layer++;
    
    }
   
    x = 0;
    y = 0;
    while (x<11){
      while (y<37){
        printf("%c", mazeDisplay[shortest][x][y]);
        y++;
      }
      printf("\n");
      x++;
      y = 0;
    }
  
    
    printf("steps: %d - %d%d%c%s\n", prevStep,rowRobot,colRobot,robotHeading,prevDirection);
    printf("Done - Shortest path with least turns found!\n");
    printf("Start - Writing path plan to PathPlanFound.txt\n");
    
    direction[0] = rowRobot + 48;
    direction[1] = colRobot + 48;
    direction[2] = robotHeading;
    for (i=0;i<prevStep;i++) {
      direction[i+3] = prevDirection[i];
    }

    FILE *fp = fopen(PATH_PLAN_FILE_NAME, "w");
 
    fprintf (fp, direction);

    fclose (fp);
  
    printf("Done - Path plan written to PathPlanFound.txt\n");

  
  }

  

  
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

    /* Process sensor data here */

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
