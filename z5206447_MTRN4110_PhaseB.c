/*
 * File:          z5206447_MTRN4110_PhaseB.c
 * Date:
 * Description:
 * Author:
 * Modifications:
 */

#include <webots/robot.h>
#include <stdio.h>
#include <string.h>

#define TIME_STEP 64

#define MAP_FILE_NAME "../../Map.txt"
#define PATH_PLAN_FILE_NAME "../../PathPlanFound.txt"

#define MAXCHAR 39
#define MAP_HEIGHT 5
#define MAP_WIDTH 36 // Last Vertical wall
#define end_row 2
#define end_col 4
#define row_len 5
#define col_len 9
#define MAXPATHSIZE 25
#define MAXPATHNO 15



typedef struct {
  int h_walls[row_len+1][col_len];
  int v_walls[row_len][col_len+1];
  int cell_value[row_len][col_len];
  int start_row;
  int start_col;
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
} PathStringArray;


void FillAllMaze (Map *Maze, int value) {
  for (int i = 0; i < row_len; i++) {
    for (int j = 0; j < col_len; j++) {
      Maze->cell_value[i][j] = value;
    }
  }
}


int read_map_file(Map *Maze) {
  FILE *fp;
  fp = fopen(MAP_FILE_NAME, "r");
  if (fp == NULL) {
    printf("Could not open file %s", MAP_FILE_NAME);
    return -1;
  }
  char line1[MAXCHAR] = {0};
  char line2[MAXCHAR] = {0};
  int k = 0;
  // Loop through each Maze row
  for (int i = 0; i < MAP_HEIGHT; i++) {
    // Check for walls
    // Horitzontal walls
    fgets(line1, MAXCHAR, fp);
    // Vertical walls
    fgets(line2, MAXCHAR, fp);

    k = 0;
    for (int j = 2; j < (MAXCHAR-2); j+=4) {
      // Check Horitzontal wall
      Maze->h_walls[i][k] = (line1[j] == '-');
      if (i != 5) {
        // Check Vertical wall
        Maze->v_walls[i][k] = (line2[j-2] == '|');
      }
      // Check for start location
      if (line2[j] != ' ') {
        Maze->start_row = i;
        Maze->start_col = k;
        //printf("\n\n%d %d\n\n",i,k);
        switch (line2[j]) {
          case '^':
            Maze->pole = 'N';
            break;
          case '>':
            Maze->pole = 'E';
            break;
          case 'v':
            Maze->pole = 'S';
            break;
          default:
            Maze->pole = 'W';
        }
      }


      //printf("%d %d, ", k, j);
      k++;
    }
    // Get the end right walls
    Maze->v_walls[i][k] = (line2[MAP_WIDTH] == '|');

    // Prints file
    printf("%s", line1);
    if (i != 5) {
      printf("%s", line2);
    }
  }
  // Get bottom end h_walls
  for (int k = 0; k < col_len; k++) {
    Maze->h_walls[row_len][k] = 1;
  }
  fgets(line1, MAXCHAR, fp);
  printf("%s", line1);
  // Fill Cellvalues with 45s
  //memset(Maze->cell_value, 45, row_len * col_len * sizeof(Maze->cell_value[0][0]));
  FillAllMaze(Maze, 45);

  // Fill center with 0
  Maze->cell_value[end_row][end_col] = 0;
  // Do one more Time
  //printf("\nStart: %d, %d", Maze->start_row, Maze->start_col);
  fclose(fp);
  return 0;
}







int possiblepath(int i, int j, char pole, Map * Maze) {
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


int possiblepathvalue (int row, int col, char pole, Map *Maze) {
  int row_neighbour = get_new_row(row, pole);
  int col_neighbour = get_new_col(col, pole);
  return Maze->cell_value[row_neighbour][col_neighbour];
}




void fillpossiblepath(int row, int col, char pole, Map *Maze, int CurrentExploredValue) {
  if (possiblepathvalue(row, col, pole, Maze) == 45) {
    int row_neighbour = get_new_row(row, pole);
    int col_neighbour = get_new_col(col, pole);
    Maze->cell_value[row_neighbour][col_neighbour] = CurrentExploredValue + 1;
  }
}




void floodfill(Map *Maze) {
  printf("working?");
  int CurrentExploredValue = 0;
  int MazeValueChanged = 1;
  char poles[4] = {'N', 'E', 'S', 'W'};
  printf("working");
  while (MazeValueChanged) {
    MazeValueChanged = 0;
    for (int i = 0; i < row_len; i++) {
      for (int j = 0; j < col_len; j++) {
        if (Maze->cell_value[i][j] == CurrentExploredValue) {
          for (int k = 0; k < 4; k++) {
            // No neighbour wall and cell = 45
            if (possiblepath(i, j, poles[k], Maze)) {
              fillpossiblepath(i, j, poles[k], Maze, CurrentExploredValue);
              MazeValueChanged = 1;
            }
          }
        }
      }
    }
    CurrentExploredValue++;
  }


}



int startingvalue (Map *Maze) {
  return Maze->cell_value[Maze->start_row][Maze->start_col];
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
}


/*
void InitialisePathSolutions (SolutionArray *Maze_solutions) {
  for (int i = 0; i < )
}*/

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
    if (possiblepath(row, col, poles[i], Maze)) {
      next_row = get_new_row(row, poles[i]);
      next_col = get_new_col(col, poles[i]);
      // If correct CurrentExploredValue
      if (possiblepathvalue(row, col, poles[i], Maze) == CurrentExploredValue - 1) {
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




void find_possible_solutions (SolutionArray *Maze_solutions, Stack *StackPath, Map *Maze) {
  int curr_row = Maze->start_row;
  int curr_col = Maze->start_col;
  int CurrentExploredValue = startingvalue(Maze);
  int visited[5][9] = {0};

  FindPaths(curr_row, curr_col, visited, Maze, StackPath, Maze_solutions, CurrentExploredValue);
}

// Copy Map Structure to another Map
void CopyMap (Map *ShortestPathMaze, Map *Maze) {
  for (int i = 0; i < row_len; i++) {
    for (int j = 0; j < col_len; j++) {
      ShortestPathMaze->h_walls[i][j] = Maze->h_walls[i][j];
      ShortestPathMaze->v_walls[i][j] = Maze->v_walls[i][j];
    }
  }
  for (int j = 0; j < col_len; j++) {
    ShortestPathMaze->h_walls[row_len][j] = Maze->h_walls[row_len][j];
  }
  for (int i = 0; i < row_len; i++) {
    ShortestPathMaze->v_walls[i][col_len] = Maze->v_walls[i][col_len];
  }
  ShortestPathMaze->start_row = Maze->start_row;
  ShortestPathMaze->start_col = Maze->start_col;
  ShortestPathMaze->pole = Maze->pole;

}

// Display Map
void displaycellvalues (Map *Maze) {
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
        if ((i == Maze->start_row) && (j == Maze->start_col)) {
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
  //printf("Finished Displaying");
}

void ConvertPathstoMazePaths(Map *ShortestPathMaze, SolutionArray *Maze_solutions, int PathNo) {
  int row = 0;
  int col = 0;
  int value = 0;

  for (int i = 0; i < MAXPATHSIZE; i++) {
    if (Maze_solutions->value[PathNo][i] != -1) {
      row = Maze_solutions->row_locs[PathNo][i];
      col = Maze_solutions->col_locs[PathNo][i];
      value = Maze_solutions->value[PathNo][i];
      ShortestPathMaze->cell_value[row][col] = value;
      //printf(":%d %d %d\n", row, col, value);
    }
  }
}


void GetMazePaths (Map *ShortestPathMaze, SolutionArray *Maze_solutions, Map *Maze) {

  CopyMap(ShortestPathMaze, Maze);
  for (int i = 0; i < Maze_solutions->counter; i++) {
    printf("--- Path %d ---\n", i+1);
    FillAllMaze(ShortestPathMaze, 45);
    ConvertPathstoMazePaths(ShortestPathMaze, Maze_solutions, i);
    displaycellvalues(ShortestPathMaze);
  }

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

void FindPathStrings (PathStringArray *StringPath, SolutionArray *Maze_solutions, Map *Maze, int PathLength) {
  int curr_dir = 0;

  // Convert curr dif to number notation
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
  //printf("Print Solns: \n");
  // For each valid soln
  for (int i = 0; i < Maze_solutions->counter; i++) {
    // Reset Variables
    StringNo = 3;
    curr_dir = start_dir;
    //printf("start dir: %d ||", curr_dir);
    // Get first 3 chars
    StringPath->PathStrings[i][0] = *Maze_solutions->row_locs[0] + '0';
    StringPath->PathStrings[i][1] = *Maze_solutions->col_locs[0] + '0';
    StringPath->PathStrings[i][2] = Maze->pole;
    //printf(": %d :", curr_dir);
    // For rest of the path
    for (int j = 1; j < PathLength; j++) {
      // Get the movements for each change it path and increment movement no
      row_diff = Maze_solutions->row_locs[i][j] - Maze_solutions->row_locs[i][j-1];
      col_diff = Maze_solutions->col_locs[i][j] - Maze_solutions->col_locs[i][j-1];
      //printf("r %d, c %d ", row_diff, col_diff);
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
      //printf(",%d %d,", curr_dir, new_dir);
      // Find which direction and add it if turning
      //printf(": %d :", curr_dir);
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

      //printf("F");
      curr_dir = new_dir;
    }
    // Add newline at the end
    StringPath->PathStrings[i][StringNo++] = 'F';
    StringPath->PathStrings[i][StringNo++] = '\n';
    StringPath->PathLength[i] = StringNo;
    //printf("F\n");

  }
  //printf("Finished Printing");
}

int WritePathStringToTxtFile (PathStringArray *StringPath) {
  FILE *fp;
  fp = fopen(PATH_PLAN_FILE_NAME, "w");
  if (fp == NULL) {
    printf("Could not write to file %s", PATH_PLAN_FILE_NAME);
    //return -1;
  }
  // Find shortest paths
  int ShortestSolution = MAXPATHSIZE * 2;
  for (int i = 0; i < 6; i++) {
    if (StringPath->PathLength[i] < ShortestSolution) {
      ShortestSolution = i;
    }

  }
  //printf("Writing: %s\n", StringPath->PathStrings[ShortestSolution]);
  fprintf(fp, StringPath->PathStrings[ShortestSolution]);
  return ShortestSolution;
}


int main(int argc, char **argv) {

  wb_robot_init();
  printf("--- Task 1 ---\n");
  Map Maze;
  Map ShortestPathMaze;
  Stack StackPath;
  SolutionArray Maze_solutions;
  PathStringArray StringPath;

  InitialisePathSolutions(&Maze_solutions);
  StackPath.counter = 0;
  Maze_solutions.counter = 0;


  if (read_map_file(&Maze) == -1) {
    return -1;
  }
  printf("\n--- Task 2 ---\n");
  floodfill(&Maze);
  //displaycellvalues(&Maze);
  //printf("\nFound starting values: %d %d\n", Maze.start_row, Maze.start_col);
  find_possible_solutions(&Maze_solutions, &StackPath, &Maze);
  GetMazePaths(&ShortestPathMaze, &Maze_solutions, &Maze);
  FindPathStrings(&StringPath, &Maze_solutions, &Maze, startingvalue(&Maze));
  // Write to Txt file and display shortest Path
  int indexShortest = WritePathStringToTxtFile(&StringPath);
  CopyMap(&ShortestPathMaze, &Maze);
  printf("--- Task 3 ---\n");
  FillAllMaze(&ShortestPathMaze, 45);
  ConvertPathstoMazePaths(&ShortestPathMaze, &Maze_solutions, indexShortest);
  displaycellvalues(&ShortestPathMaze);
  printf("Steps: %2d\nPath: %s", StringPath.PathLength[indexShortest] - 4, StringPath.PathStrings[indexShortest]);
  printf("--- Task 4 ---\n");
  printf("File: %s\n", PATH_PLAN_FILE_NAME);
  printf("Path: %s\n", StringPath.PathStrings[indexShortest]);
  //printf("Finished\n");
  //while (wb_robot_step(TIME_STEP) != -1) {
  //};
  wb_robot_cleanup();
  return 0;
}
