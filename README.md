<h1 align=center> PATH PLANNING 2D </h1>

[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)

## Project Overview
The project is a path planning library for 2D grid. It has following features:
- Ability to add custom 2D BinaryGrid* with start and goal positions
- Performs validity check on grid, start and goal
- Currently Dijkstra shortest path planning algorithm is used as a planner
- The library offers flexibility to add new planning algorithms 

>[*NOTE : 1 for obstacle and 0 for free position]

## Overview using the API advanced
The library API provides wide variety of clean and optimized methods to check for performing complete path planning in 2D grid. 
Each method of the path planning library is self sufficent to act as stand alone function. The functions offer zero memory leaks(verified from valgrind outputs). The API is Doxygen style documented and thus easier to use and understand.

## License
This project is under the [BSD License](https://github.com/krawal19/path_planning_2d/-/tree/master/LICENSE).

## Development using Solo Iterative Process (SIP) and Test-Driven Development (TDD)
This library has been developed by following the Solo Iterative Process (SIP). You can take a look at the log details by going on this [LINK]()

## Dependencies
The project requires following dependenices:
- Gcc 7.5.0
- Boost
- Googletest
- CMake 3.10.2
- Ubuntu 18.04

## Program installation
```
git clone https://github.com/krawal19/path_planning_2d.git
cd path_planning_2d
mkdir lib
cd lib 
git clone https://github.com/google/googletest/
cd ..
mkdir build
cd build
cmake ..
make

Run program: ./pathPlanning
```
## Running tests via command line
To check the tests you can run the following command.
```
Run program: ./test/pathPlanningTest
```
## Instructions
The current program version(*see note below) has pre-specified 2d grid, start and goal.
For test your own start/goal or grid follow the steps below: 
- Open the main.cpp file under /path_planning_2d/src folder in your favourite editor.
- The following fields should be modified for your input
1. For 2D grid, Modify 1 or 0 for adding or removing obstacles and free path
```
std::vector<std::vector<int>> grid_2d{{0,0,0,0,0}, <--- Here zero represents free path
                                      {0,1,0,0,0}, <--- one represents obstacle
                                      {0,0,0,0,0},
                                      {0,1,0,1,1},
                                      {0,0,0,1,0}};
```
2. For start and goal, change fileds as shown below for custom start and goal
```
start.first = 0; // Add start's 1st index
start.second = 0; // Add start's 2nd index
goal.first = 4; // Add goal's 1st index
goal.second = 4; // Add goal's 2nd index
```
- Once the fields have been modified follow the steps below to build and run the code in terminal 
```
cd path_planning_2d
mkdir build (skip if build folder already present)
cd build
cmake ..
make

Run program: ./pathPlanning
```
-  Running the program will print output as follows:
0. Here the grid used is 
```
{{0,0,0,0,0},
 {0,1,0,0,0},
 {0,0,0,0,0},
 {0,1,0,1,1},
 {0,0,0,1,0}}
```
1. If path exist between start and goal
```
Path found between start: (0, 0), Goal: (2, 4)
(0, 0) (0, 1) (0, 2) (1, 3) (2, 4) 
```
2. If no path found
```
No Path found between start: (0, 0), Goal: (4, 4)
```
3. For invalid grid, start, goal and Obstacle
```
Grid not valid!

Start not valid!

Goal not valid!

On obstacle, start/goal invalid!
```
[*NOTE: The input feature for grid, start and goal will come in next version]

## Author
- Kapil Rawal
- Email Id: Kapilrawal1995@gmail.com