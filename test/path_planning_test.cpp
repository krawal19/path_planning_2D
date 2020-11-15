/**
 * BSD 3-Clause License
 * Copyright (c) 2020, Kapil Rawal
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 * Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *  @copyright (c) BSD
 *
 *  @file   path_planning_test.cpp
 *
 *  @author   Kapil Rawal (kapilrawal1995@gmail.com)
 *
 *  @copyright   BSD License
 *
 *  @brief   PathPlanning class test implementation file
 *
 *  @section   DESCRIPTION
 *
 * This file contains the implementations of the test for PathPlanning Class methods
 *
 */

#include <gtest/gtest.h>
#include "path_planning.h"
#include <boost/operators.hpp>

/**
 * @brief TestFixture class
 * 
 */
class PathPlanningFixture : public testing::Test {
    protected:
        PathPlanning *planner;
        std::vector<std::vector<int>> grid_2d{{0,0,0,0,0},
                                              {0,1,0,0,0},
                                              {0,0,0,0,0},
                                              {0,1,0,1,0},
                                              {0,0,0,0,0}};
    void SetUp() { planner = new PathPlanning(); }
    void TearDown() {delete planner;}
};

/**
 * @brief Tests the validGrid method of PathPlanning class
 * 
 */
TEST_F(PathPlanningFixture, ValidGridTest) {
    std::vector<std::vector<int>> grid_null;
    std::vector<std::vector<int>> grid_size;

    // Checking grid with some value as well a empty grid
    EXPECT_EQ(true, planner->validGrid(grid_2d));
    EXPECT_EQ(false, planner->validGrid(grid_null));
}

/**
 * @brief Tests the validPosition method of PathPlanning class
 * 
 */
TEST_F(PathPlanningFixture, ValidPositionTest) {
    boost::optional<std::pair<int, int>> position;

    // Checking validity of the positon for invalid position, 
    // on grid, outside grid and on obstacle
    EXPECT_EQ(false, planner->validPosition(grid_2d, position));
    EXPECT_EQ(true, planner->validPosition(grid_2d, std::make_pair(2,2)));
    EXPECT_EQ(false, planner->validPosition(grid_2d, std::make_pair(-1,-1)));
    EXPECT_EQ(false, planner->validPosition(grid_2d, std::make_pair(1,1)));
}

/**
 * @brief Tests the setParams method of PathPlanning class
 * 
 */
TEST_F(PathPlanningFixture, SetParamsTest) {
    boost::optional<std::pair<int, int>> start;
    boost::optional<std::pair<int, int>> goal;

    // Checking with empty goal and start
    EXPECT_EQ(false, planner->setParams(grid_2d, start, goal));
    start = std::make_pair(2,2);
    goal = std::make_pair(1,4);
    // Checking with empty grid
    EXPECT_EQ(false, planner->setParams({{}}, start, goal));
    // Checking if paramters are valid and set
    EXPECT_EQ(true, planner->setParams(grid_2d, start, goal));
}

/**
 * @brief Tests the pathPlanner method of PathPlanning class
 * 
 */
TEST_F(PathPlanningFixture, PathPlannerTest) {
    std::pair<int, int> start(4,0);
    std::pair<int, int> goal(0,4);
    std::vector<boost::optional<std::pair<int,int>>> generatedPath;

    // Checking the condition before initializing the parameters
    planner->pathPlanner(&generatedPath, "RANDOM");
    EXPECT_EQ(true, generatedPath.empty());

    // Checking if taking planer type inputs
    planner->setParams(grid_2d, start, goal);
    planner->pathPlanner(&generatedPath, "DIJKSTRA");
    EXPECT_EQ(6, generatedPath.size());
    
    // Checking not availabe type planner inputs
    generatedPath = {};
    planner->pathPlanner(&generatedPath, "ASTAR");
    EXPECT_EQ(true, generatedPath.empty());
}

/**
 * @brief Tests the generateNeighbour method of PathPlanning class
 * 
 */
TEST_F(PathPlanningFixture, GenerateNeighbourTest) {
    std::pair<int, int> node(0,0);
    std::set<boost::optional<std::pair<int,int>>> visited_set;
    std::vector<boost::optional<std::pair<int,int>>> neighbours;

    // Generating the neighbours, and 'node' is always visited
    visited_set.insert(node);
    planner->generateNeighbour(&neighbours, &visited_set, grid_2d, node);
    // Checking if the correct neighbours are generated
    EXPECT_EQ(2, neighbours.size());
}

/**
 * @brief Tests the plannerMain method of PathPlanning class
 * 
 */
TEST_F(PathPlanningFixture, plannerMainTest) {
    std::pair<int, int> start(2,2);
    std::pair<int, int> goal(1,4);
    std::vector<boost::optional<std::pair<int,int>>> generatedPath;

    // Checking if the main plan method is working
    planner->plannerMain(grid_2d, start, goal, &generatedPath, "DIJKSTRA");
    EXPECT_EQ(3, generatedPath.size());
}

/**
 * @brief Tests the dijkstra method of PathPlanning class
 * 
 */
TEST_F(PathPlanningFixture, DijkstraTest) {
    std::pair<int, int> start(4,4);
    std::pair<int, int> goal(0,4);
    std::vector<boost::optional<std::pair<int,int>>> generatedPath;

    // Checking if the planner is able to generate the path     
    planner->setParams(grid_2d, start, goal);
    planner->dijkstra(&generatedPath);
    EXPECT_EQ(5, generatedPath.size());

    // Unable to reach goal condition
    grid_2d[1][4] = 1;
    grid_2d[0][3] = 1;
    grid_2d[1][3] = 1;
    generatedPath = {};
    planner->setParams(grid_2d, start, goal);
    planner->dijkstra(&generatedPath);
    EXPECT_EQ(true, generatedPath.empty());
}
