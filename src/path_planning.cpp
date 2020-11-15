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
 *  @file   path_planning.cpp
 *
 *  @author   Kapil Rawal (kapilrawal1995@gmail.com)
 *
 *  @copyright   BSD License
 *
 *  @brief   PathPlanning class implementation file
 *
 *  @section   DESCRIPTION
 *
 * This file contains the implementations of the methods of PathPlanning Class
 *
 */

#include "path_planning.h"

#include <iostream>
#include <queue>
#include <map>
#include <stack>

PathPlanning::PathPlanning() {
    MIN_GRID_SIZE = 2;
    TOTAL_NEIGHBOURS = 8;
    initalized_flag_ = false;
    start_ = std::make_pair(INT16_MIN, INT16_MIN);
    goal_ = std::make_pair(INT16_MIN, INT16_MIN);
}

PathPlanning::~PathPlanning() {}

void PathPlanning::plannerMain(const std::vector<std::vector<int>>& input_grid, 
                            index start, index goal, std::vector<index>* generatedPath,
                            const std::string plannerType) {
    // Calling  the planner if succesfully setted the parameters
    if (setParams(input_grid, start, goal) && generatedPath->empty())
        pathPlanner(generatedPath, plannerType);
}

bool PathPlanning::setParams(const std::vector<std::vector<int>>& input_grid, 
                            index start, index goal) {
    // Checking validity of grid before setting paramteres
    bool valid_grid = validGrid(input_grid);
    if (valid_grid) {
        bool valid_start = validPosition(input_grid, start);
        bool valid_goal = validPosition(input_grid, goal);
        // If grid and positions are valid then set the parameters
        if (valid_start && valid_goal) {
            grid_ = input_grid;
            start_ = start;
            goal_ = goal;
            // Setting initalized flag to true
            initalized_flag_ = true;
            return true;
        } else {
            if (!valid_start) std::cout << "Start not valid!" << std::endl;
            if (!valid_goal) std::cout << "Goal not valid!" << std::endl;
        }
    } else
        std::cout << "Grid not valid!" << std::endl;
    return false;
}

bool PathPlanning::validGrid(const std::vector<std::vector<int>>& input_grid) {
    // Checking if the grid is not empty
    if (!input_grid.empty()) {
        // checking size should be greater than minimum values
        int size_row = input_grid.size();
        int size_col = input_grid[0].size();
        return size_row >= MIN_GRID_SIZE && size_col >= MIN_GRID_SIZE;
    }
    return false;
}

bool PathPlanning::validPosition(const std::vector<std::vector<int>>& input_grid,
                                 index position) {
    if (validGrid(input_grid) && position) {
        int size_row = input_grid.size();
        int size_col = input_grid[0].size();
        int x = position.get().first;
        int y = position.get().second;

        // Range check of the points
        bool valid_index_first = x < size_row && y >= 0;
        bool valid_index_second = x < size_col && y >= 0;
        // Obstacle check of points
        bool on_obstacle_flag = true; // true if on obstacle else false
        if (valid_index_first && valid_index_second) {
            if (input_grid[x][y] == 0) 
                on_obstacle_flag = false;
            else
                std::cout << "On obstacle, start/goal invalid!" << std::endl;
        }
        return valid_index_first && valid_index_second && (!on_obstacle_flag);
    }
    return false;
}

void PathPlanning::pathPlanner(std::vector<index>* generatedPath,
                               const std::string plannerType) {
    // Checking if the parameters are initialized
    if (initalized_flag_ ) {
        // Checking if the input planner type is valid
        if (plannerType.compare("DIJKSTRA") == 0) {
            // Calling the valid planner, more planner can be added here
            dijkstra(generatedPath);
            // If path is generated
            if (generatedPath->size() > 0)
                std::cout << "Path found between start: (" << start_.get().first 
                        << ", " << start_.get().second << "), Goal: (" << goal_.get().first 
                        << ", " << goal_.get().second << ")" << std::endl;
            else
                std::cout << "No Path found between start: (" << start_.get().first 
                        << ", " << start_.get().second << "), Goal: (" << goal_.get().first 
                        << ", " << goal_.get().second << ")" << std::endl;
        } else
            std::cout << plannerType << " type planner not available" << std::endl;
    } else
        std::cout << "Parameters not initalized " << std::endl;
}

void PathPlanning::generateNeighbour(std::vector<index> *neighbours,
                                     std::set<index> *visited_set,
                                     const std::vector<std::vector<int>>& grid,
                                     const index node) {
    int x = node.get().first;
    int y = node.get().second;
    // Size of the grid
    int size_row = grid.size();
    int size_col = grid[0].size();
    // Generating neighbour index in eight directions
    const int cols[] = {-1, -1, -1, 0, 0, 1, 1, 1};
    const int rows[] = {-1, 0, 1, -1, 1, -1, 0, 1};
    const int neighbour_number = TOTAL_NEIGHBOURS;
    for (int n = 0; n < neighbour_number; ++n) {
        int i = x + rows[n];
        int j = y + cols[n];
        // Checking validity of the generated index
        if (i >= 0 && j >= 0 &&  i < size_row && j < size_col 
            &&  (visited_set->find(std::make_pair(i,j)) == visited_set->end()) 
            && grid[i][j] < 1) {
            // Adding to the vector of neighbours and marking them as visited
            neighbours->push_back(std::make_pair(i,j));
            visited_set->insert(std::make_pair(i,j));
        }
    }
}

void PathPlanning::dijkstra(std::vector<index>* generatedPath) {
    std::set<index> visited_set;
    std::map<index, index> prev_node;
    std::vector<std::vector<int>> distance(grid_.size(), 
                                           std::vector<int>(grid_[0].size(), INT16_MAX));
    std::priority_queue<std::pair<int,index>, 
                        std::vector<std::pair<int,index>>, 
                        std::greater<std::pair<int,index>>> vertex_queue;
    index node;
    bool goal_found_flag = false;

    // Start the planning
    vertex_queue.push(std::make_pair(0, start_));
    distance[start_.get().first][start_.get().second] = 0;
    visited_set.insert(start_);

    //  Searching in the grid for goal
    while (!vertex_queue.empty()) {
        // Node with the min distance
        int node_distance = vertex_queue.top().first;
        node = vertex_queue.top().second;
        vertex_queue.pop();

        // If goal found the exit the loop
        if (node.get().first == goal_.get().first && node.get().second == goal_.get().second) {
            goal_found_flag = true;
            break;
        }

        // Searching neighbours
        std::vector<index> neighbours_node;
        generateNeighbour(&neighbours_node, &visited_set, grid_, node);
        for (auto neighbour : neighbours_node) {
            int x = neighbour.get().first;
            int y = neighbour.get().second;
            // Keeping distance to all neighbours as 1
            int alt = node_distance + 1;
            // If current distance is less than max
            if (alt < distance[x][y]) {
                distance[x][y] = alt;
                vertex_queue.push(std::make_pair(alt, neighbour));
                // Later for getting the path from goal to start
                prev_node[neighbour] = node;
            }
        }
    }

    if (goal_found_flag) {
        // Extracting path from the grid
        auto curr_node = node;
        generatedPath->push_back(start_);
        std::stack<index> goal_to_start_nodes;
        // Tracing the path back
        while (prev_node.find(curr_node) != prev_node.end()) {
            goal_to_start_nodes.push(curr_node);
            curr_node = prev_node[curr_node];
        }
        // Adding path to the input vector
        while (!goal_to_start_nodes.empty()) {
            generatedPath->push_back(goal_to_start_nodes.top());
            goal_to_start_nodes.pop();
        }
    }
}
