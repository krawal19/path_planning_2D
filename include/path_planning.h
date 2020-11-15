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
 *  @file   path_planning.h
 *
 *  @author   Kapil Rawal (kapilrawal1995@gmail.com)
 *
 *  @copyright   BSD License
 *
 *  @brief   PathPlanning class header file
 *
 *  @section   DESCRIPTION
 *
 *  File structure containing the definations of PathPlanning class
 *
 */

// takes in 2d grid and start and end point
// checks validity of start and end point
// if valid then start the path planning
// use bfs / dfs / dijkstra / astar  

#ifndef INCLUDE_PATH_PLANNING_
#define INCLUDE_PATH_PLANNING_

#include <boost/optional.hpp>
#include <vector>
#include <string>
#include <set>

/**
 * @brief PathPlanning class defination
 * 
 */
class PathPlanning {
 private:
	int MIN_GRID_SIZE;
	int TOTAL_NEIGHBOURS;
    typedef boost::optional<std::pair<int,int>> index;
	index start_;
    index goal_;
    bool initalized_flag_;
    std::vector<std::vector<int>> grid_;

 public:
    /**
     * @brief Construct a new Path Planning object
     * 
     */
    PathPlanning();
    /**
     * @brief Destroy the Path Planning object
     * 
     */
    ~PathPlanning();
    /**
     * @brief 
     * 
     * @param input_grid 
     * @param start 
     * @param goal 
     * @param generatedPath 
     * @param plannerType 
     */
    void plannerMain(const std::vector<std::vector<int>>& input_grid, 
                  index start, index goal,
                  std::vector<index> *generatedPath,
                  const std::string plannerType);
    /**
     * @brief Set the Params object
     * 
     * @param input_grid 
     * @param start 
     * @param goal 
     * @return true 
     * @return false 
     */
    bool setParams(const std::vector<std::vector<int>>& input_grid, 
                   index start, index goal);

    /**
     * @brief 
     * 
     * @param input_grid 
     * @return true 
     * @return false
     */
    bool validGrid(const std::vector<std::vector<int>>& input_grid);
    /**
     * @brief 
     * 
     * @param input_grid 
     * @param position 
     * @return true 
     * @return false 
     */
    bool validPosition(const std::vector<std::vector<int>>& input_grid, 
                       index position);
    /**
     * @brief 
     * 
     * @param generatedPath 
     * @param plannerType 
     */
    void pathPlanner(std::vector<index>* generatedPath,
                     const std::string plannerType);
    /**
     * @brief 
     * 
     * @param neighbours 
     * @param grid 
     * @param visited_set 
     * @param node 
     */
	void generateNeighbour(std::vector<index> *neighbours,
                           std::set<index> *visited_set,
                           const std::vector<std::vector<int>>& grid,
                           const index node);
    /**
     * @brief 
     * 
     * @param generatedPath 
     */
	void dijkstra(std::vector<index>* generatedPath);
};

#endif  //  INCLUDE_PATH_PLANNING_
