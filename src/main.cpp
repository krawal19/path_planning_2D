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
 *  @file   main.cpp
 *
 *  @author   Kapil Rawal (kapilrawal1995@gmail.com)
 *
 *  @copyright   BSD License
 *
 *  @brief   main file for 
 *
 *  @section   DESCRIPTION
 *
 *  Starts the 
 *
 */

#include "path_planning.h"

#include <iostream>

int main(int argc, char** argv) {
    // Here 0 is free position and 1 is obstacle
    std::vector<std::vector<int>> grid_2d{{0,0,0,0,0},
                                          {0,1,0,0,0},
                                          {0,0,0,0,0},
                                          {0,1,0,1,1},
                                          {0,0,0,1,0}};
    std::pair<int,int> start, goal;
    start.first = 0; // Add start's 1st index
    start.second = 0; // Add start's 2nd index
    goal.first = 4; // Add goal's 1st index
    goal.second = 4; // Add goal's 2nd index

    // Creating the PathPlanning object and calling the planner 
    PathPlanning *plan = new PathPlanning();
    std::vector<boost::optional<std::pair<int,int>>> generatedPath;
    plan->plannerMain(grid_2d, start, goal, &generatedPath, "DIJKSTRA");

    // Printing out path if generated
    if (generatedPath.size() > 0) {
        for (auto index: generatedPath) 
            std::cout << "(" << index.get().first << ", " << index.get().second << ") ";
        std::cout << std::endl;
    }

    return 0;
}
