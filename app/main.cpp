/******************************************************************************
 * BSD 3-Clause License
 *
 * Copyright (C) 2019, Rohith Jayarajan
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the
 * names of its contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/
/**
 *  Copyright 2019 rohithjayarajan
 *  @file    main.cpp
 *  @author  rohithjayarajan
 *  @date 06/17/2019
 *  @version 1.0
 *
 *  @brief main file
 *
 *  @section DESCRIPTION
 *
 *  Main function
 *
 */
#include <vector>
#include "robot.hpp"
#include "world.hpp"

int main(int argc, char** argv) {
    // enter the map describing free space, obstacles and robot
    // 1: obstacle. 0: free space. "English character": robot
    std::vector<std::vector<char> > map{
        {'1', '1', '1', '1', '1', '1', '1'},
        {'1', '0', '0', '0', '0', '0', '1'},
        {'1', '0', '0', '0', '0', '0', '1'},
        {'1', '0', '0', '1', '1', '0', '1'},
        {'1', '0', '0', '1', '1', '0', '1'},
        {'1', '0', '0', '0', '0', '0', '1'},
        {'1', '1', '1', '1', '1', '1', '1'},
    };

    // create object of class World
    World w(map);

    // add robot A at <1,1> grid position
    w.addRobot(1, 1, 'A');
    // add robot B at <5,5> grid position
    w.addRobot(5, 5, 'B');
    // add robot C at <3,1> grid position
    w.addRobot(3, 1, 'C');
    // run the program to allow robots to move without collision
    w.run();
}
