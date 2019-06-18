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
 *  @file    robot.hpp
 *  @author  rohithjayarajan
 *  @date 06/17/2019
 *  @version 1.0
 *
 *  @brief robot class
 *
 *  @section DESCRIPTION
 *
 *  Class declaration for robot
 *
 */

#ifndef INCLUDE_ROBOT_HPP_
#define INCLUDE_ROBOT_HPP_

#include <iostream>
#include <utility>
#include <vector>
#include "world.hpp"

// Forward declaration
class World;

/**
 * @brief the declaration of Robot class
 *
 * Declaration of the variable and methods of Robot class
 */
class Robot {
   public:
    /**
     *   @brief Default constructor for ImageManipulate
     *
     *   @param nothing
     *   @return nothing
     */
    Robot();
    /**
     *   @brief Default destructor for ImageManipulate
     *
     *   @param nothing
     *   @return nothing
     */
    ~Robot();
    /**
     *   @brief function to move the robot
     *
     *   @param std::vector<std::vector<char>> reference of the grid
     *   @param std::pair<int, int> value of position of robot in the grid
     *   @return update std::pair<int, int> value of position
     */
    static std::pair<int, int> move(std::vector<std::vector<char>>& map_,
                                    std::pair<int, int> pos_);
};

#endif  // INCLUDE_ROBOT_HPP_
