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
 *  @file    world.hpp
 *  @author  rohithjayarajan
 *  @date 06/17/2019
 *  @version 1.0
 *
 *  @brief world map class header
 *
 *  @section DESCRIPTION
 *
 *  Class declaration for world map
 *
 */

#ifndef INCLUDE_WORLD_HPP_
#define INCLUDE_WORLD_HPP_

// #include <stdlib.h>
#include <chrono>
#include <iostream>
#include <map>
#include <mutex>
#include <thread>
#include <utility>
#include <vector>
#include "world.hpp"
// BOOST header
#include <boost/range/irange.hpp>
#include "robot.hpp"

// Forward declaration
class Robot;

/**
 * @brief the declaration of World class
 *
 * Declaration of the variable and methods of World class
 */
class World {
   private:
    // data structure to hold the map of the world
    std::vector<std::vector<char>> map_m;
    // data structure to track individual robots
    std::map<char, std::pair<int, int>> robotPos_m;

   public:
    /**
     *   @brief Default constructor for ImageManipulate
     *
     *   @param nothing
     *   @return nothing
     */
    World();
    /**
     *   @brief constructor for ImageManipulate
     *
     *   @param map reference of type std::vector<std::vector<char>>&
     *   @return nothing
     */
    explicit World(const std::vector<std::vector<char>>& map_);
    /**
     *   @brief Default destructor for ImageManipulate
     *
     *   @param nothing
     *   @return nothing
     */
    ~World();
    /**
     *   @brief Function to add robot in the map
     *
     *   @param integer value of grid row
     *   @param integer value of grid column
     *   @param char value of name of robot
     *   @return bool value indicating success(true)
     */
    bool addRobot(int row_, int col_, char robotName_);
    /**
     *   @brief function to run a single agent (robot)
     *
     *   @param char value of name of robot
     *   @return nothing
     */
    void runAgent(char robotName_);
    /**
     *   @brief function to run all agents (robots)
     *
     *   @param nothing
     *   @return nothing
     */
    void run();
    /**
     *   @brief function to display the map
     *
     *   @param nothing
     *   @return nothing
     */
    void display();
    /**
     *   @brief function to set the map
     *
     *   @param std::vector<std::vector<char>> reference of map
     *   @return nothing
     */
    void setMap(const std::vector<std::vector<char>>& map_);
    /**
     *   @brief function to set the robot information
     *
     *   @param std::map<char, std::pair<int, int>> reference of robot
     * information
     *   @return nothing
     */
    void setRobotPos(const std::map<char, std::pair<int, int>>& robotPos_);
    /**
     *   @brief function to get the map
     *
     *   @param nothing
     *   @return std::vector<std::vector<char>> value of map
     */
    std::vector<std::vector<char>> getMap();
    /**
     *   @brief function to get the robot information
     *
     *   @param nothing
     *   @return std::map<char, std::pair<int, int>> value of robot information
     */
    std::map<char, std::pair<int, int>> getRobotPos();
};

#endif  // INCLUDE_WORLD_HPP_
