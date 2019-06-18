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
 *  @file    world.cpp
 *  @author  rohithjayarajan
 *  @date 06/17/2019
 *  @version 1.0
 *
 *  @brief world map
 *
 *  @section DESCRIPTION
 *
 *  Class definition for world map
 *
 */

// #include <stdlib.h>
#include <chrono>
#include <iostream>
#include <map>
#include <mutex>
#include <thread>
#include <utility>
#include <vector>
// BOOST header
#include <boost/range/irange.hpp>
#include "robot.hpp"
#include "world.hpp"

std::mutex mtx;

World::World() {}

World::World(const std::vector<std::vector<char>>& map_) { map_m = map_; }

World::~World() {}

bool World::addRobot(int row_, int col_, char robotName_) {
    // add robot at specified row, col
    map_m[row_][col_] = robotName_;
    // save robot name and position in a map structure
    robotPos_m.insert({robotName_, std::make_pair(row_, col_)});
    std::cout << "Added robot named " << robotName_ << " at " << row_ << ", "
              << col_ << std::endl;
    // return true on success
    return true;
}

void World::runAgent(char robotName_) {
    // create object of class Robot
    Robot robotObj;
    // decalre 2d temporary grid
    std::vector<std::vector<char>> map_temp;
    while (true) {
        // to process threads alternatively so as to not produce a messy map
        // display
        mtx.lock();
        // set temporary grid to be equal to grid
        map_temp = map_m;
        // display the grid after each thread makes changes
        display();
        // get current position of robot
        std::pair<int, int> pos_ = robotPos_m[robotName_];
        // move robot and get new position of robot
        std::pair<int, int> newPos_ = robotObj.move(map_temp, pos_);
        // update grid from after this motion
        map_m = map_temp;
        // update position of robot in the robot map structure
        robotPos_m[robotName_] = newPos_;
        // to process threads alternatively so as to not produce a messy map
        // display
        mtx.unlock();
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        continue;
    }
}

void World::run() {
    // declare a vector of threads
    std::vector<std::thread> vecThreads;
    // run a thread for each robot to perform move operation in the grid
    for (auto robotN : robotPos_m) {
        // get name of the robot
        char roboName = robotN.first;
        // create a thread for the robot of above name
        std::thread t1(&World::runAgent, this, roboName);
        // push the thread in the vector of threads
        vecThreads.push_back(std::move(t1));
    }
    // sleep the main thread
    std::this_thread::sleep_for(std::chrono::seconds(1));
    // if the above doesn't work, uncomment line 45 to include #include
    // <stdlib.h> and uncomment below line
    // sleep(1);
    // join all the threads
    for (auto& th : vecThreads) {
        if (th.joinable()) {
            th.join();
        }
    }
    // clear the vector
    vecThreads.clear();
}

void World::display() {
    // "Clear" screen
    for (int i = 0; i < 20; i++) std::cout << std::endl;
    for (auto vec : map_m) {
        for (auto val : vec) {
            std::cout << val;
        }
        std::cout << std::endl;
    }
}

void World::setMap(const std::vector<std::vector<char>>& map_) { map_m = map_; }

void World::setRobotPos(const std::map<char, std::pair<int, int>>& robotPos_) {
    robotPos_m = robotPos_;
}

std::vector<std::vector<char>> World::getMap() { return map_m; }

std::map<char, std::pair<int, int>> World::getRobotPos() { return robotPos_m; }
