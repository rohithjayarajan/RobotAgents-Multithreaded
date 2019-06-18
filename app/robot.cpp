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
 *  @file    robot.cpp
 *  @author  rohithjayarajan
 *  @date 06/17/2019
 *  @version 1.0
 *
 *  @brief world map
 *
 *  @section DESCRIPTION
 *
 *  Class defintion for robot map
 *
 */

#include "robot.hpp"
#include <iostream>
#include <utility>
#include <vector>

Robot::Robot() {}

Robot::~Robot() {}

std::pair<int, int> Robot::move(std::vector<std::vector<char>>& map_,
                                std::pair<int, int> pos_) {
    // variable to store updated position of robot
    std::pair<int, int> newPos_;
    // get row and column of current position of robot
    int row_ = pos_.first;
    int col_ = pos_.second;
    // The robot motion based on a simple behavior (move one space a second in
    // one direction until blocked, then move in another direction)
    if (map_[row_ - 1][col_] == '0') {
        map_[row_ - 1][col_] = map_[row_][col_];
        newPos_ = std::make_pair(row_ - 1, col_);
        map_[row_][col_] = '0';
    } else if (map_[row_ - 1][col_ + 1] == '0') {
        map_[row_ - 1][col_ + 1] = map_[row_][col_];
        newPos_ = std::make_pair(row_ - 1, col_ + 1);
        map_[row_][col_] = '0';
    } else if (map_[row_ - 1][col_ - 1] == '0') {
        map_[row_ - 1][col_ - 1] = map_[row_][col_];
        newPos_ = std::make_pair(row_ - 1, col_ - 1);
        map_[row_][col_] = '0';
    } else if (map_[row_ + 1][col_] == '0') {
        map_[row_ + 1][col_] = map_[row_][col_];
        newPos_ = std::make_pair(row_ + 1, col_);
        map_[row_][col_] = '0';
    } else if (map_[row_ + 1][col_ + 1] == '0') {
        map_[row_ + 1][col_ + 1] = map_[row_][col_];
        newPos_ = std::make_pair(row_ + 1, col_ + 1);
        map_[row_][col_] = '0';
    } else if (map_[row_ + 1][col_ - 1] == '0') {
        map_[row_ + 1][col_ - 1] = map_[row_][col_];
        newPos_ = std::make_pair(row_ + 1, col_ - 1);
        map_[row_][col_] = '0';
    } else if (map_[row_][col_ + 1] == '0') {
        map_[row_][col_ + 1] = map_[row_][col_];
        newPos_ = std::make_pair(row_, col_ + 1);
        map_[row_][col_] = '0';
    } else if (map_[row_][col_ - 1] == '0') {
        map_[row_][col_ - 1] = map_[row_][col_];
        newPos_ = std::make_pair(row_, col_ - 1);
        map_[row_][col_] = '0';
    } else {
        return pos_;
    }
    // return updated position
    return newPos_;
}
