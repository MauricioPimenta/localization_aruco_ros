/**
 * @file kalman_filter.cpp
 * @author Mauricio Bittencourt Pimenta
 * @brief Implementation of a generic Kalman Filter algorithm, based on the Table 3.1 of the book "Probabilistic Robotics" by Sebastian Thrun, Wolfram Burgard, and Dieter Fox.
 * @version 0.1
 * @date 2024-04-23
 */
#include "matplotlibcpp.h"
#include "kalman_filter.h"

#include <iostream>
#include <string>
#include <list>
#include <array>
#include <vector>

#include <fstream>
#include <sstream>
#include <iostream>
#include <algorithm> // to use std::sort
#include <cmath>    // to use M_PI, std::sin and std::cos

#include <eigen3/Eigen/Eigen>
