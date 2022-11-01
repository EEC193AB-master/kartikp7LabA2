#pragma once

#include "Eigen/Dense"
#include "./common.h"

// Struct Representing a standard Odometry reading
// Contains two rotations and one translation
struct OdoReading{
    float r1;
    float t;
    float r2;
};

// Struct representing a reading from a laser scanner
// Each reading has an ID, a range, and a bearing
struct LaserReading {
    uint64_t id;
    float range;
    float bearing;
};

// Class to represent measured data
class MeasurementPackage {
  // TODO: Complete implementation
  // You must have a way to read and store information in sensor.dat file
  // You must store all sensor readings (Records) at each time step
  //    for each record there is:
  //      One Odometry reading at each time step
  //      All laser scans at each time step

};
