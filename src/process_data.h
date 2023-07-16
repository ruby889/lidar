
#ifndef PROCESS_DATA_H
#define PROCESS_DATA_H

#ifndef _USE_MATH_DEFINES
    #define _USE_MATH_DEFINES
#endif
#include <vector>
#include <string>
#include <core/common/ydlidar_def.h>

struct RobotCmd{
  int servo;
  int motor;
};

float cosRule(const LaserPoint& p1, const LaserPoint& p2);
float getEulerDistance(const LaserPoint& p1, const LaserPoint& p2);
bool mergeBlocks(std::vector<LaserPoint> bk1, std::vector<LaserPoint>bk2, std::vector<std::vector<LaserPoint>>& container, float robot_width);
void nextMove(RobotCmd& cmd, std::vector<LaserPoint> points, std::string write_file=NULL);

#endif
