/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018, EAIBOT, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
#define _USE_MATH_DEFINES
#include "CYdLidar.h"
#include <iostream>
#include <string>
#include <algorithm>
#include <cctype>
#include <fstream>
#include <limits>
#include <math.h>
#include "../matplotlibcpp.h"
#include <core/common/ydlidar_def.h>
namespace plt = matplotlibcpp;

using namespace std;
using namespace ydlidar;

#if defined(_MSC_VER)
#pragma comment(lib, "ydlidar_sdk.lib")
#endif
/**
 * @brief ydlidar test
 * @param argc
 * @param argv
 * @return
 * @par Flow chart
 * Step1: instance CYdLidar.\n
 * Step2: set paramters.\n
 * Step3: initialize SDK and LiDAR.(::CYdLidar::initialize)\n
 * Step4: Start the device scanning routine which runs on a separate thread and enable motor.(::CYdLidar::turnOn)\n
 * Step5: Get the LiDAR Scan Data.(::CYdLidar::doProcessSimple)\n
 * Step6: Stop the device scanning thread and disable motor.(::CYdLidar::turnOff)\n
 * Step7: Uninitialize the SDK and Disconnect the LiDAR.(::CYdLidar::disconnecting)\n
 */

float cosRule(const LaserPoint& p1, const LaserPoint& p2){
    float R1 = p1.range;
    float R2 = p2.range;
    return sqrt(R1*R1+R2*R2-2*R1*R2*cos(p2.angle - p1.angle));
}

float getEulerDistance(const LaserPoint& p1, const LaserPoint& p2){
    float dx = p2.x - p1.x;
    float dy = p2.y - p1.y;
    return sqrt(dx*dx + dy*dy);
}

bool mergeBlocks(vector<LaserPoint> bk1, vector<LaserPoint>bk2, vector<vector<LaserPoint>>& container, float robot_width){
    LaserPoint& p = bk1.back();
    LaserPoint& q = bk2[0];
    float d = cosRule(p, q);
    if (d > robot_width){
        container.push_back(bk1);
        container.push_back(bk2);
        return false;
    }else{
        float d_ave1 =(bk1.back().angle - bk1[0].angle)/bk1.size();
        float d_ave2 =(bk2.back().angle - bk2[0].angle)/bk2.size();
        float d_ave = (d_ave1+d_ave2)/2;
        int N = d/d_ave; //Number of compensated points
        for (int j =0;j <N; j++){ //Add compensated points to block
            LaserPoint comp_p;
            comp_p.range = p.range + j*(q.range - p.range)/N;
            comp_p.angle = p.angle + j*(q.angle - p.angle)/N;
            comp_p.x = comp_p.range*cos(comp_p.angle);
            comp_p.y = comp_p.range*sin(comp_p.angle);
            bk1.push_back(comp_p);
        }
        bk1.insert(bk1.end(), bk2.begin(), bk2.end());
        container.push_back(bk1);
        return true;
    }
}

void nextMove(int* move, vector<LaserPoint> points){
  LaserPoint target;
  target.x = 1;
  target.y = 0;
  target.angle = atan2(target.y, target.x);
  target.range = sqrt(target.x*target.x + target.y*target.y);

  LaserPoint robot;
  robot.x = 0;
  robot.y = 0;
  robot.range = 0;
  robot.angle = 0;

  LaserPoint robot_global;
  robot_global.x = 0;
  robot_global.y = 0;
  robot_global.range = 0;
  robot_global.angle = 0;

  LaserPoint target_global;
  target_global.x = 1;
  target_global.y = 0;
  target_global.angle = atan2(target.y, target.x);
  target_global.range = sqrt(target.x*target.x + target.y*target.y);
  
  float robot_vel = 0.1;
  float robot_dec = 0.05;
  float robot_width = 0.1;
  float gap_distance = 1.1*robot_width;
  float safety_distance = 1.1*robot_width;
  float s1 = robot_vel*robot_vel/(2*robot_dec); // Decelerating space component
  float robot_turning_delta = 0.1;

  for(int i =0; i < points.size(); i++){
      LaserPoint& prev = points[i-1];
      LaserPoint& p = points[i];
      float theta = p.angle;
      float range = p.range;
      float x = range*cos(theta);
      float y = range*sin(theta);
      p.x = x;
      p.y = y;
      
      if (range > 10){ 
          points[i].range = 0;
          continue;
      }
  }

  plt::figure_size(1500, 780);
  // plt::figure_size(1200, 700);
  vector<float> X, Y, colors;
  vector<float> X1, Y1;
  vector<float> X3, Y3;
  vector<float> X2, Y2;

  //Segmentaion - find obstacles inside robot zone
  vector<vector<LaserPoint>> blocks; 
  for (int i=0; i <points.size(); i++){
      LaserPoint& p = points[i];
      LaserPoint& prev = (blocks.size())? blocks.back().back() : points[-1];
      float angle_diff = abs(p.angle-robot.angle);
      float D = (angle_diff <= M_PI/2)? s1*(pow(cos(angle_diff),2)) + safety_distance : 0; //Distance threshold
      if (p.range != 0 && p.range <= D){
          float R1 = prev.range;
          float R2 = p.range;
          float d = cosRule(prev, p);
          float Td = 10;
          float k1 = ((R1+R2)/2 > Td)? robot_width*R1*R2/(100*R1*R2) : 0.15;
          if (blocks.size() == 0 || d > k1*robot_width){
              blocks.push_back(vector<LaserPoint>{p});
          }else{
              blocks.back().push_back(p);
          }
      }
  }
  
  //Merging
  vector< vector<LaserPoint>> merged_blocks;
  if (blocks.size() > 0){
      merged_blocks.push_back(blocks[0]);
      for (int i=1; i <blocks.size(); i++){
          vector<LaserPoint> bk1 = merged_blocks.back();
          vector<LaserPoint> bk2 = blocks[i];
          merged_blocks.pop_back();
          mergeBlocks(bk1, bk2, merged_blocks, gap_distance);
      }
  }
  
  //Merge the first and last blocks
  if (merged_blocks.size() > 1){
      vector<LaserPoint> bk1 = merged_blocks.back();
      vector<LaserPoint> bk2 = merged_blocks.front();
      merged_blocks.pop_back();
      if (mergeBlocks(bk1, bk2, merged_blocks, gap_distance)){
          merged_blocks.erase(merged_blocks.begin());
      }else{
          merged_blocks.pop_back();
      }
  }
  
  float final_pt_angle = 0;
  float final_min_cost = (std::numeric_limits<float>::max)();
  float start = - M_PI;
  float dd = 0.5*M_PI/180;
  for (int i=0; i < 360; i++){
      float theta = 2*M_PI*i/360 - M_PI;
      // if (p.angle < -M_PI/2 || p.angle > M_PI/2) continue;
      float angle_diff = abs(theta - robot.angle);
      float D = (angle_diff <= M_PI/2)? s1*(pow(cos(angle_diff),2)) + safety_distance : 0; // Distance threshold

      bool valid = true;
      for (int i =0; i <merged_blocks.size() ; i++){
          LaserPoint &p = merged_blocks[i].front();
          LaserPoint &q = merged_blocks[i].back();
          float b1 = atan2(gap_distance/2, p.range); //Minimum angle from center
          float b2 = atan2(gap_distance/2, q.range);
          if (theta <= p.angle+b1 && theta >= q.angle-b2){
              valid = false;
              break;
          }
      }

      if (valid){
          float cost = 0.6*abs(theta-target.angle) + 0.4*abs(theta-robot.angle);
          if (cost < final_min_cost){
              final_min_cost = cost;
              final_pt_angle = theta;
          }
      }
  }

  if (final_min_cost == (std::numeric_limits<float>::max)() || abs(final_pt_angle) < robot_turning_delta){
      move[0] = 1;
      move[1] = 1;
  }else if (final_pt_angle<0){
      move[0] = 1;
      move[1] = 0;
  }else{
      move[0] = 0;
      move[1] = 1;
  }

  for (int i =0; i <merged_blocks.size() ; i++){
      LaserPoint &p = merged_blocks[i].front();
      LaserPoint &q = merged_blocks[i].back();
      float b1 = atan2(gap_distance/2, p.range);
      float b2 = atan2(gap_distance/2, q.range);
      printf("Mergeblocks i: %d, start: %f, end: %f, b1: %f, b2: %f\n", i, p.angle, q.angle, b1, b2);
  }
  std::cout << "final_pt_i.angle: "<<final_pt_angle<<std::endl;

  //Plot all points
  for (int i=0; i <points.size();i++){
      if (points[i].range == 0) continue;
      X3.push_back(points[i].x);
      Y3.push_back(points[i].y);
  }

  //Plot obstables
  for (int i=0; i<merged_blocks.size(); i++){
      for (int j = 0; j <merged_blocks[i].size(); j++){
          X.push_back(merged_blocks[i][j].x);
          Y.push_back(merged_blocks[i][j].y);
          colors.push_back(i%20);
      }
  }
  
  //Plot the safety distance
  for (int i=-180; i<180; i++){
      float theta = i*M_PI/180;
      float angle_diff = abs(theta-robot.angle);
      float D = (angle_diff <= M_PI/2)? s1*(pow(cos(angle_diff),2)) + safety_distance : 0; // Distance threshold

      float D1 = 0.1+safety_distance;
      float x1 = D*cos(theta);
      float y1 = D*sin(theta);
      X1.push_back(x1);
      Y1.push_back(y1);
  }

  plt::scatter(X1, Y1, 1.0, {{"c","red"}});
  plt::scatter(X3, Y3, 1.0, {{"c","black"}});
  plt::scatter_colored(X, Y, colors, 1.0, {{"cmap","tab20"}});
  plt::plot(vector<float>{robot_width/2, robot_width/2, -robot_width/2,-robot_width/2, robot_width/2},
            vector<float>{-robot_width/2,robot_width/2, robot_width/2, -robot_width/2, -robot_width/2});
  // plt::scatter(vector<float>{robot.x}, vector<float>{robot.y}, 15.0, {{"marker", "X"}, {"c", "blue"}});
  plt::quiver(vector<float>{robot.x}, vector<float>{robot.y}, vector<float>{cos(robot.angle)}, vector<float>{sin(robot.angle)});
  plt::scatter(vector<float>{target.x}, vector<float>{target.y}, 15.0, {{"marker", "X"}, {"c", "red"}});
  // plt::scatter(X2, Y2, 25.0, {{"marker", "X"}, {"c", "purple"}});
  plt::plot(vector<float>{0, float(0.5*cos(final_pt_angle))}, vector<float>{0, float(0.5*sin(final_pt_angle))});


  printf("Move: %d, %d\nrobot:: (x,y): (%f,%f), angle: %f\n", move[0],move[1],robot_global.x, robot_global.y, robot_global.angle);
  plt::show(); 

}

int main(int argc, char *argv[])
{
  std::string port;
  ydlidar::os_init();

  std::map<std::string, std::string> ports =
      ydlidar::lidarPortList();
  std::map<std::string, std::string>::iterator it;

  if (ports.size() == 1)
  {
    port = ports.begin()->second;
  }
  else
  {
    int id = 0;

    for (it = ports.begin(); it != ports.end(); it++)
    {
      printf("%d. %s\n", id, it->first.c_str());
      id++;
    }

    if (ports.empty())
    {
      printf("Not Lidar was detected. Please enter the lidar serial port:");
      std::cin >> port;
    }
    else
    {
      while (ydlidar::os_isOk())
      {
        printf("Please select the lidar port:");
        std::string number;
        std::cin >> number;

        if ((size_t)atoi(number.c_str()) >= ports.size())
        {
          continue;
        }

        it = ports.begin();
        id = atoi(number.c_str());

        while (id)
        {
          id--;
          it++;
        }

        port = it->second;
        break;
      }
    }
  }

  int baudrate = 230400;
  std::map<int, int> baudrateList;
  baudrateList[0] = 115200;
  baudrateList[1] = 128000;
  baudrateList[2] = 153600;
  baudrateList[3] = 230400;
  baudrateList[4] = 460800;
  baudrateList[5] = 512000;

  printf("Baudrate:\n");

  for (std::map<int, int>::iterator it = baudrateList.begin();
       it != baudrateList.end(); it++)
  {
    printf("%d. %d\n", it->first, it->second);
  }

  while (ydlidar::os_isOk())
  {
    printf("Please select the lidar baudrate:");
    std::string number;
    // std::cin >> number;
    number = "2";

    if ((size_t)atoi(number.c_str()) > baudrateList.size())
    {
      continue;
    }

    baudrate = baudrateList[atoi(number.c_str())];
    break;
  }

  if (!ydlidar::os_isOk())
  {
    return 0;
  }

  bool isSingleChannel = false;
  std::string input_channel;
  printf("Whether the Lidar is one-way communication[yes/no]:");
  // std::cin >> input_channel;
  input_channel = "yes";
  std::transform(input_channel.begin(), input_channel.end(),
                 input_channel.begin(),
                 [](unsigned char c)
                 {
                   return std::tolower(c); // correct
                 });

  if (input_channel.find("y") != std::string::npos)
  {
    isSingleChannel = true;
  }

  if (!ydlidar::os_isOk())
  {
    return 0;
  }

  std::string input_frequency;

  float frequency = 5.0;

  while (ydlidar::os_isOk() && !isSingleChannel)
  {
    printf("Please enter the lidar scan frequency[5-12]:");
    std::cin >> input_frequency;
    frequency = atof(input_frequency.c_str());

    if (frequency <= 12 && frequency >= 5.0)
    {
      break;
    }

    fprintf(stderr,
            "Invalid scan frequency,The scanning frequency range is 5 to 12 HZ, Please re-enter.\n");
  }

  if (!ydlidar::os_isOk())
  {
    return 0;
  }

  CYdLidar laser;
  //////////////////////string property/////////////////
  /// lidar port
  laser.setlidaropt(LidarPropSerialPort, port.c_str(), port.size());
  /// ignore array
  std::string ignore_array;
  ignore_array.clear();
  laser.setlidaropt(LidarPropIgnoreArray, ignore_array.c_str(),
                    ignore_array.size());

  //////////////////////int property/////////////////
  /// lidar baudrate
  laser.setlidaropt(LidarPropSerialBaudrate, &baudrate, sizeof(int));
  /// tof lidar
  int optval = TYPE_TRIANGLE;
  laser.setlidaropt(LidarPropLidarType, &optval, sizeof(int));
  /// device type
  optval = YDLIDAR_TYPE_SERIAL;
  laser.setlidaropt(LidarPropDeviceType, &optval, sizeof(int));
  /// sample rate
  optval = isSingleChannel ? 3 : 4;
  laser.setlidaropt(LidarPropSampleRate, &optval, sizeof(int));
  /// abnormal count
  optval = 4;
  laser.setlidaropt(LidarPropAbnormalCheckCount, &optval, sizeof(int));
  /// Intenstiy bit count
  optval = 10;
  laser.setlidaropt(LidarPropIntenstiyBit, &optval, sizeof(int));

  //////////////////////bool property/////////////////
  /// fixed angle resolution
  bool b_optvalue = true;
  laser.setlidaropt(LidarPropFixedResolution, &b_optvalue, sizeof(bool));
  /// rotate 180
  laser.setlidaropt(LidarPropReversion, &b_optvalue, sizeof(bool));
  /// Counterclockwise
  laser.setlidaropt(LidarPropInverted, &b_optvalue, sizeof(bool));
  b_optvalue = true;
  laser.setlidaropt(LidarPropAutoReconnect, &b_optvalue, sizeof(bool));
  /// one-way communication
  laser.setlidaropt(LidarPropSingleChannel, &isSingleChannel, sizeof(bool));
  /// intensity
  b_optvalue = true;
  laser.setlidaropt(LidarPropIntenstiy, &b_optvalue, sizeof(bool));
  /// Motor DTR
  b_optvalue = false;
  laser.setlidaropt(LidarPropSupportMotorDtrCtrl, &b_optvalue, sizeof(bool));
  /// HeartBeat
  b_optvalue = false;
  laser.setlidaropt(LidarPropSupportHeartBeat, &b_optvalue, sizeof(bool));

  //////////////////////float property/////////////////
  /// unit: °
  float f_optvalue = 180.0f;
  laser.setlidaropt(LidarPropMaxAngle, &f_optvalue, sizeof(float));
  f_optvalue = -180.0f;
  laser.setlidaropt(LidarPropMinAngle, &f_optvalue, sizeof(float));
  /// unit: m
  f_optvalue = 64.f;
  laser.setlidaropt(LidarPropMaxRange, &f_optvalue, sizeof(float));
  f_optvalue = 0.05f;
  laser.setlidaropt(LidarPropMinRange, &f_optvalue, sizeof(float));
  /// unit: Hz
  laser.setlidaropt(LidarPropScanFrequency, &frequency, sizeof(float));

  //禁用阳光玻璃过滤
  laser.enableGlassNoise(false);
  laser.enableSunNoise(false);

  bool ret = laser.initialize();
  if (!ret)
  {
    fprintf(stderr, "Fail to initialize %s\n", laser.DescribeError());
    fflush(stderr);
    return -1;
  }

  ret = laser.turnOn();
  if (!ret)
  {
    fprintf(stderr, "Fail to start %s\n", laser.DescribeError());
    fflush(stderr);
    std::cin.get();
    return -1;
  }
  
  //获取用户版本
  if (ret && ydlidar::os_isOk())
  {
    std::string userVersion;
    if (laser.getUserVersion(userVersion))
    {
      printf("[YDLIDAR]: User version %s\n", userVersion.c_str());
    }
  }

  FILE * pFileTXT = fopen ("scan_pts.txt","a");
  LaserScan scan;
  while (ydlidar::os_isOk())
  {
    if (laser.doProcessSimple(scan))
    {
      printf("Scan received [%u] points inc [%f]\n",
             (unsigned int)scan.points.size(),
             scan.config.angle_increment);
      // for (size_t i = 0; i < scan.points.size(); ++i)
      // {
      //   const LaserPoint &p = scan.points.at(i);
      //   printf("%d d %f a %f\n", i, p.range, p.angle * 180.0 / M_PI);
      // }
      fflush(stdout);
      for(int i =0; i < scan.points.size(); i++){
        fprintf (pFileTXT, "%f, %f, %f\n", scan.points[i].angle, scan.points[i].range, scan.points[i].intensity);
      }

      int move[2];
      nextMove(move, scan.points);

    }else{
      fprintf(stderr, "Failed to get Lidar Data\n");
      fflush(stderr);
    }
  }
  laser.turnOff();
  laser.disconnecting();

  fclose (pFileTXT);
  return 0;
}
