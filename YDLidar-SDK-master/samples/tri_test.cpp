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
#include "CYdLidar.h"
#include <iostream>
#include <string>
#include <algorithm>
#include <cctype>
#include <fstream>
#include <math.h>
#include "../matplotlibcpp.h"
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

struct PointStruct{
  int id;
  int obj_type;
  int obj_id;
  float angle;
  float range;
  float intensity;
  float accum_angle;
  PointStruct* prev;
  PointStruct* next;
};

float calDistance(PointStruct* p1, PointStruct* p2){
  float x1,x2,y1,y2;
  float theta1 = p1->angle * M_PI / 180.0; 
  float theta2 = p2->angle * M_PI / 180.0; 
  x1 = sin(theta1)*p1->range;
  y1 = cos(theta1)*p1->range;
  x2 = sin(theta2)*p2->range;
  y2 = cos(theta2)*p2->range;
  return sqrt(pow(y1-y2, 2) + pow(x2-x1, 2));
};

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

  // float robot_facing = PI/2;
  // float robot_vel = 0.1;
  // float robot_dec = 0.05;
  // float robot_radius = 0.05;
  // float gap_distance = 1.1*robot_radius;
  // float safety_distance = 1.1*robot_radius;
  // float s1 = robot_vel*robot_vel/(2*robot_dec); // Decelerating space component

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

      
      int n = scan.points.size();
      vector<float> X, Y;
      for(int i =0; i < scan.points.size(); i++){
        float theta = scan.points[i].angle;
        float range = scan.points[i].range;
        float x = range*cos(theta);
        float y = range*sin(theta);
        // X.at(i) = x;
        // Y.at(i) = y;

        if (range < 5){
          X.push_back(x);
          Y.push_back(y);
        }
        // float angle_diff = abs(theta-robot_facing) if abs(theta-robot_facing) < PI else abs(abs(theta-robot_facing)-PI)
        // float D = s1*(cos(angle_diff)**2) + safety_distance if (angle_diff <= np.pi/2) else 0 // Distance threshold

      }
      plt::scatter(X, Y);
      plt::show();


    }
    else
    {
      fprintf(stderr, "Failed to get Lidar Data\n");
      fflush(stderr);
    }
  }
  laser.turnOff();
  laser.disconnecting();

  fclose (pFileTXT);
  return 0;
}
