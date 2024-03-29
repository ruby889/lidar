
#include "process_data.h" 
#include <math.h> 
#include <iostream> 
#include <stdio.h>
#include "../matplotlibcpp.h"
namespace plt = matplotlibcpp;
using namespace std;
static LaserPoint target;
static LaserPoint robot;
static LaserPoint robot_global;
static LaserPoint target_global;

float robot_vel = 0.2;
float robot_dec = 0.05;
float robot_width = 0.15;
float gap_distance = 1.2*robot_width;
float safety_distance = 1.35*robot_width;
float s1 = robot_vel*robot_vel/(2*robot_dec); // Decelerating space component
float robot_turning_delta = 0.1;

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

void nextMove(RobotCmd& cmd, vector<LaserPoint> points, FILE* logfile){
  target.x = 1;
  target.y = 0;
  target.angle = atan2(target.y, target.x);
  target.range = sqrt(target.x*target.x + target.y*target.y);

  robot.x = 0;
  robot.y = 0;
  robot.range = 0;
  robot.angle = 0;

  robot_global.x = 0;
  robot_global.y = 0;
  robot_global.range = 0;
  robot_global.angle = 0;

  target_global.x = 1;
  target_global.y = 0;
  target_global.angle = atan2(target.y, target.x);
  target_global.range = sqrt(target.x*target.x + target.y*target.y);
  
  //Calculate (x,y) coordinates
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

//   plt::figure_size(1500, 780);
  plt::figure_size(800, 700);
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
  
  //Merging close blocks
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
  
  //Find the optimal path by distance 
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
      //Forward
      cmd.servo = 90;
      cmd.motor = 97;
  }else if (final_pt_angle<0){
      //Turn left
      cmd.servo = 100;
      cmd.motor = 97;
  }else{
      //Turn right
      cmd.servo = 80;
      cmd.motor = 97;
  }


/*
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


  printf("servo: %d, motor: %d\nrobot:: (x,y): (%f,%f), angle: %f\n", cmd.servo,cmd.motor,robot_global.x, robot_global.y, robot_global.angle);
//  plt::show(); 
  if (!write_file.empty()){
    plt::save(write_file); 
  }

*/
  writeLog(logfile, points, merged_blocks, final_pt_angle);
}

void writeLog(FILE* logfile, const vector<LaserPoint>& points, const vector<vector<LaserPoint>>& merged_blocks, float final_pt_angle){
  if (!logfile)
    return;
  
  //Plot all points
  for (int i=0; i <points.size();i++){
      if (points[i].range == 0) continue;
      fprintf(logfile, "(%f,%f) ", points[i].x, points[i].y);
  }
  fprintf(logfile, "| ");

  //Plot obstables
  for (int i=0; i<merged_blocks.size(); i++){
      for (int j = 0; j <merged_blocks[i].size(); j++){
          fprintf(logfile, "(%f,%f) ", merged_blocks[i][j].x, merged_blocks[i][j].y);
      }
    fprintf(logfile, "/ ");
  }
  fprintf(logfile, "| ");
  fprintf(logfile, "%f, %f, %f | ", safety_distance, s1, robot_width);
  fprintf(logfile, "%f, %f, %f | ", robot.x, robot.y, robot.angle);
  fprintf(logfile, "%f, %f, %f | ", target.x, target.y, target.angle);
  fprintf(logfile, "%f | ", final_pt_angle);
  
  fprintf(logfile, "\n");
}
