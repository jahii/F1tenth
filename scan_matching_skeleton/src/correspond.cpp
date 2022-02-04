#include "scan_matching_skeleton/correspond.h"
#include "cmath"
#include "ros/ros.h"

using namespace std;

const int UP_SMALL = 0;
const int UP_BIG = 1;
const int DOWN_SMALL = 2;
const int DOWN_BIG = 3;

void getNaiveCorrespondence(vector<Point>& old_points, vector<Point>& trans_points, vector<Point>& points,
                        vector< vector<int> >& jump_table, vector<Correspondence>& c, float prob){

      c.clear();
      int last_best = -1;
      const int n = trans_points.size();
      const int m = old_points.size();
      float min_dist = 100000.00;
      int min_index = 0;
      int second_min_index = 0;

      //Do for each point
      for(int i = 0; i<n; ++i){
        min_dist = 100000.00;
        for(int j = 0; j<m; ++j){
          float dist = old_points[j].distToPoint2(&trans_points[i]);
          if(dist<min_dist){
            min_dist = dist;
            min_index = j;
            second_min_index = j-1;

          }
        }
        c.push_back(Correspondence(&trans_points[i], &points[i], &old_points[min_index], &old_points[second_min_index]));
      }

}

void getCorrespondence(vector<Point>& old_points, vector<Point>& trans_points, vector<Point>& points,
                        vector< vector<int> >& jump_table, vector<Correspondence>& c, float prob,float incre){

  // Written with inspiration from: https://github.com/AndreaCensi/gpc/blob/master/c/gpc.c
  // use helper functions and structs in transform.h and correspond.h
  // input : old_points : vector of struct points containing the old points (points of the previous frame)
  // input : trans_points : vector of struct points containing the new points transformed to the previous frame using the current estimated transform
  // input : points : vector of struct points containing the new points
  // input : jump_table : jump table computed using the helper functions from the transformed and old points
  // input : c: vector of struct correspondences . This is a refernece which needs to be updated in place and return the new correspondences to calculate the transforms.
  // output : c; update the correspondence vector in place which is provided as a reference. you need to find the index of the best and the second best point.
  //Initializecorrespondences
  c.clear();
  int last_best = -1;
  const int m = trans_points.size();
  const int n = old_points.size();

  //Do for each point
  for(int i = 0; i<m; ++i){
    double best_dis = 10000000.00;
    int best = -1;
    int second_best = -1;

    if(last_best == -1){
      for(int j=0;j<n; ++j){
        double dis = old_points[j].distToPoint2(&trans_points[i]);
        if(dis<best_dis){
          best_dis = dis;
          best = j;
          second_best = j-1;
          if(second_best == -1){
            second_best = 1;
          }
          last_best = best;
        }
      }
    }
    else{
      int up_jump;
      int down_jump;
      if(trans_points[i].r <= old_points[last_best].r){
         up_jump = jump_table[last_best][UP_SMALL];
         down_jump = jump_table[last_best][DOWN_SMALL];
      }else{
         up_jump = jump_table[last_best][UP_BIG];
         down_jump = jump_table[last_best][DOWN_BIG];
      }
       if(up_jump == n)
       up_jump = n-1;
       if(down_jump == -1)
       down_jump = 0;

    double  point_dis = trans_points[i].r;
    double  point_ang = trans_points[i].theta;

    double  up_dis = old_points[up_jump].distToPoint2(&trans_points[i]);
    double  up_angle =0;
    if (point_dis*point_dis - up_dis*up_dis > 0){
    up_angle =  atan2(up_dis, sqrt(point_dis*point_dis - up_dis*up_dis));
  }
    else{
    up_angle = 3.1415926;
  }
    int  up_low_idx = int((point_ang - up_angle)/incre);
    int  up_high_idx = int((point_ang + up_angle)/incre);
       if(up_low_idx<0)
       up_low_idx = 0;
       if(up_high_idx>=n)
       up_high_idx = n-1;


    double  down_dis = old_points[down_jump].distToPoint2(&trans_points[i]);
    double  down_angle = 0;
    if (point_dis*point_dis - down_dis*down_dis >0){
  down_angle = atan2(down_dis, sqrt(point_dis*point_dis - down_dis*down_dis));
  }
  else{
    down_angle =  3.1415926;
  }

    int   down_low_idx = int((point_ang - down_angle)/incre);
    int   down_high_idx = int((point_ang + down_angle)/incre);
       if(down_low_idx<0)
       down_low_idx = 0;
       if(down_high_idx>=n)
       down_high_idx = n-1;

       for(int j=max(int(up_jump-up_angle/incre),0);j<=min(int(up_jump+up_angle/incre),n-1); ++j){
         double dis = old_points[j].distToPoint2(&trans_points[i]);
         if(dis<best_dis){
           best_dis = dis;
           best = j;
           second_best = j-1;
           if(second_best == -1){
             second_best = 1;
           }
           last_best = best;
         }
       }

       for(int j=max(int(down_jump-down_angle/incre),0);j<min(int(down_jump+down_angle/incre),n-1); ++j){
         double dis = old_points[j].distToPoint2(&trans_points[i]);
         if(dis<best_dis){
           best_dis = dis;
           best = j;
           second_best = j-1;
           if(second_best == -1){
             second_best = 1;
           }
           last_best = best;
         }
       }




    }

    // int best = 0;
    // int second_best = best-1;

  c.push_back(Correspondence(&trans_points[i], &points[i], &old_points[best], &old_points[second_best]));
    }

  }


void computeJump(vector< vector<int> >& table, vector<Point>& points){
  table.clear();
  int n = points.size();
  for(int i = 0; i<n; ++i){
    vector<int> v = {n,n,-1,-1};
    for(int j = i+1; j<n; ++j){
      if(points[j].r<points[i].r){
        v[UP_SMALL] = j;
        break;
      }
    }
    for(int j = i+1; j<n; ++j){
      if(points[j].r>points[i].r){
        v[UP_BIG] = j;
        break;
      }
    }
    for(int j = i-1; j>=0; --j){
      if(points[j].r<points[i].r){
        v[DOWN_SMALL] = j;
        break;
      }
    }
    for(int j = i-1; j>=0; --j){
      if(points[j].r>points[i].r){
        v[DOWN_BIG] = j;
        break;
      }
    }
    table.push_back(v);
  }
}
