#include "scan_matching_skeleton/correspond.h"
#include "cmath"
#include "ros/ros.h"

using namespace std;

const int UP_SMALL = 0;
const int UP_BIG = 1;
const int DOWN_SMALL = 2;
const int DOWN_BIG = 3;

const int LAST_BEST_idx=0;
const int LAST_LOW_idx=1;
const int LAST_HIGH_idx=2;

void getNaiveCorrespondence(vector<Point>& old_points, vector<Point>& trans_points, vector<Point>& points,
                        vector< vector<int> >& jump_table, vector<Correspondence>& c, float prob,vector<int> &best_index_naive, vector< vector<int> >& index_table_naive){
      c.clear();
      best_index_naive.clear();


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
        // best_index_naive.push_back(min_index);
        c.push_back(Correspondence(&trans_points[i], &points[i], &old_points[min_index], &old_points[second_min_index]));
      }
}

void getCorrespondence(vector<Point>& old_points, vector<Point>& trans_points, vector<Point>& points,
                        vector< vector<int> >& jump_table, vector<Correspondence>& c, float prob,float incre,vector<int> &best_index_smart, vector< vector<int> >& index_table_smart){

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
  best_index_smart.clear();
  index_table_smart.clear();
  // index_table_smart[LAST_LOW_idx].clear();
  // index_table_smart[LAST_HIGH_idx].clear();


  int last_best = -1;
  double prev_point_ang=-1.0;
  const int m = trans_points.size();
  const int n = old_points.size();
  int last_low_idx;
  int last_high_idx;

  //ROS_INFO("size of m : %d, size of n: %d",m,n);
  //angle_incre:0.005823
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
      // ROS_INFO("Smart : 0th best index : %d, i : %d",best,i);

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
      // if(i==30)ROS_INFO("%dth last best point is %d",i,last_best);
      // else if(i==300) ROS_INFO("%dth last best point is %d",i,last_best);
      // else if(i==500) ROS_INFO("%dth last best point is %d",i,last_best);
      double  point_dis = trans_points[i].r;
      double  point_ang = trans_points[i].theta+M_PI;
      
      // double  up_dis = old_points[up_jump].distToPoint2(&trans_points[i]);
      double  last_best_dis = old_points[last_best].distToPoint(&trans_points[i]);
      // double  up_angle =0;
      double  last_angle = 0;
      

      if (point_dis*point_dis - last_best_dis*last_best_dis > 0){
        last_angle =  atan2(last_best_dis, sqrt(point_dis*point_dis - last_best_dis*last_best_dis));
        // if(last_angle < incre && (i==30 || i==300 || i==500)) ROS_INFO("TOO SMALL in %dth",i);
        last_low_idx = int((point_ang - last_angle)/incre)-2; //start angle = -3.1415~
        last_high_idx = int((point_ang + last_angle)/incre)+2;
      }
      else{
        
        // if (i%10==0) {ROS_INFO("%dth is TOO BIG",i);}
        // ROS_INFO("point distance : %f, last best distance : %f",point_dis, last_best_dis);
        last_angle = M_PI;//3.1415926
        last_low_idx = int((point_ang - last_angle)/incre); //start angle = -3.1415~
        last_high_idx = int((point_ang + last_angle)/incre);
      }
      ///////////////////////////////////////////need to change
        if(last_low_idx<0)
        last_low_idx += 1080;
        if(last_high_idx>=n)
        // last_high_idx=n-1;
        last_high_idx -=1080;
      double last_low_dis =old_points[last_low_idx].distToPoint(&trans_points[i]);
      if(prev_point_ang==-1){
        prev_point_ang = point_ang;
      }
      //ROS_INFO("%dth point angle-prev_point_ang: %f",i,point_ang-prev_point_ang);

     


      //add radius of up_idx
      // int up_idx_rad; //hand-made
      // if(int(point_ang/incre)-up_low_idx > up_high_idx-int(point_ang/incre)){//change
      //   up_idx_rad=int(point_ang/incre)-up_low_idx;
      // }
      // else{
      //   up_idx_rad=up_high_idx-int(point_ang/incre);
      // }

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


      //add radius of down_idx
      // int down_idx_rad; //hand made
      // if(int(point_ang/incre)-down_low_idx > down_high_idx-int(point_ang/incre)){
      //   down_idx_rad=int(point_ang/incre)-down_low_idx;
      // }
      // else{
      //   down_idx_rad=down_high_idx-int(point_ang/incre);
      // }
      //campare which index is larger
      // if(up_idx_rad>down_idx_rad){
      //   for(int j=max(int(up_jump-up_angle/incre),0);j<=min(int(up_jump+up_angle/incre),n-1); ++j){
      //   // for(int j=max(int(point_ang/incre-up_angle/incre),0);j<=min(int(point_ang/incre+up_angle/incre),n-1); ++j){
      //     double dis = old_points[j].distToPoint2(&trans_points[i]);
      //     if(dis<best_dis){
      //       best_dis = dis;
      //       best = j;
      //       second_best = j-1;
      //       if(second_best == -1){
      //         second_best = 1;
      //       }
      //       last_best = best;
      //     }
      //   }
      // }
      // else{
      //   for(int j=max(int(down_jump-down_angle/incre),0);j<min(int(down_jump+down_angle/incre),n-1); ++j){
      //     double dis = old_points[j].distToPoint2(&trans_points[i]);
      //     if(dis<best_dis){
      //       best_dis = dis;
      //       best = j;
      //       second_best = j-1;
      //       if(second_best == -1){
      //         second_best = 1;
      //       }
      //       last_best = best;
      //     }
      //   }
      // }
/*
      int up_idx_diff= up_high_idx-up_low_idx;
      int down_idx_diff= down_high_idx-down_low_idx;
      double dis=0;
      if(i=30) {ROS_INFO("up : %d,     %d",up_high_idx,up_low_idx);
                  ROS_INFO("down : %d,     %d",down_high_idx,down_low_idx);
                  ROS_INFO("point ang : %f", point_ang);
      }

      if(up_idx_diff>=down_idx_diff){
        for(int j= up_low_idx; j<=up_high_idx; j++){
          dis= old_points[j].distToPoint2(&trans_points[i]);
          // if(i==30) ROS_INFO("%ith dis is %d",i,dis);
          if(dis<best_dis){
            best_dis=dis;
            best=j;
            second_best=j-1;
            if(second_best==-1) second_best=1;
            last_best=best;
          }
        }
      }
      else {
        for(int j= down_low_idx; j<=down_high_idx; j++){
          dis= old_points[j].distToPoint2(&trans_points[i]);
          //  if(i==30) ROS_INFO("%ith dis is %d",i,dis);
          // if(i==20) ROS_INFO("%ith dis is %d",i,dis);
          if(dis<best_dis){
            best_dis=dis;
            best=j;
            second_best=j-1;
            if(second_best==-1) second_best=1;
            last_best=best;
          }
        }
      }
*/ // gunhee diff
      
        double dis = 0;
        if(last_high_idx>last_low_idx){ 
        for(int j=last_low_idx; j<=last_high_idx; ++j){
          dis= old_points[j].distToPoint2(&trans_points[i]);
          // if(i==30) ROS_INFO("%ith dis is %d",i,dis);
          if(dis<best_dis){
            best_dis=dis;
            best=j;
            second_best=j-1;
            if(second_best==-1) second_best=1;
            last_best=best;
          }

        }
        }
        else{
          for(int j=0;j<=last_high_idx;++j){
            dis= old_points[j].distToPoint2(&trans_points[i]);
          // if(i==30) ROS_INFO("%ith dis is %d",i,dis);
            if(dis<best_dis){
              best_dis=dis;
              best=j;
              second_best=j-1;
              if(second_best==-1) second_best=1;
              last_best=best;
              }
          } 
          for(int j=last_low_idx;j<1080;++j){
            dis= old_points[j].distToPoint2(&trans_points[i]);
          // if(i==30) ROS_INFO("%ith dis is %d",i,dis);
          if(dis<best_dis){
            best_dis=dis;
            best=j;
            second_best=j-1;
            if(second_best==-1) second_best=1;
            last_best=best;
            }
          }
          
        }






      // if(i==300){
      //   ROS_INFO("%ith dis is %d",i,dis);
      //   ROS_INFO("%ith Point's best point is %i ",i,best); 
      // }
      // else if(i==400){
      //   ROS_INFO("%ith dis is %d",i,dis);
      //   ROS_INFO("%ith Point's best point is %i ",i,best);
      // }
    //ROS_INFO("Starting Correspond!!!");
      // for(int j=max(int(up_jump-up_angle/incre),0);j<=min(int(up_jump+up_angle/incre),n-1); ++j){
      //     double dis = old_points[j].distToPoint2(&trans_points[i]);
      //     if(dis<best_dis){
      //       best_dis = dis;
      //       best = j;
      //       second_best = j-1;
      //       if(second_best == -1){
      //         second_best = 1;
      //       }
      //       last_best = best;
      //     }
      //   }

      //   for(int j=max(int(down_jump-down_angle/incre),0);j<min(int(down_jump+down_angle/incre),n-1); ++j){
      //     double dis = old_points[j].distToPoint2(&trans_points[i]);
      //     if(dis<best_dis){
      //       best_dis = dis;
      //       best = j;
      //       second_best = j-1;
      //       if(second_best == -1){
      //         second_best = 1;
      //       }
      //       last_best = best;
      //     }
      //   }
      prev_point_ang = point_ang;
    }

    // int best = 0;
    // int second_best = best-1;
    // vector <int> BLH = {last_best, last_low_idx, last_high_idx};


    // index_table_smart.push_back(BLH);
    // best_index_smart.push_back(best);
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
