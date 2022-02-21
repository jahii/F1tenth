#include "scan_matching_skeleton/correspond.h"
#include "cmath"
#include "math.h"
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
        best_index_naive.push_back(min_index);
        c.push_back(Correspondence(&trans_points[i], &points[i], &old_points[min_index], &old_points[second_min_index]));
      }
}

void getCorrespondence(vector<Point>& old_points, vector<Point>& trans_points, vector<Point>& points,
                        vector< vector<int> >& jump_table, vector<Correspondence>& c, float prob,float incre,vector<int> &best_index_smart, 
                        vector< vector<int> >& index_table_smart, vector< vector<double> >& distance_table,vector<int> &start_table,vector<vector<double>>& angle_table){

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
  start_table.clear();
  angle_table.clear();
  


  int last_best = -1;
  double prev_point_ang=-1.0;
  const int m = trans_points.size();
  const int n = old_points.size();
  int last_low_idx;
  int last_high_idx;
  vector <int> up_to_down;
  vector <double> distances = {-1,-1,-1,-1};
  vector <double> angles = {-1,-1,-1};
  

  for(int i = 0; i<m; ++i){
    up_to_down.clear();
    
    double best_dis = 10000000.00;
    int best = -1;
    int second_best = -1;
    double  point_dis = trans_points[i].r; angles[0]=point_dis;
    double  point_ang = trans_points[i].theta+M_PI; 

    int start_index = round(point_ang/incre);
    start_table.push_back(start_index);
    int we_start_at = (last_best!=-1)? (last_best) : start_index; //last_best+1 

    int up_check = we_start_at+1;
    int down_check = we_start_at;

    double last_dist_up = 0;
    double last_dist_down = -1;

    bool up_stopped=false, down_stopped=false;
    // ROS_INFO("%dth scan",i);
    up_to_down.push_back(last_best);


    while(!(up_stopped && down_stopped)){

      // bool now_up = !up_stopped;
      bool now_up = up_stopped ? 0 :
                  down_stopped ? 1 : last_dist_up<last_dist_down;
      // ROS_INFO("now up : %s",  now_up? "true" : "false");
      // ROS_INFO("up_stopped : %s",up_stopped? "true" : "false");
      // ROS_INFO("down_stopped : %s",down_stopped? "true" : "false");

      if(now_up){
        if(up_check >= n){up_stopped=true;continue;}
        last_dist_up = old_points[up_check].distToPoint2(&trans_points[i]);
        if(last_dist_up<best_dis) {best = up_check; best_dis = last_dist_up; up_to_down.push_back(best);}
        if(up_check>start_index){
          double del_theta = up_check*incre-point_ang;
          double min_dist_up = sin(del_theta)*point_dis;
          if(pow(min_dist_up,2)>best_dis){
            up_stopped=true; distances[0]=pow(min_dist_up,2); distances[1]=best_dis; angles[1]=del_theta; continue;
          }
          if(old_points[up_check].r<point_dis){up_check = jump_table[up_check][UP_BIG];
          }else{up_check = jump_table[up_check][UP_SMALL];}
        }else{
          up_check++;
        }
      }else{
        if(down_check < 0){down_stopped=true;continue;}
        last_dist_down = old_points[down_check].distToPoint2(&trans_points[i]);
        if(last_dist_down<best_dis) {best = down_check; best_dis = last_dist_down; up_to_down.push_back(best);}
        if(down_check<=start_index){
          double del_theta = point_ang-down_check*incre; 
          double min_dist_down = sin(del_theta)*point_dis;

          if(pow(min_dist_down,2)>best_dis){
            down_stopped=true; distances[2]=pow(min_dist_down,2); distances[3]=best_dis; angles[2]=del_theta; continue;
          }
          if(old_points[down_check].r<point_dis){down_check = jump_table[down_check][DOWN_BIG];
          }else{down_check = jump_table[down_check][DOWN_SMALL];}
        }else{
          down_check--;
        }
      }
    }

    last_best = best;
    second_best = last_best-1;
    if (second_best <=-1) second_best = 1;
    best_index_smart.push_back(best);
    distance_table.push_back(distances);
    angle_table.push_back(angles);
    
    index_table_smart.push_back(up_to_down);
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
