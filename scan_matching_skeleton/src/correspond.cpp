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

//Debugging index in jump-table
const int MIN_DIST_UP=0;
const int MIN_DIST_DOWN=1;
const int BEST_DIST_UP=2;
const int BEST_DIST_DOWN=3;
const int POINT_DIST=4;
const int SIN_UP=5;
const int SIN_DOWN=6;
const int UP_DELTA=7;
const int DOWN_DELTA=8;
const int MIN_DIST_UP_SQUARE=9;
const int MIN_DIST_DOWN_SQUARE=10;
const int DISTANCE_TO_BEST=11;
const int DISTANCE_TO_BEST_SEC=12;
const int OUT_RANGE=13;

//Debugging index in naive
const int MIN_DIST_NAIVE=0;
const int MIN_DIST_NAIVEPlus1=1;


void getNaiveCorrespondence(vector<Point>& old_points, vector<Point>& trans_points, vector<Point>& points,
                        vector< vector<int> >& jump_table, vector<Correspondence>& c, float prob,vector<int> &best_index_naive, vector< vector<int> >& index_table_naive,
                        vector<vector<double>>&debugging_table_naive){
      c.clear();
      best_index_naive.clear();
      debugging_table_naive.clear();

      int last_best = -1;
      const int n = trans_points.size();
      const int m = old_points.size();
      float min_dist = 100000.00;
      int min_index = 0;
      int second_min_index = 0;
      double theta_jump =-1;
      double best_dis_plus = -1;
      vector<double> debugs={-1,-1};
      

      //Do for each point
      for(int i = 0; i<n; ++i){
        double  point_dis = trans_points[i].r;
        min_dist = 100000.00;
        for(int j = 0; j<m; ++j){
          float dist = old_points[j].distToPoint2(&trans_points[i]);
          if(dist<=min_dist){
            min_dist = dist;
            min_index = j;
            second_min_index = j-1;
            debugs[MIN_DIST_NAIVE]=dist;
            if(min_index<m-1) best_dis_plus = old_points[min_index+1].distToPoint2(&trans_points[i]);
            theta_jump=acos((best_dis_plus+pow(old_points[min_index+1].r,2)-pow(point_dis,2))/(2*sqrt(best_dis_plus)*old_points[min_index+1].r));
            debugs[MIN_DIST_NAIVEPlus1]=theta_jump; 
          }
        }
        // ebugs[MIN_DIST_NAIVEPlus1]=d
        
        debugging_table_naive.push_back(debugs);
        best_index_naive.push_back(min_index);
        c.push_back(Correspondence(&trans_points[i], &points[i], &old_points[min_index], &old_points[second_min_index]));
      }
}

void getCorrespondence(vector<Point>& old_points, vector<Point>& trans_points, vector<Point>& points,
                        vector< vector<int> >& jump_table, vector<Correspondence>& c, float prob,float incre,vector<int> &best_index_smart, 
                        vector< vector<int> >& index_table_smart, vector< vector<double> >& debugging_table,vector<int> &start_table){

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
  debugging_table.clear();


  int last_best = -1;
  double prev_point_ang=-1.0;

  const int m = trans_points.size();
  const int n = old_points.size();
  int last_low_idx;
  int last_high_idx;
  vector <int> up_to_down;
  vector <double> debugs={-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,-1,-1};


  for(int i = 0; i<m; ++i){
    up_to_down.clear();
    debugs[OUT_RANGE]=-1;
    double best_dis = old_points[last_best].distToPoint2(&trans_points[i]);
    int best = -1;
    int second_best = -1;
    double  point_dis = trans_points[i].r; debugs[POINT_DIST]=point_dis;
    double  point_ang = trans_points[i].theta+M_PI;
    double theta_jump =-1;

    int start_index = int(point_ang/incre);
    start_table.push_back(start_index);
    //int we_start_at = (last_best!=-1)? (last_best+1) : start_index; //last_best+1 
    int we_start_at = start_index; 

    int up_check = we_start_at+1;
    int down_check = we_start_at;
    if(up_check==1080) up_check--;
    // if(down_check==-1) down_check++;

    double last_dist_up = -1;
    double last_dist_down = -2;
    double del_theta_down = -1;
    double del_theta_up=-1;

    bool up_stopped=false, down_stopped=false;
    // ROS_INFO("%dth scan",i);
    up_to_down.push_back(last_best);
    int count = 0;
    bool up_out = false;
    bool down_out = false;
    bool last_best_check=false;

    while(!(up_stopped && down_stopped)){
      
      // bool now_up = !up_stopped;
      bool now_up = up_stopped ? 0 :
                  down_stopped ? 1 : last_dist_up<last_dist_down;
      // ROS_INFO("now up : %s",  now_up? "true" : "false");
      // ROS_INFO("up_stopped : %s",up_stopped? "true" : "false");
      // ROS_INFO("down_stopped : %s",down_stopped? "true" : "false");

      if(now_up){
        up_to_down.push_back(up_check);

        if(!up_out&&(up_check >= n)){
          up_out = true;
          up_check = 0;
          debugs[OUT_RANGE]=1;continue;
        }
        if(up_out&&(up_check>start_index)){
          up_stopped=true; continue;
        }
        last_dist_up = old_points[up_check].distToPoint2(&trans_points[i]);
        if(last_dist_up<=best_dis) {best = up_check; best_dis = last_dist_up;} 

        del_theta_up = abs(point_ang-(up_check)*incre);
        if(del_theta_up > M_PI){del_theta_up = 2*M_PI-del_theta_up;}
        del_theta_up = del_theta_up>=0.5*M_PI ? 0.5*M_PI : del_theta_up;

        double min_dist_up = abs(sin(del_theta_up)*point_dis);
        if(pow(min_dist_up,2)>best_dis){
          up_stopped=true; 
          // debugs[MIN_DIST_UP]=min_dist_up; debugs[SIN_UP]=abs(sin(del_theta_up));debugs[BEST_DIST_UP]=best_dis;
          // debugs[UP_DELTA]=del_theta_up;debugs[MIN_DIST_UP_SQUARE]=pow(min_dist_up,2);
          continue;
        }

        theta_jump=acos((last_dist_up+pow(old_points[up_check].r,2)-pow(point_dis,2))/(2*sqrt(last_dist_up)*old_points[up_check].r));
        if(theta_jump>0.5*M_PI){
          up_check = jump_table[up_check][UP_BIG];
        }else if(theta_jump<0.5*M_PI){
          up_check = jump_table[up_check][UP_SMALL];
        }else{up_check++;}

      }else{ // !now_up
        up_to_down.push_back(down_check);

        if(!down_out&&(down_check < 0)){
          down_out=true;
          down_check=1079;
          debugs[OUT_RANGE]=1;continue;
        }
        if(down_out&&(down_check<start_index)){
          down_stopped=true; continue;
        }
        last_dist_down = old_points[down_check].distToPoint2(&trans_points[i]);
        if(last_dist_down<=best_dis) {best = down_check; best_dis = last_dist_down;}

        del_theta_down = abs(point_ang-(down_check)*incre);
        if(del_theta_down > M_PI){del_theta_down = 2*M_PI-del_theta_down;}
        del_theta_down = del_theta_down>=0.5*M_PI ? 0.5*M_PI : del_theta_down;

        double min_dist_down = abs(sin(del_theta_down)*point_dis);
        if(pow(min_dist_down,2)>best_dis){
            down_stopped=true; 
            // debugs[MIN_DIST_DOWN]=min_dist_down; debugs[SIN_DOWN]=abs(sin(del_theta_down)); debugs[BEST_DIST_DOWN]=best_dis;
            // debugs[DOWN_DELTA]=del_theta_down;debugs[MIN_DIST_DOWN_SQUARE]=pow(min_dist_down,2);
            continue;
        }

        theta_jump=acos((last_dist_down+pow(old_points[down_check].r,2)-pow(point_dis,2))/(2*sqrt(last_dist_down)*old_points[down_check].r));
        if(theta_jump>0.5*M_PI){
          down_check = jump_table[down_check][DOWN_BIG];
        }else if(theta_jump<0.5*M_PI){
          down_check = jump_table[down_check][DOWN_SMALL];
        }else{down_check--;}
      }
    }
    debugs[DISTANCE_TO_BEST]=old_points[best].distToPoint2(&trans_points[i]);
    // debugs[DISTANCE_TO_BEST_SEC]=old_points[best-1].distToPoint2(&trans_points[i]);
    last_best = best;
    second_best = last_best-1;
    if(second_best<0){second_best=last_best+1;}
    best_index_smart.push_back(best);

    debugging_table.push_back(debugs);
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
      // if(j==1079){
      //   for(int k=0; k<i; ++k){
      //     if(points[k].r<=points[i].r){
      //       v[UP_SMALL]=k;
      //       break;
      //     }
      //   }
        
      // }
    }
    for(int j = i+1; j<n; ++j){
      if(points[j].r>=points[i].r){
        v[UP_BIG] = j;
        break;
      }
      // if(j==1079){
      //   for(int k=0; k<i; ++k){
      //     if(points[k].r>=points[i].r){
      //       v[UP_BIG]=k;
      //       break;
      //     }
      //   }
        
      // }
    }
    for(int j = i-1; j>=0; --j){
      if(points[j].r<=points[i].r){
        v[DOWN_SMALL] = j;
        break;
      }
      // if(j==0){
      //   for(int k=1079; k>i; --k){
      //     if(points[k].r<points[i].r){
      //       v[DOWN_SMALL]=k;
      //       break;
      //     }
      //   }
        
      // }
    }
    for(int j = i-1; j>=0; --j){
      if(points[j].r>=points[i].r){
        v[DOWN_BIG] = j;
        break;
      }
      // if(j==0){
      //   for(int k=1079; k>i; --k){
      //     if(points[k].r>points[i].r){
      //       v[DOWN_BIG]=k;
      //       break;
      //     }
      //   }
        
      // }
    }
    table.push_back(v);
  }
}

// void computeJump(vector< vector<int> >& table, vector<Point>& points){
//   table.clear();
//   int n = points.size();
//   for(int i = 0; i<n; ++i){
//     vector<int> v = {n,n,-1,-1};
//     for(int j = i+1; j<n; ++j){
//       if(points[j].r==points[i].r){
//         cout<<"up_small "<<i<<"th "<<j<<endl;
//       }
//     }
//     for(int j = i+1; j<n; ++j){
//       if(points[j].r==points[i].r){
//         cout<<"up_big "<<i<<"th "<<j<<endl;
//       }
//     }
//     for(int j = i-1; j>=0; --j){
//       if(points[j].r==points[i].r){
//         cout<<"down_small "<<i<<"th "<<j<<endl;
//       }
//     }
//     for(int j = i-1; j>=0; --j){
//       if(points[j].r==points[i].r){
//         cout<<"down_big "<<i<<"th "<<j<<endl;
//       }
//     }
//     table.push_back(v);
//   }
// }

