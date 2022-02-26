#include "scan_matching_skeleton/correspond.h"
#include "cmath"
#include "ros/ros.h"

using namespace std;

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

void getSmartCorrespondence(vector<Point>& old_points, vector<Point>& trans_points, vector<Point>& points,
                        vector< vector<int> >& jump_table, vector<Correspondence>& c, float prob,float incre){

  c.clear();

  int last_best = -1;
  double prev_point_ang=-1.0;
  const int m = trans_points.size();
  const int n = old_points.size();
  int last_low_idx;
  int last_high_idx;

  //angle_incre:0.005823
  //Do for each point
  for(int i = 0; i<m; ++i){
    double best_dis = 10000000.00;
    int best = -1;
    int second_best = -1;

    if(last_best == -1){      //when i=0
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
      
      double  point_dis = trans_points[i].r;
      double  point_ang = trans_points[i].theta+M_PI;
      
      double  last_best_dis = old_points[last_best].distToPoint(&trans_points[i]);
      double  last_angle = 0;
      
      if (point_dis*point_dis - last_best_dis*last_best_dis > 0){
        last_angle =  atan2(last_best_dis, sqrt(point_dis*point_dis - last_best_dis*last_best_dis));
        last_low_idx = int((point_ang - last_angle)/incre)-2; //start angle = -3.1415~
        last_high_idx = int((point_ang + last_angle)/incre)+2;
      }

      else{
        last_angle = M_PI;//3.1415926
        last_low_idx = int((point_ang - last_angle)/incre); //start angle = -3.1415~
        last_high_idx = int((point_ang + last_angle)/incre);
      }
      
      if(last_low_idx<0) last_low_idx += 1080;
      if(last_high_idx>=n) last_high_idx -=1080;

      double last_low_dis =old_points[last_low_idx].distToPoint(&trans_points[i]);
      
      double dis = 0;
      if(last_high_idx>last_low_idx){ 
        for(int j=last_low_idx; j<=last_high_idx; ++j){
          dis= old_points[j].distToPoint2(&trans_points[i]);
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
          if(dis<best_dis){
            best_dis=dis;
            best=j;
            second_best=j-1;
            if(second_best==-1) second_best=1;
            last_best=best;
            }
        }  
      
      }
    
    }

    c.push_back(Correspondence(&trans_points[i], &points[i], &old_points[best], &old_points[second_best]));
  }

}
