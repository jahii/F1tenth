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
const int DISTANCE_TO_BEST=0;
const int BEST_INDEX = 1;
const int START_INDEX = 2;
const int OPPOSITE_START_INDEX = 3;

//Debugging index in naive
const int MIN_DIST_NAIVE=0;
const int MIN_DIST_NAIVEPlus1=1;
const int MIN_INDEX_NAVIE=2;


void getNaiveCorrespondence(vector<Point>& old_points, vector<Point>& trans_points, vector<Point>& points,
                        vector< vector<int> >& jump_table, vector<Correspondence>& c, float prob,vector< vector<double>>& debugging_table_naive){
      c.clear();
      debugging_table_naive.clear();

      int last_best = -1;
      const int n = trans_points.size();
      const int m = old_points.size();
      float min_dist = 100000.00;
      int min_index = 0;
      int second_min_index = 0;
      vector<double> debugs = {-1,-1,-1};

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
        debugs[MIN_DIST_NAIVE]=min_dist;
        debugs[MIN_INDEX_NAVIE]=min_index;
        debugging_table_naive.push_back(debugs);

        c.push_back(Correspondence(&trans_points[i], &points[i], &old_points[min_index], &old_points[second_min_index]));
      }
}

void SmartJumpCorrespondence(vector<Point>& old_points, vector<Point>& trans_points, vector<Point>& points,
                        vector< vector<int> >& jump_table, vector<Correspondence>& c, float prob, float incre,int& jump_index, vector< vector<double>> &debugging_table_jump,float angle_min, float angle_max){

  c.clear();
  debugging_table_jump.clear();
  angle_min = M_PI+angle_min;
  angle_max = M_PI+angle_max;

  int last_best = 0;
  double prev_point_ang=-1.0;

  const int m = trans_points.size();
  const int n = old_points.size();  
  if(m!=n){ROS_INFO("m and n is different! m : %d, n : %d", m, n);}
  int last_low_idx;
  int last_high_idx;
  vector <int> up_to_down;
  jump_index=0;
  vector <double> debugs={-2, -2, -2, -2};

  for(int i = 0; i<m; ++i){
    up_to_down.clear();
    double best_dis = old_points[last_best].distToPoint2(&trans_points[i]);
    int best = -1;
    int second_best = -1;
    double  point_dis = trans_points[i].r; 
    double  point_ang = trans_points[i].theta+M_PI;
    double up_theta_jump =-1;
    double down_theta_jump=-1;
    int start_index;
    if(point_ang<angle_min){
      start_index = 0;
    }else if(point_ang>angle_max){
      start_index = n-1;
    }else{
      start_index = int((point_ang-angle_min)/incre);
    }

    int opposite_start_index = int(start_index+M_PI/incre);
    if((opposite_start_index>n-1)&&(2*M_PI/incre>opposite_start_index)){
      opposite_start_index=0;
    }else if(opposite_start_index>=2*M_PI/incre){
      opposite_start_index=int(opposite_start_index-2*M_PI/incre);
    }

    debugs[START_INDEX]=start_index;
    debugs[OPPOSITE_START_INDEX]=opposite_start_index;


    int we_start_at = start_index; 

    int up_check = we_start_at+1;
    int down_check = we_start_at;
    if(up_check==n) up_check--;

    double last_dist_up = -1;
    double last_dist_down = -2;
    double del_theta_down = -1;
    double del_theta_up=-1;

    bool up_stopped=false, down_stopped=false;
    up_to_down.push_back(last_best);
    int count = 0;
    bool up_out = false;
    bool down_out = false;
    bool last_best_check=false;


    while(!(up_stopped && down_stopped)){

      jump_index++;

      // bool now_up = !up_stopped;
      bool now_up = up_stopped ? 0 :
                  down_stopped ? 1 : last_dist_up<last_dist_down;

      if(now_up){
        up_to_down.push_back(up_check);
        if(opposite_start_index<start_index){
          if(!up_out&&(up_check >= n)){
          up_out = true;
          up_check = 0;
          continue;
          }
          if(up_out&&(up_check>opposite_start_index)){
            up_stopped=true; continue;
          }
        }else{
          if(up_check>opposite_start_index){
            up_stopped=true; continue;
          }
        }

        last_dist_up = old_points[up_check].distToPoint2(&trans_points[i]);
        if(last_dist_up<=best_dis) { best = up_check; best_dis = last_dist_up;}
        
        del_theta_up = abs(point_ang-((up_check)*incre+angle_min));
        if(del_theta_up > M_PI){del_theta_up = 2*M_PI-del_theta_up;}
        del_theta_up = del_theta_up>=0.5*M_PI ? 0.5*M_PI : del_theta_up;

        double min_dist_up = abs(sin(del_theta_up)*point_dis);
        if(pow(min_dist_up,2)>best_dis){
          up_stopped=true; 
          continue;
        }

        double inverse_val_up = (last_dist_up+pow(old_points[up_check].r,2)-pow(point_dis,2))/(2*sqrt(last_dist_up)*old_points[up_check].r);
        if(inverse_val_up<-1){inverse_val_up=-1;}
        else if(inverse_val_up>1){inverse_val_up=1;}
        up_theta_jump=acos(inverse_val_up);

        if(up_theta_jump>0.5*M_PI){
          up_check = jump_table[up_check][UP_BIG]; up_to_down.push_back(-3);
        }else if(up_theta_jump<0.5*M_PI){
          up_check = jump_table[up_check][UP_SMALL]; up_to_down.push_back(-2);
        }else{ROS_INFO("last_dist_up : %f, 0tochecking point^2 : %f, point_dis^2 : %f, 2*a*b : %f", 
        last_dist_up, pow(old_points[up_check].r,2),pow(point_dis,2), 2*sqrt(last_dist_up)*old_points[up_check].r);}
      }
      
      else{ // !now_up
        up_to_down.push_back(down_check);
        if(opposite_start_index>start_index){
          if(!down_out&&(down_check < 0)){
            down_out=true;
            down_check=n-1;
          }
          if(down_out&&(down_check<opposite_start_index)){
              down_stopped=true; continue;
          }
        }else{
          if(down_check<opposite_start_index){
            down_stopped=true; continue;
          }
        }

        last_dist_down = old_points[down_check].distToPoint2(&trans_points[i]);
        if((last_dist_down<=best_dis)) {best = down_check; best_dis = last_dist_down;}

        del_theta_down = abs(point_ang-((down_check)*incre+angle_min));
        if(del_theta_down > M_PI){del_theta_down = 2*M_PI-del_theta_down;}
        del_theta_down = del_theta_down>=0.5*M_PI ? 0.5*M_PI : del_theta_down;

        double min_dist_down = abs(sin(del_theta_down)*point_dis);
        if(pow(min_dist_down,2)>best_dis){
            down_stopped=true; 
            continue;
        }

        double inverse_val_down = (last_dist_down+pow(old_points[down_check].r,2)-pow(point_dis,2))/(2*sqrt(last_dist_down)*old_points[down_check].r);
        if(inverse_val_down<-1){inverse_val_down=-1;}
        else if(inverse_val_down>1){inverse_val_down=1;}
        down_theta_jump=acos(inverse_val_down);
        
        if(down_theta_jump>0.5*M_PI){
          down_check = jump_table[down_check][DOWN_BIG]; up_to_down.push_back(-5);
        }else if(down_theta_jump<0.5*M_PI){
          down_check = jump_table[down_check][DOWN_SMALL]; up_to_down.push_back(-4);
        }else{ROS_INFO("last_dist_up : %f, 0tochecking point^2 : %f, point_dis^2 : %f, 2*a*b : %f", 
        last_dist_up, pow(old_points[up_check].r,2),pow(point_dis,2), 2*sqrt(last_dist_up)*old_points[up_check].r);}


      }
    }
    debugs[DISTANCE_TO_BEST]=old_points[best].distToPoint2(&trans_points[i]);
    debugs[BEST_INDEX]=best;

    last_best = best;
    second_best = last_best-1;
    if(second_best<0){second_best=last_best+1;}
    debugging_table_jump.push_back(debugs);

    c.push_back(Correspondence(&trans_points[i], &points[i], &old_points[best], &old_points[second_best]));
  
  }
}
  

void originalJumpCorrespondence(vector<Point>& old_points, vector<Point>& trans_points, vector<Point>& points,
                        vector< vector<int> >& jump_table, vector<Correspondence>& c, float prob,float incre, int& original_index,vector< vector<double>> &debugging_table_original,float angle_min, float angle_max){

  c.clear();
  debugging_table_original.clear();
  
  int last_best = -1;
  double prev_point_ang=-1.0;
  const int m = trans_points.size();
  const int n = old_points.size();
  int last_low_idx;
  int last_high_idx;
  vector <int> up_to_down;
  vector <double> debugs={-2, -2, -2, -2};
  
  for(int i = 0; i<m; ++i){
    up_to_down.clear();
    double best_dis = 10000000.00;
    int best = -1;
    int second_best = -1;
    double  point_dis = trans_points[i].r;
    double  point_ang = trans_points[i].theta+M_PI; 
    int start_index;
    if(point_ang<angle_min){
      start_index = 0;
    }else if(point_ang>angle_max){
      start_index = n-1;
    }else{
      start_index = int((point_ang-angle_min)/incre);
    }

    int we_start_at = (last_best!=-1)? (last_best+1) : start_index; //last_best+1 

    int up_check = we_start_at+1;
    int down_check = we_start_at;

    double last_dist_up = -1;
    double last_dist_down = -2;

    bool up_stopped=false, down_stopped=false;
    up_to_down.push_back(last_best);
    int count = 0;
    bool up_out = false;
    bool down_out = false;

    while(!(up_stopped && down_stopped)){
      
      // bool now_up = !up_stopped;
      bool now_up = up_stopped ? 0 :
                  down_stopped ? 1 : last_dist_up<last_dist_down;
      if(now_up){
        up_to_down.push_back(up_check);
        if(!up_out&&(up_check >= n)){
          up_out = true;
          up_check = 0;
          continue;
        }
        if(up_out&&(up_check>=we_start_at)){
          up_stopped=true; continue;
        }
        last_dist_up = old_points[up_check].distToPoint2(&trans_points[i]);
        if(last_dist_up<=best_dis) {best = up_check; best_dis = last_dist_up;}
        if(up_check>start_index){
          double del_theta_up = abs(up_check*incre-point_ang)>=0.5*M_PI ? 0.5*M_PI : up_check*incre-point_ang;
          double min_dist_up = abs(sin(del_theta_up)*point_dis);
          if(pow(min_dist_up,2)>best_dis){
            up_stopped=true; 
            continue;
          }
          if(old_points[up_check].r<point_dis){
            up_check = jump_table[up_check][UP_BIG];
          }else if(old_points[up_check].r>point_dis){
            up_check = jump_table[up_check][UP_SMALL];
          }else{up_check++;}
        }
        else{
          up_check++;
        }
      }else{
        up_to_down.push_back(down_check);
        if(!down_out&&(down_check < 0)){
          down_out=true;
          down_check=n-1;
          continue;}
        if(down_out&&(down_check<=(we_start_at+1))){
          down_stopped=true; continue;}
        last_dist_down = old_points[down_check].distToPoint2(&trans_points[i]);
        if(last_dist_down<best_dis) {best = down_check; best_dis = last_dist_down;}
        if(down_check<start_index){
          double del_theta_down = abs(point_ang-(down_check)*incre)>=0.5*M_PI ? 0.5*M_PI : point_ang-(down_check)*incre; 
          double min_dist_down = abs(sin(del_theta_down)*point_dis);
          if(pow(min_dist_down,2)>best_dis){
            down_stopped=true; 
            continue;
          }
          if(old_points[down_check].r<point_dis){
            down_check = jump_table[down_check][DOWN_BIG];
          }else if(old_points[down_check].r>point_dis){
            down_check = jump_table[down_check][DOWN_SMALL];
          }else{down_check--;}
        }
        else{
        down_check--;
        }
      }
    }
    
    last_best = best;
    second_best = last_best-1;
    if(second_best<0){second_best=last_best+1;}
    debugs[DISTANCE_TO_BEST]=old_points[best].distToPoint2(&trans_points[i]);
    debugs[BEST_INDEX]=best;

    debugging_table_original.push_back(debugs);

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
      if(points[j].r>=points[i].r){
        v[UP_BIG] = j;
        break;
      }
    }
    for(int j = i-1; j>=0; --j){
      if(points[j].r<=points[i].r){
        v[DOWN_SMALL] = j;
        break;
      }
    }
    for(int j = i-1; j>=0; --j){
      if(points[j].r>=points[i].r){
        v[DOWN_BIG] = j;
        break;
      }
    }
    table.push_back(v);
  }
}