#include <sstream>
#include <string>
#include <cmath>

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <geometry_msgs/PoseStamped.h>
#include "scan_matching_skeleton/correspond.h"
#include "scan_matching_skeleton/transform.h"
#include "scan_matching_skeleton/visualization.h"
#include <tf/transform_broadcaster.h>
#include "scan_matching_skeleton/time_pub.h"


using namespace std;

const string& TOPIC_SCAN  = "/scan";
const string& TOPIC_POS = "/scan_match_location";
const string& TOPIC_RVIZ = "/scan_match_debug";
const string& FRAME_POINTS = "laser";
const string& TOPIC_TIME= "/corr_time";

const float RANGE_LIMIT = 30.0; //10

const float MAX_ITER = 30.0;
const float MIN_INFO = 0.1;
const float A = (1-MIN_INFO)/MAX_ITER/MAX_ITER;
const float error_per = 5.0;
int zero_index_smart=0;
int zero_index_naive=0;
int before_naive_time;
int after_naive_time;
int after_smart_time;
int after_jump_time;
int after_original_time;
int naive_index;
int jump_index;
int smart_index;
int original_index;


//Debugging index in jump-table
const int DISTANCE_TO_BEST=0;
const int BEST_INDEX = 1;
const int START_INDEX = 2;
const int OPPOSITE_START_INDEX = 3;

//Debugging index in naive
const int MIN_DIST_NAIVE=0;
const int MIN_DIST_NAIVEPlus1=1;
const int MIN_INDEX_NAVIE=2;


class ScanProcessor {
  private:
    ros::Publisher pos_pub;
    ros::Publisher marker_pub;
    ros::Publisher pre_pub;
    ros::Publisher time_pub;

    vector<Point> points;
    vector<Point> transformed_points;
    vector<Point> prev_points;
    vector<Correspondence> corresponds_smart;
    vector<Correspondence> corresponds_naive;
    vector<Correspondence> corresponds_jump;
    vector<Correspondence> corresponds_original;
    vector< vector<int> > jump_table;
    vector< vector<int> > index_table_smart;
    vector< vector<int> > index_table_naive;
    vector< vector<double> > debugging_table;
    vector< vector<double> > debugging_table_naive;
    vector< vector<double> > debugging_table_jump;
    vector< vector<double> > debugging_table_original;
    vector<int> best_index_smart;
    vector<int> best_index_naive;
    vector<int> start_table;
    Transform prev_trans, curr_trans;
    tf::TransformBroadcaster br;
    tf::Transform tr;

    PointVisualizer* points_viz;
    PointVisualizer* prepoints_viz;
    CorrespondenceVisualizer* corr_viz;

    geometry_msgs::PoseStamped msg;
    Eigen::Matrix3f global_tf;


    std_msgs::ColorRGBA col;

  public:
    ScanProcessor(ros::NodeHandle& n) : curr_trans(Transform()) {
      pos_pub = n.advertise<geometry_msgs::PoseStamped>(TOPIC_POS, 1);
      marker_pub = n.advertise<visualization_msgs::Marker>(TOPIC_RVIZ, 1);
      pre_pub = n.advertise<visualization_msgs::Marker>("/scan_pub", 1);
      time_pub= n.advertise<scan_matching_skeleton::time_pub>("/corr_time",1);

      points_viz = new PointVisualizer(marker_pub, "scan_match", FRAME_POINTS);

      prepoints_viz = new PointVisualizer(pre_pub, "scan_match", FRAME_POINTS);
      
      corr_viz = new CorrespondenceVisualizer(marker_pub, "scan_match", FRAME_POINTS);
      global_tf = Eigen::Matrix3f::Identity(3,3);
    }

    void handleLaserScan(const sensor_msgs::LaserScan::ConstPtr& msg) {
      readScan(msg);

      //We have nothing to compare to!
      if(prev_points.empty()){
        ROS_INFO("First Scan");
        prev_points = points;
        return;
      }

      col.r = 0.0; col.b = 1.0; col.g = 0.0; col.a = 1.0;
      prepoints_viz->addPoints(points, col);
      prepoints_viz->publishPoints();
      // col.r = 1.0; col.b = 0.0; col.g = 0.0; col.a = 1.0;
      // points_viz->addPoints(prev_points, col);
      // points_viz->publishPoints();

      int count = 0;
      float x_error=0.0;
      float y_error=0.0;
      float theta_error=0.0;
      bool icp_correct=false;

      scan_matching_skeleton::time_pub time_msg;

      computeJump(jump_table, prev_points);
      ROS_INFO("Starting Optimization!!!");

      curr_trans = Transform();

      while (count < MAX_ITER && ( icp_correct==false || count==0)) {
        
        transformPoints(points, curr_trans, transformed_points);

        //************************************************ Find correspondence between points of the current and previous frames  *************** ////
        // **************************************************** getCorrespondence() function is the fast search function and getNaiveCorrespondence function is the naive search option **** ////
        //unit : msec
        before_naive_time = ros::Time::now().nsec/100000;
        getNaiveCorrespondence(prev_points, transformed_points, points, jump_table, corresponds_naive, A*count*count+MIN_INFO, debugging_table_naive);
        after_naive_time = ros::Time::now().nsec/100000;
        SmartJumpCorrespondence(prev_points, transformed_points, points, jump_table, corresponds_jump, A*count*count+MIN_INFO,msg->angle_increment, jump_index, debugging_table_jump,msg->angle_min,msg->angle_max);
        after_jump_time = ros::Time::now().nsec/100000;
        originalJumpCorrespondence(prev_points, transformed_points, points, jump_table, corresponds_original, A*count*count+MIN_INFO,msg->angle_increment, original_index, debugging_table_original,msg->angle_min,msg->angle_max);
        after_original_time = ros::Time::now().nsec/100000;

        time_msg.naive_time= after_naive_time - before_naive_time;
        if(time_msg.naive_time<0) time_msg.naive_time+=10000;

        time_msg.new_jumptable_time=after_jump_time-after_naive_time;
        if(time_msg.new_jumptable_time<0) time_msg.new_jumptable_time+=10000;

        // time_msg.original_jump_time=after_original_time-after_jump_time;
        // if(time_msg.original_jump_time<0) time_msg.original_jump_time+=10000;
        
        time_msg.jump_index = jump_index;

        time_pub.publish(time_msg);

        for(int a=0;a<transformed_points.size();a++){
          double dist2best_naive = corresponds_naive[a].p->distToPoint2(corresponds_naive[a].pj1);
          double dist2best_jump = corresponds_jump[a].p->distToPoint2(corresponds_jump[a].pj1);
          double dist2best_original = corresponds_original[a].p->distToPoint2(corresponds_original[a].pj1);
          //compare naive and jump
          /*  
          if (dist2best_naive != dist2best_jump)
          {
            ROS_INFO("%d th points unmatched!!", a);
            cout << "naive : " << corresponds_naive[a].p->distToPoint2(corresponds_naive[a].pj1) << " jump : " << corresponds_jump[a].p->distToPoint2(corresponds_jump[a].pj1) << endl;
            cout << "naive min index : " << debugging_table_naive[a][MIN_INDEX_NAVIE] << " jump min index : " << debugging_table_jump[a][BEST_INDEX] << endl;
            cout<<endl;
          }
          */
          if((dist2best_naive != dist2best_jump)||(dist2best_jump!=dist2best_original))
          {
            ROS_INFO("%d th points unmatched!!", a);
            cout << "naive : " << dist2best_naive << " jump : " << dist2best_jump <<" original : " << dist2best_original << endl;
            cout << "naive best index : " << debugging_table_naive[a][MIN_INDEX_NAVIE] << " jump best index : " << debugging_table_jump[a][BEST_INDEX] <<" orignal best index : " << debugging_table_original[a][BEST_INDEX] << endl;
            cout<<endl<<endl;
          }
        }
        
        prev_trans = curr_trans;
        ++count;
      

        // **************************************** We update the transforms here ******************************************* ////
        updateTransform(corresponds_naive, curr_trans);
        
        x_error = (curr_trans.x_disp-prev_trans.x_disp)/prev_trans.x_disp*100;
        y_error = (curr_trans.x_disp-prev_trans.x_disp)/prev_trans.x_disp*100;
        theta_error = (curr_trans.x_disp-prev_trans.x_disp)/prev_trans.x_disp*100;

        if (abs(x_error)<=error_per&&abs(y_error)<=error_per&&abs(theta_error)<=error_per) icp_correct=true;
        
      }
      
      col.r = 0.0; col.b = 0.0; col.g = 1.0; col.a = 1.0;
      // points_viz->addPoints(transformed_points, col);
      // points_viz->publishPoints();
      

      ROS_INFO("Count: %i", count);
      ROS_INFO("x_error :%f", x_error);

      this->global_tf = global_tf * curr_trans.getMatrix();

      // publishPos();
      prev_points = points;
    }

    // Handles reading of scan, pushes fills points vector
    void readScan(const sensor_msgs::LaserScan::ConstPtr& msg) {
      float range_min = msg->range_min;
      float range_max = msg->range_max;
      float angle_min = msg->angle_min;
      float angle_max = msg->angle_max;
      float angle_increment = msg->angle_increment;
      const vector<float>& ranges =  msg->ranges;
      // ROS_INFO("range_min: %f , range_max: %f , angle_min: %f , angle_max: %f ",range_min,range_max,angle_min,angle_max);

      
      points.clear();

      for (int i = 0; i < ranges.size(); ++i) {
        float range = ranges.at(i); // add noise float(1+3*(float(rand()/RAND_MAX)))*
        if (!isnan(range)&&range > RANGE_LIMIT) {
          points.push_back(Point(RANGE_LIMIT, angle_min + angle_increment * i));
          // cout<<"Its Max"<<endl;
          continue;
        }
        if (!isnan(range) && range >= range_min && range <= range_max) {
          points.push_back(Point(range, angle_min + angle_increment * i));
          // ROS_INFO("angle increment is %f",angle_increment);
        }
        if(!isnan(range)&&range<range_min){
          points.push_back(Point(range_min, angle_min + angle_increment * i));
        }

      }

    }

    void publishPos() {
     msg.pose.position.x = global_tf(0,2);
     msg.pose.position.y = global_tf(1,2);
     msg.pose.position.z = 0;
     tf::Matrix3x3 tf3d;
     tf3d.setValue(static_cast<double>(global_tf(0,0)), static_cast<double>(global_tf(0,1)), 0,
             static_cast<double>(global_tf(1,0)), static_cast<double>(global_tf(1,1)), 0, 0, 0, 1);

     tf::Quaternion q;
     tf3d.getRotation(q);
     msg.pose.orientation.x = q.x();
     msg.pose.orientation.y = q.y();
     msg.pose.orientation.z = q.z();
     msg.pose.orientation.w = q.w();
     msg.header.frame_id = "laser";
     msg.header.stamp = ros::Time::now();
     pos_pub.publish(msg);
     tr.setOrigin(tf::Vector3(global_tf(0,2), global_tf(1,2), 0));
     tr.setRotation(q);
     br.sendTransform(tf::StampedTransform(tr, ros::Time::now(), "map", "laser"));

    }

    ~ScanProcessor() {}
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "scan_matcher");
  ros::NodeHandle n;

  // processor
  ScanProcessor processor(n);

  // SUBSCRIBE
  ros::Subscriber sub = n.subscribe(TOPIC_SCAN, 1,
    &ScanProcessor::handleLaserScan, &processor);

  ros::spin();
  return 0;
}
