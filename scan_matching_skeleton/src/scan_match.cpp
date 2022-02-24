#include <sstream>
#include <string>
#include <cmath>

#include "ros/ros.h"
#include <tf2/impl/utils.h>
#include "sensor_msgs/LaserScan.h"
#include <geometry_msgs/PoseStamped.h>
#include "scan_matching_skeleton/correspond.h"
#include "scan_matching_skeleton/transform.h"
#include "scan_matching_skeleton/visualization.h"
#include <tf/transform_broadcaster.h>
#include "scan_matching_skeleton/time_pub.h"
#include "scan_matching_skeleton/car_state.hpp"


using namespace std;

const string& TOPIC_SCAN  = "/scan";
const string& TOPIC_POS = "/scan_match_location";
const string& TOPIC_RVIZ = "/scan_match_debug";
const string& FRAME_POINTS = "laser";
const string& TOPIC_TIME= "/corr_time";


const float RANGE_LIMIT = 10.0;

const float MAX_ITER = 30.0;  
const float MIN_INFO = 0.1;
const float A = (1-MIN_INFO)/MAX_ITER/MAX_ITER;
const float error_per = 5.0;
int zero_index_smart=0;
int zero_index_naive=0;
int before_naive_time;
int middle_time;
int after_smart_time;



class ScanProcessor {
  private:
    ros::Publisher pos_pub;
    ros::Publisher marker_pub;
    ros::Publisher pre_pub;
    ros::Publisher time_pub;

    ros::Subscriber pose_sub;
    ros::Subscriber pose_rviz_sub;
    

    vector<Point> points;
    vector<Point> transformed_points;
    vector<Point> prev_points;
    vector<Correspondence> corresponds_smart;
    vector<Correspondence> corresponds_naive;
    vector< vector<int> > jump_table;
    vector< vector<int> > index_table_smart;
    vector< vector<int> > index_table_naive;
    vector<int> best_index_smart;
    vector<int> best_index_naive;
    Transform prev_trans, curr_trans;
    // tf::TransformBroadcaster br;
    // tf::Transform tr;

    PointVisualizer* points_viz;
    PointVisualizer* prepoints_viz;
    CorrespondenceVisualizer* corr_viz;

    geometry_msgs::PoseStamped msg;
    Eigen::Matrix3f global_tf;
    Eigen::Matrix3f finished_tf;
    Eigen::Vector3f pose_update;
    Eigen::Matrix3f roto_translation_mat;
    Eigen::Vector3f curr_matrix;
    Eigen::Vector3f new_pose;

    double ground_x=0;
    double ground_y=0;
    double ground_theta=0;
    double icp_theta=0;
    float initial_pose_x=-5;
    float initial_pose_y=8.5;
    float x_old=-1;
    float y_old=-1;
    float x_pose;
    float y_pose;
    float x_increase;
    float y_increase;
    float theta_update;
    


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
       
      // global_tf<< 1, 0, -5, 0, 1, 8.5, 0, 0, 1;
      pose_update<< -5, 8.5,  1;
      x_pose=-5;
      y_pose=8.5;
      
      // x_increase=0;
      // y_increase=0;
      
      pose_sub=n.subscribe("/gt_pose",1,&ScanProcessor::pose_callback,this);
      // pose_rviz_sub = n.subscribe("/initialpose", 1, &ScanProcessor::pose_rviz_callback, this);
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
      // ROS_INFO("Starting Optimization!!!");
      finished_tf = Eigen::Matrix3f::Identity(3,3);

      curr_trans = Transform();
      curr_matrix<<0,0,0;
      theta_update=0.0;
      while (count < MAX_ITER && ( icp_correct==false || count==0)) {
        transformPoints(points, curr_trans, transformed_points);

        getNaiveCorrespondence(prev_points, transformed_points, points, jump_table, corresponds_naive, A*count*count+MIN_INFO, best_index_naive, index_table_naive);

        //getCorrespondence(prev_points, transformed_points, points, jump_table, corresponds_smart, A*count*count+MIN_INFO,msg->angle_increment, best_index_smart, index_table_smart);

        prev_trans = curr_trans;
        ++count;
      
        updateTransform(corresponds_naive, curr_trans);

        x_error = (curr_trans.x_disp-prev_trans.x_disp)/prev_trans.x_disp*100;
        y_error = (curr_trans.x_disp-prev_trans.x_disp)/prev_trans.x_disp*100;
        theta_error = (curr_trans.x_disp-prev_trans.x_disp)/prev_trans.x_disp*100;
        
        if (abs(x_error)<=error_per&&abs(y_error)<=error_per&&abs(theta_error)<=error_per) icp_correct=true;
        
        //this->finished_tf=curr_trans.getMatrix()*finished_tf;

        curr_matrix(0)+=curr_trans.x_disp;
        curr_matrix(1)+=curr_trans.y_disp;
        theta_update=atan2(curr_matrix(1),curr_matrix(0));
        while(theta_update>M_PI) theta_update-=2*M_PI;
        while(theta_update<M_PI) theta_update+=2*M_PI;
        curr_matrix(2)=theta_update;
        


        cout<<count<<"th curr_trans_x is "<<curr_trans.x_disp<<" "<<ground_x<<endl;
        cout<<count<<"th update_trans_x is"<<curr_matrix(0)<<endl;


        
        
      }

      col.r = 0.0; col.b = 0.0; col.g = 1.0; col.a = 1.0;
      points_viz->addPoints(transformed_points, col);
      points_viz->publishPoints();
      
      
      ROS_INFO("Count: %i", count);
      ROS_INFO("x_error :%f", x_error);


      // if(x_old ==-1) x_old=initial_pose_x;
      // if(y_old ==-1) y_old=initial_pose_y;
      
      // roto_translation_mat << cos(ground_theta),-sin(ground_theta),x_old,sin(ground_theta),cos(ground_theta),y_old,0,0,1;
      // curr_matrix << curr_matrix(0),curr_matrix(1),1;
      // this->pose_update=roto_translation_mat*curr_matrix;
      // x_old=pose_update(0);
      // y_old=pose_update(1);

      // this->x_pose=x_pose+global_tf(0,2);
      // this->y_pose=y_pose+global_tf(1,2);
      // cout<<x_pose<<" "<< y_pose<<endl;

      // // this->pose_update=finished_tf*pose_update;
      // this->global_tf = global_tf * curr_trans.getMatrix();
      // if(curr_trans.x_disp>0) cout<<"curr_trans.x_disp"<<curr_trans.x_disp<<endl;
      //  cout<<curr_trans.x_disp<<" "<<curr_trans.y_disp<<" "<<curr_trans.theta_rot<<" "<<ground_theta<<endl;
      // cout<<"ground truth"<<" "<<ground_x<<" "<<ground_y<<" "<<endl;
      // cout<<"trans_x is"<<curr_matrix(0)<<endl;
      // cout<<"icp_algorithm"<<" "<<pose_update(0)<<" "<<pose_update(1)<<endl;
      
      //publishPos();
      posetest();
      prev_points = points;
    }

    // Handles reading of scan, pushes fills points vector
    void readScan(const sensor_msgs::LaserScan::ConstPtr& msg) {
      float range_min = msg->range_min;
      float range_max = msg->range_max;
      float angle_min = msg->angle_min;
      float angle_increment = msg->angle_increment;

      const vector<float>& ranges = msg->ranges;
      points.clear();

      for (int i = 0; i < ranges.size(); ++i) {
        float range = ranges.at(i);
        if (!isnan(range)&&range > RANGE_LIMIT) {
          points.push_back(Point(RANGE_LIMIT, angle_min + angle_increment * i));
          continue;
        }
        if (!isnan(range) && range >= range_min && range <= range_max) {
          points.push_back(Point(range, angle_min + angle_increment * i));
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

    tf2::Quaternion quat(q.x(),q.y(),q.z(),q.w());
    icp_theta=tf2::impl::getYaw(quat);

     msg.header.frame_id = "laser";
     msg.header.stamp = ros::Time::now();
     pos_pub.publish(msg);
    //  tr.setOrigin(tf::Vector3(global_tf(0,2), global_tf(1,2), 0));
    //  tr.setRotation(q);
    //  br.sendTransform(tf::StampedTransform(tr, ros::Time::now(), "map", "laser"));

    }

    
    void posetest(){
      msg.pose.position.x = x_increase; 
      msg.pose.position.y = y_increase;

      msg.pose.orientation.x = 1;
      msg.pose.orientation.y = 1;
      msg.pose.orientation.z = 1;
      msg.pose.orientation.w = 0;

      msg.header.frame_id = "laser";
      msg.header.stamp = ros::Time::now();
      pos_pub.publish(msg);
    }
    
    
    
    
    
    void pose_callback(const geometry_msgs::PoseStamped & msg) {
        ground_x = msg.pose.position.x;
        ground_y = msg.pose.position.y;
        geometry_msgs::Quaternion q = msg.pose.orientation;
        tf2::Quaternion quat(q.x, q.y, q.z, q.w);
        ground_theta = tf2::impl::getYaw(quat);
    }

    // void pose_rviz_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr & msg) {
    //     geometry_msgs::PoseStamped temp_pose;
    //     temp_pose.header = msg->header;
    //     temp_pose.pose = msg->pose.pose;
    //     pose_callback(temp_pose);
    // }




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
