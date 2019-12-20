#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>

#include <iostream>
#include <fstream>

using namespace message_filters;

std::ofstream outputfile("test.txt", std::ios::app);

void geometry_quat_to_rpy(double& roll, double& pitch, double& yaw, geometry_msgs::Quaternion geometry_quat){
	tf::Quaternion quat;
	quaternionMsgToTF(geometry_quat, quat);
	tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);  //rpy are Pass by Reference
}

void callback(const nav_msgs::Odometry &odom_msg, const sensor_msgs::LaserScan &scan_msg){
  static int stamp=1;
  int count=0, deg;
  //odom
  double roll, pitch, yaw;

  geometry_quat_to_rpy(roll, pitch, yaw, odom_msg.pose.pose.orientation);
/*
  ROS_INFO("odom : x %lf y %lf th %lf",
  odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y, yaw);
*/
  //scan
/*
  for(int i=0; i<scan_msg.ranges.size(); i++){
    ROS_INFO("%d[deg], %lf, ", i, scan_msg.ranges[i]);
  }
*/

  //file output
  outputfile << "LASERSCAN " << stamp << " 10 10 " << std::flush;
  for(int i=0; i<scan_msg.ranges.size(); i++){
    if(std::isinf(scan_msg.ranges[i]) == 0){
      count++;
    }
  }
  outputfile << count << " " << std::flush;
  deg = -180;
  for(int i=0; i<scan_msg.ranges.size(); i++){
    if(std::isinf(scan_msg.ranges[i]) == 0){
      outputfile << deg << ".0 " << scan_msg.ranges[i] << " " << std::flush;
    }
    deg++;
    if(deg == 0) deg = -360;
  }

  outputfile << odom_msg.pose.pose.position.x << " " << odom_msg.pose.pose.position.y << " " << yaw << std::endl;
  stamp++;

  //DEBUG
  ROS_INFO("file output");

}

int main(int argc, char **argv){
  ros::init(argc, argv, "scan_listener");
  ros::NodeHandle nh;

  message_filters::Subscriber<nav_msgs::Odometry> odom_sub(nh, "odom", 1);
  message_filters::Subscriber<sensor_msgs::LaserScan> scan_sub(nh, "scan", 1);

  typedef sync_policies::ApproximateTime<nav_msgs::Odometry, sensor_msgs::LaserScan> MySyncPolicy;
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), odom_sub, scan_sub);
  sync.registerCallback(callback);

  ros::spin();

  return 0;
}
