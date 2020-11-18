#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose2D.h>
#include <string.h>
#include <fstream>
#include <iomanip>
#include <vector>

#define DEG2RAD(x) ((x)*M_PI/180)  // 度からラジアン
#define RAD2DEG(x) ((x)*180/M_PI)  // ラジアンから度

//クオータニオンからRPYに変換
void geometry_quat_to_rpy(double& roll, double& pitch, double& yaw, geometry_msgs::Quaternion geometry_quat){
  tf::Quaternion quat;
  quaternionMsgToTF(geometry_quat, quat);
  tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);  //rpy are Pass by Reference
}

class Listener {
public:
  ros::NodeHandle nh;
  ros::Subscriber sub_odom, sub_scan;

  geometry_msgs::Pose2D pos;
  sensor_msgs::LaserScan scan;

  int stamp;
  int data_num;
  std::vector<bool> flag;

  std::ofstream outputfile;
  std::string filename;

  Listener() : stamp(0), data_num(0) {
    sub_odom = nh.subscribe("/ypspur_ros/odom", 1, &Listener::callback_odom, this);
    sub_scan = nh.subscribe("scan", 1, &Listener::callback_scan, this);

    filename = "scan_odom1.txt";
    outputfile.open(filename, std::ios::out);
  }
  ~Listener() {
    outputfile.close();
  }

  // 5Hzでscanとodomに保存
  void loop() {
    ros::Rate loop_rate(5);

    while(ros::ok()){
      ros::spinOnce();
      write2file();
      loop_rate.sleep();
    }
  }

  // odomのコールバック
  void callback_odom(const nav_msgs::Odometry &odom_msg){
    double roll, pitch, yaw;

    pos.x = odom_msg.pose.pose.position.x;
    pos.y = odom_msg.pose.pose.position.y;
    geometry_quat_to_rpy(roll, pitch, yaw, odom_msg.pose.pose.orientation);
    pos.theta = RAD2DEG(yaw); // zではなく角度を保存

    return;
  }

  // scanのコールバック
  void callback_scan(const sensor_msgs::LaserScan &scan_msg) {
    int count = 0;
    double angle = -135.0;

    flag.clear();

    for(int i = 0; i < scan_msg.ranges.size(); i++) {
      // -90度〜90度のみ使用 無限大は無視 30m以内のセンサ値のみ使用
      if(angle > -90 && angle < 90 && std::isinf(scan_msg.ranges[i]) == 0 && scan_msg.ranges[i] < 30){
        count++;
        flag.push_back(true);
      } else {
        flag.push_back(false);
      }
      angle += 0.25;
    }
    data_num = count;
    scan = scan_msg;

    return;
  }

  void write2file() {
    double angle;

    if(stamp == 0) {
      stamp++;
      return;
    }

    // タイムスタンプ
    outputfile << "LASERSCAN " << stamp << " " << data_num << " " << std::flush;
    // スキャンデータ
    angle = -135.0;
    for(int i=0; i<scan.ranges.size(); i++) {
      if(flag[i] == true) {
        outputfile << angle << " " << scan.ranges[i] << " " << std::flush;
      }
      angle += 0.25;
    }
    // オドメトリデータ
    outputfile << pos.x << " " << pos.y << " " << pos.theta << std::endl;

    stamp++;

    ROS_INFO("Output odom and scan to file.");

    return;
  }

};

int main(int argc, char **argv){
  ros::init(argc, argv, "scan_odom_listener");

  Listener lis;
  lis.loop();

  return 0;
}
