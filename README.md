# ros_scan_textout

SLAM入門ーロボットの自己位置推定と地図構築の技術ー (友納正裕)  
のサンプルコードで使用する入力データを生成  

## spec
hokuyoのUTM-30LX-EWを使用  
LiDARの値は-90度〜90度のみ使用 30m以内のセンサ値のみ使用  
5Hzで記録

## data
nav_msgs::Odometry　：　/ypspur_ros/odom  
sensor_msgs::LaserScan　：　/scan  

output file　：　scan_odom1.txt

## format
LASERSCAN time_stamp data_num angle1 range1 angle2 range2 ... position.x position.y position.theta  
LASERSCAN time_stamp data_num angle1 range1 angle2 range2 ... position.x position.y position.theta  
...
