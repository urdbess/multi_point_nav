#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <iostream>
#include <math.h>
#include <cmath> 
#include <fstream>
#include <boost/regex.hpp>
#include "std_msgs/String.h"
#include <vector>


visualization_msgs::MarkerArray marks;
std::ifstream infile;
std::string record_file;

ros::Publisher  marker_array_pub;

std::vector<std::string> my_split(std::string str, std::string s) {
        boost::regex reg(s.c_str());
        std::vector<std::string> vec;
        boost::sregex_token_iterator it(str.begin(), str.end(), reg, -1);
        boost::sregex_token_iterator end;
        while (it != end) {
            vec.push_back(*it++);
        }
        return vec;
    }

void load_marks(std::string record_file, visualization_msgs::MarkerArray& ma){
    infile.open(record_file, std::ios::in | std::ios::binary);
    if (!infile) {
            ROS_WARN("Failed to open file!");
            return;
        }
    std::string s;
    int count = 0;
    ROS_INFO("---loading data---");
    while (getline(infile, s)) {
            visualization_msgs::Marker marker;
            marker.type = marker.TEXT_VIEW_FACING;  //选用文本类型
            marker.ns = "basic_shapes";                          //必写，否则rviz无法显示
            marker.id = count;
            marker.header.frame_id = "map";

            marker.scale.x = 0.5;
            marker.scale.y = 0.5;
            marker.scale.z = 0.5;                                    //文字的大�?
            marker.color.b = 25;
            marker.color.g = 0;
            marker.color.r = 25;                                   //文字的颜�?            marker.color.a = 1;                                    //必写，否则rviz无法显示
            marker.color.a = 1;
            marker.pose.position.x = atof(my_split(s, " ")[0].c_str());
            marker.pose.position.y = atof(my_split(s, " ")[1].c_str());
            marker.pose.orientation.w = 1;
      
            marker.text = std::to_string(count);   //文字内容
            ma.markers.push_back(marker);
            count++;
        }
    ROS_INFO("---successfully loaded---");
    infile.close();
}



int main(int argc,char **argv)
{

    ros::init(argc,argv,"show marker");

    ros::NodeHandle node;

    ROS_INFO("init..."); 
    node.getParam("/record_file", record_file);

    marker_array_pub = node.advertise<visualization_msgs::MarkerArray>("/visualization_marker_array", 10);
    //visualization_msgs::MarkerArray marks;
      //加载目标文件
    load_marks(record_file, marks);
    ROS_INFO("publish marks");
     //等待1秒让pub准备就绪，否则第一个目标点发不出去
    ros::Rate r(0.75);
    r.sleep();
    while (ros::ok())
    {
        marker_array_pub.publish(marks);
        r.sleep();
        ros::spinOnce();
        
    }
    // marker_array_pub.publish(marks);
    // ros::spin();
    
    return 0;
}
  
