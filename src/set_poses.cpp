#include "ros/ros.h"
#include "std_msgs/String.h"
#include <fstream>
#include <geometry_msgs/PoseStamped.h>

std::ofstream outfile;
std::string record_file;
int cnt = 1;
void cb_record_poses(const geometry_msgs::PoseStamped::ConstPtr& msg){
    outfile.open(record_file, std::ios::binary | std::ios::out | std::ios::app);
    ROS_INFO("goal %d position: %f, %f", cnt++, msg->pose.position.x, msg->pose.position.y);
    outfile << msg->pose.position.x << ' ' << msg->pose.position.y <<' ' 
            << msg->pose.orientation.x << ' ' << msg->pose.orientation.y << ' ' 
            << msg->pose.orientation.z << ' ' << msg->pose.orientation.w << "\n";
    outfile.close();
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "set_poses_node");
    ros::NodeHandle n;
    n.getParam("/set_poses_node/record_file", record_file);
    
    //初始化目标文件
    outfile.open(record_file);
    outfile.clear();
    outfile.close();

    ros::Subscriber goal_sub = n.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10, cb_record_poses);
    ros::spin();
    return 0;
}