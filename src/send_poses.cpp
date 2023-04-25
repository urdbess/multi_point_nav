#include "ros/ros.h"
#include "std_msgs/String.h"
#include <fstream>
#include <boost/regex.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <vector>
#include <move_base_msgs/MoveBaseActionResult.h>

std::ifstream infile;
std::string record_file;
std::vector<geometry_msgs::PoseStamped> goal_poses;
int count = 0;
int goal_poses_size;

ros::Publisher goal_pub;

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

void load_poses(std::string record_file, std::vector<geometry_msgs::PoseStamped>& vc){
    infile.open(record_file, std::ios::in | std::ios::binary);
    if (!infile) {
            ROS_WARN("Failed to open file!");
            return;
        }
    std::string s;
    ROS_INFO("---loading data---");
    while (getline(infile, s)) {
            geometry_msgs::PoseStamped pos;
            pos.pose.position.x = atof(my_split(s, " ")[0].c_str());
            pos.pose.position.y = atof(my_split(s, " ")[1].c_str());
            pos.pose.orientation.x = atof(my_split(s, " ")[2].c_str());
            pos.pose.orientation.y = atof(my_split(s, " ")[3].c_str());
            pos.pose.orientation.z = atof(my_split(s, " ")[4].c_str());
            pos.pose.orientation.w = atof(my_split(s, " ")[5].c_str());
            vc.push_back(pos);
        }
    ROS_INFO("---successfully loaded---");
    infile.close();
}

void do_something(){
    ros::Rate r(1);
    r.sleep();
}

void cb_status(move_base_msgs::MoveBaseActionResult msg){
    if (msg.status.status == 3) {
        //goal_index:已到达的目标点
        int goal_index = count++ % goal_poses_size;
        ROS_INFO("goal %d reached, try to do something", goal_index);
        do_something();
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = "map";
        pose.header.stamp = ros::Time::now();
        pose.pose = goal_poses[(goal_index + 1) % goal_poses_size].pose;
        ROS_INFO("heading to goal %d: (%f, %f)", 
                (goal_index + 1) % goal_poses_size, pose.pose.position.x, pose.pose.position.y);
        goal_pub.publish(pose);
    } else {
        int goal_index = count++ % goal_poses_size;
        ROS_WARN("goal %d cannot reached, /move_base/result status: %d! try to heading to goal %d", 
                goal_index, msg.status.status, (goal_index + 1) % goal_poses_size);
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = "map";
        pose.header.stamp = ros::Time::now();
        pose.pose = goal_poses[(goal_index + 1) % goal_poses_size].pose;
        goal_pub.publish(pose);

    }
}

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "send_poses_node");
    ros::NodeHandle n;
    n.getParam("/send_poses_node/record_file", record_file);
    ros::Subscriber goal_status_sub = n.subscribe<move_base_msgs::MoveBaseActionResult>("/move_base/result", 1, cb_status);
    goal_pub = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);
    //加载目标文件
    load_poses(record_file, goal_poses);
    goal_poses_size = goal_poses.size();
    //发布第一个目标点
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.header.stamp = ros::Time::now();
    pose.pose= goal_poses[0].pose;
    ROS_INFO("heading to goal 0: (%f, %f)", pose.pose.position.x, pose.pose.position.y);
    //等待1秒让pub准备就绪，否则第一个目标点发不出去
    ros::Rate r(0.75);
    r.sleep();
    goal_pub.publish(pose);
    ros::spin();
    return 0;
}