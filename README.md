# multi_point_nav功能包
---
## 实现功能
设定若干目标点位姿，机器人按照目标点顺序循环巡逻，每到一个目标点后可以干一件事情（这里设定为等待1秒)；  
## 运行过程
### 设定目标点位姿
启动小车以及move_base节点后执行：  
`roslaunch multi_point_nav set_poses.launch`    
然后在rviz中使用2D Nav Goal在地图上依次指定若干个目标点，这些目标点位姿会记录在multi_point_nav/goal_poses.txt中，设置巡逻点完毕后可关闭该节点；  
### 小车开始巡逻
目标点设置完成后即可执行send_poses节点实现巡逻：    
`roslaunch multi_point_nav send_poses.launch`  
### RVIZ可视化巡逻点
添加MarkerArray，订阅/marker_array_poses即可。  
