#include <ros/ros.h>
//#include <ar_track_alvar_msgs/AlvarMarker.h>
#include <geometry_msgs/Pose.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>

std::vector<std::string> object_meshes;
std::vector<geometry_msgs::Pose> object_poses;

void callback(const ar_track_alvar_msgs::AlvarMarkers &object_msg){
    int object_count = object_msg.markers.size();
    for(int i = 0; i < object_count; i++){
        object_meshes.push_back(object_msg.markers[i].id);
        object_poses.push_back(object_msg.markers[i].pose.pose);
    }
    std::cout << object_meshes << std::endl << object_poses;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "get_ar_track_msgs");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("ar_pose_marker", 1, callback);
    ros::spin();
    return 0;
}