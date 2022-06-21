#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <project2/save_map_trajectory.h>

#define SERVICE_NAME "save_map_trajectory"
#define POSE_TOPIC "amcl_pose"
#define QUEUE_SIZE 1000

bool service_callback(project2::save_map_trajectory::Request &req, project2::save_map_trajectory::Response &res);
void pose_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &pose);

int main(int argc, char **argv)
{
    ros::init(argc, argv, SERVICE_NAME);
    ros::NodeHandle n;

    // start service server
    ros::ServiceServer service = n.advertiseService<project2::save_map_trajectory::Request, project2::save_map_trajectory::Response>(
        SERVICE_NAME,
        boost::bind(&service_callback, _1, _2));

    ros::Subscriber sub_pose = n.subscribe<geometry_msgs::PoseWithCovarianceStamped>(
        POSE_TOPIC,
        QUEUE_SIZE,
        boost::bind(&pose_callback, _1));

    ros::spin();

    return 0;
}

bool service_callback(project2::save_map_trajectory::Request &req, project2::save_map_trajectory::Response &res)
{
    ROS_INFO("service called");
    return true;
}

void pose_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &pose)
{
    ROS_INFO("%f", pose->pose.pose.position.x);
    ROS_INFO("%f", pose->pose.pose.position.y);
    ROS_INFO("%f", pose->pose.pose.position.z);
}