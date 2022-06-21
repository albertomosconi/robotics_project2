#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>

#define EXE_NAME "broadcast_odom_tf"
#define QUEUE_SIZE 1000

void robot_odom_callback(tf2_ros::TransformBroadcaster *broadcaster, geometry_msgs::TransformStamped *message, const nav_msgs::OdometryConstPtr &odom);

int main(int argc, char **argv)
{
    // initialize node
    ros::init(argc, argv, EXE_NAME);
    ros::NodeHandle n;

    // read parameters
    std::string TF_PARENT, TF_CHILD, ODOM_TOPIC;
    n.param<std::string>(EXE_NAME "/parent_frame", TF_PARENT, "world");
    n.param<std::string>(EXE_NAME "/child_frame", TF_CHILD, "robot");
    n.param<std::string>(EXE_NAME "/odom_topic", ODOM_TOPIC, "odom");

    ros::Subscriber sub_odom;
    tf2_ros::TransformBroadcaster tf_odom_broadcaster;
    geometry_msgs::TransformStamped tf_message_odom;
    tf_message_odom.header.frame_id = TF_PARENT;
    tf_message_odom.child_frame_id = TF_CHILD;

    sub_odom = n.subscribe<nav_msgs::Odometry>(
        ODOM_TOPIC,
        QUEUE_SIZE,
        boost::bind(&robot_odom_callback, &tf_odom_broadcaster, &tf_message_odom, _1));

    ros::spin();

    return 0;
}

void robot_odom_callback(tf2_ros::TransformBroadcaster *broadcaster, geometry_msgs::TransformStamped *message, const nav_msgs::OdometryConstPtr &odom)
{
    message->header.stamp = ros::Time::now();

    message->transform.translation.x = odom->pose.pose.position.x;
    message->transform.translation.y = odom->pose.pose.position.y;
    message->transform.translation.z = odom->pose.pose.position.z;
    message->transform.rotation = odom->pose.pose.orientation;

    (*broadcaster).sendTransform(*message);
}