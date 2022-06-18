#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#define TOPIC_ROBOT_ODOM "odom"
#define TF_FRAME_PARENT "odom"
#define TF_FRAME_CHILD "base_link"
#define QUEUE_SIZE 1000

void robot_odom_callback(tf2_ros::TransformBroadcaster *broadcaster, geometry_msgs::TransformStamped *message, const nav_msgs::OdometryConstPtr &odom);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "odom_tf");

    ros::NodeHandle n;

    ros::Subscriber sub_odom;
    tf2_ros::TransformBroadcaster tf_odom_broadcaster;
    geometry_msgs::TransformStamped tf_message_odom;
    tf_message_odom.header.frame_id = TF_FRAME_PARENT;
    tf_message_odom.child_frame_id = TF_FRAME_CHILD;

    sub_odom = n.subscribe<nav_msgs::Odometry>(
        TOPIC_ROBOT_ODOM,
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
    message->transform.rotation.x = odom->pose.pose.orientation.x;
    message->transform.rotation.y = odom->pose.pose.orientation.y;
    message->transform.rotation.z = odom->pose.pose.orientation.z;
    message->transform.rotation.w = odom->pose.pose.orientation.w;

    (*broadcaster).sendTransform(*message);
}