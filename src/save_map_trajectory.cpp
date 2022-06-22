#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <project2/save_map_trajectory.h>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <nav_msgs/OccupancyGrid.h>
#include <opencv2/imgcodecs.hpp>
#include <ros/package.h>

#define SERVICE_NAME "save_map_trajectory"
#define POSE_TOPIC "amcl_pose"
#define MAP_TOPIC "map"
#define QUEUE_SIZE 1000

bool service_callback(
    cv_bridge::CvImage *image,
    project2::save_map_trajectory::Request &req,
    project2::save_map_trajectory::Response &res);

void pose_callback(cv_bridge::CvImage *image,
                   geometry_msgs::Point *origin,
                   float *resolution,
                   cv::Point *previous_position,
                   const geometry_msgs::PoseWithCovarianceStampedConstPtr &pose);

void map_callback(cv_bridge::CvImage *image,
                  geometry_msgs::Point *origin,
                  float *resolution,
                  cv::Point *previous_position,
                  const nav_msgs::OccupancyGrid::ConstPtr &ms);

int main(int argc, char **argv)
{
    ros::init(argc, argv, SERVICE_NAME);
    ros::NodeHandle n;

    // map image
    cv_bridge::CvImage image;
    // previous robot pose
    cv::Point previous_position;
    geometry_msgs::Point origin;
    float resolution;

    // start service server
    ros::ServiceServer service = n.advertiseService<project2::save_map_trajectory::Request, project2::save_map_trajectory::Response>(
        SERVICE_NAME,
        boost::bind(&service_callback, &image, _1, _2));

    ros::Subscriber sub_pose = n.subscribe<geometry_msgs::PoseWithCovarianceStamped>(
        POSE_TOPIC,
        QUEUE_SIZE,
        boost::bind(&pose_callback, &image, &origin, &resolution, &previous_position, _1));

    ros::Subscriber sub_map = n.subscribe<nav_msgs::OccupancyGrid>(
        MAP_TOPIC,
        QUEUE_SIZE,
        boost::bind(&map_callback, &image, &origin, &resolution, &previous_position, _1));

    ros::spin();

    return 0;
}

void map_callback(cv_bridge::CvImage *image,
                  geometry_msgs::Point *origin,
                  float *resolution,
                  cv::Point *previous_position,
                  const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
    *origin = msg->info.origin.position;
    *resolution = msg->info.resolution;
    previous_position->x = -msg->info.origin.position.x / msg->info.resolution;
    previous_position->y = -msg->info.origin.position.y / msg->info.resolution;
    std::cout << origin->x << " " << origin->y << " " << *resolution << "\n";
    std::cout << previous_position->x << " " << previous_position->y << "\n";
    // convert occupancy grid to GridMap
    grid_map::GridMap map;
    grid_map::GridMapRosConverter::fromOccupancyGrid(
        *msg,
        "project2_map_layer",
        map);
    // convert GridMap to OpenCV image
    grid_map::GridMapRosConverter::toCvImage(
        map,
        "project2_map_layer",
        sensor_msgs::image_encodings::RGB8,
        *image);
    // fix image rotation
    cv::flip(image->image, image->image, 1);
    cv::transpose(image->image, image->image);
    cv::flip(image->image, image->image, 1);
}

bool service_callback(cv_bridge::CvImage *image, project2::save_map_trajectory::Request &req, project2::save_map_trajectory::Response &res)
{
    std::string image_path = ros::package::getPath("project2") + "/maps/map_with_trajectory.png";
    bool success = cv::imwrite(image_path, image->image, {cv::IMWRITE_PNG_STRATEGY_DEFAULT});
    if (success)
        ROS_INFO("saved image with success: %s", image_path.c_str());
    else
        ROS_ERROR("error saving map and trajectory image: %s", image_path.c_str());
    return true;
}

void pose_callback(cv_bridge::CvImage *image,
                   geometry_msgs::Point *origin,
                   float *resolution,
                   cv::Point *previous_position,
                   const geometry_msgs::PoseWithCovarianceStampedConstPtr &pose)
{
    cv::Point current_position;
    current_position.x = (pose->pose.pose.position.x - origin->x) / *resolution;
    current_position.y = (pose->pose.pose.position.y - origin->y) / *resolution;

    // draw line on image from previous position to current
    const cv::Scalar line_color(0, 0, 255); // line is red
    const int line_thickness = 1;
    cv::line(image->image, *previous_position, current_position, line_color, line_thickness);

    *previous_position = current_position;
}