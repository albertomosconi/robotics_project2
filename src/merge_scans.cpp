#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#define NODE_NAME "merge_scans"
#define TOPIC_SCAN_FRONT "front/scan"
#define TOPIC_SCAN_REAR "rear/scan"
#define TOPIC_SCAN_MERGE "merge/scan"

void merge_callback(const ros::Publisher *pub_merge, const sensor_msgs::LaserScan::ConstPtr &front_scan, const sensor_msgs::LaserScan::ConstPtr &rear_scan);

int main(int argc, char **argv)
{
    ros::init(argc, argv, NODE_NAME);
    ros::NodeHandle n;

    const ros::Publisher pub_merge_scan = n.advertise<sensor_msgs::LaserScan>(TOPIC_SCAN_MERGE, 6000);

    message_filters::Subscriber<sensor_msgs::LaserScan> sub_front_scan(n, TOPIC_SCAN_FRONT, 1);
    message_filters::Subscriber<sensor_msgs::LaserScan> sub_rear_scan(n, TOPIC_SCAN_REAR, 1);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, sensor_msgs::LaserScan> MySyncPolicy;

    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), sub_front_scan, sub_rear_scan);

    sync.registerCallback(boost::bind(&merge_callback, &pub_merge_scan, _1, _2));

    ros::spin();

    return 0;
}

void merge_callback(const ros::Publisher *pub_merge, const sensor_msgs::LaserScanConstPtr &front_scan, const sensor_msgs::LaserScanConstPtr &rear_scan)
{
    // ROS_INFO("\nf) min: %f max: %f\nb) min: %f max: %f", front_scan->angle_min, front_scan->angle_max, rear_scan->angle_min, rear_scan->angle_max);
    // ROS_INFO("\nf: %f\nb: %f", front_scan->scan_time, rear_scan->scan_time);
    // ROS_INFO("\nf) len: %d\nb) len: %d", (int)front_scan->intensities.size(), (int)rear_scan->intensities.size());
    // ROS_INFO("\nf) frame: %s", front_scan->header.frame_id);

    std::vector<float> processed_rear_intensities;
    std::vector<float> processed_rear_ranges;
    for (int i = 0; i < rear_scan->intensities.size() / 2; i++)
    {
        processed_rear_intensities.push_back(rear_scan->intensities.at(i));
        processed_rear_intensities.push_back(rear_scan->intensities.at(i));
        processed_rear_ranges.push_back(rear_scan->ranges.at(i));
        processed_rear_ranges.push_back(rear_scan->ranges.at(i));
    }

    std::vector<float> merged_intensities = front_scan->intensities;
    merged_intensities.insert(merged_intensities.end(), processed_rear_intensities.begin(), processed_rear_intensities.end());
    std::vector<float> merged_ranges = front_scan->ranges;
    merged_ranges.insert(merged_ranges.end(), processed_rear_ranges.begin(), processed_rear_ranges.end());

    sensor_msgs::LaserScan merge_scan_message;
    merge_scan_message.header.stamp = rear_scan->header.stamp;
    merge_scan_message.header.frame_id = front_scan->header.frame_id;
    merge_scan_message.angle_min = rear_scan->angle_min;
    merge_scan_message.angle_max = rear_scan->angle_max;
    merge_scan_message.angle_increment = front_scan->angle_increment;
    merge_scan_message.time_increment = front_scan->time_increment;
    merge_scan_message.scan_time = std::max(front_scan->scan_time, rear_scan->scan_time);
    merge_scan_message.range_min = std::min(front_scan->range_min, rear_scan->range_min);
    merge_scan_message.range_max = std::max(front_scan->range_max, rear_scan->range_max);
    merge_scan_message.ranges = merged_ranges;
    merge_scan_message.intensities = merged_intensities;

    pub_merge->publish(merge_scan_message);
}