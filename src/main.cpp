#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

ros::Publisher backupMapPublisher;

void mapCallback(const sensor_msgs::PointCloud2& msg)
{
	backupMapPublisher.publish(msg);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "norlab_icp_mapper_backup_node");
	ros::NodeHandle nodeHandle;

	backupMapPublisher = nodeHandle.advertise<sensor_msgs::PointCloud2>("backup_map", 1, true);

	ros::Subscriber mapSubscriber = nodeHandle.subscribe("map", 1, mapCallback);

	ros::spin();

	return 0;
}
