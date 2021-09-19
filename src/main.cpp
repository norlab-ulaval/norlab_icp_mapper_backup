#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <future>
#include <fstream>
#include <pointmatcher/PointMatcher.h>
#include <pointmatcher_ros/PointMatcher_ROS.h>
#include <dirent.h>
#include <sys/stat.h>

typedef PointMatcher<float> PM;

ros::Publisher backupMapPublisher;
ros::Time timeOfLastMapSave;
ros::Time timeOfLastLocalizationSave;
std::future<void> mapSaveFuture;

const std::string MAP_FILE_PREFIX = "backup_map_";
const std::string MAP_FILE_SUFFIX = ".vtk";
const std::string LOCALIZATION_FILE_PREFIX = "backup_localization_";
const std::string LOCALIZATION_FILE_SUFFIX = ".txt";
const std::string ROS_FOLDER = std::string(getenv("HOME")) + std::string("/.ros/");

std::vector<std::string> listFilePaths()
{
	DIR* dirPtr;
	struct dirent* direntPtr;
	std::vector<std::string> filePaths;
	if((dirPtr = opendir(ROS_FOLDER.c_str())) != nullptr)
	{
		while((direntPtr = readdir(dirPtr)) != nullptr)
		{
			std::string filePath = ROS_FOLDER + direntPtr->d_name;
			struct stat path_stat;
			stat(filePath.c_str(), &path_stat);
			if(S_ISREG(path_stat.st_mode))
			{
				filePaths.emplace_back(filePath);
			}
		}
		closedir(dirPtr);
	}
	return filePaths;
}

std::map<uint64_t, std::string> retrieveMapFileTimeStamps()
{
	std::map<uint64_t, std::string> mapFiles;
	std::vector<std::string> fileNames = listFilePaths();
	for(const std::string& fileName: fileNames)
	{
		size_t prefixPos = fileName.rfind(MAP_FILE_PREFIX);
		if(prefixPos != std::string::npos)
		{
			size_t timeStampPos = prefixPos + MAP_FILE_PREFIX.size();
			std::string timeStamp = fileName.substr(timeStampPos, fileName.size() - timeStampPos - MAP_FILE_SUFFIX.size());
			mapFiles[std::stol(timeStamp)] = fileName;
		}
	}
	return mapFiles;
}

std::map<uint64_t , std::string> retrieveLocalizationFileTimeStamps()
{
	std::map<uint64_t, std::string> localizationFiles;
	std::vector<std::string> fileNames = listFilePaths();
	for(const std::string& fileName: fileNames)
	{
		size_t prefixPos = fileName.rfind(LOCALIZATION_FILE_PREFIX);
		if(prefixPos != std::string::npos)
		{
			size_t timeStampPos = prefixPos + LOCALIZATION_FILE_PREFIX.size();
			std::string timeStamp = fileName.substr(timeStampPos, fileName.size() - timeStampPos - LOCALIZATION_FILE_SUFFIX.size());
			localizationFiles[std::stol(timeStamp)] = fileName;
		}
	}
	return localizationFiles;
}

void removeOldFiles(const std::map<uint64_t, std::string>& files)
{
	int currentMapFileIndex = 0;
	for(auto it = files.begin(); currentMapFileIndex < files.size() - 2; ++it)
	{
		std::remove(it->second.c_str());
		++currentMapFileIndex;
	}
}

void saveMap(sensor_msgs::PointCloud2 mapMsg)
{
	PM::DataPoints map = PointMatcher_ROS::rosMsgToPointMatcherCloud<float>(mapMsg);
	map.save(MAP_FILE_PREFIX + std::to_string(mapMsg.header.stamp.toNSec()) + MAP_FILE_SUFFIX);
	removeOldFiles(retrieveMapFileTimeStamps());
}

void mapCallback(const sensor_msgs::PointCloud2& msg)
{
	backupMapPublisher.publish(msg);

	if(ros::Time::now() - timeOfLastMapSave >= ros::Duration(10))
	{
		// check if previous save is over
		if(!mapSaveFuture.valid() || mapSaveFuture.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready)
		{
			timeOfLastMapSave = ros::Time::now();
			mapSaveFuture = std::async(std::launch::async, &saveMap, msg);
		}
	}
}

PM::TransformationParameters rosMsgToPointMatcherPose(const geometry_msgs::Pose& pose)
{
	Eigen::Vector3f epsilon(pose.orientation.x, pose.orientation.y, pose.orientation.z);
	float eta = pose.orientation.w;
	PM::Matrix skewSymmetricEpsilon = PM::Matrix::Zero(3, 3);
	skewSymmetricEpsilon << 0, -epsilon[2], epsilon[1],
			epsilon[2], 0, -epsilon[0],
			-epsilon[1], epsilon[0], 0;
	PM::Matrix rotationMatrix = (((eta * eta) - epsilon.dot(epsilon)) * PM::Matrix::Identity(3, 3)) +
								(2 * eta * skewSymmetricEpsilon) + (2 * epsilon * epsilon.transpose());

	Eigen::Vector3f positionVector(pose.position.x, pose.position.y, pose.position.z);

	PM::TransformationParameters transformationParameters = PM::TransformationParameters::Identity(4, 4);
	transformationParameters.topLeftCorner(3, 3) = rotationMatrix.topLeftCorner(3, 3);
	transformationParameters.topRightCorner(3, 1) = positionVector.head(3);
	return transformationParameters;
}

void localizationCallback(const nav_msgs::Odometry& msg)
{
	if(ros::Time::now() - timeOfLastLocalizationSave >= ros::Duration(10))
	{
		timeOfLastLocalizationSave = ros::Time::now();
		std::ofstream localizationBackupFile(LOCALIZATION_FILE_PREFIX + std::to_string(msg.header.stamp.toNSec()) + LOCALIZATION_FILE_SUFFIX);
		PM::TransformationParameters pose = rosMsgToPointMatcherPose(msg.pose.pose);
		localizationBackupFile << pose << std::endl;
		localizationBackupFile.close();
		removeOldFiles(retrieveLocalizationFileTimeStamps());
	}
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "norlab_icp_mapper_backup_node");
	ros::NodeHandle nodeHandle;

	std::map<uint64_t, std::string> mapFileTimeStamps = retrieveMapFileTimeStamps();
	if(mapFileTimeStamps.size() > 2)
	{
		std::remove((--mapFileTimeStamps.end())->second.c_str());
	}
	removeOldFiles(mapFileTimeStamps);

	std::map<uint64_t, std::string> localizationFileTimeStamps = retrieveLocalizationFileTimeStamps();
	if(localizationFileTimeStamps.size() > 2)
	{
		std::remove((--localizationFileTimeStamps.end())->second.c_str());
	}
	removeOldFiles(localizationFileTimeStamps);

	while(ros::Time::now().isZero());

	timeOfLastMapSave = ros::Time::now();
	timeOfLastLocalizationSave = ros::Time::now();

	backupMapPublisher = nodeHandle.advertise<sensor_msgs::PointCloud2>("backup_map", 1, true);

	ros::Subscriber mapSubscriber = nodeHandle.subscribe("map", 1, mapCallback);
	ros::Subscriber localizationSubscriber = nodeHandle.subscribe("icp_odom", 1, localizationCallback);

	ros::spin();

	return 0;
}
