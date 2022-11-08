#include "math.h"
#include <string>
#include "stdio.h"
#include <fstream>
#include <sstream>
#include <iostream>
#include <ros/ros.h>
#include <ros/package.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <std_msgs/String.h>
#include "sensor_msgs/Imu.h"
#include <pcl/conversions.h>
// #include <libnotify/notify.h>
#include <nav_msgs/Odometry.h>
#include <pcl_ros/transforms.h>
#include <tf2_eigen/tf2_eigen.h>
#include <pcl/registration/icp.h>
#include <tf/transform_listener.h>
#include <pcl/filters/voxel_grid.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/filters/passthrough.h>
#include <tf/transform_broadcaster.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <pcl_conversions/pcl_conversions.h>

class icp_localization
{

private:
	// =============== ros util parameters ===============
	ros::NodeHandle nh;
	ros::Subscriber sub_map;
	ros::Subscriber sub_gps;
	ros::Subscriber sub_odom;
	ros::Publisher pub_lidar;
	bool gps_ready, map_ready;
	ros::Subscriber sub_lidar_scan;
	tf::TransformListener tf_listener;
	tf::TransformBroadcaster tf_broadcaster;

	// =============== variables of transformation ===============
	double odom_ratio;
	double lidar_ratio;
	double frequency_ratio;
	double odom_x, odom_y, odom_z;
	double diff_x, diff_y, diff_z;
	Eigen::Matrix4f initial_guess;
	sensor_msgs::PointCloud2 Final_map;
	Eigen::Matrix4f c2l_eigen_transform;
	sensor_msgs::PointCloud2 Final_cloud;
	pcl::PointCloud<pcl::PointXYZI>::Ptr map;

	// =============== variables of output file ===============
	std::ofstream outfile;
	std::ofstream transformation_record;
	std::string map_path, result_path, transformation_path;

	// =============== variables of ICP parameters ===============
	double map_leaf_size;
	double scan_leaf_size;

public:
	int frame_number;

	/**
	 * @brief Construct a new icp localization object, initializing ICP(get initial guess)
	 *
	 * @param _nh ros node handler
	 */
	icp_localization(ros::NodeHandle _nh) : map(new pcl::PointCloud<pcl::PointXYZI>)
	{

		std::cout << "Initializing ICP...\n";
		this->nh = _nh;
		this->odom_x = 0;
		this->odom_y = 0;
		this->odom_z = 0;
		this->diff_x = 0;
		this->diff_y = 0;
		this->diff_z = 0;
		this->frame_number = 0;
		this->gps_ready = false;
		this->map_ready = false;
		std::vector<float> trans, rot;
		this->pub_lidar = this->nh.advertise<sensor_msgs::PointCloud2>("/transformed_points", 1);
		this->sub_map = this->nh.subscribe("/map", 4000000, &icp_localization::map_callback, this);
		this->sub_odom = this->nh.subscribe("/wheel_odometry", 4000000, &icp_localization::odom_callback, this);
		this->sub_lidar_scan = this->nh.subscribe("/lidar_points", 4000000, &icp_localization::lidar_scanning, this);

		// grasping ros parameters
		_nh.param<double>("odom_ratio", odom_ratio, 1.0);
		_nh.param<double>("lidar_ratio", lidar_ratio, 1.0);
		_nh.param<double>("mapLeafSize", map_leaf_size, 0.15);
		_nh.param<double>("scanLeafSize", scan_leaf_size, 0.15);
		_nh.param<std::string>("map_path", map_path, "nuscenes_map.pcd");
		_nh.param<std::string>("result_save_path", result_path, "result2.csv");
		_nh.param<std::vector<float>>("baselink2lidar_rot", rot, std::vector<float>());
		_nh.param<std::vector<float>>("baselink2lidar_trans", trans, std::vector<float>());
		_nh.param<std::string>("transformation_path", transformation_path, "transformation.txt");

		// 把itri.yaml中的transform link存下來
		if (trans.size() != 3 | rot.size() != 4)
			ROS_ERROR("transform not set properly");

		Eigen::Quaternionf link_quaternion(rot.at(3), rot.at(0), rot.at(1), rot.at(2));
		Eigen::Matrix3f link_rotation = link_quaternion.toRotationMatrix();

		c2l_eigen_transform << 		link_rotation(0, 0), 	link_rotation(0, 1), 	link_rotation(0, 2), 	trans.at(0),
									link_rotation(1, 0), 	link_rotation(1, 1), 	link_rotation(1, 2), 	trans.at(1),
									link_rotation(2, 0), 	link_rotation(2, 1), 	link_rotation(2, 2), 	trans.at(2),
													  0, 					  0, 					  0, 			  1;

		this->frequency_ratio = lidar_ratio / (double)odom_ratio;

		// getting initial guess
		std::cout << "Finding initial guess. \n";
		this->initial_guess = get_initial_guess();
		std::cout << "Get initial guess: \n";
		std::cout << this->initial_guess << std::endl;
		std::cout << "Ready to localization\n";

		std::cout << "Result path: " << result_path << std::endl;
		outfile.open(result_path);
		transformation_record.open(transformation_path);
		outfile << "id,x,y,z,yaw,pitch,roll" << std::endl;
	}

	/**
	 * @brief Get the initial guess object
	 *
	 * @return Eigen::Matrix4f initial guess of map to car transfromation
	 */
	Eigen::Matrix4f get_initial_guess()
	{

		Eigen::Matrix4f initial_guess;
		geometry_msgs::PointStampedConstPtr gps_point;
		gps_point = ros::topic::waitForMessage<geometry_msgs::PointStamped>("/gps", this->nh);
		this->gps_ready = true;
		std::cout << "Get GPS.\n";

		double yaw = 0;

		initial_guess << 		cos(yaw), -sin(yaw), 	0, 	gps_point->point.x,
								sin(yaw),  cos(yaw), 	0, 	gps_point->point.y,
									   0, 		  0, 	1, 	gps_point->point.z,
									   0, 		  0, 	0, 					 1;

		return initial_guess;
	}

	/**
	 * @brief Down sampling pointCloud of lidar scan
	 *
	 * @param msg ros topic of lidar scan
	 * @return pcl::PointCloud<pcl::PointXYZI>::Ptr pointer of PointCloud of lidar scan after downsampling
	 */
	pcl::PointCloud<pcl::PointXYZI>::Ptr down_sampling(const sensor_msgs::PointCloud2::ConstPtr &msg)
	{

		pcl::PointCloud<pcl::PointXYZI>::Ptr raw_scan(new pcl::PointCloud<pcl::PointXYZI>);
		pcl::PointCloud<pcl::PointXYZI>::Ptr result_scan(new pcl::PointCloud<pcl::PointXYZI>);
		pcl::PCLPointCloud2::Ptr filtered_scan(new pcl::PCLPointCloud2());

		// sensor_msgs -> PointCloud
		pcl::fromROSMsg(*msg, *raw_scan);
		// PointCloud -> PointCloud2
		pcl::toPCLPointCloud2(*raw_scan, *filtered_scan);

		// 可以傳PointCloud 也可以傳PointCloud2, 這邊因為我們傳的是PointCloud2(放在<>裡面的型別)
		// 因此我們filter的input、output的型別都是PointCloud2
		// 所以我們上面需要先將msg轉成PointCloud後 再轉乘PointCloud2,
		pcl::VoxelGrid<pcl::PCLPointCloud2> voxel_filter;
		voxel_filter.setInputCloud(filtered_scan);
		voxel_filter.setFilterFieldName("z");
		voxel_filter.setFilterLimits(-2.0, 10.5);
		voxel_filter.setLeafSize(0.1f, 0.1f, 0.6f);
		voxel_filter.filter(*filtered_scan);

		// PointCloud2 -> PointCloud
		pcl::fromPCLPointCloud2(*filtered_scan, *result_scan);

		return result_scan;
	}

	/**
	 * @brief callback fcn when receiving lidar scan, perform icp here
	 *
	 * @param msg ros topic of lidar scan
	 */
	void lidar_scanning(const sensor_msgs::PointCloud2::ConstPtr &msg)
	{

		while (!(gps_ready & map_ready))
		{
			// if (gps_ready)
			// 	ROS_WARN("waiting for map data ...");
			// if (map_ready)
			// 	ROS_WARN("waiting for gps data ...");
			ros::Duration(0.05).sleep();
			ros::spinOnce();
		}
		pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_map(new pcl::PointCloud<pcl::PointXYZI>);
		pcl::PointCloud<pcl::PointXYZI> aligned_points;

		// =============== Passthrough ===============
		pcl::PassThrough<pcl::PointXYZI> filter;
		filter.setInputCloud(this->map);
		filter.setFilterFieldName("x");
		filter.setFilterLimits(this->initial_guess(0, 3) - 100.0, this->initial_guess(0, 3) + 100.0);
		filter.filter(*filtered_map);

		filter.setInputCloud(filtered_map);
		filter.setFilterFieldName("y");
		filter.setFilterLimits(this->initial_guess(1, 3) - 100.0, this->initial_guess(1, 3) + 100.0);
		filter.filter(*filtered_map);

		filter.setInputCloud(filtered_map);
		filter.setFilterFieldName("z");
		filter.setFilterLimits(1, 8);
		filter.filter(*filtered_map);

		// =============== Down sampling lidar scan ===============
		pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_scan(new pcl::PointCloud<pcl::PointXYZI>);
		pcl::PointCloud<pcl::PointXYZI>::Ptr final_filtered_scan(new pcl::PointCloud<pcl::PointXYZI>);
		filtered_scan = down_sampling(msg);

		// =============== transform scan to car ===============
		// Eigen::Matrix4f trans = get_transform("nuscenes_lidar");
		transformPointCloud(*filtered_scan, *filtered_scan, c2l_eigen_transform);
		// std::cout << "Trans from tf:" << std::endl;
		// std::cout << trans << std::endl;
		// std::cout << "Trans from yaml:" << std::endl;
		// std::cout << c2l_eigen_transform << std::endl;

		pcl::VoxelGrid<pcl::PointXYZI> voxel_filter;
		voxel_filter.setInputCloud(filtered_scan);
		voxel_filter.setFilterFieldName("z");
		voxel_filter.setFilterLimits(1.0, 7.5);
		voxel_filter.setLeafSize(0.1f, 0.1f, 0.6f);
		voxel_filter.filter(*filtered_scan);

		// =============== start performing ICP ===============
		pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;
		icp.setInputSource(filtered_scan);
		icp.setInputTarget(filtered_map);
		icp.setMaximumIterations(1000);				 
		icp.setTransformationEpsilon(1e-12);		 
		icp.setMaxCorrespondenceDistance(0.75);		
		icp.setEuclideanFitnessEpsilon(0.00075);		 
		icp.setRANSACOutlierRejectionThreshold(0.05); 
		icp.align(aligned_points, this->initial_guess);
		
		// publish transformed points
		sensor_msgs::PointCloud2::Ptr out_msg(new sensor_msgs::PointCloud2);
		// pcl_ros::transformPointCloud(this->initial_guess, *msg, *out_msg);
		pcl::toROSMsg(aligned_points, *out_msg);
		out_msg->header = msg->header;
		out_msg->header.frame_id = "world";
		pub_lidar.publish(out_msg);


		// =============== Get car pos using ICP result===============
		// initial guess是map 看向 car的轉換
		this->initial_guess = icp.getFinalTransformation();
		Eigen::Matrix4f transformation = this->initial_guess;



		tf2::Matrix3x3 m2c_trans_rotation;
		m2c_trans_rotation.setValue(
			initial_guess(0, 0), initial_guess(0, 1), initial_guess(0, 2),
			initial_guess(1, 0), initial_guess(1, 1), initial_guess(1, 2),
			initial_guess(2, 0), initial_guess(2, 1), initial_guess(2, 2)
		);
		tf2::Quaternion m2c_rotation_quaternion;
		m2c_trans_rotation.getRotation(m2c_rotation_quaternion);

		tf::Quaternion m2c_quaternion_matrix(m2c_rotation_quaternion[0], m2c_rotation_quaternion[1], m2c_rotation_quaternion[2], m2c_rotation_quaternion[3]);
		tf::Matrix3x3 m2c_rotation_angle(m2c_quaternion_matrix);
		double roll, pitch, yaw;
		m2c_rotation_angle.getRPY(roll, pitch, yaw);

		std::cout << "Now frame: " << this->frame_number << std::endl;
		outfile << ++this->frame_number << "," << initial_guess(0, 3) << "," << initial_guess(1, 3) << "," << 0 << "," << yaw << "," << pitch << "," << roll << std::endl;
		// transformation_record << transformation << std::endl
		// 					  << std::endl
		// 					  << std::endl
		// 					  << std::endl;

		if (this->frame_number == 396)
			ROS_INFO("Nuscenes bag finished");



		// 除以frequency ratio 算出在一frame的lidar point當中我們的odom是多少
		// 觀察csv後發現單純的icp下點基本上沒有移動
		// 我猜或許是因為定位的環境是沒有甚麼特徵的地方，所以icp基本不會移動，只好靠odom來幫我們修正了
		initial_guess(0, 3) += this->diff_x / this->frequency_ratio;
		initial_guess(1, 3) += this->diff_y / this->frequency_ratio;
		initial_guess(2, 3) += this->diff_z / this->frequency_ratio;


	}

	/**
	 * @brief Destroy the icp localization object
	 *
	 */
	~icp_localization()
	{
		this->outfile.close();
	}

	/**
	 * @brief Get the transform between link_name(source) to base_link(target)
	 *
	 * @param link_name source link(lidar)
	 * @return Eigen::Matrix4f Homogeneous Transformation Matrix
	 */
	Eigen::Matrix4f get_transform(std::string link_name)
	{
		tf::StampedTransform transform;
		Eigen::Matrix4f eigen_transform;

		try
		{
			tf_listener.waitForTransform("car", link_name, ros::Time(0), ros::Duration(5.0));
			tf_listener.lookupTransform("car", link_name, ros::Time(0), transform);
		}
		catch (tf::TransformException ex)
		{
			ROS_ERROR("%s", ex.what());
			return eigen_transform;
		}
		Eigen::Quaternionf q(transform.getRotation().getW(), transform.getRotation().getX(), transform.getRotation().getY(), transform.getRotation().getZ());
		Eigen::Matrix3f mat = q.toRotationMatrix();
		eigen_transform << 		mat(0, 0), 		mat(0, 1), 		mat(0, 2), 		transform.getOrigin().getX(),
								mat(1, 0), 		mat(1, 1), 		mat(1, 2), 		transform.getOrigin().getY(),
								mat(2, 0), 		mat(2, 1), 		mat(2, 2), 		transform.getOrigin().getZ(),
										0, 				0, 				0, 								   1;
		return eigen_transform;
	}

	/**
	 * @brief transfer sensor_msgs::PointCloud2 to pcl::pointcloud
	 *
	 * @param msg a rostopic from map publisher using sensor_msgs::PointCloud2
	 */
	void map_callback(const sensor_msgs::PointCloud2::ConstPtr &msg)
	{

		pcl::PCLPointCloud2::Ptr filtered_map(new pcl::PCLPointCloud2());
		std::cout << "Get map.\n";
		// ROS_INFO("Get map");
		pcl::fromROSMsg(*msg, *map);

		map_ready = true;
	}

	/**
	 * @brief
	 *
	 * @param msg a rostopic from wheel_odometry using nav_msgs::Odometry
	 */
	void odom_callback(const nav_msgs::Odometry::ConstPtr &msg){

		this->diff_x = msg->pose.pose.position.x - this->odom_x;
		this->diff_y = msg->pose.pose.position.y - this->odom_y;
		this->diff_z = msg->pose.pose.position.z - this->odom_z;
		this->odom_x = msg->pose.pose.position.x;
		this->odom_y = msg->pose.pose.position.y;
		this->odom_z = msg->pose.pose.position.z;

	}
};

int main(int argc, char **argv)
{

	ros::init(argc, argv, "icp_locolization");
	ros::NodeHandle n("~");
	icp_localization icp_localizer(n);
	ros::spin();
}
