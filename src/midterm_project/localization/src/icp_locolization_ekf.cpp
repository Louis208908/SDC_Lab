#include "math.h"
#include <string>
#include "stdio.h"
#include <fstream>
#include <sstream>
#include <iostream>
#include <ros/ros.h>
#include <ros/package.h>
#include <pcl/io/pcd_io.h>
#include "sensor_msgs/Imu.h"
#include <pcl/point_types.h>
#include <std_msgs/String.h>
#include <pcl/conversions.h>
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
#include <geometry_msgs/PoseWithCovarianceStamped.h>

class icp_localization
{

private:
	// =============== ros util parameters ===============
	ros::NodeHandle nh;
	ros::Publisher pub_map;
	ros::Subscriber sub_map;
	ros::Subscriber sub_gps;
	ros::Publisher pub_pose;
	ros::Subscriber sub_odom;
	ros::Publisher pub_lidar;
	bool gps_ready, map_ready;
	ros::Subscriber sub_filter;
	ros::Publisher pub_set_pose;
	ros::Publisher pub_car_pose;
	ros::Subscriber sub_lidar_scan;
	tf::TransformListener tf_listener;
	tf::TransformBroadcaster tf_broadcaster;

	// =============== variables of transformation ===============
	bool use_gps;
	bool use_odom;
	double odom_ratio;
	double lidar_ratio;
	double frequency_ratio;
	double odom_x, odom_y, odom_z;
	double diff_x, diff_y, diff_z;
	Eigen::Matrix4f initial_guess;
	sensor_msgs::PointCloud2 Final_map;
	geometry_msgs::Transform car2Lidar;
	Eigen::Matrix4f c2l_eigen_transform;
	sensor_msgs::PointCloud2 Final_cloud;
	double init_x, init_y, init_z,init_yaw;
	pcl::PointCloud<pcl::PointXYZI>::Ptr map;
	double filtered_x, filtered_y, filtered_z;

	// =============== variables of output file ===============
	std::ofstream outfile;
	std::ofstream transformation_record;
	std::string map_path, result_path, transformation_path;

	// =============== variables of ICP parameters ===============
	int total_frame;
	bool use_filter;
	double fix_rate;
	double map_leaf_size;
	double scan_leaf_size;
	double previous_score;

public:
	int frame_number;

	/**
	 * @brief Construct a new icp localization object, initializing ICP(get initial guess)
	 *
	 * @param _nh ros node handler
	 */
	icp_localization(ros::NodeHandle _nh) : map(new pcl::PointCloud<pcl::PointXYZI>)
	{

		std::vector<float> trans, rot;
		std::cout << "Initializing ICP...\n";
		this->nh = _nh;

		// grasping ros parameters
		_nh.param<bool>("use_gps", use_gps, true);
		_nh.param<double>("init_x", init_x, 0.15);
		_nh.param<double>("init_y", init_y, 0.15);
		_nh.param<double>("init_z", init_z, 0.15);
		_nh.param<bool>("use_odom", use_gps, true);
		_nh.param<double>("fix_rate", fix_rate, 1.0);
		_nh.param<double>("init_yaw", init_yaw, 0.15);
		_nh.param<int>("total_frame", total_frame, 1);
		_nh.param<bool>("use_filter", use_filter, true);
		_nh.param<double>("odom_ratio", odom_ratio, 1.0);
		_nh.param<double>("lidar_ratio", lidar_ratio, 1.0);
		_nh.param<double>("mapLeafSize", map_leaf_size, 0.15);
		_nh.param<double>("scanLeafSize", scan_leaf_size, 0.15);
		_nh.param<std::string>("map_path", map_path, "nuscenes_map.pcd");
		_nh.param<std::string>("result_save_path", result_path, "result2.csv");
		_nh.param<std::vector<float>>("baselink2lidar_rot", rot, std::vector<float>());
		_nh.param<std::vector<float>>("baselink2lidar_trans", trans, std::vector<float>());
		_nh.param<std::string>("transformation_path", transformation_path, "transformation.txt");

		this->odom_x = 0;
		this->odom_y = 0;
		this->odom_z = 0;
		this->diff_x = 0;
		this->diff_y = 0;
		this->diff_z = 0;
		this->filtered_x = 0;
		this->filtered_y = 0;
		this->filtered_z = 0;
		this->frame_number = 0;
		this->previous_score = 0;
		this->pub_map = nh.advertise<sensor_msgs::PointCloud2>("/map", 1);
		this->pub_lidar = this->nh.advertise<sensor_msgs::PointCloud2>("/transformed_points", 1);
		if(this->use_odom)
			this->sub_odom = this->nh.subscribe("/wheel_odometry", 4000000, &icp_localization::odom_callback, this);
		this->sub_filter = this->nh.subscribe("/odometry/filtered_wheel", 4000000, &icp_localization::filter_callback, this);
		this->sub_lidar_scan = this->nh.subscribe("/lidar_points", 4000000, &icp_localization::lidar_scanning, this);
		pub_pose = this->nh.advertise<geometry_msgs::PoseStamped>("/lidar_pose", 1);
		pub_car_pose = this->nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/car_pose", 1);
		pub_set_pose = this->nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/set_pose", 1);

		// 把itri.yaml中的transform link存下來
		if (trans.size() != 3 | rot.size() != 4)
			ROS_ERROR("transform not set properly");

		Eigen::Quaternionf link_quaternion(rot.at(3), rot.at(0), rot.at(1), rot.at(2));
		Eigen::Matrix3f link_rotation = link_quaternion.toRotationMatrix();

		c2l_eigen_transform << 		link_rotation(0, 0), 	link_rotation(0, 1), 	link_rotation(0, 2), 	trans.at(0),
									link_rotation(1, 0), 	link_rotation(1, 1), 	link_rotation(1, 2), 	trans.at(1),
									link_rotation(2, 0), 	link_rotation(2, 1), 	link_rotation(2, 2), 	trans.at(2),
													  0, 					  0, 					  0, 			  1;

		car2Lidar.translation.x = trans.at(0);
		car2Lidar.translation.y = trans.at(1);
		car2Lidar.translation.z = trans.at(2);
		car2Lidar.rotation.x = rot.at(0);
		car2Lidar.rotation.y = rot.at(1);
		car2Lidar.rotation.z = rot.at(2);
		car2Lidar.rotation.w = rot.at(3);

		// load map
		this->map = (new pcl::PointCloud<pcl::PointXYZI>)->makeShared();
		if (pcl::io::loadPCDFile<pcl::PointXYZI>(map_path, *this->map) == -1)
		{
			PCL_ERROR("Couldn't read file map_downsample.pcd \n");
			exit(0);
		}

		std::cout << "Loaded "
				  << map->width * map->height
				  << " data points from nuscenes_map_downsample.pcd with the following fields: "
				  << std::endl;

		this->frequency_ratio = lidar_ratio / (double)odom_ratio;

		// getting initial guess
		std::cout << "Finding initial guess. \n";
		this->initial_guess = get_initial_guess();
		std::cout << "Get initial guess: \n";
		std::cout << this->initial_guess << std::endl;
		std::cout << "Ready to localization\n";

		std::cout << "Result path: " << result_path << std::endl;
		// result_path += ".csv";
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

		// double init_x = 1773.433472;
		// double init_y = 866.344177;
		// double init_z = 0.0;
		double yaw = this->init_yaw;
		double init_x = this->init_x;
		double init_y = this->init_y;
		double init_z = this->init_z;

		if(this->use_gps){

			initial_guess << 		cos(yaw), -sin(yaw), 	0, 	gps_point->point.x,
									sin(yaw),  cos(yaw), 	0, 	gps_point->point.y,
										   0, 		  0, 	1, 	gps_point->point.z,
										   0, 		  0, 	0, 					 1;
		}
		else{

			initial_guess << 	cos(yaw),   -sin(yaw), 		0, 		init_x,
								sin(yaw),    cos(yaw), 		0, 		init_y,
									   0,           0, 		1, 		init_z,
									   0,           0, 		0, 		     1;
		}

		tf::Matrix3x3 rot;
		rot.setValue(
			static_cast<double>(this->initial_guess(0, 0)), static_cast<double>(this->initial_guess(0, 1)), static_cast<double>(this->initial_guess(0, 2)),
			static_cast<double>(this->initial_guess(1, 0)), static_cast<double>(this->initial_guess(1, 1)), static_cast<double>(this->initial_guess(1, 2)),
			static_cast<double>(this->initial_guess(2, 0)), static_cast<double>(this->initial_guess(2, 1)), static_cast<double>(this->initial_guess(2, 2)));
		tf::Vector3 trans(this->initial_guess(0, 3), this->initial_guess(1, 3), this->initial_guess(2, 3));
		tf::Transform transform(rot, trans);

		geometry_msgs::PoseWithCovarianceStamped pose_car;
		pose_car.header = gps_point->header;
		pose_car.header.frame_id = "world"; // this map is world frame
		pose_car.pose.pose.position.x = init_x;
		pose_car.pose.pose.position.y = init_y;
		pose_car.pose.pose.position.z = init_z;
		pose_car.pose.pose.orientation.x = transform.getRotation().getX(); // orientation ~ rotation
		pose_car.pose.pose.orientation.y = transform.getRotation().getY();
		pose_car.pose.pose.orientation.z = transform.getRotation().getZ();
		pose_car.pose.pose.orientation.w = transform.getRotation().getW();
		pose_car.pose.covariance = {10, 0, 0, 0, 0, 0,
									0, 10, 0, 0, 0, 0,
									0, 0, 10, 0, 0, 0,
									0, 0, 0, 10, 0, 0,
									0, 0, 0, 0, 10, 0,
									0, 0, 0, 0, 0, 10};
		pub_set_pose.publish(pose_car); // publish car pose

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
		std::cout << "Init guess by EKF\n";
		std::cout << this->initial_guess << std::endl;

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
		voxel_filter.setFilterLimits(-2.0, 8);
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

		pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_map(new pcl::PointCloud<pcl::PointXYZI>);
		pcl::PointCloud<pcl::PointXYZI> aligned_points;

		// =============== Passthrough ===============
		if(this->use_filter){
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
		}

		// =============== Down sampling lidar scan ===============
		pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_scan(new pcl::PointCloud<pcl::PointXYZI>);
		pcl::PointCloud<pcl::PointXYZI>::Ptr final_filtered_scan(new pcl::PointCloud<pcl::PointXYZI>);
		filtered_scan = down_sampling(msg);

		// =============== transform scan to car ===============
		transformPointCloud(*filtered_scan, *filtered_scan, c2l_eigen_transform);


		pcl::VoxelGrid<pcl::PointXYZI> voxel_filter;
		voxel_filter.setInputCloud(filtered_scan);
		voxel_filter.setFilterFieldName("z");
		voxel_filter.setFilterLimits(1.0, 7.5);
		voxel_filter.setLeafSize(0.1f, 0.1f, 0.4f);
		voxel_filter.filter(*filtered_scan);

		// =============== start performing ICP ===============
		pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;
		icp.setInputSource(filtered_scan);
		if(this->use_filter)
			icp.setInputTarget(filtered_map);
		else
			icp.setInputTarget(this->map);
		icp.setMaximumIterations(1000);				 
		icp.setTransformationEpsilon(1e-12);		 
		icp.setMaxCorrespondenceDistance(0.75);		
		icp.setEuclideanFitnessEpsilon(0.00075);		 
		icp.setRANSACOutlierRejectionThreshold(0.05); 
		icp.align(aligned_points, this->initial_guess);

		// publish transformed points and map
		sensor_msgs::PointCloud2::Ptr out_msg(new sensor_msgs::PointCloud2);
		pcl::toROSMsg(aligned_points, *out_msg);
		out_msg->header = msg->header;
		out_msg->header.frame_id = "world";
		pub_lidar.publish(out_msg);

		sensor_msgs::PointCloud2::Ptr map_cloud(new sensor_msgs::PointCloud2);
		if(use_filter)
			pcl::toROSMsg(*filtered_map, *map_cloud);
		else
			pcl::toROSMsg(*this->map, *map_cloud);

		map_cloud->header.frame_id = "world";
		this->pub_map.publish(*map_cloud);


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
		std::cout << "Init guess by ICP\n";
		std::cout << this->initial_guess << std::endl;

		if (this->frame_number == this->total_frame){
			ROS_INFO("Nuscenes bag finished");
			system("pkill roslaunch");
		}

		// broadcast transforms
		tf::Matrix3x3 rot;
		rot.setValue(
			static_cast<double>(this->initial_guess(0, 0)), static_cast<double>(this->initial_guess(0, 1)), static_cast<double>(this->initial_guess(0, 2)),
			static_cast<double>(this->initial_guess(1, 0)), static_cast<double>(this->initial_guess(1, 1)), static_cast<double>(this->initial_guess(1, 2)),
			static_cast<double>(this->initial_guess(2, 0)), static_cast<double>(this->initial_guess(2, 1)), static_cast<double>(this->initial_guess(2, 2)));
		tf::Vector3 trans(this->initial_guess(0, 3), this->initial_guess(1, 3), this->initial_guess(2, 3));
		tf::Transform transform(rot, trans);
		// br.sendTransform(tf::StampedTransform(transform.inverse(), msg->header.stamp, lidarFrame, mapFrame));


		geometry_msgs::PoseWithCovarianceStamped pose_car;
		pose_car.header = msg->header;
		pose_car.header.frame_id = "world"; // this map is world frame
		pose_car.pose.pose.position.x = trans.getX();
		pose_car.pose.pose.position.y = trans.getY();
		pose_car.pose.pose.position.z = trans.getZ();
		pose_car.pose.pose.orientation.x = transform.getRotation().getX(); // orientation ~ rotation
		pose_car.pose.pose.orientation.y = transform.getRotation().getY();
		pose_car.pose.pose.orientation.z = transform.getRotation().getZ();
		pose_car.pose.pose.orientation.w = transform.getRotation().getW();
		pose_car.pose.covariance = {10, 0, 0, 0, 0, 0,
									0, 10, 0, 0, 0, 0,
									0, 0, 10, 0, 0, 0,
									0, 0, 0, 10, 0, 0,
									0, 0, 0, 0, 10, 0,
									0, 0, 0, 0, 0, 10};
		pub_car_pose.publish(pose_car); // publish car pose

		// 除以frequency ratio 算出在一frame的lidar point當中我們的odom是多少
		// 觀察csv後發現單純的icp下點基本上沒有移動
		// 我猜或許是因為定位的環境是沒有甚麼特徵的地方，所以icp基本不會移動，只好靠odom來幫我們修正了
		initial_guess(0, 3) += this->diff_x / this->frequency_ratio;
		initial_guess(1, 3) += this->diff_y / this->frequency_ratio;
		initial_guess(2, 3) += this->diff_z / this->frequency_ratio;

		if (icp.getFitnessScore() > this->previous_score || !icp.hasConverged())
			this->frequency_ratio * this->fix_rate;
		else
			this->frequency_ratio / this->fix_rate;
		this->previous_score = icp.getFitnessScore();
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
	 * @brief Get the transform between base_link(target) to  link_name(source)在target坐標系當中看向source
	 *
	 * @param link_name source link(lidar)
	 * @return Eigen::Matrix4f Homogeneous Transformation Matrix
	 */
	Eigen::Matrix4f get_transform(std::string target,std::string link_name)
	{
		tf::StampedTransform transform;
		Eigen::Matrix4f eigen_transform;

		try
		{
			tf_listener.waitForTransform(target, link_name, ros::Time(0), ros::Duration(5.0));
			tf_listener.lookupTransform(target, link_name, ros::Time(0), transform);
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

	/**
	 * @brief
	 *
	 * @param msg a rostopic from wheel_odometry using nav_msgs::Odometry
	 */
	void filter_callback(const nav_msgs::Odometry::ConstPtr &msg){

		// nav_msgs::Odometry to Eigen::Matrix4f
		Eigen::Matrix4f EKFeigen;

		Eigen::Quaterniond quat;
		quat.w() = msg->pose.pose.orientation.w;
		quat.x() = msg->pose.pose.orientation.x;
		quat.y() = msg->pose.pose.orientation.y;
		quat.z() = msg->pose.pose.orientation.z;

		Eigen::Isometry3d isometry = Eigen::Isometry3d::Identity();
		isometry.linear() = quat.toRotationMatrix();
		isometry.translation() = Eigen::Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
		EKFeigen = isometry.matrix().cast<float>();

		// Eigen::Matrix4f EKFmatrix4f = EKFeigen * transform_c2l_4f; // (Affine3f) * (Matrix4f)
		Eigen::Matrix4f EKFmatrix4f = EKFeigen * get_transform("origin", "car").inverse() * get_transform("world", "origin").inverse(); // (Affine3f) * (Matrix4f)

		this->initial_guess(0,3) = (EKFmatrix4f.inverse())(0,3);
		this->initial_guess(1,3) = (EKFmatrix4f.inverse())(1,3);
		this->initial_guess(2,3) = (EKFmatrix4f.inverse())(2,3);

		// std::cout << "Init guess by EKF\n";
		// std::cout << EKFmatrix4f.inverse() << std::endl;
	}
};

int main(int argc, char **argv)
{

	ros::init(argc, argv, "icp_locolization");
	ros::NodeHandle n("~");
	icp_localization icp_localizer(n);
	ros::spin();
}
