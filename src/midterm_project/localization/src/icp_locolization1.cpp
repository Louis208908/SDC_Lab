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


class icp_localization{

private:

	// =============== ros util parameters ===============
	ros::NodeHandle nh;
	ros::Subscriber sub_lidar_scan;
	tf::TransformListener tf_listener;
	tf::TransformBroadcaster tf_broadcaster;


	// =============== variables of transformation ===============zz
	double init_yaw;
	double init_x, init_y,init_z;
	Eigen::Matrix4f initial_guess;
	sensor_msgs::PointCloud2 Final_map;
	Eigen::Matrix4f c2l_eigen_transform;
	sensor_msgs::PointCloud2 Final_cloud;
	pcl::PointCloud<pcl::PointXYZI>::Ptr map;	


	// =============== variables of output file ===============
	int frame;
	std::ofstream outfile;
	std::string map_path, result_path;

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
	icp_localization(ros::NodeHandle _nh){

		std::vector<float> trans, rot;
		this->nh = _nh;
		this->frame_number = 0;
		std::cout << "Initializing ICP...\n";
		this->sub_lidar_scan = this->nh.subscribe("/lidar_points", 400000, &icp_localization::lidar_scanning, this);

		// grasping ros parameters
		_nh.param<int>("frame", frame, 0);
		_nh.param<double>("init_x", init_x, 0.15);
		_nh.param<double>("init_y", init_y, 0.15);
		_nh.param<double>("init_z", init_z, 0.15);
		_nh.param<double>("init_yaw", init_yaw, 0.15);
		_nh.param<double>("mapLeafSize", map_leaf_size, 0.15);
		_nh.param<double>("scanLeafSize", scan_leaf_size, 0.15);
		_nh.param<std::string>("map_path", map_path, "itri_map.pcd");
		_nh.param<std::string>("result_save_path", result_path, "result1.csv");
		_nh.param<std::vector<float>>("baselink2lidar_rot", rot, std::vector<float>());
		_nh.param<std::vector<float>>("baselink2lidar_trans", trans, std::vector<float>());

		// 把itri.yaml中的transform link存下來
		if (trans.size() != 3 | rot.size() != 4)
			ROS_ERROR("transform not set properly");

		Eigen::Quaternionf link_quaternion(rot.at(3), rot.at(0), rot.at(1), rot.at(2));
		Eigen::Matrix3f link_rotation = link_quaternion.toRotationMatrix();

		// equivalent to get_transform("velodyne");
		c2l_eigen_transform << 		link_rotation(0, 0), link_rotation(0, 1), link_rotation(0, 2), trans.at(0),
									link_rotation(1, 0), link_rotation(1, 1), link_rotation(1, 2), trans.at(1),
									link_rotation(2, 0), link_rotation(2, 1), link_rotation(2, 2), trans.at(2),
													  0, 		   		   0, 		  			0, 		     1;

		// load map
		this->map = (new pcl::PointCloud<pcl::PointXYZI>)->makeShared();
		if (pcl::io::loadPCDFile<pcl::PointXYZI>(map_path, *this->map) == -1) {
			PCL_ERROR("Couldn't read file map_downsample.pcd \n");
			exit(0);
		}

		// getting initial guess
		this->initial_guess = get_initial_guess();
		std::cout << "Get initial guess: \n";
		std::cout << this->initial_guess << std::endl;
		std::cout << "Ready to localization\n";
		
		std::cout << "Result path: " << result_path << std::endl;
		outfile.open(result_path);
		outfile << "id,x,y,z,yaw,pitch,roll" << std::endl;
	}

	/**
	 * @brief Get the initial guess object 
	 * 
	 * @return Eigen::Matrix4f initial guess of map to car transfromation
	 */
	Eigen::Matrix4f get_initial_guess(){

		Eigen::Matrix4f initial_guess;
		geometry_msgs::PointStampedConstPtr gps_point;
		gps_point = ros::topic::waitForMessage<geometry_msgs::PointStamped>("/gps", this->nh);

		double yaw = this->init_yaw;


		// initial_guess << 	cos(yaw),	-sin(yaw),		0,		gps_point->point.x,
		// 					sin(yaw),	 cos(yaw),		0,		gps_point->point.y,
		// 						   0,			0,		1,		gps_point->point.z,
		// 						   0,			0,		0,						 1;


		initial_guess << 	cos(yaw),	-sin(yaw),		0,		this->init_x,
							sin(yaw),	 cos(yaw),		0,		this->init_y,
								   0,			0,		1,		this->init_z,
								   0,			0,		0,				   1;

		return initial_guess;
	}


	/**
	 * @brief Down sampling pointCloud of lidar scan
	 * 
	 * @param msg ros topic of lidar scan
	 * @return pcl::PointCloud<pcl::PointXYZI>::Ptr pointer of PointCloud of lidar scan after downsampling
	 */
	pcl::PointCloud<pcl::PointXYZI>::Ptr down_sampling(const sensor_msgs::PointCloud2::ConstPtr &msg){

		pcl::PCLPointCloud2::Ptr filtered_scan(new pcl::PCLPointCloud2());
		pcl::PointCloud<pcl::PointXYZI>::Ptr raw_scan(new pcl::PointCloud<pcl::PointXYZI>);
		pcl::PointCloud<pcl::PointXYZI>::Ptr result_scan(new pcl::PointCloud<pcl::PointXYZI>);

		// sensor_msgs -> PointCloud
		pcl::fromROSMsg(*msg, *raw_scan);
		// PointCloud -> PointCloud2
		pcl::toPCLPointCloud2(*raw_scan, *filtered_scan);

		// 可以傳PointCloud 也可以傳PointCloud2, 這邊因為我們傳的是PointCloud2(放在<>裡面的型別)
		// 因此我們filter的input、output的型別都是PointCloud2
		// 所以我們上面需要先將msg轉成PointCloud後 再轉乘PointCloud2,
		pcl::VoxelGrid<pcl::PCLPointCloud2> voxel_filter;
		voxel_filter.setInputCloud(filtered_scan);
		voxel_filter.setLeafSize((float)scan_leaf_size, (float)scan_leaf_size, (float)scan_leaf_size);
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
	void lidar_scanning(const sensor_msgs::PointCloud2::ConstPtr &msg){

		pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_map(new pcl::PointCloud<pcl::PointXYZI>);
		pcl::PointCloud<pcl::PointXYZI> aligned_points;

		// =============== Passthrough ===============
		pcl::PassThrough<pcl::PointXYZI> passthrough_filter;
		passthrough_filter.setInputCloud(this->map);
		passthrough_filter.setFilterFieldName("x");
		passthrough_filter.setFilterLimits(this->initial_guess(0, 3) - 100.0, this->initial_guess(0, 3) + 100.0);
		passthrough_filter.filter(*filtered_map);

		passthrough_filter.setInputCloud(filtered_map);
		passthrough_filter.setFilterFieldName("y");
		passthrough_filter.setFilterLimits(this->initial_guess(1, 3) - 100.0, this->initial_guess(1, 3) + 100.0);
		passthrough_filter.filter(*filtered_map);

		// =============== Down sampling lidar scan ===============
		pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_scan(new pcl::PointCloud<pcl::PointXYZI>);
		pcl::PointCloud<pcl::PointXYZI>::Ptr final_filtered_scan(new pcl::PointCloud<pcl::PointXYZI>);
		filtered_scan = down_sampling(msg);

		// =============== transform lidar scan to car ===============
		// transformPointCloud(source, target, transform)
		// =============== Illustration of transformPointCloud ===============
		// Task: 將source的點雲使用transform轉換
		//
		//               資料                                        資料
		// 			    /										   /
		// 			  /										     /
		// 			/										   /	
		//	 source				======>					source
		//											   /
		// 											 /	transform
		// 										   /
		// 									target
		// =============== Illustration End ===============
		// 本來是由lidar看出去的資料，現在變成從車子(base_link)看出去
		transformPointCloud(*filtered_scan, *filtered_scan, c2l_eigen_transform);

		// =============== start performing ICP ===============
		pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;
		icp.setInputSource(filtered_scan);
		icp.setInputTarget(filtered_map);
		icp.setMaximumIterations(1000);		  			//最大迭代次數
		icp.setTransformationEpsilon(1e-12);		  	//上次轉換與當前轉換的差值(early stop)
		icp.setMaxCorrespondenceDistance(1.00);		  	// Distance to see
		icp.setEuclideanFitnessEpsilon(0.01);		  	//前後兩次迭代誤差的差值(early stop)
		icp.setRANSACOutlierRejectionThreshold(0.05); 	//距離小於等於inlier_threshold的點對才會被當成是RANSAC的inlier
		icp.align(aligned_points, this->initial_guess);

		// =============== Get car pos using ICP result===============
		// initial guess是map 看向 car的轉換
		this->initial_guess = icp.getFinalTransformation();
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
		outfile << ++this->frame_number << "," << initial_guess(0, 3) << "," << initial_guess(1, 3) << "," << initial_guess(2, 3) << "," << yaw << "," << pitch << "," << roll << std::endl;

		if (this->frame_number == this->frame){
			ROS_INFO("ITRI bag finished");
		}
	}


	/**
	 * @brief Destroy the icp localization object
	 * 
	 */
	~icp_localization(){
		this->outfile.close();
	}

	/**
	 * @brief Get the transform between base_link(target) to link_name(source)  (由target看向source)
	 * 
	 * @param link_name source link(lidar)
	 * @return Eigen::Matrix4f Homogeneous Transformation Matrix
	 */
	Eigen::Matrix4f get_transform(std::string link_name){
		tf::StampedTransform transform;
		Eigen::Matrix4f eigen_transform;

		try
		{
			tf_listener.waitForTransform("base_link", link_name, ros::Time(0), ros::Duration(5.0));
			tf_listener.lookupTransform("base_link", link_name, ros::Time(0), transform);
		}
		catch (tf::TransformException ex)
		{
			ROS_ERROR("%s", ex.what());
			return eigen_transform;
		}
		Eigen::Quaternionf q(transform.getRotation().getW(), transform.getRotation().getX(), transform.getRotation().getY(), transform.getRotation().getZ());
		Eigen::Matrix3f mat = q.toRotationMatrix();
		eigen_transform << 	mat(0, 0), mat(0, 1), mat(0, 2), transform.getOrigin().getX(),
							mat(1, 0), mat(1, 1), mat(1, 2), transform.getOrigin().getY(),
							mat(2, 0), mat(2, 1), mat(2, 2), transform.getOrigin().getZ(),
									0, 		   0, 		  0, 							1;
		return eigen_transform;
	}

};


int main(int argc, char **argv){

	ros::init(argc, argv, "icp_locolization");
	ros::NodeHandle n("~");
	icp_localization icp_localizer(n);
	ros::spin();

}
