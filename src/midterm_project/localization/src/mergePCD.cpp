#include "dirent.h"
#include <ros/ros.h>
#include <ros/package.h>
#include "bits/stdc++.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>

#define FilePath "/home/louis/sdc_ws/data/nuscenes_maps"

using namespace std;


void findFileAmount(vector<string> &pcd_file_name)
{
    DIR *dir = NULL;
    struct dirent *entry;
    int fileAmount = 0;
    dir = opendir(FilePath);
    if (dir == NULL){
        printf("opendir failed!");
        return ;
    }
    else{
        while (entry = readdir(dir)){
            if (strstr(entry->d_name, ".pcd") != NULL){
                fileAmount ++;
                pcd_file_name.push_back(FilePath + string("/") + string(entry->d_name));
            }
            if (strstr(entry->d_name, "output.csv") != NULL)
                fileAmount--;
        }
        // show how many pcd file do we find
        cout << "fileAmount: " << fileAmount << endl;
        closedir(dir);
    }
    return ;
}

int main(int argc, char **argv){

    ros::init(argc, argv, "icp_locolization");
    ros::NodeHandle n("~");
    

    vector<string> pcd_file_name;

    findFileAmount(pcd_file_name);
    // merge all pcd map found in pcd_file_name into one pcd map
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZI>);

    for(auto pcd_file : pcd_file_name){
        cout << pcd_file << endl;
        // load pcd from pcd_file to cloud
        pcl::io::loadPCDFile<pcl::PointXYZI>(pcd_file, *cloud);
        *cloud_out += *cloud;
        // clear cloud
        cloud->clear();

    }

    // output merged pcd map
    pcl::io::savePCDFileASCII(FilePath + string("/") + string("merged.pcd"), *cloud_out);

    return 0;
}
