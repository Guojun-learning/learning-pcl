#pragma once 
#ifndef EUCLIDEAN_CLUSTER_H
#define EUCLIDEAN_CLUSTER_H

#include<iostream>

#include<ros/ros.h>

#include<pcl_conversions/pcl_conversions.h>
#include<pcl/point_types.h>
#include<pcl/conversions.h>
#include<pcl_ros/transforms.h>

#include<pcl/filters/extract_indices.h>
#include<pcl/filters/voxel_grid.h>
#include<pcl/segmentation/sac_segmentation.h>
#include<pcl/segmentation/extract_clusters.h>
#include<pcl/segmentation/conditional_euclidean_clustering.h>

#include <pcl/common/common.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/search/organized.h>
#include <pcl/search/kdtree.h>

#include <std_msgs/Header.h>

#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>

#include <sensor_msgs/PointCloud2.h>

namespace sensor_lidar
{
    #define LEAF_SIZE  0.1  //定义降采样的 leaf size 
    #define MIN_CLUSTER_SIZE  20 //最小聚类点数
    #define MAX_CLUSTER_SIZE 2000 //最大聚类点数

    class Euclidean
    {
        private:
            //定义结构体用来存放检测的目标体
            struct Detected_Obj
            {
                jsk_recognition_msgs::BoundingBox bounding_box_;
                pcl::PointXYZ min_point_;
                pcl::PointXYZ max_point_;
                pcl::PointXYZ centroid_;
            };
            ros::Subscriber sub_point_colud_;
            ros::Publisher pub_bounding_boxs_;

            std::vector<double> seg_distance_;
            std::vector<double> cluster_distance_;

            std_msgs::Header point_cloud_header_;

            //点云降采样处理
            void voxel_grid_filer(pcl::PointCloud<pcl::PointXYZ>::Ptr in, pcl::PointCloud<pcl::PointXYZ>::Ptr out, double leaf_size);

            //
            void cluster_by_distance(pcl::PointCloud<pcl::PointXYZ>::Ptr in_pc, std::vector<sensor_lidar::Euclidean::Detected_Obj> &obj_list);

            //细分聚类
            void cluster_segment(pcl::PointCloud<pcl::PointXYZ>::Ptr in_pc,double in_max_cluster_distance, std::vector<sensor_lidar::Euclidean::Detected_Obj> & obj_list);

            //回调函数
            void cluster_CallBack(const sensor_msgs::PointCloud2ConstPtr &in_cloud_ptr);

            //信息发布
            void publish_cloud(const ros::Publisher &in_publisher,const pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud_to_publish_ptr,const std_msgs::Header &in_header);

        public:
            //构造函数
            Euclidean(ros::NodeHandle &nh);
            ~Euclidean();

    };

}


















#endif