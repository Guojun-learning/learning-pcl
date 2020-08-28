#pragma once
#ifndef DELETE_GROUND_H
#define DELETE_GROUND_H

#include<iostream>
#include<ros/ros.h>
#include<sensor_msgs/PointCloud2.h>

#include<pcl_conversions/pcl_conversions.h>
#include<pcl/point_types.h>
#include<pcl/conversions.h>
#include<pcl/filters/extract_indices.h>
#include<pcl_ros/transforms.h>

namespace sensor_lidar
{

    #define CLIP_HEIGHT 0.2 //截取掉高于雷达自身0.2米的点
    #define MIN_DISTANCE 2.4 //裁剪小于此距离的点，消除近距离点的影响
    #define MAX_DISTANCE -1//裁剪超过此距离的点，消除远距离的点(小于0.0不执行)
    #define RADIAL_DIVIDER_ANGLE 0.18 //360度均分的角度（0.18度是VLP32C雷达的水平光束发散间隔）
    #define SENSOR_HEIGHT 1.78 //表示lidar布置高度

    #define concentric_divider_distance_ 0.01 //0.1 meters default
    #define min_height_threshold_ 0.05
    #define local_max_slope_ 8   //设定的同条射线上邻近两点的坡度阈值
    #define general_max_slope_ 5 //整个地面的坡度阈值（单位：度（degree））
    #define reclass_distance_threshold_ 0.2//两点距离（再次分类用）

    class DeleteGround
    {
        private:

            ros::Subscriber sub_point_cloud_;
            ros::Publisher pub_ground_;
            ros::Publisher pub_no_ground_;
            //新的结构体保存点
            struct PointXYZIRTColor
            {
                pcl::PointXYZI point;
                float radius;//XY 平面，R(m)
                float theta;//XY平面，angle(deg)

                size_t radial_div;//index of radial to which this point belongs to 
                size_t concentric_div;//index of the concentric  division to which this points belongs to 
                size_t original_index;//index of this point in the source pointcloud
            };
            typedef std::vector<PointXYZIRTColor> PointCloudXYZRTColor;
    
            size_t radial_dividers_num_;
            size_t concentric_dividers_num_;

        private:
            //回调函数
            void callBack_point(const sensor_msgs::PointCloud2ConstPtr &in_cloud_ptr);

            //PointXYZI转换为定义的结构体类型点
            void pointXYZI_to_pointCloudXYZRTColor(const pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud,PointCloudXYZRTColor &out_organized_points,std::vector<pcl::PointIndices>&out_radial_divided_indices,std::vector<PointCloudXYZRTColor>&out_radial_ordered_clouds);

            //点云裁减和过滤
            void clip_above_pointCloud(double clip_height,const pcl::PointCloud<pcl::PointXYZI>::Ptr in,const pcl::PointCloud<pcl::PointXYZI>::Ptr out);

            //裁减一定距离距离的点
            void remove_close_pointsCloud(double min_distance,double max_distance,const pcl::PointCloud<pcl::PointXYZI>::Ptr in,const pcl::PointCloud<pcl::PointXYZI>::Ptr out);

            //地面分割
            void classify_pointColud(std::vector<PointCloudXYZRTColor> &input_radial_ordered_clouds, pcl::PointIndices &out_ground_indices,pcl::PointIndices &out_no_ground_indices);

            //发布点云信息
            void publish_pointCloud(const ros::Publisher &input_publisher,const pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud_to_publish_ptr,const std_msgs::Header &input_header);
            
        public:
            DeleteGround(ros::NodeHandle &nh);
            ~DeleteGround();

    };
}

#endif