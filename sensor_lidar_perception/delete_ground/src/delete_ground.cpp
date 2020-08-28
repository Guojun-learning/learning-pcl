#include"delete_ground.h"
#include<ros/ros.h>


//构造函数
sensor_lidar::DeleteGround::DeleteGround(ros::NodeHandle &nh)
{
    sub_point_cloud_=nh.subscribe("/velodyne_points",10,&sensor_lidar::DeleteGround::callBack_point,this);//测试
    //sub_point_cloud_=nh.subscribe("/lslidar_point_cloud",10,&sensor_lidar::DeleteGround::callBack_point,this);//雷达C16

    pub_ground_=nh.advertise<sensor_msgs::PointCloud2>("/filtered_points_ground",10);
    pub_no_ground_=nh.advertise<sensor_msgs::PointCloud2>("/filtered_points_no_ground",10);
    ros::spin();
}
sensor_lidar::DeleteGround::~DeleteGround(){};

//回调函数
void sensor_lidar::DeleteGround::callBack_point(const sensor_msgs::PointCloud2ConstPtr &in_cloud_ptr)
{
    //step1:点云数据转换
    pcl::PointCloud<pcl::PointXYZI>::Ptr current_pc_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*in_cloud_ptr,*current_pc_ptr);//PointCloud2  to PointCloud

    //step2裁减点云
    //裁减高度点云
    pcl::PointCloud<pcl::PointXYZI>::Ptr cliped_pc_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    sensor_lidar::DeleteGround::clip_above_pointCloud(CLIP_HEIGHT,current_pc_ptr,cliped_pc_ptr);
    //裁减远近点云
    pcl::PointCloud<pcl::PointXYZI>::Ptr remove_pc_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    sensor_lidar::DeleteGround::remove_close_pointsCloud(MIN_DISTANCE,MAX_DISTANCE,cliped_pc_ptr,remove_pc_ptr);
    
    //step3
    //点云数据结构转换
    PointCloudXYZRTColor organized_points;
    std::vector<pcl::PointIndices> radial_division_indices;
    std::vector<PointCloudXYZRTColor> radial_ordered_clouds;
    radial_dividers_num_ =ceil(360/RADIAL_DIVIDER_ANGLE);
    sensor_lidar::DeleteGround::pointXYZI_to_pointCloudXYZRTColor(remove_pc_ptr,organized_points,radial_division_indices,radial_ordered_clouds);
    
   // std::vector<pcl::PointIndices> closest_indices;
    
     //step4
    //地面点云分割
    pcl::PointIndices ground_indices;
    pcl::PointIndices no_ground_indices;
    sensor_lidar::DeleteGround::classify_pointColud(radial_ordered_clouds,ground_indices,no_ground_indices);

    //step5
    //提取点云
    pcl::PointCloud<pcl::PointXYZI>::Ptr ground_pointColund_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr no_ground_pointColund_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    
    pcl::ExtractIndices<pcl::PointXYZI>extract_ground;
    extract_ground.setInputCloud(remove_pc_ptr);
    extract_ground.setIndices(boost::make_shared<pcl::PointIndices>(ground_indices));
    //地面
    extract_ground.setNegative(false);//true is remove the indices,false is leaves only the indices
    extract_ground.filter(*ground_pointColund_ptr);
    //非地面
    extract_ground.setNegative(true);
    extract_ground.filter(*no_ground_pointColund_ptr);

    //step6
    //发布点云
    sensor_lidar::DeleteGround::publish_pointCloud(pub_ground_,ground_pointColund_ptr,in_cloud_ptr->header);
    sensor_lidar::DeleteGround::publish_pointCloud(pub_no_ground_,no_ground_pointColund_ptr,in_cloud_ptr->header);

}

//PointXYZI转换为定义的结构体类型点
void sensor_lidar::DeleteGround::pointXYZI_to_pointCloudXYZRTColor(const pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud,
                                                                                                                                                    PointCloudXYZRTColor &out_organized_points,
                                                                                                                                                    std::vector<pcl::PointIndices>&out_radial_divided_indices,
                                                                                                                                                    std::vector<PointCloudXYZRTColor>&out_radial_ordered_clouds)
{
    out_organized_points.resize(input_cloud->points.size());
    out_radial_divided_indices.clear();
    out_radial_divided_indices.resize(radial_dividers_num_);
    out_radial_ordered_clouds.resize(radial_dividers_num_);
#pragma omp for
    for(size_t i=0;i<input_cloud->points.size();i++)
    {
        sensor_lidar::DeleteGround::PointXYZIRTColor new_points;
        auto radius=(float)sqrt((input_cloud->points[i].x*input_cloud->points[i].x)+(input_cloud->points[i].y*input_cloud->points[i].y));
        auto theta=(float)atan2(input_cloud->points[i].y,input_cloud->points[i].x)*180/M_PI;
        if(theta < 0)
        {
            theta +=360;
        }
        //角度微分
        auto radial_div=(size_t)floor(theta/RADIAL_DIVIDER_ANGLE);
        //半径微分
        auto concentric_div=(size_t)floor(radius/concentric_divider_distance_);

        new_points.point=input_cloud->points[i];
        new_points.radius=radius;
        new_points.theta=theta;
        new_points.radial_div=radial_div;
        new_points.concentric_div=concentric_div;
        new_points.original_index=i;

        out_organized_points[i] = new_points;

        //角度的微分组织射线
        out_radial_divided_indices[radial_div].indices.push_back(i);
        out_radial_ordered_clouds[radial_div].push_back(new_points);
    }
#pragma omp for
    //半径排序
    for(size_t i=0;i<radial_dividers_num_;i++)
    {
        std::sort(out_radial_ordered_clouds[i].begin(),out_radial_ordered_clouds[i].end(),
                        [](const PointXYZIRTColor &a,const PointXYZIRTColor &b){return a.radius <b.radius; });
    }
}

//点云剪裁
void sensor_lidar::DeleteGround::clip_above_pointCloud(double clip_height,//裁减高度（大于）
                                                                                                                        const pcl::PointCloud<pcl::PointXYZI>::Ptr in,//输入点云
                                                                                                                        const pcl::PointCloud<pcl::PointXYZI>::Ptr out//输出点云
                                                                                        )
{
    pcl::ExtractIndices<pcl::PointXYZI> cliper;
    cliper.setInputCloud(in);
    pcl::PointIndices indices;

#pragma omp for
    for(size_t i=0;i<in->points.size();i++)
    {
        if(in->points[i].z > clip_height)
        {
            indices.indices.push_back(i);
        }
    }

    cliper.setIndices(boost::make_shared<pcl::PointIndices>(indices));
    cliper.setNegative(true);//if true ,remove the indices
    cliper.filter(*out);
}

//裁减近一定范围的点
void sensor_lidar::DeleteGround::remove_close_pointsCloud(double min_distance,//裁减距离(小于)
                                                                                                                                double max_distance,//裁减远距离（大于）,小于0.0不执行
                                                                                                                                const pcl::PointCloud<pcl::PointXYZI>::Ptr in,//输入点云
                                                                                                                                const pcl::PointCloud<pcl::PointXYZI>::Ptr out//输出点云
                                                                                                )
{
    pcl::ExtractIndices<pcl::PointXYZI> cliper;
    cliper.setInputCloud(in);
    pcl::PointIndices indices;
#pragma omp for
    for(size_t i=0;i<in->points.size();i++)
    {
        double distence;
        distence=sqrt((in->points[i].x*in->points[i].x)+(in->points[i].y*in->points[i].y));

        if(max_distance<0.0)
        {
            if(distence<min_distance)
            {
                    indices.indices.push_back(i);
            }
        }
        else
        {
            if(distence < min_distance && distence > max_distance)
            {
                indices.indices.push_back(i);
            }
        }
    }
    cliper.setIndices(boost::make_shared<pcl::PointIndices>(indices));
    cliper.setNegative(true);//if ture ,remove the indices
    cliper.filter(*out);
}

//地面分割
void sensor_lidar::DeleteGround::classify_pointColud(std::vector<PointCloudXYZRTColor> &input_radial_ordered_clouds, 
                                                                                                                pcl::PointIndices &out_ground_indices,
                                                                                                                pcl::PointIndices &out_no_ground_indices
                                                                                                                )
{
    out_ground_indices.indices.clear();
    out_no_ground_indices.indices.clear();

#pragma omp for
    //遍历射线
    for(size_t i=0;i<input_radial_ordered_clouds.size();i++)
    {
        float prev_radius =0.f;
        float prev_height =-SENSOR_HEIGHT;
        bool prev_ground=false;
        bool current_ground=false;
        for(size_t j=0;j<input_radial_ordered_clouds[i].size();j++)
        {
            float points_distance =input_radial_ordered_clouds[i][j].radius-prev_radius;
            float height_threshold =tan(DEG2RAD(local_max_slope_))*points_distance;
            float current_height=input_radial_ordered_clouds[i][j].point.z;
            float general_height_threshold=tan(DEG2RAD(general_max_slope_))*input_radial_ordered_clouds[i][j].radius;
            //设置最小高度阈值
            if(points_distance > concentric_divider_distance_ && height_threshold<min_height_threshold_)
            {
                height_threshold=min_height_threshold_;
            }

            //检查前一个点高度
            if(current_height <=(prev_height+height_threshold) && current_height >=prev_height-height_threshold)
            {
                //判断是否地面
                if(!prev_ground)
                {
                    if(current_height <=(-SENSOR_HEIGHT+general_height_threshold) && current_height >=(-SENSOR_HEIGHT-general_height_threshold))
                    {
                        current_ground=true;
                    }
                    else
                    {
                        current_ground=false;
                    }
                }
                else
                {
                    current_ground=true;
                }
            }
            else
            {
                //判断是否地面
                if(points_distance > reclass_distance_threshold_  && current_height <=(-SENSOR_HEIGHT+height_threshold) 
                    && current_height >=(-SENSOR_HEIGHT-height_threshold))
                {
                    current_ground=true;
                }
                else
                {
                    current_ground=false;
                }
            }
            prev_radius=input_radial_ordered_clouds[i][j].radius;
            prev_height=input_radial_ordered_clouds[i][j].point.z;
        }
    }
}

//发布点云信息
void sensor_lidar::DeleteGround::publish_pointCloud(const ros::Publisher &input_publisher,
                                                                                                                const pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud_to_publish_ptr,
                                                                                                                const std_msgs::Header &input_header
                                                                                                                )
{
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*input_cloud_to_publish_ptr,cloud_msg);
    cloud_msg.header = input_header;
    input_publisher.publish(cloud_msg);
}