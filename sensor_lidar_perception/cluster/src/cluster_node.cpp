#include"euclidean_cluster.h"

int main(int argc,char **argv)
{
    ros::init(argc,argv,"cluster_node");
    ros::NodeHandle nh;
    sensor_lidar::Euclidean euclidean(nh);

    return 0;
}