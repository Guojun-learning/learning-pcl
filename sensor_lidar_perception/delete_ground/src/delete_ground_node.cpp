#include"delete_ground.h"

int main(int argc,char **argv)
{
    ros::init(argc,argv,"delete_ground_node");
    ros::NodeHandle nh;
    sensor_lidar::DeleteGround  delete_ground(nh);
    
    return 0;
}