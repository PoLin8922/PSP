#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include "path_planning_ver1/path_planning_ver1.h"

#include"planning.h"
#include"pointcloud_process.h"
#include"workingSpaceTF.h"
#include "json.h"

using namespace std;
using json = nlohmann::json;

double DOWN_SAMPLE_SIZE = 0.005;
double TF_Z_BIAS = 0;
double height=0.03;
double velocity = 300;
double PLASMA_DIA = 0.05;
double CLOUD_SEARCHING_RANGE = 0.002;
int rounds=5;

int readParameters ()
{
    const char *homeDir = getenv ( "HOME" );
    if ( homeDir == nullptr )
    {
        std::cerr << "Failed to get the home directory." << std::endl;
    }

    std::string filePath = std::string ( homeDir ) + "/home/honglang/PSP/tuning/path.json";
    std::ifstream file ( filePath );
    if ( !file.is_open() )
    {
        std::cerr << "Error opening parameters.json" << std::endl;
        return 0;
    }

    json parameters;
    file >> parameters;
    file.close();

    PLASMA_DIA = parameters[ "PLASMA_DIA" ];
    TF_Z_BIAS = parameters[ "TF_Z_BIAS" ];
    velocity = parameters[ "velocity" ];
    rounds = parameters[ "rounds" ];
    height = parameters[ "height" ];
    
    return 1;
}

void get_path ()
{
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud( new pcl::PointCloud<pcl::PointXYZRGBA> );

    const char* homeDir = getenv( "HOME" );
    if ( homeDir == nullptr )
    {
        std::cerr << "Failed to get the home directory." << std::endl;
    }

    std::string pointCloudPath = std::string( homeDir ) + "/PSP/files/point_cloud.pcd";

    if ( pcl::io::loadPCDFile<pcl::PointXYZRGBA>( pointCloudPath, *cloud ) == -1 )
    {
        //do nothing
    }

    // Downsample
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr downsampledCloud ( new pcl::PointCloud<pcl::PointXYZRGBA> );
    pcl::VoxelGrid<pcl::PointXYZRGBA> voxelGrid;
    voxelGrid.setInputCloud( cloud );
    voxelGrid.setLeafSize( DOWN_SAMPLE_SIZE, DOWN_SAMPLE_SIZE, DOWN_SAMPLE_SIZE );
    voxelGrid.filter( *downsampledCloud );

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr smooth( new pcl::PointCloud<pcl::PointXYZRGBA> );

    vector<vector<double>> cloud_w_vector = estimateNormals( downsampledCloud );
    vector<vector<double>> correction_cloud = OriginCorrectionPointCloud( cloud_w_vector );

    // jylong edit
    std::vector<std::vector<double>> point_cloud = PathPlanning( correction_cloud, rounds, CLOUD_SEARCHING_RANGE, PLASMA_DIA );

    for ( auto &point : point_cloud )
    {
        point[ 0 ] = point[ 0 ] * 1000;
        point[ 1 ] = point[ 1 ] * 1000;
        point[ 2 ] = ( point[ 2 ] + height )* 1000;
    }

    std::vector<Waypoint> waypoints;
    double theta = 0;
    vector2Angle( point_cloud );
    workingSpaceTF( point_cloud, waypoints, theta, TF_Z_BIAS, velocity );

    if ( homeDir == nullptr )
    {
        std::cerr << "Failed to get the home directory." << std::endl;
    }

    std::string absfile_path = std::string( homeDir ) + "/PSP/files/H001.LS";

    const std::string file_path = "H001.LS";

    if ( writeLsFile( absfile_path,file_path ,waypoints ) )
    {
        printf( "Write LS error !!!\n" );
    }
    else
    {
        printf( "Sucess!!!\n" );
    }
}

bool server_callback ( path_planning_ver1::path_planning_ver1::Request &req, 
                       path_planning_ver1::path_planning_ver1::Response &res )
{
    if ( req.REQU_PP == true )
    {
        get_path();
        res.RESP_PP = true;
    }
    else
    {
        res.RESP_PP = false;
    }

    return true;
}

int main ( int argc, char **argv )
{
    readParameters();
    get_path();

    // ros::init( argc, argv, "path_planning_ver1" );
    // ros::NodeHandle nh;
    // ros::Rate loop_rate( 30 );
    // ros::ServiceServer service = nh.advertiseService( "path_planning_ver1", server_callback );

    // while( ros::ok() ) 
    // {
    //     loop_rate.sleep();
    //     ros::spinOnce();
    // }

    return 0;
}