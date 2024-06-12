#include <iostream>
#include "workingSpaceTF.h"
#include "json.h"

using namespace std;
using json = nlohmann::json;

double DOWN_SAMPLE_SIZE = 0.005;
double TF_Z_BIAS = 0;
double TF_X_BIAS = 0;
double TF_Y_BIAS = 0;
double velocity = 300;
double PLASMA_DIA = 0.05;
double CLOUD_SEARCHING_RANGE = 0.002;
int rounds=5;

int readParameters ()
{
    std::string filePath = "tuning/path.json";
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
    TF_X_BIAS = parameters[ "TF_X_BIAS" ];
    TF_Y_BIAS = parameters[ "TF_Y_BIAS" ];
    velocity = parameters[ "velocity" ];
    rounds = parameters[ "rounds" ];
                           
    return 1;
}

int main()
{
    readParameters ();

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);

    std::string pointCloudPath = "files/boundary_cloud.pcd";

    if (pcl::io::loadPCDFile<pcl::PointXYZRGBA>(pointCloudPath, *cloud) == -1)
    {
        cerr << "[calibration] load pcd failed " << pointCloudPath << endl;
        return -1;
    }

    // transfer to vector
    vector<vector<double>> path_point_cloud;

    for (size_t i = 0; i < cloud->points.size(); ++i)
    {
        std::vector<double> point;
        point.push_back(cloud->points[i].x);
        point.push_back(cloud->points[i].y);
        point.push_back(cloud->points[i].z);
        point.push_back(0);
        point.push_back(0);
        point.push_back(0);

        path_point_cloud.push_back(point);
    }

    // print the path_point_cloud
    cout << "path_point_cloud:" << endl;
    for (size_t i = 0; i < path_point_cloud.size(); ++i)
    {
        cout << "Point " << i << ": ";
        for (size_t j = 0; j < path_point_cloud[i].size(); ++j)
        {
            cout << path_point_cloud[i][j] << " ";
        }
        cout << endl;
    }

    for ( auto &point : path_point_cloud )
    {
        point[ 0 ] = point[ 0 ] * 1000;
        point[ 1 ] = point[ 1 ] * 1000;
        point[ 2 ] = ( point[ 2 ] )* 1000;
    }

    std::vector<Waypoint> waypoints;
    double theta = 0;
    vector2Angle( path_point_cloud, TF_Z_BIAS );
    workingSpaceTF( path_point_cloud, waypoints, theta, TF_Z_BIAS, TF_X_BIAS, TF_Y_BIAS, velocity );

    std::string absfile_path = "files/H002.LS";

    const std::string file_path = "H002.LS";

    if ( writeLsFile( absfile_path,file_path ,waypoints ) )
    {
        printf( "Write LS File error !!!\n" );
    }
    else
    {
        printf( "Write LS File Sucess!!!\n" );
    }

    return 0;
}
