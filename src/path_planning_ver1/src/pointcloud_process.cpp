#include"pointcloud_process.h"

bool isNearEdge ( vector<double> point, double &refer_height, vector<vector<double>> edge_contour )
{
    
    for ( auto edge_point : edge_contour )
    {
        if ( abs( edge_point[ 0 ] - point[ 0 ] ) + abs( edge_point[ 1 ] - point[ 1 ] ) < 0.01 ) 
        {
            return true;
        }
    }

    return false;
}


double midHighestHeightOfShoe ( vector<vector<double>> point_cloud )
{
    priority_queue<double> pq;

    for ( auto point : point_cloud )
    {
        pq.push( point[ 2 ] );
    }

    for ( int i = 0; i < point_cloud.size() / 2; i++ )
    { 
         pq.pop();
    }
       
    return pq.top();
}

vector<vector<double>> smoothEdgePointCloud ( vector<vector<double>> point_cloud, vector<vector<double>> edge_contour )
{
    double refer_height = midHighestHeightOfShoe( point_cloud );
    vector<vector<double>> return_cloud = point_cloud;
    for ( auto &point : return_cloud )
    {
        if ( isNearEdge( point, refer_height, edge_contour ) )
        {
            point[ 2 ] = refer_height;
        }
    }

    return return_cloud;
}

vector<vector<double>> estimateNormals ( pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud )
{
    // Normal estimation
    pcl::PointCloud<pcl::Normal>::Ptr normals( new pcl::PointCloud<pcl::Normal> );
    pcl::NormalEstimation<pcl::PointXYZRGBA, pcl::Normal> ne;
    ne.setInputCloud( cloud );
    ne.setRadiusSearch( 0.1 );
    ne.compute( *normals );

    // Convert to Open3D's PointCloud format
    open3d::geometry::PointCloud o3dCloud;
    for ( const auto &point : cloud->points )
    {
        o3dCloud.points_.push_back( Eigen::Vector3d( point.x, point.y, point.z ) );
    }

    // Adjusting the normal directions
    open3d::geometry::PointCloud o3dNormals;
    for ( const auto &normal : normals->points )
    {
        o3dNormals.points_.push_back( Eigen::Vector3d( normal.normal_x, normal.normal_y, normal.normal_z ) );
    }

    o3dCloud.normals_ = o3dNormals.points_;
    o3dCloud.OrientNormalsTowardsCameraLocation(Eigen::Vector3d::Zero());
    o3dCloud.EstimateNormals( open3d::geometry::KDTreeSearchParamHybrid(0.05,20) );
    o3dCloud.OrientNormalsTowardsCameraLocation( Eigen::Vector3d( 0, 0, -1) );

    // Convert to (x, y, z, a, b, c) vector format
    vector<vector<double>> vectors;
    for ( size_t i = 0; i < o3dCloud.points_.size(); ++i )
    {
        const auto &point = o3dCloud.points_[ i ];
        const auto &normal = o3dCloud.normals_[ i ];
        vector<double> vector{ point.x(), point.y(), point.z(), normal.x(), normal.y(), normal.z() };
        vectors.push_back( vector );
    }

    vector<shared_ptr<const open3d::geometry::Geometry>> geometries;
    geometries.push_back( make_shared<const open3d::geometry::PointCloud>( o3dCloud ) );

    open3d::visualization::DrawGeometries(geometries, "result", 800, 800, 50, 50, true);

    return vectors;
}

void printPointCloudRange ( const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud )
{
    float min_x = std::numeric_limits<float>::max();
    float max_x = -std::numeric_limits<float>::max();
    float min_y = std::numeric_limits<float>::max();
    float max_y = -std::numeric_limits<float>::max();
    float min_z = std::numeric_limits<float>::max();
    float max_z = -std::numeric_limits<float>::max();

    for ( const pcl::PointXYZRGBA &point : cloud->points )
    {
        if ( point.x < min_x )
        {
            min_x = point.x;
        }

        if ( point.x > max_x ) 
        {
            max_x = point.x;
        }

        if ( point.y < min_y )
        {
            min_y = point.y;
        }
            
        if ( point.y > max_y )
        {
            max_y = point.y;
        }
            
        if ( point.z < min_z )
        {
            min_z = point.z;
        }
            
        if ( point.z > max_z )
        {
            max_z = point.z;
        }  
    }
}

vector<double> FindCorrectionCenter(vector<vector<double>> cloud)
{
    float center_x, center_y, low_z, max_x, min_x, max_y, min_y = 0;
    max_x = cloud[0][0];
    for (int i = 0; i < cloud.size(); i++)
    {
        if (cloud[i][0] > max_x)
        {
            max_x = cloud[i][0];
        }
    }

    min_x = cloud[0][0];
    for (int i = 0; i < cloud.size(); i++)
    {
        if (cloud[i][0] < min_x)
        {
            min_x = cloud[i][0];
        }
    }

    max_y = cloud[0][1];
    for (int i = 0; i < cloud.size(); i++)
    {
        if (cloud[i][1] > max_y)
        {
            max_y = cloud[i][1];
        }
    }

    min_y = cloud[0][1];
    for (int i = 0; i < cloud.size(); i++)
    {
        if (cloud[i][1] < min_y)
        {
            min_y = cloud[i][1];
        }
    }

    center_x = (max_x + min_x) / 2;
    center_y = (max_y + min_y) / 2;

    vector<double> center = {{center_x, center_y, low_z}};

    return center;
}

vector<vector<double>> OriginCorrectionPointCloud ( vector<vector<double>> cloud )
{
    float center_x, center_y, low_z, max_x, min_x, max_y, min_y = 0;
    max_x = cloud[ 0 ][ 0 ];
    for ( int i = 0; i < cloud.size(); i++ )
    {
        if ( cloud[ i ][ 0 ] > max_x )
        {
            max_x = cloud[ i ][ 0 ];
        }
    }

    min_x = cloud[ 0 ][ 0 ];
    for ( int i = 0; i < cloud.size(); i++ )
    {
        if ( cloud[ i ][ 0 ] < min_x )
        {
            min_x = cloud[ i ][ 0 ];
        }
    }

    max_y = cloud[ 0 ][ 1 ];
    for ( int i = 0; i < cloud.size(); i++ )
    {
        if ( cloud[ i ][ 1 ] > max_y )
        {
            max_y = cloud[ i ][ 1 ];
        }
    }

    min_y = cloud[ 0 ][ 1 ];
    for ( int i = 0; i < cloud.size(); i++ )
    {
        if ( cloud[ i ][ 1 ] < min_y )
        {
            min_y = cloud[ i ][ 1 ];
        }
    }

    low_z = cloud[ 0 ][ 2 ];
    for ( int i = 0; i < cloud.size(); i++ )
    {
        if ( cloud[ i ][ 2 ] < low_z )
        {
            low_z = cloud[ i ][ 2 ];
        }
    }

    center_x = ( max_x + min_x ) / 2;
    center_y = ( max_y + min_y ) / 2;

    for ( int i = 0; i < cloud.size(); i++ )
    {
        cloud[ i ][ 0 ] = cloud[ i ][ 0 ] - center_x;
        cloud[ i ][ 1 ] = cloud[ i ][ 1 ] - center_y;
        cloud[ i ][ 2 ] = cloud[ i ][ 2 ] - low_z;
    }

    return cloud;
}

vector<vector<double>> ResumePointCloudFromOrigin ( vector<vector<double>> cloud, vector<double> center )
{
    for ( int i = 0; i < cloud.size(); i++ )
    {
        cloud[ i ][ 0 ] = cloud[ i ][ 0 ] + center[0];
        cloud[ i ][ 1 ] = cloud[ i ][ 1 ] + center[1];
        cloud[ i ][ 2 ] = cloud[ i ][ 2 ] + center[2];
    }

    return cloud;
}

vector<vector<double>> removeBouncePoints ( vector<vector<double>> cloud )
{

    float temp_z = cloud[ 0 ][ 2 ];
    for ( int i = 1; i < cloud.size(); i++ )
    {
        if ( abs( cloud[ i ][ 2 ] - temp_z ) > 0.1 )
        {
            if ( cloud[ i ][ 2 ] < temp_z )
            {
                cloud[ i ][ 2 ] = temp_z;
            }

            temp_z = cloud[ i ][ 2 ];
        }
    }

    return cloud;
}