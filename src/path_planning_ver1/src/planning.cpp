#include"planning.h"

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

bool SortYaxisBigToSmall ( vector<double> a, vector<double> b )
{
    return a[ 1 ] > b[ 1 ];
}

bool SortYaxisSmallToBig ( vector<double> a, vector<double> b )
{
    return a[ 1 ] < b[ 1 ];
}

double polar_angle ( vector<double> center, vector<double> p )
{
    return atan2( p[ 1 ] - center[ 1 ], p[ 0 ] - center[ 0 ] );
}

bool customCompare ( const vector<double> &a, const vector<double> &b )
{
    return a[ 7 ] > b[ 7 ];
}

/*
vector<vector<double>> BorderReinforcement ( vector<vector<double>> cloud )
{
    open3d::geometry::PointCloud pcd;
    vector<vector<double>> square_path;
    vector<vector<double>> contour = edge_contour;

    double center_x = ( contour[ contour.size()-1 ][ 0 ] + contour[ contour.size() - 2 ][ 0 ]) / 2;
    double center_y = ( contour[ contour.size()-1] [ 1 ] + contour[ contour.size() - 2 ][ 1 ]) / 2;
    vector<double> center{ center_x, center_y };

    sort( contour.begin(), contour.end(), [&](const vector<double> &a, const vector<double> &b )
    {
        return polar_angle ( center, a ) < polar_angle( center, b ); 
    });

    for( auto& point: contour ) 
    {
        point[ 2 ]+=0.03;
        pcd.points_.push_back( { point[ 0 ], point[ 1 ], point[ 2 ] } );
    }

    open3d::visualization::DrawGeometries( { make_shared<open3d::geometry::PointCloud>( pcd ) } );

    return contour;
}
*/


vector<vector<double>> PathCloudFilter ( vector<vector<double>> input_cloud, int rounds, double CLOUD_SEARCHING_RANGE, double PLASMA_DIA )
{
    vector<vector<double>> ok_cloud_1;
    vector<vector<double>> cloud=input_cloud;
    vector<vector<double>> edge_contour;

    for(auto& point:cloud)
    {
        double tmp=point[0];
        point[0]=point[1];
        point[1]=tmp;

    }

    float max_x = cloud[ 0 ][ 0 ];
    // find the pointcloud range
    for ( int i = 0; i < cloud.size(); i++ )
    {
        if ( cloud[ i ][ 0 ] > max_x )
        {
            max_x = cloud[ i ][ 0 ];
        }
    }

    float min_x = cloud[ 0 ][ 0 ];

    for ( int i = 0; i < cloud.size(); i++ ) 
    {
        if ( cloud[ i ][ 0 ] < min_x ) 
        {
            min_x = cloud[ i ][ 0 ];
        }
    }

    float max_y = cloud[ 0 ][ 1 ];

    for ( int i = 0; i < cloud.size(); i++ )
    {
        if ( cloud[ i ][ 1 ] > max_y )
        {
            max_y = cloud[ i ][ 1 ];
        }
    }

    float min_y = cloud[ 0 ][ 1 ];

    for ( int i = 0; i < cloud.size(); i++ )
    {
        if ( cloud[ i ][ 1 ] < min_y )
        {
            min_y = cloud[ i ][ 1 ];
        }
    }

    float shift_distance = ( max_x - min_x ) / rounds;

    vector<double> startPoint = { 0, 0, -0.3, 0, 0, 0 };

    for ( int i = 0; i <= rounds; i++ )
    {
        float x = max_x - ( shift_distance * i );
        float up_x = x + CLOUD_SEARCHING_RANGE;
        float low_x = x - CLOUD_SEARCHING_RANGE;

        vector<vector<double>> tmp_cloud;

        for ( int j = 0; j < cloud.size(); j++ )
        {
            if ( cloud[ j ][ 0 ] > low_x && cloud[ j ][ 0 ] < up_x )
            {
                tmp_cloud.push_back( cloud[ j ] );
            }
        }

        if ( i % 2 == 0 )
        {
            std::sort( tmp_cloud.begin(), tmp_cloud.end(), SortYaxisBigToSmall );
            vector<double> ap_max_y = { x, max_y + PLASMA_DIA + 0.02, tmp_cloud[ 0 ][ 2 ] + 0.02, 0, 0, 0 };
            vector<double> ap_min_y = { x, min_y - PLASMA_DIA - 0.02, tmp_cloud[ tmp_cloud.size() - 1 ][ 2 ] + 0.02, 0, 0, 0 };
            edge_contour.push_back( tmp_cloud.front() );
            edge_contour.push_back( tmp_cloud.back() );
            ok_cloud_1.push_back( ap_max_y );

            for ( auto c : tmp_cloud )
            {
                ok_cloud_1.push_back( c );
            }
                
            ok_cloud_1.push_back( ap_min_y );

        }
        else
        {
            std::sort( tmp_cloud.begin(), tmp_cloud.end(), SortYaxisSmallToBig );
            vector<double> ap_max_y = { x, max_y + PLASMA_DIA + 0.02, tmp_cloud[ tmp_cloud.size() - 1 ][ 2 ]+ 0.02, 0, 0, 0 };
            vector<double> ap_min_y = { x, min_y - PLASMA_DIA - 0.02, tmp_cloud[ 0 ][ 2 ]+ 0.02, 0, 0, 0 };
            edge_contour.push_back( tmp_cloud.back() );
            edge_contour.push_back( tmp_cloud.front() );
            ok_cloud_1.push_back( ap_min_y );

            for ( auto c : tmp_cloud )
            {
                ok_cloud_1.push_back( c );
            }
                
            ok_cloud_1.push_back( ap_max_y );
        }
    }

    return ok_cloud_1;
}

vector<vector<double>> PathPlanning ( vector<vector<double>> cloud, int rounds, double CLOUD_SEARCHING_RANGE, double PLASMA_DIA )
{
    vector<vector<double>> filtered_cloud = PathCloudFilter( cloud, rounds, CLOUD_SEARCHING_RANGE, PLASMA_DIA );

    // Convert input cloud to Open3D format
    open3d::geometry::PointCloud open3d_cloud;
    for ( const auto &point : filtered_cloud )
    {
        open3d_cloud.points_.push_back( Eigen::Vector3d ( point[ 0 ], point[ 1 ], point[ 2 ] ) );
    }

    // Filter the point cloud using Open3D functions
    open3d::geometry::PointCloud filtered_open3d_cloud = open3d_cloud; // Perform your filtering operation here
    
    // Visualize the filtered point cloud
    open3d::visualization::Visualizer visualizer;
    visualizer.CreateVisualizerWindow( "Open3D Point Cloud", 800, 800 );

    std::shared_ptr<const open3d::geometry::Geometry> filtered_geometry_ptr = std::make_shared<const open3d::geometry::PointCloud>( filtered_open3d_cloud );
    visualizer.AddGeometry( filtered_geometry_ptr );
    visualizer.Run();
    visualizer.DestroyVisualizerWindow();

    return filtered_cloud;
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
    o3dCloud.EstimateNormals( open3d::geometry::KDTreeSearchParamHybrid(0.01,10) );
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