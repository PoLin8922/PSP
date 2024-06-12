#include"planning.h"

bool debug = true;

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
            vector<double> ap_max_y = { x, max_y + PLASMA_DIA + 0.02, tmp_cloud[ 0 ][ 2 ] + 0.01, 0, 0, 0 };
            vector<double> ap_min_y = { x, min_y - PLASMA_DIA - 0.02, tmp_cloud[ tmp_cloud.size() - 1 ][ 2 ] + 0.01, 0, 0, 0 };
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
            vector<double> ap_max_y = { x, max_y + PLASMA_DIA + 0.02, tmp_cloud[ tmp_cloud.size() - 1 ][ 2 ] + 0.01, 0, 0, 0 };
            vector<double> ap_min_y = { x, min_y - PLASMA_DIA - 0.02, tmp_cloud[ 0 ][ 2 ] + 0.01, 0, 0, 0 };
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
    if(debug){
        open3d::visualization::Visualizer visualizer;
        visualizer.CreateVisualizerWindow( "Open3D Point Cloud", 800, 800 );

        std::shared_ptr<const open3d::geometry::Geometry> filtered_geometry_ptr = std::make_shared<const open3d::geometry::PointCloud>( filtered_open3d_cloud );
        visualizer.AddGeometry( filtered_geometry_ptr );
        visualizer.Run();
        visualizer.DestroyVisualizerWindow();
    }
    
    return filtered_cloud;
}