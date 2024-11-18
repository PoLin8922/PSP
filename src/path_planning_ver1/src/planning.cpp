#include "planning.h"

bool visual = true;

bool SortXaxisBigToSmall(vector<double> a, vector<double> b)
{
    return a[0] > b[0];
}

bool SortXaxisSmallToBig(vector<double> a, vector<double> b)
{
    return a[0] < b[0];
}

bool SortYaxisBigToSmall(vector<double> a, vector<double> b)
{
    return a[1] > b[1];
}

bool SortYaxisSmallToBig(vector<double> a, vector<double> b)
{
    return a[1] < b[1];
}

double polar_angle(vector<double> center, vector<double> p)
{
    return atan2(p[1] - center[1], p[0] - center[0]);
}

bool customCompare(const vector<double> &a, const vector<double> &b)
{
    return a[7] > b[7];
}

// prevent the reciprocating path problem
void average_cloud( vector<vector<double>>& input_cloud, int direction)
{
    if(direction == 0)  // y
    {
        float sum_y = 0;
        
        for( int i=0; i<input_cloud.size(); i++)
        {
            sum_y += input_cloud[i][1];
            
        }

        float x = sum_y / input_cloud.size();
        
        
        for(int i=0; i<input_cloud.size(); i++)
        {
            input_cloud[i][1] = x;
            
        }
    }
    else    // x
    {
        float sum_x = 0;
        float sum_z = 0;
        for( int i=0; i<input_cloud.size(); i++)
        {
            sum_x += input_cloud[i][0];
            sum_z += input_cloud[i][2];
        }

        float y = sum_x / input_cloud.size();
        float z = sum_z / input_cloud.size();
        
        for(int i=0; i<input_cloud.size(); i++)
        {
            input_cloud[i][0] = y;
            input_cloud[i][2] = z;
        }
    }
}

vector<vector<double>> PathCloudFilter(vector<vector<double>> input_cloud, int rounds, double CLOUD_SEARCHING_RANGE, double PLASMA_DIA)
{
    vector<vector<double>> ok_cloud_1;
    vector<vector<double>> cloud = input_cloud;
    vector<vector<double>> edge_contour;

    float max_y = cloud[0][1];
    // find the pointcloud range
    for (int i = 0; i < cloud.size(); i++)
    {
        if (cloud[i][1] > max_y)
        {
            max_y = cloud[i][1];
        }
    }

    float min_y = cloud[0][1];

    for (int i = 0; i < cloud.size(); i++)
    {
        if (cloud[i][1] < min_y)
        {
            min_y = cloud[i][1];
        }
    }

    float max_x = cloud[0][0];

    for (int i = 0; i < cloud.size(); i++)
    {
        if (cloud[i][0] > max_x)
        {
            max_x = cloud[i][0];
        }
    }

    float min_x = cloud[0][0];

    for (int i = 0; i < cloud.size(); i++)
    {
        if (cloud[i][0] < min_x)
        {
            min_x = cloud[i][0];
        }
    }

    float shift_distance = (max_y - min_y) / rounds;

    for (int i = 0; i <= rounds; i++)
    {
        float y = max_y - (shift_distance * i);
        float up_y = y + CLOUD_SEARCHING_RANGE;
        float low_y = y - CLOUD_SEARCHING_RANGE;

        vector<vector<double>> tmp_cloud;

        if (i == 0 || i == rounds) // If it's first or last round, we process it later
            continue;

        for (int j = 0; j < cloud.size(); j++)
        {
            if (cloud[j][1] > low_y && cloud[j][1] < up_y)
            {
                // average
                tmp_cloud.push_back(cloud[j]);
                average_cloud(tmp_cloud, 0);
            }
        }

        if (i % 2 == 0)
        {
            std::sort(tmp_cloud.begin(), tmp_cloud.end(), SortXaxisBigToSmall);
            vector<double> ap_max_x = {max_x + PLASMA_DIA + 0.02, y, tmp_cloud[0][2] +0.01, 0, 0, 0};
            vector<double> ap_min_x = {min_x - PLASMA_DIA - 0.02, y, tmp_cloud[tmp_cloud.size() - 1][2] +0.01, 0, 0, 0};
            edge_contour.push_back(tmp_cloud.back());
            edge_contour.push_back(tmp_cloud.front());

            tmp_cloud.insert(tmp_cloud.begin(), ap_max_x);
            tmp_cloud.insert(tmp_cloud.end(), ap_min_x);

            for (auto c : tmp_cloud)
            {
                ok_cloud_1.push_back(c);
            }
        }
        else
        {
            std::sort(tmp_cloud.begin(), tmp_cloud.end(), SortXaxisSmallToBig);
            vector<double> ap_max_x = {max_x + PLASMA_DIA + 0.02, y, tmp_cloud[tmp_cloud.size() - 1][2] +0.01, 0, 0, 0};
            vector<double> ap_min_x = {min_x - PLASMA_DIA - 0.02, y, tmp_cloud[0][2] +0.01, 0, 0, 0};
            edge_contour.push_back(tmp_cloud.back());
            edge_contour.push_back(tmp_cloud.front());

            tmp_cloud.insert(tmp_cloud.begin(), ap_min_x);
            tmp_cloud.insert(tmp_cloud.end(), ap_max_x);

            for (auto c : tmp_cloud)
            {
                ok_cloud_1.push_back(c);
            }
        }
        if (i == 1)
        {
            for (auto &c : tmp_cloud)
            {
                c[1] = c[1] + shift_distance;
            }
            std::reverse(tmp_cloud.begin(), tmp_cloud.end());
            ok_cloud_1.insert(ok_cloud_1.begin(), tmp_cloud.begin(), tmp_cloud.end());
        }
        if (i == (rounds - 1))
        {
            for (auto &c : tmp_cloud)
            {
                c[1] = c[1] - shift_distance;
            }
            std::reverse(tmp_cloud.begin(), tmp_cloud.end());
            ok_cloud_1.insert(ok_cloud_1.end(), tmp_cloud.begin(), tmp_cloud.end());
        }
    }

    return ok_cloud_1;
}

vector<vector<double>> PathCloudFilter_short(vector<vector<double>> input_cloud, int rounds, double CLOUD_SEARCHING_RANGE, double PLASMA_DIA)
{
    vector<vector<double>> ok_cloud_1;
    vector<vector<double>> cloud = input_cloud;
    vector<vector<double>> edge_contour;

    float max_y = cloud[0][1];
    // find the pointcloud range
    for (int i = 0; i < cloud.size(); i++)
    {
        if (cloud[i][1] > max_y)
        {
            max_y = cloud[i][1];
        }
    }

    float min_y = cloud[0][1];

    for (int i = 0; i < cloud.size(); i++)
    {
        if (cloud[i][1] < min_y)
        {
            min_y = cloud[i][1];
        }
    }

    float max_x = cloud[0][0];

    for (int i = 0; i < cloud.size(); i++)
    {
        if (cloud[i][0] > max_x)
        {
            max_x = cloud[i][0];
        }
    }

    float min_x = cloud[0][0];

    for (int i = 0; i < cloud.size(); i++)
    {
        if (cloud[i][0] < min_x)
        {
            min_x = cloud[i][0];
        }
    }

    float shift_distance = (max_x - min_x) / rounds;

    for (int i = 0; i <= rounds; i++)
    {
        float x = max_x - (shift_distance * i);
        float up_x = x + CLOUD_SEARCHING_RANGE;
        float low_x = x - CLOUD_SEARCHING_RANGE;

        vector<vector<double>> tmp_cloud;

        if (i == 0 || i == rounds) // If it's first or last round, we process it later
            continue;

        for (int j = 0; j<cloud.size(); j++)
        {
            if (cloud[j][0] > low_x && cloud[j][0] < up_x)
            {
                tmp_cloud.push_back(cloud[j]);
            }
        }
        average_cloud(tmp_cloud, 1);

        if (i % 2 == 0)
        {
            std::sort(tmp_cloud.begin(), tmp_cloud.end(), SortYaxisBigToSmall);
            vector<double> ap_max_y = {x, max_y + PLASMA_DIA + 0.02, tmp_cloud[0][2] +0.01, 0, 0, 0};
            vector<double> ap_min_y = {x, min_y - PLASMA_DIA - 0.02, tmp_cloud[tmp_cloud.size() - 1][2] +0.01, 0, 0, 0};
            edge_contour.push_back(tmp_cloud.back());
            edge_contour.push_back(tmp_cloud.front());

            tmp_cloud.insert(tmp_cloud.begin(), ap_max_y);
            tmp_cloud.insert(tmp_cloud.end(), ap_min_y);

            for (auto c : tmp_cloud)
            {
                ok_cloud_1.push_back(c);
            }
        }
        else
        {
            std::sort(tmp_cloud.begin(), tmp_cloud.end(), SortYaxisSmallToBig);
            vector<double> ap_max_y = {x, max_y + PLASMA_DIA + 0.02, tmp_cloud[tmp_cloud.size() - 1][2] +0.01, 0, 0, 0};
            vector<double> ap_min_y = {x, min_y - PLASMA_DIA - 0.02, tmp_cloud[0][2] +0.01, 0, 0, 0};
            edge_contour.push_back(tmp_cloud.back());
            edge_contour.push_back(tmp_cloud.front());

            tmp_cloud.insert(tmp_cloud.begin(), ap_min_y);
            tmp_cloud.insert(tmp_cloud.end(), ap_max_y);

            for (auto c : tmp_cloud)
            {
                ok_cloud_1.push_back(c);
            }
        }
        if (i == 1)
        {
            float push_down = 0.015;
            for (auto &c : tmp_cloud)
            {
                c[0] = c[0] + shift_distance;
                c[2] = c[2] + push_down;
            }
            std::reverse(tmp_cloud.begin(), tmp_cloud.end());
            ok_cloud_1.insert(ok_cloud_1.begin(), tmp_cloud.begin(), tmp_cloud.end());
        }
        if (i == (rounds - 1))
        {
            for (auto &c : tmp_cloud)
            {
                c[0] = c[0] - shift_distance;
            }
            std::reverse(tmp_cloud.begin(), tmp_cloud.end());
            ok_cloud_1.insert(ok_cloud_1.end(), tmp_cloud.begin(), tmp_cloud.end());
        }

    }

    // improve shoe heel
    vector<vector<double>> tmp_cloud;

    for (int j = 0; j<cloud.size(); j++)
    {
        if (cloud[j][0] > min_x - 0.005 && cloud[j][0] < min_x + 0.005)
        {
            tmp_cloud.push_back(cloud[j]);
        }
    }
    average_cloud(tmp_cloud, 1);

    vector<double> ap_max_y = {min_x, max_y + PLASMA_DIA + 0.02, tmp_cloud[tmp_cloud.size() - 1][2] +0.01, 0, 0, 0};
    vector<double> ap_min_y = {min_x, min_y - PLASMA_DIA - 0.02, tmp_cloud[0][2] +0.01, 0, 0, 0};

    if(ok_cloud_1[ok_cloud_1.size()-1][1] - ok_cloud_1[ok_cloud_1.size()-2][1] > 0)
    {
        std::sort(tmp_cloud.begin(), tmp_cloud.end(), SortYaxisBigToSmall); 
        tmp_cloud.insert(tmp_cloud.begin(), ap_max_y);
        tmp_cloud.insert(tmp_cloud.end(), ap_min_y);
    }
    else
    {
        std::sort(tmp_cloud.begin(), tmp_cloud.end(), SortYaxisSmallToBig);   
        tmp_cloud.insert(tmp_cloud.begin(), ap_min_y);
        tmp_cloud.insert(tmp_cloud.end(), ap_max_y);  
    }

    for (auto &c : tmp_cloud)
    {
        c[0] = c[0] - shift_distance/2;
        // c[2] = c[2] + push_down;
    }
    ok_cloud_1.insert(ok_cloud_1.end(), tmp_cloud.begin(), tmp_cloud.end());

    return ok_cloud_1;
}

vector<vector<double>> PathPlanning(vector<vector<double>> cloud, int rounds, double CLOUD_SEARCHING_RANGE, double PLASMA_DIA)
{
    vector<vector<double>> filtered_cloud = PathCloudFilter(cloud, rounds, CLOUD_SEARCHING_RANGE, PLASMA_DIA);

    // Convert input cloud to Open3D format
    open3d::geometry::PointCloud open3d_cloud;
    for (const auto &point : filtered_cloud)
    {
        open3d_cloud.points_.push_back(Eigen::Vector3d(point[0], point[1], point[2]));
    }

    // Filter the point cloud using Open3D functions
    open3d::geometry::PointCloud filtered_open3d_cloud = open3d_cloud; // Perform your filtering operation here

    // Visualize the filtered point cloud
    if(visual){
        open3d::visualization::Visualizer visualizer;
        visualizer.CreateVisualizerWindow("Open3D Point Cloud", 800, 800);

        std::shared_ptr<const open3d::geometry::Geometry> filtered_geometry_ptr = std::make_shared<const open3d::geometry::PointCloud>(filtered_open3d_cloud);
        visualizer.AddGeometry(filtered_geometry_ptr);
        visualizer.Run();
        visualizer.DestroyVisualizerWindow();
    }

    return filtered_cloud;
}

vector<vector<double>> PathPlanning_short( vector<vector<double>> cloud, int rounds, double CLOUD_SEARCHING_RANGE, double PLASMA_DIA )
{
    vector<vector<double>> filtered_cloud = PathCloudFilter_short(cloud, rounds, CLOUD_SEARCHING_RANGE, PLASMA_DIA);

    // Convert input cloud to Open3D format
    open3d::geometry::PointCloud open3d_cloud;
    for (const auto &point : filtered_cloud)
    {
        open3d_cloud.points_.push_back(Eigen::Vector3d(point[0], point[1], point[2]));
    }

    // Filter the point cloud using Open3D functions
    open3d::geometry::PointCloud filtered_open3d_cloud = open3d_cloud; // Perform your filtering operation here

    // Visualize the filtered point cloud
    if(visual){
        open3d::visualization::Visualizer visualizer;
        visualizer.CreateVisualizerWindow("Open3D Point Cloud", 800, 800);

        std::shared_ptr<const open3d::geometry::Geometry> filtered_geometry_ptr = std::make_shared<const open3d::geometry::PointCloud>(filtered_open3d_cloud);
        visualizer.AddGeometry(filtered_geometry_ptr);
        visualizer.Run();
        visualizer.DestroyVisualizerWindow();
    }

    return filtered_cloud;
}