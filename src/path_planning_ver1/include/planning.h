#ifndef PLANNING_H
#define PLANNING_H

#include"common_include.h"
// #include"global_variable.h"

bool isNearEdge ( vector<double> point, double &refer_height, vector<vector<double>> edge_contour );

bool SortYaxisBigToSmall ( vector<double> a, vector<double> b );

bool SortYaxisSmallToBig ( vector<double> a, vector<double> b );

double polar_angle ( vector<double> center, vector<double> p );

bool customCompare ( const vector<double> &a, const vector<double> &b );

vector<vector<double>> BorderReinforcement ( vector<vector<double>> cloud );

vector<vector<double>> PathCloudFilter ( vector<vector<double>> input_cloud, int rounds, double CLOUD_SEARCHING_RANGE, double PLASMA_DIA );

vector<vector<double>> PathPlanning ( vector<vector<double>> cloud, int rounds, double CLOUD_SEARCHING_RANGE, double PLASMA_DIA );

double midHighestHeightOfShoe ( vector<vector<double>> point_cloud );

vector<vector<double>> smoothEdgePointCloud ( vector<vector<double>> point_cloud, vector<vector<double>> edge_contour );

vector<vector<double>> estimateNormals ( pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud );

void printPointCloudRange ( const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud );

vector<vector<double>> OriginCorrectionPointCloud ( vector<vector<double>> cloud );

vector<vector<double>> removeBouncePoints ( vector<vector<double>> cloud );


#endif