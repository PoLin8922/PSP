#ifndef POINTCLOUD_PROCESS_H
#define POINTCLOUD_PROCESS_H

#include"common_include.h"

double midHighestHeightOfShoe ( vector<vector<double>> point_cloud );

vector<vector<double>> smoothEdgePointCloud ( vector<vector<double>> point_cloud, vector<vector<double>> edge_contour );

vector<vector<double>> estimateNormals ( pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud );

void printPointCloudRange ( const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud );

vector<vector<double>> OriginCorrectionPointCloud ( vector<vector<double>> cloud );

vector<vector<double>> removeBouncePoints ( vector<vector<double>> cloud );

#endif