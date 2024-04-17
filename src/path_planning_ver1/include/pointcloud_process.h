#ifndef POINTCLOUD_PROCESS_H
#define POINTCLOUD_PROCESS_H

#include"common_include.h"

double midHighestHeightOfShoe ( vector<vector<double>> point_cloud );

bool isNearEdge ( vector<double> point, double &refer_height, vector<vector<double>> edge_contour );

vector<vector<double>> smoothEdgePointCloud ( vector<vector<double>> point_cloud, vector<vector<double>> edge_contour );

vector<vector<double>> estimateNormals ( pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud );

void printPointCloudRange ( const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud );

vector<double> FindCorrectionCenter ( vector<vector<double>> cloud );

vector<vector<double>> OriginCorrectionPointCloud ( vector<vector<double>> cloud );

vector<vector<double>> ResumePointCloudFromOrigin ( vector<vector<double>> cloud, vector<double> center );

vector<vector<double>> removeBouncePoints ( vector<vector<double>> cloud );

#endif