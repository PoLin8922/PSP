#ifndef PLANNING_H
#define PLANNING_H

#include"common_include.h"


bool SortYaxisBigToSmall ( vector<double> a, vector<double> b );

bool SortYaxisSmallToBig ( vector<double> a, vector<double> b );

double polar_angle ( vector<double> center, vector<double> p );

bool customCompare ( const vector<double> &a, const vector<double> &b );

vector<vector<double>> BorderReinforcement ( vector<vector<double>> cloud );

void average_cloud( vector<vector<double>>& input_cloud, int direction);

vector<vector<double>> PathCloudFilter ( vector<vector<double>> input_cloud, int rounds, double CLOUD_SEARCHING_RANGE, double PLASMA_DIA );
vector<vector<double>> PathCloudFilter_short(vector<vector<double>> input_cloud, int rounds, double CLOUD_SEARCHING_RANGE, double PLASMA_DIA);

vector<vector<double>> PathPlanning ( vector<vector<double>> cloud, int rounds, double CLOUD_SEARCHING_RANGE, double PLASMA_DIA );
vector<vector<double>> PathPlanning_short( vector<vector<double>> cloud, int rounds, double CLOUD_SEARCHING_RANGE, double PLASMA_DIA );

//void PathCloudFilter ( vector<vector<double>>& input_cloud, int rounds, double CLOUD_SEARCHING_RANGE, double PLASMA_DIA );

//void PathPlanning ( vector<vector<double>>& cloud, int rounds, double CLOUD_SEARCHING_RANGE, double PLASMA_DIA );

#endif