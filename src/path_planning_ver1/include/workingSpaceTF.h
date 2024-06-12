#ifndef WORKINGSPACE_TF_H
#define WORKINGSPACE_TF_H

#include"common_include.h"

struct Waypoint
{
    double x;
    double y;
    double z;
    double W;
    double P;
    double R;
    double V;
    std::string C;
};

void initializeWaypoints(Waypoint *waypoint,double V);

void zBiasWithNormal(std::vector<std::vector<double>> &points, double TF_Z_BIAS);

void vector2Angle(std::vector<std::vector<double>>& points);

void workingSpaceTF(const std::vector<std::vector<double>>& points, 
                    std::vector<Waypoint>& waypoints, 
                    double theta,double TF_Z_BIAS,double TF_X_BIAS ,double TF_Y_BIAS ,double vel
                   );

int writeLsFile(const std::string& absfile, 
                const std::string& file, 
                const std::vector<Waypoint>& waypoints
               );

#endif