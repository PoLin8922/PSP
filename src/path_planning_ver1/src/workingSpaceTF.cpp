#include "workingSpaceTF.h"

void initializeWaypoints(Waypoint *waypoint, double V)
{
    waypoint->x = 0.000;
    waypoint->y = 0.000;
    waypoint->z = 0.000;
    waypoint->W = 0.000;
    waypoint->P = 0.000;
    waypoint->R = 0.000;
    waypoint->V = V;
    waypoint->C = "CNT100";
}

void zBiasWithNormal(std::vector<std::vector<double>> &points, double TF_Z_BIAS)
{   
    double roll = 0, pitch = 0, yaw = 0;
    Eigen::Vector4d translation;
    translation << 0, 0, -TF_Z_BIAS, 1;

    for (int i = 0; i < points.size(); ++i)
    {
        double roll_fanuc = 0, roll = 0, pitch = 0, yaw = 0;
        Eigen::Vector4d origin_position;
        origin_position << points[i][0], points[i][1], points[i][2], 1;

        roll = std::atan2(points[i][3], points[i][5]);
        pitch = std::atan2(points[i][4], points[i][5]);
        yaw = std::atan2(points[i][3], points[i][4]);

        Eigen::Matrix3d rotation_matrix;
        rotation_matrix = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                          Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                          Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());

        Eigen::Matrix4d transition_matrix = Eigen::Matrix4d::Identity();
        transition_matrix.block<3, 3>(0, 0) = rotation_matrix;
        transition_matrix.block<3, 1>(0, 3) = translation.head<3>();

        Eigen::Vector4d new_position = transition_matrix * origin_position;
        points[i][0] = new_position(0);
        points[i][1] = new_position(1);
        points[i][2] = new_position(2);
    }
}

void vector2Angle(std::vector<std::vector<double>> &points )
{
    for (int i = 0; i < points.size(); ++i)
    {
        double roll_fanuc = 0;
        double limit = 10.0;

        if (!points[i][4] && !points[i][5])
            roll_fanuc = 0;
        else
            roll_fanuc = std::atan2(-points[i][3], -points[i][5]);

        roll_fanuc = -roll_fanuc * 180.0 / M_PI;
        if (roll_fanuc > limit)
            roll_fanuc = limit;
        if (roll_fanuc < -limit)
            roll_fanuc = -limit;

        points[i][3] = roll_fanuc;
        points[i][4] = 0.0;
        points[i][5] = 0.0;
    }
}

void workingSpaceTF(const std::vector<std::vector<double>> &points, std::vector<Waypoint> &waypoints, double theta, double TF_Z_BIAS, double TF_X_BIAS, double TF_Y_BIAS, double vel)
{
    // Transform robot base to camera
    double transition_rtc[3] = {0.000, 0.000, 0.000};
    Eigen::MatrixXd tf_robot_to_camera(4, 4);
    tf_robot_to_camera << 0.0, -1.0, 0.0, transition_rtc[0],
        -1.0, 0.0, 0.0, transition_rtc[1],
        0.0, 0.0, -1.0, transition_rtc[2],
        0.0, 0.0, 0.0, 1.0;
    // std::cout << "tf_robot_to_camera.inverse():" << std::endl << tf_robot_to_camera.inverse() << std::endl;

    // Transform robot base to workspace
    cout << TF_Z_BIAS << endl;
    double transition_p[3] = {515.000 + TF_X_BIAS, -27.000 + TF_Y_BIAS, -325.827 + TF_Z_BIAS};
    double transition_v[3] = {-180, 0, 0};

    theta = theta * (3.1415 / 180);
    Eigen::MatrixXd tf_robot_workspace(4, 4);
    tf_robot_workspace << cos(theta), -sin(theta), 0, transition_p[0],
        sin(theta), cos(theta), 0, transition_p[1],
        0.0, 0.0, 1.0, transition_p[2],
        0.0, 0.0, 0.0, 1.0;
    // std::cout << "tf_robot_workspace:" << std::endl << tf_robot_workspace << std::endl;

    // start point
    Waypoint startPoint;
    initializeWaypoints(&startPoint, vel);
    startPoint.x = 420.246;
    startPoint.y = 0.000;
    startPoint.z = 53.417;
    startPoint.W = -180.000;
    startPoint.P = 0.000;
    startPoint.R = 0.000;

    waypoints.push_back(startPoint);
    for (int i = 0; i < points.size(); i++)
    {
        Eigen::Vector4d point_matrix(points[i][0], points[i][1], points[i][2], 1.0);
        Eigen::MatrixXd position_tf = tf_robot_workspace * tf_robot_to_camera.inverse() * point_matrix;

        // Transform vectors to workspace
        double vector_tf[3];
        for (int j = 0; j < 3; j++)
        {
            vector_tf[j] = points[i][j + 3] + transition_v[j];
        }

        // Output waypoints
        Waypoint newPoint;
        initializeWaypoints(&newPoint, vel);
        newPoint.x = position_tf(0, 0);
        newPoint.y = position_tf(1, 0);
        newPoint.z = position_tf(2, 0);
        newPoint.W = vector_tf[0];
        newPoint.P = vector_tf[1];
        newPoint.R = vector_tf[2];
        waypoints.push_back(newPoint);
    }
    waypoints.push_back(startPoint);
}

int writeLsFile(const std::string &absfile, const std::string &file, const std::vector<Waypoint> &waypoints)
{
    std::ofstream f(absfile);
    if (!f.is_open())
    {
        std::cout << "Error opening file: " << file << std::endl;
        return -1;
    }

    f << "/PROG  " << file << "\n";
    f << "/ATTR\n";
    f << "OWNER       = MNEDITOR;\n";
    f << "COMMENT     = \"\";\n";
    f << "PROG_SIZE   = 636;\n";
    f << "CREATE      = DATE 23-01-07  TIME 11:59:14;\n";
    f << "MODIFIED    = DATE 23-01-07  TIME 12:02:18;\n";
    f << "FILE_NAME   = ;\n";
    f << "VERSION     = 0;\n";
    f << "LINE_COUNT  = 4;\n";
    f << "MEMORY_SIZE = 992;\n";
    f << "PROTECT     = READ_WRITE;\n";
    f << "TCD:  STACK_SIZE    = 0,\n";
    f << "      TASK_PRIORITY = 50,\n";
    f << "      TIME_SLICE    = 0,\n";
    f << "      BUSY_LAMP_OFF = 0,\n";
    f << "      ABORT_REQUEST = 0,\n";
    f << "      PAUSE_REQUEST = 0;\n";
    f << "DEFAULT_GROUP    = 1,*,*,*,*;\n";
    f << "CONTROL_CODE     = 00000000 00000000;\n";

    f << "/MN\n";
    f << "   1:J P[1] 100" << "%" << " FINE    ;\n";
    for (size_t i = 2; i <= waypoints.size(); ++i)
    {
        f << "   " << i << ":L P[" << i << "] " << waypoints[i - 1].V << "mm/sec " << waypoints[i - 1].C << "    ;\n";
    }

    f << "/POS\n";
    for (size_t i = 1; i <= waypoints.size(); ++i)
    {
        f << "P[" << i << "]{\n";
        f << "   GP1:\n";
        f << "    UF : 0, UT : 6,      CONFIG : 'N U T, 0, 0, 0',\n";
        f << "    X =  " << std::fixed << std::setprecision(3) << waypoints[i - 1].x << "  mm,    Y =   " << waypoints[i - 1].y << "  mm,    Z =   " << waypoints[i - 1].z << "  mm,\n";
        f << "    W =  " << waypoints[i - 1].W << " deg,    P =   " << waypoints[i - 1].P << " deg,    R =   " << waypoints[i - 1].R << " deg\n";
        f << "};\n";
    }

    f << "/END\n";
    f.close();

    return 0;
}