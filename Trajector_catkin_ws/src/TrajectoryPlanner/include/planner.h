#ifndef __PLANNER_H
#define __PLANNER_H

#include "config.h"

class path_planner {
private:
    Eigen::MatrixXd coef;
    double T;
    bool is_empty_flag;

public:
    path_planner();
    void reset();
    void generateTraj(double _T, 
                    Eigen::Vector3d start_pos, 
                    Eigen::Vector3d start_vel, 
                    Eigen::Vector3d start_acc,
                    Eigen::Vector3d end_pos,
                    Eigen::Vector3d end_vel,
                    Eigen::Vector3d end_acc);
    Eigen::Vector3d getPos(double t);
    Eigen::Vector3d getVel(double t);
    Eigen::Vector3d getAcc(double t);
};

#endif // __PLANNER_H
