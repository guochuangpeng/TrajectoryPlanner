#include "planner.h"

path_planner::path_planner() {
    is_empty_flag = 1;
    coef.resize(3, 6);
}

void path_planner::reset() {
    is_empty_flag = 1;
}

void path_planner::generateTraj(double _T, 
                Eigen::Vector3d start_pos, 
                Eigen::Vector3d start_vel, 
                Eigen::Vector3d start_acc,
                Eigen::Vector3d end_pos,
                Eigen::Vector3d end_vel,
                Eigen::Vector3d end_acc) {
    is_empty_flag = 0;
    T = _T;

    Eigen::MatrixXd A(6, 6), A_inv(6, 6);
    Eigen::VectorXd tt(6);
    tt(0) = 1;
    for (int i = 1; i < 6; i++) {
        tt(i) = tt(i - 1) * _T;
    }
    A(0, 5) = A(1, 4) = A(2, 3) = 1;
    A.row(3) << tt(5), tt(4), tt(3), tt(2), tt(1), tt(0);
    A.row(4) << 5*tt(4), 4*tt(3), 3*tt(2), 2*tt(1), tt(0), 0;
    A.row(5) << 20*tt(3), 12*tt(2), 6*tt(1), 2*tt(0), 0, 0;
    A_inv = A.inverse();

    for (int i = 0; i < 3; i++) {
        Eigen::VectorXd b(6);
        b << start_pos(i), start_vel(i), start_acc(i), end_pos(i), end_vel(i), end_acc(i);
        coef.row(i) = (A_inv * b).transpose();
    }
}

Eigen::Vector3d path_planner::getPos(double t) {
    if (is_empty_flag || t < 0 || t > T) {
        return Eigen::Vector3d(0);
    }

    Eigen::Vector3d pos;
    Eigen::VectorXd tt(6);
    tt(0) = 1;
    for (int i = 1; i < 6; i++) {
        tt(i) = tt(i - 1) * t;
    }

    pos << coef(0)*tt, coef(1)*tt, coef(2)*tt;
    return pos;
}

Eigen::Vector3d path_planner::getVel(double t) {
    if (is_empty_flag || t < 0 || t > T) {
        return Eigen::Vector3d(0);
    }

    Eigen::Vector3d vel;
    Eigen::VectorXd tt(6);
    tt(0) = 0; tt(1) = 1;
    for (int i = 2; i < 6; i++) {
        tt(i) = tt(i - 1) * t * i;
    }

    vel << coef(0)*tt, coef(1)*tt, coef(2)*tt;
    return vel;
}

Eigen::Vector3d path_planner::getAcc(double t) {
    if (is_empty_flag || t < 0 || t > T) {
        return Eigen::Vector3d(0);
    }

    Eigen::Vector3d acc;
    Eigen::VectorXd tt(6);
    tt(0) = 0; tt(1) = 0; tt(2) = 1;
    for (int i = 3; i < 6; i++) {
        tt(i) = tt(i - 1) * t * i * (i - 1);
    }

    acc << coef(0)*tt, coef(1)*tt, coef(2)*tt;
    return acc;
}
