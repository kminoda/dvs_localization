#include <iostream>
#include <vector>
#include <fstream>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include "ceres/ceres.h"
#include "glog/logging.h"

using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;

std::string WORKDIR_PATH = "/home/koji/git/dvs_localization/";

/**
 * @brief Cost function
 **/
struct CostFunctor{
    double fx, fy, cx, cy, X1, Y1, Z1, X2, Y2, Z2, u1, v1, u2, v2;
    double d1, d2, d3, d4, d5;
    CostFunctor() {
        std::ifstream infile_calibration(WORKDIR_PATH+"localization/data/ceres_input/calibration.txt");
        infile_calibration >> this->fx >> this->fy;
        infile_calibration >> this->cx >> this->cy; 
        infile_calibration >> this->d1 >> this->d2 >> this->d3 >> this->d4 >> this->d5;
        
        std::ifstream infile_markers(WORKDIR_PATH+"localization/data/ceres_input/markers.txt");
        infile_markers >> this->u1 >> this->v1 >> this->X1 >> this->Y1 >> this->Z1;
        infile_markers >> this->u2 >> this->v2 >> this->X2 >> this->Y2 >> this->Z2;
    }

    template <typename T>
    bool operator()(const T* const x, 
                    // const T* const y, 
                    const T* const z, 
                    const T* const yaw, 
                    const T* const l1, 
                    const T* const l2, 
                    T* residual) const{
        
        double y = -1.0;
        // residual[0] = l1[0]*T(u1) - T(fx)*(T(X1)*cos(yaw[0]) - T(Y1)*sin(yaw[0]) + x[0]) - T(cx)*(T(Z1)+z[0]);
        // residual[1] = l1[0]*T(v1) - T(fy)*(T(X1)*sin(yaw[0]) + T(Y1)*cos(yaw[0]) + y[0]) - T(cy)*(T(Z1)+z[0]);
        // residual[2] = l1[0] - T(Z1) - z[0];
        // residual[3] = l2[0]*T(u2) - T(fx)*(T(X2)*cos(yaw[0]) - T(Y2)*sin(yaw[0]) + x[0]) - T(cx)*(T(Z2)+z[0]);
        // residual[4] = l2[0]*T(v2) - T(fy)*(T(X2)*sin(yaw[0]) + T(Y2)*cos(yaw[0]) + y[0]) - T(cy)*(T(Z2)+z[0]);
        // residual[5] = l2[0] - T(Z2) - z[0];
        residual[0] = l1[0]*T(u1) - T(fx)*(T(X1)*cos(yaw[0]) - T(Y1)*sin(yaw[0]) + x[0]) - T(cx)*(T(Z1)+z[0]);
        residual[1] = l1[0]*T(v1) - T(fy)*(T(X1)*sin(yaw[0]) + T(Y1)*cos(yaw[0]) + T(y)) - T(cy)*(T(Z1)+z[0]);
        residual[2] = l1[0] - T(Z1) - z[0];
        residual[3] = l2[0]*T(u2) - T(fx)*(T(X2)*cos(yaw[0]) - T(Y2)*sin(yaw[0]) + x[0]) - T(cx)*(T(Z2)+z[0]);
        residual[4] = l2[0]*T(v2) - T(fy)*(T(X2)*sin(yaw[0]) + T(Y2)*cos(yaw[0]) + T(y)) - T(cy)*(T(Z2)+z[0]);
        residual[5] = l2[0] - T(Z2) - z[0];
        return true;
    }
};

int main(int argc, char** argv)
{
    google::InitGoogleLogging(argv[0]);
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double yaw = 0.0;
    double l1 = 0.0;
    double l2 = 0.0;
    int method;

    ceres::Problem problem;

    // ceres::CostFunction* cost_function=new ceres::AutoDiffCostFunction<CostFunctor, 6, 1, 1, 1, 1, 1, 1>(new CostFunctor);
    ceres::CostFunction* cost_function=new ceres::AutoDiffCostFunction<CostFunctor, 6, 1, 1, 1, 1, 1>(new CostFunctor);

    // problem.AddResidualBlock(cost_function, NULL, &x, &y, &z, &yaw, &l1, &l2);
    problem.AddResidualBlock(cost_function, NULL, &x, &z, &yaw, &l1, &l2);

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = false;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    std::cout << summary.BriefReport() << std::endl;
    // std::cout << "x:" << x << ", y:" << y << ", z:" << z << ", yaw:" << yaw << std::endl;
    std::cout << "x:" << x << ", z:" << z << ", yaw:" << yaw << std::endl;

    return 0;
}
