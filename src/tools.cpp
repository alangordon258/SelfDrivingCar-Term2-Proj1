#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
    VectorXd rmse(4);
    rmse << 0, 0, 0, 0;
    
    // Check inputs
    if (estimations.size() != ground_truth.size()
        || estimations.size() == 0) {
        cout << "Invalid estimation or ground_truth data" << endl;
        return rmse;
    }
    
    // Calculate RMSE
    for (int i = 0; i < estimations.size(); ++i) {
        VectorXd res = estimations[i] - ground_truth[i];
        res = res.array() * res.array();
        rmse += res;
    }
    
    rmse /= estimations.size();
    rmse = rmse.array().sqrt();
    
    return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_current) {
    MatrixXd hj(3, 4);
    // Get the components of the x vector
    double px = x_current(0);
    double py = x_current(1);
    double vx = x_current(2);
    double vy = x_current(3);
    
    // Calculate the square of the x and y components
    double px2 = px * px;
    double py2 = py * py;
    
    // Avoid division by zero
    if (fabs(px2) < EPSILON)
        px2 = EPSILON;
    
    // Avoid division by zero
    if (fabs(py2) < EPSILON)
        py2 = EPSILON;
    
    double ss = px2 + py2;
    double srss = sqrt(ss);
    
    // Create the Jacobian
    hj(0, 0) = px / srss;
    hj(0, 1) = py / srss;
    hj(0, 2) = 0;
    hj(0, 3) = 0;
    
    hj(1, 0) = -py / ss;
    hj(1, 1) = px / ss;
    hj(1, 2) = 0;
    hj(1, 3) = 0;
    
    hj(2, 0) = (py * (vx * py - px * vy)) / (ss * srss);
    hj(2, 1) = (px * (vy * px - py * vx)) / (ss * srss);
    hj(2, 2) = px / srss;
    hj(2, 3) = py / srss;
    
    return hj;
}

VectorXd Tools::CartesianToPolar(const VectorXd x) {
    VectorXd z_pred(3);
    
    // Get the components of the x vector
    double px = x(0);
    double py = x(1);
    double vx = x(2);
    double vy = x(3);
    
    if (fabs(px) < EPSILON)
        px = EPSILON;
    
    // Convert from cartesian to polar
    double px2 = px * px;
    double py2 = py * py;
    double rho = sqrt(px2 + py2);
    
    // Avoid division by zero
    if (fabs(rho) < EPSILON)
        rho = EPSILON;
    
    z_pred[0] = rho;
    z_pred[1] = atan2(py, px);
    z_pred[2] = (px * vx + py * vy) / rho;
    
    return z_pred;
}

double Tools::NormalizeAngle(double phi)
{
    double newPhi;
    const double twoPI = 2.0*M_PI;
    if (phi < -M_PI) {
        newPhi=std::fmod(phi+M_PI,twoPI)+M_PI;
    } else {
        newPhi=std::fmod(phi+M_PI,twoPI)-M_PI;
    }
    return newPhi;
}
