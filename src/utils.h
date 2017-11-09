#ifndef UTILS_H
#define UTILS_H

#include "Eigen-3.3/Eigen/Core"

Eigen::MatrixXd mapToCar(double car_x, double car_y, double car_psi);
Eigen::MatrixXd carToMap(double car_x, double car_y, double car_psi);

Eigen::VectorXd derivative(Eigen::VectorXd coeffs);

#endif
