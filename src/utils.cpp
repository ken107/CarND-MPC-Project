#include "utils.h"

Eigen::MatrixXd mapToCar(double car_x, double car_y, double car_psi) {
  car_x = -car_x;
  car_y = -car_y;
  car_psi = -car_psi;

  Eigen::MatrixXd mat(3,3);
  mat << cos(car_psi), -sin(car_psi), car_x*cos(car_psi)-car_y*sin(car_psi),
    sin(car_psi), cos(car_psi), car_x*sin(car_psi)+car_y*cos(car_psi),
    0, 0, 1;
  return mat;
}

Eigen::MatrixXd carToMap(double car_x, double car_y, double car_psi) {
  Eigen::MatrixXd mat(3,3);
  mat << cos(car_psi), -sin(car_psi), car_x,
    sin(car_psi), cos(car_psi), car_y,
    0, 0, 1;
  return mat;
}

Eigen::VectorXd derivative(Eigen::VectorXd coeffs) {
  Eigen::VectorXd out(coeffs.size()-1);
  for (auto i=1; i<coeffs.size(); i++) out(i-1) = coeffs(i)*i;
  return out;
}
