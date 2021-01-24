#include <iostream>
#include<fstream>
#include <Eigen/Core>

using namespace std;

void compute_sigma_points(Eigen::MatrixXd sigma_points, Eigen::VectorXd mu, Eigen::MatrixXd sigma, double lambda, double alpha, double beta);

int main() {
    // initial distribution
    Eigen::Matrix2d sigma = 0.1 * Eigen::Matrix2d::Identity();
    Eigen::Vector2d mu;
    mu << 1, 2;
    int n = 2;

    // compute lambda
    double alpha = 0.9, beta = 2, kappa = 1;
    double lambda = alpha * alpha * (n + kappa) - n;

    // compute the sigma points corresponding to mu and sigma
    Eigen::MatrixXd sigma_points(n, 2*n+1);
    sigma_points.setZero();
        
}

void compute_sigma_points(Eigen::MatrixXd sigma_points, Eigen::VectorXd mu, Eigen::MatrixXd sigma, double lambda, double alpha, double beta) {
    // first sigma points is the mean
    
}

