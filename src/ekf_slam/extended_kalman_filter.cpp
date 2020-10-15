#include <ekf_slam/extended_kalman_filter.h>
#include <iostream>
#include <cmath>

void EKF::prediction_step(int step) {
    vector<double> odom = sensdata_[step].odom;
    mu_(0,0) = mu_(0,0) + (odom[1] * cos(mu_(3,0)+odom[0]));
    mu_(1,0) = mu_(1,0) + (odom[1] * cos(mu_(3,0)+odom[0]));
    mu_(2,0) = mu_(2,0) + (odom[0] + odom[2]);
    mu_(2,0) = remainder(mu_(2,0), 2*M_PI);
    //cerr << mu_.matrix() << endl;
    
    // Compute the 3x3 Jacobian Gx of the motion model
    Eigen::Matrix3d Gx = Eigen::Matrix3d::Identity();
    Gx(0,2) = -odom[1] * cos(mu_(2,0)+odom[0]);
    Gx(1,2) = odom[1] * sin(mu_(2,0)+odom[0]);
    
    // Construct the full Jacobian G
    Eigen::MatrixXd Gl(3, dim-3);
    Gl.setZero();
    Eigen::MatrixXd Gll(dim-3, dim-3);
    Gll.setIdentity();
    Eigen::MatrixXd G(dim, dim);
    G << Gx, Gl, Gl.transpose(), Gll;
    cerr << "----" << endl;
    cerr << G.matrix() << endl;

    // Compute the predicted sigma after incorporating the motion
    sigma_ = G * sigma_ * G.transpose() + R;
    cerr << "----" << endl;
    cerr << sigma_.matrix() << endl;
}

void EKF::correction_step(int step) {
    // Number of Measurements
    vector<vector<double>> laser = sensdata_[step].laser;
    int mNum = laser.size();

    Eigen::MatrixXd Z(mNum*2, 1);
    Z.setZero();
    Eigen::MatrixXd expectedZ(mNum*2, 1);
    expectedZ.setZero();

    // Iterate over the measurements and compute the H matrix
    // (stacked Jacobian blocks of the measurement function)
    Eigen::MatrixXd H(2*mNum, dim);
    for(int i = 0; i < mNum; ++i) {
        int ldmkId = laser[i][0];
        if(!observed_[ldmkId]) {
            // TODO: Initialize its pose in mu based on the measurement and the current robot pose:    

            observed_[ldmkId] = 1;
        }
        // TODO: Add the landmark measurement to the Z vector

    }
}

Eigen::MatrixXd EKF::getmu() {
    return mu_;
}

Eigen::MatrixXd EKF::getsigma() {
    return sigma_;
}
