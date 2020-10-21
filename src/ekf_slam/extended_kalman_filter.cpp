#include <ekf_slam/extended_kalman_filter.h>
#include <iostream>
#include <cmath>
#include <Eigen/Dense>

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
    Eigen::MatrixXd Te = sigma_ * G.transpose();
    cerr << "----" << endl;
    cerr << Te.matrix() << endl;

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
            mu_(2*ldmkId+1,0) = mu_(0, 0) + laser[i][1] * cos(laser[i][2]+mu_(2,0));
            mu_(2*ldmkId+2,0) = mu_(1, 0) + laser[i][1] * sin(laser[i][2]+mu_(2,0));

            observed_[ldmkId] = 1;
        }
        // TODO: Add the landmark measurement to the Z vector
        Z(2*i, 0) = laser[i][1];
        Z(2*i+1, 0) = laser[i][2];

        // TODO: Use the current estimate of the landmark pose to compute the corresponding expected measurement in expectedZ:
        double deltax = mu_(2*ldmkId+1, 0) - mu_(0, 0);
        double deltay = mu_(2*ldmkId+2, 0) - mu_(1, 0);
        double distance2 = deltax*deltax+deltay*deltay;
        
        expectedZ(2*i-2, 0) = sqrt(distance2);
        expectedZ(2*i-1, 0) = remainder(atan2(deltay, deltax)-mu_(2, 0), 2*M_PI);

        // TODO: Compute the Jacobian Hi of the measurement function h for this observation
        Eigen::MatrixXd Hi(2, 5);
        Hi << -sqrt(distance2)*deltax, -sqrt(distance2)*deltay, 0, sqrt(distance2)*deltax, sqrt(distance2)*deltay, 
            deltay, -deltax, -distance2, -deltay, deltax;
        Hi = 1/distance2 * Hi;
        // Map Jacobian Hi to high dimensional space by a mapping matrix Fxj
        Fxj(4, 2*ldmkId+1) = 1;
        Fxj(5, 2*ldmkId+2) = 1;
        Hi = Hi * Fxj;
        //H.block<2, dim>(2*i, 0) = Hi;     
        H.row(2*i) = Hi.row(0);
        H.row(2*i+1) = Hi.row(1);     
    }
    // TODO: Construct the sensor noise matrix Q
    Eigen::MatrixXd Q(2*mNum, 2*mNum);
    Q.setIdentity();
    Q = 0.01 * Q;
    // TODO: Compute the Kalman gain
    Eigen::MatrixXd temp = H * sigma_ * H.transpose() + Q;
    Eigen::MatrixXd K = sigma_ * H.transpose() * temp.inverse();
    // TODO: Compute the difference between the expected and recorded measurements
    Eigen::MatrixXd diffZ = Z - expectedZ;
    for(int i = 1; i < mNum*2; ++i) {
        diffZ(i) = remainder(diffZ(i), 2*M_PI);
    }    
    // TODO: Finish the correction step by computing the new mu and sigma.
    mu_ = mu_ + K * diffZ;
    Eigen::MatrixXd Iden(dim, dim);
    Iden.setIdentity();
    sigma_ = (Iden-K*H) * sigma_;
}

Eigen::MatrixXd EKF::getmu() {
    return mu_;
}

Eigen::MatrixXd EKF::getsigma() {
    return sigma_;
}
