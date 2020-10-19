#ifndef EKF_H
#define EKF_H

#include <Eigen/Core>
#include <types.h>
#include <vector>

class EKF {
public:
    EKF(Eigen::MatrixXd mu, Eigen::MatrixXd sigma, std::vector<sensors> sens, std::vector<int> obser):mu_(mu),sigma_(sigma),sensdata_(sens),observed_(obser) { 
        dim = sigma.cols(); 
        R = Eigen::MatrixXd::Constant(dim,dim,0);
        R(0, 0) = motionNoise;
        R(1, 1) = motionNoise;
        R(2, 2) = motionNoise / 10;
        Fxj = Eigen::MatrixXd::Constant(5,dim,0);
        Fxj(0, 0) = 1; Fxj(1, 1) = 1; Fxj(2, 2) = 1;
    } 
    void param_init(Eigen::MatrixXd mu, Eigen::MatrixXd sigma, std::vector<sensors> sens, std::vector<int> obser);
    void prediction_step(int step);
    void correction_step(int step);
    Eigen::MatrixXd getmu();
    Eigen::MatrixXd getsigma();
private:                                                         
    Eigen::MatrixXd mu_, sigma_;
    std::vector<sensors> sensdata_;
    std::vector<int> observed_;
    int dim;
    double motionNoise = 0.1;
    Eigen::MatrixXd R;
    Eigen::MatrixXd Fxj;
};                                
                                  
#endif
