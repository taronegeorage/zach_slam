#ifndef EKF_H
#define EKF_H

#include <Eigen/Core>
#include <types.h>
#include <vector>

class EKF {
public:
    EKF(Eigen::MatrixXd mu, Eigen::MatrixXd sigma, std::vector<sensors> sens, std::vector<int> obser):mu_(mu),sigma_(sigma),sensdata_(sens),observed_(obser) { 
        dim = sigma.cols(); 
        R << motionNoise, 0, 0, 0, motionNoise, 0, 0, 0, motionNoise/10;
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
    Eigen::Matrix3d R;
};                                
                                  
#endif
