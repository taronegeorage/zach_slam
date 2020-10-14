#ifndef EKF_H
#define EKF_H

#include <Eigen/Core>
#include <types.h>
#include <vector>

class EKF {
public:
    EKF(Eigen::MatrixXd mu, Eigen::MatrixXd sigma, std::vector<sensors> sens, std::vector<int> obser):mu_(mu),sigma_(sigma),sensdata_(sens),observed_(obser) {} 
    void prediction_step(Eigen::MatrixXd& mu, Eigen::MatrixXd& sigma);
    void correction_step(Eigen::MatrixXd& mu, Eigen::MatrixXd& sigma);
private:                                                         
    Eigen::MatrixXd mu_, sigma_;
    std::vector<sensors> sensdata_;
    std::vector<int> observed_;                                 
};                                
                                  
#endif
