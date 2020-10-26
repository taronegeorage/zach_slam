#include <iostream>
#include<fstream>
#include<types.h>
#include <vector>
#include <sstream>
#include <Eigen/Core>
#include <ekf_slam/extended_kalman_filter.h>

using namespace std;

void LoadLandmark(const string &ldmarkFile, vector<landmark> &ldmks);
void LoadSensors(const string &sensorFile, vector<sensors> &sens);

int main(int argc, char **argv) {
    if(argc != 3) {
        cerr << endl << "Usage: ./ekf_slam path_to_landmark_pos path_to_sensor_data" << endl;
        return 1;
    }
    vector<landmark> ldmks;    
    LoadLandmark(argv[1], ldmks);
    vector<sensors> sens;
    LoadSensors(argv[2], sens);
    
    // Initialize belief
    int n = ldmks.size();
    Eigen::VectorXd mu(2*n+3);// = Eigen::VectorXd::Constant(2*n+3, 0.0);
    mu.setZero();
    Eigen::Matrix3d robSigma = Eigen::Matrix3d::Zero();
    Eigen::MatrixXd robMapSigma(3, 2*n);
    robMapSigma.setZero();
    Eigen::MatrixXd mapSigma(2*n, 2*n);
    mapSigma.setIdentity();
    mapSigma.diagonal().array() = INT_MAX;
    Eigen::MatrixXd sigma(3+2*n, 3+2*n);
    sigma << robSigma, robMapSigma, robMapSigma.transpose(), mapSigma;
    // cerr << sigma << endl;
    
    vector<int> observedLdmks(n, 0);
    EKF ekf(mu, sigma, sens, observedLdmks);
    // ekf.prediction_step(0);
    // ekf.correction_step(0);
    for(int i = 0; i < sens.size(); ++i) {
        cerr << i << endl;
        ekf.prediction_step(i);
        ekf.correction_step(i);
        cerr << ekf.getmu().transpose().matrix() << endl;
    }
    return 0;
}

void LoadLandmark(const string &ldmarkFile, vector<landmark> &ldmks) {
    ifstream inf;
    inf.open(ldmarkFile.c_str());
    while(!inf.eof()) {
        string s;
        getline(inf,s);
        if(!s.empty()) {
            stringstream ss;
            ss << s;
            int id;
            double x, y;
            ss >> id >> x >> y;
            ldmks.push_back({id, x, y});
        }
    }
    inf.close();
}

void LoadSensors(const string &sensorFile, vector<sensors> &sens) {
    ifstream inf;
    inf.open(sensorFile.c_str());
    vector<double> odom(3, 0);
    vector<vector<double>> laser;
    int num = 0;
    while(!inf.eof()) {
        string s;
        getline(inf, s);
        if(!s.empty()) {
            stringstream ss;
            ss << s;
            string senType;
            ss >> senType;
            if(senType == "ODOMETRY") {
                if(num > 0) {
                    sensors s(odom, laser);
                    sens.emplace_back(s);
                }
                laser.clear();
                // read r1, t and r2
                ss >> odom[0] >> odom[1] >> odom[2];
                num++;
            } else {
                double id, range, bearing;
                // read id, range and bearing
                ss >> id >> range >> bearing;
                laser.push_back({id, range, bearing});
            }
        }
    }
    inf.close();
}

