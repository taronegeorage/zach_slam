#include <iostream>
#include<fstream>
#include<types.h>
#include <vector>
#include <sstream>
#include <Eigen/Core>

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
    // for(auto& l : ldmks)
    //    cout << l.id << " " << l.x << " " << l.y << endl; 
    vector<sensors> sens;
    LoadSensors(argv[2], sens);
    
    int n = ldmks.size();
    cerr << 2*n+3 << endl;
    Eigen::VectorXd mu(2*n+3);
    for(int i = 0; i < 2*n+3; ++i) mu[i] = 0;
    cout << mu.matrix() << endl;
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
                ss >> odom[0] >> odom[1] >> odom[2];
                num++;
            } else {
                double id, range, bearing;
                ss >> id >> range >> bearing;
                laser.push_back({id, range, bearing});
            }
        }
    }
    inf.close();
}

