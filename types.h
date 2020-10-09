#ifndef TYPES_H
#define TYPES_H

#include <string>
#include <vector>

using namespace std;

struct landmark {
    int id;
    double x, y;
    landmark(int _id, double _x, double _y) {
        id = _id;
        x = _x;
        y = _y;
    }
    landmark(string _id, string _x, string _y) {
        id = atoi(_id.c_str());
        x = atof(_x.c_str());
        y = atof(_y.c_str());
    }
};

struct sensors {
    vector<double> odom;
    vector<vector<double>> laser;
    sensors(vector<double> _odom, vector<vector<double>> _laser): odom(_odom), laser(_laser) { }
};

#endif
