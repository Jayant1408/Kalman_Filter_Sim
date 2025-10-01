#pragma once
#include <Eigen/Dense>
#include <vector>
#include <string>

struct KFConfig{

    double dt{0.1};
    double accel{0.5};
    double q_var{1e-3};  // process noise scalling
    double r_var{4.0};
    int steps{120};

};

struct KFOutputs {
    std::vector<double> t, x_true, v_true, z_meas, x_hat, v_hat;
};

class KalmanFilter1D{
    public:
        explicit KalmanFilter1D(const KFConfig& cfg);
        KFOutputs run();
        static bool writeCSV(const std::string& path, const KFOutputs& out);

    private:
        KFConfig cfg_;
        Eigen::Matrix<double,2,2> A_;
        Eigen::Matrix<double,2,1> B_;
        Eigen::Matrix<double,1,2> H_;
        Eigen::Matrix<double,2,2> Q_;
        Eigen::Matrix<double,1,1> R_;
        Eigen::Vector2d x_est_;
        Eigen::Matrix2d P_;
};
