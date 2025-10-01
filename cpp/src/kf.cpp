#include "kf.hpp"
#include <random>
#include <fstream>
#include <iomanip>
#include <cmath>

KalmanFilter1D::KalmanFilter1D(const KFConfig& cfg) : cfg_(cfg) {
    const double dt = cfg_.dt;

    // --- Safer explicit assignments (avoid comma-initializer pitfalls) ---
    A_.setZero();
    A_(0,0) = 1.0;  A_(0,1) = dt;
    A_(1,0) = 0.0;  A_(1,1) = 1.0;

    B_.setZero();
    B_(0,0) = 0.5 * dt * dt;
    B_(1,0) = dt;

    H_.setZero();
    H_(0,0) = 1.0;  H_(0,1) = 0.0;

    Q_.setZero();
    Q_(0,0) = std::pow(dt,4) / 4.0;
    Q_(0,1) = std::pow(dt,3) / 2.0;
    Q_(1,0) = std::pow(dt,3) / 2.0;
    Q_(1,1) = std::pow(dt,2);
    Q_ *= cfg_.q_var;

    R_.setZero();
    R_(0,0) = cfg_.r_var;

    x_est_.setZero();
    P_.setIdentity();
    P_ *= 10.0;
}

KFOutputs KalmanFilter1D::run() {
    KFOutputs out;
    out.t.reserve(cfg_.steps);
    out.x_true.reserve(cfg_.steps);
    out.v_true.reserve(cfg_.steps);
    out.z_meas.reserve(cfg_.steps);
    out.x_hat.reserve(cfg_.steps);
    out.v_hat.reserve(cfg_.steps);

    std::mt19937 rng(0);
    std::normal_distribution<double> meas_noise(0.0, std::sqrt(cfg_.r_var));

    Eigen::Vector2d x_true(0.0, 0.0);
    Eigen::Vector2d x_pred;
    Eigen::Matrix2d P_pred;
    const Eigen::Matrix2d I = Eigen::Matrix2d::Identity();

    for (int k = 0; k < cfg_.steps; ++k) {
        double tk = k * cfg_.dt;
        out.t.push_back(tk);

        // true dynamics
        x_true = A_ * x_true + B_ * cfg_.accel;
        double z_k = x_true(0) + meas_noise(rng);

        // predict
        x_pred = A_ * x_est_ + B_ * cfg_.accel;
        P_pred = A_ * P_ * A_.transpose() + Q_;

        // update (use scalar S for numerical simplicity)
        Eigen::Matrix<double,1,1> y = Eigen::Matrix<double,1,1>(z_k) - H_ * x_pred;
        double S_scalar = (H_ * P_pred * H_.transpose())(0,0) + R_(0,0);
        Eigen::Matrix<double,2,1> K = (P_pred * H_.transpose()) / S_scalar;

        x_est_ = x_pred + K * y;
        P_ = (I - K * H_) * P_pred;

        // store
        out.x_true.push_back(x_true(0));
        out.v_true.push_back(x_true(1));
        out.z_meas.push_back(z_k);
        out.x_hat.push_back(x_est_(0));
        out.v_hat.push_back(x_est_(1));
    }
    return out;
    
}

// Now, OUTSIDE of run(), add writeCSV:
bool KalmanFilter1D::writeCSV(const std::string& path, const KFOutputs& out) {
    std::ofstream ofs(path);
    if (!ofs.is_open()) return false;

    ofs << std::fixed << std::setprecision(6);
    ofs << "t,true_pos,true_vel,meas_pos,est_pos,est_vel\n";
    for (size_t i = 0; i < out.t.size(); ++i) {
        ofs << out.t[i] << ","
            << out.x_true[i] << ","
            << out.v_true[i] << ","
            << out.z_meas[i] << ","
            << out.x_hat[i] << ","
            << out.v_hat[i] << "\n";
    }
    return true;
}
