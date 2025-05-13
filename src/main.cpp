#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>
#include <Eigen/Dense>
#include <iomanip>
#include <iostream>

#include "KalmanFilter.h"

int main(int argc, char* argv[]) {
    const int n = 6; // Number of states: pos_x, pos_y, vel_x, vel_y, acc_x, acc_y
    const int m = 6; // Number of measurements: pos_x, pos_y, vel_x, vel_y, acc_x, acc_y

    Eigen::MatrixXd A(n, n); // System dynamics matrix (to be updated per dt)
    Eigen::MatrixXd C(m, n); // Output (measurement) matrix
    Eigen::MatrixXd Q(n, n); // Process noise covariance
    Eigen::MatrixXd R(m, m); // Measurement noise covariance
    Eigen::MatrixXd P(n, n); // Estimate error covariance

    // Initialize measurement model (C)
    C.setZero();
    C(0, 0) = 1; // pos_x
    C(1, 1) = 1; // pos_y
    C(2, 2) = 1; // vel_x
    C(3, 3) = 1; // vel_y
    C(4,4)=1; // acc_x
    C(5,5)=1; // acc_y

    Q = Eigen::MatrixXd::Identity(n, n) * 0.01;
    R = Eigen::MatrixXd::Identity(m, m) * 0.1;
    P = Eigen::MatrixXd::Identity(n, n) * 1.0;

    // Initialize Kalman Filter (with dummy dt, real dt used in update)
    KalmanFilter kf(0.033, A, C, Q, R, P);

    std::ifstream file("data/merged_files/merged_data.csv");
    std::ofstream out("kf_output.csv");
    std::string line;

    if (!file.is_open()) {
        std::cerr << "Failed to open merged_data.csv" << std::endl;
        return 1;
    }

    std::getline(file, line); // Skip CSV header
    out << "timestamp,est_pos_x,est_pos_y,est_vel_x,est_vel_y,est_acc_x,est_acc_y\n";

    bool initialized = false;
    double timestamp;


    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string cell;
        std::vector<double> row;

        while (std::getline(ss, cell, ',')) {
            row.push_back(std::stod(cell));
        }
/*
        std::cout << "Row size: " << row.size() << ", values: ";
        for (double val : row) std::cout << val << " ";
        std::cout << std::endl;
        */
        if (row.size() < 8) continue;

        timestamp = row[0];
        double pos_x = row[1], pos_y = row[2];
        double vel_x = row[3], vel_y = row[4];
        double acc_x = row[5], acc_y = row[6];
        double dt = row[7];

        // Build A matrix dynamically with current dt
        A.setIdentity();
        A(0, 2) = dt;
        A(1, 3) = dt;
        A(0, 4) = 0.5 * dt * dt;
        A(1, 5) = 0.5 * dt * dt;
        A(2, 4) = dt;
        A(3, 5) = dt;

        if (!initialized) {
            Eigen::VectorXd x0(n);
            x0 << pos_x, pos_y, vel_x, vel_y, acc_x, acc_y;
            kf.init(timestamp, x0);
            initialized = true;
        }


        // Create measurement vector from odometry
        Eigen::VectorXd y(m);
        y << pos_x, pos_y, vel_x, vel_y, acc_x, acc_y;

        // Update Kalman filter
        kf.update(y, dt, A);

        // Output estimated state
        Eigen::VectorXd x_hat = kf.state();


        //out << timestamp;
        out << std::fixed << std::setprecision(9) << timestamp;
        for (int i = 0; i < x_hat.size(); ++i) {
            out << "," << x_hat(i);
        }
        out << "\n";

        //std::cout << "Wrote state at t=" << timestamp << ": " << x_hat.transpose() << std::endl;
    }

    file.close();
    out.close();

    std::cout << "Check output file size in explorer: should be > 0 bytes.\n";

    std::cout << "Output saved to kf_output.csv\n";
    return 0;
}

