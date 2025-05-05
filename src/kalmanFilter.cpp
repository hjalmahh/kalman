#include <iostream>
#include <stdexcept>

#include "KalmanFilter.h"


KalmanFilter::KalmanFilter(
    double dt,
    const Eigen::MatrixXd& A, //system dynamics
    const Eigen::MatrixXd& C, //output matrix
    const Eigen::MatrixXd& Q,//Process noise covariance
    const Eigen::MatrixXd& R,//Measurment noise covariance
    const Eigen::MatrixXd& P)//Estimate erro covariance
    : input(A), output(C), meas_noise(R),
        proc_noise(Q), est_error0(P), dt(dt), initialized(false)
{
    m=C.rows();
    n=A.rows();
    I= Eigen::MatrixXd::Identity(n,n);
    x_hat = Eigen::VectorXd::Zero(n);
    x_hat_new = Eigen::VectorXd::Zero(n);
}


KalmanFilter::KalmanFilter() {}

void KalmanFilter::init(double t0, const Eigen::VectorXd& x0)
{
    x_hat=x0;
    est_error= est_error0;
    this-> t0 = t0;
    t=t0;
    initialized=true;
}

void KalmanFilter::init()
{
    x_hat.setZero();
    est_error=est_error0;
    t0=0;
    t=t0;
    initialized=true;
}

void KalmanFilter::update(const Eigen::VectorXd& y)
{
    if(!initialized)
        throw std::runtime_error("KalmanFilter::update: not initilaized");

    x_hat_new= input * x_hat;
    est_error = input * est_error *input.transpose() + proc_noise;
    K = est_error * output.transpose()*(output*est_error*output.transpose() + meas_noise).inverse();
    x_hat_new += K *  (y-output*x_hat_new);
    est_error = (I-K*output)*est_error;
    x_hat = x_hat_new;

    t += dt;
}

void KalmanFilter::update(const Eigen::VectorXd& y, double dt, const Eigen::MatrixXd& input)
{
    this-> input = input;
    this-> dt = dt;
    update(y);
}
