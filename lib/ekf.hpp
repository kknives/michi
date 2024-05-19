#include <Eigen/Dense>
#include <chrono>
#include <math.h>
#include <stdlib.h>
#include <tuple>

using Eigen::Matrix;
using Eigen::MatrixXf;
const float DEG_TO_RAD = 0.01745329251;
const float DT = 0.01;
class EKF {

public:
  // Covariance Matrix
  Matrix<float, 4, 4> m_predicted_noise_cov;
  Matrix<float, 2, 2> m_measurement_noise_cov;

  // ip noise
  Matrix<float, 2, 2> m_ip_noise;

  // measurement matrix
  Matrix<float, 2, 4> m_H;

  // estimated state vector
  Matrix<float, 4, 1> m_xEst;

  // True state
  Matrix<float, 4, 1> m_xTrue;
  Matrix<float, 4, 4> m_PEst;

  EKF()
      : m_predicted_noise_cov{{0.1, 0.0, 0.0, 0.0},
                              {0.0, 0.1, 0.0, 0.0},
                              {0.0, 0.0, (1 * DEG_TO_RAD), 0.0},
                              {0.0, 0.0, 0.0, 1.0}},

        m_measurement_noise_cov{
            {0.1, 0},
            {0, 0.1},
        },
        m_ip_noise{{1.0, 0.0}, {0.0, (30 * DEG_TO_RAD)}}, m_H{{1, 0, 0, 0},
                                                              {0, 1, 0, 0}},
        m_xEst(MatrixXf::Zero(4, 1)), m_xTrue(MatrixXf::Zero(4, 1)),
        m_PEst(MatrixXf::Identity(4, 4)) {}

  MatrixXf control_input(Eigen::Vector3f linear_accel,
                         Eigen::Vector3f angular_vel,
                         Eigen::Vector3f position) {
    Matrix<float, 2, 1> u;
    float vel, imu_vel = 0.0, vel_x, vel_y, odom_vel = 0.0, yaw_vel = 0.0, dS,
               current_pos, prev_pos;

    current_pos = position.norm();

    // change in distance
    dS = current_pos - prev_pos;

    // calculating odometry velocity
    odom_vel = dS / DT;

    // calculating imu velocity
    vel_x = vel_x + linear_accel(0) * DT;

    vel_y = vel_y + linear_accel(1) * DT;

    imu_vel = sqrt(pow(vel_x, 2) + pow(vel_y, 2));

    // calc imu velocity
    if (odom_vel < 0)
      imu_vel = -1 * imu_vel;
    else
      imu_vel = imu_vel;

    // calc yaw velocity
    yaw_vel = angular_vel(2);

    vel = complementary(imu_vel, odom_vel);

    u << vel, yaw_vel;

    prev_pos = current_pos;

    return u;
  }

  std::tuple<MatrixXf, MatrixXf> observation(MatrixXf xTrue, MatrixXf u) {
    xTrue = state_model(xTrue, u);

    Matrix<float, 2, 1> ud;
    ud = u + (m_ip_noise * MatrixXf::Random(2, 1));

    return std::make_tuple(xTrue, ud);
  }

  MatrixXf state_model(MatrixXf x, MatrixXf u) {
    Matrix<float, 4, 4> A{
        {1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 0}};

    Matrix<float, 4, 2> B;
    B << (DT * cos(x(2))), 0, (DT * sin(x(2))), 0, 0, DT, 1, 0;
    x = (A * x) + (B * u);

    return x;
  }

  MatrixXf jacob_f(MatrixXf x, MatrixXf u) {
    float yaw = x.coeff(2, 0);

    float v = u.coeff(0, 0);

    Matrix<float, 4, 4> jF;
    jF << 1.0, 0.0, (-DT * v * sin(yaw)), (DT * cos(yaw)), 0.0, 1.0,
        (DT * v * cos(yaw)), (DT * sin(yaw)), 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
        1.0;

    return jF;
  }

  MatrixXf observation_model(MatrixXf x) {
    Matrix<float, 2, 1> z;

    z = m_H * x;

    return z;
  }

  std::tuple<MatrixXf, MatrixXf> ekf_estimation(MatrixXf xEst, MatrixXf PEst,
                                                MatrixXf z, MatrixXf u) {
    // Predict
    Matrix<float, 4, 1> xPred;
    xPred = state_model(xEst, u);

    // state vector
    Matrix<float, 4, 4> jF;
    jF = jacob_f(xEst, u);

    Matrix<float, 4, 4> PPred;
    PPred = (jF * PEst * jF.transpose()) + m_predicted_noise_cov;

    // Update
    Matrix<float, 2, 1> zPred;
    zPred = observation_model(xPred);

    Matrix<float, 2, 1> y;
    y = z - zPred; // measurement residual

    Matrix<float, 2, 2> S;
    S = (m_H * PPred * m_H.transpose()) +
        m_measurement_noise_cov; // Innovation Covariance

    Matrix<float, 4, 2> K;
    K = PPred * m_H.transpose() * S.inverse(); // Kalman Gain

    xEst = xPred + K * y; // update step

    PEst = (MatrixXf::Identity(4, 4) - (K * m_H)) * PPred;

    return std::make_tuple(xEst, PEst);
  }

  float complementary(float IMU_vel, float ODOM_vel) {
    // accelerometer wt.
    float alpha = 0.4;

    // complementary velocity
    float compl_vel = (alpha * IMU_vel) + (1 - alpha) * (ODOM_vel);

    return compl_vel;
  }

  std::tuple<MatrixXf, MatrixXf> run_ekf(Matrix<float, 2, 1> control_ip) {
    Matrix<float, 2, 1> ud = MatrixXf::Zero(2, 1);

    // observation vector
    Matrix<float, 2, 1> z = MatrixXf::Zero(2, 1);

    Matrix<float, 2, 1> u = control_ip;
    std::tie(m_xTrue, ud) = observation(m_xTrue, u);
    z = observation_model(m_xTrue);
    std::tie(m_xEst, m_PEst) = ekf_estimation(m_xEst, m_PEst, z, ud);

    return std::make_tuple(m_xEst, m_PEst);
  }
};
