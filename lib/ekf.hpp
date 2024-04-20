#include <Eigen/Dense>
#include <chrono>

using Eigen::Matrix;
using Eigen::MatrixXf;
const float DEG_TO_RAD = 0.01745329251;

class EKF
{
public:
  // Covariance Matrix
  Matrix<float, 4, 4> m_Q;
  Matrix<float, 2, 2> m_R;

  // ip noise
  Matrix<float, 2, 2> m_ip_noise;

  // measurement matrix
  Matrix<float, 2, 4> m_H;

  // acceleration & gyro variables
  Eigen::Vector3f m_gyro_msg;
  Eigen::Vector3f m_accel_msg;
  Eigen::Vector3f m_odom_msg;
  float m_distance;
  float m_accel_net;

  EKF() : m_Q { 0.1,  0.0,      0.0,         0.0,
                0.0,  0.1,      0.0,         0.0,
                0.0,  0.0, (1 * DEG_TO_RAD), 0.0,
                0.0,  0.0,      0.0,         1.0,},

              m_R { 0.1, 0,
                     0, 0.1, },
              m_ip_noise { 1.0,                 0.0,
                         0.0, (30*DEG_TO_RAD) },
              m_H { 1, 0, 0, 0,
                    0, 1, 0, 0},

              m_accel_net {0.0}
  {
    // Covariance Matrix
    // m_Q << 0.1, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, (1 * deg_to_rad),
    //   0.0, 0.0, 0.0, 0.0, 1.0;

    // m_R << 0.1, 0, 0, 0.1;

    // // input noise
    // m_ip_noise << 1.0, 0, 0, (30 * deg_to_rad);

    // // measurement matrix
    // m_H << 1, 0, 0, 0, 0, 1, 0, 0;

    // // acceleration
    // m_accel_net = 0.0;
  }

  // time-step
  const float m_DT = 0.1;

  MatrixXf control_input(float imu_vel, float odom_vel, float yaw_vel)
  {
     Matrix<float, 2, 1> u;
     float vel;

     vel = complementary(imu_vel, odom_vel);

     u << vel, yaw_vel;

     return u;
  }

  std::tuple<MatrixXf, MatrixXf> observation(MatrixXf xTrue, MatrixXf u)
  {
    xTrue = state_model(xTrue, u);

    Matrix<float, 2, 1> ud;
    ud = u + (m_ip_noise * MatrixXf::Random(2, 1));

    return std::make_tuple(xTrue, ud);
  }

  MatrixXf state_model(MatrixXf x, MatrixXf u)
  {
    Matrix<float, 4, 4> A {1, 0, 0, 0,
                           0, 1, 0, 0,
                           0, 0, 1, 0,
                           0, 0, 0, 0};

    Matrix<float, 4, 2> B;
        B << (m_DT*cos(x.coeff(2,0))), 0,
             (m_DT*sin(x.coeff(2,0))), 0,
    			      0, m_DT,
    		              1, 0;
    x = (A * x) + (B * u);

    return x;
  }

  MatrixXf jacob_f(MatrixXf x, MatrixXf u)
  {
    float yaw = x.coeff(2, 0);

    float v = u.coeff(0, 0);

    Matrix<float, 4, 4> jF;
    jF << 1.0, 0.0, (-dt * v * sin(yaw)), (dt * cos(yaw)), 0.0, 1.0,
      (dt * v * cos(yaw)), (dt * sin(yaw)), 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
      1.0;

    return jF;
  }

  MatrixXf observation_model(MatrixXf x)
  {
    Matrix<float, 2, 1> z;

    z = m_H * x;

    return z;
  }

  std::tuple<MatrixXf, MatrixXf> ekf_estimation(MatrixXf xEst,
                                                MatrixXf PEst,
                                                MatrixXf z,
                                                MatrixXf u)
  {
    // Predict
    Matrix<float, 4, 1> xPred;
    xPred = state_model(xEst, u);

    // state vector
    Matrix<float, 4, 4> jF;
    jF = jacob_f(xEst, u);

    Matrix<float, 4, 4> PPred;
    PPred = (jF * PEst * jF.transpose()) + m_Q;

    // Update
    Matrix<float, 2, 1> zPred;
    zPred = observation_model(xPred);

    Matrix<float, 2, 1> y;
    y = z - zPred; // measurement residual

    Matrix<float, 2, 2> S;
    S = (m_H * PPred * m_H.transpose()) + m_R; // Innovation Covariance

    Matrix<float, 4, 2> K;
    K = PPred * m_H.transpose() * S.inverse(); // Kalman Gain

    xEst = xPred + K * y; // update step

    PEst = (MatrixXf::Identity(4, 4) - (K * m_H)) * PPred;

    return std::make_tuple(xEst, PEst);
  }

  float complementary(float IMU_vel, float EC_vel)
  {
    // accelerometer wt.
    float alpha = 0.4;

    // complementary velocity
    float compl_vel = (alpha * IMU_vel) + (1 - alpha) * (compl_vel + IMU_vel);

    return compl_vel;
  }
};
