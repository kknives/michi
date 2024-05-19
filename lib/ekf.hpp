#include <Eigen/Dense>
#include <chrono>
#include <tuple>
#include <cmath>
#include <stdlib.h>

using Eigen::Matrix;
using Eigen::MatrixXf;

const float DEG_TO_RAD = 0.01745329251;
const float DT = 0.01;

class EKF
{
  // time-step
  const float m_DT = 0.1;
public:
  // Covariance Matrix
  Matrix<float, 4, 4> m_predicted_noise_cov;
  Matrix<float, 2, 2> m_measurement_noise_cov;

  // ip noise
  Matrix<float, 2, 2> m_ip_noise;

  // measurement matrix
  Matrix<float, 2, 4> m_H;


  EKF() : m_predicted_noise_cov { 0.1,  0.0,      0.0,         0.0,
                0.0,  0.1,      0.0,         0.0,
                0.0,  0.0, (1 * DEG_TO_RAD), 0.0,
                0.0,  0.0,      0.0,         1.0,},

              m_measurement_noise_cov { 0.1, 0,
                     0, 0.1, },
              m_ip_noise { 1.0,                 0.0,
                         0.0, (30*DEG_TO_RAD) },
              m_H { 1, 0, 0, 0,
                    0, 1, 0, 0},

  {
  }


  MatrixXf control_input(Eigen::Vector3f linear_accel,Eigen::Vector3f angular_vel,Eigen::Vector3f position)
  {
     Matrix<float, 2, 1> u;
     float vel = 0.0, imu_vel = 0.0, odom_vel = 0.0, yaw_vel = 0.0, prev_pos, current_pos, dS; 
     
     current_pos = position.norm();

     //calculating small change in distance
     dS = current_pos - prev_pos; 

     imu_vel = sqrt(pow(linear_accel.x(),2) + pow(linear_accel.y(),2))*DT;
     odom_vel = dS/DT;
     yaw_vel = angular_vel(2);

     vel = complementary(imu_vel, odom_vel);

     u << vel, yaw_vel;

     prev_pos = current_pos;

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
    Matrix<float, 4, 4> A {{1, 0, 0, 0},
                           {0, 1, 0, 0},
                           {0, 0, 1, 0},
                  			   {0, 0, 0, 0}};

    Matrix<float, 4, 2> B;
        B << (DT*cos(x.coeff(2,0))), 0,
             (DT*sin(x.coeff(2,0))), 0,
    			      0, DT,
    		              1, 0;
    x = (A * x) + (B * u);

    return x;
  }

  MatrixXf jacob_f(MatrixXf x, MatrixXf u)
  {
    float yaw = x.coeff(2, 0);

    float v = u.coeff(0, 0);

    Matrix<float, 4, 4> jF;
    jF << 1.0, 0.0, (-m_DT * v * sin(yaw)), (m_DT * cos(yaw)), 0.0, 1.0,
      (m_DT * v * cos(yaw)), (m_DT * sin(yaw)), 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
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
    PPred = (jF * PEst * jF.transpose()) + m_predicted_noise_cov;

    // Update
    Matrix<float, 2, 1> zPred;
    zPred = observation_model(xPred);

    Matrix<float, 2, 1> y;
    y = z - zPred; // measurement residual

    Matrix<float, 2, 2> S;
    S = (m_H * PPred * m_H.transpose()) + m_measurement_noise_cov; // Innovation Covariance

    Matrix<float, 4, 2> K;
    K = PPred * m_H.transpose() * S.inverse(); // Kalman Gain

    xEst = xPred + K * y; // update step

    PEst = (MatrixXf::Identity(4, 4) - (K * m_H)) * PPred;

    return std::make_tuple(xEst, PEst);
  }

  float complementary(float IMU_vel, float ODOM_vel)
  {
    // accelerometer wt.
    float alpha = 0.4;

    // complementary velocity
    float compl_vel = (alpha * IMU_vel) + (1 - alpha) * (compl_vel + ODOM_vel);

    return compl_vel;
  }
 
  std::tuple<MatrixXf, MatrixXf> run_ekf(MatrixXf control_ip)
  {
    // state vector
    Matrix<float, 4, 1> xEst = MatrixXf::Zero(4, 1);
    Matrix<float, 4, 1> xTrue = MatrixXf::Zero(4, 1);
  
    // Predicted Covariance
    Matrix<float, 4, 4> PEst = MatrixXf::Identity(4, 4);
  
    // control input
    Matrix<float, 2, 1> u;
    Matrix<float, 2, 1> ud = MatrixXf::Zero(2, 1);
  
    // observation vector
    Matrix<float, 2, 1> z = MatrixXf::Zero(2, 1);
  
    while (true) {
  
      u = control_ip;
  
      std::tie(xTrue, ud) = obj.observation(xTrue, u);
  
      z = obj.observation_model(xTrue);
  
      std::tie(xEst, PEst) = obj.ekf_estimation(xEst, PEst, z, ud);
    
      return std::make_tuple(xEst, PEst);
    }  
  }
};
