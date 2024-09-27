#include <Eigen/Dense>
#include <chrono>
#include <cmath>
#include <gz/msgs.hh>
#include <gz/transport/Node.hh>
#include <iostream>
#include <stack>
#include <stdlib.h>
#include <thread>
#include <tuple>

using namespace std;
using namespace Eigen;

const float deg_to_rad = 0.01745329251;
const float rad_to_deg = 57.296;
const float dt = 0.01;

class EKF {
public:
  // Covariance Matrix
  Matrix<float, 4, 4> Q;
  Matrix<float, 2, 2> R;

  // ip noise
  Matrix<float, 2, 2> ip_noise;

  // measurement matrix
  Matrix<float, 2, 4> H;

  // acceleration & gyro variables
  Eigen::Vector3f gyro_msg;
  Eigen::Vector3f accel_msg;
  Eigen::Vector3f odom_msg;
  float prev_distance;
  float current_distance;
  float dS;
  float imu_vel;
  float yaw;

  EKF() {

    // Covariance Matrix
    Q << 0.1, 0.0, 0.0, 0.0, 
         0.0, 0.1, 0.0, 0.0, 
	 0.0, 0.0, (1 * deg_to_rad), 0.0, 
	 0.0, 0.0, 0.0, 1.0;

    // Measurement Noise Covaraince
    R << 0.1, 0.0, 
         0.0, 0.1;

    // input noise
    ip_noise << 1.0, 0.0, 
	        0.0, (30 * deg_to_rad);

    // measurement matrix
    H << 1, 0, 0, 0, 
         0, 1, 0, 0;

    // velocity from imu 
    imu_vel = 0.0;
    
    // distance 
    prev_distance = 0.0;
    current_distance = 0.0;
    dS = 0.0;

    //angle 
    yaw = 0.0;
  }

  tuple<MatrixXf, MatrixXf> observation(MatrixXf xTrue, MatrixXf u) {
    xTrue = state_model(xTrue, u);

    Matrix<float, 2, 1> ud;
    ud = u + (ip_noise * MatrixXf::Random(2, 1));

    return make_tuple(xTrue, ud);
  }

  MatrixXf state_model(MatrixXf x, MatrixXf u) {

    Matrix<float, 4, 4> A;
    A << 1, 0, 0, 0, 
         0, 1, 0, 0, 
	 0, 0, 1, 0, 
	 0, 0, 0, 0;

    Matrix<float, 4, 2> B;
    B << (dt * cos(yaw)), 0, 
         (dt * sin(yaw)), 0, 
	                        0, dt, 
				 1, 0;

    x = (A * x) + (B * u);

    return x;
  }

  MatrixXf jacob_f(MatrixXf x, MatrixXf u) {
    float yaw = x.coeff(2, 0);

    float v = u.coeff(0, 0);

    Matrix<float, 4, 4> jF;
    jF << 1.0, 0.0, (-dt * v * sin(yaw)), (dt * cos(yaw)), 
          0.0, 1.0, (dt * v * cos(yaw)), (dt * sin(yaw)), 
	  0.0, 0.0, 1.0, 0.0, 
	  0.0, 0.0, 0.0, 1.0;

    return jF;
  }

  MatrixXf observation_model(MatrixXf x) {
    Matrix<float, 2, 1> z;

    z = H * x;

    return z;
  }

  tuple<MatrixXf, MatrixXf> ekf_estimation(MatrixXf xEst, MatrixXf PEst,
                                           MatrixXf z, MatrixXf u) {
    // Predict
    Matrix<float, 4, 1> xPred;
    xPred = state_model(xEst, u);

    // state vector
    Matrix<float, 4, 4> jF;
    jF = jacob_f(xEst, u);

    Matrix<float, 4, 4> PPred;
    PPred = (jF * PEst * jF.transpose()) + Q;

    // Update
    Matrix<float, 2, 1> zPred;
    zPred = observation_model(xPred);

    Matrix<float, 2, 1> y;
    y = z - zPred; // measurement residual

    Matrix<float, 2, 2> S;
    S = (H * PPred * H.transpose()) + R; // Innovation Covariance

    Matrix<float, 4, 2> K;
    K = PPred * H.transpose() * S.inverse(); // Kalman Gain

    xEst = xPred + K * y; // update step

    PEst = (MatrixXf::Identity(4, 4) - (K * H)) * PPred;

    return make_tuple(xEst, PEst);
  }

  float complementary(float IMU_vel, float EC_vel) {

    float compl_vel;

    // accelerometer wt.
    float alpha = 0.4;

    // complementary velocity
    compl_vel = (alpha * IMU_vel) + (1 - alpha) * (EC_vel);

    return compl_vel;
  }

  void IMU_cb(const gz::msgs::IMU &imu) {

    float vel_x, vel_y;

    accel_msg << imu.linear_acceleration().x(), (imu.linear_acceleration().y()),
        imu.linear_acceleration().z();

    gyro_msg << imu.angular_velocity().x(), (imu.angular_velocity().y()),
        imu.angular_velocity().z();
    
    vel_x = vel_x + accel_msg(0)*dt;

    vel_y = vel_y + accel_msg(1)*dt;

    imu_vel = sqrt(pow(vel_x,2) + pow(vel_y,2));
  }

  void Odometry_cb(const gz::msgs::Odometry &odom) {

    odom_msg << odom.pose().position().x(), odom.pose().position().y(),
        odom.pose().position().z();

    current_distance = odom_msg.norm(); 
    
    // small change in distance 
    dS = current_distance - prev_distance; 

    prev_distance = current_distance;
  }
};

int main() {

  EKF obj;

  gz::transport::Node node;
  std::string topic_imu =
      "/world/default/model/rover/link/base_link/sensor/imu_sensor/imu";
  std::string topic_odom = "/model/rover/odometry";

  // Subscribe to a topic by registering a callback.

  if (node.Subscribe(topic_imu, &EKF::IMU_cb, &obj)) {
    std::cerr << "Subscribing to topic [" << topic_imu << "]" << endl;
  }

  if (node.Subscribe(topic_odom, &EKF::Odometry_cb, &obj)) {
    std::cerr << "Subscribing to topic [" << topic_odom << "]" << endl;
  }

  // velocity variables
  float odom_vel = 0.0, vel = 0.0, yaw_vel = 0.0;

  bool print_to_cout = true;

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

    //std::this_thread::sleep_for(std::chrono::milliseconds(90));

    // calculating Encoder veclotiy
    odom_vel = obj.dS/dt;

    vel = obj.complementary(obj.imu_vel, odom_vel);

    // calculating yaw velocity in rad/s 
    yaw_vel = obj.gyro_msg.z();

    // control input
    u << vel, yaw_vel;

    // calculating yaw angle 
    obj.yaw = obj.yaw + yaw_vel*dt; 

    tie(xTrue, ud) = obj.observation(xTrue, u);

    z = obj.observation_model(xTrue);

    tie(xEst, PEst) = obj.ekf_estimation(xEst, PEst, z, ud);

    // visualisation
    if (print_to_cout) {

      // synchronising with python visualisation
     std::this_thread::sleep_for(std::chrono::milliseconds(10));

      // Estimation + True
      cout << xEst.transpose() << endl
	   << "Accel : " << obj.accel_msg.transpose() << endl 
	   << "Odometry : " << obj.odom_msg.transpose() << endl 
	   << "Control Input :" << u.transpose() << endl 
	   << "odom_vel : " << odom_vel << " imu_vel : " << obj.imu_vel << endl;
    }
    
  }

  return 0;
}
