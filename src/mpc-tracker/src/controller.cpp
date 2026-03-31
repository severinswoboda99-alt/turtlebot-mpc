#include <chrono>
#include <memory>
#include <string>
#include <cmath>
#include <vector>
#include <iostream>
#include <Eigen/Dense>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

using namespace std::chrono_literals;

// Function to guarantee the angles are between pi and -pi
// double angle_wrap(double alpha){
//   while (alpha <= -M_PI)
//   {
//     alpha = alpha + 2 * M_PI;
//   }
//   while (alpha > M_PI)
//   {
//     alpha = alpha - 2 * M_PI;
//   }
//   return alpha;
// }

// Calculate the stacked A matrices, using the reference trajectory data
Eigen::MatrixXd calc_A_stacked(double theta_ref[], double v_ref[], double delta_t, int N) {
  // Calculate A matrices
  Eigen::Matrix<double,3,3> A[N];
  for (int k = 0; k < N; ++k) {
    A[k] << 1, 0, -v_ref[k] * sin(theta_ref[k]) * delta_t,
            0, 1, v_ref[k] * cos(theta_ref[k]) * delta_t,
            0, 0, 1;
  }

  // A_stacked is formed from the stacked Jacobi matrices
  Eigen::MatrixXd A_stacked;
  for (int i = 0; i < N; ++i) {
    int offset = i * 3;
    Eigen::Matrix<double,3,3> Interim = Eigen::MatrixXd::Identity(3,3);
    for (int j = 0; j <= i; ++j) {
      Interim = A[j] * Interim;
    }
    A_stacked.block(offset, 0, 3, 3) = Interim;
  }
  return A_stacked;
}

// Calculate the stacked B matrices, using the reference trajectory data
Eigen::MatrixXd calc_B_stacked(double theta_ref[], double v_ref[], double delta_t, int N) {
  // Calculate B matrices
  Eigen::Matrix<double,3,2> B[N];
  for (int k = 0; k < N; ++k) {
    B[k] << cos(theta_ref[k]) * delta_t, 0,
            sin(theta_ref[k]) * delta_t, 0,
            0, delta_t;
  }

  // B_stacked is a lower diagonal matrix
  Eigen::MatrixXd B_stacked = Eigen::MatrixXd::Zero(3*N, 2*N);
  for (int i = 0; i < N; ++i) {
    for (int j = 0; j <= i; ++j) {
      int offset_x = i * 3;
      int offset_y = j * 2;
      Eigen::Matrix<double,3,2> Interim = B[j];
      for (int k = j + 1; k <= i; ++k) {
        Interim = B[k] * Interim;
      }
      B_stacked.block(offset_x, offset_y, 3, 2) = Interim;
    }
  }
  return B_stacked;
}

// Calculate the block diagonal Q_bar matrix (last entry: P)
Eigen::MatrixXd calc_Q_bar(Eigen::MatrixXd Q, Eigen::MatrixXd P, int N) {
  // Q_bar is block-diagonal: [Q, Q, ..., P]
  Eigen::MatrixXd Q_bar = Eigen::MatrixXd::Zero(3*N, 3*N);
  for (int i = 0; i < N; ++i) {
    int offset = i * 3;
    if (i == N - 1) {
      // Last block is P
      Q_bar.block(offset, offset, 3, 3) = P;
    } else {
      // All other blocks are Q
      Q_bar.block(offset, offset, 3, 3) = Q;
    }
  }
  return Q_bar;
}

// Calculate the block diagonal R_bar matrix
Eigen::MatrixXd calc_R_bar(Eigen::MatrixXd R, int N) {
  // R_bar is block-diagonal: [R, R, ..., R]
  Eigen::MatrixXd R_bar = Eigen::MatrixXd::Zero(2*N, 2*N);
  for (int i = 0; i < N; ++i) {
    int offset = i * 2;
    R_bar.block(offset, offset, 2, 2) = R;
  }
  return R_bar;
}

// Calculate the Hessian matrix H, used by OSQP
Eigen::MatrixXd calc_H(Eigen::MatrixXd Q_bar, Eigen::MatrixXd R_bar, Eigen::MatrixXd B_stacked) {
  // Transpose B_stacked
  Eigen::MatrixXd B_stacked_T = B_stacked.transpose();
  // Calculate H
  Eigen::MatrixXd H = 2 * (B_stacked_T * Q_bar * B_stacked + R_bar);
  return H;
}

// Calculate f, used by OSQP
Eigen::MatrixXd calc_f(Eigen::MatrixXd Q_bar, Eigen::MatrixXd A_stacked, Eigen::MatrixXd B_stacked, Eigen::MatrixXd delta_X_0) {
  // Transpose B_stacked
  Eigen::MatrixXd B_stacked_T = B_stacked.transpose();
  // Calculate f
  Eigen::MatrixXd f = 2 * B_stacked_T * Q_bar * A_stacked * delta_X_0;
  return f;
}

// Calculate l and u, used as OSQP constraints
Eigen::MatrixXd calc_l(double v_ref[], double w_ref[], int N) {
  Eigen::MatrixXd l(2*N,1);
  for (int i = 0; i <= i; ++i) {
    int offset = i * 2;
    l.block(offset, 0, 2, 1) << -0.22 - v_ref[i],
                                -2.84 - w_ref[i];
  }
}
Eigen::MatrixXd calc_u(double v_ref[], double w_ref[], int N) {
  Eigen::MatrixXd u(2*N,1);
  for (int i = 0; i <= i; ++i) {
    int offset = i * 2;
    u.block(offset, 0, 2, 1) << 0.22 - v_ref[i],
                                2.84 - w_ref[i];
  }
}

double calc_placeholder(){
  double r = 0;
  return r;
}

class MPCNode : public rclcpp::Node {
public:
  MPCNode() : rclcpp::Node("mpc_node") {

    // Subscriber to reference trajectory

    // Calculate A_stacked and B_stacked from reference trajectory data
    Eigen::MatrixXd A_stacked = calc_A_stacked(this->theta_ref[], this->v_ref[], this->delta_t, this->N);
    Eigen::MatrixXd B_stacked = calc_B_stacked(this->theta_ref[], this->v_ref[], this->delta_t, this->N);

    // Calculate Q_bar and R_bar from tuning matrices Q, P, R
    Eigen::MatrixXd Q_bar = calc_Q_bar(this->Q, this->P, this->N);
    Eigen::MatrixXd R_bar = calc_R_bar(this->R, this->N);

    // Subscriber to current pose of TurtleBot
    auto odom_callback = [this](const nav_msgs::msg::Odometry & msg){
      this->x = msg.pose.pose.position.x;
      this->y = msg.pose.pose.position.y;
      double qx = msg.pose.pose.orientation.x;
      double qy = msg.pose.pose.orientation.y;
      double qz = msg.pose.pose.orientation.z;
      double qw = msg.pose.pose.orientation.w;
      this->theta = std::atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz)); // Conversion, quaternion to radian
    };
    sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10, odom_callback);
    
    // Calculate delta_X_0 from current state and reference
    this->delta_X_0 <<  this->x - this->x_ref[0], 
                        this->y - this->y_ref[0], 
                        this->theta - this->theta_ref[0];
    
    // Calculate constraint vectors, for the form l <= A Delta_U <= u
    Eigen::MatrixXd l = calc_l(this->v_ref[], this->w_ref[], this->N);
    Eigen::MatrixXd u = calc_u(this->v_ref[], this->w_ref[], this->N);
    Eigen::MatrixXd A = Eigen::Matrix::Identity(this->N*2,this->N*2);

    // Calculate H and f from resulting matrices
    Eigen::MatrixXd H = calc_H(this->Q_bar, this->R_bar, this->B_stacked);
    Eigen::MatrixXd f = calc_f(this->Q_bar, this->A_stacked, this->B_stacked, this->delta_X_0);

    // OSQP solver

    // Create Publisher for the TurtleBot3 movement topic
    pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", 10);

    timer_ = this->create_wall_timer(std::chrono::milliseconds(100), [this]() {
      auto r = calc_placeholder();
      auto message = geometry_msgs::msg::TwistStamped();
      message.twist.linear.x = r;
      message.twist.angular.z = r;
      RCLCPP_INFO(this->get_logger(), "Publishing: linear.x=%.2f angular.z=%.2f",
                  message.twist.linear.x, message.twist.angular.z);
      pub_->publish(message);
    });
  }

private:
    double x, y, theta;
    double x_ref[], y_ref[], theta_ref[];
    double v_ref[], w_ref[];
    double delta_t; // in s
    int N; // prediction horizon size
    Eigen::MatrixXd Q, P, R;
    Eigen::MatrixXd delta_X_0;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

// Main function to run the node
int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MPCNode>());
  rclcpp::shutdown();
  return 0;
}