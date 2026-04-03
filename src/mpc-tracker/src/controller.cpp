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
double angle_wrap(double alpha){
  while (alpha <= -M_PI)
  {
    alpha = alpha + 2 * M_PI;
  }
  while (alpha > M_PI)
  {
    alpha = alpha - 2 * M_PI;
  }
  return alpha;
}

// Calculate the stacked A matrices, using the reference trajectory data
Eigen::MatrixXd calc_A_stacked(const std::vector<double>& theta_ref, const std::vector<double>& v_ref, double delta_t, int N) {
  // Calculate A matrices
  std::vector<Eigen::Matrix<double,3,3>> A(N);
  for (int k = 0; k < N; ++k) {
    A[k] << 1, 0, -v_ref[k] * sin(theta_ref[k]) * delta_t,
            0, 1, v_ref[k] * cos(theta_ref[k]) * delta_t,
            0, 0, 1;
  }

  // A_stacked is formed from the stacked Jacobi matrices
  Eigen::MatrixXd A_stacked(3*N, 3);
  for (int i = 0; i < N; ++i) {
    int offset = i * 3;
    Eigen::Matrix<double,3,3> Interim = Eigen::Matrix<double,3,3>::Identity();
    for (int j = 0; j <= i; ++j) {
      Interim = A[j] * Interim;
    }
    A_stacked.block(offset, 0, 3, 3) = Interim;
  }
  return A_stacked;
}

// Calculate the stacked B matrices, using the reference trajectory data
Eigen::MatrixXd calc_B_stacked(const std::vector<double>& theta_ref, const std::vector<double>& v_ref, double delta_t, int N) {
  // Calculate A matrices
  std::vector<Eigen::Matrix<double,3,3>> A(N);
  for (int k = 0; k < N; ++k) {
    A[k] << 1, 0, -v_ref[k] * sin(theta_ref[k]) * delta_t,
            0, 1, v_ref[k] * cos(theta_ref[k]) * delta_t,
            0, 0, 1;
  }

  // Calculate B matrices
  std::vector<Eigen::Matrix<double,3,2>> B(N);
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
        Interim = A[k] * Interim;
      }
      B_stacked.block(offset_x, offset_y, 3, 2) = Interim;
    }
  }
  return B_stacked;
}

// Calculate the block diagonal Q_bar matrix (last entry: P)
Eigen::MatrixXd calc_Q_bar(const Eigen::MatrixXd& Q, const Eigen::MatrixXd& P, int N) {
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
Eigen::MatrixXd calc_R_bar(const Eigen::MatrixXd& R, int N) {
  // R_bar is block-diagonal: [R, R, ..., R]
  Eigen::MatrixXd R_bar = Eigen::MatrixXd::Zero(2*N, 2*N);
  for (int i = 0; i < N; ++i) {
    int offset = i * 2;
    R_bar.block(offset, offset, 2, 2) = R;
  }
  return R_bar;
}

// Calculate the Hessian matrix H, used by OSQP
Eigen::MatrixXd calc_H(const Eigen::MatrixXd& Q_bar, const Eigen::MatrixXd& R_bar, const Eigen::MatrixXd& B_stacked) {
  // Calculate H
  Eigen::MatrixXd H = 2 * (B_stacked.transpose() * Q_bar * B_stacked + R_bar);
  return H;
}

// Calculate f, used by OSQP
Eigen::MatrixXd calc_f(const Eigen::MatrixXd& Q_bar, const Eigen::MatrixXd& A_stacked, const Eigen::MatrixXd& B_stacked, const Eigen::MatrixXd& delta_X_0) {
  // Calculate f
  Eigen::MatrixXd f = 2 * B_stacked.transpose() * Q_bar * A_stacked * delta_X_0;
  return f;
}

// Calculate l and u, used as OSQP constraints
Eigen::MatrixXd calc_l(const std::vector<double>& v_ref, const std::vector<double>& w_ref, int N) {
  Eigen::MatrixXd l(2*N,1);
  for (int i = 0; i <= N-1; ++i) {
    int offset = i * 2;
    l.block(offset, 0, 2, 1) << -0.22 - v_ref[i],
                                -2.84 - w_ref[i];
  }
  return l;
}
Eigen::MatrixXd calc_u(const std::vector<double>& v_ref, const std::vector<double>& w_ref, int N) {
  Eigen::MatrixXd u(2*N,1);
  for (int i = 0; i <= N-1; ++i) {
    int offset = i * 2;
    u.block(offset, 0, 2, 1) << 0.22 - v_ref[i],
                                2.84 - w_ref[i];
  }
  return u;
}

// Path-Tracking Node
class MPCNode : public rclcpp::Node {
public:
  MPCNode() : rclcpp::Node("mpc_node") {
    // Initialize member variables
    state_received = false;
    delta_X_0 = Eigen::Vector3d::Zero();
      // Tuning Parameters
    delta_t = 0.05; // time step, in s
    N = 10; // prediction horizon size
    Q = Eigen::MatrixXd::Identity(3,3);
    P = Q;
    R = Eigen::MatrixXd::Identity(2,2);

    // Calculate Q_bar and R_bar from tuning matrices Q, P, R
    Q_bar = calc_Q_bar(Q, P, N);
    R_bar = calc_R_bar(R, N);

    // Subscriber to reference trajectory

    // Subscriber to current pose of TurtleBot
    auto odom_callback = [this](const nav_msgs::msg::Odometry & msg){
      this->x = msg.pose.pose.position.x;
      this->y = msg.pose.pose.position.y;
      double qx = msg.pose.pose.orientation.x;
      double qy = msg.pose.pose.orientation.y;
      double qz = msg.pose.pose.orientation.z;
      double qw = msg.pose.pose.orientation.w;
      this->theta = angle_wrap(::atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))); // Conversion, quaternion to radian
      this->state_received = true;
    };
    sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10, odom_callback);
    
    // Create Publisher for the TurtleBot3 movement topic
    pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", 10);
    const auto timer_period = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::duration<double>(delta_t * 1000));
    timer_ = this->create_wall_timer(timer_period, [this]() {
      if (!state_received) return;
      // Main MPC loop

      // Calculate A_stacked and B_stacked from reference trajectory data
      Eigen::MatrixXd A_stacked = calc_A_stacked(theta_ref, v_ref, delta_t, N);
      Eigen::MatrixXd B_stacked = calc_B_stacked(theta_ref, v_ref, delta_t, N);

      // Calculate delta_X_0 from current state and reference
      delta_X_0 <<  x - x_ref[0], 
                    y - y_ref[0], 
                    angle_wrap(theta - theta_ref[0]);

      // Calculate constraint vectors, for the form l <= A Delta_U <= u
      Eigen::MatrixXd l = calc_l(v_ref, w_ref, N);
      Eigen::MatrixXd u = calc_u(v_ref, w_ref, N);
      Eigen::MatrixXd A = Eigen::MatrixXd::Identity(N*2, N*2);

      // Calculate H and f from resulting matrices
      Eigen::MatrixXd H = calc_H(Q_bar, R_bar, B_stacked);
      Eigen::MatrixXd f = calc_f(Q_bar, A_stacked, B_stacked, delta_X_0);

      // OSQP solver
      
      auto message = geometry_msgs::msg::TwistStamped();
      message.twist.linear.x = 0.0;
      message.twist.angular.z = 0.0;
      RCLCPP_INFO(this->get_logger(), "Publishing: linear.x=%.2f angular.z=%.2f",
                  message.twist.linear.x, message.twist.angular.z);
      pub_->publish(message);
    });
  }

private:
    // Safety flag
    bool state_received;
    // Current state
    double x, y, theta;
    // Reference State and Input
    std::vector<double> x_ref, y_ref, theta_ref;
    std::vector<double> w_ref, v_ref;
    // MPC Parameters
    Eigen::MatrixXd Q_bar, R_bar;
    Eigen::Vector3d delta_X_0;
    double delta_t; // time step, in s
    int N; // prediction horizon size
    Eigen::MatrixXd Q, P, R; // State, Terminal and Input Cost

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