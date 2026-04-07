#include <chrono>
#include <algorithm>
#include <memory>
#include <string>
#include <cmath>
#include <limits>
#include <vector>
#include <iostream>
#include <Eigen/Dense>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
extern "C" {
#include <osqp.h>
}
#include <OsqpEigen/OsqpEigen.h>

using namespace std::chrono_literals;

// Function to guarantee the angles are between pi and -pi
double angle_wrap(double alpha){
  while (alpha <= -M_PI)
  {
    alpha = alpha + 2 * M_PI;
  }
  while (alpha > M_PI) {
    alpha = alpha - 2 * M_PI;
  }
  return alpha;
}

// Function to find the index of the currently closest point
int closest_index(double x, double y, std::vector<double> x_ref, std::vector<double> y_ref) {
  double min_dist = std::numeric_limits<double>::max();
  int index = 0;

  for (int i = 0; i < (int)x_ref.size(); ++i) {
    double dx = x - x_ref[i];
    double dy = y - y_ref[i];
    double dist = dx*dx + dy*dy;

    if (dist < min_dist) {
      min_dist = dist;
      index = i;
    }
  }
  return index;
}

// Calculate the stacked A matrices, using the local horizon trajectory data
Eigen::MatrixXd calc_A_stacked(const std::vector<double>& theta_local, const std::vector<double>& v_local, double delta_t, int N) {
  // Calculate A matrices
  std::vector<Eigen::Matrix<double,3,3>> A(N);
  for (int k = 0; k < N; ++k) {
    A[k] << 1, 0, -v_local[k] * sin(theta_local[k]) * delta_t,
            0, 1, v_local[k] * cos(theta_local[k]) * delta_t,
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

// Calculate the stacked B matrices, using the local horizon trajectory data
Eigen::MatrixXd calc_B_stacked(const std::vector<double>& theta_local, const std::vector<double>& v_local, double delta_t, int N) {
  // Calculate A matrices
  std::vector<Eigen::Matrix<double,3,3>> A(N);
  for (int k = 0; k < N; ++k) {
    A[k] << 1, 0, -v_local[k] * sin(theta_local[k]) * delta_t,
            0, 1, v_local[k] * cos(theta_local[k]) * delta_t,
            0, 0, 1;
  }

  // Calculate B matrices
  std::vector<Eigen::Matrix<double,3,2>> B(N);
  for (int k = 0; k < N; ++k) {
    B[k] << cos(theta_local[k]) * delta_t, 0,
            sin(theta_local[k]) * delta_t, 0,
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
Eigen::VectorXd calc_f(const Eigen::MatrixXd& Q_bar, const Eigen::MatrixXd& A_stacked, const Eigen::MatrixXd& B_stacked, const Eigen::MatrixXd& delta_X_0) {
  // Calculate f
  Eigen::VectorXd f = 2 * B_stacked.transpose() * Q_bar * A_stacked * delta_X_0;
  return f;
}

// Calculate l and u, used as OSQP constraints
Eigen::VectorXd calc_l(const std::vector<double>& v_local, const std::vector<double>& w_local, int N) {
  Eigen::VectorXd l(2*N,1);
  for (int i = 0; i <= N-1; ++i) {
    int offset = i * 2;
    l.segment(offset, 2) << -0.22 - v_local[i],
                                -2.84 - w_local[i];
  }
  return l;
}
Eigen::VectorXd calc_u(const std::vector<double>& v_local, const std::vector<double>& w_local, int N) {
  Eigen::VectorXd u(2*N,1);
  for (int i = 0; i <= N-1; ++i) {
    int offset = i * 2;
    u.segment(offset, 2) << 0.22 - v_local[i],
                                2.84 - w_local[i];
  }
  return u;
}

// Path-Tracking Node
class MPCNode : public rclcpp::Node {
public:
  MPCNode() : rclcpp::Node("mpc_node") {
    // Dynamic Reconfigue
    this->declare_parameter<bool>("start", false);
    start = this->get_parameter("start").as_bool();
    param_cb_handle_ = this->add_on_set_parameters_callback(
      std::bind(&MPCNode::on_param_set, this, std::placeholders::_1));

    // Initialize member variables
    state_received = false;
    path_received = false;
    solver_initialized = false;
    delta_X_0 = Eigen::Vector3d::Zero();

      // Tuning Parameters
    delta_t = 0.1; // time step, in s
    N = 10; // prediction horizon size
    Q = 10 * Eigen::MatrixXd::Identity(3,3);
    P = Q;
    R = 0.1 * Eigen::MatrixXd::Identity(2,2);

      // OSQP:Eigen Initialization
    solver.settings()->setVerbosity(false);
    solver.settings()->setWarmStart(true);
    solver.data()->setNumberOfVariables(2*N);
    solver.data()->setNumberOfConstraints(2*N);
      // Constraint matrix A stays constant
    A_sparse = Eigen::MatrixXd::Identity(2*N, 2*N).sparseView();

    // Calculate Q_bar and R_bar from tuning matrices Q, P, R
    Q_bar = calc_Q_bar(Q, P, N);
    R_bar = calc_R_bar(R, N);

    // Subscriber to reference trajectory
    auto ref_callback = [this](const nav_msgs::msg::Path & path){
      // Clear the vectors, should the path change
      this->x_ref.clear();
      this->y_ref.clear();
      this->theta_ref.clear();
      this->v_ref.clear();
      this->w_ref.clear();

      // Keep the full reference path so the controller can look ahead.
      const int size = static_cast<int>(path.poses.size());
      this->x_ref.reserve(size);
      this->y_ref.reserve(size);
      this->theta_ref.reserve(size);

      // Populate pose vectors
      for (int i = 0; i < size; ++i) {
        const auto & pose = path.poses[i].pose;
        this->x_ref.push_back(pose.position.x);
        this->y_ref.push_back(pose.position.y);

        const double qx = pose.orientation.x;
        const double qy = pose.orientation.y;
        const double qz = pose.orientation.z;
        const double qw = pose.orientation.w;
        this->theta_ref.push_back(
          angle_wrap(::atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz)))); // Conversion, quaternion to radian
      }

      // Populate input vectors
      for (int i = 0; i < size - 1; ++i) {
        double dx = x_ref[i+1] - x_ref[i];
        double dy = y_ref[i+1] - y_ref[i];

        double v = std::sqrt(dx*dx + dy*dy) / delta_t;
        double w = angle_wrap(theta_ref[i+1] - theta_ref[i]) / delta_t;

        v_ref.push_back(v);
        w_ref.push_back(w);
      }

      this->path_received = true;
    };
    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
      "/path", 10, ref_callback);

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
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10, odom_callback);
    
    // Create Publisher for the TurtleBot3 movement topic
    pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", 10);
    auto timer_period = std::chrono::milliseconds(static_cast<int>(delta_t * 1000));
    timer_ = this->create_wall_timer(timer_period, [this]() {
      // Only drive the robot when the state and reference path have been received, and the start signal has been given
      if (!state_received || !path_received || !start) {
        auto message = geometry_msgs::msg::TwistStamped();
        message.twist.linear.x = 0.0;
        message.twist.angular.z = 0.0;
        RCLCPP_INFO(this->get_logger(), "Stopped: linear.x=%.2f angular.z=%.2f",
                  message.twist.linear.x, message.twist.angular.z);
        pub_->publish(message);
        return;
      }

      // Main MPC loop
      // Create Local Horizon, up to N steps, starting from the nearest reference point
      int i_c = closest_index(x, y, x_ref, y_ref);
      int remaining_points = static_cast<int>(x_ref.size()) - i_c;
        // If no points remain, stop the robot
      if (remaining_points <= 0) {
        auto stop_msg = geometry_msgs::msg::TwistStamped();
        stop_msg.twist.linear.x = 0.0;
        stop_msg.twist.angular.z = 0.0;
        pub_->publish(stop_msg);
        return;
      }
        // Populate local horizon vectors with data from the path planner
      std::vector<double> x_local, y_local, theta_local, v_local, w_local;
      for (int i = 0; i < N; ++i) {
        x_local.push_back(x_ref[i_c + i]);
        y_local.push_back(y_ref[i_c + i]);
        theta_local.push_back(theta_ref[i_c + i]);
        v_local.push_back(v_ref[i_c + i]);
        w_local.push_back(w_ref[i_c + i]);
      }

      // Calculate A_stacked and B_stacked from local horizon trajectory data
      Eigen::MatrixXd A_stacked = calc_A_stacked(theta_local, v_local, delta_t, N);
      Eigen::MatrixXd B_stacked = calc_B_stacked(theta_local, v_local, delta_t, N);

      // Calculate delta_X_0 from current state and local horizon
      delta_X_0 <<  x - x_local[0], 
                    y - y_local[0], 
                    angle_wrap(theta - theta_local[0]);

      // Calculate constraint vectors, for the form l <= A Delta_U <= u
      Eigen::VectorXd l = calc_l(v_local, w_local, N);
      Eigen::VectorXd u = calc_u(v_local, w_local, N);

      // Calculate H and f from resulting matrices
      Eigen::MatrixXd H = calc_H(Q_bar, R_bar, B_stacked);
      Eigen::VectorXd f = calc_f(Q_bar, A_stacked, B_stacked, delta_X_0);

      // OSQP, convert to sparse matrices
      Eigen::SparseMatrix<double> H_sparse = H.sparseView();

      // OSQP, initialize once; update only afterwards.
      if (!solver_initialized) {
        if (!solver.data()->setHessianMatrix(H_sparse) ||
            !solver.data()->setGradient(f) ||
            !solver.data()->setLinearConstraintsMatrix(A_sparse) ||
            !solver.data()->setLowerBound(l) ||
            !solver.data()->setUpperBound(u) ||
            !solver.initSolver()) {
          RCLCPP_ERROR(this->get_logger(), "Failed to initialize solver.");
          return;
        }
        solver_initialized = true;
      } else {
        if (!solver.updateHessianMatrix(H_sparse) ||
            !solver.updateGradient(f) ||
            !solver.updateLowerBound(l) ||
            !solver.updateUpperBound(u)) {
          RCLCPP_ERROR(this->get_logger(), "Failed to update solver.");
          return;
        }
      }

      // OSQP, solve
      if (solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError) {
        RCLCPP_ERROR(this->get_logger(), "OSQP solveProblem() failed.");
        return;
      }

      // OSQP, get solution and clamp output for safety
      Eigen::VectorXd delta_U = solver.getSolution();
      double v_cmd = v_local[0] + delta_U[0];
      double w_cmd = w_local[0] + delta_U[1];
      v_cmd = std::clamp(v_cmd, -0.22, 0.22);
      w_cmd = std::clamp(w_cmd, -2.84, 2.84);
      
      // Publish to /cmd_vel
      auto message = geometry_msgs::msg::TwistStamped();
      message.twist.linear.x = v_cmd;
      message.twist.angular.z = w_cmd;
      RCLCPP_INFO(this->get_logger(), "Publishing: linear.x=%.2f angular.z=%.2f",
                  message.twist.linear.x, message.twist.angular.z);
      pub_->publish(message);
    });
  }

private:
    // Safety flag
    bool state_received, path_received, start;
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

    // OSQP
    OsqpEigen::Solver solver;
    bool solver_initialized;
    Eigen::SparseMatrix<double> A_sparse;

    // Dynamic Reconfigure
    rcl_interfaces::msg::SetParametersResult
    on_param_set(const std::vector<rclcpp::Parameter> &params) {
      rcl_interfaces::msg::SetParametersResult res;
      res.successful = true;

      for (const auto &param : params) {
      const std::string &name = param.get_name();

      if (name == "start") {
        start = param.as_bool();
      } else {
        res.successful = false;
        res.reason = "Unknown parameter: " + name;
      }
    }
    return res;
    }

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
};

// Main function to run the node
int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MPCNode>());
  rclcpp::shutdown();
  return 0;
}