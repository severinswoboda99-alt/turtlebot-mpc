#include <chrono>
#include <algorithm>
#include <memory>
#include <string>
#include <cmath>
#include <limits>
#include <vector>
#include <iostream>
#include <Eigen/Dense>

#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
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

// Function to find the index of the currently closest refernce path point
int closest_index(double x, double y, const std::vector<double>& x_ref, const std::vector<double>& y_ref) {
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
    this->declare_parameter<double>("delta_t", 0.1);  // time step size, in s
    this->declare_parameter<int>("N", 10);            // Horizon size
    this->declare_parameter<double>("q_1", 1.0);
    this->declare_parameter<double>("q_2", 1.0);
    this->declare_parameter<double>("q_3", 1.0);
    this->declare_parameter<double>("p_1", 1.0);
    this->declare_parameter<double>("p_2", 1.0);
    this->declare_parameter<double>("p_3", 1.0);
    this->declare_parameter<double>("r_1", 1.0);
    this->declare_parameter<double>("r_2", 1.0);

    start = this->get_parameter("start").as_bool();
    delta_t = this->get_parameter("delta_t").as_double();
    N = this->get_parameter("N").as_int();
    q_1 = this->get_parameter("q_1").as_double();
    q_2 = this->get_parameter("q_2").as_double();
    q_3 = this->get_parameter("q_3").as_double();
    p_1 = this->get_parameter("p_1").as_double();
    p_2 = this->get_parameter("p_2").as_double();
    p_3 = this->get_parameter("p_3").as_double();
    r_1 = this->get_parameter("r_1").as_double();
    r_2 = this->get_parameter("r_2").as_double();

    param_cb_handle_ = this->add_on_set_parameters_callback(
      std::bind(&MPCNode::on_param_set, this, std::placeholders::_1));

    // Initialize member variables
    state_received = false;
    solver_initialized = false;
    delta_X_0 = Eigen::Vector3d::Zero();
    v_last = 0.0;
    w_last = 0.0;
    acc_cycles = 0;

    // Tuning Parameters
    rebuild_cost_matrices();

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

    // Create Publisher for the TurtleBot3 movement topic
    cmd_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", 10);
    // Create Publisher for the local horizon
    horizon_pub_ = this->create_publisher<nav_msgs::msg::Path>("/horizon", 10);
    // Create Publisher for solve time
    loop_time_pub_ = this->create_publisher<std_msgs::msg::Float64>("/loop_time", 10);
    // Create Publisher for inputs
    v_ref_pub_ = this->create_publisher<std_msgs::msg::Float64>("/v_ref", 10);
    w_ref_pub_ = this->create_publisher<std_msgs::msg::Float64>("/w_ref", 10);

    // Subscriber to reference trajectory
    auto ref_callback = [this](const nav_msgs::msg::Path & path){
      // Clear the vectors, should the path change
      this->x_ref.clear();
      this->y_ref.clear();
      this->theta_ref.clear();
      this->v_ref.clear();
      this->w_ref.clear();

      // Size pose vectors
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
        v = std::clamp(v, -0.22, 0.22);
        w = std::clamp(w, -2.84, 2.84);

        v_ref.push_back(v);
        w_ref.push_back(w);
      }

      // Keep input vectors aligned with state vectors for safe horizon indexing
      if (size >= 2) {
        v_ref.push_back(v_ref.back());
        w_ref.push_back(w_ref.back());
      } else if (size == 1) {
        v_ref.push_back(0.0);
        w_ref.push_back(0.0);
      }
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

    // Create main timer and corresponding callback function
    auto timer_period = std::chrono::milliseconds(static_cast<int>(delta_t * 1000)); // Conversion to ms
    timer_ = this->create_wall_timer(timer_period, [this]() {
      // Track time of complete loop
      auto t0 = std::chrono::steady_clock::now();

      // Only drive the robot when the state and reference path have been received, and the start signal has been given
      if (!state_received || !start) {
        auto message = geometry_msgs::msg::TwistStamped();
        message.twist.linear.x = 0.0;
        message.twist.angular.z = 0.0;
        RCLCPP_INFO(this->get_logger(), "Stopped: Waiting for State or Start Data");
        cmd_pub_->publish(message);
        return;
      }

      // Check for reference data before continuing to main loop
      if (x_ref.empty() || y_ref.empty() || theta_ref.empty() || v_ref.empty() || w_ref.empty()) {
        auto message = geometry_msgs::msg::TwistStamped();
        message.twist.linear.x = 0.0;
        message.twist.angular.z = 0.0;
        RCLCPP_WARN(this->get_logger(), "Stopped: Incomplete Reference Data");
        cmd_pub_->publish(message);
        return;
      }

      // Main MPC loop
      // Controller has reference data, current state and the start signal
      // Create Local Horizon, up to N steps, starting from the nearest reference point idx
      int idx = closest_index(x, y, x_ref, y_ref);
  
      // Check if robot is near the end of the path (5 cm), and stop, if necessary
      double dx = x - x_ref.back();
      double dy = y - y_ref.back();
      double dist_to_end = std::sqrt(dx * dx + dy * dy);

      if (dist_to_end < 0.05) {
        auto message = geometry_msgs::msg::TwistStamped();
        message.twist.linear.x = 0.0;
        message.twist.angular.z = 0.0;
        RCLCPP_INFO(this->get_logger(), "Stopped: End of Path");
        cmd_pub_->publish(message);
        return;
      }
      
      // Populate local horizon vectors with data from the path planner
      // N input and states are needed for the prediction horizon
      // The path planner provides a series of approximately equally spaced poses, not time-stamped!
      // The controller has already constructed a reference trajectory, adding the inputs for each step, still not time-stamped!
      // The controller model works with time steps, so for each time step the appropriate reference trajectory step is approximated
      // by projecting the robots movement forwards, resulting in the local horizon vectors:
      std::vector<double> x_local, y_local, theta_local, v_local, w_local;

      double total_dist = 0.0;

      for (int k = 0; k < N; ++k) {
        const int idx_state = std::clamp(idx, 0, static_cast<int>(x_ref.size()) - 1);
        const int idx_input = std::clamp(idx, 0, static_cast<int>(v_ref.size()) - 1);

        x_local.push_back(x_ref[idx_state]);
        y_local.push_back(y_ref[idx_state]);
        theta_local.push_back(theta_ref[idx_state]);
        v_local.push_back(v_ref[idx_input]);
        w_local.push_back(w_ref[idx_input]);

          // Target distance for next step
        double target_dist = std::max(0.0, v_ref[idx_input] * delta_t);

        total_dist = 0.0;

          // Move forward along path until distance ~ v*dt
        while (idx + 1 < (int)x_ref.size()) {
          double dx = x_ref[idx+1] - x_ref[idx];
          double dy = y_ref[idx+1] - y_ref[idx];
          double ds = std::sqrt(dx*dx + dy*dy);

          total_dist += ds;
          idx++;

          if (total_dist >= target_dist) {
            break;
          }
        }
      }

      // Publish the horizon dynamically for visualization
      auto horizon = nav_msgs::msg::Path();
      auto now = this->now();
      horizon.header.stamp = now;
      horizon.header.frame_id = "odom";
      for (int i = 0; i < N; ++i) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = "odom";
        pose.header.stamp = now;
        pose.pose.position.x = x_local[i];
        pose.pose.position.y = y_local[i];
        horizon.poses.push_back(pose);
      }
      horizon_pub_->publish(horizon);

      // Publish the next input for evaluation
      std_msgs::msg::Float64 v_ref_msg;
      v_ref_msg.data = v_local[0];
      v_ref_pub_->publish(v_ref_msg);

      std_msgs::msg::Float64 w_ref_msg;
      w_ref_msg.data = w_local[0];
      w_ref_pub_->publish(w_ref_msg);

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

      // Solve complete, get time and publish
      auto t1 = std::chrono::steady_clock::now();
      double loop_time = std::chrono::duration<double, std::milli>(t1 - t0).count();
      std_msgs::msg::Float64 msg;
      msg.data = loop_time;
      loop_time_pub_->publish(msg);

      // OSQP, get solution and clamp output for safety
      Eigen::VectorXd delta_U = solver.getSolution();
      double v_cmd = v_local[0] + delta_U[0];
      double w_cmd = w_local[0] + delta_U[1];
      v_cmd = std::clamp(v_cmd, -0.22, 0.22);
      w_cmd = std::clamp(w_cmd, -2.84, 2.84);

      // Additional real-world clamp for initial acceleration
      if(acc_cycles < 3) {
        const double max_delta_v = 0.02;  // m/s per loop
        v_cmd = std::clamp(v_cmd, v_last - max_delta_v, v_last + max_delta_v);
        v_last = v_cmd;

        const double max_delta_w = 0.4;   // rad/s per loop
        w_cmd = std::clamp(w_cmd, w_last - max_delta_w, w_last + max_delta_w);
        w_last = w_cmd;

        ++acc_cycles;
      }
      
      // Publish to /cmd_vel
      auto message = geometry_msgs::msg::TwistStamped();
      message.twist.linear.x = v_cmd;
      message.twist.angular.z = w_cmd;
      RCLCPP_INFO(this->get_logger(), "Publishing: linear.x=%.2f angular.z=%.2f",
                  message.twist.linear.x, message.twist.angular.z);
      cmd_pub_->publish(message);
    });
  }

private:
  // (Re-)Build cost matrices during initialisation or after parameter change
  void rebuild_cost_matrices() {
    Q << q_1, 0, 0,
      0, q_2, 0,
      0, 0, q_3;

    P << p_1, 0, 0,
      0, p_2, 0,
      0, 0, p_3;

    R << r_1, 0,
      0, r_2;

    Q_bar = calc_Q_bar(Q, P, N);
    R_bar = calc_R_bar(R, N);
    RCLCPP_INFO(this->get_logger(), "Rebuild successful, Data Updated!");
  }

    // Safety flag
    bool state_received, start;
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
    Eigen::Matrix3d Q, P; // State and Terminal Cost
    Eigen::Matrix2d R; // Input Cost
    double q_1, q_2, q_3, p_1, p_2, p_3, r_1, r_2; // Matrix entries

    // OSQP
    OsqpEigen::Solver solver;
    bool solver_initialized;
    Eigen::SparseMatrix<double> A_sparse;

    // Hardware Safety
    double v_last, w_last;
    int acc_cycles;

    // Dynamic Reconfigure
    rcl_interfaces::msg::SetParametersResult
    on_param_set(const std::vector<rclcpp::Parameter> &params) {
      rcl_interfaces::msg::SetParametersResult res;
      res.successful = true;
      bool weights_changed = false;
      bool horizon_changed = false;

      for (const auto &param : params) {
      const std::string &name = param.get_name();

      if (name == "start") {
        start = param.as_bool();
      } else if (name == "delta_t") {
        delta_t = param.as_double();
      } else if (name == "N") {
        N = param.as_int();
        horizon_changed = true;
      } else if (name == "q_1") {
        q_1 = param.as_double();
        weights_changed = true;
      } else if (name == "q_2") {
        q_2 = param.as_double();
        weights_changed = true;
      } else if (name == "q_3") {
        q_3 = param.as_double();
        weights_changed = true;
      } else if (name == "p_1") {
        p_1 = param.as_double();
        weights_changed = true;
      } else if (name == "p_2") {
        p_2 = param.as_double();
        weights_changed = true;
      } else if (name == "p_3") {
        p_3 = param.as_double();
        weights_changed = true;
      } else if (name == "r_1") {
        r_1 = param.as_double();
        weights_changed = true;
      } else if (name == "r_2") {
        r_2 = param.as_double();
        weights_changed = true;
      } else {
        res.successful = false;
        res.reason = "Unknown parameter: " + name;
      }
    }

    // The matrices and/or solver might need to be rebuilt after parameter change!
    if (weights_changed || horizon_changed) {
      rebuild_cost_matrices();
    }

    if (horizon_changed) {
      A_sparse = Eigen::MatrixXd::Identity(2*N, 2*N).sparseView();

      // Reset solver dimensions for new horizon.
      solver.clearSolver();
      solver.data()->setNumberOfVariables(2*N);
      solver.data()->setNumberOfConstraints(2*N);
      solver_initialized = false;
    }

    return res;
    }

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr horizon_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr loop_time_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr v_ref_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr w_ref_pub_;
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