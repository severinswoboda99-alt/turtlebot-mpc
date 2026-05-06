import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import matplotlib.pyplot as plt
import numpy as np
from scipy.interpolate import interp1d
import math
from scipy.signal import savgol_filter

def quaternion_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

def angle_diff(a, b):
    d = a - b
    return abs(math.atan2(math.sin(d), math.cos(d)))  # Take absolute value

# Read data from rosbag2
def read_bag(bag_path, topic_name, msg_type_str):
    storage_options = rosbag2_py.StorageOptions(
        uri=bag_path,
        storage_id='mcap'
    )
    converter_options = rosbag2_py.ConverterOptions('', '')

    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    msg_type = get_message(msg_type_str)

    data = []
    while reader.has_next():
        topic, raw_data, t = reader.read_next()
        if topic == topic_name:
            msg = deserialize_message(raw_data, msg_type)
            data.append((t, msg))
    return data

# Extract planned path data, only last message
def extract_planned(path_msgs):
    if not path_msgs:
        return [], [], []

    _, msg = path_msgs[-1]

    xs, ys, yaws = [], [], []
    for pose in msg.poses:
        xs.append(pose.pose.position.x)
        ys.append(pose.pose.position.y)
        yaws.append(quaternion_to_yaw(pose.pose.orientation))

    return xs, ys, yaws

# Extract traveled path data, only last message
def extract_traveled(traveled_msgs, movement_threshold=0.01):
    if not traveled_msgs:
        return [], [], []

    _, msg = traveled_msgs[-1]

    xs, ys, yaws = [], [], []

    prev_x = None
    prev_y = None
    start_idx = 0

    for i, pose_stamped in enumerate(msg.poses):
        x = pose_stamped.pose.position.x
        y = pose_stamped.pose.position.y

        if prev_x is not None:
            dist = math.hypot(x - prev_x, y - prev_y)
            if dist > movement_threshold:
                start_idx = i
                break

        prev_x = x
        prev_y = y

    for pose_stamped in msg.poses[start_idx:]:
        xs.append(pose_stamped.pose.position.x)
        ys.append(pose_stamped.pose.position.y)
        yaws.append(quaternion_to_yaw(pose_stamped.pose.orientation))

    return np.array(xs), np.array(ys), np.array(yaws)

# Extract Float64 data (loop time, velocities)
def extract_float64(fl_msgs):
    times, fl = [], []
    for t, msg in fl_msgs:
        times.append(t)
        fl.append(msg.data)
    return np.array(times), np.array(fl)


# Extract inputs
def extract_cmd_vel(cmd_vel_msgs, threshold=1e-6):
    if not cmd_vel_msgs:
        return [], [], []

    times, v_actuals, w_actuals = [], [], []
    for t, msg in cmd_vel_msgs:
        v = msg.twist.linear.x
        w = msg.twist.angular.z
        if abs(v) > threshold or abs(w) > threshold:
            times.append(t)
            v_actuals.append(v)
            w_actuals.append(w)

    return np.array(times), np.array(v_actuals), np.array(w_actuals)

# Extract aruco pose data
def extract_aruco_poses(pose_msgs):
    if not pose_msgs:
        return [], [], [], []

    t_vals = []
    xa, ya, yawa = [], [], []

    for t, msg in pose_msgs:
        for marker in msg.markers:
            pose = marker.pose.pose

            xa.append(pose.position.x)
            ya.append(pose.position.y)
            yawa.append(quaternion_to_yaw(pose.orientation))
            t_vals.append(t * 1e-9)  # convert ns → seconds

    return np.array(t_vals), np.array(xa), np.array(ya), np.array(yawa)

# Compute deviation between target and traveled trajectory
def compute_pos_error(px, py, tx, ty):
    pos_errors = []
    planned = np.array(list(zip(px, py)))
    for x, y in zip(tx, ty):
        dists = np.linalg.norm(planned - np.array([x, y]), axis=1)
        pos_errors.append(np.min(dists))
    return pos_errors

def compute_heading_error(px, py, pyaw, tx, ty, tyaw):
    errors = []
    planned_xy = np.array(list(zip(px, py)))

    for x, y, yaw_t in zip(tx, ty, tyaw):
        dists = np.linalg.norm(planned_xy - np.array([x, y]), axis=1)
        idx = np.argmin(dists)

        yaw_p = pyaw[idx]
        err = angle_diff(yaw_t, yaw_p)
        errors.append(abs(err))  # Take absolute value here

    return np.array(errors)

def compute_control_effort(v, w):
    dv = np.diff(v)
    dw = np.diff(w)
    return dv, dw

# Aruco Marker path normalization and filters
def normalize_trajectory(x, y, yaw):
    # Initial pose
    x0 = x[0]
    y0 = y[0]
    yaw0 = yaw[0]

    # translate
    x_shift = x - x0
    y_shift = y - y0

    # rotate
    cos_yaw = np.cos(-yaw0)
    sin_yaw = np.sin(-yaw0)

    x_rot = cos_yaw * x_shift - sin_yaw * y_shift
    y_rot = sin_yaw * x_shift + cos_yaw * y_shift

    # normalize heading
    yaw_rot = yaw - yaw0

    return x_rot, y_rot, yaw_rot

def filter_outliers(x, y, yaw, max_jump=0.15):
    x_f, y_f, yaw_f = [x[0]], [y[0]], [yaw[0]]

    for i in range(1, len(x)):
        dist = np.hypot(x[i] - x_f[-1], y[i] - y_f[-1])

        if dist < max_jump:
            x_f.append(x[i])
            y_f.append(y[i])
            yaw_f.append(yaw[i])
        # else: skip outlier

    return np.array(x_f), np.array(y_f), np.array(yaw_f)

# Plotting Execution
plt.rcParams['font.size'] = '12'

if __name__ == "__main__":
    bag_path = "/home/swo/turtlebot-mpc/src/mpc-tracker/rosbag2/Horizon Size bag/bag_horizon_path3_N40_Q1_1_1_R1_1"  # rosbag2 folder
    # bag_path_camera = "/home/swo/turtlebot-mpc/src/mpc-tracker/rosbag2/real_pathX_bagX"
    planned = read_bag(bag_path, "/path", "nav_msgs/msg/Path")
    traveled = read_bag(bag_path, "/traveled_path", "nav_msgs/msg/Path")
    loop_time = read_bag(bag_path, "/loop_time", "std_msgs/msg/Float64")
    cmd_vel = read_bag(bag_path, "/cmd_vel", "geometry_msgs/msg/TwistStamped")
    v_ref = read_bag(bag_path, "/v_ref", "std_msgs/msg/Float64")
    w_ref = read_bag(bag_path, "/w_ref", "std_msgs/msg/Float64")
    # aruco__poses = read_bag(bag_path_camera, "/aruco/markers", "aruco_markers_msgs/msg/MarkerArray")
    
    px, py, pyaw = extract_planned(planned)
    tx, ty, tyaw = extract_traveled(traveled)
    t_l_times, t_l_values = extract_float64(loop_time)
    t_v_ref, vs_ref = extract_float64(v_ref)
    t_w_ref, ws_ref = extract_float64(w_ref)
    t_cmd, v_cmd, w_cmd = extract_cmd_vel(cmd_vel)
    # ta, xa, ya, yawa = extract_aruco_poses(aruco__poses)
    
    # Naming Parameters
    P = 3
    N = 40
    # run = 4
    # print(f"Run Number: {run}")
    print(f"Path Number: {P}")
    print(f"Horizon N: {N}")
    
    # Computing Time per Loop
    filtered = t_l_values[t_l_values < np.percentile(t_l_values, 99)]
    avg_time = np.mean(filtered)
    std_time = np.std(filtered)
    max_time = np.max(filtered)
    min_time = np.min(filtered)

    print(f"Average solve time: {avg_time:.3f} ms")
    print(f"Std dev: {std_time:.3f} ms")
    print(f"Min: {min_time:.3f} ms")
    print(f"Max: {max_time:.3f} ms")

    # # Position Error
    # pos_errors = compute_pos_error(px, py, tx, ty)
    
    # avg_pos_error = np.mean(pos_errors)
    # std_pos_error = np.std(pos_errors)
    # max_pos_error = np.max(pos_errors)
    
    # print(f"Average pos error: {avg_pos_error:.3f} m")
    # print(f"Std dev: {std_pos_error:.3f} m")
    # print(f"Max: {max_pos_error:.3f} m")
    
    # # Heading Error
    # heading_errors = compute_heading_error(px, py, pyaw, tx, ty, tyaw)
    # heading_errors_deg = np.degrees(heading_errors)
    
    # avg_heading_error = np.mean(heading_errors_deg)
    # std_heading_error = np.std(heading_errors_deg)
    # max_heading_error = np.max(heading_errors_deg)
    
    # print(f"Average heading error: {avg_heading_error:.3f} deg")
    # print(f"Std dev: {std_heading_error:.3f} deg")
    # print(f"Max: {max_heading_error:.3f} deg")
    
    # # Control Error
    # # Interpolate reference inputs to match cmd_vel timestamps
    # v_ref_interp = interp1d(t_v_ref, vs_ref, kind='nearest', fill_value='extrapolate')(t_cmd)
    # w_ref_interp = interp1d(t_w_ref, ws_ref, kind='nearest', fill_value='extrapolate')(t_cmd)

    # # Compute input errors
    # v_errors = np.abs(v_ref_interp - v_cmd)
    # print(f"Average linear input error: {np.mean(v_errors):.3f} m/s")
    # print(f"Std dev: {np.std(v_errors):.3f} m/s")
    # print(f"Max: {np.max(v_errors):.3f} m/s")
    
    # w_errors = np.abs(w_ref_interp - w_cmd)
    # print(f"Average angular input error: {np.mean(w_errors):.3f} rad/s")
    # print(f"Std dev: {np.std(w_errors):.3f} rad/s")
    # print(f"Max: {np.max(w_errors):.3f} rad/s")
    
    # # Control Effort
    # dv, dw = compute_control_effort(v_cmd, w_cmd)
    
    # dv_abs = np.abs(dv)
    # dw_abs = np.abs(dw)

    # print(f"Average Δv: {np.mean(dv_abs):.3f} m/s")
    # print(f"Std dev: {np.std(dv_abs):.3f} m/s")
    # print(f"Max: {np.max(dv_abs):.3f} m/s")

    # print(f"Average Δw: {np.mean(dw_abs):.3f} rad/s")
    # print(f"Std dev: {np.std(dw_abs):.3f} rad/s")
    # print(f"Max: {np.max(dw_abs):.3f} rad/s")
    
    # # Aruco Transform
    # xa_n, ya_n, yawa_n = normalize_trajectory(xa, ya, yawa)
    # xa_f, ya_f, yawa_f = filter_outliers(xa_n, ya_n, yawa_n)
    # xa_s = savgol_filter(xa_f, window_length=11, polyorder=2)
    # ya_s = savgol_filter(ya_f, window_length=11, polyorder=2)


    # # Plot planned path
    # # Spline Points
    # # x_spline = [0, 0.5, 1, 1.5, 2, 2.5, 3]
    # # y_spline = [0, 0.5, 0.25, -0.5, -0.25, 0.5, 0]
    
    # # plt.figure()
    # # plt.plot(px, py, linestyle='--', label='Planned Path', color='b')
    # # plt.scatter(x_spline, y_spline, label='Spline Points', color='r')
    # # plt.xlabel("X [m]")
    # # plt.ylabel("Y [m]")
    # # plt.legend()
    # # plt.axis('equal')
    # # plt.title("Path 3")
    # # plt.grid(linewidth = 0.5)
    # # plt.savefig("planned_3.pdf")
    # # plt.show()
    
    # # Plot trajectories
    # plt.figure()
    # plt.plot(px, py, linestyle='--', label='Planned Path', color='b')
    # plt.plot(tx, ty, label='Robot Trajectory', color='crimson')
    # plt.xlabel("X [m]")
    # plt.ylabel("Y [m]")
    # plt.legend()
    # plt.axis('equal')
    # plt.title(f"Path Comparison, Run {run}, Path {P}")
    # plt.grid(linewidth = 0.5)
    # plt.savefig(f"trajectory_comp_R{run}_P{P}.pdf")
    # plt.show()
    
    # # # Plot computing loop time
    # # plt.figure()
    # # plt.plot(filtered, color='turquoise')
    # # plt.axhline(y=avg_time, color='r', linestyle='-')
    # # plt.xlabel("Time index")
    # # plt.ylabel("Time [ms]")
    # # plt.title("Computing Time, N = 50, Path 3")
    # # plt.grid(linewidth = 0.5)
    # # plt.savefig("loop_time_plot_N50_P3.pdf")
    # # plt.show()
    
    # # # Histogram loop time
    # # plt.figure()
    # # plt.hist(filtered, bins=30, color='lightseagreen')
    # # plt.xlabel("Time [ms]")
    # # plt.ylabel("Frequency")
    # # plt.title("Computing Time Distribution, N = 50")
    # # plt.grid()
    # # plt.savefig("loop_time_hist_N50.pdf")
    # # plt.show()
    
    # # Plot position error
    # plt.figure()
    # plt.plot(pos_errors, color='blueviolet')
    # plt.axhline(y=avg_pos_error, color='r', linestyle='-')
    # plt.axhline(y=0, color='k', linestyle='--')
    # plt.xlabel("Time index")
    # plt.ylabel("Deviation [m]")
    # plt.title(f"Position Error, Run {run}, Path {P}")
    # plt.grid(linewidth = 0.5)
    # plt.savefig(f"pos_error_plot_R{run}_P{P}.pdf")
    # plt.show()
    
    # # # Histogram position error
    # # plt.figure()
    # # plt.hist(pos_errors, bins=30, color='indigo')
    # # plt.xlabel("Deviation [m]")
    # # plt.ylabel("Frequency")
    # # plt.title("Position Error Distribution")
    # # plt.grid()
    # # plt.savefig("pos_error_hist.pdf")
    # # plt.show()
    
    # # Plot heading error
    # plt.figure()
    # plt.plot(heading_errors_deg, color='slateblue')
    # plt.axhline(y=avg_heading_error, color='r', linestyle='-')
    # plt.axhline(y=0, color='k', linestyle='--')
    # plt.xlabel("Time index")
    # plt.ylabel("Deviation [deg]")
    # plt.title(f"Heading Error, Run {run}, Path {P}")
    # plt.grid()
    # plt.savefig(f"heading_error_plot_R{run}_P{P}.pdf")
    # plt.show()
    
    # # # Histogram heading error
    # # plt.figure()
    # # plt.hist(heading_errors_deg, bins=30, color='darkslateblue')
    # # plt.xlabel("Deviation [deg]")
    # # plt.ylabel("Frequency")
    # # plt.title("Heading Error Distribution")
    # # plt.grid()
    # # plt.savefig("heading_error_hist.pdf")
    # # plt.show()
    
    # # # Plot control effort
    # # plt.figure()
    # # plt.plot(t_cmd[1:], dv_abs, color='darkslateblue')
    # # plt.axhline(y=np.mean(dv_abs), color='r', linestyle='-')
    # # plt.axhline(y=0, color='k', linestyle='--')
    # # plt.xlabel("Time [s]")
    # # plt.ylabel("Control Effort [m/s]")
    # # plt.title(f"Linear Control Effort, Run {run}, Path {P}")
    # # plt.grid()
    # # plt.savefig(f"linear_control_effort_R{run}_P{P}.pdf")
    # # plt.show()
    
    # plt.figure()
    # plt.hist(dv_abs, bins=30, color='darkslateblue')
    # plt.xlabel("Control Effort [m/s]")
    # plt.ylabel("Frequency")
    # plt.title(f"Linear Control Effort, Run {run}, Path {P}")
    # plt.grid()
    # plt.savefig(f"linear_control_effort_R{run}_P{P}.pdf")
    # plt.show()
    
    # # plt.figure()
    # # plt.plot(t_cmd[1:], dw_abs, color='mediumslateblue')
    # # plt.axhline(y=np.mean(dw_abs), color='r', linestyle='-')
    # # plt.axhline(y=0, color='k', linestyle='--')
    # # plt.xlabel("Time [s]")
    # # plt.ylabel("Control Effort [rad/s]")
    # # plt.title(f"Angular Control Effort, Run {run}, Path {P}")
    # # plt.grid()
    # # plt.savefig(f"angular_control_effort_R{run}_P{P}.pdf")
    # # plt.show()
    
    # plt.figure()
    # plt.hist(dw_abs, bins=30, color='mediumslateblue')
    # plt.xlabel("Control Effort [rad/s]")
    # plt.ylabel("Frequency")
    # plt.title(f"Angular Control Effort, Run {run}, Path {P}")
    # plt.grid()
    # plt.savefig(f"angular_control_effort_R{run}_P{P}.pdf")
    # plt.show()
    
    # # # Plot input errors
    # # plt.figure()
    # # plt.plot(t_cmd, v_errors, color='midnightblue')
    # # plt.axhline(y=np.mean(v_errors), color='r', linestyle='-')
    # # plt.axhline(y=0, color='k', linestyle='--')
    # # plt.xlabel("Time [s]")
    # # plt.ylabel("Velocity Error [m/s]")
    # # plt.title(f"Linear Velocity Input Error, Run {run}, Path {P}")
    # # plt.grid()
    # # plt.savefig(f"linear_input_error_R{run}_P{P}.pdf")
    # # plt.show()
    
    # plt.figure()
    # plt.hist(v_errors, bins=30, color='midnightblue')
    # plt.xlabel("Velocity Error [m/s]")
    # plt.ylabel("Frequency")
    # plt.title(f"Linear Velocity Input Error, Run {run}, Path {P}")
    # plt.grid()
    # plt.savefig(f"linear_input_error_R{run}_P{P}.pdf")
    # plt.show()
    
    # # plt.figure()
    # # plt.plot(t_cmd, w_errors, color='darkblue')
    # # plt.axhline(y=np.mean(w_errors), color='r', linestyle='-')
    # # plt.axhline(y=0, color='k', linestyle='--')
    # # plt.xlabel("Time [s]")
    # # plt.ylabel("Velocity Error [rad/s]")
    # # plt.title(f"Angular Velocity Input Error, Run {run}, Path {P}")
    # # plt.grid()
    # # plt.savefig(f"angular_input_error_R{run}_P{P}.pdf")
    # # plt.show()
    
    # plt.figure()
    # plt.hist(w_errors, bins=30, color='darkblue')
    # plt.xlabel("Velocity Error [rad/s]")
    # plt.ylabel("Frequency")
    # plt.title(f"Angular Velocity Input Error, Run {run}, Path {P}")
    # plt.grid()
    # plt.savefig(f"angular_input_error_R{run}_P{P}.pdf")
    # plt.show()
    
    # # Aruco trajectory comparison
    # plt.figure()
    # plt.plot(px, py, linestyle='--', label='Planned Path', color='b')
    # plt.plot(tx, ty, label='Robot Trajectory', color='crimson')
    # plt.plot(-ya_s, -xa_s, label='ArUco Trajectory', color='darkorange')
    # plt.xlabel("X [m]")
    # plt.ylabel("Y [m]")
    # plt.legend()
    # plt.axis('equal')
    # plt.title(f"test")
    # plt.grid(linewidth = 0.5)
    # plt.savefig(f"test.pdf")
    # plt.show()